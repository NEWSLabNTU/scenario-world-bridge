"""AutowarePort's RTC on each Autoware's API, probed and never configured: TIER IV's
tier4_ad_api_adaptor (Autoware 1.5.0: /api/external/*/rtc_*), the planning modules' own
rtc_interface (Autoware 2.1.0 has no adaptor), or neither. Fake Autoware nodes in a domain of
their own, on localhost."""

import os

os.environ["ROS_DOMAIN_ID"] = "78"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ.pop("CYCLONEDDS_URI", None)   # see test_ros_adapter.py

import threading  # noqa: E402
import time  # noqa: E402

import pytest  # noqa: E402

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("autoware_adapi_v1_msgs")
rtc_msgs = pytest.importorskip("tier4_rtc_msgs")

from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from tier4_rtc_msgs.msg import CooperateResponse, CooperateStatus, CooperateStatusArray  # noqa: E402,E501
from tier4_rtc_msgs.srv import AutoMode, AutoModeWithModule, CooperateCommands  # noqa: E402

from scensim_agent.adapters import autoware_ros  # noqa: E402
from scensim_agent.adapters.autoware_ros import AutowarePort  # noqa: E402

UUID_TL = "0a" * 16
UUID_VTL = "0b" * 16
UUID_INT = "0c" * 16


def _status(module, uuid_hex):
    s = CooperateStatus()
    s.module.type = autoware_ros.RTC_MODULES[module]
    s.uuid.uuid = list(bytes.fromhex(uuid_hex))
    s.safe, s.requested, s.auto_mode = True, True, False
    s.start_distance, s.finish_distance = 10.0, 20.0
    return s


class FakeAutoware:
    """``adaptor``: tier4_ad_api_adaptor's aggregated RTC; ``modules``: the rtc_interface
    modules ``intersection``, ``traffic_light`` and ``virtual_traffic_light``, each with one
    status of its own."""

    def __init__(self, adaptor=False, modules=False):
        self.node = rclpy.create_node("fake_autoware")
        self.calls = []        # (service, module type or name, payload)
        n = self.node
        self.timers = []
        if adaptor:
            n.create_service(CooperateCommands, "/api/external/set/rtc_commands",
                             self._commands("adaptor"))
            n.create_service(AutoModeWithModule, "/api/external/set/rtc_auto_mode",
                             self._auto_with_module)
            pub = n.create_publisher(CooperateStatusArray, "/api/external/get/rtc_status", 1)
            msg = CooperateStatusArray(statuses=[_status("INTERSECTION", UUID_INT)])
            self.timers.append(n.create_timer(0.1, lambda p=pub, m=msg: p.publish(m)))
        if modules:
            for name, module, uuid in (("intersection", "INTERSECTION", UUID_INT),
                                       ("traffic_light", "TRAFFIC_LIGHT", UUID_TL),
                                       ("virtual_traffic_light", "TRAFFIC_LIGHT", UUID_VTL)):
                n.create_service(CooperateCommands, f"/planning/cooperate_commands/{name}",
                                 self._commands(name, uuid))
                n.create_service(AutoMode, f"/planning/enable_auto_mode/{name}",
                                 self._auto(name))
                pub = n.create_publisher(CooperateStatusArray,
                                         f"/planning/cooperate_status/{name}", 1)
                msg = CooperateStatusArray(statuses=[_status(module, uuid)])
                self.timers.append(n.create_timer(0.1, lambda p=pub, m=msg: p.publish(m)))
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(n)
        self.spinner = threading.Thread(target=self.executor.spin, daemon=True)
        self.spinner.start()

    def _commands(self, name, uuid=None):
        def handle(req, res):
            for c in req.commands:
                got = bytes(c.uuid.uuid).hex()
                self.calls.append(("commands", name, got))
                # a module accepts only the uuids it holds (rtc_interface's validation)
                res.responses.append(CooperateResponse(
                    success=uuid is None or got == uuid, module=c.module, uuid=c.uuid))
            return res
        return handle

    def _auto(self, name):
        def handle(req, res):
            self.calls.append(("auto_mode", name, req.enable))
            res.success = True
            return res
        return handle

    def _auto_with_module(self, req, res):
        self.calls.append(("auto_mode", "adaptor", req.module.type, req.enable))
        res.success = True
        return res

    def close(self):
        for t in self.timers:
            t.cancel()
        self.executor.shutdown(timeout_sec=2.0)
        self.spinner.join(timeout=2.0)
        self.node.destroy_node()


class Logger:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)


@pytest.fixture
def make():
    if not rclpy.ok():
        rclpy.init()
    made = []

    def build(**kw):
        fake = FakeAutoware(**kw)
        node = rclpy.create_node("port_under_test")
        port = AutowarePort(node)
        log = Logger()
        node.get_logger = lambda: log
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        spinner = threading.Thread(target=ex.spin, daemon=True)
        spinner.start()
        made.append((fake, node, ex, spinner))
        return fake, port, log

    yield build
    # executors and their threads before the nodes, then the context: a spin thread alive at
    # interpreter exit aborts the process ("terminate called without an active exception")
    for fake, node, ex, spinner in made:
        ex.shutdown(timeout_sec=2.0)
        spinner.join(timeout=2.0)
        node.destroy_node()
        fake.close()
    rclpy.try_shutdown()


def settle(port, until, timeout=10.0):
    """Snapshots (each re-probing) until ``until(snapshot)``; the last snapshot."""
    deadline = time.monotonic() + timeout
    while True:
        port._ready_checked = 0.0
        s = port.snapshot()
        if until(s) or time.monotonic() > deadline:
            return s
        time.sleep(0.1)


def test_the_adaptor_is_used_when_it_runs_even_beside_the_modules(make):
    fake, port, log = make(adaptor=True, modules=True)
    s = settle(port, lambda s: s.rtc and s.rtc_status)
    assert s.rtc and [st["uuid"] for st in s.rtc_status] == [UUID_INT]
    assert port.cooperate("INTERSECTION", "ACTIVATE", UUID_INT) == (True, "")
    assert port.cooperate_auto("TRAFFIC_LIGHT", True) == (True, "")
    assert ("commands", "adaptor", UUID_INT) in fake.calls
    assert ("auto_mode", "adaptor", autoware_ros.RTC_MODULES["TRAFFIC_LIGHT"], True) \
        in fake.calls
    assert not [c for c in fake.calls if c[1] != "adaptor"]
    assert port._rtc_via == "adaptor"


def test_without_the_adaptor_the_modules_rtc_interface_is_used(make):
    fake, port, log = make(modules=True)
    s = settle(port, lambda s: s.rtc and s.rtc_status and len(s.rtc_status) == 3)
    assert s.rtc
    assert sorted(st["uuid"] for st in s.rtc_status) == sorted([UUID_INT, UUID_TL, UUID_VTL])
    assert {st["module"] for st in s.rtc_status} == {"INTERSECTION", "TRAFFIC_LIGHT"}
    # a uuid goes to the module holding it: the virtual traffic light, not the traffic light
    assert port.cooperate("TRAFFIC_LIGHT", "ACTIVATE", UUID_VTL) == (True, "")
    assert [c for c in fake.calls if c[0] == "commands"] == \
        [("commands", "virtual_traffic_light", UUID_VTL)]
    assert port.cooperate("INTERSECTION", "DEACTIVATE", UUID_INT) == (True, "")
    # auto mode goes to every module of the kind
    assert port.cooperate_auto("TRAFFIC_LIGHT", False) == (True, "")
    assert {c[1] for c in fake.calls if c[0] == "auto_mode"} == \
        {"traffic_light", "virtual_traffic_light"}
    ok, why = port.cooperate("CROSSWALK", "ACTIVATE", None)
    assert not ok and "no running planning module answers CROSSWALK" in why
    assert port._rtc_via == "modules"


def test_neither_means_no_rtc(make):
    _fake, port, log = make()
    s = settle(port, lambda s: False, timeout=1.5)
    assert not s.rtc and s.rtc_status is None
    ok, why = port.cooperate("INTERSECTION", "ACTIVATE", UUID_INT)
    assert not ok


class FakeClient:
    def __init__(self, ready):
        self.ready = ready
        self.srv_name = "fake"

    def service_is_ready(self):
        return self.ready[0]


class FakeNode:
    """Enough of a node for AutowarePort's probe: every client answers once ``core[0]``,
    the adaptor's once ``adaptor[0]``; ``emergency[0]`` publishers on the emergency topic."""

    def __init__(self):
        self.core, self.adaptor, self.emergency = [False], [False], [0]
        self.log = Logger()

    def create_subscription(self, *a, **kw):
        return None

    def create_client(self, _type, name, **kw):
        return FakeClient(self.adaptor if "/api/external/" in name else self.core)

    def count_publishers(self, topic):
        return self.emergency[0] if topic == "/api/external/get/emergency" else 0

    def get_service_names_and_types(self):
        return []

    def get_logger(self):
        return self.log


def probe(port):
    port._ready_checked = 0.0
    port.snapshot()


def test_the_api_is_logged_once_the_core_answers_then_on_each_change():
    node = FakeNode()
    port = AutowarePort(node)
    node.adaptor[0] = True          # the adaptor answers before the core: nothing logged yet
    probe(port)
    assert node.log.lines == []
    node.core[0], node.emergency[0] = True, 1
    probe(port)
    probe(port)                     # nothing changed: no second line
    assert node.log.lines == [
        "Autoware API: RTC through tier4_ad_api_adaptor (/api/external/*/rtc_*); "
        "emergency from /api/external/get/emergency"]
    node.adaptor[0], node.emergency[0] = False, 0    # the adaptor goes away
    probe(port)
    assert node.log.lines[1:] == [
        "Autoware API: no RTC; faults from /api/fail_safe/mrm_state only"]
