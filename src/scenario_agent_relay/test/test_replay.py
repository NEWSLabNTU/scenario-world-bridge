"""roadmap 077, G32: an agent that registers again within its episode (it went silent, or
restarted) is handed its goal and speed limit back; when that fails the entity is `lost`
and a commander's query says why (the runner ends the run with it)."""
import threading
import time

import pytest

from swb_agent import protocol as P
from scenario_agent_relay.server import AgentServer

from test_server import FakeAgent, wait_until

GOAL = {"x": 30.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


@pytest.fixture
def server():
    s = AgentServer("127.0.0.1", 0, peer_timeout=0.6, heartbeat_period=0.2)
    s.replay_wait = 1.0
    s.start()
    yield s
    s.stop()


def state(agent, phase):
    agent.send({"type": P.STATE, "phase": phase,
                "pose": {"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}})


def answer(agent, status=P.OK, message=""):
    """The next command the agent gets, answered."""
    m = agent.recv()
    assert m["type"] == P.COMMAND
    agent.send(P.reply(m["id"], status, message))
    return m


def commanded(server, name, **args):
    out = {}
    t = threading.Thread(target=lambda: out.update(r=server.command("ego", name, 3.0, **args)))
    t.start()
    return t, out


def drive(server):
    """An agent registered, given a speed limit and a goal, driving; then killed."""
    a = FakeAgent(server.port)
    assert wait_until(lambda: server.session("ego") is not None)
    state(a, P.IDLE)
    for name, args in ((P.SET_SPEED_LIMIT, {"mps": 8.0}),
                       (P.SET_GOAL, {"goal": GOAL, "waypoints": []})):
        t, out = commanded(server, name, **args)
        answer(a)
        t.join()
        assert out["r"][0] == P.OK
    state(a, P.DRIVING)
    assert wait_until(lambda: server._phases.get("ego") == P.DRIVING)
    a.close()
    assert wait_until(lambda: server.session("ego") is None)


def test_a_killed_agent_that_registers_again_gets_its_speed_limit_and_goal_back(server):
    drive(server)
    b = FakeAgent(server.port)
    state(b, P.IDLE)
    m1 = answer(b)
    m2 = answer(b)
    assert (m1["command"], m1["args"]) == (P.SET_SPEED_LIMIT, {"mps": 8.0})
    assert m2["command"] == P.SET_GOAL and m2["args"]["goal"] == GOAL
    assert wait_until(lambda: server.lost("ego") is None, 0.5)
    b.close()


def test_a_goal_refused_on_the_way_back_is_lost_and_the_query_says_why(server):
    drive(server)
    b = FakeAgent(server.port)
    state(b, P.IDLE)
    answer(b)                                   # the speed limit
    answer(b, P.FAILED, "no route from here")   # the goal
    assert wait_until(lambda: server.lost("ego") is not None)
    assert "refused its goal handed back: FAILED no route from here" in server.lost("ego")
    c = FakeAgent(server.port, entity="", register=False)
    c.send(P.register_commander("runner"))
    assert c.recv()["type"] == P.REGISTERED
    c.send(P.query(7, "ego"))
    r = c.recv()
    while r.get("id") != 7:
        r = c.recv()
    assert "no route from here" in r["lost"]
    assert abs(r["lost_at"] - time.time()) < 10
    b.close()
    c.close()


def test_an_agent_that_reports_nothing_after_it_registers_again_is_lost(server):
    server.replay_wait = 0.3   # inside the 0.6 s peer timeout of this silent fake agent
    drive(server)
    b = FakeAgent(server.port)
    assert wait_until(lambda: server.lost("ego") is not None, 3.0)
    assert "reported no state" in server.lost("ego")
    b.close()


def test_a_stopped_car_or_a_new_episode_is_not_given_the_old_goal(server):
    drive(server)
    b = FakeAgent(server.port)
    state(b, P.IDLE)
    answer(b)
    answer(b)
    t, out = commanded(server, P.STOP)
    answer(b)
    t.join()
    state(b, P.STOPPED)
    assert wait_until(lambda: server._phases.get("ego") == P.STOPPED)
    b.close()
    assert wait_until(lambda: server.session("ego") is None)
    c = FakeAgent(server.port)
    state(c, P.IDLE)
    c.sock.settimeout(0.8)
    with pytest.raises(OSError):   # nothing is handed back: a timeout, no command
        c.recv()
    assert server.lost("ego") is None
    c.close()


def test_a_new_agent_after_a_goal_that_ended_is_not_lost(server):
    """Item 8 (2026-11-16): i1's pilot ended DRIVING with its goal gone at the scenario's
    end; i3's fresh pilot registered and the run was ended as a lost goal."""
    drive(server)
    with server._lock:
        server._standing["ego"]["goal"] = None    # what CLEAR_GOAL, STOP or a new episode do
    b = FakeAgent(server.port)
    state(b, P.IDLE)
    b.sock.settimeout(0.8)
    with pytest.raises(OSError):   # nothing handed back
        b.recv()
    assert server.lost("ego") is None
    b.close()


def test_an_episode_end_after_its_agent_left_drops_the_goal(server):
    """swb 118, item 36 (G57): a `lifetime: run` pilot is stopped with its run, and the next
    run's agent was handed the old goal before its teleport."""
    drive(server)
    status, message = server.command("ego", P.EPISODE_END, 1.0, episode="r1",
                                     result={"passed": True})
    assert status == P.OK and "standing commands are dropped" in message
    b = FakeAgent(server.port)
    state(b, P.IDLE)
    b.sock.settimeout(0.8)
    with pytest.raises(OSError):   # nothing handed back
        b.recv()
    assert server.lost("ego") is None
    b.close()
