"""vehicle_agent's spec_hash: from `spec_hash_file` when given, else the vehicle model's
description package (roadmap 051)."""

import pytest

vehicle_agent = pytest.importorskip("swb_agent.adapters.vehicle_agent")


class Log:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def warning(self, msg):
        self.lines.append(("warning", msg))

    def error(self, msg):
        self.lines.append(("error", msg))


def test_a_spec_hash_file_is_used_instead_of_the_package(tmp_path):
    f = tmp_path / "spec_hash.yaml"
    f.write_text("spec_hash: abcdef0123456789\n")
    log = Log()
    # The vehicle model names no installed package: the file alone decides.
    assert vehicle_agent.spec_hash_of("no_such_car", log, str(f)) == "abcdef0123456789"
    assert log.lines[0][0] == "info"


def test_a_missing_or_unreadable_spec_hash_file_is_an_error_and_no_hash(tmp_path):
    log = Log()
    assert vehicle_agent.spec_hash_of("none", log, str(tmp_path / "absent.yaml")) == ""
    assert log.lines[-1][0] == "error" and "absent.yaml" in log.lines[-1][1]
    bad = tmp_path / "bad.yaml"
    bad.write_text("spec_hash: [unclosed\n")
    assert vehicle_agent.spec_hash_of("none", log, str(bad)) == ""
    assert log.lines[-1][0] == "error"
    listed = tmp_path / "list.yaml"
    listed.write_text("- 1\n")
    assert vehicle_agent.spec_hash_of("none", log, str(listed)) == ""
    assert log.lines[-1][0] == "error"


def test_no_file_and_no_model_is_no_hash():
    log = Log()
    for empty in ("", "none"):
        assert vehicle_agent.spec_hash_of("none", log, empty) == ""
        assert vehicle_agent.spec_hash_of("", log, empty) == ""
    assert log.lines == []
