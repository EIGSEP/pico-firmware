"""Tests for picohost.resolve."""

import pytest

import picohost.resolve as resolve_mod
from picohost.resolve import main, resolve_port


@pytest.fixture
def _mock_ports(monkeypatch):
    monkeypatch.setattr(
        resolve_mod,
        "find_pico_ports",
        lambda: {
            "/dev/ttyACM0": "SER_A",
            "/dev/ttyACM1": "SER_B",
        },
    )


class TestResolvePort:
    def test_hit(self, _mock_ports):
        assert resolve_port("SER_B") == "/dev/ttyACM1"

    def test_miss(self, _mock_ports):
        assert resolve_port("MISSING") is None

    def test_no_ports(self, monkeypatch):
        monkeypatch.setattr(resolve_mod, "find_pico_ports", lambda: {})
        assert resolve_port("SER_A") is None


class TestMain:
    def test_lookup_prints_port(self, _mock_ports, monkeypatch, capsys):
        monkeypatch.setattr("sys.argv", ["picohost-resolve", "SER_A"])
        main()
        assert capsys.readouterr().out.strip() == "/dev/ttyACM0"

    def test_lookup_miss_exits_nonzero(
        self, _mock_ports, monkeypatch, capsys
    ):
        monkeypatch.setattr("sys.argv", ["picohost-resolve", "NOPE"])
        with pytest.raises(SystemExit) as exc:
            main()
        assert exc.value.code == 1
        assert "NOPE" in capsys.readouterr().err

    def test_list_all(self, _mock_ports, monkeypatch, capsys):
        monkeypatch.setattr("sys.argv", ["picohost-resolve"])
        main()
        lines = capsys.readouterr().out.strip().splitlines()
        assert lines == [
            "SER_A\t/dev/ttyACM0",
            "SER_B\t/dev/ttyACM1",
        ]
