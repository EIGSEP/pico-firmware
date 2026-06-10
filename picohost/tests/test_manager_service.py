import subprocess

import pytest

from picohost import manager_service as ms


def _completed(cmd, rc):
    return subprocess.CompletedProcess(cmd, rc, "", "")


class TestManagerIsActive:
    def test_active(self, monkeypatch):
        monkeypatch.setattr(
            ms.subprocess, "run", lambda cmd, **kw: _completed(cmd, 0)
        )
        assert ms.manager_is_active() is True

    def test_inactive(self, monkeypatch):
        monkeypatch.setattr(
            ms.subprocess, "run", lambda cmd, **kw: _completed(cmd, 3)
        )
        assert ms.manager_is_active() is False

    def test_no_systemctl_means_inactive(self, monkeypatch):
        def raise_fnf(cmd, **kw):
            raise FileNotFoundError(cmd[0])

        monkeypatch.setattr(ms.subprocess, "run", raise_fnf)
        assert ms.manager_is_active() is False


class TestStopManager:
    def test_plain_systemctl_suffices(self, monkeypatch):
        calls = []

        def fake_run(cmd, **kw):
            calls.append(cmd)
            return _completed(cmd, 0)

        monkeypatch.setattr(ms.subprocess, "run", fake_run)
        ms.stop_manager()
        assert calls == [
            ["systemctl", "stop", "--no-ask-password", "picomanager.service"]
        ]

    def test_falls_back_to_passwordless_sudo(self, monkeypatch):
        calls = []

        def fake_run(cmd, **kw):
            calls.append(cmd)
            rc = 1 if cmd[0] == "systemctl" else 0
            return _completed(cmd, rc)

        monkeypatch.setattr(ms.subprocess, "run", fake_run)
        ms.stop_manager()
        assert calls == [
            ["systemctl", "stop", "--no-ask-password", "picomanager.service"],
            [
                "sudo",
                "-n",
                "systemctl",
                "stop",
                "--no-ask-password",
                "picomanager.service",
            ],
        ]

    def test_raises_actionable_error_when_both_fail(self, monkeypatch):
        monkeypatch.setattr(
            ms.subprocess, "run", lambda cmd, **kw: _completed(cmd, 1)
        )
        with pytest.raises(RuntimeError, match="--keep-manager"):
            ms.stop_manager()

    def test_raises_when_systemctl_missing(self, monkeypatch):
        def raise_fnf(cmd, **kw):
            raise FileNotFoundError(cmd[0])

        monkeypatch.setattr(ms.subprocess, "run", raise_fnf)
        with pytest.raises(RuntimeError, match="--keep-manager"):
            ms.stop_manager()


class TestStartManager:
    def test_success_is_silent(self, monkeypatch, caplog):
        monkeypatch.setattr(
            ms.subprocess, "run", lambda cmd, **kw: _completed(cmd, 0)
        )
        with caplog.at_level("ERROR"):
            ms.start_manager()
        assert "picomanager" not in caplog.text

    def test_failure_logs_instead_of_raising(self, monkeypatch, caplog):
        monkeypatch.setattr(
            ms.subprocess, "run", lambda cmd, **kw: _completed(cmd, 1)
        )
        with caplog.at_level("ERROR"):
            ms.start_manager()
        assert "picomanager.service" in caplog.text
