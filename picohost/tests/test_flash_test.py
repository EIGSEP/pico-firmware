"""Tests for picohost.flash_test."""

import subprocess
from pathlib import Path

import pytest

from picohost.flash_test import build_picotool_cmd, flash_test_image


class TestBuildPicotoolCmd:
    def test_minimal(self):
        cmd = build_picotool_cmd("foo.uf2")
        assert cmd == ["picotool", "load", "-f", "-x", "foo.uf2"]

    def test_with_bus_and_address(self):
        cmd = build_picotool_cmd("foo.uf2", bus=1, address=4)
        assert cmd == [
            "picotool", "load", "-f", "-x", "foo.uf2",
            "--bus", "1", "--address", "4",
        ]

    def test_accepts_path(self):
        cmd = build_picotool_cmd(Path("foo.uf2"))
        assert cmd[-1] == "foo.uf2"


class TestFlashTestImage:
    def test_missing_uf2_raises(self, tmp_path):
        with pytest.raises(FileNotFoundError, match="UF2 file not found"):
            flash_test_image(tmp_path / "nonexistent.uf2")

    def test_invokes_picotool(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        captured = {}

        def fake_run(cmd, **kwargs):
            captured["cmd"] = cmd
            return subprocess.CompletedProcess(cmd, 0, stdout="ok\n")

        monkeypatch.setattr(subprocess, "run", fake_run)
        flash_test_image(uf2)

        assert captured["cmd"] == [
            "picotool", "load", "-f", "-x", str(uf2),
        ]

    def test_passes_bus_and_address(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        captured = {}

        def fake_run(cmd, **kwargs):
            captured["cmd"] = cmd
            return subprocess.CompletedProcess(cmd, 0, stdout="ok\n")

        monkeypatch.setattr(subprocess, "run", fake_run)
        flash_test_image(uf2, bus=2, address=7)

        assert captured["cmd"][-4:] == ["--bus", "2", "--address", "7"]

    def test_nonzero_exit_raises(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        def fake_run(cmd, **kwargs):
            return subprocess.CompletedProcess(cmd, 1, stdout="boom\n")

        monkeypatch.setattr(subprocess, "run", fake_run)
        with pytest.raises(RuntimeError, match="picotool failed"):
            flash_test_image(uf2)
