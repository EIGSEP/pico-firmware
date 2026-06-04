"""Tests for picohost.flash_test."""

import subprocess
from pathlib import Path

import pytest

from picohost import flash_test as flash_test_mod
from picohost.flash_test import (
    build_picotool_cmd,
    find_bootsel_devices,
    flash_test_image,
    main,
)


def _make_usb_device(
    root, name, vid, pid, *, serial=None, bus=None, devnum=None
):
    """Create a fake sysfs USB device directory for tests."""
    dev = root / name
    dev.mkdir()
    (dev / "idVendor").write_text(vid + "\n")
    (dev / "idProduct").write_text(pid + "\n")
    if serial is not None:
        (dev / "serial").write_text(serial + "\n")
    if bus is not None:
        (dev / "busnum").write_text(f"{bus}\n")
    if devnum is not None:
        (dev / "devnum").write_text(f"{devnum}\n")
    return dev


class TestBuildPicotoolCmd:
    def test_minimal(self):
        cmd = build_picotool_cmd("foo.uf2")
        assert cmd == ["picotool", "load", "-f", "-x", "foo.uf2"]

    def test_with_bus_and_address(self):
        cmd = build_picotool_cmd("foo.uf2", bus=1, address=4)
        assert cmd == [
            "picotool",
            "load",
            "-f",
            "--bus",
            "1",
            "--address",
            "4",
            "-x",
            "foo.uf2",
        ]

    def test_with_usb_serial(self):
        cmd = build_picotool_cmd("foo.uf2", usb_serial="E66160F423456789")
        assert cmd == [
            "picotool",
            "load",
            "-f",
            "--ser",
            "E66160F423456789",
            "-x",
            "foo.uf2",
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
            "picotool",
            "load",
            "-f",
            "-x",
            str(uf2),
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

        assert captured["cmd"] == [
            "picotool",
            "load",
            "-f",
            "--bus",
            "2",
            "--address",
            "7",
            "-x",
            str(uf2),
        ]

    def test_passes_usb_serial(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        captured = {}

        def fake_run(cmd, **kwargs):
            captured["cmd"] = cmd
            return subprocess.CompletedProcess(cmd, 0, stdout="ok\n")

        monkeypatch.setattr(subprocess, "run", fake_run)
        flash_test_image(uf2, usb_serial="E66160F4ABCDEF01")

        assert captured["cmd"] == [
            "picotool",
            "load",
            "-f",
            "--ser",
            "E66160F4ABCDEF01",
            "-x",
            str(uf2),
        ]

    def test_nonzero_exit_raises(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        def fake_run(cmd, **kwargs):
            return subprocess.CompletedProcess(cmd, 1, stdout="boom\n")

        monkeypatch.setattr(subprocess, "run", fake_run)
        with pytest.raises(RuntimeError, match="picotool failed"):
            flash_test_image(uf2)


class TestFindBootselDevices:
    def test_missing_sysfs_returns_empty(self, tmp_path):
        assert find_bootsel_devices(tmp_path / "nope") == []

    def test_skips_non_pico_devices(self, tmp_path):
        _make_usb_device(
            tmp_path,
            "1-1",
            "1234",
            "5678",
            serial="other",
            bus=1,
            devnum=2,
        )
        assert find_bootsel_devices(tmp_path) == []

    def test_skips_pico_in_cdc_mode(self, tmp_path):
        _make_usb_device(
            tmp_path,
            "1-2",
            "2e8a",
            "0009",
            serial="CDCPICO",
            bus=1,
            devnum=3,
        )
        assert find_bootsel_devices(tmp_path) == []

    def test_finds_bootsel_pico(self, tmp_path):
        _make_usb_device(
            tmp_path,
            "1-3",
            "2e8a",
            "000f",
            serial="E66160F4ABCDEF01",
            bus=1,
            devnum=4,
        )
        assert find_bootsel_devices(tmp_path) == [
            {"usb_serial": "E66160F4ABCDEF01", "bus": 1, "address": 4},
        ]

    def test_accepts_uppercase_vid_pid(self, tmp_path):
        _make_usb_device(
            tmp_path,
            "1-4",
            "2E8A",
            "000f",
            serial="UPPER1",
            bus=2,
            devnum=5,
        )
        devs = find_bootsel_devices(tmp_path)
        assert len(devs) == 1
        assert devs[0]["usb_serial"] == "UPPER1"

    def test_finds_multiple(self, tmp_path):
        _make_usb_device(
            tmp_path,
            "1-3",
            "2e8a",
            "000f",
            serial="AAA",
            bus=1,
            devnum=3,
        )
        _make_usb_device(
            tmp_path,
            "1-5",
            "2e8a",
            "000f",
            serial="BBB",
            bus=1,
            devnum=6,
        )
        _make_usb_device(
            tmp_path,
            "2-1",
            "2e8a",
            "0009",
            serial="not-bootsel",
            bus=2,
            devnum=2,
        )
        devs = find_bootsel_devices(tmp_path)
        serials = {d["usb_serial"] for d in devs}
        assert serials == {"AAA", "BBB"}

    def test_missing_optional_fields(self, tmp_path):
        dev = tmp_path / "1-6"
        dev.mkdir()
        (dev / "idVendor").write_text("2e8a\n")
        (dev / "idProduct").write_text("000f\n")
        # no serial, no busnum, no devnum
        devs = find_bootsel_devices(tmp_path)
        assert devs == [
            {"usb_serial": None, "bus": None, "address": None},
        ]


class TestMainAutoDiscover:
    def test_rejects_bus_without_address(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        monkeypatch.setattr(
            "sys.argv", ["flash-test", "--uf2", str(uf2), "--bus", "1"]
        )
        with pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 2

    def test_rejects_address_without_bus(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        monkeypatch.setattr(
            "sys.argv", ["flash-test", "--uf2", str(uf2), "--address", "7"]
        )
        with pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 2

    def test_rejects_usb_serial_with_bus_address(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        monkeypatch.setattr(
            "sys.argv",
            [
                "flash-test",
                "--uf2",
                str(uf2),
                "--usb-serial",
                "AAA",
                "--bus",
                "1",
                "--address",
                "7",
            ],
        )
        with pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 2

    def test_flashes_each_bootsel_device(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        sysfs = tmp_path / "sysfs"
        sysfs.mkdir()
        _make_usb_device(
            sysfs,
            "1-1",
            "2e8a",
            "000f",
            serial="AAA",
            bus=1,
            devnum=3,
        )
        _make_usb_device(
            sysfs,
            "1-2",
            "2e8a",
            "000f",
            serial="BBB",
            bus=1,
            devnum=4,
        )

        monkeypatch.setattr(flash_test_mod, "SYSFS_USB_DEVICES", sysfs)

        calls = []

        def fake_flash(uf2_path, bus=None, address=None, usb_serial=None):
            calls.append(
                {
                    "uf2": str(uf2_path),
                    "bus": bus,
                    "address": address,
                    "usb_serial": usb_serial,
                }
            )

        monkeypatch.setattr(flash_test_mod, "flash_test_image", fake_flash)
        monkeypatch.setattr("sys.argv", ["flash-test", "--uf2", str(uf2)])

        main()

        # Both devices expose bus/address, so we flash by those (not
        # --ser), which is reliable on the congested hub.
        addrs = sorted((c["bus"], c["address"]) for c in calls)
        assert addrs == [(1, 3), (1, 4)]
        for c in calls:
            assert c["usb_serial"] is None
            assert c["uf2"] == str(uf2)

    def test_prefers_bus_address_over_serial(self, monkeypatch, tmp_path):
        # A discovered BOOTSEL device with BOTH serial and bus/address is
        # flashed by bus/address: --ser forces a serial-string descriptor
        # read that intermittently fails on the congested hub.
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        sysfs = tmp_path / "sysfs"
        sysfs.mkdir()
        _make_usb_device(
            sysfs, "1-1", "2e8a", "000f", serial="AAA", bus=1, devnum=9
        )

        monkeypatch.setattr(flash_test_mod, "SYSFS_USB_DEVICES", sysfs)
        calls = []

        def fake_flash(uf2_path, bus=None, address=None, usb_serial=None):
            calls.append(
                {"bus": bus, "address": address, "usb_serial": usb_serial}
            )

        monkeypatch.setattr(flash_test_mod, "flash_test_image", fake_flash)
        monkeypatch.setattr("sys.argv", ["flash-test", "--uf2", str(uf2)])

        main()
        assert calls == [
            {"bus": 1, "address": 9, "usb_serial": None},
        ]

    def test_no_devices_exits_error(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        sysfs = tmp_path / "sysfs"
        sysfs.mkdir()

        monkeypatch.setattr(flash_test_mod, "SYSFS_USB_DEVICES", sysfs)
        called = []

        def fake_flash(*a, **kw):
            called.append((a, kw))

        monkeypatch.setattr(flash_test_mod, "flash_test_image", fake_flash)
        monkeypatch.setattr("sys.argv", ["flash-test", "--uf2", str(uf2)])

        with pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 1
        assert called == []

    def test_explicit_target_skips_discovery(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        sysfs = tmp_path / "sysfs"
        sysfs.mkdir()
        # Two BOOTSEL devices present, but user asked for one specifically.
        _make_usb_device(
            sysfs,
            "1-1",
            "2e8a",
            "000f",
            serial="AAA",
            bus=1,
            devnum=3,
        )
        _make_usb_device(
            sysfs,
            "1-2",
            "2e8a",
            "000f",
            serial="BBB",
            bus=1,
            devnum=4,
        )

        monkeypatch.setattr(flash_test_mod, "SYSFS_USB_DEVICES", sysfs)

        calls = []

        def fake_flash(uf2_path, bus=None, address=None, usb_serial=None):
            calls.append(
                {
                    "bus": bus,
                    "address": address,
                    "usb_serial": usb_serial,
                }
            )

        monkeypatch.setattr(flash_test_mod, "flash_test_image", fake_flash)
        monkeypatch.setattr(
            "sys.argv",
            ["flash-test", "--uf2", str(uf2), "--usb-serial", "BBB"],
        )

        main()
        assert calls == [
            {"bus": None, "address": None, "usb_serial": "BBB"},
        ]

    def test_continues_on_individual_failure(self, monkeypatch, tmp_path):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        sysfs = tmp_path / "sysfs"
        sysfs.mkdir()
        _make_usb_device(
            sysfs,
            "1-1",
            "2e8a",
            "000f",
            serial="AAA",
            bus=1,
            devnum=3,
        )
        _make_usb_device(
            sysfs,
            "1-2",
            "2e8a",
            "000f",
            serial="BBB",
            bus=1,
            devnum=4,
        )

        monkeypatch.setattr(flash_test_mod, "SYSFS_USB_DEVICES", sysfs)

        attempted = []

        def fake_flash(uf2_path, bus=None, address=None, usb_serial=None):
            # Flashed by bus/address now, so key the failure on address.
            attempted.append(address)
            if address == 3:
                raise RuntimeError("picotool failed")

        monkeypatch.setattr(flash_test_mod, "flash_test_image", fake_flash)
        monkeypatch.setattr("sys.argv", ["flash-test", "--uf2", str(uf2)])

        with pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 1
        assert sorted(attempted) == [3, 4]

    def test_falls_back_to_bus_address_when_no_serial(
        self, monkeypatch, tmp_path
    ):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        sysfs = tmp_path / "sysfs"
        sysfs.mkdir()
        dev = sysfs / "1-1"
        dev.mkdir()
        (dev / "idVendor").write_text("2e8a\n")
        (dev / "idProduct").write_text("000f\n")
        (dev / "busnum").write_text("1\n")
        (dev / "devnum").write_text("7\n")
        # no serial file

        monkeypatch.setattr(flash_test_mod, "SYSFS_USB_DEVICES", sysfs)
        calls = []

        def fake_flash(uf2_path, bus=None, address=None, usb_serial=None):
            calls.append(
                {
                    "bus": bus,
                    "address": address,
                    "usb_serial": usb_serial,
                }
            )

        monkeypatch.setattr(flash_test_mod, "flash_test_image", fake_flash)
        monkeypatch.setattr("sys.argv", ["flash-test", "--uf2", str(uf2)])

        main()
        assert calls == [
            {"bus": 1, "address": 7, "usb_serial": None},
        ]

    def test_skips_device_with_incomplete_selector(
        self, monkeypatch, tmp_path
    ):
        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        sysfs = tmp_path / "sysfs"
        sysfs.mkdir()
        _make_usb_device(
            sysfs,
            "1-1",
            "2e8a",
            "000f",
            serial="AAA",
            bus=1,
            devnum=3,
        )
        dev = sysfs / "1-2"
        dev.mkdir()
        (dev / "idVendor").write_text("2e8a\n")
        (dev / "idProduct").write_text("000f\n")
        # no serial, no busnum/devnum -> incomplete selector

        monkeypatch.setattr(flash_test_mod, "SYSFS_USB_DEVICES", sysfs)
        calls = []

        def fake_flash(uf2_path, bus=None, address=None, usb_serial=None):
            calls.append(
                {
                    "bus": bus,
                    "address": address,
                    "usb_serial": usb_serial,
                }
            )

        monkeypatch.setattr(flash_test_mod, "flash_test_image", fake_flash)
        monkeypatch.setattr("sys.argv", ["flash-test", "--uf2", str(uf2)])

        with pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 1
        assert calls == [
            {"bus": 1, "address": 3, "usb_serial": None},
        ]
