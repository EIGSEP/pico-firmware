"""Tests for picohost.flash_picos.flash_and_discover."""

import types

import pytest

from picohost.flash_picos import (
    _resolve_post_flash_port,
    flash_and_discover,
    flash_uf2,
)


@pytest.fixture
def _mock_flash(monkeypatch, tmp_path):
    """
    Patch USB discovery, picotool flashing, and serial reading so
    flash_and_discover can run without hardware.
    """
    import picohost.flash_picos as fp

    uf2 = tmp_path / "test.uf2"
    uf2.write_bytes(b"\x00")

    monkeypatch.setattr(
        fp,
        "find_pico_ports",
        lambda: {
            "/dev/ttyACM0": "SER_A",
            "/dev/ttyACM1": "SER_B",
        },
    )

    flashed = []
    monkeypatch.setattr(
        fp,
        "flash_uf2",
        lambda path, serial: flashed.append(serial),
    )

    serial_data = {
        "/dev/ttyACM0": {"app_id": 0},
        "/dev/ttyACM1": {"app_id": 5},
    }
    monkeypatch.setattr(
        fp,
        "read_json_from_serial",
        lambda port, baud, timeout: serial_data[port],
    )

    # Skip polling delays and udev settling in the test environment.
    monkeypatch.setattr(fp.time, "sleep", lambda _: None)
    monkeypatch.setattr(fp, "_udev_settle", lambda: None)

    return uf2, flashed


class TestFlashAndDiscover:
    def test_returns_device_list(self, _mock_flash):
        uf2, flashed = _mock_flash
        devices = flash_and_discover(uf2_path=uf2)
        assert len(devices) == 2
        assert {d["app_id"] for d in devices} == {0, 5}
        assert all("port" in d for d in devices)
        assert all("usb_serial" in d for d in devices)
        assert set(flashed) == {"SER_A", "SER_B"}

    def test_single_port_filter(self, _mock_flash):
        uf2, _ = _mock_flash
        devices = flash_and_discover(uf2_path=uf2, port="/dev/ttyACM0")
        assert len(devices) == 1
        assert devices[0]["port"] == "/dev/ttyACM0"

    def test_usb_serial_filter(self, _mock_flash):
        uf2, flashed = _mock_flash
        devices = flash_and_discover(uf2_path=uf2, usb_serial="SER_B")
        assert len(devices) == 1
        assert devices[0]["usb_serial"] == "SER_B"
        assert devices[0]["port"] == "/dev/ttyACM1"
        assert flashed == ["SER_B"]

    def test_usb_serial_no_match_returns_empty(self, _mock_flash):
        uf2, _ = _mock_flash
        assert flash_and_discover(uf2_path=uf2, usb_serial="MISSING") == []

    def test_port_and_usb_serial_must_both_match(self, _mock_flash):
        uf2, _ = _mock_flash
        # SER_A is on /dev/ttyACM0, so this combination is empty
        devices = flash_and_discover(
            uf2_path=uf2, port="/dev/ttyACM0", usb_serial="SER_B"
        )
        assert devices == []

    def test_missing_uf2_raises(self, tmp_path):
        with pytest.raises(FileNotFoundError, match="UF2 file not found"):
            flash_and_discover(uf2_path=tmp_path / "nonexistent.uf2")

    def test_no_picos_returns_empty(self, monkeypatch, tmp_path):
        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        monkeypatch.setattr(fp, "find_pico_ports", lambda: {})
        assert flash_and_discover(uf2_path=uf2) == []

    def test_flash_failure_skips_device(self, monkeypatch, tmp_path):
        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        monkeypatch.setattr(
            fp,
            "find_pico_ports",
            lambda: {
                "/dev/ttyACM0": "SER_A",
            },
        )
        monkeypatch.setattr(
            fp,
            "flash_uf2",
            lambda path, serial: (_ for _ in ()).throw(
                RuntimeError("picotool failed")
            ),
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        monkeypatch.setattr(fp, "_udev_settle", lambda: None)
        assert flash_and_discover(uf2_path=uf2) == []

    def test_serial_read_failure_skips_device(self, monkeypatch, tmp_path):
        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        monkeypatch.setattr(
            fp,
            "find_pico_ports",
            lambda: {
                "/dev/ttyACM0": "SER_A",
            },
        )
        monkeypatch.setattr(fp, "flash_uf2", lambda path, serial: None)
        monkeypatch.setattr(
            fp,
            "read_json_from_serial",
            lambda port, baud, timeout: (_ for _ in ()).throw(
                RuntimeError("timeout")
            ),
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        monkeypatch.setattr(fp, "_udev_settle", lambda: None)
        assert flash_and_discover(uf2_path=uf2) == []

    def test_settles_udev_before_opening_post_flash_port(
        self, monkeypatch, tmp_path
    ):
        """``udevadm settle`` must run between re-enumeration and the
        serial open. The new ttyACM node exists with driver-default
        permissions before udev applies the rule that chgrps it to
        ``dialout``; opening immediately races against that and fails
        intermittently with ``EACCES``. Settling closes the window.
        """
        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        monkeypatch.setattr(
            fp, "find_pico_ports", lambda: {"/dev/ttyACM0": "SER_A"}
        )
        monkeypatch.setattr(fp, "flash_uf2", lambda path, serial: None)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        events = []
        monkeypatch.setattr(
            fp, "_udev_settle", lambda: events.append("settle")
        )

        def fake_read(port, baud, timeout):
            events.append(("read", port))
            return {"app_id": 0}

        monkeypatch.setattr(fp, "read_json_from_serial", fake_read)

        devices = flash_and_discover(uf2_path=uf2)
        assert len(devices) == 1
        assert events == ["settle", ("read", "/dev/ttyACM0")]

    def test_resolves_current_port_after_reenumeration(
        self, monkeypatch, tmp_path
    ):
        """Picos may land on a different /dev/ttyACMn after the
        post-flash reboot. The recorded ``port`` must reflect the
        current path (looked up via the stable ``usb_serial``), not
        the pre-flash path — otherwise two flashed Picos can collide
        on the same recorded port and a Pico that drifted to an
        unsnapshotted slot is silently dropped.
        """
        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        monkeypatch.setattr(
            fp,
            "find_pico_ports",
            lambda: {
                "/dev/ttyACM0": "SER_A",
                "/dev/ttyACM1": "SER_B",
            },
        )

        # Post-flash, both Picos have re-enumerated to different slots.
        post_flash = {"SER_A": "/dev/ttyACM5", "SER_B": "/dev/ttyACM6"}
        monkeypatch.setattr(
            fp,
            "_resolve_post_flash_port",
            lambda serial: post_flash[serial],
        )

        monkeypatch.setattr(fp, "flash_uf2", lambda path, serial: None)
        serial_data = {
            "/dev/ttyACM5": {"app_id": 0},
            "/dev/ttyACM6": {"app_id": 5},
        }
        monkeypatch.setattr(
            fp,
            "read_json_from_serial",
            lambda port, baud, timeout: serial_data[port],
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        monkeypatch.setattr(fp, "_udev_settle", lambda: None)

        devices = flash_and_discover(uf2_path=uf2)
        by_serial = {d["usb_serial"]: d for d in devices}
        assert by_serial["SER_A"]["port"] == "/dev/ttyACM5"
        assert by_serial["SER_A"]["app_id"] == 0
        assert by_serial["SER_B"]["port"] == "/dev/ttyACM6"
        assert by_serial["SER_B"]["app_id"] == 5

    def test_skips_device_that_does_not_reenumerate(
        self, monkeypatch, tmp_path
    ):
        """A Pico that never re-appears after its post-flash reboot
        must be dropped from the result rather than capturing some
        other Pico's JSON via a stale port.
        """
        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        monkeypatch.setattr(
            fp,
            "find_pico_ports",
            lambda: {
                "/dev/ttyACM0": "SER_A",
                "/dev/ttyACM1": "SER_B",
            },
        )

        # SER_A never re-appears; SER_B comes back fine.
        monkeypatch.setattr(
            fp,
            "_resolve_post_flash_port",
            lambda serial: {"SER_B": "/dev/ttyACM1"}.get(serial),
        )

        monkeypatch.setattr(fp, "flash_uf2", lambda path, serial: None)
        monkeypatch.setattr(
            fp,
            "read_json_from_serial",
            lambda port, baud, timeout: {"app_id": 5},
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        monkeypatch.setattr(fp, "_udev_settle", lambda: None)

        devices = flash_and_discover(uf2_path=uf2)
        assert len(devices) == 1
        assert devices[0]["usb_serial"] == "SER_B"
        assert devices[0]["port"] == "/dev/ttyACM1"

    def test_inter_device_settle_delay_before_second_and_later_flash(
        self, monkeypatch, tmp_path
    ):
        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        monkeypatch.setattr(
            fp,
            "find_pico_ports",
            lambda: {
                "/dev/ttyACM0": "SER_A",
                "/dev/ttyACM1": "SER_B",
                "/dev/ttyACM2": "SER_C",
            },
        )

        events = []
        monkeypatch.setattr(
            fp,
            "flash_uf2",
            lambda path, serial: events.append(("flash", serial)),
        )
        monkeypatch.setattr(
            fp,
            "_resolve_post_flash_port",
            lambda serial: {
                "SER_A": "/dev/ttyACM0",
                "SER_B": "/dev/ttyACM1",
                "SER_C": "/dev/ttyACM2",
            }[serial],
        )
        monkeypatch.setattr(
            fp,
            "read_json_from_serial",
            lambda port, baud, timeout: (
                events.append(("read", port)) or {"app_id": 0}
            ),
        )
        monkeypatch.setattr(fp, "_udev_settle", lambda: None)
        monkeypatch.setattr(
            fp.time, "sleep", lambda s: events.append(("sleep", s))
        )

        flash_and_discover(uf2_path=uf2)

        assert events == [
            ("flash", "SER_A"),
            ("read", "/dev/ttyACM0"),
            ("sleep", fp._INTER_DEVICE_SETTLE_S),
            ("flash", "SER_B"),
            ("read", "/dev/ttyACM1"),
            ("sleep", fp._INTER_DEVICE_SETTLE_S),
            ("flash", "SER_C"),
            ("read", "/dev/ttyACM2"),
        ]


class TestFlashUf2:
    """picotool's ``-f`` reboots the target into BOOTSEL and must then
    re-discover it before loading; on a busy hub that re-enumeration
    can fall outside picotool's window and fail intermittently. The
    flash step retries with backoff so a transient miss does not
    abandon the device.
    """

    def _patch_run(self, monkeypatch, returncodes):
        """Make subprocess.run return the given codes in sequence."""
        import picohost.flash_picos as fp

        calls = {"n": 0, "cmds": []}

        def fake_run(cmd, **kwargs):
            calls["cmds"].append(cmd)
            rc = returncodes[calls["n"]]
            calls["n"] += 1
            return types.SimpleNamespace(returncode=rc, stdout="picotool out")

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 35, False)
        )
        return calls

    def test_succeeds_on_first_attempt(self, monkeypatch):
        calls = self._patch_run(monkeypatch, [0])
        flash_uf2("x.uf2", "SER_A")
        assert calls["n"] == 1

    def test_retries_then_succeeds(self, monkeypatch):
        calls = self._patch_run(monkeypatch, [1, 1, 0])
        flash_uf2("x.uf2", "SER_A", attempts=3, backoff=0.0)
        assert calls["n"] == 3

    def test_raises_after_exhausting_attempts(self, monkeypatch):
        calls = self._patch_run(monkeypatch, [1, 1, 1])
        with pytest.raises(RuntimeError, match="after 3 attempts"):
            flash_uf2("x.uf2", "SER_A", attempts=3, backoff=0.0)
        assert calls["n"] == 3

    def test_backoff_between_attempts(self, monkeypatch):
        import picohost.flash_picos as fp

        codes = iter([1, 0])
        monkeypatch.setattr(
            fp.subprocess,
            "run",
            lambda cmd, **kw: types.SimpleNamespace(
                returncode=next(codes), stdout=""
            ),
        )
        sleeps = []
        monkeypatch.setattr(fp.time, "sleep", lambda s: sleeps.append(s))
        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 35, False)
        )
        flash_uf2("x.uf2", "SER_A", attempts=3, backoff=2.0)
        assert sleeps == [2.0]

    def test_selects_cdc_by_bus_address_with_force(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (2, 9, False)
        )
        captured = {}

        def fake_run(cmd, **kw):
            captured["cmd"] = cmd
            return types.SimpleNamespace(returncode=0, stdout="")

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        fp.flash_uf2("x.uf2", "SER_A")
        assert captured["cmd"] == [
            "picotool", "load", "-f",
            "--bus", "2", "--address", "9",
            "-x", "x.uf2",
        ]
        assert "--ser" not in captured["cmd"]

    def test_loads_bootsel_device_without_force(self, monkeypatch):
        # A device already in BOOTSEL needs no reboot, so -f is omitted
        # and picotool loads it straight away.
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (2, 9, True)
        )
        captured = {}

        def fake_run(cmd, **kw):
            captured["cmd"] = cmd
            return types.SimpleNamespace(returncode=0, stdout="")

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        fp.flash_uf2("x.uf2", "SER_A")
        assert captured["cmd"] == [
            "picotool", "load",
            "--bus", "2", "--address", "9",
            "-x", "x.uf2",
        ]
        assert "-f" not in captured["cmd"]

    def test_reresolves_address_each_attempt(self, monkeypatch):
        # Attempt 1 reboots the CDC device into BOOTSEL but the load
        # fails; attempt 2 must re-resolve, find it now in BOOTSEL at a
        # new address, and finish the load without -f.
        import picohost.flash_picos as fp

        addrs = iter([(1, 10, False), (1, 22, True)])
        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: next(addrs)
        )
        codes = iter([1, 0])
        seen = []

        def fake_run(cmd, **kw):
            seen.append(cmd)
            return types.SimpleNamespace(returncode=next(codes), stdout="")

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        fp.flash_uf2("x.uf2", "SER_A", attempts=3, backoff=0.0)
        assert seen[0][seen[0].index("--address") + 1] == "10"
        assert "-f" in seen[0]
        assert seen[1][seen[1].index("--address") + 1] == "22"
        assert "-f" not in seen[1]

    def test_unresolvable_attempt_skips_picotool_then_raises(
        self, monkeypatch
    ):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (None, None, None)
        )
        ran = []
        monkeypatch.setattr(
            fp.subprocess, "run", lambda *a, **k: ran.append(1)
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        with pytest.raises(RuntimeError, match="after 3 attempts"):
            fp.flash_uf2("x.uf2", "SER_A", attempts=3, backoff=0.0)
        assert ran == []


class TestResolvePostFlashPort:
    def test_returns_current_path_when_serial_is_present(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp,
            "find_pico_ports",
            lambda: {"/dev/ttyACM3": "SER_X", "/dev/ttyACM4": "SER_Y"},
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        assert _resolve_post_flash_port("SER_X", timeout=0.1) == "/dev/ttyACM3"
        assert _resolve_post_flash_port("SER_Y", timeout=0.1) == "/dev/ttyACM4"

    def test_returns_none_when_serial_never_appears(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp,
            "find_pico_ports",
            lambda: {"/dev/ttyACM3": "SER_X"},
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        assert _resolve_post_flash_port("MISSING", timeout=0.05) is None

    def test_resolves_after_delayed_reenumeration(self, monkeypatch):
        """First poll sees the Pico absent (still rebooting); a later
        poll sees it back. The helper must wait, not give up.
        """
        import picohost.flash_picos as fp

        calls = {"n": 0}

        def fake_find():
            calls["n"] += 1
            if calls["n"] < 3:
                return {}
            return {"/dev/ttyACM7": "SER_Z"}

        monkeypatch.setattr(fp, "find_pico_ports", fake_find)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        assert _resolve_post_flash_port("SER_Z", timeout=1.0) == "/dev/ttyACM7"
        assert calls["n"] >= 3


class TestResolveBusAddress:
    def _make(
        self, root, name, vid, pid, *, serial=None, bus=None, devnum=None
    ):
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

    def test_resolves_cdc_device(self, tmp_path):
        from picohost.flash_picos import _resolve_bus_address

        self._make(
            tmp_path, "1-3", "2e8a", "0009",
            serial="SER_A", bus=1, devnum=35,
        )
        assert _resolve_bus_address("SER_A", sysfs_root=tmp_path) == (
            1,
            35,
            False,
        )

    def test_resolves_bootsel_device_flagged_in_bootsel(self, tmp_path):
        # A device a prior attempt left stranded in BOOTSEL (000f) must
        # still be found, flagged in_bootsel, so a retry can finish the
        # load without another reboot rather than abandoning it.
        from picohost.flash_picos import _resolve_bus_address

        self._make(
            tmp_path, "1-3", "2e8a", "000f",
            serial="SER_A", bus=1, devnum=40,
        )
        assert _resolve_bus_address("SER_A", sysfs_root=tmp_path) == (
            1,
            40,
            True,
        )

    def test_returns_none_when_serial_absent(self, tmp_path):
        from picohost.flash_picos import _resolve_bus_address

        self._make(
            tmp_path, "1-3", "2e8a", "0009",
            serial="SER_A", bus=1, devnum=35,
        )
        assert _resolve_bus_address("MISSING", sysfs_root=tmp_path) == (
            None,
            None,
            None,
        )

    def test_ignores_non_pico_with_matching_serial(self, tmp_path):
        from picohost.flash_picos import _resolve_bus_address

        self._make(
            tmp_path, "1-4", "1234", "5678",
            serial="SER_A", bus=1, devnum=41,
        )
        assert _resolve_bus_address("SER_A", sysfs_root=tmp_path) == (
            None,
            None,
            None,
        )

    def test_returns_none_when_sysfs_missing(self, tmp_path):
        from picohost.flash_picos import _resolve_bus_address

        assert _resolve_bus_address(
            "SER_A", sysfs_root=tmp_path / "nope"
        ) == (None, None, None)
