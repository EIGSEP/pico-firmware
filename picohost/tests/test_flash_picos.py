"""Tests for picohost.flash_picos.flash_and_discover."""

import types

import pytest

from picohost.flash_picos import (
    _resolve_post_flash_port,
    flash_and_discover,
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
    """flash_uf2 reboots a CDC Pico into BOOTSEL with ``picotool reboot``
    — which, unlike ``load -f``, does not re-acquire the device by a live
    serial-descriptor read (that read corrupts under bus contention on
    the deep hub) — waits for it to re-enumerate in BOOTSEL via sysfs,
    then loads it by ``--bus/--address`` without ``-f``. Retries with
    backoff.
    """

    def test_cdc_device_rebooted_then_loaded(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 104, False)
        )
        monkeypatch.setattr(fp, "_wait_for_bootsel", lambda s: (1, 107))
        cmds = []

        def fake_run(cmd, **kw):
            cmds.append(cmd)
            return types.SimpleNamespace(returncode=0, stdout="")

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        fp.flash_uf2("x.uf2", "SER_A")
        assert cmds == [
            ["picotool", "reboot", "-u", "-f",
             "--bus", "1", "--address", "104"],
            ["picotool", "load",
             "--bus", "1", "--address", "107", "-x", "x.uf2"],
        ]
        assert "--ser" not in cmds[0] and "--ser" not in cmds[1]
        assert "-f" not in cmds[1]

    def test_bootsel_device_loaded_directly(self, monkeypatch):
        # A device already in BOOTSEL is loaded straight away — no reboot.
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 50, True)
        )
        monkeypatch.setattr(
            fp,
            "_wait_for_bootsel",
            lambda s: pytest.fail("must not reboot a BOOTSEL device"),
        )
        cmds = []

        def fake_run(cmd, **kw):
            cmds.append(cmd)
            return types.SimpleNamespace(returncode=0, stdout="")

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        fp.flash_uf2("x.uf2", "SER_A")
        assert cmds == [
            ["picotool", "load",
             "--bus", "1", "--address", "50", "-x", "x.uf2"],
        ]

    def test_retries_load_then_succeeds(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 50, True)
        )
        codes = iter([1, 0])
        cmds = []

        def fake_run(cmd, **kw):
            cmds.append(cmd)
            return types.SimpleNamespace(returncode=next(codes), stdout="")

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        fp.flash_uf2("x.uf2", "SER_A", attempts=3, backoff=0.0)
        assert [c for c in cmds if c[1] == "load"] == [
            ["picotool", "load",
             "--bus", "1", "--address", "50", "-x", "x.uf2"],
            ["picotool", "load",
             "--bus", "1", "--address", "50", "-x", "x.uf2"],
        ]

    def test_raises_after_exhausting_attempts(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 50, True)
        )
        monkeypatch.setattr(
            fp.subprocess,
            "run",
            lambda cmd, **kw: types.SimpleNamespace(
                returncode=1, stdout="boom"
            ),
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        with pytest.raises(RuntimeError, match="after 3 attempts"):
            fp.flash_uf2("x.uf2", "SER_A", attempts=3, backoff=0.0)

    def test_backoff_between_attempts(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 50, True)
        )
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
        fp.flash_uf2("x.uf2", "SER_A", attempts=3, backoff=2.0)
        assert sleeps == [2.0]

    def test_recovers_bootsel_device_on_retry(self, monkeypatch):
        # Attempt 1: reboot CDC->BOOTSEL, load fails. Attempt 2: device
        # is now already in BOOTSEL, so it loads directly (no reboot).
        import picohost.flash_picos as fp

        resolves = iter([(1, 104, False), (1, 107, True)])
        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: next(resolves)
        )
        monkeypatch.setattr(fp, "_wait_for_bootsel", lambda s: (1, 107))
        codes = iter([1, 0])
        cmds = []

        def fake_run(cmd, **kw):
            cmds.append(cmd)
            rc = next(codes) if cmd[1] == "load" else 0
            return types.SimpleNamespace(returncode=rc, stdout="")

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        fp.flash_uf2("x.uf2", "SER_A", attempts=3, backoff=0.0)
        assert cmds == [
            ["picotool", "reboot", "-u", "-f",
             "--bus", "1", "--address", "104"],
            ["picotool", "load",
             "--bus", "1", "--address", "107", "-x", "x.uf2"],
            ["picotool", "load",
             "--bus", "1", "--address", "107", "-x", "x.uf2"],
        ]

    def test_reboot_not_entering_bootsel_retries_then_raises(
        self, monkeypatch
    ):
        # The reboot request is lost on the congested hub: the device
        # never enters BOOTSEL, so no load is attempted and it retries.
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 104, False)
        )
        monkeypatch.setattr(
            fp, "_wait_for_bootsel", lambda s: (None, None)
        )
        cmds = []

        def fake_run(cmd, **kw):
            cmds.append(cmd)
            return types.SimpleNamespace(returncode=0, stdout="")

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        with pytest.raises(RuntimeError, match="after 2 attempts"):
            fp.flash_uf2("x.uf2", "SER_A", attempts=2, backoff=0.0)
        assert all(c[1] == "reboot" for c in cmds)
        assert len(cmds) == 2

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


class TestWaitForBootsel:
    def test_returns_address_when_in_bootsel(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 107, True)
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        assert fp._wait_for_bootsel("SER_A", timeout=0.1) == (1, 107)

    def test_returns_none_on_timeout(self, monkeypatch):
        import picohost.flash_picos as fp

        # Never enters BOOTSEL (stays CDC) -> times out.
        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 104, False)
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        assert fp._wait_for_bootsel("SER_A", timeout=0.05) == (None, None)

    def test_waits_through_cdc_then_bootsel(self, monkeypatch):
        import picohost.flash_picos as fp

        seq = iter([(1, 104, False), (1, 104, False), (1, 107, True)])
        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: next(seq)
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        assert fp._wait_for_bootsel("SER_A", timeout=1.0) == (1, 107)


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


def _make_usb_dev(
    root, name, vid, pid, *, serial=None, bus=None, devnum=None
):
    """Create a fake sysfs USB device directory under *root*."""
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


class TestPicotoolLoad:
    def _capture_run(self, monkeypatch, returncode=0):
        import picohost.flash_picos as fp

        captured = {}

        def fake_run(cmd, **kw):
            captured["cmd"] = cmd
            return types.SimpleNamespace(
                returncode=returncode, stdout="picotool out"
            )

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        return captured

    def test_execute_appends_x(self, monkeypatch):
        from picohost.flash_picos import _picotool_load

        captured = self._capture_run(monkeypatch)
        _picotool_load(2, 9, "x.uf2", execute=True)
        assert captured["cmd"] == [
            "picotool", "load",
            "--bus", "2", "--address", "9",
            "-x", "x.uf2",
        ]

    def test_no_execute_omits_x(self, monkeypatch):
        # The GPIO mass-flash path loads without -x: the device stays in
        # BOOTSEL (no re-enumeration) and a single mass reset at the end
        # boots every pico. -f must never appear either — its reboot
        # path does the live serial-descriptor read that corrupts under
        # hub contention.
        from picohost.flash_picos import _picotool_load

        captured = self._capture_run(monkeypatch)
        _picotool_load(1, 42, "y.uf2", execute=False)
        assert captured["cmd"] == [
            "picotool", "load",
            "--bus", "1", "--address", "42",
            "y.uf2",
        ]
        assert "-x" not in captured["cmd"]
        assert "-f" not in captured["cmd"]

    def test_returns_completed_process(self, monkeypatch):
        from picohost.flash_picos import _picotool_load

        self._capture_run(monkeypatch, returncode=1)
        res = _picotool_load(1, 42, "y.uf2", execute=False)
        assert res.returncode == 1
        assert res.stdout == "picotool out"


class TestFindBootselDevices:
    def test_finds_all_bootsel_devices_sorted(self, tmp_path):
        from picohost.flash_picos import _find_bootsel_devices

        _make_usb_dev(
            tmp_path, "1-4", "2e8a", "000f",
            serial="SER_B", bus=1, devnum=51,
        )
        _make_usb_dev(
            tmp_path, "1-3", "2e8a", "000f",
            serial="SER_A", bus=1, devnum=50,
        )
        # CDC pico and non-pico devices must be ignored.
        _make_usb_dev(
            tmp_path, "1-5", "2e8a", "0009",
            serial="SER_C", bus=1, devnum=52,
        )
        _make_usb_dev(
            tmp_path, "1-6", "1234", "5678",
            serial="SER_D", bus=1, devnum=53,
        )
        assert _find_bootsel_devices(sysfs_root=tmp_path) == [
            {"usb_serial": "SER_A", "bus": 1, "address": 50},
            {"usb_serial": "SER_B", "bus": 1, "address": 51},
        ]

    def test_serial_optional(self, tmp_path):
        # A wiped board can enumerate in BOOTSEL without a serial; it
        # must still be flashable (keyed by bus/address).
        from picohost.flash_picos import _find_bootsel_devices

        _make_usb_dev(tmp_path, "1-3", "2e8a", "000f", bus=1, devnum=50)
        assert _find_bootsel_devices(sysfs_root=tmp_path) == [
            {"usb_serial": None, "bus": 1, "address": 50},
        ]

    def test_skips_device_missing_busnum(self, tmp_path):
        from picohost.flash_picos import _find_bootsel_devices

        _make_usb_dev(tmp_path, "1-3", "2e8a", "000f", serial="SER_A")
        assert _find_bootsel_devices(sysfs_root=tmp_path) == []

    def test_empty_when_sysfs_missing(self, tmp_path):
        from picohost.flash_picos import _find_bootsel_devices

        assert _find_bootsel_devices(sysfs_root=tmp_path / "nope") == []


class TestWaitForStableBootselSet:
    def _scanner(self, monkeypatch, frames):
        """Patch _find_bootsel_devices to step through *frames*, then
        repeat the last frame forever."""
        import picohost.flash_picos as fp

        calls = {"n": 0}

        def fake_find():
            idx = min(calls["n"], len(frames) - 1)
            calls["n"] += 1
            return frames[idx]

        monkeypatch.setattr(fp, "_find_bootsel_devices", fake_find)
        return calls

    def test_returns_after_set_stabilizes(self, monkeypatch):
        import picohost.flash_picos as fp

        d1 = {"usb_serial": "SER_A", "bus": 1, "address": 50}
        d2 = {"usb_serial": "SER_B", "bus": 1, "address": 51}
        self._scanner(monkeypatch, [[], [d1], [d1, d2]])
        result = fp._wait_for_stable_bootsel_set(
            timeout=5.0, stable=0.05, poll=0.005
        )
        assert result == [d1, d2]

    def test_waits_through_growing_set(self, monkeypatch):
        # Devices enumerate one by one; the wait must not latch onto
        # the first non-empty set it sees.
        import picohost.flash_picos as fp

        d1 = {"usb_serial": "SER_A", "bus": 1, "address": 50}
        d2 = {"usb_serial": "SER_B", "bus": 1, "address": 51}
        self._scanner(monkeypatch, [[d1], [d1], [d1], [d1, d2]])
        result = fp._wait_for_stable_bootsel_set(
            timeout=5.0, stable=0.3, poll=0.005
        )
        assert result == [d1, d2]

    def test_empty_on_timeout_when_nothing_appears(self, monkeypatch):
        import picohost.flash_picos as fp

        self._scanner(monkeypatch, [[]])
        result = fp._wait_for_stable_bootsel_set(
            timeout=0.05, stable=0.5, poll=0.005
        )
        assert result == []

    def test_returns_last_seen_on_timeout(self, monkeypatch):
        # The set never stabilizes within the timeout; the last
        # observation is returned so the caller can proceed/warn.
        import picohost.flash_picos as fp

        d1 = {"usb_serial": "SER_A", "bus": 1, "address": 50}
        self._scanner(monkeypatch, [[d1]])
        result = fp._wait_for_stable_bootsel_set(
            timeout=0.05, stable=10.0, poll=0.005
        )
        assert result == [d1]
