"""Tests for picohost.flash_picos.flash_and_discover."""

import types

import pytest

import picohost.flash_picos as fp
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
            [
                "picotool",
                "reboot",
                "-u",
                "-f",
                "--bus",
                "1",
                "--address",
                "104",
            ],
            [
                "picotool",
                "load",
                "--bus",
                "1",
                "--address",
                "107",
                "-x",
                "x.uf2",
            ],
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
            [
                "picotool",
                "load",
                "--bus",
                "1",
                "--address",
                "50",
                "-x",
                "x.uf2",
            ],
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
            [
                "picotool",
                "load",
                "--bus",
                "1",
                "--address",
                "50",
                "-x",
                "x.uf2",
            ],
            [
                "picotool",
                "load",
                "--bus",
                "1",
                "--address",
                "50",
                "-x",
                "x.uf2",
            ],
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
            [
                "picotool",
                "reboot",
                "-u",
                "-f",
                "--bus",
                "1",
                "--address",
                "104",
            ],
            [
                "picotool",
                "load",
                "--bus",
                "1",
                "--address",
                "107",
                "-x",
                "x.uf2",
            ],
            [
                "picotool",
                "load",
                "--bus",
                "1",
                "--address",
                "107",
                "-x",
                "x.uf2",
            ],
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
        monkeypatch.setattr(fp, "_wait_for_bootsel", lambda s: (None, None))
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
        monkeypatch.setattr(fp, "_resolve_bus_address", lambda s: next(seq))
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


class TestReadDeviceInfo:
    """The post-flash device-info readback retries before giving up.

    After the GPIO mass reset the whole fleet re-enumerates at once, so
    a board that flashed fine can briefly fail to yield its JSON — a
    single timeout must not drop it.
    """

    def _patch_common(self, monkeypatch, fp):
        monkeypatch.setattr(fp, "_udev_settle", lambda: None)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

    def test_retries_then_succeeds(self, monkeypatch):
        import picohost.flash_picos as fp

        self._patch_common(monkeypatch, fp)
        monkeypatch.setattr(
            fp, "_resolve_post_flash_port", lambda s: "/dev/ttyACM0"
        )

        attempts = {"n": 0}

        def fake_read(port, baud, timeout):
            attempts["n"] += 1
            if attempts["n"] == 1:  # first read flakes
                raise RuntimeError("Timed out waiting for JSON")
            return {"app_id": 3}

        monkeypatch.setattr(fp, "read_json_from_serial", fake_read)

        data = fp._read_device_info("SER_A", 115200, 1)
        assert data == {
            "app_id": 3,
            "port": "/dev/ttyACM0",
            "usb_serial": "SER_A",
        }
        assert attempts["n"] == 2

    def test_returns_none_after_exhausting_attempts(self, monkeypatch):
        import picohost.flash_picos as fp

        self._patch_common(monkeypatch, fp)
        monkeypatch.setattr(
            fp, "_resolve_post_flash_port", lambda s: "/dev/ttyACM0"
        )

        calls = {"n": 0}

        def always_timeout(port, baud, timeout):
            calls["n"] += 1
            raise RuntimeError("Timed out waiting for JSON")

        monkeypatch.setattr(fp, "read_json_from_serial", always_timeout)

        data = fp._read_device_info("SER_A", 115200, 1, attempts=3)
        assert data is None
        assert calls["n"] == 3

    def test_reresolves_port_each_attempt(self, monkeypatch):
        """The port may rename across re-enumeration; resolve it fresh
        on every attempt rather than reusing a stale path."""
        import picohost.flash_picos as fp

        self._patch_common(monkeypatch, fp)

        ports = iter(["/dev/ttyACM0", "/dev/ttyACM4"])
        monkeypatch.setattr(
            fp, "_resolve_post_flash_port", lambda s: next(ports)
        )

        seen = []

        def fake_read(port, baud, timeout):
            seen.append(port)
            if len(seen) == 1:
                raise RuntimeError("Timed out waiting for JSON")
            return {"app_id": 0}

        monkeypatch.setattr(fp, "read_json_from_serial", fake_read)

        data = fp._read_device_info("SER_A", 115200, 1)
        assert seen == ["/dev/ttyACM0", "/dev/ttyACM4"]
        assert data["port"] == "/dev/ttyACM4"

    def test_gpio_flow_recovers_flaky_readback(self, _mock_gpio_flash):
        """End-to-end: a board whose first read times out is still
        published once the retry succeeds."""
        m = _mock_gpio_flash
        attempts = {"/dev/ttyACM5": 0, "/dev/ttyACM6": 0}

        def flaky_read(port, baud, timeout):
            attempts[port] += 1
            # SER_B's port flakes once, then yields JSON.
            if port == "/dev/ttyACM6" and attempts[port] == 1:
                raise RuntimeError("Timed out waiting for JSON")
            return {
                "/dev/ttyACM5": {"app_id": 0},
                "/dev/ttyACM6": {"app_id": 5},
            }[port]

        m.monkeypatch.setattr(m.fp, "read_json_from_serial", flaky_read)

        devices = m.fp.flash_and_discover_gpio(uf2_path=m.uf2)

        assert sorted(d["usb_serial"] for d in devices) == ["SER_A", "SER_B"]


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
            tmp_path,
            "1-3",
            "2e8a",
            "0009",
            serial="SER_A",
            bus=1,
            devnum=35,
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
            tmp_path,
            "1-3",
            "2e8a",
            "000f",
            serial="SER_A",
            bus=1,
            devnum=40,
        )
        assert _resolve_bus_address("SER_A", sysfs_root=tmp_path) == (
            1,
            40,
            True,
        )

    def test_returns_none_when_serial_absent(self, tmp_path):
        from picohost.flash_picos import _resolve_bus_address

        self._make(
            tmp_path,
            "1-3",
            "2e8a",
            "0009",
            serial="SER_A",
            bus=1,
            devnum=35,
        )
        assert _resolve_bus_address("MISSING", sysfs_root=tmp_path) == (
            None,
            None,
            None,
        )

    def test_ignores_non_pico_with_matching_serial(self, tmp_path):
        from picohost.flash_picos import _resolve_bus_address

        self._make(
            tmp_path,
            "1-4",
            "1234",
            "5678",
            serial="SER_A",
            bus=1,
            devnum=41,
        )
        assert _resolve_bus_address("SER_A", sysfs_root=tmp_path) == (
            None,
            None,
            None,
        )

    def test_returns_none_when_sysfs_missing(self, tmp_path):
        from picohost.flash_picos import _resolve_bus_address

        assert _resolve_bus_address("SER_A", sysfs_root=tmp_path / "nope") == (
            None,
            None,
            None,
        )


def _make_usb_dev(root, name, vid, pid, *, serial=None, bus=None, devnum=None):
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
            "picotool",
            "load",
            "--bus",
            "2",
            "--address",
            "9",
            "-x",
            "x.uf2",
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
            "picotool",
            "load",
            "--bus",
            "1",
            "--address",
            "42",
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
            tmp_path,
            "1-4",
            "2e8a",
            "000f",
            serial="SER_B",
            bus=1,
            devnum=51,
        )
        _make_usb_dev(
            tmp_path,
            "1-3",
            "2e8a",
            "000f",
            serial="SER_A",
            bus=1,
            devnum=50,
        )
        # CDC pico and non-pico devices must be ignored.
        _make_usb_dev(
            tmp_path,
            "1-5",
            "2e8a",
            "0009",
            serial="SER_C",
            bus=1,
            devnum=52,
        )
        _make_usb_dev(
            tmp_path,
            "1-6",
            "1234",
            "5678",
            serial="SER_D",
            bus=1,
            devnum=53,
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


@pytest.fixture
def _mock_gpio_flash(monkeypatch, tmp_path):
    """Patch GPIO actions, sysfs scans, picotool, and serial reads so
    flash_and_discover_gpio can run without hardware.

    ``find_pico_ports`` and ``_find_bootsel_devices`` are time-dependent:
    they return the pre-entry view until ``gpio.reset()`` fires, then the
    post-reset view (the flashed fleet back in CDC under their real CDC
    serials, and whatever is still stuck in BOOTSEL). picotool goes
    through a fake subprocess.run so tests can assert the actual argv
    (no reboot, no -x, no -f).

    Tests tweak behaviour by mutating the exposed dicts/lists in place:
    ``bootsel`` (BOOTSEL set), ``pre_cdc`` (CDC before entry), ``post_cdc``
    (CDC after reset), ``reads`` (port → JSON), ``stuck`` (still in
    BOOTSEL after reset).
    """
    import picohost.flash_picos as fp
    import picohost.gpio as gpio_mod

    uf2 = tmp_path / "test.uf2"
    uf2.write_bytes(b"\x00")

    state = {"reset_done": False}
    events = []
    monkeypatch.setattr(
        gpio_mod, "enter_bootsel", lambda: events.append("enter_bootsel")
    )

    def fake_reset():
        events.append("reset")
        state["reset_done"] = True

    monkeypatch.setattr(gpio_mod, "reset", fake_reset)

    bootsel = [
        {"usb_serial": "SER_A", "bus": 1, "address": 50},
        {"usb_serial": "SER_B", "bus": 1, "address": 51},
    ]
    # CDC view before the mass entry (only used for the missing-from-
    # BOOTSEL warning).
    pre_cdc = {"/dev/ttyACM0": "SER_A", "/dev/ttyACM1": "SER_B"}
    # CDC view after the mass reset: the flashed fleet, at new /dev nodes
    # and under their real CDC serials.
    post_cdc = {"/dev/ttyACM5": "SER_A", "/dev/ttyACM6": "SER_B"}
    reads = {"/dev/ttyACM5": {"app_id": 0}, "/dev/ttyACM6": {"app_id": 5}}
    # Boards still in BOOTSEL after the reset (default: none).
    stuck = []

    monkeypatch.setattr(
        fp, "_wait_for_stable_bootsel_set", lambda: list(bootsel)
    )

    def fake_find_bootsel():
        return list(stuck) if state["reset_done"] else list(bootsel)

    monkeypatch.setattr(fp, "_find_bootsel_devices", fake_find_bootsel)

    def fake_find_pico_ports():
        return dict(post_cdc) if state["reset_done"] else dict(pre_cdc)

    monkeypatch.setattr(fp, "find_pico_ports", fake_find_pico_ports)

    cmds = []

    def fake_run(cmd, **kw):
        cmds.append(cmd)
        events.append(("load", cmd[cmd.index("--address") + 1]))
        return types.SimpleNamespace(returncode=0, stdout="")

    monkeypatch.setattr(fp.subprocess, "run", fake_run)

    monkeypatch.setattr(
        fp,
        "_resolve_post_flash_port",
        lambda s: {v: k for k, v in post_cdc.items()}.get(s),
    )
    monkeypatch.setattr(fp, "_udev_settle", lambda: None)
    monkeypatch.setattr(
        fp,
        "read_json_from_serial",
        lambda port, baud, timeout: reads[port],
    )
    monkeypatch.setattr(fp.time, "sleep", lambda _: None)

    return types.SimpleNamespace(
        uf2=uf2,
        events=events,
        cmds=cmds,
        bootsel=bootsel,
        pre_cdc=pre_cdc,
        post_cdc=post_cdc,
        reads=reads,
        stuck=stuck,
        fp=fp,
        monkeypatch=monkeypatch,
    )


class TestFlashAndDiscoverGpio:
    def test_happy_path_returns_device_list(self, _mock_gpio_flash):
        m = _mock_gpio_flash
        devices = m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        by_serial = {d["usb_serial"]: d for d in devices}
        assert by_serial["SER_A"]["port"] == "/dev/ttyACM5"
        assert by_serial["SER_A"]["app_id"] == 0
        assert by_serial["SER_B"]["port"] == "/dev/ttyACM6"
        assert by_serial["SER_B"]["app_id"] == 5

    def test_never_invokes_picotool_reboot(self, _mock_gpio_flash):
        # The whole point of the GPIO path: BOOTSEL entry happens on
        # the shared GPIO lines, never via per-device USB reboots.
        m = _mock_gpio_flash
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert m.cmds, "expected picotool load invocations"
        assert all(c[:2] == ["picotool", "load"] for c in m.cmds)

    def test_loads_omit_execute_and_force(self, _mock_gpio_flash):
        # No -x: devices stay in BOOTSEL until the mass reset. No -f:
        # its reboot path does the live serial-descriptor read that
        # corrupts under hub contention.
        m = _mock_gpio_flash
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        for cmd in m.cmds:
            assert "-x" not in cmd
            assert "-f" not in cmd

    def test_phase_ordering(self, _mock_gpio_flash):
        # enter_bootsel first, then all loads, then exactly one reset.
        m = _mock_gpio_flash
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert m.events[0] == "enter_bootsel"
        assert m.events.count("reset") == 1
        load_idx = [i for i, e in enumerate(m.events) if isinstance(e, tuple)]
        assert load_idx, "expected load events"
        assert m.events.index("reset") > max(load_idx)

    def test_warns_on_snapshot_serial_missing_from_bootsel(
        self, _mock_gpio_flash, caplog
    ):
        # SER_C was a live CDC Pico before the mass entry but never
        # appeared in BOOTSEL — name it so the operator notices.
        m = _mock_gpio_flash
        m.pre_cdc["/dev/ttyACM2"] = "SER_C"
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert "SER_C" in caplog.text

    def test_empty_bootsel_set_raises(self, _mock_gpio_flash):
        m = _mock_gpio_flash
        m.monkeypatch.setattr(m.fp, "_wait_for_stable_bootsel_set", lambda: [])
        with pytest.raises(RuntimeError, match="no Picos entered BOOTSEL"):
            m.fp.flash_and_discover_gpio(uf2_path=m.uf2)

    def test_failed_load_excluded_but_reset_still_fires(
        self, _mock_gpio_flash
    ):
        # SER_A's load never succeeds: it never reaches CDC (stuck in
        # BOOTSEL) so it is absent from the results, but the mass reset
        # must still run (it cannot target — and the stragglers reboot
        # into whatever firmware they have).
        m = _mock_gpio_flash
        del m.post_cdc["/dev/ttyACM5"]  # SER_A doesn't come back to CDC
        m.reads.pop("/dev/ttyACM5", None)
        m.stuck.append({"usb_serial": "SER_A", "bus": 1, "address": 50})

        def fake_run(cmd, **kw):
            m.cmds.append(cmd)
            address = cmd[cmd.index("--address") + 1]
            m.events.append(("load", address))
            rc = 1 if address == "50" else 0
            return types.SimpleNamespace(returncode=rc, stdout="boom")

        m.monkeypatch.setattr(m.fp.subprocess, "run", fake_run)
        devices = m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert [d["usb_serial"] for d in devices] == ["SER_B"]
        assert m.events.count("reset") == 1

    def test_stuck_in_bootsel_after_reset_is_logged(
        self, _mock_gpio_flash, caplog
    ):
        # A board still in BOOTSEL after the mass reset never took the
        # flash — surface it by serial/bus/address for the operator.
        m = _mock_gpio_flash
        m.stuck.append({"usb_serial": "SER_B", "bus": 1, "address": 51})
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert "still in BOOTSEL" in caplog.text
        assert "SER_B" in caplog.text

    def test_load_retry_reresolves_address(self, _mock_gpio_flash):
        # A failed load retries against a freshly resolved address in
        # case the device re-enumerated meanwhile.
        m = _mock_gpio_flash
        m.monkeypatch.setattr(
            m.fp,
            "_find_bootsel_devices",
            lambda: [
                {"usb_serial": "SER_A", "bus": 1, "address": 60},
                {"usb_serial": "SER_B", "bus": 1, "address": 51},
            ],
        )
        codes = iter([1, 0, 0])

        def fake_run(cmd, **kw):
            m.cmds.append(cmd)
            return types.SimpleNamespace(returncode=next(codes), stdout="")

        m.monkeypatch.setattr(m.fp.subprocess, "run", fake_run)
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        ser_a_addrs = [c[c.index("--address") + 1] for c in m.cmds[:2]]
        assert ser_a_addrs == ["50", "60"]

    def test_serialless_bootsel_device_reported_via_cdc(
        self, _mock_gpio_flash
    ):
        # A board with no serial in BOOTSEL still flashes (by bus/
        # address) and, once it reboots into CDC under its real serial,
        # is reported — the readback keys off the CDC serial, not the
        # BOOTSEL one.
        m = _mock_gpio_flash
        m.bootsel.append({"usb_serial": None, "bus": 1, "address": 52})
        m.post_cdc["/dev/ttyACM7"] = "SER_C"
        m.reads["/dev/ttyACM7"] = {"app_id": 6}
        devices = m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        flashed_addrs = {c[c.index("--address") + 1] for c in m.cmds}
        assert "52" in flashed_addrs
        assert {d["usb_serial"] for d in devices} == {
            "SER_A",
            "SER_B",
            "SER_C",
        }

    def test_reports_board_with_mismatched_bootsel_serial(
        self, _mock_gpio_flash
    ):
        # The serial a board presents in BOOTSEL can differ from its CDC
        # serial; keying the readback off the CDC serial still reports it.
        m = _mock_gpio_flash
        m.pre_cdc.clear()  # boards were already in BOOTSEL, not prior CDC
        m.bootsel[0]["usb_serial"] = "BOOTSEL_ONLY"  # SER_A in BOOTSEL
        devices = m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert {d["usb_serial"] for d in devices} == {"SER_A", "SER_B"}

    def test_missing_uf2_raises_before_any_gpio_action(
        self, _mock_gpio_flash, tmp_path
    ):
        m = _mock_gpio_flash
        with pytest.raises(FileNotFoundError, match="UF2 file not found"):
            m.fp.flash_and_discover_gpio(uf2_path=tmp_path / "nonexistent.uf2")
        assert m.events == []


class TestMainRouting:
    def _run_main(self, monkeypatch, argv, gpio_available=True):
        import picohost.flash_picos as fp
        import picohost.gpio as gpio_mod

        monkeypatch.setattr(
            fp.manager_service, "manager_is_active", lambda: False
        )
        calls = []
        monkeypatch.setattr(
            fp,
            "flash_and_discover",
            lambda **kw: calls.append(("usb", kw)) or [{"app_id": 0}],
        )
        monkeypatch.setattr(
            fp,
            "flash_and_discover_gpio",
            lambda **kw: calls.append(("gpio", kw)) or [{"app_id": 0}],
        )
        monkeypatch.setattr(gpio_mod, "available", lambda: gpio_available)
        fp.main(argv + ["--no-redis"])
        return calls

    def test_default_routes_to_gpio(self, monkeypatch):
        calls = self._run_main(monkeypatch, ["--uf2", "x.uf2"])
        assert [c[0] for c in calls] == ["gpio"]

    def test_no_gpio_flag_routes_to_usb(self, monkeypatch):
        calls = self._run_main(monkeypatch, ["--uf2", "x.uf2", "--no-gpio"])
        assert [c[0] for c in calls] == ["usb"]

    def test_port_targeting_routes_to_usb(self, monkeypatch):
        # GPIO mass reset cannot target a single Pico.
        calls = self._run_main(
            monkeypatch, ["--uf2", "x.uf2", "--port", "/dev/ttyACM0"]
        )
        assert [c[0] for c in calls] == ["usb"]
        assert calls[0][1]["port"] == "/dev/ttyACM0"

    def test_usb_serial_targeting_routes_to_usb(self, monkeypatch):
        calls = self._run_main(
            monkeypatch, ["--uf2", "x.uf2", "--usb-serial", "SER_A"]
        )
        assert [c[0] for c in calls] == ["usb"]
        assert calls[0][1]["usb_serial"] == "SER_A"

    def test_gpio_unavailable_fails_fast(self, monkeypatch, capsys):
        # No silent fallback: tell the operator to fix the backend or
        # pass --no-gpio explicitly.
        with pytest.raises(SystemExit) as excinfo:
            self._run_main(
                monkeypatch, ["--uf2", "x.uf2"], gpio_available=False
            )
        assert excinfo.value.code == 1
        assert "--no-gpio" in capsys.readouterr().err

    def test_gpio_flow_runtime_error_exits_nonzero(self, monkeypatch, capsys):
        import picohost.flash_picos as fp
        import picohost.gpio as gpio_mod

        monkeypatch.setattr(
            fp.manager_service, "manager_is_active", lambda: False
        )
        monkeypatch.setattr(gpio_mod, "available", lambda: True)

        def boom(**kw):
            raise RuntimeError(
                "no Picos entered BOOTSEL after mass GPIO entry"
            )

        monkeypatch.setattr(fp, "flash_and_discover_gpio", boom)
        with pytest.raises(SystemExit) as excinfo:
            fp.main(["--uf2", "x.uf2", "--no-redis"])
        assert excinfo.value.code == 1
        assert "BOOTSEL" in capsys.readouterr().err


class TestManagerAutoStop:
    """main() stops an active picomanager around the flash window."""

    def _setup(
        self, monkeypatch, tmp_path, active, devices=None, flash_exc=None
    ):
        uf2 = tmp_path / "fw.uf2"
        uf2.write_bytes(b"\x00")
        events = []
        monkeypatch.setattr(
            fp.manager_service, "manager_is_active", lambda: active
        )
        monkeypatch.setattr(
            fp.manager_service,
            "stop_manager",
            lambda: events.append("stop"),
        )
        monkeypatch.setattr(
            fp.manager_service,
            "start_manager",
            lambda: events.append("start"),
        )

        def fake_flash(**kwargs):
            events.append("flash")
            if flash_exc is not None:
                raise flash_exc
            return list(devices or [])

        monkeypatch.setattr(fp, "flash_and_discover", fake_flash)
        return uf2, events

    def _argv(self, uf2):
        return ["--uf2", str(uf2), "--no-gpio", "--no-redis"]

    def test_stops_then_flashes_then_restarts(self, monkeypatch, tmp_path):
        uf2, events = self._setup(
            monkeypatch,
            tmp_path,
            active=True,
            devices=[{"app_id": 0, "port": "p", "usb_serial": "s"}],
        )
        fp.main(self._argv(uf2))
        assert events == ["stop", "flash", "start"]

    def test_restarts_after_flash_failure(self, monkeypatch, tmp_path):
        uf2, events = self._setup(
            monkeypatch, tmp_path, active=True,
            flash_exc=RuntimeError("boom"),
        )
        with pytest.raises(SystemExit) as excinfo:
            fp.main(self._argv(uf2))
        assert excinfo.value.code == 1
        assert events == ["stop", "flash", "start"]

    def test_restarts_when_no_devices_found(self, monkeypatch, tmp_path):
        uf2, events = self._setup(
            monkeypatch, tmp_path, active=True, devices=[]
        )
        with pytest.raises(SystemExit) as excinfo:
            fp.main(self._argv(uf2))
        assert excinfo.value.code == 1
        assert events == ["stop", "flash", "start"]

    def test_inactive_manager_left_alone(self, monkeypatch, tmp_path):
        # The eigsep-field patch flow: the coordinator already stopped
        # the unit; flash-picos must not restart it behind its back.
        uf2, events = self._setup(
            monkeypatch,
            tmp_path,
            active=False,
            devices=[{"app_id": 0, "port": "p", "usb_serial": "s"}],
        )
        fp.main(self._argv(uf2))
        assert events == ["flash"]

    def test_keep_manager_skips_stop(self, monkeypatch, tmp_path):
        uf2, events = self._setup(
            monkeypatch,
            tmp_path,
            active=True,
            devices=[{"app_id": 0, "port": "p", "usb_serial": "s"}],
        )
        fp.main(self._argv(uf2) + ["--keep-manager"])
        assert events == ["flash"]

    def test_stop_failure_aborts_before_flash(self, monkeypatch, tmp_path, capsys):
        uf2, events = self._setup(
            monkeypatch,
            tmp_path,
            active=True,
            devices=[{"app_id": 0, "port": "p", "usb_serial": "s"}],
        )

        def failing_stop():
            raise RuntimeError("cannot stop")

        monkeypatch.setattr(
            fp.manager_service, "stop_manager", failing_stop
        )
        with pytest.raises(SystemExit) as excinfo:
            fp.main(self._argv(uf2))
        assert excinfo.value.code == 1
        assert "cannot stop" in capsys.readouterr().err
        assert events == []
