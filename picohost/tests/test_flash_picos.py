"""Tests for picohost.flash_picos.flash_and_discover."""

import errno
import json
import logging
import types

import pytest

import picohost.flash_picos as fp
from picohost.flash_picos import (
    _classify_read_failure,
    _log_readback_reconciliation,
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

    cdc = {
        "/dev/ttyACM0": "SER_A",
        "/dev/ttyACM1": "SER_B",
    }
    monkeypatch.setattr(fp, "find_pico_ports", lambda: dict(cdc))
    monkeypatch.setattr(fp, "_wait_for_stable_cdc_set", lambda: dict(cdc))
    monkeypatch.setattr(fp, "_find_bootsel_devices", lambda *a, **k: [])

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
    @pytest.fixture(autouse=True)
    def _stub_discovery(self, monkeypatch):
        # The no-gpio path settles the CDC set and scans for stranded
        # BOOTSEL boards; by default delegate the settle to each test's
        # find_pico_ports mock and report no BOOTSEL boards. Tests that
        # exercise recovery override _find_bootsel_devices.
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_wait_for_stable_cdc_set", lambda: fp.find_pico_ports()
        )
        monkeypatch.setattr(fp, "_find_bootsel_devices", lambda *a, **k: [])

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
            lambda serial, **k: post_flash[serial],
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

        # SER_A is discovered (so it is flashed) but never re-enumerates —
        # absent from find_pico_ports post-flash — so neither the first-pass
        # read nor the sweep can recover it; only SER_B is published.
        monkeypatch.setattr(
            fp,
            "_wait_for_stable_cdc_set",
            lambda: {"/dev/ttyACM0": "SER_A", "/dev/ttyACM1": "SER_B"},
        )
        # Post-flash CDC view: SER_A never re-enumerated.
        monkeypatch.setattr(
            fp,
            "find_pico_ports",
            lambda: {"/dev/ttyACM1": "SER_B"},
        )
        # SER_A never re-appears; SER_B comes back fine.
        monkeypatch.setattr(
            fp,
            "_resolve_post_flash_port",
            lambda serial, **k: {"SER_B": "/dev/ttyACM1"}.get(serial),
        )
        # No boards stranded in BOOTSEL (deterministic; avoids real sysfs).
        monkeypatch.setattr(fp, "_find_bootsel_devices", lambda *a, **k: [])

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
            lambda serial, **k: {
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

    def test_recovers_board_stranded_in_bootsel(self, monkeypatch, tmp_path):
        # A board left in BOOTSEL by a prior aborted run is invisible to
        # find_pico_ports (CDC only); the no-gpio path now recovers it by
        # loading + running it directly (no reboot trigger needed).
        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        monkeypatch.setattr(fp, "find_pico_ports", lambda: {})
        monkeypatch.setattr(
            fp,
            "_find_bootsel_devices",
            lambda *a, **k: [{"usb_serial": "SER_B", "bus": 1, "address": 50}],
        )
        loaded = []
        monkeypatch.setattr(
            fp,
            "_load_bootsel_device",
            lambda dev, path, execute=False: (
                loaded.append((dev["usb_serial"], execute)) or True
            ),
        )
        monkeypatch.setattr(
            fp,
            "_read_device_info",
            lambda serial, baud, **k: {
                "app_id": 5,
                "port": "/dev/ttyACM9",
                "usb_serial": serial,
            },
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        devices = fp.flash_and_discover(uf2_path=uf2)
        assert [d["usb_serial"] for d in devices] == ["SER_B"]
        # Recovered boards load WITH execute=True (run immediately).
        assert loaded == [("SER_B", True)]

    def test_reconciliation_names_flashed_but_unread_board(
        self, monkeypatch, tmp_path, caplog
    ):
        import logging

        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        monkeypatch.setattr(
            fp,
            "find_pico_ports",
            lambda: {"/dev/ttyACM0": "SER_A", "/dev/ttyACM1": "SER_B"},
        )
        monkeypatch.setattr(fp, "flash_uf2", lambda path, serial: None)
        # SER_A reads back; SER_B flashes but never reports.
        monkeypatch.setattr(
            fp,
            "_read_device_info",
            lambda serial, baud, **k: (
                {"app_id": 0, "port": "/dev/ttyACM0", "usb_serial": serial}
                if serial == "SER_A"
                else None
            ),
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        with caplog.at_level(logging.WARNING):
            devices = fp.flash_and_discover(uf2_path=uf2)
        assert [d["usb_serial"] for d in devices] == ["SER_A"]
        assert "SER_B" in caplog.text
        assert "NOT reported" in caplog.text

    def test_expected_warns_when_fewer_discovered(
        self, monkeypatch, tmp_path, caplog
    ):
        import logging

        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")
        monkeypatch.setattr(
            fp, "find_pico_ports", lambda: {"/dev/ttyACM0": "SER_A"}
        )
        monkeypatch.setattr(fp, "flash_uf2", lambda path, serial: None)
        monkeypatch.setattr(
            fp,
            "_read_device_info",
            lambda serial, baud, **k: {
                "app_id": 0,
                "port": "/dev/ttyACM0",
                "usb_serial": serial,
            },
        )
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        with caplog.at_level(logging.WARNING):
            fp.flash_and_discover(uf2_path=uf2, expected=3)
        assert "--expected 3" in caplog.text

    def test_first_pass_uses_fast_timeouts(self, monkeypatch, tmp_path):
        """The per-board inline read gives up fast (short timeouts), so a
        failing board does not stall the run."""
        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        monkeypatch.setattr(
            fp, "find_pico_ports", lambda: {"/dev/ttyACM0": "SER_A"}
        )
        monkeypatch.setattr(fp, "_wait_for_stable_cdc_set", lambda: {"/dev/ttyACM0": "SER_A"})
        monkeypatch.setattr(fp, "_find_bootsel_devices", lambda *a, **k: [])
        monkeypatch.setattr(fp, "flash_uf2", lambda path, serial: None)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

        seen = {}

        def fake_read_device_info(serial, baud, read_timeout=None, reenum_timeout=None):
            seen["read_timeout"] = read_timeout
            seen["reenum_timeout"] = reenum_timeout
            return {"app_id": 0, "port": "/dev/ttyACM0", "usb_serial": serial}

        monkeypatch.setattr(fp, "_read_device_info", fake_read_device_info)
        # No stragglers, so the sweep (added later) is a no-op here.
        fp.flash_and_discover(uf2_path=uf2)

        assert seen["read_timeout"] == fp._FIRST_PASS_READ_TIMEOUT_S
        assert seen["reenum_timeout"] == fp._FIRST_PASS_REENUM_TIMEOUT_S

    def test_straggler_recovered_by_sweep(self, monkeypatch, tmp_path):
        """A board whose first-pass read fails is recovered by the post-loop
        sweep and ends up in the published set."""
        import picohost.flash_picos as fp

        uf2 = tmp_path / "test.uf2"
        uf2.write_bytes(b"\x00")

        cdc = {"/dev/ttyACM0": "SER_A", "/dev/ttyACM1": "SER_B"}
        monkeypatch.setattr(fp, "find_pico_ports", lambda: dict(cdc))
        monkeypatch.setattr(fp, "_wait_for_stable_cdc_set", lambda: dict(cdc))
        monkeypatch.setattr(fp, "_find_bootsel_devices", lambda *a, **k: [])
        monkeypatch.setattr(fp, "flash_uf2", lambda path, serial: None)
        monkeypatch.setattr(fp, "_udev_settle", lambda: None)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        monkeypatch.setattr(
            fp, "_resolve_post_flash_port", lambda serial, **k: {
                "SER_A": "/dev/ttyACM0", "SER_B": "/dev/ttyACM1"
            }[serial]
        )

        # SER_B is mute on its first read (pass 1); every later read succeeds
        # (the quiet-bus sweep).
        state = {"ser_b_reads": 0}

        def read(port, baud, timeout):
            if port == "/dev/ttyACM1":
                state["ser_b_reads"] += 1
                if state["ser_b_reads"] == 1:
                    raise RuntimeError("Timed out waiting for JSON")
                return {"app_id": 5}
            return {"app_id": 0}

        monkeypatch.setattr(fp, "read_json_from_serial", read)

        devices = fp.flash_and_discover(uf2_path=uf2)
        by_serial = {d["usb_serial"]: d for d in devices}
        assert set(by_serial) == {"SER_A", "SER_B"}  # SER_B recovered
        assert by_serial["SER_B"]["app_id"] == 5


class TestFlashUf2:
    """flash_uf2 reboots a CDC Pico into BOOTSEL with ``picotool reboot``
    — which, unlike ``load -f``, does not re-acquire the device by a live
    serial-descriptor read (that read corrupts under bus contention on
    the deep hub) — waits for it to re-enumerate in BOOTSEL via sysfs,
    then loads it by ``--bus/--address`` without ``-f``. Retries with
    backoff.
    """

    def test_cdc_command_enters_bootsel_skips_picotool_reboot(
        self, monkeypatch
    ):
        # PRIMARY path: the firmware {"cmd":"bootsel"} command puts the
        # board into BOOTSEL, so picotool reboot is never run — only load.
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 104, False)
        )
        monkeypatch.setattr(fp, "_enter_bootsel_via_cdc", lambda s: (1, 200))
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
                "200",
                "-x",
                "x.uf2",
            ],
        ]
        assert all(c[1] != "reboot" for c in cmds)

    def test_cdc_device_rebooted_then_loaded(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "_resolve_bus_address", lambda s: (1, 104, False)
        )
        # CDC command fails -> fall back to picotool's USB reset.
        monkeypatch.setattr(
            fp, "_enter_bootsel_via_cdc", lambda s: (None, None)
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
        monkeypatch.setattr(
            fp, "_enter_bootsel_via_cdc", lambda s: (None, None)
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
        monkeypatch.setattr(
            fp, "_enter_bootsel_via_cdc", lambda s: (None, None)
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


class TestSendSerialLine:
    def test_runs_writer_child_with_argv(self, monkeypatch):
        import picohost.flash_picos as fp

        captured = {}

        def fake_run(cmd, **kw):
            captured["cmd"] = cmd
            captured["timeout"] = kw.get("timeout")
            return types.SimpleNamespace(returncode=0)

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        fp._send_serial_line("/dev/ttyACM3", '{"cmd":"bootsel"}', 115200, 5.0)
        assert captured["cmd"][1] == fp._SERIAL_WRITER_SCRIPT
        assert captured["cmd"][2:] == [
            "/dev/ttyACM3",
            "115200",
            '{"cmd":"bootsel"}',
        ]
        assert captured["timeout"] == 5.0

    def test_swallows_writer_timeout(self, monkeypatch):
        import picohost.flash_picos as fp

        def fake_run(cmd, **kw):
            raise fp.subprocess.TimeoutExpired(cmd, kw.get("timeout"))

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        # Must not raise — a wedged writer child is abandoned, not awaited.
        fp._send_serial_line("/dev/ttyACM3", "x", 115200, 0.1)


class TestEnterBootselViaCdc:
    """The no-gpio path triggers BOOTSEL via the firmware CDC command
    ({"cmd":"bootsel"} -> reset_usb_boot in the main loop) first, because
    picotool's own USB reset is unreliable on long-running boards. This
    resolves the board's CDC port, sends the command, and waits for it to
    re-enumerate in BOOTSEL.
    """

    def test_sends_command_and_returns_bootsel_address(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "find_pico_ports", lambda: {"/dev/ttyACM3": "SER_A"}
        )
        sent = {}
        monkeypatch.setattr(
            fp,
            "_send_serial_line",
            lambda port, line, baud, timeout: sent.update(
                port=port, line=line
            ),
        )
        monkeypatch.setattr(fp, "_wait_for_bootsel", lambda s, **k: (1, 77))

        assert fp._enter_bootsel_via_cdc("SER_A") == (1, 77)
        assert sent["port"] == "/dev/ttyACM3"
        assert sent["line"] == fp._BOOTSEL_COMMAND

    def test_no_port_does_not_send_and_returns_none(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "find_pico_ports", lambda: {"/dev/ttyACM3": "OTHER"}
        )

        def must_not_send(*a, **k):
            raise AssertionError("must not send to an unresolved port")

        monkeypatch.setattr(fp, "_send_serial_line", must_not_send)
        assert fp._enter_bootsel_via_cdc("SER_A") == (None, None)

    def test_returns_none_when_board_never_enters_bootsel(self, monkeypatch):
        import picohost.flash_picos as fp

        monkeypatch.setattr(
            fp, "find_pico_ports", lambda: {"/dev/ttyACM3": "SER_A"}
        )
        monkeypatch.setattr(fp, "_send_serial_line", lambda *a, **k: None)
        monkeypatch.setattr(
            fp, "_wait_for_bootsel", lambda s, **k: (None, None)
        )
        assert fp._enter_bootsel_via_cdc("SER_A") == (None, None)


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
    """The post-flash device-info readback is a single attempt.

    Failing boards fail persistently (wrong DIP, failed boot), so a
    board whose read fails is dropped immediately rather than retried.
    """

    def _patch_common(self, monkeypatch, fp):
        monkeypatch.setattr(fp, "_udev_settle", lambda: None)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

    def test_reads_device_info(self, monkeypatch):
        import picohost.flash_picos as fp

        self._patch_common(monkeypatch, fp)
        monkeypatch.setattr(
            fp, "_resolve_post_flash_port", lambda s, **k: "/dev/ttyACM0"
        )
        monkeypatch.setattr(
            fp,
            "read_json_from_serial",
            lambda port, baud, timeout: {"app_id": 3},
        )

        data = fp._read_device_info("SER_A", 115200, 1)
        assert data == {
            "app_id": 3,
            "port": "/dev/ttyACM0",
            "usb_serial": "SER_A",
        }

    def test_returns_none_on_read_failure(self, monkeypatch):
        import picohost.flash_picos as fp

        self._patch_common(monkeypatch, fp)
        monkeypatch.setattr(
            fp, "_resolve_post_flash_port", lambda s, **k: "/dev/ttyACM0"
        )

        calls = {"n": 0}

        def always_timeout(port, baud, timeout):
            calls["n"] += 1
            raise RuntimeError("Timed out waiting for JSON")

        monkeypatch.setattr(fp, "read_json_from_serial", always_timeout)

        data = fp._read_device_info("SER_A", 115200, 1)
        assert data is None
        assert calls["n"] == 1  # no retry

    def test_returns_none_when_port_missing(self, monkeypatch):
        """A Pico that never re-enumerated has no port to read from."""
        import picohost.flash_picos as fp

        self._patch_common(monkeypatch, fp)
        monkeypatch.setattr(fp, "_resolve_post_flash_port", lambda s, **k: None)

        def fail_read(port, baud, timeout):
            raise AssertionError("must not read without a port")

        monkeypatch.setattr(fp, "read_json_from_serial", fail_read)

        assert fp._read_device_info("SER_A", 115200, 1) is None

    def test_gpio_flow_drops_unreadable_board(self, _mock_gpio_flash):
        """End-to-end: a board whose read times out is dropped; the
        rest of the fleet is still published."""
        m = _mock_gpio_flash

        def read(port, baud, timeout):
            if port == "/dev/ttyACM6":  # SER_B never yields JSON
                raise RuntimeError("Timed out waiting for JSON")
            return {"/dev/ttyACM5": {"app_id": 0}}[port]

        m.monkeypatch.setattr(m.fp, "read_json_from_serial", read)

        devices = m.fp.flash_and_discover_gpio(uf2_path=m.uf2)

        assert [d["usb_serial"] for d in devices] == ["SER_A"]


class TestSerialReaderChild:
    """The child reader (_serial_reader.run) must deliver its line BEFORE
    closing the port.

    A board whose USB endpoint is stuck ("-110 mute") can block the
    kernel's cdc_acm teardown: the tty ``os.close()`` never returns. The
    child therefore writes+flushes its one protocol line first, so a line
    read just before a wedging close still reaches the parent.
    """

    def test_emits_data_before_close(self, monkeypatch):
        import io

        import picohost._serial_reader as sr

        events = []

        class FakeSerial:
            def __init__(self, *a, **k):
                pass

            def readline(self):
                return b'{"app_id": 5}\n'

            def close(self):
                events.append("close")

        class RecordingOut(io.StringIO):
            def write(self, s):
                events.append("write")
                return super().write(s)

        monkeypatch.setattr(sr, "Serial", FakeSerial)
        out = RecordingOut()

        assert sr.run("/dev/ttyACM4", 115200, 10, out) == 0
        assert json.loads(out.getvalue()) == {"data": {"app_id": 5}}
        # The line must be written before the (possibly wedging) close.
        assert events == ["write", "close"]

    def test_open_error_preserves_errno(self, monkeypatch):
        import io

        import picohost._serial_reader as sr

        class FailingSerial:
            def __init__(self, *a, **k):
                raise OSError(errno.EACCES, "Permission denied")

        monkeypatch.setattr(sr, "Serial", FailingSerial)
        out = io.StringIO()

        sr.run("/dev/ttyACM4", 115200, 10, out)
        msg = json.loads(out.getvalue())
        assert msg["errno"] == errno.EACCES
        assert "Permission denied" in msg["err"]

    def test_silent_port_reports_timeout(self, monkeypatch):
        import io

        import picohost._serial_reader as sr

        class SilentSerial:
            def __init__(self, *a, **k):
                pass

            def readline(self):
                return b""  # never emits a line

            def close(self):
                pass

        monkeypatch.setattr(sr, "Serial", SilentSerial)
        out = io.StringIO()

        sr.run("/dev/ttyACM4", 115200, 0.1, out)
        assert json.loads(out.getvalue()) == {"timeout": True}


class TestSerialWriterChild:
    """The child writer (_serial_writer.run) sends one line to the CDC
    port (the {"cmd":"bootsel"} reflash trigger) and closes.

    It is fire-and-forget: the caller confirms the reboot by watching
    sysfs for BOOTSEL, not by this child. The newline terminator is added
    by the writer so callers pass the bare command.
    """

    def test_writes_line_with_newline_then_closes(self, monkeypatch):
        import picohost._serial_writer as sw

        events = []

        class FakeSerial:
            def __init__(self, *a, **k):
                pass

            def write(self, b):
                events.append(("write", b))

            def flush(self):
                events.append("flush")

            def close(self):
                events.append("close")

        monkeypatch.setattr(sw, "Serial", FakeSerial)
        assert sw.run("/dev/ttyACM4", 115200, '{"cmd":"bootsel"}') == 0
        assert events == [("write", b'{"cmd":"bootsel"}\n'), "flush", "close"]

    def test_open_failure_returns_nonzero(self, monkeypatch):
        import picohost._serial_writer as sw

        class FailingSerial:
            def __init__(self, *a, **k):
                raise OSError(errno.EBUSY, "resource busy")

        monkeypatch.setattr(sw, "Serial", FailingSerial)
        assert sw.run("/dev/ttyACM4", 115200, '{"cmd":"bootsel"}') == 1


class _FakeReaderProc:
    """Stand-in for the reader child's Popen, backed by a real OS pipe so
    the parent's select()/os.read() path is exercised for real.

    With ``eof=False`` the write end is left open, modelling a child that
    has delivered its line but not exited (its close is wedging). ``wait``
    always raises :class:`~subprocess.TimeoutExpired`, modelling the worst
    case — a child that will not reap within the grace — so every parent
    test also proves :func:`read_json_from_serial` never blocks on it.
    """

    def __init__(self, payload=b"", *, eof=True):
        import os

        r, self._w = os.pipe()
        self.stdout = os.fdopen(r, "rb", buffering=0)
        if payload:
            os.write(self._w, payload)
        if eof:
            os.close(self._w)
            self._w = None
        self.killed = False
        self.returncode = None

    def poll(self):
        return self.returncode

    def kill(self):
        self.killed = True

    def wait(self, timeout=None):
        import subprocess

        raise subprocess.TimeoutExpired("reader", timeout)

    def cleanup(self):
        import os

        if self._w is not None:
            os.close(self._w)
            self._w = None


class TestReadJsonFromSerialChildProcess:
    """read_json_from_serial runs the open/read/close in a child process,
    not a thread, so a wedged CDC port pins the child (which flash-picos
    abandons) and never the flash-picos process — a thread stuck in the
    kernel's uninterruptible cdc_acm close would otherwise keep the
    process from exiting, hanging ``eigsep-field patch`` forever.
    """

    def _patch(self, monkeypatch, fake):
        monkeypatch.setattr(fp.subprocess, "Popen", lambda *a, **k: fake)

    def test_returns_data_even_if_child_does_not_exit(self, monkeypatch):
        fake = _FakeReaderProc(b'{"data": {"app_id": 5}}\n', eof=False)
        self._patch(monkeypatch, fake)
        try:
            assert fp.read_json_from_serial("/dev/ttyACM4", 115200, 10) == {
                "app_id": 5
            }
        finally:
            fake.cleanup()
        assert fake.killed  # the lingering child was abandoned, not awaited

    def test_does_not_leak_a_thread(self, monkeypatch):
        import threading

        base = threading.active_count()
        fake = _FakeReaderProc(b'{"data": {"x": 1}}\n')
        self._patch(monkeypatch, fake)
        fp.read_json_from_serial("/dev/ttyACM4", 115200, 10)
        # The read no longer runs on an in-process thread; nothing it spawns
        # can survive to pin process exit.
        assert threading.active_count() == base

    def test_no_output_within_bound_raises(self, monkeypatch):
        fake = _FakeReaderProc(b"", eof=False)  # never speaks, never closes
        self._patch(monkeypatch, fake)
        monkeypatch.setattr(fp, "_SERIAL_TEARDOWN_GRACE_S", 0.2)
        try:
            with pytest.raises(RuntimeError, match="wedged"):
                fp.read_json_from_serial("/dev/ttyACM4", 115200, 0.2)
        finally:
            fake.cleanup()
        assert fake.killed

    def test_err_line_raises_oserror_with_errno(self, monkeypatch):
        fake = _FakeReaderProc(
            b'{"err": "could not open port", "errno": %d}\n' % errno.EBUSY
        )
        self._patch(monkeypatch, fake)
        with pytest.raises(OSError) as ei:
            fp.read_json_from_serial("/dev/ttyACM4", 115200, 10)
        assert ei.value.errno == errno.EBUSY

    def test_timeout_line_raises_runtimeerror(self, monkeypatch):
        fake = _FakeReaderProc(b'{"timeout": true}\n')
        self._patch(monkeypatch, fake)
        with pytest.raises(RuntimeError, match="Timed out"):
            fp.read_json_from_serial("/dev/ttyACM4", 115200, 10)


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


class TestWaitForStableCdcSet:
    """The no-gpio path settles the CDC device set before flashing, so a
    board slow to enumerate on the contended hub is not silently skipped
    by a single snapshot (the way `find_pico_ports()` alone would)."""

    def _scanner(self, monkeypatch, frames):
        import picohost.flash_picos as fp

        calls = {"n": 0}

        def fake_find():
            idx = min(calls["n"], len(frames) - 1)
            calls["n"] += 1
            return frames[idx]

        monkeypatch.setattr(fp, "find_pico_ports", fake_find)
        return calls

    def test_returns_after_set_stabilizes(self, monkeypatch):
        import picohost.flash_picos as fp

        self._scanner(
            monkeypatch,
            [
                {"/dev/ttyACM0": "A"},
                {"/dev/ttyACM0": "A", "/dev/ttyACM1": "B"},
            ],
        )
        result = fp._wait_for_stable_cdc_set(
            timeout=5.0, stable=0.05, poll=0.005
        )
        assert result == {"/dev/ttyACM0": "A", "/dev/ttyACM1": "B"}

    def test_waits_through_growing_set(self, monkeypatch):
        import picohost.flash_picos as fp

        d1 = {"/dev/ttyACM0": "A"}
        d2 = {"/dev/ttyACM0": "A", "/dev/ttyACM1": "B"}
        self._scanner(monkeypatch, [d1, d1, d1, d2])
        result = fp._wait_for_stable_cdc_set(
            timeout=5.0, stable=0.3, poll=0.005
        )
        assert result == d2

    def test_returns_last_seen_on_timeout(self, monkeypatch):
        import picohost.flash_picos as fp

        self._scanner(monkeypatch, [{"/dev/ttyACM0": "A"}])
        result = fp._wait_for_stable_cdc_set(
            timeout=0.05, stable=10.0, poll=0.005
        )
        assert result == {"/dev/ttyACM0": "A"}


@pytest.fixture
def _mock_gpio_flash(monkeypatch, tmp_path):
    """Patch GPIO entry, sysfs scans, picotool loads, the staggered fleet
    boot, and serial reads so flash_and_discover_gpio runs without
    hardware.

    ``find_pico_ports`` and ``_find_bootsel_devices`` are time-dependent:
    they return the pre-boot view until ``_boot_fleet_staggered`` runs,
    then the post-boot view (the flashed fleet back in CDC under their
    real CDC serials, and whatever is still stuck in BOOTSEL). picotool
    load goes through a fake subprocess.run so tests can assert the argv
    (no reboot, no -x, no -f); the per-board boot itself is mocked as the
    phase boundary (it is unit-tested separately in TestBootFleetStaggered).

    Tests tweak behaviour by mutating the exposed dicts/lists in place:
    ``bootsel`` (BOOTSEL set), ``pre_cdc`` (CDC before entry), ``post_cdc``
    (CDC after boot), ``reads`` (port → JSON), ``stuck`` (still in BOOTSEL
    after boot).
    """
    import picohost.flash_picos as fp
    import picohost.gpio as gpio_mod

    uf2 = tmp_path / "test.uf2"
    uf2.write_bytes(b"\x00")

    state = {"booted": False}
    events = []
    monkeypatch.setattr(
        gpio_mod, "enter_bootsel", lambda: events.append("enter_bootsel")
    )

    bootsel = [
        {"usb_serial": "SER_A", "bus": 1, "address": 50},
        {"usb_serial": "SER_B", "bus": 1, "address": 51},
    ]
    # CDC view before the mass entry (only used for the missing-from-
    # BOOTSEL warning).
    pre_cdc = {"/dev/ttyACM0": "SER_A", "/dev/ttyACM1": "SER_B"}
    # CDC view after the fleet boot: the flashed fleet, at new /dev nodes
    # and under their real CDC serials.
    post_cdc = {"/dev/ttyACM5": "SER_A", "/dev/ttyACM6": "SER_B"}
    reads = {"/dev/ttyACM5": {"app_id": 0}, "/dev/ttyACM6": {"app_id": 5}}
    # Boards still in BOOTSEL after the boot (default: none).
    stuck = []

    monkeypatch.setattr(
        fp, "_wait_for_stable_bootsel_set", lambda: list(bootsel)
    )

    def fake_find_bootsel():
        return list(stuck) if state["booted"] else list(bootsel)

    monkeypatch.setattr(fp, "_find_bootsel_devices", fake_find_bootsel)

    def fake_find_pico_ports():
        return dict(post_cdc) if state["booted"] else dict(pre_cdc)

    monkeypatch.setattr(fp, "find_pico_ports", fake_find_pico_ports)

    # _boot_fleet_staggered is the phase boundary: it flips the view to
    # post-boot and returns the serials that left BOOTSEL (every flashed
    # board except the hardware-stuck ones). The real per-board reboot
    # logic is unit-tested separately.
    def fake_boot_fleet(flashed, **kw):
        events.append("boot_fleet")
        state["booted"] = True
        stuck_serials = {d["usb_serial"] for d in stuck}
        return {
            d["usb_serial"]
            for d in flashed
            if d["usb_serial"] and d["usb_serial"] not in stuck_serials
        }

    monkeypatch.setattr(fp, "_boot_fleet_staggered", fake_boot_fleet)

    cmds = []

    def fake_run(cmd, **kw):
        cmds.append(cmd)
        events.append(("load", cmd[cmd.index("--address") + 1]))
        return types.SimpleNamespace(returncode=0, stdout="")

    monkeypatch.setattr(fp.subprocess, "run", fake_run)

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

    def test_phase2_loads_use_only_picotool_load(self, _mock_gpio_flash):
        # Phase 2 (loading) never reboots: it loads each BOOTSEL device by
        # bus/address and leaves it there. The per-board boot is a separate
        # phase, mocked here (see TestBootFleetStaggered).
        m = _mock_gpio_flash
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert m.cmds, "expected picotool load invocations"
        assert all(c[:2] == ["picotool", "load"] for c in m.cmds)

    def test_loads_omit_execute_and_force(self, _mock_gpio_flash):
        # No -x: devices stay in BOOTSEL until the per-board boot. No -f:
        # its reboot path does the live serial-descriptor read that
        # corrupts under hub contention.
        m = _mock_gpio_flash
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        for cmd in m.cmds:
            assert "-x" not in cmd
            assert "-f" not in cmd

    def test_phase_ordering(self, _mock_gpio_flash):
        # enter_bootsel first, then all loads, then the staggered boot.
        m = _mock_gpio_flash
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert m.events[0] == "enter_bootsel"
        assert m.events.count("boot_fleet") == 1
        load_idx = [i for i, e in enumerate(m.events) if isinstance(e, tuple)]
        assert load_idx, "expected load events"
        assert m.events.index("boot_fleet") > max(load_idx)

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

    def test_failed_load_excluded_but_boot_still_fires(self, _mock_gpio_flash):
        # SER_A's load never succeeds, so it is not booted and never
        # reaches CDC — absent from the results — but the fleet boot must
        # still run for the boards that did load.
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
        assert m.events.count("boot_fleet") == 1

    def test_stuck_in_bootsel_is_logged_as_hardware(
        self, _mock_gpio_flash, caplog
    ):
        # A flashed board that will not leave BOOTSEL even after its
        # per-board reboot is the hardware case — surface it by
        # serial/bus/address for the operator.
        m = _mock_gpio_flash
        del m.post_cdc["/dev/ttyACM6"]  # SER_B never reaches CDC
        m.reads.pop("/dev/ttyACM6", None)
        m.stuck.append({"usb_serial": "SER_B", "bus": 1, "address": 51})
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert "still in BOOTSEL" in caplog.text
        assert "held low at boot" in caplog.text
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
            # Only the load calls draw from `codes`; the post-reset
            # straggler reboots (this static mock keeps reporting both
            # boards as in BOOTSEL) succeed without consuming it.
            rc = next(codes) if cmd[1] == "load" else 0
            return types.SimpleNamespace(returncode=rc, stdout="")

        m.monkeypatch.setattr(m.fp.subprocess, "run", fake_run)
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        load_addrs = [
            c[c.index("--address") + 1] for c in m.cmds if c[1] == "load"
        ]
        assert load_addrs[:2] == ["50", "60"]

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

    def test_reconciliation_confirms_all_reported(
        self, _mock_gpio_flash, caplog
    ):
        # Happy path: the readback report confirms every flashed board
        # reported, so an operator can trust the count at a glance.
        m = _mock_gpio_flash
        with caplog.at_level(logging.INFO, logger="picohost.flash_picos"):
            m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert "all 2 flashed Pico(s) reported" in caplog.text

    def test_reconciliation_names_silent_board(self, _mock_gpio_flash, caplog):
        # SER_B re-enumerates but never emits JSON, even on the re-read:
        # the report must name it (with the read-failure reason), not
        # silently drop the count.
        m = _mock_gpio_flash

        def read(port, baud, timeout):
            if port == "/dev/ttyACM6":  # SER_B
                raise RuntimeError("Timed out waiting for JSON")
            return m.reads[port]

        m.monkeypatch.setattr(m.fp, "read_json_from_serial", read)
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert "1 of 2 flashed Pico(s) reported" in caplog.text
        assert "serial=SER_B NOT reported" in caplog.text
        assert "no JSON before timeout" in caplog.text

    def test_mute_board_recovered_by_reread(self, _mock_gpio_flash, caplog):
        # SER_B is mute on the first read (a momentary enumeration race)
        # but answers on a re-read once the bus quiets. It must be
        # recovered by re-READING it — there is no re-flash fallback.
        m = _mock_gpio_flash
        calls = {"/dev/ttyACM6": 0}

        def read(port, baud, timeout):
            if port == "/dev/ttyACM6":  # SER_B: mute on pass 1, OK on re-read
                calls[port] += 1
                if calls[port] == 1:
                    raise RuntimeError("Timed out waiting for JSON")
                return {"app_id": 5}
            return m.reads[port]

        m.monkeypatch.setattr(m.fp, "read_json_from_serial", read)
        with caplog.at_level(logging.INFO, logger="picohost.flash_picos"):
            devices = m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert {d["usb_serial"] for d in devices} == {"SER_A", "SER_B"}
        assert "re-read recovered serial=SER_B" in caplog.text

    def test_absent_board_reported_not_recovered(
        self, _mock_gpio_flash, caplog
    ):
        # A board that never re-enumerated as CDC cannot be read or
        # recovered over USB — it is reported as not re-enumerated, not
        # silently dropped (and there is no re-flash to attempt).
        m = _mock_gpio_flash
        m.bootsel.append({"usb_serial": "SER_D", "bus": 1, "address": 52})
        m.monkeypatch.setattr(m.fp, "_CDC_DISCOVER_TIMEOUT_S", 0.01)
        m.fp.flash_and_discover_gpio(uf2_path=m.uf2)
        assert "serial=SER_D NOT reported" in caplog.text
        assert "did not re-enumerate as CDC" in caplog.text

    def test_missing_uf2_raises_before_any_gpio_action(
        self, _mock_gpio_flash, tmp_path
    ):
        m = _mock_gpio_flash
        with pytest.raises(FileNotFoundError, match="UF2 file not found"):
            m.fp.flash_and_discover_gpio(uf2_path=tmp_path / "nonexistent.uf2")
        assert m.events == []


class TestBootFleetStaggered:
    """_boot_fleet_staggered boots each loaded board into its image one at
    a time via ``picotool reboot -a`` (by bus/address, no --ser/-f),
    retrying a board that has not left BOOTSEL, and returns the booted
    serials."""

    def _run_mock(self, monkeypatch, cmds):
        def fake_run(cmd, **kw):
            cmds.append(cmd)
            return types.SimpleNamespace(returncode=0, stdout="")

        import picohost.flash_picos as fp

        monkeypatch.setattr(fp.subprocess, "run", fake_run)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

    def test_boots_each_board_once_when_they_leave_bootsel(self, monkeypatch):
        import picohost.flash_picos as fp

        flashed = [
            {"usb_serial": "SER_A", "bus": 1, "address": 50},
            {"usb_serial": "SER_B", "bus": 1, "address": 51},
        ]
        # Each board leaves BOOTSEL right after its reboot.
        monkeypatch.setattr(fp, "_find_bootsel_devices", lambda: [])
        cmds = []
        self._run_mock(monkeypatch, cmds)

        booted = fp._boot_fleet_staggered(flashed)
        assert booted == {"SER_A", "SER_B"}
        assert cmds == [
            ["picotool", "reboot", "-a", "--bus", "1", "--address", "50"],
            ["picotool", "reboot", "-a", "--bus", "1", "--address", "51"],
        ]

    def test_hardware_stuck_board_retries_then_excluded(self, monkeypatch):
        import picohost.flash_picos as fp

        flashed = [{"usb_serial": "SER_A", "bus": 1, "address": 50}]
        # SER_A never leaves BOOTSEL: rebooted once per attempt, then
        # excluded from the booted set.
        monkeypatch.setattr(
            fp,
            "_find_bootsel_devices",
            lambda: [{"usb_serial": "SER_A", "bus": 1, "address": 50}],
        )
        cmds = []
        self._run_mock(monkeypatch, cmds)

        booted = fp._boot_fleet_staggered(flashed, attempts=3)
        assert booted == set()
        assert len(cmds) == 3
        assert all(c[:3] == ["picotool", "reboot", "-a"] for c in cmds)
        assert all("--ser" not in c and "-f" not in c for c in cmds)

    def test_retries_from_freshly_resolved_address(self, monkeypatch):
        import picohost.flash_picos as fp

        flashed = [{"usb_serial": "SER_A", "bus": 1, "address": 50}]
        # Attempt 1: still in BOOTSEL at a NEW address (60). Attempt 2:
        # gone. The retry must target the freshly resolved address.
        scans = iter(
            [
                [{"usb_serial": "SER_A", "bus": 1, "address": 60}],
                [],
            ]
        )
        monkeypatch.setattr(fp, "_find_bootsel_devices", lambda: next(scans))
        cmds = []
        self._run_mock(monkeypatch, cmds)

        booted = fp._boot_fleet_staggered(flashed, attempts=3)
        assert booted == {"SER_A"}
        addrs = [c[c.index("--address") + 1] for c in cmds]
        assert addrs == ["50", "60"]

    def test_serialless_board_booted_once_not_tracked(self, monkeypatch):
        import picohost.flash_picos as fp

        flashed = [{"usb_serial": None, "bus": 1, "address": 50}]
        monkeypatch.setattr(
            fp,
            "_find_bootsel_devices",
            lambda: [{"usb_serial": None, "bus": 1, "address": 50}],
        )
        cmds = []
        self._run_mock(monkeypatch, cmds)

        booted = fp._boot_fleet_staggered(flashed)
        # Best-effort single reboot; a serialless board cannot be tracked
        # by serial, so it is not in the returned booted set.
        assert booted == set()
        assert cmds == [
            ["picotool", "reboot", "-a", "--bus", "1", "--address", "50"],
        ]


class TestRereadMuteBoards:
    """_reread_mute_boards re-reads CDC-present boards that lost the first
    read, recovering them without a re-flash."""

    def _patch(self, monkeypatch, fp):
        monkeypatch.setattr(
            fp, "find_pico_ports", lambda: {"/dev/ttyACM6": "SER_B"}
        )
        monkeypatch.setattr(fp, "_udev_settle", lambda: None)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

    def test_recovers_on_second_attempt(self, monkeypatch):
        import picohost.flash_picos as fp

        self._patch(monkeypatch, fp)
        calls = {"n": 0}

        def read(port, baud, timeout):
            calls["n"] += 1
            if calls["n"] == 1:
                raise RuntimeError("Timed out waiting for JSON")
            return {"app_id": 5}

        monkeypatch.setattr(fp, "read_json_from_serial", read)
        devices, outcomes = fp._reread_mute_boards({"SER_B"}, 115200)
        assert [d["usb_serial"] for d in devices] == ["SER_B"]
        assert devices[0]["port"] == "/dev/ttyACM6"
        assert outcomes == {"SER_B": None}

    def test_gives_up_and_returns_reason(self, monkeypatch):
        import picohost.flash_picos as fp

        self._patch(monkeypatch, fp)
        monkeypatch.setattr(
            fp,
            "read_json_from_serial",
            lambda *a, **k: (_ for _ in ()).throw(
                RuntimeError("Timed out waiting for JSON")
            ),
        )
        devices, outcomes = fp._reread_mute_boards(
            {"SER_B"}, 115200, attempts=2
        )
        assert devices == []
        assert outcomes["SER_B"] is not None

    def test_uses_short_read_timeout(self, monkeypatch):
        import picohost.flash_picos as fp

        self._patch(monkeypatch, fp)
        seen = {}

        def read(port, baud, timeout):
            seen["timeout"] = timeout
            return {"app_id": 5}

        monkeypatch.setattr(fp, "read_json_from_serial", read)
        fp._reread_mute_boards({"SER_B"}, 115200)
        assert seen["timeout"] == fp._MUTE_REREAD_TIMEOUT_S

    def test_scans_ports_once_across_attempts(self, monkeypatch):
        # The marginal lidar node stalls when USB descriptors are
        # re-scanned mid-readback, so the port map must be resolved a
        # single time even when recovery spans several read attempts.
        import picohost.flash_picos as fp

        monkeypatch.setattr(fp, "_udev_settle", lambda: None)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)
        scans = {"n": 0}

        def fake_find():
            scans["n"] += 1
            return {"/dev/ttyACM6": "SER_B"}

        monkeypatch.setattr(fp, "find_pico_ports", fake_find)
        reads = {"n": 0}

        def read(port, baud, timeout):
            reads["n"] += 1
            if reads["n"] < 3:
                raise RuntimeError("Timed out waiting for JSON")
            return {"app_id": 5}

        monkeypatch.setattr(fp, "read_json_from_serial", read)
        devices, outcomes = fp._reread_mute_boards({"SER_B"}, 115200)
        assert [d["usb_serial"] for d in devices] == ["SER_B"]
        assert reads["n"] == 3  # took three attempts
        assert scans["n"] == 1  # but only one USB descriptor scan


class TestClassifyReadFailure:
    def test_runtime_timeout_is_silent_firmware(self):
        reason = _classify_read_failure(
            RuntimeError("[/dev/ttyACM0] Timed out waiting for JSON")
        )
        assert "no JSON before timeout" in reason

    def test_eacces_by_errno(self):
        reason = _classify_read_failure(
            OSError(errno.EACCES, "Permission denied")
        )
        assert "EACCES" in reason

    def test_ebusy_by_errno(self):
        reason = _classify_read_failure(
            OSError(errno.EBUSY, "Device or resource busy")
        )
        assert "EBUSY" in reason

    def test_eacces_by_message_when_errno_absent(self):
        # pyserial's SerialException does not always carry errno; fall
        # back to the message text.
        reason = _classify_read_failure(
            OSError("could not open port /dev/ttyACM0: Permission denied")
        )
        assert "EACCES" in reason

    def test_etimedout_is_usb_endpoint_stall(self):
        reason = _classify_read_failure(
            OSError(errno.ETIMEDOUT, "Connection timed out")
        )
        assert "-110" in reason or "ETIMEDOUT" in reason

    def test_etimedout_by_message_when_errno_absent(self):
        reason = _classify_read_failure(
            OSError("read failed: [Errno 110] Connection timed out")
        )
        assert "-110" in reason or "ETIMEDOUT" in reason

    def test_other_oserror_is_verbatim(self):
        reason = _classify_read_failure(OSError("some other failure"))
        assert "some other failure" in reason


class TestLogReadbackReconciliation:
    def _reasons(self, caplog):
        return caplog.text

    def test_all_reported_logs_confirmation(self, caplog):
        with caplog.at_level(logging.INFO, logger="picohost.flash_picos"):
            _log_readback_reconciliation(
                {"A", "B"}, {"A", "B"}, {"A": None, "B": None}
            )
        assert "all 2 flashed Pico(s) reported" in caplog.text

    def test_board_absent_from_cdc_attributed_to_reenumeration(self, caplog):
        _log_readback_reconciliation({"A", "B"}, {"A"}, {"A": None})
        assert "serial=B NOT reported" in caplog.text
        assert "did not re-enumerate as CDC" in caplog.text

    def test_present_but_failed_carries_its_reason(self, caplog):
        _log_readback_reconciliation(
            {"A", "B"}, {"A", "B"}, {"A": None, "B": "port busy (EBUSY ...)"}
        )
        assert "serial=B NOT reported: port busy (EBUSY" in caplog.text

    def test_unexpected_serial_is_named(self, caplog):
        _log_readback_reconciliation({"A"}, {"A", "X"}, {"A": None, "X": None})
        assert (
            "serial=X reported but was not in the flashed set" in caplog.text
        )

    def test_none_baseline_falls_back_to_failure_logs(self, caplog):
        # Without a flashed-serial baseline, still surface read failures.
        _log_readback_reconciliation(
            None, {"A"}, {"A": "permission denied opening port (EACCES ...)"}
        )
        assert "could not read device info for serial=A" in caplog.text
        assert "EACCES" in caplog.text


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

    def test_default_routes_to_usb(self, monkeypatch):
        # The USB per-device path is the default; --gpio opts into the
        # GPIO mass-BOOTSEL flow.
        calls = self._run_main(monkeypatch, ["--uf2", "x.uf2"])
        assert [c[0] for c in calls] == ["usb"]

    def test_gpio_flag_routes_to_gpio(self, monkeypatch):
        calls = self._run_main(monkeypatch, ["--uf2", "x.uf2", "--gpio"])
        assert [c[0] for c in calls] == ["gpio"]

    def test_port_targeting_routes_to_usb(self, monkeypatch):
        # GPIO mass reset cannot target a single Pico (even with --gpio).
        calls = self._run_main(
            monkeypatch, ["--uf2", "x.uf2", "--gpio", "--port", "/dev/ttyACM0"]
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
        # --gpio requested but pinctrl missing: no silent fallback — tell
        # the operator to fix the backend or drop --gpio.
        with pytest.raises(SystemExit) as excinfo:
            self._run_main(
                monkeypatch, ["--uf2", "x.uf2", "--gpio"], gpio_available=False
            )
        assert excinfo.value.code == 1
        assert "--gpio" in capsys.readouterr().err

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
            fp.main(["--uf2", "x.uf2", "--gpio", "--no-redis"])
        assert excinfo.value.code == 1
        assert "BOOTSEL" in capsys.readouterr().err

    def test_expected_passed_through_to_usb_flash(self, monkeypatch):
        calls = self._run_main(
            monkeypatch, ["--uf2", "x.uf2", "--expected", "1"]
        )
        assert calls[0][1]["expected"] == 1

    def test_default_expected_is_seven(self, monkeypatch):
        # With no --expected, the default fleet size of 7 is passed down.
        calls = self._run_main(monkeypatch, ["--uf2", "x.uf2"])
        assert calls[0][1]["expected"] == 7

    def test_expected_shortfall_warns_does_not_fail(self, monkeypatch, capsys):
        # Fewer boards report than --expected: a loud warning, NOT a
        # failure — the run still publishes whatever reported and exits 0.
        import picohost.flash_picos as fp
        import picohost.gpio as gpio_mod

        monkeypatch.setattr(
            fp.manager_service, "manager_is_active", lambda: False
        )
        monkeypatch.setattr(gpio_mod, "available", lambda: True)
        monkeypatch.setattr(
            fp, "flash_and_discover", lambda **kw: [{"app_id": 0}]
        )
        # Must NOT raise SystemExit.
        fp.main(["--uf2", "x.uf2", "--no-redis", "--expected", "3"])
        err = capsys.readouterr().err
        assert "warning" in err.lower()
        assert "expected 3" in err.lower()

    def test_targeting_bypasses_expected_warning(self, monkeypatch, capsys):
        # A deliberate single-board flash must not warn about the fleet
        # count: targeting sets the effective expected to None.
        import picohost.flash_picos as fp
        import picohost.gpio as gpio_mod

        monkeypatch.setattr(
            fp.manager_service, "manager_is_active", lambda: False
        )
        monkeypatch.setattr(gpio_mod, "available", lambda: True)
        calls = []
        monkeypatch.setattr(
            fp,
            "flash_and_discover",
            lambda **kw: calls.append(kw) or [{"app_id": 0}],
        )
        fp.main(["--uf2", "x.uf2", "--no-redis", "--usb-serial", "SER_A"])
        assert "expected" not in capsys.readouterr().err.lower()
        assert calls[0]["expected"] is None

    def test_timeout_flag_removed(self):
        """--timeout was removed; argparse must reject it."""
        import picohost.flash_picos as fp

        with pytest.raises(SystemExit):
            fp.main(["--uf2", "x.uf2", "--no-redis", "--timeout", "5"])


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
        return ["--uf2", str(uf2), "--no-redis"]

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
            monkeypatch,
            tmp_path,
            active=True,
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

    def test_stop_failure_aborts_before_flash(
        self, monkeypatch, tmp_path, capsys
    ):
        uf2, events = self._setup(
            monkeypatch,
            tmp_path,
            active=True,
            devices=[{"app_id": 0, "port": "p", "usb_serial": "s"}],
        )

        def failing_stop():
            raise RuntimeError("cannot stop")

        monkeypatch.setattr(fp.manager_service, "stop_manager", failing_stop)
        with pytest.raises(SystemExit) as excinfo:
            fp.main(self._argv(uf2))
        assert excinfo.value.code == 1
        assert "cannot stop" in capsys.readouterr().err
        assert events == []


class TestReconcileUsbStragglers:
    """The post-loop sweep recovers boards flashed but unreported, on a
    now-quiet bus: re-read CDC-present boards, reload BOOTSEL-stuck ones."""

    def _patch_quiet(self, monkeypatch, fp):
        monkeypatch.setattr(fp, "_udev_settle", lambda: None)
        monkeypatch.setattr(fp.time, "sleep", lambda _: None)

    def test_no_pending_is_noop(self, monkeypatch, tmp_path):
        import picohost.flash_picos as fp

        self._patch_quiet(monkeypatch, fp)

        def boom(*a, **k):
            raise AssertionError("must not touch the bus when nothing pends")

        monkeypatch.setattr(fp, "find_pico_ports", boom)
        monkeypatch.setattr(fp, "_find_bootsel_devices", boom)

        devices, outcomes = fp._reconcile_usb_stragglers(
            {"SER_A", "SER_B"}, {"SER_A", "SER_B"}, tmp_path / "x.uf2", 115200
        )
        assert devices == []
        assert outcomes == {}

    def test_rereads_cdc_straggler(self, monkeypatch, tmp_path):
        import picohost.flash_picos as fp

        self._patch_quiet(monkeypatch, fp)
        monkeypatch.setattr(
            fp, "find_pico_ports", lambda: {"/dev/ttyACM6": "SER_B"}
        )
        monkeypatch.setattr(fp, "_find_bootsel_devices", lambda *a, **k: [])
        monkeypatch.setattr(
            fp, "read_json_from_serial", lambda port, baud, timeout: {"app_id": 5}
        )

        devices, outcomes = fp._reconcile_usb_stragglers(
            {"SER_A", "SER_B"}, {"SER_A"}, tmp_path / "x.uf2", 115200
        )
        assert [d["usb_serial"] for d in devices] == ["SER_B"]
        assert outcomes["SER_B"] is None

    def test_recovers_bootsel_straggler(self, monkeypatch, tmp_path):
        import picohost.flash_picos as fp

        self._patch_quiet(monkeypatch, fp)
        # Not present as CDC, but sitting in BOOTSEL.
        monkeypatch.setattr(fp, "find_pico_ports", lambda: {})
        monkeypatch.setattr(
            fp,
            "_find_bootsel_devices",
            lambda *a, **k: [{"usb_serial": "SER_B", "bus": 1, "address": 9}],
        )
        loaded = []
        monkeypatch.setattr(
            fp,
            "_load_bootsel_device",
            lambda dev, uf2, execute=False: loaded.append(dev["usb_serial"]) or True,
        )
        monkeypatch.setattr(
            fp,
            "_read_device_info",
            lambda serial, baud, **k: {
                "app_id": 5, "port": "/dev/ttyACM6", "usb_serial": serial
            },
        )

        devices, outcomes = fp._reconcile_usb_stragglers(
            {"SER_B"}, set(), tmp_path / "x.uf2", 115200
        )
        assert loaded == ["SER_B"]
        assert [d["usb_serial"] for d in devices] == ["SER_B"]
        assert outcomes["SER_B"] is None

    def test_gives_up_after_cap_on_silent_cdc(self, monkeypatch, tmp_path):
        import picohost.flash_picos as fp

        self._patch_quiet(monkeypatch, fp)
        monkeypatch.setattr(
            fp, "find_pico_ports", lambda: {"/dev/ttyACM6": "SER_B"}
        )
        monkeypatch.setattr(fp, "_find_bootsel_devices", lambda *a, **k: [])
        reads = {"n": 0}

        def always_fail(port, baud, timeout):
            reads["n"] += 1
            raise RuntimeError("Timed out waiting for JSON")

        monkeypatch.setattr(fp, "read_json_from_serial", always_fail)

        devices, outcomes = fp._reconcile_usb_stragglers(
            {"SER_B"}, set(), tmp_path / "x.uf2", 115200
        )
        assert devices == []
        assert outcomes["SER_B"] is not None
        assert reads["n"] == fp._SWEEP_REREAD_ATTEMPTS  # capped, not 5
