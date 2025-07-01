import pytest

import serial

import switch_network
from switch_network import SwitchNetwork


@pytest.fixture
def dummy_switch():
    return switch_network.testing.DummySwitchNetwork()


def test_init_failure():
    # different number of pins
    path1 = "0" * 8
    path2 = "0" * 7
    paths = {"test_path1": path1, "test_path2": path2}
    with pytest.raises(ValueError):
        SwitchNetwork(paths=paths)


def test_make_serial_failure(monkeypatch):
    def fake_serial(port, baudrate, timeout=None):
        raise serial.SerialException("Cannot open serial port")

    monkeypatch.setattr(serial, "Serial", fake_serial)
    with pytest.raises(RuntimeError):
        SwitchNetwork()  # can't open serial port


def test_init(dummy_switch):
    # test with default settings
    assert dummy_switch.paths == switch_network.switch.PATHS
    npins = len(next(iter(switch_network.switch.PATHS.values())))
    assert dummy_switch.npins == npins
    assert dummy_switch.ser is not None
    # test with custom paths
    path1 = "0" * 8
    path2 = "1" * 8
    paths = {"test_path1": path1, "test_path2": path2}
    custom_switch = switch_network.testing.DummySwitchNetwork(paths=paths)
    assert custom_switch.paths == paths


def test_switch(dummy_switch, mocker):
    # spy on check_switch
    spy = mocker.spy(SwitchNetwork, "check_switch")
    assert dummy_switch is not None
    # switch paths
    for pathname in dummy_switch.paths:
        path = dummy_switch.paths[pathname]
        dummy_switch.switch(pathname, verify=False)
        spy.assert_not_called()  # check_switch not called
        nread = len(path) + 2
        read = dummy_switch.pico.readline(nread).strip().decode()
        assert read == path
    spy.reset_mock()  # reset spy for next test
    # verify switch states
    for pathname in dummy_switch.paths:
        path = dummy_switch.paths[pathname]
        # runs without error
        dummy_switch.switch(pathname, verify=True)
    assert spy.call_count == len(dummy_switch.paths)
    # failure in switching
    dummy_switch.fail_switch = True
    for pathname in dummy_switch.paths:
        path = dummy_switch.paths[pathname]
        with pytest.raises(RuntimeError):
            dummy_switch.switch(pathname, verify=True)


def test_check_switch(dummy_switch):
    for pathname in dummy_switch.paths:
        path = dummy_switch.paths[pathname]
        back = f"STATES:{path}\n"
        dummy_switch.pico.write(back.encode())
        set_path = SwitchNetwork.check_switch(dummy_switch)
        assert set_path == path
    # send no path
    dummy_switch.pico.write(b"")
    with pytest.raises(TimeoutError):
        SwitchNetwork.check_switch(dummy_switch)
    # send a path without STATES:
    dummy_switch.pico.write(b"0" * len(dummy_switch.setpins) + b"\n")
    with pytest.raises(ValueError):
        SwitchNetwork.check_switch(dummy_switch)


def test_powerdown(dummy_switch):
    # set all pins to 1
    for pin in dummy_switch.setpins:
        pin.value(1)
    dummy_switch.powerdown(verify=False)  # should set all pins to 0
    for pin in dummy_switch.setpins:
        assert pin.value() == 0
    # with verify
    for pin in dummy_switch.setpins:
        pin.value(1)
    dummy_switch.powerdown(verify=True)
    for pin in dummy_switch.setpins:
        assert pin.value() == 0
