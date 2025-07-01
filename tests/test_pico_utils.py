import switch_network
from switch_network.pico_utils import set_switch_states


def reset_pins(pins):
    for pin in pins:
        pin.value(0)


def test_set_switch_states():
    pins = [switch_network.testing.DummyPin(gpio) for gpio in range(8)]
    for pathname, path in switch_network.switch.PATHS.items():
        set_switch_states(path, pins)
        for v, pin in zip(path, pins):
            assert pin.value() == int(v)
    reset_pins(pins)

    # with verify
    for pathname, path in switch_network.switch.PATHS.items():
        send = path + "!"
        out = set_switch_states(send, pins, return_states=True)
        assert out == f"STATES:{path}\n"
        for v, pin in zip(path, pins):
            assert pin.value() == int(v)
    reset_pins(pins)

    # invalid pathlength
    too_short = "0" * (len(pins) - 1) + "!"
    out = set_switch_states(too_short, pins, return_states=True)
    assert out is None
    reset_pins(pins)

    # invalid path, containing 2
    path = "0" * (len(pins) - 1) + "2" + "!"
    out = set_switch_states(path, pins, return_states=True)
    assert out is None
    reset_pins(pins)
