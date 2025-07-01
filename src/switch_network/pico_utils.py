def set_switch_states(statestr, pins, return_states=False):
    """
    Take a string of 0s and 1s from the controlling computer and set the
    GPIO pins to the corresponding states.

    Parameters
    ----------
    statestr : str
        A string of 0s and 1s representing the desired states of the
        pins. This may also include a trailing ! character to indicate
        verification.
    pins : list
        A list of machine.Pin objects, corresponding to the GPIO pins.
    return_states : bool
        If True and verify is True, the function will return the current
        states of the pins after setting them. Defaults to False as it
        is not useful on the Pico. This is primarily for testing.

    Returns
    -------
    str
        The current states of the pins as a string if `return_states'
        is True and the command ends with '!', otherwise None.

    """
    verify = statestr.endswith("!")
    if verify:
        statestr = statestr[:-1]  # remove the trailing ! character
    states = [int(i) for i in statestr]  # processes the command
    if len(states) != len(pins):  # not enough states for the number of pins
        return

    for pindex, state in enumerate(states):
        if state not in (0, 1):  # only allow 0 or 1
            return

        # set the value of the pin at pindex
        pins[pindex].value(state)

    if verify:
        reply = "".join(str(pin.value()) for pin in pins)
        out = f"STATES:{reply}"
        if return_states:
            return out + "\n"  # print automatically added the newline
        else:
            print(out)
