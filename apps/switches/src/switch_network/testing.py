from mockserial import create_serial_connection

from . import SwitchNetwork, pico_utils


class DummyPin:
    """
    Mock up of a machine.Pin for testing.
    """

    def __init__(self, gpio):
        self.gpio = gpio
        self._value = 0

    def value(self, val=None):
        """
        Get or set the value of the pin.
        """
        if val is None:
            return self._value
        else:
            self._value = val


class DummySwitchNetwork(SwitchNetwork):
    """
    Mimic SwitchNetwork, but do not open a serial port.
    """

    def __init__(self, *args, **kwargs):
        """
        Initialize the DummySwitchNetwork with a mock serial connection.

        Notes
        -----
        Input arguments are ignored, they are only here for
        compatibility with the original SwitchNetwork class.

        """
        super().__init__(*args, **kwargs)
        self.redis = None
        self.fail_switch = False  # simulate a failure in switching
        # create dummy setpins
        self.setpins = [DummyPin(gpio) for gpio in range(self.npins)]

    def _make_serial(self, serport, timeout=None):
        """
        Create a dummy serial connection.
        """
        ser, self.pico = create_serial_connection(timeout=1)
        return ser

    def _do_switch_on_pico(self):
        """
        Simulate the pico switching.
        """
        nread = self.npins + 2
        command = self.pico.readline(nread).decode().strip()
        if command:
            if self.fail_switch:  # swap 0 and 1
                c1 = command.replace("0", "2")  # swap 0 with 2
                c2 = c1.replace("1", "0")  # swap 1 with 0
                command = c2.replace("2", "1")  # swap 2 with 1
            reply = pico_utils.set_switch_states(
                command, self.setpins, return_states=True
            )
            if reply:
                self.pico.write(reply.encode())

    def check_switch(self):
        """
        Override verify method by mocking a Pico switching and
        responding. If the attribute fail_switch is set to True,
        it will simulate a failure in switching.
        """
        # this part is in scripts/main.py and runs on the Pico
        self._do_switch_on_pico()
        # run the verify method
        return super().check_switch()

    def powerdown(self, verify=True):
        """
        Override powerdown method to simulate power down.
        """
        if verify:  # calls _do_switch_on_pico under the hood
            return super().powerdown(verify=True)
        # need to call _do_switch_on_pico manually
        super().powerdown(verify=False)
        self._do_switch_on_pico()
