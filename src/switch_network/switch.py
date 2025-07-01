import logging
import serial
import time

logger = logging.getLogger(__name__)

PATHS = {
    "VNAO": "10000000",
    "VNAS": "11000000",
    "VNAL": "00100000",
    "VNAANT": "00000100",
    "VNANON": "00000111",
    "VNANOFF": "00000110",
    "VNARF": "00011000",
    "RFNON": "00000011",
    "RFNOFF": "00000010",
    "RFANT": "00000000",
}


class SwitchNetwork:

    def __init__(
        self,
        paths=PATHS,
        serport="/dev/ttyACM0",
        timeout=10,
        redis=None,
    ):
        """
        Initialize the SwitchNetwork class.

        Parameters
        ----------
        paths : dict
            Dictionary mapping path names to their corresponding switch states.
        serport : str
            Serial port for Pico connection.
        timeout : float
            Timeout for each blocking call to the serial port.
        redis : eigsep_observing.EigsepRedis
            Redis instance to push observing modes to.

        Raises
        ------
        ValueError
            If the paths do not have the same number of GPIO pins.

        """
        self.logger = logger
        npins = len(next(iter(paths.values())))
        for path in paths.values():
            if len(path) != npins:
                raise ValueError(
                    "All paths must have the same number of GPIO pins."
                )
        self.paths = paths
        self.npins = npins
        self.inv_paths = {v: k for k, v in paths.items()}
        self.low_power_path = "0" * self.npins  # all GPIOs low
        self.low_power_pathname = self.inv_paths[self.low_power_path]
        self.ser = self._make_serial(serport, timeout=timeout)
        self.redis = redis

    def _make_serial(self, serport, timeout=None):
        """
        Create a serial connection to the Pico.

        Parameters
        ----------
        serport : str
            The serial port to connect to.
        timeout : float

        Returns
        -------
        ser : serial.Serial
            The serial connection object.

        Raises
        ------
        RuntimeError
            If the serial port cannot be opened.

        """
        try:
            ser = serial.Serial(serport, 115200, timeout=timeout)
        except serial.SerialException as e:
            error_msg = f"Could not open serial port {serport}: {e}"
            self.logger.error(error_msg)
            raise RuntimeError(error_msg) from e
        return ser

    def switch(self, pathname, verify=True):
        """
        Set switches at given GPIO pins to the low/high power modes
        specified by paths.

        Parameters
        ----------
        pathname : str
            The key for the path you want to switch to.
        verify : bool
            If True, will verify the switch state after setting it.

        Raises
        -------
        RuntimeError
            If `verify` is True and the switch state does not match the
            requested path.

        """
        path = self.paths[pathname]
        if verify:
            path = path + "!"  # add a verification character
        # clear the serial buffer
        self.ser.reset_input_buffer()
        # encode the path and write to the Pico
        self.ser.write(path.encode() + b"\n")
        self.ser.flush()
        time.sleep(0.05)  # wait for switch
        self.logger.info(f"{pathname} is set.")
        if verify:
            set_path = self.check_switch()
            match = set_path == path[:-1]  # remove the verification character
            if match:
                self.logger.info(f"Switch verified: {set_path}.")
            else:
                raise RuntimeError(
                    f"Switch verification failed: {set_path} != {path[:-1]}."
                )
        if self.redis is not None:
            self.redis.add_metadata("obs_mode", pathname)

    def check_switch(self):
        """
        Check the current switch state by reading from the serial port.

        Returns
        -------
        set_path : str
            The current path set on the switch.

        Raises
        ------
        TimeoutError
            If no reply is received from the switch before timeout.

        ValueError
            If the reply from the switch does not start with "STATES".

        """
        reply = self.ser.readline().decode()
        if not reply:
            self.logger.error("No reply from the switch.")
            raise TimeoutError("No reply from the switch.")
        if not reply.startswith("STATES"):
            self.logger.error(f"Unexpected reply from switch: {reply}")
            raise ValueError(f"Unexpected reply from switch: {reply}")
        set_path = reply.rstrip("\n").split(":")[1]  # remove prefix
        set_path = set_path.strip()
        return set_path

    def powerdown(self, verify=True):
        """
        Switch to the low power state by setting all GPIOs to low.

        Parameters
        ----------
        verify : bool
            If True, will verify the switch state after setting it.

        """
        self.logger.info("Switching to low power mode.")
        self.switch(pathname=self.low_power_pathname, verify=verify)
