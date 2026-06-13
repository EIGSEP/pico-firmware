"""Stop/restart the PicoManager systemd unit around a flash.

flash-picos must not run while PicoManager holds the Picos' CDC
ports: neither side opens the tty exclusively, so the post-flash
device-info readback races the manager's reconnect loop and both
read interleaved garbage. The unit name matches the eigsep-field
image's ``[services.picomanager]`` entry and the file shipped at
``picohost/picomanager.service``.

``eigsep-field patch pico-firmware`` stops/starts the unit itself
around its own flash-picos invocation; callers here must therefore
only restart the manager when they were the ones to stop it (see
``flash_picos.main``), or the unit would come back up before
eigsep-field writes its ExecStart drop-in.
"""

import logging
import subprocess

logger = logging.getLogger(__name__)

MANAGER_UNIT = "picomanager.service"


def _systemctl(*args, sudo=False):
    """Run ``systemctl *args``; return the CompletedProcess or None.

    ``sudo=True`` prefixes ``sudo -n`` (non-interactive — never
    prompts; fails instead). Returns ``None`` when the binary cannot be
    executed (e.g. not installed on a dev host without systemd), which
    callers treat as "no manager here".
    """
    cmd = ["systemctl", *args]
    if sudo:
        cmd = ["sudo", "-n", *cmd]
    try:
        return subprocess.run(cmd, capture_output=True, text=True)
    except OSError:
        return None


def manager_is_active():
    """Return True when ``picomanager.service`` is currently active."""
    res = _systemctl("is-active", "--quiet", MANAGER_UNIT)
    return res is not None and res.returncode == 0


def _stop_or_start(verb):
    """``systemctl <verb> picomanager.service``, retrying via sudo -n.

    ``--no-ask-password`` keeps the unprivileged attempt from hanging
    on an interactive polkit prompt — it fails immediately instead,
    handing off to the passwordless-sudo fallback.
    """
    res = _systemctl(verb, "--no-ask-password", MANAGER_UNIT)
    if res is not None and res.returncode == 0:
        return True
    res = _systemctl(verb, "--no-ask-password", MANAGER_UNIT, sudo=True)
    return res is not None and res.returncode == 0


def stop_manager():
    """Stop the manager; raise RuntimeError if it cannot be stopped.

    Flashing with the manager still attached is exactly the flaky
    behavior the auto-stop exists to prevent, so failure here should
    abort the flash rather than degrade it.
    """
    if not _stop_or_start("stop"):
        raise RuntimeError(
            f"{MANAGER_UNIT} is active but could not be stopped "
            "(tried `systemctl stop` and `sudo -n systemctl stop`). "
            f"Stop it manually (sudo systemctl stop {MANAGER_UNIT}) "
            "or re-run with --keep-manager."
        )


def start_manager():
    """Best-effort restart after the flash; logs instead of raising.

    The flash already happened — failing the whole run because the
    restart needs a password would mislead more than it helps.
    """
    if not _stop_or_start("start"):
        logger.error(
            "could not restart %s; start it manually: sudo systemctl start %s",
            MANAGER_UNIT,
            MANAGER_UNIT,
        )
