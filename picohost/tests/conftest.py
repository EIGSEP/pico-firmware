"""Shared test helpers for picohost test suite."""

import time

_SENTINEL = object()


def wait_for_settle(
    getter,
    *,
    initial=_SENTINEL,
    timeout=None,
    poll_interval=None,
    stable_count=3,
    cadence_ms=None,
    max_cycles=100,
):
    """Poll getter() until the returned value stops changing.

    Returns the settled value so callers can assert on it directly::

        assert wait_for_settle(lambda: motor.status.get("az_pos")) == 500

    When *cadence_ms* is provided, *timeout* and *poll_interval* are derived
    from the emulator cadence unless explicitly overridden::

        timeout     = max_cycles * cadence_ms / 1000
        poll_interval = cadence_ms / 1000

    This ties wait behaviour to the emulator's timing model so tests fail
    fast when convergence takes longer than the model predicts.

    Parameters
    ----------
    getter : callable
        Zero-argument function returning the value to watch.
    initial : object, optional
        If given, skip readings equal to this value (wait for change first).
    timeout : float, optional
        Maximum seconds to wait.  Derived from *cadence_ms* when omitted.
    poll_interval : float, optional
        Seconds between polls.  Derived from *cadence_ms* when omitted.
    stable_count : int
        Number of consecutive identical readings required (default 3).
    cadence_ms : float, optional
        Emulator status cadence in milliseconds.  When provided, *timeout*
        and *poll_interval* default to cadence-derived values.
    max_cycles : int
        Maximum number of emulator cycles to wait (only used when
        *cadence_ms* is provided and *timeout* is not).
    """
    if cadence_ms is not None:
        cadence_s = cadence_ms / 1000.0
        if timeout is None:
            timeout = max_cycles * cadence_s
        if poll_interval is None:
            poll_interval = cadence_s
    else:
        if timeout is None:
            timeout = 2.0
        if poll_interval is None:
            poll_interval = 0.02

    deadline = time.monotonic() + timeout
    prev = _SENTINEL
    run = 0
    while time.monotonic() < deadline:
        val = getter()
        if initial is not _SENTINEL and val == initial:
            time.sleep(poll_interval)
            continue
        if val == prev:
            run += 1
            if run >= stable_count:
                return val
        else:
            prev = val
            run = 1
        time.sleep(poll_interval)
    raise TimeoutError(
        f"Value did not settle within {timeout}s (last value: {prev!r})"
    )


def wait_for_condition(
    predicate,
    *,
    timeout=None,
    poll_interval=None,
    cadence_ms=None,
    max_cycles=100,
):
    """Poll until predicate() returns True.

    When *cadence_ms* is provided, *timeout* and *poll_interval* are derived
    from the emulator cadence unless explicitly overridden.

    Parameters
    ----------
    predicate : callable
        Zero-argument function returning a boolean.
    timeout : float, optional
        Maximum seconds to wait.  Derived from *cadence_ms* when omitted.
    poll_interval : float, optional
        Seconds between polls.  Derived from *cadence_ms* when omitted.
    cadence_ms : float, optional
        Emulator status cadence in milliseconds.
    max_cycles : int
        Maximum emulator cycles to wait (default 100).
    """
    if cadence_ms is not None:
        cadence_s = cadence_ms / 1000.0
        if timeout is None:
            timeout = max_cycles * cadence_s
        if poll_interval is None:
            poll_interval = cadence_s
    else:
        if timeout is None:
            timeout = 2.0
        if poll_interval is None:
            poll_interval = 0.02

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(poll_interval)
    raise TimeoutError(f"Condition not met within {timeout}s")
