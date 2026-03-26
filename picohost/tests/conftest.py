"""Shared test helpers for picohost test suite."""

import time

_SENTINEL = object()


def wait_for_settle(getter, *, initial=_SENTINEL, timeout=2.0, poll_interval=0.02,
                    stable_count=3):
    """Poll getter() until the returned value stops changing.

    Returns the settled value so callers can assert on it directly::

        assert wait_for_settle(lambda: motor.status.get("az_pos")) == 500

    When *initial* is provided, readings equal to *initial* are ignored so
    the helper waits for the value to change before checking stability.

    Parameters
    ----------
    getter : callable
        Zero-argument function returning the value to watch.
    initial : object, optional
        If given, skip readings equal to this value (wait for change first).
    timeout : float
        Maximum seconds to wait before raising TimeoutError.
    poll_interval : float
        Seconds between polls (default 20 ms).
    stable_count : int
        Number of consecutive identical readings required (default 3).
    """
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


def wait_for_condition(predicate, *, timeout=2.0, poll_interval=0.02):
    """Poll until predicate() returns True.

    Parameters
    ----------
    predicate : callable
        Zero-argument function returning a boolean.
    timeout : float
        Maximum seconds to wait before raising TimeoutError.
    poll_interval : float
        Seconds between polls (default 20 ms).
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(poll_interval)
    raise TimeoutError(f"Condition not met within {timeout}s")
