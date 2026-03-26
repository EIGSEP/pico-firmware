"""
Tests for the main loop's op() anti-starvation logic.

The C firmware in src/main.c prioritises draining the serial FIFO (via
``continue``) so that slow app_op() calls don't block command receipt.
However, if serial data arrives continuously without a newline terminator,
app_op() would be starved indefinitely.  The MAX_READ_ONLY_US threshold
(50 ms) guarantees that the loop will break out of reading and run op()
within that window.

These tests model the main loop algorithm in Python and verify that:
  1. Normal commands (terminated by newline) always run op() promptly.
  2. Continuous input without a newline eventually triggers an op() call.
  3. Partial commands are preserved across the forced op() call.
"""

import time


# ---------------------------------------------------------------------------
# Minimal Python model of the C main loop scheduling logic
# ---------------------------------------------------------------------------

MAX_READ_ONLY_S = 0.050  # 50 ms, matching MAX_READ_ONLY_US in pico_multi.h


class MainLoopModel:
    """Pure-Python model of the serial-read / op() scheduling in main.c.

    Instead of real serial I/O, callers push characters into ``input_queue``
    and the model records when op() and server() are called.
    """

    def __init__(self, max_read_only_s=MAX_READ_ONLY_S):
        self.max_read_only_s = max_read_only_s
        self.input_queue: list[str] = []
        self.buffer: list[str] = []
        self.op_call_times: list[float] = []
        self.server_calls: list[str] = []  # completed commands
        self._last_op_time = time.monotonic()

    def push_input(self, data: str):
        """Enqueue characters for the model to read."""
        self.input_queue.extend(data)

    def _getchar(self):
        """Non-blocking read of one character (models getchar_timeout_us(0))."""
        if self.input_queue:
            return self.input_queue.pop(0)
        return None

    def _app_op(self):
        """Record an op() call."""
        self.op_call_times.append(time.monotonic())

    def _app_server(self, line: str):
        """Record a dispatched command."""
        self.server_calls.append(line)

    def tick(self):
        """Execute one iteration of the main loop.

        Returns True if the loop ran op(), False if it ``continue``d back
        to reading (i.e. op() was skipped this iteration).
        """
        c = self._getchar()

        if c is not None:
            if c == "\n":
                # Complete command — dispatch, then fall through to op()
                self._app_server("".join(self.buffer))
                self.buffer.clear()
            else:
                self.buffer.append(c)
                # This is the anti-starvation check from main.c lines 105-108
                elapsed = time.monotonic() - self._last_op_time
                if elapsed < self.max_read_only_s:
                    return False  # continue — skip op() this iteration

        # Fall through: run op()
        self._app_op()
        self._last_op_time = time.monotonic()
        return True


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestNormalCommandProcessing:
    """Newline-terminated commands always let op() run promptly."""

    def test_op_runs_after_complete_command(self):
        model = MainLoopModel()
        model.push_input('{"cmd":"ping"}\n')

        # Drain all characters — the newline triggers server + op
        ran_op = False
        for _ in range(50):
            if model.tick():
                ran_op = True
                break

        assert ran_op
        assert model.server_calls == ['{"cmd":"ping"}']
        assert len(model.op_call_times) >= 1

    def test_op_runs_between_commands(self):
        model = MainLoopModel()
        model.push_input('{"cmd":"a"}\n{"cmd":"b"}\n')

        op_count = 0
        for _ in range(100):
            if model.tick():
                op_count += 1
        # op() should have run at least twice (once per command + idle ticks)
        assert op_count >= 2
        assert model.server_calls == ['{"cmd":"a"}', '{"cmd":"b"}']


class TestAntiStarvation:
    """Continuous input without newline triggers op() within the threshold."""

    def test_op_forced_during_unterminated_flood(self):
        """Simulate a flood of characters with no newline.

        Without the MAX_READ_ONLY_US guard, op() would never run.
        With it, op() must run within the threshold.

        Uses a short threshold (5 ms) so the test completes quickly —
        the algorithm is identical regardless of the threshold value.
        """
        threshold = 0.005  # 5 ms
        model = MainLoopModel(max_read_only_s=threshold)
        # Push a large block of data with no newline
        model.push_input("x" * 500_000)

        start = time.monotonic()
        op_ran = False
        for _ in range(500_000):
            if model.tick():
                op_ran = True
                break

        elapsed = time.monotonic() - start
        assert op_ran, "op() was never called despite exceeding threshold"
        # op() should have been forced within roughly the threshold window.
        # Allow generous slack since Python timing is imprecise.
        assert elapsed < threshold * 5, (
            f"op() took {elapsed:.3f}s, expected within ~{threshold:.3f}s"
        )

    def test_buffer_preserved_across_forced_op(self):
        """Partial command data must survive the forced op() call."""
        model = MainLoopModel(max_read_only_s=0.005)
        # Push partial command (no newline), enough to trigger forced op()
        partial = '{"cmd":"hello'
        padding = "x" * 500_000  # ensure we exceed the time threshold
        model.push_input(partial + padding)

        # Run until op() is forced (and beyond)
        for _ in range(500_000):
            model.tick()

        assert len(model.op_call_times) >= 1, "op() should have been forced"

        # Now complete the command
        rest = '"}\n'
        model.push_input(rest)
        for _ in range(100):
            model.tick()

        # The full command should have been dispatched intact
        assert len(model.server_calls) == 1
        expected = partial + padding + rest.rstrip("\n")
        assert model.server_calls[0] == expected

    def test_op_called_multiple_times_during_long_flood(self):
        """During a very long flood, op() should be called repeatedly."""
        model = MainLoopModel(max_read_only_s=0.005)
        model.push_input("x" * 500_000)

        for _ in range(500_000):
            model.tick()

        # With 5ms threshold, we should get multiple op() calls
        assert len(model.op_call_times) >= 2, (
            f"Expected multiple op() calls, got {len(model.op_call_times)}"
        )

    def test_no_starvation_without_guard(self):
        """Demonstrate that removing the guard DOES starve op().

        This is the "control" test: with max_read_only_s=infinity,
        op() never runs during a flood.
        """
        model = MainLoopModel(max_read_only_s=float("inf"))
        model.push_input("x" * 1_000)

        for _ in range(1_000):
            model.tick()

        # With the guard disabled, op() should never have been called
        # (all ticks returned False via continue)
        assert len(model.op_call_times) == 0


class TestIdleBehavior:
    """When no input arrives, op() runs every tick."""

    def test_op_runs_on_empty_input(self):
        model = MainLoopModel()
        for _ in range(10):
            assert model.tick() is True
        assert len(model.op_call_times) == 10
