"""Single-line progress bar for long-running device commands.

Renders to stderr only, and only when the stream is a tty, so piped or
captured output stays clean.  Typical frame (redrawn in place with \\r):

    [############------------]  42/100  3 err  eta 1m07s
"""
import sys
import time
from typing import TextIO


def fmt_dur(s: float) -> str:
    """Compact duration: 42s, 3m07s (negative input clamps to 0s)."""
    s = max(0, int(s))
    if s < 60:
        return f"{s}s"
    return f"{s // 60}m{s % 60:02d}s"


class Progress:
    """Accumulating one-line progress bar; mutable by design — update()
    and finish() drive the draw state across the life of one command."""

    MIN_REDRAW_S = 0.1

    def __init__(self, total: int, enabled: bool = True,
                 stream: TextIO | None = None, width: int = 24) -> None:
        self.total = max(1, int(total))
        self.stream = stream if stream is not None else sys.stderr
        self.active = enabled and self.stream.isatty()
        self.width = width
        self.done = 0
        self.errors = 0
        self._t0 = time.monotonic()
        self._last_draw = 0.0
        self._last_len = 0

    def update(self, done: int, errors: int) -> None:
        """Record absolute completed/error counts; redraw if active.

        Redraws are throttled to MIN_REDRAW_S except when the run
        reaches its total, so the 100% frame always shows."""
        self.done, self.errors = done, errors
        if not self.active:
            return
        now = time.monotonic()
        if now - self._last_draw < self.MIN_REDRAW_S and self.done < self.total:
            return
        self._draw(now)

    def finish(self) -> None:
        """Draw the final state once (no eta) and terminate the line."""
        if not self.active:
            return
        self._draw(time.monotonic(), final=True)
        self.stream.write("\n")
        self.stream.flush()
        self.active = False

    def _draw(self, now: float, final: bool = False) -> None:
        frac = max(0.0, min(1.0, self.done / self.total))
        filled = min(self.width, round(frac * self.width))
        parts = [f"[{'#' * filled}{'-' * (self.width - filled)}]",
                 f"{self.done}/{self.total}"]
        if self.errors:
            parts.append(f"{self.errors} err")
        if 0 < self.done < self.total and not final:
            left = (now - self._t0) * (self.total - self.done) / self.done
            parts.append("eta " + fmt_dur(left))
        line = "  ".join(parts)
        self.stream.write("\r" + line + " " * max(0, self._last_len - len(line)))
        self.stream.flush()
        self._last_draw = now
        self._last_len = max(self._last_len, len(line))
