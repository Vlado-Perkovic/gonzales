#!/usr/bin/env python3
"""Tests for the run progress bar (no device needed).

    uv run test_progress.py
"""
import io
import re
import sys

from gonzales import Gonz
from progress import Progress, fmt_dur


class TtyBuffer(io.StringIO):
    """stderr stand-in that reports itself as a tty."""

    def isatty(self):
        return True


class FakeSerial:
    """Replays scripted lines to Gonz.readline; records writes."""

    def __init__(self, lines):
        self.lines = list(lines)
        self.written = []
        self.timeout = 0.5

    def write(self, data):
        self.written.append(data)

    def readline(self):
        return self.lines.pop(0).encode() if self.lines else b""

    def reset_input_buffer(self):
        pass


def fake_gonz(lines):
    """A Gonz instance talking to a FakeSerial (no real port)."""
    g = object.__new__(Gonz)  # skip __init__: no device
    g.ser = FakeSerial(lines)
    return g


def with_fake_stderr(fn):
    """Run fn with sys.stderr replaced by a TtyBuffer; return the buffer."""
    buf = TtyBuffer()
    old, sys.stderr = sys.stderr, buf
    try:
        fn()
    finally:
        sys.stderr = old
    return buf


# -- fmt_dur ------------------------------------------------------------------

def test_fmt_dur_formats_seconds_and_minutes():
    assert fmt_dur(0) == "0s"
    assert fmt_dur(59.9) == "59s"
    assert fmt_dur(60) == "1m00s"
    assert fmt_dur(187) == "3m07s"
    assert fmt_dur(-5) == "0s"


# -- Progress -----------------------------------------------------------------

def test_bar_renders_fill_counts_and_eta_on_tty():
    buf = TtyBuffer()
    p = Progress(10, stream=buf)
    p.update(5, 0)
    assert re.fullmatch(
        r"\r\[#{12}-{12}\]  5/10  eta \d+(m\d{2}s|s)", buf.getvalue())


def test_bar_shows_error_count_and_pads_shrinking_line():
    buf = TtyBuffer()
    p = Progress(10, stream=buf)
    p.update(9, 3)   # draws with eta
    p.finish()       # final frame: shorter -> padded with spaces
    last = buf.getvalue().rsplit("\r", 1)[1]
    assert re.fullmatch(r"\[#{22}-{2}\]  9/10  3 err +\n", last)


def test_bar_never_renders_on_non_tty_stream():
    buf = io.StringIO()  # isatty() -> False
    p = Progress(10, enabled=True, stream=buf)
    p.update(5, 0)
    p.finish()
    assert buf.getvalue() == ""


def test_bar_never_renders_when_disabled():
    buf = TtyBuffer()
    p = Progress(10, enabled=False, stream=buf)
    p.update(5, 0)
    p.finish()
    assert buf.getvalue() == ""


def test_bar_throttles_redraw_but_completion_draws_immediately():
    buf = TtyBuffer()
    p = Progress(100, stream=buf)
    p.update(1, 0)
    n_frames = buf.getvalue().count("\r")
    p.update(2, 0)   # within the throttle window -> no redraw
    p.update(3, 0)
    assert buf.getvalue().count("\r") == n_frames
    p.update(100, 0)  # reaching total bypasses the throttle
    assert buf.getvalue().count("\r") == n_frames + 1
    p.finish()
    assert re.search(r"100/100 *\n$", buf.getvalue())


# -- Gonz.run wiring ----------------------------------------------------------

RUN_LINES = [
    "M,1,8000,9000,10000,12000,0,100,900,ok",
    "M,2,8000,9000,10000,12000,0,100,900,d?",
    "E,3,to,timeout",
    "# run done n=3",
]


def test_run_with_progress_reaches_full_bar_and_keeps_contract():
    result = {}

    def run_it():
        g = fake_gonz(RUN_LINES)
        result["r"] = g.run(3, progress=True)

    buf = with_fake_stderr(run_it)
    samples, errors, summary = result["r"]
    assert len(samples) == 2 and samples[0]["seq"] == 1
    assert errors == [("to", "timeout")]
    assert summary == "# run done n=3"
    # errors count as progress (2 M + 1 E = 3/3); trailing * = pad before \n
    assert re.search(r"\[#+\]  3/3  1 err *\n$", buf.getvalue())


def test_run_without_progress_is_silent_on_stderr():
    g = fake_gonz(RUN_LINES)
    buf = with_fake_stderr(lambda: g.run(3))
    assert buf.getvalue() == ""


# -- runner -------------------------------------------------------------------

if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"[PASS] {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"[FAIL] {fn.__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} tests passed")
    sys.exit(0 if not failed else 1)
