#!/usr/bin/env python3
"""Smoke test: exercise the gonzales firmware shell end-to-end over UART.

Works with or without the LDR connected (floating ADC pin exercises all
error paths deterministically). Exit code 0 = all checks passed.
"""
import io
import sys
import time

from gonzales import Gonz, GonzError

PASS, FAIL = "PASS", "FAIL"
results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"[{PASS if ok else FAIL}] {name}" + (f"  ({detail})" if detail else ""))


def main():
    g = Gonz()
    try:
        # 1. handshake
        v = g.version()
        check("handshake", v["name"] == "gonzales", f"{v['version']} / {v['idf']}")

        # 2. help
        _, lines = g.cmd("help",
                         lambda s: "other input is discarded" in s, 3)
        check("help", any("run [n]" in l for l in lines))

        # 3. get
        _, lines = g.cmd("get", lambda s: s.startswith("# dark="), 3)
        joined = "\n".join(lines)
        check("get params", "interval=" in joined and "th90=" in joined)

        # 4. set / get roundtrip + restore
        g.set("jitter", "123")
        _, lines = g.cmd("get", lambda s: s.startswith("# dark="), 3)
        check("set jitter=123", "jitter=123" in "\n".join(lines))
        g.set("jitter", "100")

        # 5. set rejects bad values
        try:
            g.set("timeout", "999999")
            check("set rejects bad value", False, "accepted out-of-range")
        except GonzError as e:
            check("set rejects bad value", "range" in str(e), str(e))

        # 6. cal (floating pin -> expected E,cal; with LDR -> C)
        try:
            c = g.cal()
            check("cal", c["span"] >= 100, f"span={c['span']}")
        except GonzError as e:
            check("cal (no sensor -> clean error)",
                  "cal" in str(e) and ("no_rise" in str(e) or "span" in str(e)),
                  str(e))

        # 7. mon
        buf = io.StringIO()
        n = g.monitor(period=50, count=5, out=buf)
        check("mon 50 5 -> 5 R lines", n == 5, f"n={n}")

        # 8. run
        samples, errors, summary = g.run(2)
        check("run 2 completes", "run done n=2" in summary, summary)
        check("run produces M or E per pulse",
              len(samples) + len(errors) >= 2,
              f"M={len(samples)} E={len(errors)}")

        # 9. stop when idle
        l, _ = g.cmd("stop",
                     lambda s: s.startswith("#") and not s.startswith("# >"), 3)
        check("stop idle", "nothing running" in l, l)

        # 10. unknown command
        l, _ = g.cmd("definitelynotacommand",
                     lambda s: s.startswith("# unknown"), 3)
        check("unknown cmd", True, l)

    finally:
        g.close()

    n_ok = sum(results)
    print(f"\n{n_ok}/{len(results)} checks passed")
    sys.exit(0 if n_ok == len(results) else 1)


if __name__ == "__main__":
    main()
