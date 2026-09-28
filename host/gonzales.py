#!/usr/bin/env python3
"""gonzales host tool — drive the gonzales firmware over UART and extract
glass-to-glass latency statistics.

Device protocol (fw >= 0.2.0), line-based on UART0 @ 115200:
  '# ...'                                     info / command echo
  V,name,version,idf_version,build_date       version
  C,dark,bright,span                          calibration (ADC counts)
  M,seq,lat10,lat25,lat50,lat90,t0,dark,span,flags   measurement (lat* in us,
                                              relative to LED-on t0)
  R,adc_raw                                   raw adc sample (mon)
  E,seq,code,msg                              error

Usage:
  gonzales.py [-p PORT] version | cal | oneshot | params
  gonzales.py [-p PORT] run -n 100 [--interval 500] [-o OUTDIR]
              [--cam-fps 30] [--disp-hz 144] [--no-plots]
  gonzales.py [-p PORT] monitor [--period 100] [--count 0]
  gonzales.py [-p PORT] set <key> <value>
"""
import argparse
import csv
import glob
import os
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("pyserial missing — install with: pip install -r requirements.txt")

FW_MIN = (0, 2, 0)

# Onset extrapolation from an exponential rise through the 10% and 50%
# crossings:  t_on = t10 - 0.1792 * (t50 - t10).  Cancels most of the LDR
# sensor lag; with a fast photodiode raw and corrected converge.
K_ONSET = (0.10536 / 0.58779)  # = 0.17919...


class GonzError(Exception):
    pass


def parse_m(line):
    f = line.split(",")
    if len(f) < 10:
        raise GonzError(f"short M line: {line!r}")
    return {
        "seq": int(f[1]), "lat10": int(f[2]), "lat25": int(f[3]),
        "lat50": int(f[4]), "lat90": int(f[5]), "t0": int(f[6]),
        "dark": int(f[7]), "span": int(f[8]), "flags": f[9],
    }


def usable(m):
    # d? = dark level not settled before the pulse (residual light from the
    # previous pulse) -> instant/garbage crossings, exclude from statistics
    return m["lat50"] > 0 and "d?" not in m["flags"]


def onset_us(m):
    return m["lat10"] - K_ONSET * (m["lat50"] - m["lat10"])


def autodetect_port(baud=115200):
    cands = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    for p in cands:
        try:
            g = Gonz(p, baud, _quiet=True)
            v = g.version()
            g.close()
            if v.get("name") == "gonzales":
                return p
        except Exception:
            continue
    raise GonzError("no gonzales device found on " + ", ".join(cands or ["<no ports>"]))


class Gonz:
    def __init__(self, port=None, baud=115200, _quiet=False):
        self.port = port or autodetect_port(baud)
        self.ser = serial.Serial(self.port, baud, timeout=0.3)
        time.sleep(0.15)
        self.ser.reset_input_buffer()
        if not _quiet:
            print(f"# device on {self.port}")

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def readline(self, timeout=0.5):
        old = self.ser.timeout
        self.ser.timeout = max(0.05, min(timeout, 5.0))
        raw = self.ser.readline()
        self.ser.timeout = old
        if not raw:
            return None
        return raw.decode(errors="replace").rstrip("\r\n")

    def cmd(self, cmd_str, until, timeout=10.0):
        """Send a command, collect lines until `until(line)` matches.
        Returns (matched_line, lines)."""
        self.ser.reset_input_buffer()
        self.send(cmd_str)
        deadline = time.monotonic() + timeout
        lines = []
        while time.monotonic() < deadline:
            l = self.readline(0.5)
            if l is None:
                continue
            lines.append(l)
            if until(l):
                return l, lines
        raise GonzError(f"timeout waiting for reply to {cmd_str!r} "
                        f"(last: {lines[-1] if lines else None!r})")

    # -- commands ---------------------------------------------------------

    def version(self):
        l, _ = self.cmd("version", lambda s: s.startswith("V,"), 3)
        f = l.split(",")
        try:
            ver = tuple(int(x) for x in f[2].split("."))
        except ValueError:
            ver = ()
        if len(ver) == 3 and ver < FW_MIN:
            raise GonzError(f"firmware {f[2]} too old, need >= {'.'.join(map(str, FW_MIN))}")
        return {"name": f[1], "version": f[2], "idf": f[3],
                "build": ",".join(f[4:])}

    def cal(self, timeout=10.0):
        def done(l):
            return l.startswith("C,") or l.startswith("E,")
        l, lines = self.cmd("cal", done, timeout)
        if l.startswith("C,"):
            f = l.split(",")
            return {"dark": int(f[1]), "bright": int(f[2]), "span": int(f[3])}
        raise GonzError("cal failed: " + l)

    def oneshot(self, timeout=10.0):
        def done(l):
            return l.startswith("M,") or l.startswith("E,")
        l, _ = self.cmd("oneshot", done, timeout)
        if l.startswith("M,"):
            return parse_m(l)
        raise GonzError("oneshot failed: " + l)

    def run(self, n, interval=None, progress=False):
        if not 1 <= n <= 10000:
            raise GonzError("n must be 1..10000")
        cmd = f"run {n}" + (f" {interval}" if interval else "")
        timeout = n * (interval or 500) / 1000 * 3 + n * 2.5 + 60
        samples, errors = [], []
        def done(l):
            return l.startswith("# run done") or l.startswith("# run stopped")
        _, lines = self.cmd(cmd, done, timeout)
        for l in lines:
            if l.startswith("M,"):
                samples.append(parse_m(l))
            elif l.startswith("E,"):
                f = l.split(",", 3)
                errors.append((f[2], f[3] if len(f) > 3 else ""))
        summary = next(l for l in lines if l.startswith("# run"))
        return samples, errors, summary

    def monitor(self, period=100, count=0, out=sys.stdout):
        cmd = f"mon {period} {count}"
        self.ser.reset_input_buffer()
        self.send(cmd)
        n = 0
        try:
            while True:
                l = self.readline(2.0)
                if l is None:
                    continue
                if l.startswith("R,"):
                    n += 1
                    print(l[2:], file=out)
                elif l.startswith("# mon"):
                    return n
        except KeyboardInterrupt:
            self.send("stop")
            while True:
                l = self.readline(1.0)
                if l is None:
                    break
                if l.startswith("R,"):
                    n += 1
                    print(l[2:], file=out)
                elif l.startswith("# mon"):
                    break
            return n

    def stop_run(self, timeout=10.0):
        """Abort a device-side run: send stop, drain until it confirms."""
        self.send("stop")
        seen_m = 0
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            l = self.readline(0.5)
            if l is None:
                continue
            last = l
            if l.startswith("M,"):
                seen_m += 1
            elif l.startswith("# run"):
                print(l)
                return seen_m, l
        return seen_m, last

    def params(self):
        _, lines = self.cmd("get", lambda s: s.startswith("# dark="), 3)
        return [l[2:] for l in lines if l.startswith("# ")]

    def set(self, key, val):
        l, _ = self.cmd(f"set {key} {val}",
                        lambda s: s.startswith("# set ") or s.startswith("E,"), 3)
        if l.startswith("E,"):
            raise GonzError(l)


# -- statistics -------------------------------------------------------------

import numpy as np


def stats(a_ms):
    if len(a_ms) == 0:
        return None
    p = np.percentile(a_ms, [5, 25, 50, 75, 95])
    med = p[2]
    # percentile-based MAD: np.median drags in numpy.ma, broken on some installs
    mad = float(np.percentile(np.abs(a_ms - med), 50))
    return {
        "n": len(a_ms), "min": a_ms.min(), "p5": p[0], "p25": p[1],
        "med": med, "p75": p[3], "p95": p[4], "max": a_ms.max(),
        "mean": a_ms.mean(), "std": a_ms.std(ddof=1) if len(a_ms) > 1 else 0.0,
        "mad": mad,
    }


STAT_KEYS = ["n", "min", "p5", "p25", "med", "p75", "p95", "max", "mean", "std", "mad"]


def print_table(rows):
    w = 8
    print(f"{'':12s}" + "".join(f"{k:>{w}s}" for k in STAT_KEYS))
    for name, st in rows:
        if st is None:
            print(f"{name:12s}  (no usable samples)")
            continue
        print(f"{name:12s}" + "".join(
            f"{st[k]:{w}.3f}" if k != "n" else f"{int(st[k]):{w}d}"
            for k in STAT_KEYS))


# -- plots -------------------------------------------------------------------

def plots(outdir, samples, cam_fps=None, disp_hz=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("# matplotlib missing — skipping PNGs")
        return
    raw = np.array([m["lat50"] for m in samples]) / 1000.0
    cor = np.array([onset_us(m) for m in samples]) / 1000.0

    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for a, (dat, name) in zip(ax, [(raw, "raw lat50"), (cor, "onset (corrected)")]):
        a.hist(dat, bins=max(10, int(np.ceil(dat.max() - dat.min())) + 1))
        a.set_xlabel("latency [ms]")
        a.set_title(name)
        lo, hi = a.get_xlim()
        if disp_hz:
            for k in range(1, int(hi * disp_hz / 1000) + 2):
                x = k * 1000.0 / disp_hz
                if lo < x < hi:
                    a.axvline(x, color="green", alpha=0.3, lw=0.8)
        if cam_fps:
            for k in range(1, int(hi * cam_fps / 1000) + 2):
                x = k * 1000.0 / cam_fps
                if lo < x < hi:
                    a.axvline(x, color="red", alpha=0.25, lw=0.8)
    fig.suptitle("gonzales glass-to-glass latency")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "hist.png"), dpi=120)

    fig, a = plt.subplots(figsize=(11, 3))
    a.plot(raw, ".", ms=3, label="raw lat50")
    a.plot(cor, ".", ms=3, label="onset (corrected)")
    a.set_xlabel("sample")
    a.set_ylabel("latency [ms]")
    a.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "timeline.png"), dpi=120)
    plt.close("all")


# -- CLI ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="gonzales latency statistics")
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("-b", "--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("version")
    sub.add_parser("cal")
    sub.add_parser("oneshot")
    sub.add_parser("params")

    p_run = sub.add_parser("run")
    p_run.add_argument("-n", type=int, default=100)
    p_run.add_argument("--interval", type=int, default=220,
                   help="pulse spacing ms; default matches the tuned device "
                        "defaults (chain round-trip + margin)")
    p_run.add_argument("-o", "--outdir", default=None)
    p_run.add_argument("--cam-fps", type=float, default=None)
    p_run.add_argument("--disp-hz", type=float, default=None)
    p_run.add_argument("--no-plots", action="store_true")

    p_mon = sub.add_parser("monitor")
    p_mon.add_argument("--period", type=int, default=100)
    p_mon.add_argument("--count", type=int, default=0)

    p_set = sub.add_parser("set")
    p_set.add_argument("key")
    p_set.add_argument("value")

    args = ap.parse_args()

    g = Gonz(args.port, args.baud)
    try:
        if args.cmd == "version":
            v = g.version()
            print(f"V,{v['name']},{v['version']},{v['idf']},{v['build']}")
        elif args.cmd == "cal":
            c = g.cal()
            print(f"C,{c['dark']},{c['bright']},{c['span']}")
        elif args.cmd == "oneshot":
            m = g.oneshot()
            print(f"M seq={m['seq']} lat50={m['lat50']/1000:.3f} ms "
                  f"onset={onset_us(m)/1000:.3f} ms flags={m['flags']}")
        elif args.cmd == "params":
            for l in g.params():
                print(l)
        elif args.cmd == "set":
            g.set(args.key, args.value)
            print("# ok")
        elif args.cmd == "monitor":
            n = g.monitor(args.period, args.count)
            print(f"# monitor done n={n}", file=sys.stderr)
        elif args.cmd == "run":
            try:
                samples, errors, summary = g.run(args.n, args.interval)
            except KeyboardInterrupt:
                print("\n# interrupted - stopping device run")
                seen, last = g.stop_run()
                print(f"# device stopped, {seen} samples measured"
                      + (f" | {last}" if last else ""))
                sys.exit(130)
            print(summary)
            for code, msg in errors[:10]:
                print(f"# err {code}: {msg}")
            if len(errors) > 10:
                print(f"# ... {len(errors) - 10} more errors")
            use = [m for m in samples if usable(m)]
            excluded = len(samples) - len(use)
            if excluded:
                print(f"# excluded {excluded} unsettled-dark (d?) samples")
            if not use:
                print("# no usable samples")
                sys.exit(2)
            raw = np.array([m["lat50"] for m in use]) / 1000.0
            cor = np.array([onset_us(m) for m in use]) / 1000.0

            outdir = args.outdir or os.path.join(
                "results", time.strftime("%Y%m%d_%H%M%S"))
            os.makedirs(outdir, exist_ok=True)
            with open(os.path.join(outdir, "samples.csv"), "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["seq", "lat10_us", "lat25_us", "lat50_us", "lat90_us",
                            "t0_us", "dark", "span", "flags",
                            "lat50_ms", "onset_ms"])
                for m in samples:
                    w.writerow([m["seq"], m["lat10"], m["lat25"], m["lat50"],
                                m["lat90"], m["t0"], m["dark"], m["span"],
                                m["flags"],
                                f"{m['lat50']/1000:.3f}",
                                f"{onset_us(m)/1000:.3f}"])

            print_table([("lat50 ms", stats(raw)), ("onset ms", stats(cor))])
            with open(os.path.join(outdir, "summary.txt"), "w") as fh:
                fh.write(summary + "\n")
                fh.write(f"n_total={len(samples)} usable={len(use)} "
                         f"errors={len(errors)}\n")
                for name, st in [("lat50_ms", stats(raw)), ("onset_ms", stats(cor))]:
                    if st:
                        fh.write(name + ": " +
                                 " ".join(f"{k}={st[k]:.3f}" for k in STAT_KEYS) + "\n")
            if not args.no_plots:
                plots(outdir, use, args.cam_fps, args.disp_hz)
            print(f"# artifacts in {outdir}/")
    finally:
        g.close()


if __name__ == "__main__":
    main()
