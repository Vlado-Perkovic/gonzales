# gonzales

Glass-to-glass latency meter for camera-to-display chains. An ESP32 drives a
stimulus LED inside a dark enclosure with the camera under test and
timestamps a light sensor mounted on the display that shows the camera feed.
The interval between LED-on and the sensor response is the latency of the
entire chain. Measurements are reported over UART as CSV lines; a host tool
collects them and computes statistics.

Firmware: ESP-IDF v5.5, target `esp32` (ESP32-DevKitC, WROOM-32UE).
Sensor front-end (CdS LDR divider or BPW34 photodiode) and wiring are
documented in `hardware/`.

## Build and flash

    . ~/.espressif/v5.5.4/esp-idf/export.sh
    cd firmware
    idf.py set-target esp32        # first build only
    idf.py -p /dev/ttyUSB0 flash

If flashing fails with serial corruption, drop the baud: `idf.py -b 115200
-p /dev/ttyUSB0 flash`. Close any terminal program (e.g. minicom) before
flashing; an open port breaks the flasher handshake.

## Device shell

UART0, 115200 8N1, line-based. Identical over any terminal
(`minicom -D /dev/ttyUSB0 -b 115200`) and the host tool.

| command | function |
|---|---|
| `help` | command list |
| `version` | firmware/IDF version |
| `cal` | learn dark/bright levels (chain-aware: waits for the signal to rise, up to `timeout`) |
| `oneshot` | one measurement |
| `run [n] [interval_ms]` | n measurements (default 10), prints one M line each plus a summary |
| `mon [period_ms] [count]` | stream raw ADC readings; `count` 0 = until `stop` |
| `stop` | abort `run`/`mon`; other input during a run is discarded |
| `set <key> <value>` | set a parameter (see below), persisted in NVS |
| `get` | show parameters and calibration |
| `reset` | restore default parameters |

Output lines:

| line | meaning |
|---|---|
| `# ...` | info, command echo |
| `M,seq,lat10,lat25,lat50,lat90,t0,dark,span,flags` | measurement; `lat*` in µs from LED-on to the crossing of `dark + {10,25,50,90}%` of span; `t0` absolute µs |
| `C,dark,bright,span` | calibration, ADC counts |
| `R,adc` | raw ADC sample (`mon`) |
| `E,seq,code,msg` | error (`to` timeout, `cal`, `abort`, `set`, ...) |

`flags`: `ok`; `d?` = dark level not settled before the pulse (residual
light, sample excluded by the host tool); `pXX` = XX% crossing missing
within `timeout`.

## Host tool

Requires [uv](https://docs.astral.sh/uv/). From `host/`:

    uv sync
    uv run gonzales.py run -n 100 --cam-fps 30 --disp-hz 144 -o results/run1

Subcommands: `run`, `oneshot`, `cal`, `monitor`, `params`, `set`, `version`.
`run` prints a statistics table for raw (`lat50`) and onset-corrected
latency (min/p5/p25/med/p75/p95/max/mean/std/MAD) and writes
`samples.csv`, `summary.txt`, `hist.png`, `timeline.png` to the output
directory. `--cam-fps`/`--disp-hz` mark frame-period combs in the
histogram. Ctrl-C stops the device-side run and exits. Exit codes: 0 ok,
1 device error, 2 no usable samples.

Regression check against a live device: `uv run smoke_test.py`
(11 checks; works without a sensor attached).

## Parameters

Timing parameters are sized against the chain's measured median latency L
(get a rough L from a few `oneshot`s at a generous `interval`, e.g. 1000 ms).
Defaults below were tuned on an ~80 ms phone chain.

| key | default | sizing |
|---|---|---|
| `interval` | 220 | min pulse spacing; ≈ 2.5–3× L. Below the chain's natural cycle (~2.3× L) the cycle itself paces the run and higher values change nothing |
| `settle` | 200 | max wait for dark before each pulse (exits early); ≥ the chain's off-edge ≈ L |
| `timeout` | 500 | max capture window; ≥ p95 latency (3–5× L) |
| `on` | 0 | fixed LED on-time in ms; 0 = LED off at the `th90` crossing |
| `jitter` | 50 | ±random spacing added per pulse; anti-aliasing against frame clocks; keep ≥ a few frame periods |
| `calbright` | 300 | stabilization after the cal rise is detected |
| `margin` | 10 | dark-settle tolerance, ADC counts |
| `th10 th25 th50 th90` | 1000/2500/5000/7000 | crossing fractions ×10000; keep the 10/25/50 geometry — the onset correction is derived for it |
| `med` | 1 | median-of-3 ADC reads |
| `led` / `mirror` | 26 / 27 | stimulus GPIO / loopback mirror (-1 disables) |

Symptoms: `d?` flags → raise `settle` or `interval`; `E,to` → weak signal
or `timeout` too small; `p90` on most samples → source brightness ramps
(lock camera AE) or lower `th90`; spacing stuck at `timeout`+off-edge →
`th90` unreachable.

## Methodology

- Lock camera exposure, focus, and white balance. Auto-exposure in a dark
  enclosure modulates the stimulus brightness every pulse, widens the
  distribution, and invalidates the onset correction.
- Run `cal` after any physical change to the setup. Span below 100 counts
  fails with `E,cal`.
- Pulse spacing is randomized (±`jitter`, including when the chain's
  round-trip overruns `interval`), so latency samples cannot phase-lock to
  frame clocks; the histogram shows the true distribution.
- Instrument floor: point the sensor at the LED directly (no camera/screen)
  and `run` — the result is the device's own sensor lag. With a photodiode
  expect ~10–30 µs raw; with a CdS LDR ~1 ms raw, cancelled by the onset
  correction.
- Electrical self-test: jumper `mirror` (GPIO27) to GPIO34, `cal`, `run` —
  µs-class M lines prove the timing path with no sensor.
- Samples far below the chain's physical floor (< 30 ms on typical chains)
  indicate residual light or a setup fault, not latency.

## lat50 vs onset

`lat50` is the time from LED-on to the sensor crossing 50% of the span; it
overstates the true latency by the sensor's response lag. `onset` removes
that lag using the measurement itself. Model the sensor as first-order
exponential after an instantaneous light step at `t_on`:

    s(t)  = dark + span · (1 − e^(−(t−t_on)/τ))
    t_f   = t_on − τ · ln(1 − f)
    τ     = (t50 − t10) / 0.58779
    t_on  = t10 − 0.17919 · (t50 − t10)

    lat50 = t50 − t0                (biased high by 0.69·τ)
    onset = t_on − t0               (unbiased for exponential sensors)

The correction assumes only the sensor smears an instantaneous step. A
ramping source (unlocked AE) is indistinguishable from sensor lag and
drives onset low, possibly negative — negative onsets indicate a ramping
source. With a fast photodiode τ is tens of µs and raw ≈ onset; divergence
between the two means something is ramping.

## Repository layout

    firmware/   ESP-IDF project: cfg.c params/NVS, console.c shell, measure.c engine
    host/       gonzales.py CLI, smoke_test.py, pyproject.toml (uv)
    hardware/   schematic and wiring documentation
