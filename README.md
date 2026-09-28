# gonzales

Glass-to-glass latency meter for camera-to-display chains. An ESP32 drives a
stimulus LED inside a dark enclosure with the camera under test and
timestamps a light sensor mounted on the display that shows the camera feed.
The interval between LED-on and the sensor response is the latency of the
entire chain: sensor exposure, camera processing, transmission, host
pipeline, display scanout and pixel response.

Measurements are reported over UART as CSV lines. A host tool collects them
and computes statistics (raw and sensor-lag-corrected).

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

## Usage

### Device shell

UART0, 115200 8N1, line-based. Any terminal works
(`minicom -D /dev/ttyUSB0 -b 115200`); the host tool speaks the same
protocol. Parameters persist in NVS across reboots.

A typical first-time session, end to end:

    # gonzales v0.3.1 ready - type help
    # > mon 100 0              # stream sensor level while positioning
    R,0
    R,3                        # ... slide the sensor over the LED image
    R,1842                     # ... until it peaks, then:
    # > stop
    # mon stopped n=14
    # > cal                    # learn dark/bright levels (takes ~1 s)
    C,0,311,311
    # > oneshot                # single measurement
    M,4,127196,139047,173903,191540,10238138,0,311,ok
    # > run 10 1000            # 10 pulses, 1 s spacing
    M,5,131408,141512,177517,198003,11238151,0,311,ok
    ...
    # run done n=10 ok=10 err=0 lat50_ms min=171.2 med=178.3 max=195.4
    # > get                    # inspect parameters + calibration
    # interval=220 settle=200 timeout=500 on=0 jitter=50 calbright=300 margin=10 monperiod=100
    # med=1 dbg=0 adc=oneshot adcrate=500000 th10=1000 th25=2500 th50=5000 th90=7000 led=26 mirror=27
    # dark=0 span=311

Command reference:

| command | function |
|---|---|
| `help` | command list |
| `version` | firmware/IDF version |
| `cal` | learn dark/bright levels. Chain-aware: waits for the signal to rise through the chain (up to `timeout`), then stabilizes for `calbright` |
| `oneshot` | one measurement |
| `run [n] [interval_ms]` | n measurements (default 10), one M line each plus a summary |
| `mon [period_ms] [count]` | stream raw ADC readings; `count` 0 = until `stop`. Used for sensor alignment |
| `stop` | abort `run`/`mon`; other input during a run is discarded |
| `set <key> <value>` | set a parameter (table below), persisted in NVS |
| `get` | show parameters and calibration |
| `reset` | restore default parameters |

Output lines:

| line | meaning |
|---|---|
| `# ...` | info, command echo |
| `M,seq,lat10,lat25,lat50,lat90,t0,dark,span,flags` | measurement; `lat*` = µs from LED-on to the crossing of `dark + {10,25,50,90}%` of span; `t0` absolute µs |
| `C,dark,bright,span` | calibration, ADC counts |
| `R,adc` | raw ADC sample (`mon`) |
| `E,seq,code,msg` | error (`to` timeout, `cal`, `abort`, `set`, ...) |

`flags`: `ok` (all four crossings recorded); `d?` = dark level not settled
before the pulse (residual light; the host tool excludes these samples);
`pXX` = the XX% crossing was not reached within `timeout`.

### Host tool

Requires [uv](https://docs.astral.sh/uv/). From `host/`:

    uv sync
    uv run gonzales.py run -n 100 -o results/phone1

Typical full session:

    # calibrate (chain-aware, prints C line)
    uv run gonzales.py cal
    C,0,311,311

    # 100-sample measurement; stats table + artifacts
    uv run gonzales.py run -n 100 --interval 1000 --cam-fps 30 --disp-hz 60 \
        -o results/phone1

    # run done n=100 ok=100 err=0 lat50_ms min=152.685 med=185.714 max=235.940
                       n     min      p5     p25      med      p75      p95      max     mean      std      mad
    lat50 ms         100  152.685  167.969  175.059  185.660  201.412  218.605  235.940  188.694   17.240   14.621
    onset ms         100  149.233  163.862  168.151  182.008  197.774  215.047  232.632  184.346   17.527   15.403
    # artifacts in results/phone1/

    # re-analyze a saved run later, e.g. after looking up the frame rates
    uv run gonzales.py analyze results/phone1 --cam-fps 30 --disp-hz 60

Subcommands: `run`, `oneshot`, `cal`, `monitor`, `params`, `set`, `version`,
`analyze`. `run` writes `samples.csv` (all raw crossings — re-analysis is
lossless), `summary.txt`, `hist.png`, `timeline.png`; `--cam-fps`/
`--disp-hz` mark frame-period combs in the histogram and can be applied or
changed post-hoc via `analyze`. Ctrl-C during a run stops the device-side
run and exits. Exit codes: 0 ok, 1 device error, 2 no usable samples.

Alignment workflow before first use: run `uv run gonzales.py monitor` (or
`mon 100 0` in a terminal), display the camera feed fullscreen, and slide
the sensor across the screen until the reading peaks where the LED image
appears; Ctrl-C / `stop` ends the stream.

### Self-tests

    # sensor-free timing check: jumper GPIO27 (mirror) -> GPIO34
    uv run gonzales.py cal           # expect C,0,4095,4095
    uv run gonzales.py run -n 20     # expect µs-class lat50, flags ok

    # sensor floor: point the sensor at the LED directly (no camera/screen)
    uv run gonzales.py run -n 50     # the result is the sensor's own lag

    # full regression against a live device (works without a sensor)
    uv run smoke_test.py

## ADC backends

Two sampling backends, switchable at runtime and persisted (`fw >= 0.3.0`):

| | oneshot (default) | continuous |
|---|---|---|
| mechanism | software busy-poll, one conversion per call | hardware-paced DMA, frames of 128 samples |
| effective rate | ~9 kHz | `adcrate` (10 kHz – 2 MHz, default 500 kHz) |
| crossing resolution | ~116 µs | 1/`adcrate` (2 µs at 500 kHz) |
| timestamps | esp_timer at read start | reconstructed: `t(i) = t_frame − (n−1−i)/adcrate` |
| command | `set adc oneshot` | `set adc continuous` [+ `set adcrate <hz>`] |

Both were cross-validated on a live chain: medians agree to 0.4%. For chain
measurements (ms-scale latencies) they are interchangeable; continuous mode
matters when the phenomenon itself is fast — loopback tests, photodiode
steps, µs-class work. Mode switches never deinit a running DMA driver;
`set adc continuous` while already continuous is a light reconfig.

## Parameters

Timing parameters are sized against the chain's measured median latency L
(get a rough L from a few `oneshot`s at a generous `interval`, e.g. 1000 ms).
Defaults below were tuned on an ~80 ms phone chain.

| key | default | sizing |
|---|---|---|
| `interval` | 220 | min pulse spacing; ≈ 2.5–3× L. Below the chain's natural cycle (~2.3× L) the cycle itself paces the run |
| `settle` | 200 | max wait for dark before each pulse (exits early); ≥ the chain's off-edge ≈ L |
| `timeout` | 500 | max capture window; ≥ p95 latency (3–5× L) |
| `on` | 0 | fixed LED on-time in ms; 0 = LED off at the `th90` crossing |
| `jitter` | 50 | ±random spacing per pulse; anti-aliasing against frame clocks; keep ≥ a few frame periods |
| `calbright` | 300 | stabilization after the cal rise is detected |
| `margin` | 10 | dark-settle tolerance, ADC counts |
| `th10 th25 th50 th90` | 1000/2500/5000/7000 | crossing fractions ×10000; keep the 10/25/50 geometry — the onset correction constant is derived for it |
| `adc` / `adcrate` | oneshot / 500000 | backend selection and its rate |
| `med` | 1 | median-of-3 ADC reads (oneshot mode) |
| `led` / `mirror` | 26 / 27 | stimulus GPIO / loopback mirror (-1 disables) |

Symptoms: `d?` flags → raise `settle` or `interval`; `E,to` → weak signal
or `timeout` too small; `p90` on most samples → source brightness ramps
(lock camera AE) or lower `th90`; spacing stuck at `timeout`+off-edge →
`th90` unreachable.

## Methodology

- Lock camera exposure, focus, and white balance. Auto-exposure in a dark
  enclosure modulates the stimulus brightness every pulse, widens the
  distribution, and invalidates the onset correction (below).
- Run `cal` after any physical change to the setup. Span below 100 counts
  fails with `E,cal`.
- Pulse spacing is randomized (±`jitter`, including when the chain's
  round-trip overruns `interval`), so latency samples cannot phase-lock to
  frame clocks; the histogram shows the true distribution, including
  frame-quantization combs.
- Samples far below the chain's physical floor (< 30 ms on typical chains)
  indicate residual light or a setup fault, not latency.
- Verify the instrument with the loopback and floor tests (above) before
  trusting absolute numbers.

## lat50 and the onset correction

`lat50` is the time from LED-on (`t0`) to the sensor signal crossing 50% of
the calibrated span. It always overstates the true latency by the sensor's
own response lag: the light arrives at the display at some moment `t_on`,
and the sensor then takes time to climb to half-scale. `onset` removes that
lag using nothing but the measurement itself.

Model the sensor as a first-order exponential responding to an
instantaneous light step (true for CdS cells and RC-loaded photodiodes):

    s(t) = dark + span · (1 − e^(−(t−t_on)/τ))

A threshold at fraction f of span is crossed at

    t_f = t_on − τ·ln(1 − f)
    t10 = t_on + 0.10536·τ          (ln(1/0.9))
    t50 = t_on + 0.69315·τ          (ln(1/0.5))

Subtracting eliminates `t_on` and yields the sensor's time constant from
the measurement itself:

    τ = (t50 − t10) / 0.58779

Substituting back eliminates τ and recovers the true arrival time:

    t_on = t10 − 0.17919 · (t50 − t10)

The host tool reports both:

    lat50 = t50 − t0                (biased high by 0.69·τ)
    onset = t_on − t0               (unbiased for exponential sensors)

Worked example — LDR floor test (sensor facing the LED directly, true
latency ≈ 0): measured `lat10 = 137 µs`, `lat50 = 950 µs`:

    τ     = (950 − 137)/0.58779     = 1382 µs
    onset = 137 − 0.179·813         ≈ −9 µs ≈ 0

The entire 950 µs of apparent latency was sensor lag, and the correction
removed it without ever being told the LDR's time constant.

Validity — the derivation assumes exactly one thing: the display switched
on as a step, and only the sensor smears it. Consequences:

- A ramping source (unlocked auto-exposure brightening the blob) is
  mathematically indistinguishable from sensor lag; the correction
  subtracts it too and onset underestimates, often going negative.
  Negative onsets are the signature of a ramping source, not a math bug.
- With a fast photodiode τ is tens of µs, so raw ≈ onset. Their
  convergence is the built-in sanity check that nothing in the chain is
  ramping; significant divergence means sensor or source is slow.

## Repository layout

    firmware/   ESP-IDF project: cfg.c params/NVS, console.c shell, measure.c engine
    host/       gonzales.py CLI, smoke_test.py, pyproject.toml (uv)
    hardware/   schematic and wiring documentation
