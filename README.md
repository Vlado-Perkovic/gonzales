# gonzales — camera-to-display glass-to-glass latency meter

Measures the latency of any camera→display chain: a stimulus LED sits in a
dark box with the camera; the ESP32 timestamps LED-on, then timestamps the
photoresistor's response when the LED's image appears on the screen. The
difference, minus sensor lag, is the glass-to-glass latency.

## Wiring (ESP32-DevKitC, WROOM-32UE)

```
GPIO26 ──[150 Ω]──▶|── GND      stimulus LED (red 5 mm), ~9 mA
3V3 ──LDR──●──[10 kΩ]── GND     CdS LDR divider
           └── GPIO34          (ADC1_CH6, input-only; NO filter capacitor)
```

- The LDR glues face-down on the screen, over the LED's image in the camera
  feed. Tape the rim so only screen light reaches it.
- Divider node rises when light hits the LDR (dark ≈ 0 ADC counts).
- Same three wires accept a future photodiode (BPW34 reverse-biased into
  47–100 kΩ) or an LM393 module DO (GPIO34 supports interrupts) — no
  firmware redesign, that's the sub-0.1 ms upgrade path.
- **GPIO27 mirrors the stimulus LED** (`set mirror -1` disables). Handy for
  the electrical loopback self-test: jumper GPIO27→GPIO34, then `cal` + a
  run yields µs-class M lines, proving the full timing path sensor-free.

## Build & flash

    . ~/.espressif/v5.5.4/esp-idf/export.sh
    cd firmware && idf.py -p /dev/ttyUSB0 flash

## Using the shell (minicom or any terminal)

    minicom -D /dev/ttyUSB0 -b 115200

    help                     command list
    version                  V,gonzales,0.2.0,...
    mon 100 0                stream raw ADC @10 Hz — use to align the LDR
    cal                      learn dark/bright levels (prints C line)
    oneshot                  one measurement (prints M line)
    run 100                  100 measurements + summary
    stop                     abort run/mon
    set <key> <value>        interval settle timeout jitter calbright margin
                             monperiod med dbg th10 th25 th50 th90 led
    get / reset              show / restore params (persisted in NVS)

### Output lines

| line | meaning |
|---|---|
| `# ...` | info, command echo |
| `M,seq,lat10,lat25,lat50,lat90,t0,dark,span,flags` | measurement; `lat*` = µs from LED-on to ADC crossing dark+{10,25,50,90}%·span; `flags`: `ok`, `d?` (dark not settled), `pXX` (crossing XX missing) |
| `C,dark,bright,span` | calibration (ADC counts) |
| `R,adc` | raw ADC sample (`mon`) |
| `E,seq,code,msg` | error (`to` = timeout/no crossing, `cal`, `abort`, ...) |

During `run`/`mon` only `stop` is honored; other input is discarded.

## Parameters (`set`/`get`, persisted in NVS)

Timing parameters are expressed against the *chain's own latency* L (your
measured median, e.g. ~200 ms for a phone). Get a rough L first with a few
`oneshot`s at a generous `interval`, then size everything from it.

| key | meaning | how to determine |
|---|---|---|
| `interval` | minimum spacing between pulse starts. Actual spacing = max(`interval`, natural cycle) + random jitter. | ≈ 3–5× L. Must exceed capture (~1× L) + off-edge recovery (~1× L) so the floor actually paces the run. |
| `settle` | max wait for the dark level before each pulse. It is a *cap*, not a delay — exits as soon as the sensor reads dark, so oversizing is nearly free. | ≥ the chain's **off-edge** latency (blob disappearing ≈ L). Rule: 2–3× L. Too small → `d?` flags → samples get excluded from stats. |
| `timeout` | max capture window after LED-on. | ≥ p95 latency, i.e. 3–5× L. Too small → `E,to`; too large → slow runs when crossings fail. |
| `on` | fixed LED on-time in ms. `0` = automatic (LED off at the `th90` crossing or `timeout`). With `on > 0` the LED flashes for exactly that long while the capture keeps polling — timestamps stay relative to LED-on, so a short flash measures normally and agitates auto-exposure less. | `0` normally; 50–200 ms for a camera-flash-style stimulus. Must be well below `timeout`. |
| `jitter` | ±random variation added to each pulse's spacing. Destroys phase-lock between pulse timing and camera/display frame clocks (anti-aliasing). | 100–150 ms ≫ any frame period; applied even when the chain's round-trip overruns `interval` (fw ≥ 0.2.3). |
| `calbright` | stabilization wait in `cal` *after* the rise is detected. | 200–500 ms. (fw ≥ 0.2.4: waiting for the blob to arrive through the chain is automatic — `cal` polls until the signal rises, up to `timeout`.) |
| `margin` | dark-settle tolerance in ADC counts. | 40 default. Raise only if `d?` flags persist with verified-dark optics (noisy dark level). |
| `th10 th25 th50 th90` | crossing thresholds as fractions of span (×10000). `lat50` is the primary statistic; `th10` anchors the onset correction. | Defaults 1000/2500/5000/7000 — keep the 10/25/50 geometry intact, the onset-correction constant is derived for 10%/50%. Lower `th90` further only if the source ramps (AE hunting); `lat50` is unaffected. |
| `med` | median-of-3 ADC reads per poll (noise rejection). | keep 1. |
| `monperiod` | `mon` stream period. | setup tool only. |
| `led` / `mirror` | stimulus GPIO / its loopback mirror (-1 disables). | fixed unless rewiring. |
| `dark` / `span` | learned calibration (read-only via `get`). | refreshed by `cal` whenever the physical setup changes. |

Symptom → fix:

| symptom | fix |
|---|---|
| `d?` flags / "excluded N unsettled-dark" | raise `settle` (residual light from previous pulse) |
| `E,to no_crossing` | signal absent/weak, or `timeout` too small |
| `p90` on every sample | source brightness ramps below 90% of settled span → lock camera AE, or lower `th90` |
| spacing stuck at `timeout` + off-edge | captures run to full timeout (`th90` unreachable) → same fix as above |
| raw ≫ onset divergence | something is ramping: slow sensor, or unlocked AE |

## Host tool (statistics)

    cd host && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
    .venv/bin/python gonzales.py run -n 100 --interval 500 \
        --cam-fps 30 --disp-hz 144 -o results/test1

Prints raw (lat50) and onset-corrected statistics (min/p5/p25/med/p75/p95/
max/mean/stddev), writes `samples.csv`, `summary.txt`, `hist.png`,
`timeline.png`. The onset correction `t10 − 0.179·(t50 − t10)` extrapolates
the LDR's exponential rise back to true onset, cancelling most sensor lag;
with a fast photodiode raw and corrected converge (built-in sanity check).

Other subcommands: `version`, `cal`, `oneshot`, `monitor`, `params`,
`set key value`. Smoke test: `.venv/bin/python smoke_test.py` (11 checks,
works without the LDR connected).

## lat50 vs onset — the sensor-lag cancellation math

**lat50 (raw)** is the time from LED-on (`t0`) to the sensor signal crossing
50% of the calibrated span. **onset** is an extrapolated estimate of the time
the light actually *arrived* at the sensor. lat50 always overshoots the true
latency by the sensor's lag; onset removes that lag using only the
measurement itself — no sensor characterization required.

Model: when the display pixel lights up at time `t_on`, a first-order sensor
(CdS cell, RC-coupled photodiode — anything exponential) responds as

    s(t) = dark + span · (1 − e^(−(t − t_on)/τ))

A threshold at fraction f of span is crossed at

    t_f = t_on − τ · ln(1 − f)

so for the 10% and 50% crossings:

    t10 = t_on + 0.10536·τ          (ln(1/0.9))
    t50 = t_on + 0.69315·τ          (ln(1/0.5))

Subtracting eliminates `t_on` and yields τ from the measurement itself:

    τ = (t50 − t10) / 0.58779

Substituting back eliminates τ and recovers the true arrival time:

    t_on = t10 − 0.17919 · (t50 − t10)

The host tool computes both:

    lat50  = t50 − t0                     (biased high by 0.69·τ)
    onset  = t_on − t0                    (unbiased for exponential sensors)

Worked example — the LDR floor test (LDR staring at the LED directly, where
the true latency is ~0): measured `lat10 = 137 µs`, `lat50 = 950 µs`:

    τ     = (950 − 137) / 0.5878 = 1382 µs
    onset = 137 − 0.179 · (950 − 137) = −9 µs ≈ 0   ✓

The 950 µs of apparent latency was *entirely* sensor lag, and the correction
annihilated it without ever being told the LDR's time constant.

Validity conditions — the math assumes exactly one thing: the light step at
the display is **instantaneous** and only the sensor smears it. Therefore:

- **Lock camera exposure/AE.** If the *source* brightness ramps (AE
  re-adjusting to each pulse), the ramp is indistinguishable from sensor lag;
  the correction subtracts it too and onset *underestimates* — it can even
  go negative (observed in the AE-hunting runs). Negative onsets are the
  tell-tale of a ramping source, not a math bug.
- **Sanity check built in:** with the fast photodiode (τ ≈ 30 µs),
  `t50 − t10` is a few tens of µs, so raw ≈ onset. If the two diverge
  significantly, something is ramping — sensor, source, or display
  brightness curve.

## Methodology notes

- **Lock the camera**: manual exposure (≤5 ms), fixed focus, fixed WB. In a
  dark box auto-exposure will ruin the measurement — and it invalidates the
  onset correction (see the math section: a ramping source masquerades as
  sensor lag).
- **Randomized pulse spacing** (default jitter ±100 ms) prevents phase-locking
  to the display/camera refresh — the histogram then shows the true
  distribution, including frame-quantization combs (annotate them with
  `--cam-fps`/`--disp-hz`).
- **Measure on the bright edge only**; the LDR's light→dark recovery is the
  slow edge (settle wait between pulses, ~2 samples/s).
- **Instrument floor test**: point the sensor at the stimulus LED directly
  (no camera/screen), `run 50` → that number is the device's own sensor lag,
  the offset to keep in mind (mostly cancelled by the onset correction).
- **Electrical loopback self-test**: jumper GPIO26→GPIO34 — cal then run
  yields µs-class M lines, proving the full timing path without any sensor.

## Photodiode upgrade (BPW34)

Same three wires, no firmware change — thresholds are calibrated fractions,
sensor-agnostic. Desolder LDR + 10 kΩ, solder in:

```
3V3 ──[BPW34 cathode→anode]──●──[47 kΩ]── GND
                              └── GPIO34    light → node rises (same polarity)
```

- Cathode is the pin at the notched/tabbed corner of the case — cathode
  to 3V3 (reverse-biased as drawn).
- 47 kΩ: ~3 µs response, ADC-friendly source impedance. 100 kΩ if the
  calibrated span is weak (2× signal, ~6 µs). Still NO capacitor on the node.
- BPW34 active area (~7 mm²) is ~3× smaller than the LDR face — center it
  on the blob's bright core during `mon` alignment.

After the swap, re-tune (LDR-era values were compensating for a slow cell)
and recalibrate:

    set settle 100      # PD dark recovery is µs, not ~400 ms
    set calbright 300   # only the chain's own latency remains
    set interval 300    # ~3 samples/s
    set timeout 1500
    cal

Redo the floor test: expect ~10-20 µs raw with raw ≈ corrected — that
convergence is the built-in proof the sensor no longer adds lag. Caveat: a
fast PD can see low-frequency backlight PWM (175-250 Hz panels) as crossing
jitter; a bimodal histogram or a ~4-6 ms comb is the signature.

## Repo layout

    firmware/   ESP-IDF v5.5 project (esp32 target, 4 MB flash)
      main/       cfg.c params+NVS · console.c shell · measure.c engine
    host/       gonzales.py CLI · smoke_test.py · requirements.txt
