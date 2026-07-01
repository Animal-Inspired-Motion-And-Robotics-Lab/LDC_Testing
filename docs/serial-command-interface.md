# Serial Command Interface

> **Purpose of this document.** A complete, ground-truth reference for the runtime serial
> command-line interface (CLI) exposed by this firmware, written so it can be used as
> source material for a paper's "operation" / "configuration" section or as an operator's
> manual. Everything here is taken directly from the source (`src/serial_commands.cpp`,
> `include/serial_commands.h`, `src/main.cpp`, `src/calibration.cpp`,
> `src/measurement_arrays.cpp`, `src/crack_detection.cpp`). Companion algorithm details
> live in [`crack-detection-algorithm.md`](crack-detection-algorithm.md).

---

## 1. Overview

The firmware exposes a **line-oriented text CLI over the USB serial link** — the same
link it streams measurements on. The CLI lets an operator reconfigure every runtime
tunable (sensor specs, measurement mode, filter window, calibration, crack-detector
tuning) and manage persistent per-material profiles **without rebuilding or reflashing**.

The CLI and the measurement stream share one connection: you type commands into the same
serial port that is emitting Teleplot data. Command responses (`OK …`, `ERR …`, status
dumps) are interleaved with the data stream. Use `stream off` to quiet the data while
configuring if the interleaving is distracting.

- **Transport:** USB-CDC serial on the Seeed XIAO ESP32-S3. The monitor speed is
  115200 baud (`platformio.ini`); note USB-CDC ignores the firmware-side
  `Serial.begin()` baud, so the link works regardless.
- **Polling:** `serialCommandsPoll()` runs once per main-loop tick and is non-blocking —
  commands are processed promptly but never stall the measurement loop.
- **When changes take effect:** the loop drains queued CLI input at the *start* of each
  tick, so a command applies on the very next measurement/emit cycle.

---

## 2. Line format and parsing rules

```
<command> [arg1] [arg2 ...]<newline>
```

| Rule | Behavior |
|------|----------|
| **Terminator** | A command is dispatched when a newline (`\n`) is received. Carriage returns (`\r`) are ignored, so both LF and CRLF line endings work. |
| **Separators** | The command name and arguments are split on spaces or tabs. |
| **Leading whitespace** | Stripped. Blank / whitespace-only lines are silently ignored (no-op). |
| **Case** | Command names and keyword arguments (`on`/`off`, `lrp`/`lhr`, etc.) are **case-sensitive** — all are lowercase. |
| **Line length** | Up to **95 characters** per line. Bytes beyond that are dropped; the truncated line is still dispatched on the next `\n`. |
| **Extra args** | Several commands explicitly reject trailing extra arguments with an `ERR usage:` message; others ignore them. When in doubt, pass exactly the documented arguments. |

### Argument conventions

The CLI follows a consistent **"set vs. show"** convention:

- **Numeric value commands** (e.g. `l_h`, `angle`, `crack_threshold`): with **no
  argument** they *show* the current value; with **one argument** they *set* then echo
  the post-clamp value. Example: `crack_r2` prints the current R², `crack_r2 0.7` sets it.
- **Boolean toggles** (e.g. `stream`, `rotated`, `crack_output`): always require an
  explicit `on` or `off` argument. There is no bare "show" form; use `status`.
- **Action commands** (e.g. `calibrate`, `save`): take their required argument(s) and
  perform an operation.

### Response format

| Response | Meaning |
|----------|---------|
| `<name>=<value>` | Echo of a numeric value (after any clamping). Emitted by set/show commands. |
| `OK <name> <value>` / `OK <name> on\|off` | Confirmation of a toggle or action. |
| `ERR <message>` | Bad input: malformed number, out-of-range value, wrong usage, or unknown command. The command made no change. |
| Multi-line dumps | `help`, `status`, `materials`, `calibrate`, `baseline`, `get_material` print several lines. |

**Clamping:** many setters silently clamp out-of-range-but-parseable values to their
valid range and echo the clamped result, rather than erroring. Always read the echoed
value to confirm what was actually applied. (Hard rejections — negative where ≥0 is
required, non-numeric, etc. — produce `ERR` and change nothing.)

### Units

Sensor values are entered and displayed in **convenient units** but stored internally in
SI:

| Command | Entered / shown as | Stored as |
|---------|--------------------|-----------|
| `l_h` | microhenries (µH) | henries (H) |
| `c_f` | picofarads (pF) | farads (F) |
| `angle`, `crack_deviation` | radians | radians |
| `delay` | milliseconds | milliseconds |

---

## 3. Discoverability: `help` and `status`

Two commands make the interface self-documenting, and they are kept **mirror-image**:
every tunable shown by `status` as `<name>=<value>` has a matching `<name>` command in
`help`, in the same order and with the same name.

- **`help`** — prints the full command list with one-line descriptions. Also printed once
  automatically at boot.
- **`status`** — prints every current value in `name=value` form, grouped:
  sensor params → reading mode → signal processing → crack detection.

When extending the firmware, the convention is to add the new tunable to *both* `help`
and `status` so it stays discoverable.

---

## 4. Command reference

Grouped by function. "Args" shows the expected argument(s); `[x]` is optional, `<x>` is
required. Defaults are the **boot defaults** from `main.cpp::setup()`.

### 4.1 Meta

| Command | Args | Description |
|---------|------|-------------|
| `help` | — | Print the full command list. Also shown at boot. |
| `status` | — | Print all current config/state values as `name=value`. |

### 4.2 Sensor parameters (register-affecting)

These feed the LDC1101's `RP_SET` / `TC1` / `TC2` / `DIG_CONF` derivation, so **changing
any of them immediately re-pushes the register set to the chip** (`configureSensor()`).

| Command | Args | Default | Description |
|---------|------|---------|-------------|
| `l_h` (alias `lh`) | `[µH]` | 11.8 µH | Sensor (tank coil) inductance. Must be > 0. Show or set. |
| `c_f` (alias `cf`) | `[pF]` | 220 pF | Sensor tank capacitance. Must be > 0. Show or set. |
| `q` | `[ratio]` | 25.0 | LC tank quality factor. Must be > 0. Show or set. |

### 4.3 Measurement mode and timing

| Command | Args | Default | Description |
|---------|------|---------|-------------|
| `mode` | `lrp\|lhr` | `lrp` | Measurement mode. `lrp` = combined Rp + L conversion; `lhr` = high-resolution L only (Rp reported as NaN). Re-pushes registers. |
| `speed` | `accuracy\|balanced1\|balanced2\|fast` | `balanced1` | Accuracy↔throughput tradeoff (sets the chip's conversion-time / response-time bits). Aliases: `bal1`, `bal2`. Re-pushes registers. |
| `stream` | `on\|off` | `on` | Master switch for the Teleplot measurement stream. `off` halts emission (CLI stays responsive). Does **not** touch the chip. |
| `delay` | `<ms>` | 25 ms | Sample/emit period in milliseconds. Range 1…60000. Sets the loop pacing only. |

### 4.4 Signal processing and calibration

These operate on the smoothing filter and the calibration rotation owned by
`measurement_arrays` (see the algorithm doc for the math).

| Command | Args | Default | Description |
|---------|------|---------|-------------|
| `smoothing` | `<n>` | 25 | Moving-average filter window length (samples), applied before rotation/storage. ≥ 1; `1` disables smoothing. Clamped to ≤ 512. |
| `rotated` (aliases `rotate`, `rotation`) | `on\|off` | `off` | Enable/disable applying the calibrated rotation to incoming samples. Mirrors the internal rotation-enabled flag. |
| `angle` | `[radians]` | 0 | Show or directly set the rotation angle, **leaving the rotation center untouched**. For manual override; normally set by `calibrate`. |
| `calibrate` | `[samples]` | 40 | **Per-substrate calibration.** Estimates the substrate's baseline trend angle from the most recent *N* smoothed samples, installs that rotation (angle + center), and enables rotation. LED flashes during the run; prints the dominant angle (rad + deg) and sample counts. Run while sweeping a crack-free region of the new substrate. *N* range 1…50000. |
| `baseline` | `[samples]` | 40 | **Re-anchor only the rotation center** to the recent mean (Rp, L), leaving the angle and enable flag alone. Use to re-zero the crack-height reference after thermal/baseline drift without redoing the full trend fit. Prints the new center. *N* range 1…50000. |

> The `help` text labels `calibrate` as "PCA calibration." The implementation actually
> uses a least-squares trend-line slope, which is first-order equivalent to PCA on an
> elongated cloud — see the algorithm doc's accuracy note.

### 4.5 Crack-detector tuning

All write through to the crack detector and re-clamp on the way in; the echo prints the
post-clamp value. Full meaning of each knob is in the algorithm doc.

| Command | Args | Default | Range | Description |
|---------|------|---------|-------|-------------|
| `crack_window` | `[n]` | 110 | ≥ 1 | Rolling window length (samples) used for both the parabola fit and the direction check. Raise as sweep speed drops. |
| `crack_threshold` | `[val]` | 0.01 | ≥ 0 | Minimum fitted parabola peak height above the rotated baseline for a candidate to qualify (amplitude gate). |
| `crack_r2` | `[0..1]` | 0.5 | 0…1 | Minimum R² of the parabola fit (shape-quality gate). |
| `crack_deviation` | `[rad]` | 1.0 (≈57°) | 0…π/2 | Max angular tilt of the 3-D (t, Rp, L) curve off the t-L plane. **Lower → stricter → fewer false positives;** `π/2` disables the check. |
| `crack_scale` | `[scale]` | 220 | ≥ 0 | Scale factor turning fitted peak height into the reported `crack_size` length estimate. |
| `crack_output` | `on\|off` | `on` | — | Append the per-crack Teleplot fields (`>mag >crack_x >crack_size >width`) to the stream on a confirmed detection. |
| `crack_debug` | `on\|off` | `off` | — | Emit the per-tick detector debug line (`detected`, `reject_reason`, `fit_peak`, `fit_width`, `fit_r2`) and a `>reason:` line on rejections. Not Teleplot-parseable; for tuning sessions. |

### 4.6 Persistent material profiles (NVS flash)

A *material profile* is a complete snapshot of all user-configurable settings — sensor
params, mode/speed, smoothing, rotation (angle + center), and the full crack-detector
config — persisted to the ESP32-S3's NVS flash so it survives a power cycle, with several
materials stored side by side.

| Command | Args | Description |
|---------|------|-------------|
| `save` | `<material>` | Snapshot all current settings into a profile named `<material>`. Name ≤ 15 chars, may not start with `_` (reserved for the internal index). Prints `OK saved <name>` or an error. |
| `retrieve` (alias `load`) | `<material>` | Load the named profile, re-push sensor/mode/speed to the chip, apply the stored rotation and crack settings, then print `status`. Errors if the name isn't found. |
| `materials` | — | List all saved material names and their stored rotation angle. |
| `get_material` | — | **Identify** the saved profile whose stored rotation angle is closest to the *current* (just-calibrated) angle, using a π-periodic orientation distance. Report-only — prints the closest name and the angle difference but loads nothing. Intended workflow: `calibrate` an unknown substrate, then `get_material`. |
| `forget` | `<material>` | Delete the named saved profile. Errors if not found. |

---

## 5. Typical operator workflows

**First-time setup on a new substrate**

```
stream on              # confirm data is flowing (Rp/L X-Y plot in Teleplot)
l_h 42.6               # set the installed coil's modeled inductance (µH)
q 24.6                 # set its modeled Q
# sweep the probe over a clean, crack-free region, then:
calibrate              # learn the baseline orientation, enable rotation
rotated on             # (calibrate enables it; explicit for clarity)
save aluminum_6061     # persist this material's full tuning
```

**Returning to a known material**

```
retrieve aluminum_6061 # restores sensor + rotation + crack tuning, prints status
```

**Identifying an unknown substrate**

```
# sweep a clean region, then:
calibrate
get_material           # prints the closest stored profile by angle
retrieve <that name>   # operator decides whether to load it
```

**Reducing false detections** (see also the algorithm doc)

```
crack_debug on         # watch reject reasons + fit values live
crack_deviation 0.6    # tighten the direction check (was 1.0)
crack_r2 0.7           # demand a cleaner parabola
crack_threshold 0.02   # demand a taller bump
crack_debug off
save <material>        # persist the tuning once satisfied
```

**Re-zeroing after drift** (no need to re-find the substrate trend)

```
baseline               # re-anchor the rotation center to the current flat signal
```

---

## 6. Error and edge-case behavior

- **Unknown command** → `ERR unknown command: <token>`.
- **Malformed number** (non-numeric, trailing garbage) → `ERR <name> must be …`; no change.
- **Out of hard range** (e.g. `delay 0`, `crack_r2 1.5`) → `ERR`; no change.
- **Out of soft range** (parseable but beyond a clamp, e.g. `smoothing 9999`,
  `crack_deviation 3.0`) → silently clamped; the echoed value reflects the clamp.
- **Missing required toggle arg** (e.g. bare `stream`) → `ERR usage: …`.
- **Calibration with too few / degenerate samples** → calibration reports a result but
  makes no rotation change if the trend angle is undefined (zero Rp variance).
- **Overlong line** (> 95 chars) → excess bytes dropped; the truncated remainder is
  dispatched at the next newline and will likely produce an unknown-command error.
- **Profile name issues** (`save _foo`, name > 15 chars, storage error) →
  `ERR save failed (bad name or storage error)`.

---

## 7. Quick reference (all commands)

| Command | Args | Group | One-line description |
|---------|------|-------|----------------------|
| `help` | — | meta | Show command list. |
| `status` | — | meta | Show all current values. |
| `l_h` / `lh` | `[µH]` | sensor | Sensor inductance (re-pushes registers). |
| `c_f` / `cf` | `[pF]` | sensor | Sensor capacitance (re-pushes registers). |
| `q` | `[ratio]` | sensor | Tank quality factor (re-pushes registers). |
| `mode` | `lrp\|lhr` | mode | Rp+L vs high-resolution-L mode. |
| `speed` | `accuracy\|balanced1\|balanced2\|fast` | mode | Accuracy↔throughput tradeoff. |
| `stream` | `on\|off` | mode | Master measurement-stream switch. |
| `delay` | `<ms>` | mode | Sample/emit period (1…60000 ms). |
| `smoothing` | `<n>` | signal | Moving-average window (1 = off, ≤ 512). |
| `rotated` / `rotate` / `rotation` | `on\|off` | signal | Apply calibrated rotation. |
| `angle` | `[rad]` | signal | Show/set rotation angle (center untouched). |
| `calibrate` | `[samples]` | signal | Learn substrate trend, install rotation, enable. |
| `baseline` | `[samples]` | signal | Re-anchor rotation center only (drift re-zero). |
| `crack_window` | `[n]` | crack | Fit/direction window length. |
| `crack_threshold` | `[val]` | crack | Min peak height (amplitude gate). |
| `crack_r2` | `[0..1]` | crack | Min parabola R² (shape gate). |
| `crack_deviation` | `[rad]` | crack | Max curve tilt off t-L plane (0…π/2; lower = stricter). |
| `crack_scale` | `[scale]` | crack | Peak-height → length scale factor. |
| `crack_output` | `on\|off` | crack | Append per-crack fields to the stream. |
| `crack_debug` | `on\|off` | crack | Emit detector debug / reject-reason lines. |
| `save` | `<material>` | profile | Persist all settings under a name. |
| `retrieve` / `load` | `<material>` | profile | Load a saved profile, re-push to chip. |
| `materials` | — | profile | List saved materials + angles. |
| `get_material` | — | profile | Name closest saved profile to current angle (report-only). |
| `forget` | `<material>` | profile | Delete a saved profile. |

---

## 8. Notes for paper authors

- **Single shared link.** Emphasize that configuration and data egress share one USB
  serial connection; there is no separate control channel. This keeps the payload's wire
  count minimal — relevant for an insect-scale robot.
- **No reflash for retuning.** All sensor, filter, calibration, and detector parameters
  are live-tunable, which is what makes field calibration per-substrate practical.
- **Persistence is per-material.** The NVS profile mechanism is what lets the operator
  pre-characterize several substrates in the lab and recall them in the field by name (or
  by angle-based identification).
- **`help`/`status` mirror.** The interface is self-describing; a reader reproducing the
  setup can enumerate every tunable from `status` alone.
- **Wording caveat.** `calibrate`'s help string says "PCA"; the implementation is a
  least-squares trend-line fit (first-order equivalent). Keep the paper's wording
  consistent with whichever description you adopt in the methods section.
