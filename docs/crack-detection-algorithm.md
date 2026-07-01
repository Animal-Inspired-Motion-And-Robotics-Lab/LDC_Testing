# Eddy-Current Crack Detection: Calibration and Detection Algorithm

> **Purpose of this document.** This is a self-contained, ground-truth description of
> the signal-processing algorithm implemented in this firmware, written so it can be
> used as source material for the methods/algorithm section of a paper. Every formula,
> default value, and design decision here is taken directly from the source
> (`src/calibration.cpp`, `src/measurement_arrays.cpp`, `src/crack_detection.cpp`,
> `src/telemetry.cpp`, `src/main.cpp`). Where the code's behavior differs from a
> textbook description (e.g. it uses a least-squares slope where one might expect full
> PCA), that is called out explicitly so the paper does not overclaim.

---

## 1. Physical setup and what is being measured

The sensor is a **TI LDC1101 inductance-to-digital converter** driving an **LC tank
that acts as an eddy-current probe**. The probe is mounted on an **insect-scale mobile
robot** that sweeps it across a conductive substrate. When the probe passes over a
**crack** in the substrate, the discontinuity perturbs the eddy currents induced in the
material, which changes the effective inductance and loss seen by the tank.

The LDC1101 reports two scalars per sample:

| Symbol | Quantity | Physical meaning |
|--------|----------|------------------|
| `Rp`   | Parallel resistance (Ω) | Loss term of the tank; tracks eddy-current energy dissipation in the substrate. |
| `L`    | Inductance (µH)         | Reactive term; tracks coupling between the probe coil and the substrate. |

As the probe sweeps in time `t`, each measurement is a point in a 2-D
**(`Rp`, `L`) plane**, and the time-ordered stream traces a curve. The detection
problem is: **given the recent history of `(Rp, L)` samples, decide whether a crack
just passed under the probe**, while rejecting confounders (baseline drift, lift-off,
boundaries between material patches, and sensor noise).

**Key physical premises the algorithm relies on:**

1. **A crack shows up primarily as an `L` excursion (a bump), not an `Rp` excursion.**
   Over a calibrated, homogeneous substrate the loss term `Rp` stays near a constant
   baseline while a crack produces a transient rise-and-fall in `L`.
2. **The "no-crack" baseline trend in the `(Rp, L)` plane is a straight line whose
   orientation is set by the substrate's conductivity.** This orientation is the
   substrate's *phase response*. It differs from material to material, which is why
   calibration is **per-substrate** and must be repeated whenever the robot moves to a
   new material.
3. **Confounders (material-boundary transitions, baseline slides, lift-off) move the
   operating point along, or tilt it away from, that baseline line** — i.e. they drag
   `Rp` as well as `L`. A true crack does not.

The algorithm is built to exploit premises 1–3: it learns the baseline orientation
(calibration), rotates the data so a crack becomes a pure `L` bump, then tests each
window for (a) the right *shape* and (b) the right *direction* of change.

---

## 2. Pipeline overview

Each control tick (`loop()` in `src/main.cpp`) executes, in order:

```
raw (Rp, L)  ──►  smoothing  ──►  rotation  ──►  history buffer  ──►  crack detector  ──►  serial out
  (LDC1101)      (moving avg)   (calibrated)    (rolling 1000)      (4-stage check)     (Teleplot)
```

1. **Read** raw `(Rp, L)` from the LDC1101.
2. **Smooth** with a moving-average filter (`appendMeasurement` → `filterSample`).
3. **Rotate** the smoothed sample about a calibrated center into the "crack frame"
   (only if calibration has been run).
4. **Store** both the smoothed and rotated samples in fixed-length rolling histories.
5. **Detect**: run the multi-stage crack check over the most recent window of the
   *rotated* stream.
6. **Emit** the latest sample (and, on a confirmed detection, a crack-magnitude
   description) over USB serial in Teleplot format.

The two interesting algorithmic blocks are **calibration** (§4) and **detection** (§5).

---

## 3. Preprocessing: moving-average smoothing

Source: `src/measurement_arrays.cpp`, `filterSample()`.

Each raw `(Rp, L)` sample is pushed into a ring buffer, and the output is the unweighted
mean of the most recent `W_f` samples:

$$
\bar{Rp}_k = \frac{1}{n}\sum_{j=0}^{n-1} Rp_{k-j}, \qquad
\bar{L}_k = \frac{1}{n}\sum_{j=0}^{n-1} L_{k-j},
\qquad n = \min(W_f,\ \text{samples seen so far})
$$

- `W_f` is the **filter window** (`filterWindow`, serial command `smoothing`),
  default **25 samples**, clamped to `[1, 512]`. `W_f = 1` disables smoothing.
- During warm-up (before `W_f` samples have arrived) the average is taken over however
  many samples exist, so the filter produces a useful output immediately rather than
  waiting to fill.

This is a simple, zero-phase-at-DC low-pass with a flat (rectangular) kernel. Its only
job is to suppress per-sample sensor noise before the geometric processing.

All downstream calibration and detection operate on the **smoothed** stream, never the
raw samples.

---

## 4. Calibration (per-substrate)

Source: `src/calibration.cpp` (`calibrationRun`), `src/measurement_arrays.cpp`
(`calculateDominantAngleRecent`, `getRecentMeasurementMean`, rotation functions).
Triggered by the `calibrate [N]` serial command.

### 4.1 Goal

Find a 2-D rigid transform (a rotation about a center) that maps the **substrate's
no-crack baseline trend onto the horizontal axis**, so that in the transformed frame:

- motion *along* the baseline (normal sweeping, slow drift) changes the new **x**
  coordinate (the rotated `Rp`), and
- a crack's `L` bump appears as a change in the new **y** coordinate (the rotated `L`).

This makes the crack signature axis-aligned, which is what the detector's shape and
direction tests assume.

### 4.2 Estimating the baseline orientation θ

The robot is held sweeping over a **clean (crack-free) region** of the target substrate
while samples accumulate. Calibration then operates on the most recent `N` smoothed
samples (`N` = the `calibrate` argument, default **40**).

The dominant trend angle `θ` is estimated as the **least-squares slope of `L` vs `Rp`**,
i.e. treating `Rp` as the independent variable:

$$
\text{slope} = \frac{\operatorname{cov}(Rp, L)}{\operatorname{var}(Rp)}
= \frac{\sum_i (Rp_i - \overline{Rp})(L_i - \overline{L})}{\sum_i (Rp_i - \overline{Rp})^2},
\qquad
\theta = \arctan(\text{slope}).
$$

If `var(Rp) = 0` (degenerate, vertical, or single-valued cloud) the routine returns
`NaN` and calibration makes no change.

> **Accuracy note for the paper.** The in-code comment and the project notes sometimes
> describe this as "PCA on the recent cloud." The implementation does **not** compute a
> full principal-component decomposition; it computes the ordinary least-squares slope
> of `L` on `Rp`. The two agree to first order when the cloud is strongly elongated and
> `var(Rp) > 0`, but they are not identical (OLS minimizes vertical residuals; PCA
> minimizes orthogonal residuals). If the paper claims PCA, it should either switch the
> wording to "least-squares trend line" or note the approximation. The angle is the
> `arctan` of a slope, so it is naturally **π-periodic** (θ and θ ± π describe the same
> line) — this matters for material matching (§6).

### 4.3 Setting the rotation

With θ in hand, calibration installs three things (`measurement_arrays`):

1. **Rotation center** = the centroid of the same `N` samples:
   $(\overline{Rp}, \overline{L})$. Rotating about the centroid (rather than the
   origin) keeps the operating point fixed and only re-orients the cloud.
2. **Rotation angle** = $-\theta$. The negative sign rotates the trend *down onto* the
   new x-axis.
3. **Rotation enabled** = true.

### 4.4 The rotation applied per sample

Source: `appendMeasurement()` / `rotateSample()`. For each new smoothed sample
$(\bar{Rp}, \bar{L})$, the stored *rotated* sample is a standard 2-D rotation by
$-\theta$ about the center $(c_{Rp}, c_L)$:

$$
\begin{bmatrix} Rp' \\ L' \end{bmatrix}
=
\begin{bmatrix} \cos(-\theta) & -\sin(-\theta) \\ \sin(-\theta) & \cos(-\theta) \end{bmatrix}
\begin{bmatrix} \bar{Rp} - c_{Rp} \\ \bar{L} - c_L \end{bmatrix}
+
\begin{bmatrix} c_{Rp} \\ c_L \end{bmatrix}.
$$

After this transform, an ideal no-crack trace runs horizontally through the center, and
a crack appears as a positive excursion in `L'` (the rotated inductance). All detection
math below operates on `(Rp', L')`.

### 4.5 Baseline re-zero (without re-finding orientation)

There is a second, lighter command, `baseline [N]` (`baselineRun`), that recomputes
**only the rotation center** from the recent mean and leaves the angle and enable flag
untouched. It is used to re-zero the detector's height reference after slow thermal /
baseline drift, without re-estimating the substrate orientation. After it runs, the
rotated baseline `L'` again sits at ~0 relative to the new center, so the parabola
peak-height test (§5.2) measures bumps against the corrected baseline.

---

## 5. Crack detection

Source: `src/crack_detection.cpp` (`crackDetectionCheck` and helpers), config struct in
`include/crack_detection.h`. Runs every tick over the most recent
`window_samples` (`W`) entries of the **rotated** stream `(Rp', L')`, with sample index
`x = 0 … W−1` running oldest → newest.

The check is a cascade of four stages; the **first failing stage rejects the window** and
the tick emits no detection. A window must clear all four to fire.

### 5.0 Stage 0 — Parabola fit feasibility

Source: `fitParabolaForWindow()`.

A quadratic is fit by ordinary least squares to the rotated inductance as a function of
sample index:

$$
y(x) = a x^2 + b x + c, \qquad y \equiv L'(x) - c_L
$$

where `y` is measured **relative to the calibrated rotation center** `c_L`, so a flat
no-crack baseline sits near `y = 0`. The fit solves the 3×3 normal equations
$M[a\;b\;c]^\top = v$ with

$$
M = \begin{bmatrix} \sum x^4 & \sum x^3 & \sum x^2 \\ \sum x^3 & \sum x^2 & \sum x \\ \sum x^2 & \sum x & n \end{bmatrix},
\qquad
v = \begin{bmatrix} \sum x^2 y \\ \sum x y \\ \sum y \end{bmatrix}
$$

via Gaussian elimination with partial-pivot guards. The fit is rejected outright
(treated as "not even a candidate," no reason logged) if any of the following hold:

- fewer than 3 samples, or the matrix is singular (any vanishing pivot);
- the parabola opens **upward** (`a ≥ 0`) — a crack is a *peak*, i.e. a downward-opening
  parabola, so `a` must be negative;
- the **vertex** $x_v = -b/(2a)$ falls **outside** the window `[0, W−1]` (an
  extrapolated peak is not credible);
- the fitted **peak height** or **width** is non-positive or non-finite.

From a valid fit the detector derives the analytic descriptors:

| Quantity | Formula | Meaning |
|----------|---------|---------|
| Peak height | $h = c - \dfrac{b^2}{4a}$ | Vertex value above the rotated baseline. |
| Peak position | $x_v = -\dfrac{b}{2a}$ | Vertex location (samples from oldest). |
| Half height | $h/2$ | Kept for the debug stream. |
| Width | $w = 2\sqrt{-h/(2a)}$ | Full width between the parabola's zero crossings (relative to baseline). |
| Fit quality | $R^2 = 1 - \dfrac{\text{SSE}}{\text{SST}}$ | Standard coefficient of determination. |

### 5.1 Stage 1 — Shape check (does it look like a crack?)

Two thresholds, checked in order (first failure wins):

1. **Fit quality:** reject if $R^2 < $ `min_parabola_r2` (reason `low_r2`). Ensures the
   window is actually well-described by a single bump, not noise or a step.
2. **Amplitude:** reject if $h < $ `threshold` (reason `threshold`). Ensures the bump is
   tall enough above the substrate baseline to be a real crack rather than a ripple.

A window passing both is a **crack candidate** in the shape sense.

### 5.2 Stage 2 — 3-D direction check (is it a crack or a substrate transition?)

Source: `getDeviationAngleForWindow()`. This is the key discriminator that separates a
true crack from a confounding **substrate transition** (material-patch boundary, baseline
slide, lift-off), and it is the stage the `crack_deviation` knob tunes.

**The geometric idea.** Lift the window into 3-D space with coordinates $(t, Rp', L')$
where `t` is sample index. A *true crack* has `L'` tracing a parabola in time while `Rp'`
stays pinned at the calibrated baseline — so the 3-D curve lies in the **`t`–`L` plane**
(the plane `Rp' = const`). A *substrate transition* drags `Rp'` along too, tilting the
curve out of the `t`–`L` plane toward the `L`–`Rp` plane.

**The analytic characterization.** Model the rotated loss term as drifting linearly in
time, $Rp'(t) = m\,t + b$, on top of the parabolic `L'`. Then the 3-D curve lies exactly
in the plane spanned by $(1, m, 0)$ and $(0, 0, 1)$, whose unit normal is
$(-m, 1, 0)/\sqrt{m^2+1}$. The angle between that plane and the `t`–`L` plane (normal
$(0,1,0)$) is

$$
\phi = \arctan(|m|).
$$

Equivalently, the curve's tangent at the parabola vertex is $(1, m, 0)$, whose angle off
the `t`–`L` plane is also $\arctan(|m|)$. **So the entire "direction of change" of the
window is captured by a single number: the slope `m` of rotated `Rp'` versus sample
index.**

**The fit.** `m` is the ordinary least-squares slope of `Rp'` against `x = 0 … W−1`:

$$
m = \frac{N\sum_i x_i Rp'_i - \sum_i x_i \sum_i Rp'_i}{N\sum_i x_i^2 - \left(\sum_i x_i\right)^2}.
$$

**The test.** Reject the candidate (reason `deviation`) if

$$
\arctan(|m|) > \texttt{max\_deviation\_rad}.
$$

`max_deviation_rad` is clamped to `[0, π/2]`:

- `0` → the curve must lie *exactly* in the `t`–`L` plane (zero `Rp'` drift allowed) —
  strictest.
- `π/2` → the check is effectively **disabled** (any tilt accepted).
- default **1.0 rad ≈ 57°**.

**Interpreting the units.** The implicit scale is `arctan(Ω per sample)`. At the default
`W = 110` window, a threshold of 1.0 rad permits the linear `Rp'` fit to drift by up to
$\tan(1.0)\cdot 109 \approx 170\ \Omega$ across the full window before rejection — far
above what sensor-noise wiggle produces on a calibrated substrate, but far below the
multi-kΩ drift a material transition produces. **Lowering `crack_deviation` makes the
detector stricter (fewer false positives); raising it toward π/2 loosens it.**

### 5.3 Stage 3 — Dedup / refractory (don't fire twice for one crack)

A single physical crack stays inside the rolling window for many ticks, so without
suppression it would re-fire every tick as its bump slides through. Two mechanisms
prevent this:

1. **Previous-qualified hold.** A boolean `gPreviousQualified` latches whenever a tick
   passes shape + direction. While it is set, an otherwise-good window does **not**
   re-fire (reason `held`). Only the first qualifying tick of a run fires.
2. **Refractory cooldown.** On a confirmed detection a countdown `gRefractoryRemaining`
   is loaded and decremented once per tick (wall-clock in sample ticks). While it is
   nonzero, qualifying windows are suppressed (reason `refractory`). Its length is

   $$
   \texttt{refractory} = \max\big(\lceil w \rceil,\ \lfloor W/4 \rfloor,\ 1\big)
   $$

   i.e. at least the fit's reported width `w` (so the same crack's tail can roll out of
   the window) but never shorter than a quarter window (so a wide window can't fire on
   every quarter-window slide).

A window that clears all four stages **fires a detection** (`detected = true`), latches
the hold flag, and arms the cooldown.

### 5.4 Rejection-reason taxonomy

When `crack_debug` is on, each suppressed tick reports why (static strings in
`crack_detection_result_t::reject_reason`):

| Reason | Stage | Meaning |
|--------|-------|---------|
| *(none, silent)* | 0 | No usable parabola fit (warm-up, singular, opens up, vertex out of window). |
| `low_r2` | 1 | Fit explained too little variance. |
| `threshold` | 1 | Bump too short above baseline. |
| `deviation` | 2 | Curve tilts too far off the `t`–`L` plane (Rp drift → substrate transition). |
| `refractory` | 3 | Inside the post-detection cooldown. |
| `held` | 3 | Previous tick already qualified (same event). |

---

## 6. Material identification (orientation matching)

Source: project memory / `memory.cpp` (`memoryMatchByAngle`). Not part of the per-tick
loop but part of the calibration story.

Because the calibrated rotation angle θ encodes the substrate's conductivity-driven phase
response, it serves as a **material fingerprint**. After calibrating on an unknown
substrate, the `get_material` command compares the current θ against the stored θ of each
saved material profile and reports the closest. The comparison uses a **π-periodic
orientation distance** (θ and θ ± π are the same line, since θ came from `arctan` of a
slope), which also folds the near-vertical ±90° sign flip into a small distance. It is
**report-only**: the operator decides whether to `retrieve` the matched profile.

---

## 7. Output / crack magnitude reporting

Source: `src/telemetry.cpp`. Every tick emits a Teleplot line:

```
>Rp:<val>>L:<val>[ >mag: >crack_x: >crack_size: >width: ]>t:<ms>|xy
```

The `|xy` suffix tells Teleplot to render `Rp` vs `L` as an X-Y plot. On a confirmed
detection (with `crack_output on`) four extra fields are appended:

| Field | Source | Meaning |
|-------|--------|---------|
| `mag` | `fit_peak_height` (`h`) | Raw fitted bump height above the rotated baseline (µH). |
| `crack_x` | derived | Absolute timestamp of the crack's peak, back-computed from the vertex position: $t_{\text{now}} - (W-1-x_v)\,\Delta t$, clamped ≥ 0. |
| `crack_size` | `h ×` `length_estimate_scale` | Peak height scaled into a physical length estimate (default scale **220**, "thou per µH"). |
| `width` | `fit_width_samples` (`w`) | Full width of the bump in samples. |

The mapping from peak height to crack length is a **single linear scale factor**
(`length_estimate_scale`), not a calibrated physical model — the paper should describe it
as a tunable proportionality constant, not a derived length.

---

## 8. Parameters, defaults, and bounds

All values are runtime-tunable over the serial CLI and persist per-material in NVS flash.
Boot defaults are set in `main.cpp::setup()`.

### Preprocessing / calibration

| Parameter | Serial cmd | Default | Bounds | Role |
|-----------|-----------|---------|--------|------|
| Filter window `W_f` | `smoothing` | 25 | `[1, 512]` | Moving-average length (1 = off). |
| Calibration sample count `N` | `calibrate [N]` | 40 | ≥ 1 | Samples used to estimate θ and center. |
| Rotation angle θ | `angle` / `calibrate` | 0 | — | Baseline orientation (set by calibration). |
| Rotation enabled | `rotated` / `calibrate` | off | — | Whether rotation is applied. |

### Crack detector (`crack_detection_config_t`)

| Parameter | Serial cmd | Default | Bounds | Role |
|-----------|-----------|---------|--------|------|
| `threshold` | `crack_threshold` | 0.01 | ≥ 0 | Min fitted peak height above baseline (Stage 1). |
| `window_samples` `W` | `crack_window` | 110 | ≥ 1 | Fit + deviation window length. Raise as sweep speed drops. |
| `min_parabola_r2` | `crack_r2` | 0.5 | `[0, 1]` | Min fit `R²` (Stage 1). |
| `max_deviation_rad` | `crack_deviation` | 1.0 (≈57°) | `[0, π/2]` | Max curve tilt off the `t`–`L` plane (Stage 2). π/2 disables. |
| `length_estimate_scale` | `crack_scale` | 220 | ≥ 0 | Peak-height → length scale for `crack_size` output. |

> Note: the boot default for `min_parabola_r2` in `main.cpp` is **0.5**, while the
> module's hard-coded fallback default (used only if `init()` is skipped) is 0.90. The
> operative value at runtime is 0.5 unless changed.

---

## 9. Notation glossary (for consistent paper symbols)

| Symbol used here | Code name | Meaning |
|------------------|-----------|---------|
| `Rp` | `Rp_ohms` | Raw parallel resistance (Ω). |
| `L` | `L_uH` | Raw inductance (µH). |
| `\bar{Rp}, \bar{L}` | filtered arrays | Moving-average-smoothed `Rp`, `L`. |
| `Rp', L'` | rotated arrays | Smoothed samples after calibration rotation. |
| `θ` | `rotationAngleRad` (= −fit) | Substrate baseline orientation in the `(Rp, L)` plane. |
| `(c_Rp, c_L)` | `rotationCenter*` | Rotation center = recent centroid. |
| `W_f` | `filterWindow` | Smoothing window length. |
| `W` | `window_samples` | Detector fit/deviation window length. |
| `x` | sample index | 0 … W−1, oldest → newest within a window. |
| `a, b, c` | parabola coeffs | `L'(x) − c_L ≈ a x² + b x + c`. |
| `h` | `fit_peak_height` | Parabola vertex height above baseline. |
| `x_v` | `fit_peak_x_samples` | Vertex position in samples. |
| `w` | `fit_width_samples` | Parabola zero-crossing full width. |
| `m` | local `slope` | LS slope of `Rp'` vs sample index. |
| `φ` | `deviationRad` | `arctan(|m|)`, curve tilt off the `t`–`L` plane. |

---

## 10. Design rationale summary (one-paragraph version for an abstract)

A crack passing under an eddy-current probe produces a transient, bump-shaped excursion in
the tank inductance `L` while the loss term `Rp` remains at the substrate's baseline,
whereas confounders such as material-boundary transitions and lift-off drag both `L` and
`Rp`. The firmware exploits this by (i) per-substrate calibration that estimates the
baseline trend orientation in the `(Rp, L)` plane via a least-squares trend line and
rotates incoming samples so a crack becomes a pure-`L` excursion; (ii) a per-window
least-squares **parabola fit** to the rotated `L` that gates on goodness-of-fit (`R²`) and
amplitude, ensuring the right *shape*; and (iii) an analytic **3-D planarity test** that
rejects windows whose `(t, Rp, L)` curve tilts more than `arctan(|m|)` off the
time–inductance plane, where `m` is the linear drift slope of the rotated loss term —
cleanly separating genuine cracks from substrate transitions. A refractory/hold dedup
stage ensures one physical crack yields exactly one detection.

---

## 11. Caveats to keep the paper honest

- **"PCA" vs least-squares.** §4.2 — the orientation is an OLS slope of `L` on `Rp`, not a
  principal-component direction. Either reword or note the first-order equivalence.
- **Linear-drift model in Stage 2.** The planarity test assumes `Rp'(t)` drift is *linear*
  over the window. Nonlinear `Rp` excursions are only approximately captured by their
  best-fit slope; the test is a tilt cap, not an exact planarity certificate.
- **Length estimate is a single scale factor**, not a physics-derived crack length (§7).
- **Detector runs pre-calibration too**, but the firmware only acts on detections
  (LED/flagging) once rotation is enabled, because pre-calibration geometry is
  meaningless. Make clear that calibration is a prerequisite, not optional.
- **Warm-up.** Both the smoothing filter and the detector produce degraded/empty output
  until enough samples have accumulated (`W_f` for the filter, `W` ≥ 3 for the fit).
- **Sample timing.** "Time" in all of the above is **sample index**, converted to
  milliseconds only at output using the configured `reading_delay_ms` tick period; the
  loop is paced non-blocking, so a dropped/slow tick stretches the effective `Δt`.
