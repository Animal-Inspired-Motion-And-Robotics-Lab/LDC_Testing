# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

PlatformIO firmware targeting the **Seeed XIAO ESP32-S3** (Arduino framework) that drives a **TI LDC1101** inductance-to-digital converter over SPI. The LDC1101 is wired to an LC tank used as an **eddy-current probe**; the firmware is the sensor electronics for a crack-detection payload mounted on an **insect-scale mobile robot** that sweeps the probe across conductive substrates and watches for the EM signature of a crack passing underneath. It samples `Rp` (parallel resistance) and `L` (inductance), runs them through a smoothing → rotation → fit-then-phase pipeline, and streams the result over USB serial in Teleplot format. Runtime behavior (sensor specs, mode, filter window, crack-detector tuning) is reconfigurable over the same serial link.

The LDC1101 datasheet (`lib/LDC1101/ldc1101.pdf`, grep-friendly text extraction at `lib/LDC1101/ldc1101.txt`) is the reference for register layout, RP-table values, and the formulas the driver uses to derive `RP_SET`, `TC1`, `TC2`, and `DIG_CONF` from sensor `L`, `C`, `Q`.

## Common commands

Run from the project root (where `platformio.ini` lives):

- `pio run` — build
- `pio run -t upload` — build and flash
- `pio device monitor` — serial monitor (115200 baud per `platformio.ini`; note that USB-CDC on the XIAO ESP32-S3 ignores the firmware-side `Serial.begin()` baud, which is why `main.cpp` can call `Serial.begin(9600)` without breaking the link)
- `pio run -t clean` — clean build artifacts

## Architecture

The codebase splits cleanly into two layers:

- **Reusable driver** under `lib/LDC1101/` — a clean C API for talking to the chip. No application logic.
- **Application** under `src/` and `include/` — the measurement pipeline, crack detector, serial command interpreter, and LED feedback that together form the firmware.

### LDC1101 driver — framework mixing, important

`platformio.ini` declares `framework = arduino`, and `src/main.cpp` is a standard Arduino sketch. **However, `lib/LDC1101/src/ldc1101.cpp` is written against ESP-IDF directly** (`driver/spi_master.h`, `driver/gpio.h`, `freertos/FreeRTOS.h`, `esp_log.h`). This works because the Arduino-ESP32 core is built on top of ESP-IDF and re-exports those headers. Implication: this driver is **ESP32-specific** — it will not compile on AVR/ARM Arduino targets. When modifying the driver, prefer ESP-IDF SPI/GPIO APIs to keep the style consistent, rather than mixing in Arduino `SPI.h`.

### SPI pin & bus configuration via build flags

The driver's pin assignments and SPI host are not hard-coded in source — they come from `-D` defines in `platformio.ini` (`LDC_MOSI`, `LDC_MISO`, `LDC_SCK`, `LDC_CS`, `LDC_SPI_HOST`, `LDC_SPI_CLOCK_HZ`). `ldc1101.cpp` provides fallback `#ifndef` defaults but production wiring lives in `platformio.ini`. To re-pin the LDC1101, edit `build_flags` there, not the `.cpp`.

### Driver API surface

`lib/LDC1101/include/ldc1101.h` is the public header, wrapped in `extern "C"`. Lifecycle:

1. `ldc1101_init()` — initializes the SPI bus and reads the chip ID.
2. `ldc1101_configure(L_h, C_sensor, Q, mode, speed_mode, switch_enable, switch_gpio)` — computes all register values from the sensor's specs and writes them. `mode` selects `LDC1101_MODE_RP_L` (Rp + L conversion) or `LDC1101_MODE_LHR` (high-resolution L). `speed_mode` selects an accuracy↔throughput tradeoff that drives the `C1`/`C2` and response-time bits. The optional `switch_gpio` toggles an external mux/switch before configuring.
3. `ldc1101_read(C_sensor)` returns an `ldc1101_measurement_t { Rp_ohms, L_uH, timestamp_ms }`. In `LHR` mode `Rp_ohms` is `NAN`.

`ldc1101_sleep()` / `ldc1101_wake()` toggle `REG_START_CONFIG`. The register-derivation logic (`ldc1101_make_rp_set`, `make_tc1`, `make_tc2`, `make_dig_conf`) implements the formulas from the LDC1101 datasheet — cross-reference the PDF rather than guessing.

### Application measurement pipeline

Each `loop()` tick in [src/main.cpp](src/main.cpp) does, in order:

1. `ldc1101_read()` → raw `(Rp, L)`.
2. `appendMeasurement(rp, l)` in [src/measurement_arrays.cpp](src/measurement_arrays.cpp) pushes the sample through a moving-average filter (window settable via `setFilterWindow`, exposed as the `smoothing` serial command), then through a 2-D rotation about a stored center if rotation is enabled.
3. `crackDetectionCheck()` in [src/crack_detection.cpp](src/crack_detection.cpp) runs a two-stage detector:
   1. **Shape check (L vs time).** Fit a parabola to the most recent `window_samples` of rotated `L` (x is sample index). If R² ≥ `min_parabola_r2` and fitted peak height ≥ `threshold`, the window is a *crack candidate*.
   2. **3D direction check (t, Rp, L).** Picture the candidate window as a curve in `(t, Rp, L)` space. A real crack lies near the `t-L` plane — L bumps in time while Rp stays at the calibrated substrate baseline — whereas substrate motion (material patch boundary, baseline slide) tilts the curve off that plane toward the L-Rp plane. We characterize the tilt analytically: for a parabolic L plus a linear-in-time Rp drift with slope `m`, the curve lies exactly in the plane spanned by `(1, m, 0)` and `(0, 0, 1)`, whose unit normal is `(−m, 1, 0)/√(m²+1)`, so the angle between that plane and the t-L plane is `arctan(|m|)`. The tangent at the parabola vertex is `(1, m, 0)`, with the same angle off the t-L plane — the curve's "direction of change" is captured entirely by `m`. We fit `m` by least squares over the rotated window and reject when `arctan(|m|) > max_deviation_rad`. Tuned via `crack_deviation` (default `1.0` rad ≈ 57°; `π/2` disables the check). The implicit unit scale is arctan(Ω per sample): at the default 110-sample window, 1.0 rad corresponds to roughly 170 Ω of total predicted Rp drift across the window, which sensor-noise wiggle never approaches but a material transition blows past.
4. The latest sample — rotated or unrotated, depending on `state.rotated` — is emitted to serial in Teleplot format, with the optional crack-magnitude triple appended when a confirmed detection fires this tick.

**Calibration sets the rotation, and it is per-material.** [src/calibration.cpp](src/calibration.cpp) runs `calculateDominantAngleRecent` (PCA on the most recent `N` filtered samples in `measurement_arrays`) to find the principal axis of the `Rp`/`L` cloud — physically, this axis encodes the substrate's baseline phase response, which is set by its conductivity. The routine then calls `setRotationAngle(-theta)` and `setRotationCenter(mean_rp, mean_l)`, so subsequent samples are rotated to put the "no crack" trend on the new x-axis and crack excursions on the new L axis. The `calibrate` serial command triggers this; recalibrate whenever the robot is moved to a different substrate.

### Serial command interface

[src/serial_commands.cpp](src/serial_commands.cpp) is a line-oriented CLI polled from `loop()`. It owns runtime state (`mode`, `speed_mode`, `streaming_enabled`, `rotated`, `crack_output`, `crack_debug_output`, `reading_delay_ms`) and sensor config (`sensor_l_h`, `sensor_c_f`, `sensor_q`, switch settings); `main.cpp` reads these every tick via `serialCommandsGetState()` / `GetConfig()`. Commands that change LDC1101 parameters call `ldc1101_configure()` internally to push the new register values. Send `help` over serial for the full command list, or `status` for current values.

When extending the firmware with a new tunable, the convention is: add it to the relevant module's getter/setter pair, then add a `processCommand` clause in `serial_commands.cpp` and a line to `printHelp()` / `printStatus()` so it's discoverable. If the value is part of a material's identity (it should survive a power cycle and travel with `save`/`retrieve`), also add a key for it in `memory.cpp` — see below.

### Persistent material profiles (NVS)

[src/memory.cpp](src/memory.cpp) persists a complete settings snapshot — a *material profile* — to the ESP32-S3's NVS flash so a substrate's tuning survives a power cycle and several materials can be stored side by side. Five serial commands drive it: `save <material>`, `retrieve <material>`, `materials` (list), `get_material` (identify closest by angle), and `forget <material>`.

- **Storage layout.** One NVS namespace per material (named after the material itself), one key per setting. A reserved `_materials` namespace holds a `\n`-delimited index of saved names so `materials` can enumerate them, since NVS can't list namespaces. NVS caps namespace names at 15 chars, so material names are limited to `MEMORY_MAX_NAME_LEN` and a leading `_` is rejected to keep the index private.
- **What's saved.** Everything user-configurable: the `serial_command_config_t` / `serial_command_state_t` fields, plus the live `measurement_arrays` rotation (filter window, angle, center) and the full `crack_detection_config_t`. `memory.cpp` is the single owner of the key layout — it reads/writes the rotation and crack settings through their module getters/setters directly; only the sensor/mode fields are passed in via the structs.
- **Division of labor.** `memorySaveMaterial` / `memoryRetrieveMaterial` do **not** touch the LDC1101. After a successful retrieve, `serial_commands.cpp` calls `configureSensor()` to push the restored sensor/mode/speed to the chip; the rotation-enable and crack settings are already applied inside `memoryRetrieveMaterial`. A `v` (schema-version) key written on save doubles as the "profile exists" sentinel that `retrieve` checks.
- **Material identification (`get_material`).** `memoryMatchByAngle` scans every saved profile and reports the one whose stored rotation angle is closest to the current (just-calibrated) angle from `getRotationAngle()`. The intended workflow is `calibrate` on an unknown substrate, then `get_material` to see which stored profile it resembles (named distinctly from the `materials` list command). The rotation angle is the discriminating parameter because it encodes the substrate's conductivity-driven phase response. Matching uses a **π-periodic orientation distance** (`orientationDistance`), since the angle comes from `atan` of a slope — θ and θ±π are the same trend line, and this folds the near-vertical ±90° sign flip into a small distance instead of ~180°. It is **report-only** by design (per the operator's choice): it prints `closest=`, the saved/current angles, and the difference, but changes nothing — the operator runs `retrieve <name>` to actually load it.

This means adding a new persisted tunable is a two-touch change: the getter/setter + CLI clause above, **and** a matching `prefs.putX`/`getX` pair in `memorySaveMaterial`/`memoryRetrieveMaterial` (keep the key name ≤ 15 chars).

### Serial output format

Streaming lines are Teleplot-compatible: `>Rp:<val>>L:<val>` plus an optional `>mag:>half:>width:` triple when a qualified crack is being emitted (`crack_output on`), then `>t:<ms>|xy`. The trailing `|xy` flag asks Teleplot to render `Rp` vs `L` as an X-Y plot. The `crackdebug on` command additionally emits a plain-text key=value debug line per tick — not Teleplot-parseable, intended for tuning sessions. Keep the `>Rp:>L:` prefix and `|xy` suffix if you want existing plots to keep working.

## Conventions

- The LDC1101 driver is a clean C API under `extern "C"` — keep it C-callable; don't leak C++ types through `ldc1101.h`.
- Sensor parameters (`kSensorL_H`, `kSensorC_F`, `kSensorQ` in `main.cpp`) are **boot defaults only** — the live values are owned by `serial_commands` and changeable at runtime. The comment in `main.cpp` records modeled values for the stacked-inductor variants (L = 11.8, 42.6, 90.0 µH at 220 pF, Q ≈ 23.6/24.6/25.6); update both the value and the comment together.
- Crack detector tuning lives in `crack_detection_config_t` constructed in `main.cpp::setup()`. These are also serial-tunable (`crack_window`, `crack_threshold`, `crack_r2`, `crack_deviation`, `crack_scale`), so changes to the struct defaults should be matched against the serial command bounds checks. `max_deviation_rad` is clamped to `[0, π/2]` — the codomain of `arctan(|m|)` for any real slope `m`; `π/2` disables the check.
