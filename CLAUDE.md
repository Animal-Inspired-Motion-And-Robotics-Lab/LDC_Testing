# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

PlatformIO firmware targeting the **Seeed XIAO ESP32-S3** (Arduino framework) that drives a **TI LDC1101** inductance-to-digital converter over SPI. The application reads `Rp` (parallel resistance) and `L` (inductance) and streams them over USB serial in Teleplot format.

The LDC1101 datasheet (`lib/LDC1101/ldc1101.pdf`) is the reference for register layout, RP-table values, and the formulas the driver uses to derive `RP_SET`, `TC1`, `TC2`, and `DIG_CONF` from sensor `L`, `C`, `Q`.

## Common commands

Run from the project root (where `platformio.ini` lives):

- `pio run` — build
- `pio run -t upload` — build and flash
- `pio device monitor` — serial monitor (115200 baud per `platformio.ini`)
- `pio run -t clean` — clean build artifacts

## Architecture

### Framework mixing — important

`platformio.ini` declares `framework = arduino`, and `src/main.cpp` is a standard Arduino sketch. **However, `lib/LDC1101/src/ldc1101.cpp` is written against ESP-IDF directly** (`driver/spi_master.h`, `driver/gpio.h`, `freertos/FreeRTOS.h`, `esp_log.h`). This works because the Arduino-ESP32 core is built on top of ESP-IDF and re-exports those headers. Implication: this driver is **ESP32-specific** — it will not compile on AVR/ARM Arduino targets. When modifying the driver, prefer ESP-IDF SPI/GPIO APIs to keep the style consistent, rather than mixing in Arduino `SPI.h`.

### SPI pin & bus configuration via build flags

The driver's pin assignments and SPI host are not hard-coded in source — they come from `-D` defines in `platformio.ini` (`LDC_MOSI`, `LDC_MISO`, `LDC_SCK`, `LDC_CS`, `LDC_SPI_HOST`, `LDC_SPI_CLOCK_HZ`). `ldc1101.cpp` provides fallback `#ifndef` defaults but production wiring lives in `platformio.ini`. To re-pin the LDC1101, edit `build_flags` there, not the `.cpp`.

### Library API surface

`lib/LDC1101/include/ldc1101.h` is the public header, wrapped in `extern "C"`. The lifecycle is:

1. `ldc1101_init()` — initializes the SPI bus and reads the chip ID.
2. `ldc1101_configure(L_h, C_sensor, Q, mode, speed_mode, switch_enable, switch_gpio)` — computes all register values from the sensor's specs and writes them. `mode` selects `LDC1101_MODE_RP_L` (Rp + L conversion) or `LDC1101_MODE_LHR` (high-resolution L). `speed_mode` selects an accuracy↔throughput tradeoff that drives the `C1`/`C2` and response-time bits. The optional `switch_gpio` toggles an external mux/switch before configuring.
3. `ldc1101_read(C_sensor)` returns an `ldc1101_measurement_t { Rp_ohms, L_uH, timestamp_ms }`. In `LHR` mode `Rp_ohms` is `NAN`.

`ldc1101_sleep()` / `ldc1101_wake()` toggle `REG_START_CONFIG`.

The register-derivation logic (`ldc1101_make_rp_set`, `make_tc1`, `make_tc2`, `make_dig_conf`) implements the formulas from the LDC1101 datasheet. When changing these, cross-reference the PDF rather than guessing.

### Measurement post-processing

`include/measurement_arrays.h` + `src/measurement_arrays.cpp` (application-level, not part of the LDC1101 library) maintain a sliding window of the most recent `array_length` `(Rp, L)` samples and compute the principal-axis angle (`calculateDominantAngle`) via PCA on that 2-D point cloud. The intent is to extract a dominant trend direction in the `Rp`/`L` plane.

### Serial output format

`main.cpp` emits Teleplot-compatible lines: `>Rp:<val>>L:<val>>t:<ms>|xy`. The trailing `|xy` flag asks Teleplot to render `Rp` vs `L` as an X-Y plot. Keep this format if you want existing plots to keep working.

## Conventions

- The LDC1101 driver is a clean C API under `extern "C"` — keep it C-callable; don't leak C++ types through `ldc1101.h`.
- Sensor parameters (`kSensorL_H`, `kSensorC_F`, `kSensorQ` in `main.cpp`) must match the physical LC tank wired to the chip. Comment in `main.cpp` records values for the stacked-inductor variants; update both the value and the comment together.
