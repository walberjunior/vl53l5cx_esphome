# VL53L5CX ESPHome External Component
This component was developed with AI (claude) assistance and tested on real hardware, although not every feature or configuration has been hardware-validated.
> [!WARNING]
> **Be careful when configuring `Matrix Update Interval`.** The VL53L5CX can generate a large amount of matrix data, especially at 8×8 resolution and when several matrices or targets are enabled. Setting this interval too low can flood ESPHome/Home Assistant with state updates, increase memory and processing load, and may cause Home Assistant to restart. **Start with `1000 ms` (the default) and lower it gradually only after verifying stability.** Setting it to `0` publishes every newly acquired frame and can create a very high update rate.
>
> `Matrix Update Interval` controls how often matrix states are published; it does **not** change the sensor's ranging frequency. Use `Ranging Frequency` to change the acquisition rate.

An ESPHome external component for the **STMicroelectronics VL53L5CX multizone time-of-flight (ToF) distance sensor**, based on **`VL53L5CX_ULD_driver_2.0.1`**.

It exposes distance and diagnostic matrices as Home Assistant text sensors, provides controls for acquisition and software-side filtering, and includes an additional empty-environment calibration workflow.

## Features

### Distance matrices and diagnostics

The component supports the sensor's **4×4 and 8×8 resolutions**. Matrix entities are published as text, with comma-separated values and `|` between rows. Unavailable target slots are represented by `nan`. The output is kept within Home Assistant's 255-character text-state limit; in 8×8 matrices, numeric values are limited to three digits in the formatted output.

The following text-sensor matrices can be enabled individually in YAML:

| Entity key | Contents | Processing |
|---|---|---|
| `ambient_per_spad_raw` | Ambient rate per SPAD, per zone | Raw ULD values |
| `number_of_targets_detected_raw` | Number of targets detected, per zone | Raw ULD values |
| `number_of_targets_detected` | Number of accepted targets, per zone | Processed; applies the status filter and distance/environment rules |
| `number_of_spads_enabled_raw` | Enabled SPAD count, per zone | Raw ULD values |
| `signal_per_spad_raw` | Signal rate for each target index | Raw ULD values |
| `range_sigma_raw` | Range sigma for each target index | Raw ULD values |
| `target_status_raw` | ULD target-status code for each target index | Raw status codes; not filtered |
| `reflectance_raw` | Reflectance for each target index | Raw ULD values |
| `distance_raw` | Distance in centimetres for each target index | Raw distance; not status-, minimum-, maximum-, or environment-filtered |
| `distance` | Distance in centimetres for each target index | Processed distance |
| `motion_indicator_raw` | Motion indicator aggregates | Optional; always a 4×4 matrix when enabled |

For per-target keys, configure a YAML list with one text sensor per target index (for example, `T1` and `T2`). The number of target slots is controlled by `targets_per_zone` at compile time. `Target Order` chooses whether target slots are ordered by **Closest** or **Strongest**.

Two regular diagnostic sensor entities are also available:

- `driver_error_raw`: most recent ULD/driver status code (`0` means no error).
- `temperature_raw`: sensor silicon temperature in °C, republished when it changes and at least once every 60 seconds.

### Processed distance and target-status filtering

Processing affects `distance` and `number_of_targets_detected`; the `*_raw` matrices retain the original ULD readings.

- **Target Status Filter** — software-side filter with the following modes:
  - `Disabled`: accepts every status code for a detected target.
  - `Range Valid`: accepts status codes `5` and `9`.
  - `Permissive`: accepts every code except `0` and `255`.
  - `High Confidence`: accepts only code `5`.
  - `Confidence Based`: accepts codes `5`, `6`, and `9`.
- **Minimum Distance** — readings closer than the configured value are not counted as targets. Useful for rejecting near-field glitches, such as an unexpected `0 cm` reading.
- **Maximum Distance** — invalid, missing, or farther-than-limit readings are represented by the configured maximum in the processed `Distance` matrix. A target is counted only when it passes the status/minimum-distance checks, is below the maximum, and is not rejected by the active environment filter.
- **Matrix Rotation** — rotates every displayed matrix clockwise by `0°`, `90°`, `180°`, or `270°`. It is a software-side transformation; it does not rotate the physical sensor. All matrices use the same mapping.

Default filter mode is `Range Valid`. Default minimum and maximum distances are `10 cm` and `400 cm` respectively.

### Empty-environment filter and calibration

The additional **Calibrate Empty Environment** button records the background of an empty detection area and prints suggested per-zone distance references to the ESPHome logs. Those references can then be saved in `components/vl53l5cx/environment.conf` and compiled into the firmware.

1. Keep the monitored passage or area completely empty.
2. Set the sensor to **8×8**. Calibration runs temporarily at **1 Hz**; the previous ranging frequency is restored at the end when still compatible with the configuration.
3. Press **Calibrate Empty Environment** and wait for the configured `Calibration Duration`.
4. Copy the reported `#!` directives and zone lines (without ESPHome's timestamp/log prefix) into `environment.conf`. Review the suggestions, recompile, and flash the firmware.
5. Use the **Environment Filter** switch to enable or disable filtering at runtime.

This calibration is a custom empty-environment feature; it is **not** the original ST calibration flow. Calibration is supported only at 8×8 in this version. The passage must remain empty throughout calibration. The report can flag noisy or wide distance bands for review; do not blindly accept every suggested reference.

`environment.conf` is a required file, but it can be empty or contain only comments, in which case no environment references are applied. If zone entries are included, the file must specify `#! resolution: 8x8` and `#! rotation: 0`, `90`, `180`, or `270`. An optional `#! max_distance_cm: ...` records the maximum distance used at calibration time. Zone entries support up to four non-overlapping `distance/delta` references per zone, in centimetres. See the comments in `components/vl53l5cx/environment.conf` for the file format.

The environment filter is bypassed while the sensor is running at 4×4. If the environment file is empty, turning the filter on has no effect. Distance limits, environment-filter state, and calibration duration are saved across reboots.

### Acquisition controls

The `select` and `number` entities provide runtime controls for:

| Entity key | Purpose | Supported values / notes |
|---|---|---|
| `resolution` | Matrix resolution | `4x4`, `8x8` |
| `ranging_mode` | Sensor ranging mode | `Continuous`, `Autonomous` |
| `ranging_frequency` | Measurement frequency | 1–60 Hz at 4×4; 1–15 Hz at 8×8 |
| `integration_time` | Integration time | 2–1000 ms; only effective in Autonomous mode and constrained by resolution/frequency |
| `sharpener` | Sensor sharpening setting | 0–99% |
| `target_order` | Order of target slots | `Closest`, `Strongest` |
| `periodic_temperature_compensation` | Interval between periodic VHV/temperature compensations | 0–86400 seconds; `0` disables it |
| `matrix_update_interval` | Matrix state publication interval | 0–60000 ms; `0` publishes every new frame; does not change ranging frequency |
| `target_status_filter` | Software target-status filter | `Disabled`, `Range Valid`, `Permissive`, `High Confidence`, `Confidence Based` |
| `matrix_rotation` | Displayed matrix orientation | `0°`, `90°`, `180°`, `270°` clockwise |
| `minimum_distance` | Reject processed targets that are too close | 0–100 cm, and must remain below Maximum Distance |
| `maximum_distance` | Upper bound for processed distances | 1–400 cm, and must remain above Minimum Distance |
| `calibration_duration` | Empty-environment calibration duration | 10–3600 seconds |

Hardware-affecting settings are validated and applied by stopping, reconfiguring, and restarting ranging. Software-side settings such as target-status filtering, matrix rotation, distance processing, matrix publication interval, and logging do not require sensor ranging to be restarted. Settings are persisted across reboot, except for matrix logging switches.

### Matrix logging

Each matrix has its own logging switch. Enabling a switch prints that matrix to the ESPHome logs for every acquired frame, independently of the Home Assistant publication interval. Logging switches always start **OFF after boot**. Enable only the matrices needed for debugging, especially at high ranging frequencies, to avoid excessive log output.

### Optional motion indicator

The ST motion-indicator plugin is **disabled by default** to reduce firmware result-data size and I²C traffic. To compile it in, set `motion_indicator: true` under `vl53l5cx` and configure `motion_distance_min` and `motion_distance_max` in millimetres. The distance span must not exceed 1500 mm. When enabled, `motion_indicator_raw` exposes the plugin's 4×4 aggregate grid, and an optional debug switch can print the raw motion structure to the logs.

The example YAML in this repository contains motion-related entries commented with `#M#`; remove that marker from the required lines when enabling the feature.

## Requirements and limitations

- ESPHome external component, using the **ESP32** target. **ESP8266 is not supported** because the ULD firmware image and data structures do not fit its available RAM.
- An STMicroelectronics VL53L5CX sensor connected over I²C.
- The sensor's 7-bit I²C address defaults to `0x29`. ST documentation sometimes lists `0x52`, which is the corresponding 8-bit address form.
- The sample configuration uses ESP-IDF and I²C at 1 MHz. The example comments note that 400 kHz can also work, although 8×8 with four targets may require a lower ranging frequency at that bus speed.
- `targets_per_zone` is a **compile-time** ULD setting from 1 to 4. Changing it requires recompiling and flashing the firmware. Text-sensor target lists must not contain more entries than this setting.
- `environment_file` defaults to `environment.conf` and must exist in the component directory, even if it is empty or fully commented.

### Hardware options not yet tested

The following options are supported by the code but the configurations below have **not been tested on hardware**:

```yaml
vl53l5cx:
  address: 0x29
  lpn_pin: GPIO4       # Optional; recommended to let the component power-cycle the sensor
  interrupt_pin:       # Optional A3/INT line; polling is used when omitted
    number: GPIO5
    mode: INPUT_PULLUP
```

With `lpn_pin`, the component toggles the LPn pin during startup to power-cycle the sensor. With `interrupt_pin`, the component uses the falling edge of the INT line to signal that data is ready, and includes a polling fallback if an interrupt is not observed. Without the interrupt pin, it polls the sensor over I²C. Verify the electrical connections and pin behavior for your board before relying on these options.

The component also accepts an optional `i2c_reset_pin` for the sensor's I²C reset line; follow the sensor/module's electrical requirements before using it.

## Installation and configuration

Place this component under the ESPHome configuration directory so that its files are available at `components/vl53l5cx/`. The supplied `example.yaml` uses:

```yaml
external_components:
  - source:
      type: local
      path: components
```

A minimal hub configuration looks like this:

```yaml
i2c:
  sda: GPIO21
  scl: GPIO22
  frequency: 1MHz
  scan: true

vl53l5cx:
  id: tof
  address: 0x29
  targets_per_zone: 4
  i2c_chunk_size: 64
  environment_file: environment.conf
```

Declare the entity platforms you want (`sensor`, `text_sensor`, `select`, `number`, `switch`, and/or `button`) and associate each with `vl53l5cx_id: tof`. The complete configuration, with example entities and optional motion settings, is provided in [`example.yaml`](example.yaml).

### Main component options

| Option | Default | Description |
|---|---:|---|
| `address` | `0x29` | I²C device address |
| `lpn_pin` | Not set | Optional LPn output used to power-cycle the sensor at startup |
| `i2c_reset_pin` | Not set | Optional I²C reset output |
| `interrupt_pin` | Not set | Optional GPIO input for the sensor's data-ready INT signal; otherwise the component polls |
| `targets_per_zone` | `4` | Compile-time target slots per zone, from 1 to 4 |
| `i2c_chunk_size` | `64` bytes | I²C transfer chunk size, from 16 to 1024 bytes |
| `environment_file` | `environment.conf` | Required file of empty-environment references; may be empty/comment-only |
| `motion_indicator` | `false` | Enables the optional ST motion-indicator plugin and related entities |
| `motion_distance_min` | `400` mm | Minimum range for the motion indicator; used when the plugin is enabled |
| `motion_distance_max` | `1500` mm | Maximum range for the motion indicator; used when the plugin is enabled |

## Not implemented from the original driver functionality

The following original features are not implemented by this component:

- Calibration flow (the component has its own, separate **empty-environment calibration** described above).
- Xtalk margin.
- Detection thresholds.

## Error handling

If initialization or data acquisition fails, the component reports ULD status codes in the logs and through `driver_error_raw` when that entity is configured. Failed initialization triggers a full initialization retry after 15 seconds. Check I²C wiring, power, address, bus speed, and the diagnostic code if the sensor is not detected or frames cannot be read.

## Source and licensing

This component is based on STMicroelectronics **`VL53L5CX_ULD_driver_2.0.1`** with ESPHome integration code and additional software-side processing. See [`components/vl53l5cx/LICENSE_ST.txt`](components/vl53l5cx/LICENSE_ST.txt) and any applicable package license terms for licensing details.
