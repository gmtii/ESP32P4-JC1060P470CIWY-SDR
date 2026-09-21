# ESP32-P4 SDR receiver for the JC1060P470CIWY board

A standalone software-defined radio receiver running on an ESP32-P4 board
(JC1060P470CIWY, 1024×600 MIPI-DSI display). It receives with an
**RTL-SDR Blog V4** connected to the P4's USB High-Speed host port, demodulates
on the chip, shows a spectrum/waterfall on the LVGL touch-screen UI and plays
the audio through an external **NAU8822** codec.

> **Status: experimental.** The RTL-SDR input path has been validated with
> host-side simulations (see [`test/`](test/)). On-air results: *(update me)*.

## Features

- Demodulation: USB, LSB, AM, synchronous AM (SAM, SAM-L, SAM-U) and FM
- Selectable filter bandwidth per mode, noise reduction, software AGC
- Spectrum and waterfall display, S-meter
- Rotary encoder + button for tuning, LVGL menus for mode, filter and gain
- UART command interface
- 100 % on-chip DSP (ESP-DSP): no PC required

## Hardware

| Part | Notes |
|------|-------|
| Board | ESP32-P4, JC1060P470CIWY, 1024×600 display (JD9165 controller) |
| Receiver | RTL-SDR Blog V4 on the **USB High-Speed host** port |
| Audio out | NAU8822 codec, I2S at 48 kHz (control over a 3-wire serial bus) |
| Controls | Rotary encoder with push button |

The USB connector used for the dongle must be wired to the P4's **High-Speed**
USB PHY and must provide 5 V on VBUS. The driver does not manage VBUS: board
power switching, if any, is the application's job. Check your board schematic
for the correct connector and any OTG host/device jumper. Do not use the
UART/flash port.

## Signal path

```
RTL-SDR Blog V4 ──USB HS──► esp_rtl_sdr (USB Host client, callback mode)
        CU8 I/Q @ 960 kSps
              │  rtl_dsp.c
              │   CIC 5:1  ─► variable-delay cubic resampler (192 kSps) ─► 96-tap FIR 4:1
              ▼
        int16 I/Q @ 48 kSps ──► FIFO (rtl_source.c) ──► sdr.c
              │                      ▲
              │        FIFO level steers the resampler to absorb
              │        the dongle-vs-codec clock difference
              ▼
   ±fs/4 shift, ÷4 decimation (12 kHz), demodulation, filters, NR, AGC,
   ×4 interpolation ─► I2S TX ─► NAU8822 ─► speaker/headphones
```

Design notes:

- **LO offset.** The dongle's LO is tuned to `VFO − 12 kHz` (`FREQ_CONV_OFFSET`);
  the wanted signal sits 12 kHz off centre and is brought to baseband by the
  fs/4 shift in `sdr.c`. This also keeps the RTL's DC spike out of the passband.
- **Clock drift.** The dongle and the codec run from different crystals. The
  consumer measures the FIFO level once per audio block (extrapolating the time
  since the last USB block arrived) and steers the resampler step with a
  proportional controller. Samples are never dropped or repeated in normal
  operation; a 100 ppm error costs a 0.01 % pitch offset.
- **Spectrum orientation.** `sdr.c` expects the signal at −12 kHz for an LO at
  `VFO − 12 kHz`, so `rtl_source.c` negates Q by default
  (`RTL_SOURCE_CONJUGATE_IQ 1` in `main/include/rtl_source.h`). If USB and LSB
  come out swapped, or a known carrier is not at the centre marker of the
  spectrum, set it to 0.
- **Tuning is non-blocking.** `rtl_source_set_freq()` and
  `rtl_source_set_gain_db()` only post a request; a control task talks to the
  dongle (latest request wins), so they are safe to call from LVGL callbacks.

## RTL-SDR driver (`esp_rtl_sdr`)

USB access to the dongle is provided by
[**esp-rtl-sdr**](https://github.com/hardcoreerik/esp-rtl-sdr) by hardcoreerik,
a clean-room ESP-IDF USB Host client for RTL2832U-class dongles (not a librtlsdr
port). It targets the ESP32-P4 High-Speed host, with the RTL-SDR Blog V4 as its
primary supported device. This project was developed against v0.8.0-rc3.

The driver is used in **callback delivery mode**: it hands each USB block to
`rtl_source.c`, which converts it to 48 kSps I/Q and keeps its own FIFO.

### Adding the driver to the project

1. Put the driver in `components/esp_rtl_sdr` (the folder name is the component
   name; the folder that contains the driver's `CMakeLists.txt` and `include/`
   must sit directly under `components/`):

   ```sh
   git submodule add https://github.com/hardcoreerik/esp-rtl-sdr components/esp_rtl_sdr
   ```

2. Add the component to `main/CMakeLists.txt` if the build cannot find
   `esp_rtl_sdr.h`:

   ```cmake
   idf_component_register(... PRIV_REQUIRES esp_rtl_sdr)
   ```

3. With **ESP-IDF 6.x** the USB Host stack is no longer part of IDF; add it to
   `main/idf_component.yml`:

   ```yaml
   dependencies:
     espressif/usb: "*"
   ```

   The driver itself declares ESP-IDF ≥ 5.5 and its author tests on 5.5.x, so
   6.x is not covered by the driver's own testing.

4. In `sdkconfig.defaults`:

   ```
   CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=1024
   ```

   Keep the P4 chip-revision settings your board needs.

### Runtime API (`main/include/rtl_source.h`)

| Function | Purpose |
|----------|---------|
| `rtl_source_init(lo_hz, gain_db)` | Start the control task; installs the driver and streams as soon as a dongle is present. Does not block on USB. |
| `rtl_source_set_freq(lo_hz)` | Request a new LO frequency (VFO − `FREQ_CONV_OFFSET`). |
| `rtl_source_set_gain_db(db)` / `rtl_source_set_gain_auto(bool)` | Manual tuner gain (0–50 dB) or the tuner's AGC. |
| `rtl_source_read_float(i, q, n, timeout_ms)` | Blocking read of `n` I/Q frames at 48 kSps, called from the SDR task. Times out if no dongle is present. |
| `rtl_source_is_streaming()` | Dongle attached and streaming. |
| `rtl_source_get_stats(&s)` | FIFO level, drift correction (ppm), under/overruns, driver USB counters. |

Hot-plug is handled by the control task: on disconnect or fault it stops and
resets the driver, then restarts the stream with the last requested frequency
and gain.

### Notes and limits

- **8-bit ADC.** The RTL-SDR has far less dynamic range than the previous
  16-bit codec input. Set the gain (menu slider, 0–50 dB) so strong signals do
  not overload the receiver, especially on crowded HF bands.
- **HF and LF.** Below 28.8 MHz the V4 receives through its built-in
  upconverter, handled by the driver. The driver documents LF (below 500 kHz)
  as experimental. Minimum tuning frequency for the V4 is 24 kHz of LO.
- **S-meter.** The level scale differs from the codec input; the S-meter offset
  in `smeter.c` needs recalibration.
- **Retunes are USB transactions.** Each VFO change reprograms the dongle's LO
  over USB and takes a few milliseconds. Fine tuning by shifting in software
  inside the 960 kHz capture window (retuning only when leaving it) is a
  possible future improvement.
- **Task priorities.** The driver's USB task runs at priority 20 and its
  delivery task at 18. If `usb_overruns` grows in `rtl_source_get_stats()`, lower
  the SDR task priority below 18.

### Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| `ERR_NO_DEVICE`, no `streaming` log line | Dongle in the wrong port (must be the HS host port), no VBUS, or dongle not a supported Blog V4 |
| `install()` returns `ESP_RTL_SDR_ERR_USB_SAFE_MODE` | The driver's fault guard latched after repeated enumeration panics. Unplug the dongle, call `esp_rtl_sdr_usb_fault_guard_reset()`, reconnect, then install again |
| USB and LSB swapped, or carrier off the centre marker | Flip `RTL_SOURCE_CONJUGATE_IQ` |
| `usb_overruns` > 0, `effective_sps` < 960000 | USB port not at High-Speed, or SDR task starving the driver's delivery task |
| Periodic `underruns` | SDR task not keeping up; check DSP load and LVGL task priority |

## Building

Requires ESP-IDF for the ESP32-P4 target.

```sh
idf.py set-target esp32p4
idf.py build
idf.py -p PORT flash monitor
```

Managed dependencies include LVGL 9.2, ESP-DSP, `esp_lvgl_port`,
`esp_codec_dev`, the display driver and the knob/button components (see
`main/idf_component.yml`). Some of them need explicit version constraints
depending on the ESP-IDF release; the working set is recorded in
`dependencies.lock`.

## Tests

The DSP chain and `rtl_source.c` have host-side tests that need only `gcc` (a
fake driver and a pthread FreeRTOS shim stand in for the hardware):

```sh
ESP_RTL_SDR_DIR=components/esp_rtl_sdr ./test/run_host_tests.sh
```

They cover amplitude response and alias rejection, I/Q orientation, closed-loop
drift tracking (±100 ppm, no under/overruns, no phase discontinuities), retune,
gain, unplug and replug.

## Repository layout

```
main/
  main.c, sdr.c            application entry, SDR task (demodulation chain)
  rtl_source.c/.h          esp_rtl_sdr glue: control task, FIFO, drift loop
  rtl_dsp.c/.h             CU8 → 48 kSps I/Q conversion (pure C, host-testable)
  ui.c, menu.c, smeter.c   LVGL interface
  nau8822.c, i2s_driver.c  audio output codec
  uart_commands.c          UART command interface
components/esp_rtl_sdr/    RTL-SDR USB driver (git submodule)
test/                      host-side tests
```

## Licence

Project licence: *(add yours)*.

`esp_rtl_sdr` is licensed **AGPL-3.0-only**. Firmware that links it is a
combined work: check the AGPL's terms before distributing binaries or hosting
the firmware as a service.

## Credits

- [esp-rtl-sdr](https://github.com/hardcoreerik/esp-rtl-sdr) by hardcoreerik: the RTL2832U USB Host driver
- [ESP-DSP](https://github.com/espressif/esp-dsp), [LVGL](https://lvgl.io/) and the Espressif component ecosystem
