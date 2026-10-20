# Batmobile on the original comma 3

`batmobile-c3` ports Batmobile to the original comma 3 (`tici`). This is a hardware-validation candidate: host builds and unit tests do not establish that it boots, runs inference in real time, or controls a vehicle correctly on the device.

## Compatibility changes

- AGNOS **12.8** on `tici`, with the original signed partition manifest. Other hardware retains its existing OS manifest. The updater rejects a target branch requesting a newer OS on `tici`.
- AR0231 camera probing, exposure controls, HDR registers and ISP configuration. The F4 camera clock and camerad both use simultaneous start-of-frame pulses, including the cabin camera.
- Original DOS/F4 Panda board, USB transport, recovery and bootloader identification. The Panda fork retains the current health/CAN protocol and this branch's existing opendbc safety rules. F4 queues use 512 receive entries and 128 entries per transmit queue to fit the existing SRAM/bootloader layout; a linker assertion protects the stack and reset cookies.
- An isolated Python 3.12 environment from `uv.lock`, retaining AGNOS 12.8's libusb1 3.1.0 compatibility pin, created by `tools/c3_setup.sh`. First boot needs internet access for these dependencies. Later boots reuse the environment until the lockfile changes.
- AGNOS 12.8's Weston service is stopped for the current raylib UI. Modem port aliases are installed in `/run`. Both changes reset on reboot when returning to a legacy branch.
- Device-specific speaker routing/EQ, USB IRQ affinity and reset-enumeration delay, AR0231 brightness scaling and a 20 FPS UI.
- EG25 SIM detection configured for the original board wiring; Wi-Fi strength/metering read through AGNOS 12.8's NetworkManager D-Bus interface.
- The original 75 C fan target, NVMe temperature telemetry and missing-storage alert.
- Sentry's hardware gate accepts the original comma 3. Its existing parking, ignition, temperature and power checks remain in use. Update helpers retain `batmobile-c3`.

## Follow-up peripheral audit

Compared the candidate with upstream `master-tici` at `8450f9f3330221caa89f38807c916f35fcbae615`, the tici removal commits, and AGNOS 12.8 build source `1a63095a493207e1dd1a9eaa1c08ff4ebe4ebf81`.

| Area | Finding and action |
| --- | --- |
| u-blox GPS | Retained: `/dev/ttyHS0`, GPIO 32/33/34 power/reset, `pigeond`, `ubloxd`, `gpsLocationExternal` and `UbloxAvailable` selection. The Python parser rewrite did not remove the receiver. No additional GPS driver port needed. |
| Quectel GPS/cellular | `qcomgpsd` remains the alternate GPS path, selected when u-blox is absent or explicitly disabled. Fixed EG25 SIM-detect configuration: the hot-swap signal is not connected on tici. Configuration runs even when SIM identity is incomplete. AGNOS 12.8 includes PPP. |
| Main IMU | LSM6DS3/LSM6DS3TR-C accel, gyro and temperature drivers, I2C bus 1 and interrupt GPIO 84 remain. All original sensor configurations in the pre-removal tests included an LSM. |
| Secondary sensors | BMX055 secondary IMU support was removed before `master-tici` froze; MMC5603NJ magnetometer support was removed later. Current locationd ignores BMX055 and does not consume magnetometer readings. These optional telemetry paths are not restored or replaced with fabricated data. |
| Audio | Restored original MAX98089 right-speaker routing, gain and five EQ bands. The 3X profile is preserved. AGNOS 12.8 retains sound-card/microphone initialization. |
| USB Panda | Restored a three-second enumeration interval after GPIO reset/recovery, plus USB host IRQ affinity. An immediate reset/probe loop can continually interrupt USB enumeration. |
| Display | Restored AR0231 exposure-to-brightness scaling and selected the same 20 FPS UI budget as 3X. Backlight, touch and display drivers come from AGNOS 12.8. |
| Wi-Fi | UI connection management still uses NetworkManager. Hardware status now also uses its D-Bus API on tici; it no longer depends on the newer supplicant control socket for signal strength and metering. |
| Cooling | Restored the 75 C fan target. The later 5 C increase assumes the thermal-throttling change in AGNOS 18.1 and is inappropriate for the pinned 12.8 image. |
| NVMe | AGNOS 12.8 retains the NVMe driver and `/data/media` mount. Restored temperature telemetry at the existing Cap'n Proto field ordinal 35 and a missing-mount alert, including devices configured for eMMC-only logging. Current logger process-failure checks remain in force. |

The old NVMe model-brand warning, logger-failure exemption and source-tree factory-reset NVMe formatting were also removed upstream. They are not required drivers and are not reintroduced by this audit. Device erase/recovery behavior must be qualified separately on the pinned OS.

Additional host validation: **51 tests passed**, covering GPS receiver/service selection, u-blox power sequencing and a framed NAV-PVT packet, speaker register writes, USB reset timing and IRQ placement, modem/SIM setup, Wi-Fi D-Bus requests/metering, NVMe serialization/mount handling, OS selection and fan control. Native params/messaging bindings and the updated schema built successfully; Python lint and diff checks passed. These are host tests, not receiver acquisition, acoustic, network or thermal measurements on a device.

## Preparing a device checkout

Keep a working device recovery path and the previous checkout. Use the `batmobile-c3` branch of `gm1500/openpilot` and initialize its submodules recursively; the Panda submodule points to `gm1500/panda`. Download Git LFS assets before starting the launcher. This source branch builds on the device and does not include prebuilt binaries.

The launcher selects `openpilot/common/hardware/comma/agnos-tici.json` from the hardware model. If the installed OS differs, the normal A/B updater stages and verifies AGNOS 12.8, switches slots and reboots. Do not substitute the main `agnos.json`, which targets newer devices.

## Validation completed for this port

- STM32F4 application and bootstub, and STM32H7 application compiled successfully.
- `pandad` and its CAN protocol test built and ran, including eight-byte classic CAN frames on DOS.
- `camerad` compiled and linked on Linux with Clang 18 and the restored Qualcomm headers (host build, not a device run).
- 152 Python tests and 3 subtests passed across Panda USB protocol/legacy identification, device OS selection, update guards and Sentry activation/update behavior.
- C++ hardware identification checked for `tici`, `tizi` and `mici`; Python lint and shell syntax checks passed.
- All seven AGNOS 12.8 manifest image URLs returned HTTP 200.

No original comma 3 was connected during development. Full device builds, runtime behavior and hardware/vehicle tests remain outstanding.

## Required device qualification

Before any road use, validate on the original hardware while parked:

1. Cold boot, UI/touch/audio, display blanking/wake and clean shutdown on AGNOS 12.8.
2. All three AR0231 streams, exposure/HDR, continuous frame IDs and simultaneous camera timestamps; run `python -m unittest openpilot.system.camerad.test.test_camerad` with manager stopped and Panda initialized. Check driver monitoring and IR in light and darkness.
3. DOS Panda identification, firmware/recovery, ignition, harness orientation, relay behavior, CAN errors/queue overflow and fan control. Verify safety mode and firmware signatures agree with the checked-out branch.
4. IMU, GNSS, Wi-Fi, cellular, model inference timing and thermal behavior under sustained load. The current tinygrad/raylib stack on the older kernel requires hardware verification.
5. Sentry recording/playback, ignition transitions, camera restart, thermal pause, parking power accounting and shutdown. Review recordings and logs before enabling unattended use.
6. Perform vehicle replay and controlled validation for the specific supported vehicle before relying on driving behavior.

## Upstream provenance

- AGNOS 12.8 manifest: commaai/openpilot `31101ecaab71620cfdcc31dc3074117b78c7e10a`.
- AR0231 sources: parent of commaai/openpilot `1d8dc8a69a188285e65595d83224e54d497178dc` (AR0231 removal).
- USB pandad and Qualcomm UAPI headers: parent of commaai/openpilot `96d1b876bbd7441cac08bab842aded6729ae1c6d`.
- F4 platform: commaai/panda `3dc21386239e3073a623156b75901aa302340d6c`, adapted to Batmobile's current Panda interfaces.
- Modem aliases: commaai/agnos-builder's current `78-modem.rules`.

Follow-up audit references:

- tici-specific removal: commaai/openpilot `3e2549f2b8675ce482200e06855857122f3583e3`.
- NVMe diagnostics removal: `a8fd55740bd7c88ecb9cc34546d2a72b840d3817`.
- BMX055 removal / supported sensor combinations: `50aaa691373222dcc71c506c1d055e21c5bc97ea`.
- Remaining dual-IMU services removal: `752ef8696af9687b7ea3d0309ab8f73990d6b61d`.
- MMC5603NJ removal: `f8372ccc4df0b5b0e49572f8817ce7d5b14c6db9`.
- Weston updater fallback removal (already handled by the initial port): `8499de6afe1b7aa22a8ee753c25b1ac595cc6e8a`.
