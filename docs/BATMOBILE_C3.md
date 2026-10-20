# Batmobile on the original comma 3

`batmobile-c3` ports Batmobile to the original comma 3 (`tici`). This is a hardware-validation candidate: host builds and unit tests do not establish that it boots, runs inference in real time, or controls a vehicle correctly on the device.

## Compatibility changes

- AGNOS **12.8** on `tici`, with the original signed partition manifest. Other hardware retains its existing OS manifest. The updater rejects a target branch requesting a newer OS on `tici`.
- AR0231 camera probing, exposure controls, HDR registers and ISP configuration. The F4 camera clock and camerad both use simultaneous start-of-frame pulses, including the cabin camera.
- Original DOS/F4 Panda board, USB transport, recovery and bootloader identification. The Panda fork retains the current health/CAN protocol and this branch's existing opendbc safety rules. F4 queues use 512 receive entries and 128 entries per transmit queue to fit the existing SRAM/bootloader layout; a linker assertion protects the stack and reset cookies.
- An isolated Python 3.12 environment from `uv.lock`, retaining AGNOS 12.8's libusb1 3.1.0 compatibility pin, created by `tools/c3_setup.sh`. First boot needs internet access for these dependencies. Later boots reuse the environment until the lockfile changes.
- AGNOS 12.8's Weston service is stopped for the current raylib UI. Modem port aliases are installed in `/run`. Both changes reset on reboot when returning to a legacy branch.
- Sentry's hardware gate accepts the original comma 3. Its existing parking, ignition, temperature and power checks remain in use. Update helpers retain `batmobile-c3`.

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
