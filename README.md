# ESP-NOW backed serial connection
Transparent serial link between two ESP32 devices. Primarily made for MAVLink
communication with Holsatus Flight+Ground, but can be used for anything else.
This code was originally thrown together quickly, so things can likely be
improved. Feel free to submit changes.

If the ESP32 has an on-board LED, it will blink when the two devices are not
paired, and light up constantly once they reliably receive each others heartbeat
messages.

## Xtensa toolchain

This is designed for an Xtensa-based ESP32, so a toolchain must be installed
manually. See [this](https://github.com/esp-rs/rust-build) repository for more
information.