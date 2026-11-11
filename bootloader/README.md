# Bootloader

`bootloader-axis_basic-76871b5.bin` is the board's USB bootloader. It sits in
the first 16 KiB of flash and starts the firmware at `0x4000`. Through it,
firmware can be uploaded over USB with `bossac` (`just flash-usb`), or by
copying a UF2 file to the `AXISBASIC` drive the board appears as while the
bootloader runs.

Flash it once per board over SWD with `just flash-bootloader`; see
[Flashing](../README.md#flashing).

## Building it

The bootloader is [uf2-samdx1](https://github.com/adafruit/uf2-samdx1) built
for this board, whose definition is in [`axis_basic`](axis_basic). To build
it, copy that directory into `boards/` of a uf2-samdx1 checkout and run

```sh
make BOARD=axis_basic
```

which needs the `arm-none-eabi-gcc` toolchain. The image is then
`build/axis_basic/bootloader-axis_basic-<version>.bin`.

The firmware asks for the bootloader by writing `0xF01669EF` to the last word
of RAM and resetting, which it does when its serial port is opened and closed
at 1200 baud (see [`fw/src/bootloader.rs`](../fw/src/bootloader.rs)).
Double-tapping the reset button does the same.
