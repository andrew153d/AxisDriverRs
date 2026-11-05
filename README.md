# AxisDriver SAMD51 LED firmware

This workspace builds a `no_std` SAMD51J firmware image that alternates PB13
as a push-pull status output and sets the PB5 WS2812 LED to dim green. It
targets `thumbv7em-none-eabihf`.

Build the ELF:

```sh
cargo build --release -p fw
```

Upload through SAM-BA on ACM0:

```sh
chmod +x flash.sh
./flash.sh
```

`flash.sh` creates `target/thumbv7em-none-eabihf/release/fw.bin` and uses the
SAM-BA command for an ATSAMD51J19A. Override the serial device or exact part number
when needed:

```sh
PORT=/dev/ttyACM0 SAMBA_DEVICE=atsamd51j20a ./flash.sh
```

`fw/memory.x` is sized for a SAMD51J19A (512 KiB flash, 192 KiB RAM). Update it
and `SAMBA_DEVICE` if the controller has a different SAMD51 package or density.
