#!/bin/sh
set -eu

PORT=${PORT:-/dev/ttyACM0}
SAMBA_DEVICE=${SAMBA_DEVICE:-atsamd51j19a}
TARGET=thumbv7em-none-eabihf
ELF=target/$TARGET/release/fw
IMAGE=target/$TARGET/release/fw.bin

cargo build --release -p fw
llvm-objcopy -O binary "$ELF" "$IMAGE"
sam-ba -p "serial:$PORT" -d "$SAMBA_DEVICE" -c "send_file {Flash} $IMAGE 0x00000000"
