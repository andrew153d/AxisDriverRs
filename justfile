bossa := env_var('HOME') + "/.platformio/packages/tool-bossac/bossac"
chip := "ATSAMD51J19A"
elf := "target/thumbv7em-none-eabihf/release/fw"
# Where the firmware starts in flash, after the bootloader.
app_offset := "0x4000"

# Build the firmware.
build:
    cargo build --release -p fw

# Run the unit tests of the hardware-independent logic on the host.
test:
    cargo test -p axis-core --target "$(rustc -vV | sed -n 's/^host: //p')"

# Build and upload the firmware over USB, through the bootloader.
flash-usb port="/dev/ttyACM0": build
    llvm-objcopy -O binary "{{elf}}" "{{elf}}.bin"
    just enter-bootloader "{{port}}"
    "{{bossa}}" --port "{{port}}" --usb-port --offset "{{app_offset}}" --erase --write --verify --reset "{{elf}}.bin"

# Ask running firmware to reboot into the bootloader (the "1200 baud touch").
# Does nothing if the board is already in the bootloader.
enter-bootloader port="/dev/ttyACM0":
    stty -F "{{port}}" 1200
    sleep 3

# Print the firmware's log, fetched over USB. Pass "--follow" to keep
# watching, e.g. `just logs --follow`.
logs *args:
    python3 tools/logs.py {{args}}

# Move the motor, e.g. `just move to 90`, `just move spin 400 -t 3` or
# `just move demo`. See `just move --help`.
move *args:
    python3 tools/move.py {{args}}

# Query the bootloader. The board must be in bootloader mode.
ping port="/dev/ttyACM0":
    "{{bossa}}" --port "{{port}}" --usb-port --info

# Build and flash the firmware over SWD with the Atmel-ICE.
flash probe="03eb:2141:J42700037763": build
    probe-rs download --chip {{chip}} --probe "{{probe}}" --protocol swd --verify "{{elf}}"
    probe-rs reset --chip {{chip}} --probe "{{probe}}" --protocol swd

# Flash the USB bootloader over SWD. Only needed once per board, or after
# something has overwritten the first 16 KiB of flash.
flash-bootloader probe="03eb:2141:J42700037763":
    probe-rs download --chip {{chip}} --probe "{{probe}}" --protocol swd --verify --binary-format bin --base-address 0x0 bootloader/bootloader-axis_basic-76871b5.bin
    probe-rs reset --chip {{chip}} --probe "{{probe}}" --protocol swd
