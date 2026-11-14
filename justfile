# The BOSSA command line tool, version 1.9 or later. Set BOSSAC to use one
# that is not on the PATH.
bossa := env_var_or_default('BOSSAC', 'bossac')
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

# Run the tests of the web control panel. Needs Node.js 20 or later.
test-web:
    node --test web/tests/*.test.mjs

# Serve the web control panel at http://localhost:8137 for trying it locally.
# Pass another port if that one is taken, e.g. `just web 8200`.
web port="8137":
    python3 -m http.server --bind localhost --directory web "{{port}}"

# Create the Python environment in .venv with the `axis` package installed.
python-setup:
    python3 -m venv .venv
    .venv/bin/pip install --quiet --editable python

# Regenerate the Python message classes after changing proto/axis.proto.
# The Rust ones are generated at build time.
proto:
    protoc -Iproto --python_out=python/axis --pyi_out=python/axis proto/axis.proto

# Show the board's identity, self test result and configuration.
info *args:
    .venv/bin/python -m axis {{args}} info

# Show the state of the motor.
status *args:
    .venv/bin/python -m axis {{args}} status

# Print the firmware's log, fetched over USB. Pass "--follow" to keep
# watching, e.g. `just logs --follow`.
logs *args:
    .venv/bin/python -m axis logs {{args}}

# Open the control panel.
ui *args:
    .venv/bin/python -m axis {{args}} ui

# Run the motor through a short sequence of moves.
demo *args:
    .venv/bin/python python/examples/demo.py {{args}}

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
