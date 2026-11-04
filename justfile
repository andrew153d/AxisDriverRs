bossa := env_var('HOME') + "/.platformio/packages/tool-bossac/bossac"

# Query the SAMD51 bootloader. Pass its bootloader serial port, e.g. /dev/ttyACM0.
ping port:
    "{{bossa}}" --port "{{port}}" --info

# Build and flash the LED firmware over SWD with the Atmel-ICE.
flash-led probe="03eb:2141:J42700037763":
    cargo build --release -p fw
    probe-rs download --chip ATSAMD51J19A --probe "{{probe}}" --protocol swd --verify target/thumbv7em-none-eabihf/release/fw
    probe-rs reset --chip ATSAMD51J19A --probe "{{probe}}" --protocol swd
