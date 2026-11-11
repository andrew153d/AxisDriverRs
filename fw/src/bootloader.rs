//! Handing over to the UF2 bootloader that lives in the first 16 KiB of
//! flash, so that new firmware can be uploaded over USB.

use cortex_m::peripheral::SCB;

/// End of RAM, as in `memory.x`.
const RAM_END: u32 = 0x2000_0000 + 192 * 1024;
/// Where the bootloader looks, after a reset, for a request to stay in
/// bootloader mode instead of starting the firmware: the last word of RAM.
const REQUEST_ADDRESS: *mut u32 = (RAM_END - 4) as *mut u32;
const REQUEST_MAGIC: u32 = 0xF016_69EF;

/// Baud rate a host sets, before closing the port again, to ask for the
/// bootloader: the "1200 baud touch" convention of Arduino-style boards.
pub const TOUCH_BAUD_RATE: u32 = 1_200;

/// Resets the microcontroller into its bootloader.
pub fn enter() -> ! {
    cortex_m::interrupt::disable();
    // SAFETY: the address is the top word of the stack, which is not in use
    // this deep into the program, and nothing runs between this write and
    // the reset.
    unsafe { REQUEST_ADDRESS.write_volatile(REQUEST_MAGIC) };
    SCB::sys_reset()
}
