/* ATSAMD51J19A: 512 KiB flash, 192 KiB RAM.

   The first 16 KiB of flash hold the UF2 bootloader (see bootloader/), which
   starts the firmware at 0x4000. */
MEMORY
{
  FLASH : ORIGIN = 0x00004000, LENGTH = 512K - 16K
  RAM : ORIGIN = 0x20000000, LENGTH = 192K
}
