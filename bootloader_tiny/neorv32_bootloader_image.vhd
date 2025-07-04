-- The NEORV32 RISC-V Processor - github.com/stnolting/neorv32
-- Auto-generated memory initialization image (for internal BOOTROM)
-- Source: bootloader_tiny/build/main.bin
-- Built: 04.07.2025 15:37:36

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

package neorv32_bootloader_image is

constant bootloader_init_size_c  : natural := 24; -- bytes
constant bootloader_init_image_c : mem32_t := (
x"002077b7",
x"ffef0737",
x"60178793",
x"00f72023",
x"e00002b7",
x"00028067"
);

end neorv32_bootloader_image;
