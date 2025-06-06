-- The NEORV32 RISC-V Processor: https://github.com/stnolting/neorv32
-- Auto-generated memory initialization file (for BOOTLOADER) from source file <bootloader_tiny/main.bin>
-- Size: 20 bytes
-- MARCH: default
-- Built: 06.06.2025 13:17:44

-- prototype defined in 'neorv32_package.vhd'
package body neorv32_bootloader_image is

constant bootloader_init_image : mem32_t := (
x"024077b7",
x"60578793",
x"f4f02023",
x"200002b7",
x"00028067"
);

end neorv32_bootloader_image;
