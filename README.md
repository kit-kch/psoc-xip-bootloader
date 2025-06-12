# The NEORV32 PSoC Bootloader

## Introduction

The PSoC Bootloader is a fork of the [XIP Bootloader](https://github.com/betocool-prog/neorv32-xip-bootloader).
It comes in two versions: A [full one](./bootloader_full/) and a [tiny one](./bootloader_tiny/), that's used to reduce code size and therefore the bootloader rom.

The tiny version can only boot an external flash in XIP mode and can't do anything else.
Programming then needs to be done with an external flash programmer or with a JTAG based flash programmer.

The full version allows you to write a binary file to flash, read data from flash, erase sectors, check the status byte and execute your program from flash using UART commands.
To reduce the code size, a binary protocol is used for UART communication.

The bootloader setup consists of three files, `bootloader_xip.c` for each bootloader and `neorv_sh.py` for the full one.

* [bootloader_xip.c](bootloader_xip.c) is the source code for each bootloader.
* [neorv_sh.py](neorv_sh.py) is a command line interface which lets you interact with the full bootloader.


## Usage

### NEORV Version

This bootloader was tested and built for NEORV32 version `1.11.0`.

### Prebuilt files

Each of the bootloaders comes with a ready to use `neorv32_bootloader_image.vhd` file.
In addition, we also ship the compiled main.elf to enable debugging of the bootloader.

### Compiling the bootloader
Running
```bash
make clean_all
make bl_image
```
will generate the XIP bootloader VHDL file `neorv32_bootloader_image.vhd` in the bootloader folder.
Reference this one instead of the one in the NEORV32 directory and re-build your VHDL project.

The default settings for the XIP bootloader are:
- 3 byte addresses
- Executable start address in flash:  0x000000
- XIP base address 0xE0000000 (hardcoded in NEORV32 v1.11.0)

The values above can be updated in the `bootloader_xip.c` files.

### Compiling a firmware application
For a compiled application to run, the `__neorv32_rom_base` needs to be defined to the (XIP base address + Flash start address) when linking.
You can achieve this by adding the following to your `USER_FLAGS` in the `makefile`, before including the `common.mk` file:

```Makefile
USER_FLAGS=-Wl,--defsym=__neorv32_rom_base=0xE0000000
```

Instead of modifying makefiles, you can also define this flag on the command line:
```bash
make USER_FLAGS=-Wl,--defsym=__neorv32_rom_base=0xE0000000 elf
```

### Debugging the Bootloader

You can debug the bootloader using JTAG and gdb.
To get proper debug info, do not load your applications `main.elf` but the one belonging to the bootloader.
If you want to soft-reset while the JTAG debugger is active, just jump to the bootloader address:

```bash
riscv-none-elf-gdb

target extended-remote localhost:3333
file main.elf
# Whatever you want to debug
#hbreak _start
hbreak main
j _start
```

Note: You can also jump to the bootloader when debugging an applicaiton, but you'll have to use the address:
```bash
j *(0xffe00000)
```

### Running the command line interface, neorv_sh.py

The command line interface (CLI) requires Python 3.x to run, as it uses the CMD module. It is based on a simple request - response communication mode. The CLI sends a 1-byte command requesting a write or a read, waits for the response, and either sends the rest of the data or processes the information received.

Run the script using `python neorv_sh.py`.

At bootloader start, you have 5 seconds to *poke* the bootloader and prevent it from jumping to the application. There are two ways of doing this:
- Reset the processor. It starts in bootloader mode. Start the CLI script. It always sends a *poke* command at startup.
- If the CLI is already running, reset the processor and execute the *poke* command within 8 seconds.

The implemented commands are:

- help: Shows help information
- erase_sector *address*: Erases a flash sector by address. Default sector size is 64KBytes.
- execute: Executes the program from flash stored at the base address using the XIP module.
- flash_read *address* *length*: Dumps the data at the specified address, for the specified length, on screen.
- flash_write *address* *path/to/file.bin*: Writes the contents of file.bin at the specified address.
- poke: Send a '0' character and expect a '0' back. Used to prevent the bootloader from timing out at the start or to check if it's still running.
- reset: Resets the bootloader
- status_byte: Returns the value of the flash memory's status byte