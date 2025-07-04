// #################################################################################################
// # << NEORV32 - XIP Bootloader >>                                                                #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Alberto Fahrenkrog. All rights reserved.                                  #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32       (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file bootloader_xip.c
 * @author Alberto Fahrenkrog
 * @brief NEORV32 XIP Bootloader
 **************************************************************************/

// Libraries
#include <stdint.h>
#include <neorv32.h>


/**********************************************************************//**
 * @name Bootloader configuration (override via console to customize)
 * default values are used if not explicitly customized
 **************************************************************************/
/**@{*/

/* ---- UART interface configuration ---- */

/** Set to 0 to disable UART interface */
#ifndef UART_EN
  #define UART_EN 1
#endif

/** UART BAUD rate for serial interface */
#ifndef UART_BAUD
  #define UART_BAUD 115200
#endif

/* ---- Status LED ---- */

/** Set to 0 to disable bootloader status LED (heart beat) at GPIO.gpio_o(STATUS_LED_PIN) */
#ifndef STATUS_LED_EN
  #define STATUS_LED_EN 0
#endif

/** GPIO output pin for high-active bootloader status LED (heart beat) */
#ifndef STATUS_LED_PIN
  #define STATUS_LED_PIN 0
#endif

/* ---- Boot configuration ---- */

/** Set to 1 to enable automatic (after reset) only boot from external SPI flash at address SPI_BOOT_BASE_ADDR */
#ifndef AUTO_BOOT_SPI_EN
  #define AUTO_BOOT_SPI_EN 0
#endif

/** Set to 1 to enable boot only via on-chip debugger (keep CPU in halt loop until OCD takes over control) */
#ifndef AUTO_BOOT_OCD_EN
  #define AUTO_BOOT_OCD_EN 0
#endif

/** Set to 1 to enable simple UART executable upload (no console, no SPI flash) */
#ifndef AUTO_BOOT_SIMPLE_UART_EN
  #define AUTO_BOOT_SIMPLE_UART_EN 0
#endif

/** Time until the auto-boot sequence starts (in seconds); 0 = disabled */
#ifndef AUTO_BOOT_TIMEOUT
  #define AUTO_BOOT_TIMEOUT 5
#endif

/* ---- XIP configuration ---- */

/** Enable XIP module (default) for flash boot options */
#ifndef XIP_EN
  #define XIP_EN 1
#endif

/** SPI flash address width (in numbers of bytes; 2,3,4) */
#ifndef SPI_FLASH_ADDR_BYTES
  #define SPI_FLASH_ADDR_BYTES 3 // default = 3 address bytes = 24-bit
#endif

/** SPI flash sector size in bytes */
#ifndef SPI_FLASH_SECTOR_SIZE
  #define SPI_FLASH_SECTOR_SIZE 65536 // default = 64kB
#endif

/** SPI flash boot base address */
#ifndef SPI_FLASH_BASE_ADDR
  #define SPI_FLASH_BASE_ADDR 0x00000000
#endif

/** XIP Page Base */
#ifndef XIP_PAGE_BASE
  #define XIP_PAGE_BASE 0xE0000000
#endif
/**@}*/


/**********************************************************************//**
 * Error codes
 **************************************************************************/
enum ERROR_CODES {
  ERROR_FLASH     = 0x08, /**< 8: SPI flash access error */
  ERROR_XIP_SETUP = 0x10, /**< 16: XIP Setup error */
  ERROR_FLASH_WR  = 0x40, /**< 64: Flash write error */
};


/**********************************************************************//**
 * SPI flash commands
 **************************************************************************/
enum SPI_FLASH_CMD {
  SPI_FLASH_CMD_WRITE_BYTES   = 0x02, /**< Program page */
  SPI_FLASH_CMD_READ          = 0x03, /**< Read data */
  SPI_FLASH_CMD_READ_STATUS   = 0x05, /**< Get status register */
  SPI_FLASH_CMD_WRITE_ENABLE  = 0x06, /**< Allow write access */
  SPI_FLASH_CMD_SECTOR_ERASE  = 0xD8  /**< Erase complete sector */
};


/**********************************************************************//**
 * Bootloader Console commands
 **************************************************************************/
enum CONSOLE_CMDS {
    POKE            = 0x00, /** Just a simple poke to check if bootloader is on **/
    EXECUTE         = 0x01, /** Executes application from SPI Flash using XIP **/
    FLASH_WRITE     = 0x02, /** Loads new executable to flash **/
    FLASH_READ      = 0x03, /** Reads the executable from flash and sends it to the console **/
    RESET           = 0x04, /** Resets the bootloader **/
    ERASE_SECTOR    = 0x06, /** Flash Erase Sector Command **/
    STATUS_BYTE     = 0x07, /** Read Status Byte Command **/
};

// SPI Flash data
union {
  uint64_t uint64;
  uint32_t uint32[sizeof(uint64_t)/sizeof(uint32_t)];
} data;

// Function prototypes
void system_error(uint8_t err_code);

// Flash function prototypes 
void flash_read(uint32_t addr, uint32_t len);
void flash_write(uint32_t addr, uint32_t len, char* data_ptr);
void flash_erase_sector(uint32_t addr);
void flash_read_status(void);


/**********************************************************************//**
 * Sanity check: Base ISA only!
 **************************************************************************/
#if defined __riscv_atomic || defined __riscv_a || __riscv_b || __riscv_compressed || defined __riscv_c || defined __riscv_mul || defined __riscv_m
  #warning In order to allow the bootloader to run on *any* CPU configuration it should be compiled using the base ISA only.
#endif


/**********************************************************************//**
 * Bootloader main.
 **************************************************************************/
int main(void)
{
  char console_cmd = 0;
  uint32_t aux = 0;
  uint32_t retval = 0;

#if (XIP_EN != 0)
  neorv32_xip_setup(CLK_PRSC_2, 1, 0, 0, 0x03);
#else
  #error In order to use the XIP bootloader, the XIP module must be implemented
#endif

#if (UART_EN != 0)
  // setup UART0 (primary UART, no parity bit, no hardware flow control)
  neorv32_uart0_setup(UART_BAUD, 0);
#endif

  // ------------------------------------------------
  // Auto boot sequence
  // ------------------------------------------------
  {
    uint64_t timeout_time = neorv32_clint_time_get() + (uint64_t)(AUTO_BOOT_TIMEOUT * NEORV32_SYSINFO->CLK);

    while(retval == 0)
    {
      // wait for a poke...
      if (neorv32_uart0_char_received())
      {
        neorv32_uart0_putc(0x00);
        retval = 1;
      }

      if(retval == 0)
      {
        if (neorv32_clint_time_get() >= timeout_time) 
        { // timeout? start auto boot sequence
          // configure and enable the actual XIP mode
          // * configure 3 address bytes send to the SPI flash for addressing
          // * map the XIP flash to the address space starting at XIP_PAGE_BASE
          if (neorv32_xip_start(SPI_FLASH_ADDR_BYTES))
          {
            system_error(0xFF & ERROR_XIP_SETUP);
          }

            // finally, jump to the XIP flash's base address we have configured to start execution **from there**
            asm volatile ("call %[dest]" : : [dest] "i" (XIP_PAGE_BASE + SPI_FLASH_BASE_ADDR));
            while(1);
        }
      }
    }
  }

  uint32_t addr = 0;
  uint32_t len = 0;
  char data_page[256];

  // ------------------------------------------------
  // Main loop
  // ------------------------------------------------
  while (1)
  {

    console_cmd = neorv32_uart0_getc();

    if(console_cmd == RESET)
    {
      neorv32_uart0_putc(console_cmd);
      while (neorv32_uart0_tx_busy()); /** Wait for transmission to end **/
      
      asm volatile ("li t0, %[input_i]; jr t0" :  : [input_i] "i" (NEORV32_BOOTROM_BASE)); // jump to beginning of boot ROM
    }
    else if (console_cmd == POKE)
    {
      neorv32_uart0_putc(console_cmd);
    }
    else if (console_cmd == EXECUTE)
    {
      neorv32_uart0_putc(console_cmd);
      // configure and enable the actual XIP mode
      // * configure 3 address bytes send to the SPI flash for addressing
      // * map the XIP flash to the address space starting at XIP_PAGE_BASE
      retval = 0;
      if (neorv32_xip_start(SPI_FLASH_ADDR_BYTES))
      {
        system_error(0xFF & ERROR_XIP_SETUP);
      }

      // finally, jump to the XIP flash's base address we have configured to start execution **from there**
      asm volatile ("call %[dest]" : : [dest] "i" (XIP_PAGE_BASE + SPI_FLASH_BASE_ADDR));
      while(1);
    }
    else if (console_cmd == FLASH_WRITE)
    {
      /* This only writes max. 256 bytes at a time! */
      for(aux = 0; aux < 4; aux++)
      {
        ((char *)&addr)[aux] = neorv32_uart0_getc();
      }

      for(aux = 0; aux < 4; aux++)
      {
        ((char *)&len)[aux] = neorv32_uart0_getc();
      }
      
      for(aux = 0; aux < len; aux++)
      {
        data_page[aux] = neorv32_uart0_getc();
      }

      flash_write(addr, aux, data_page);
      neorv32_uart0_putc(0x00);

    }
    else if (console_cmd == FLASH_READ)
    {
      for(aux = 0; aux < 4; aux++)
      {
        ((char *)&addr)[aux] = neorv32_uart0_getc();
      }

      for(aux = 0; aux < 4; aux++)
      {
        ((char *)&len)[aux] = neorv32_uart0_getc();
      }
      flash_read(addr, len);
    }
    else if (console_cmd == ERASE_SECTOR)
    {
      for(aux = 0; aux < 4; aux++)
      {
        ((char *)&addr)[aux] = neorv32_uart0_getc();
      }
      flash_erase_sector(addr);
      neorv32_uart0_putc(console_cmd);
    }
    else if (console_cmd == STATUS_BYTE)
    {
      flash_read_status();
    }
  }

  return 1; // bootloader should never return
}


/**********************************************************************//**
 * Erase flash sector by address (24 bit)
 **************************************************************************/
void flash_erase_sector(uint32_t addr)
{
  data.uint64 = 0; // Init to zero before any operation
  
  // set status bits to 000
  // 1 byte command
  //data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
  // data.uint32[1] = 0; // command: set write-enable latch
  neorv32_xip_spi_trans(1, &data.uint64);
    
  // set write-enable latch
  // 1 byte command
  //data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
  data.uint32[1] = (SPI_FLASH_CMD_WRITE_ENABLE << 24) & 0xFF000000; // command: set write-enable latch
  neorv32_xip_spi_trans(1, &data.uint64);

  data.uint64 = 0; 
  // erase sector
  // 1 byte command + 3 byte address
  //data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
  data.uint32[1] = (SPI_FLASH_CMD_SECTOR_ERASE << 24) & 0xFF000000; // command: erase sector
  data.uint32[1] |= addr & 0x00FFFFFF; // address data
  neorv32_xip_spi_trans(SPI_FLASH_ADDR_BYTES + 1, &data.uint64);

  data.uint64 = 0; 
  // check status register: WIP bit has to clear
  while(1) {
    // data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
    data.uint32[1] = (SPI_FLASH_CMD_READ_STATUS << 24) & 0xFF000000;
    neorv32_xip_spi_trans(2, &data.uint64);
    if ((data.uint32[0] & 0x01) == 0) { // WIP bit cleared?
      break;
    }
  }
}


/**********************************************************************//**
 * Read content from flash and send it to client
 **************************************************************************/
void flash_read(uint32_t addr, uint32_t len)
{
  uint32_t tmp = 0;
  uint32_t cnt = 0;
  uint32_t aux = 0;

  cnt = 0;
  aux = addr;
  while(cnt < len)
  {
    // read word
    // 1 byte command, 3 bytes address, 1 byte data
    tmp = SPI_FLASH_CMD_READ << 24; // command: byte read
    tmp |= (aux & 0x00FFFFFF); // address
    data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
    data.uint32[1] = tmp;
    neorv32_xip_spi_trans(SPI_FLASH_ADDR_BYTES + 2, &data.uint64);
    aux++;
    cnt++;

    neorv32_uart0_putc(data.uint32[0] & 0xFF);
  }
}

/**********************************************************************//**
 * Write data to flash
 **************************************************************************/
void flash_write(uint32_t addr, uint32_t len, char* data_ptr)
{
  uint32_t idx;
  uint32_t data_idx = 0;
  uint32_t tmp = 0;
  char* tmp_ptr = 0;

  tmp_ptr = (char *)&data.uint32[0];

  data_idx = 0;
  while(data_idx < len)
  {
    // Set Enable Write 
    data.uint32[1] = (SPI_FLASH_CMD_WRITE_ENABLE << 24) & 0xFF000000;
    neorv32_xip_spi_trans(1, &data.uint64);
    // write word
    // 1 byte command, 3 bytes address, 4 bytes data
    tmp = SPI_FLASH_CMD_WRITE_BYTES << 24; // command: byte read
    tmp |= ((addr + data_idx) & 0x00FFFFFF); // address

    for(idx = 0; idx < 4; idx++)
    {
      if(data_idx < len)
      {
        tmp_ptr[3 - idx] = data_ptr[data_idx];
        data_idx++;
      }
    }
    data.uint32[1] = tmp;
    neorv32_xip_spi_trans(SPI_FLASH_ADDR_BYTES + 5, &data.uint64);

    // check status register: WIP bit has to clear
    while(1)
    {
      // data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
      data.uint32[1] = (SPI_FLASH_CMD_READ_STATUS << 24) & 0xFF000000;
      neorv32_xip_spi_trans(2, &data.uint64);
      if ((data.uint32[0] & 0x01) == 0)
      { // WIP bit cleared?
        break;
      }
    }
  }
}


/**********************************************************************//**
 * Read status byte from flash
 **************************************************************************/
void flash_read_status(void)
{
  // read flash status byte
  // 1 byte command, 1 byte data
  data.uint32[0] = 0; // irrelevant, TX packet is MSB-aligned
  data.uint32[1] = (SPI_FLASH_CMD_READ_STATUS << 24) & 0xFF000000;
  neorv32_xip_spi_trans(3, &data.uint64);

  neorv32_uart0_putc(data.uint32[0] & 0xFF);  
}

/**********************************************************************//**
 * Output system error ID and stall.
 *
 * @param[in] err_code Error code. See #ERROR_CODES and #error_message.
 **************************************************************************/
void system_error(uint8_t err_code) {
  while(1); // freeze
}
