#include <neorv32.h>

#define SPI_FLASH_ADDR_BYTES 3
#define SPI_FLASH_BASE_ADDR 0x00000000
#define XIP_PAGE_BASE 0xE0000000

__attribute__((noreturn)) void _start(void)
{
    uint32_t ctrl = 0;
    ctrl |= ((uint32_t)(1)) << XIP_CTRL_EN; // enable module
    ctrl |= ((uint32_t)(CLK_PRSC_2 & 0x07)) << XIP_CTRL_PRSC0;
    ctrl |= ((uint32_t)(0   & 0x01)) << XIP_CTRL_CPOL;
    ctrl |= ((uint32_t)(0   & 0x01)) << XIP_CTRL_CPHA;
    ctrl |= ((uint32_t)(0x3 & 0xff)) << XIP_CTRL_RD_CMD_LSB;
    ctrl |= 1 << XIP_CTRL_SPI_CSEN; // finally enable automatic SPI chip-select

    // address bytes send to SPI flash
    ctrl |= ((uint32_t)(SPI_FLASH_ADDR_BYTES-1)) << XIP_CTRL_XIP_ABYTES_LSB; // set new configuration

    // total number of bytes to transfer via SPI
    // 'abytes' address bytes + 1 command byte + 4 bytes RX data (one 32-bit word)
    ctrl |= ((uint32_t)(SPI_FLASH_ADDR_BYTES+1+4)) << XIP_CTRL_SPI_NBYTES_LSB; // set new configuration

    ctrl |= 1 << XIP_CTRL_XIP_EN; // enable XIP mode

    NEORV32_XIP->CTRL = ctrl;

    __asm__ volatile (
      "li t0, %0\n\t"
      "jalr zero, 0(t0)"  
      : : "i"(XIP_PAGE_BASE + SPI_FLASH_BASE_ADDR)
    );

    __builtin_unreachable();
}