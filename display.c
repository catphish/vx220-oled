/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "can.h"
#include "font.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define PIN_CAN_SCK 2
#define PIN_CAN_MOSI 3
#define PIN_CAN_MISO 4
#define PIN_CAN_CS 9
#define PIN_CAN_INT 11

#define PIN_DISPLAY_SCK 6
#define PIN_DISPLAY_MOSI 7
#define PIN_DISPLAY_CS 23
#define PIN_DISPLAY_RESET 21
#define PIN_DISPLAY_DC 22

#define SPI_PORT spi0

uint8_t vram[128 * 8];

float cell_max;
float cell_min;
int32_t cell_percent;
uint16_t temp_inv;
uint16_t temp_mot;
uint16_t temp_batt;

float temperature(uint16_t adc) {
  float r = 0.0000000347363427499292f * adc * adc - 0.001025770762903f * adc +
            2.68235340614337f;
  float t = log(r) * -30.5280964239816f + 95.6841501312447f;
  return t;
}
void pixel(uint16_t x, uint16_t y, uint16_t state) {
  x = 63 - x;
  uint16_t xx = x / 8;
  uint16_t xxx = x % 8;
  uint16_t segment = xx * 128 + y;
  if (state) {
    vram[segment] |= 1 << xxx;
  } else {
    vram[segment] &= ~(1 << xxx);
  }
}

void print_char(uint16_t x, uint16_t y, uint16_t character) {
  character -= 0x20;
  for (uint16_t n = 0; n < 64; n++) {
    uint16_t xx = x + (n % 8);
    uint16_t yy = y + (n / 8);
    pixel(xx, yy, font8[(character * 8 + (n / 8))] & (1 << (7 - n % 8)));
  }
}

void print_string(uint16_t x, uint16_t y, uint8_t* character) {
  while (*character) {
    print_char(x, y, *character++);
    x += 8;
  }
}

void CAN_reset() {
  gpio_put(PIN_CAN_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET}, 1);
  gpio_put(PIN_CAN_CS, 1);
  busy_wait_us(100);
}

uint8_t CAN_reg_read(uint8_t reg) {
  uint8_t data;
  gpio_put(PIN_CAN_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
  spi_read_blocking(SPI_PORT, 0, &data, 1);
  gpio_put(PIN_CAN_CS, 1);
  return (data);
}

void CAN_reg_write(uint8_t reg, uint8_t val) {
  gpio_put(PIN_CAN_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_WRITE, reg, val}, 3);
  gpio_put(PIN_CAN_CS, 1);
}

void CAN_reg_modify(uint8_t reg, uint8_t mask, uint8_t val) {
  gpio_put(PIN_CAN_CS, 0);
  busy_wait_us(2);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_MODIFY, reg, mask, val}, 4);
  busy_wait_us(2);
  gpio_put(PIN_CAN_CS, 1);
}

void CAN_configure() {
  // Configure speed to 500kbps based on 16MHz Crystal
  // Magic constants from
  // https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
  CAN_reg_write(REG_CNF1, 0x00);
  CAN_reg_write(REG_CNF2, 0xf0);
  CAN_reg_write(REG_CNF3, 0x86);

  // Enable Filters
  CAN_reg_write(REG_RXBnCTRL(0), 0);  // Enable filters, no rollover
  CAN_reg_write(REG_RXBnCTRL(1), 0);
  // Set masks RXM0 and RXM1 to exact match (0x7FF)
  for (int n = 0; n < 2; n++) {
    uint32_t mask = 0x7FF;
    CAN_reg_write(REG_RXMnSIDH(n), mask >> 3);
    CAN_reg_write(REG_RXMnSIDL(n), mask << 5);
    CAN_reg_write(REG_RXMnEID8(n), 0);
    CAN_reg_write(REG_RXMnEID0(n), 0);
  }
  // Set up filters RXF0 and RFX2 to match 2 addresses
  uint32_t addr = 0x014;
  CAN_reg_write(REG_RXFnSIDH(0), addr >> 3);
  CAN_reg_write(REG_RXFnSIDL(0), addr << 5);
  CAN_reg_write(REG_RXFnEID8(0), 0);
  CAN_reg_write(REG_RXFnEID0(0), 0);

  addr = 0x4F1;
  CAN_reg_write(REG_RXFnSIDH(2), addr >> 3);
  CAN_reg_write(REG_RXFnSIDL(2), addr << 5);
  CAN_reg_write(REG_RXFnEID8(2), 0);
  CAN_reg_write(REG_RXFnEID0(3), 0);

  // Set normal operation mode
  CAN_reg_write(REG_CANCTRL, MODE_NORMAL);
}

void CAN_receive(uint8_t n) {
  uint8_t intf = CAN_reg_read(REG_CANINTF);

  if (intf & FLAG_RXnIF(n) == 0) {
    return;
  }

  if (n) {
    uint16_t cell_max_i = CAN_reg_read(REG_RXBnD0(n) + 0) << 8;
    cell_max_i |= CAN_reg_read(REG_RXBnD0(n) + 1);
    cell_max = (float)cell_max_i / 13107.0f;
    uint16_t cell_min_i = CAN_reg_read(REG_RXBnD0(n) + 2) << 8;
    cell_min_i |= CAN_reg_read(REG_RXBnD0(n) + 3);
    cell_min = (float)cell_min_i / 13107.0f;
    uint16_t temp_batt_i = CAN_reg_read(REG_RXBnD0(n) + 4) << 8;
    temp_batt_i |= CAN_reg_read(REG_RXBnD0(n) + 5);
    temp_batt = temperature(temp_batt_i);
    // Calculate voltage as a % between 3.4V and 4.1V
    cell_percent = ((int32_t)cell_min_i - 44564) * 124 / 9175;
  } else {
    temp_inv = CAN_reg_read(REG_RXBnD0(n) + 1) << 8;
    temp_inv |= CAN_reg_read(REG_RXBnD0(n) + 0);
    temp_inv >>= 5;
    temp_mot = CAN_reg_read(REG_RXBnD0(n) + 5) << 8;
    temp_mot |= CAN_reg_read(REG_RXBnD0(n) + 4);
    temp_mot >>= 5;
  }

  CAN_reg_modify(REG_CANINTF, FLAG_RXnIF(n), 0x00);
}

int main() {
  spi_init(SPI_PORT, 500 * 1000);
  gpio_set_function(PIN_DISPLAY_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_DISPLAY_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CAN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CAN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CAN_MISO, GPIO_FUNC_SPI);
  gpio_init(PIN_DISPLAY_CS);
  gpio_set_dir(PIN_DISPLAY_CS, GPIO_OUT);
  gpio_init(PIN_CAN_CS);
  gpio_set_dir(PIN_CAN_CS, GPIO_OUT);
  gpio_init(PIN_DISPLAY_RESET);
  gpio_set_dir(PIN_DISPLAY_RESET, GPIO_OUT);
  gpio_init(PIN_DISPLAY_DC);
  gpio_set_dir(PIN_DISPLAY_DC, GPIO_OUT);

  gpio_init(PIN_CAN_INT);
  gpio_set_dir(PIN_CAN_INT, GPIO_IN);
  gpio_pull_up(PIN_CAN_INT);

  gpio_put(PIN_CAN_CS, 1);
  gpio_put(PIN_DISPLAY_CS, 1);

  CAN_reset();
  CAN_configure();
  // gpio_set_irq_enabled_with_callback(PIN_CAN_INT, GPIO_IRQ_LEVEL_LOW, true,
  //                                   &gpio_callback);

  // Reset display
  gpio_put(PIN_DISPLAY_RESET, 0);
  sleep_ms(100);
  gpio_put(PIN_DISPLAY_RESET, 1);
  sleep_ms(100);

  // Enable and configure display
  gpio_put(PIN_DISPLAY_DC, 0);
  gpio_put(PIN_DISPLAY_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){0xAF}, 1);
  sleep_ms(10);
  spi_write_blocking(SPI_PORT, (uint8_t[]){0x20}, 1);
  sleep_ms(10);
  spi_write_blocking(SPI_PORT, (uint8_t[]){0x00}, 1);
  sleep_ms(10);
  gpio_put(PIN_DISPLAY_CS, 1);

  print_string(0, 0, "Min:3.91");
  print_string(0, 10, "Max:3.94");
  print_string(0, 25, "Inv:100`");
  print_string(0, 35, "Mot:100`");
  print_string(0, 45, "Bat:100`");

  while (1) {
    for (int addr = 0; addr < 1024; addr += 1) {
      CAN_receive(0);
      CAN_receive(1);

      gpio_put(PIN_DISPLAY_DC, 1);
      gpio_put(PIN_DISPLAY_CS, 0);
      spi_write_blocking(SPI_PORT, vram + addr, 1);
      gpio_put(PIN_DISPLAY_CS, 1);
    }
  }
  return 0;
}
