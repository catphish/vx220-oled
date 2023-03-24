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
int32_t current;

float temperature(uint16_t adc) {
  float r = 0.0000000347363427499292f * adc * adc - 0.001025770762903f * adc + 2.68235340614337f;
  float t = log(r) * -30.5280964239816f + 95.6841501312447f;
  return t;
}

void pixel(uint16_t x, uint16_t y, uint16_t state) {
  if (x > 127) return;
  if (y > 63) return;
  // Rotate 180 degres
  x = 127 - x;
  y = 63 - y;
  uint16_t yy = y / 8;
  uint16_t yyy = y % 8;
  uint16_t segment = yy * 128 + x;
  if (state) {
    vram[segment] |= 1 << yyy;
  } else {
    vram[segment] &= ~(1 << yyy);
  }
}

void print_char(uint16_t x, uint16_t y, uint16_t character) {
  character -= 0x20;
  for (uint16_t n = 0; n < 64; n++) {
    uint16_t xx = x + (n % 8);
    uint16_t yy = y + (n / 8);
    if (xx > 127) return;
    pixel(xx, yy, font8[(character * 8 + (n / 8))] & (1 << (7 - n % 8)));
  }
}

void print_string(uint16_t x, uint16_t y, uint8_t *character) {
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

  // Disable filters, enable rollover
  CAN_reg_write(REG_RXBnCTRL(0), 0x64);
  CAN_reg_write(REG_RXBnCTRL(1), 0x60);

  // Set normal operation mode
  CAN_reg_write(REG_CANCTRL, MODE_NORMAL);
}

void CAN_process(uint32_t id, uint8_t *data) {
  uint8_t buf[128];
  if (id == 0x4F1) {
    // Cell minimum
    uint16_t cell_max_i = (data[0] << 8) | data[1];
    float cell_max = (float)cell_max_i / 13107.0f;
    // Cell maximum
    uint16_t cell_min_i = (data[2] << 8) | data[3];
    float cell_min = (float)cell_min_i / 13107.0f;
    // Battery max temperature
    uint16_t temp_batt_max_i = (data[4] << 8) | data[5];
    float temp_batt_max = temperature(temp_batt_max_i);
    // Battery min temperature
    uint16_t temp_batt_min_i = (data[6] << 8) | data[7];
    float temp_batt_min = temperature(temp_batt_min_i);
    // Print to screen
    sprintf(buf, "Min Cell %.2f    ", cell_min);
    print_string(0, 0, buf);
    sprintf(buf, "Max Cell %.2f    ", cell_max);
    print_string(0, 10, buf);
    sprintf(buf, "Battery  %.0f` %.0f`  ", temp_batt_min, temp_batt_max);
    print_string(0, 56, buf);
    if (current > -20000 && current < 20000) {
      float soc = (float)cell_min_i - 44564.f;
      soc /= 9174.9;
      soc *= 100;
      if (soc < 0) soc = 0;
      if (soc > 100) soc = 100;
      sprintf(buf, "Charge   %.0f%%  ", soc);
      print_string(0, 23, buf);
    }
  }
  if (id == 0x14) {
    // Inverter temperature
    int16_t temp_inv = (data[1] << 8) | data[0];
    temp_inv >>= 5;
    // Motor temperature
    int16_t temp_mot = (data[5] << 8) | data[4];
    temp_mot >>= 5;
    // Print to screen
    sprintf(buf, "Inverter %i`     ", temp_inv);
    print_string(0, 36, buf);
    sprintf(buf, "Motor    %i`     ", temp_mot);
    print_string(0, 46, buf);
  }
  if (id == 0x521) {
    current = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
    // int32_t current_64 = -current / 7500;
    // current_64 = 61 - current_64;
    // for (int n = 2; n < 62; n++) {
    //   if (current_64 < n) {
    //     pixel(114, n, 1);
    //     pixel(115, n, 1);
    //     pixel(116, n, 1);
    //     pixel(117, n, 1);
    //   } else {
    //     pixel(114, n, 0);
    //     pixel(115, n, 0);
    //     pixel(116, n, 0);
    //     pixel(117, n, 0);
    //   }
    // }
  }
}

void CAN_receive() {
  uint8_t intf = CAN_reg_read(REG_CANINTF);
  uint8_t data[8];
  uint32_t id;

  for (int n = 0; n < 2; n++) {
    if (intf & FLAG_RXnIF(n)) {
      id = (CAN_reg_read(REG_RXBnSIDH(n)) << 3) | (CAN_reg_read(REG_RXBnSIDL(n)) >> 5);
      data[0] = CAN_reg_read(REG_RXBnD0(n) + 0);
      data[1] = CAN_reg_read(REG_RXBnD0(n) + 1);
      data[2] = CAN_reg_read(REG_RXBnD0(n) + 2);
      data[3] = CAN_reg_read(REG_RXBnD0(n) + 3);
      data[4] = CAN_reg_read(REG_RXBnD0(n) + 4);
      data[5] = CAN_reg_read(REG_RXBnD0(n) + 5);
      data[6] = CAN_reg_read(REG_RXBnD0(n) + 6);
      data[7] = CAN_reg_read(REG_RXBnD0(n) + 7);
      CAN_reg_modify(REG_CANINTF, FLAG_RXnIF(n), 0x00);
      CAN_process(id, data);
    }
  }
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
  gpio_put(PIN_DISPLAY_RESET, 1);
  sleep_ms(50);

  CAN_reset();
  CAN_configure();

  // Reset display
  gpio_put(PIN_DISPLAY_RESET, 0);
  sleep_ms(50);
  gpio_put(PIN_DISPLAY_RESET, 1);
  sleep_ms(50);

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
  gpio_put(PIN_DISPLAY_DC, 1);

  // pixel(113, 0, 1);
  // pixel(114, 0, 1);
  // pixel(115, 0, 1);
  // pixel(116, 0, 1);
  // pixel(117, 0, 1);
  // pixel(118, 0, 1);
  // pixel(113, 63, 1);
  // pixel(114, 63, 1);
  // pixel(115, 63, 1);
  // pixel(116, 63, 1);
  // pixel(117, 63, 1);
  // pixel(118, 63, 1);
  // for (int n = 0; n < 64; n++) {
  //   pixel(112, n, 1);
  //   pixel(119, n, 1);
  // }
  print_string(0, 0, "Waiting...");

  while (1) {
    for (int addr = 0; addr < 1024; addr += 1) {
      CAN_receive();

      gpio_put(PIN_DISPLAY_CS, 0);
      spi_write_blocking(SPI_PORT, vram + addr, 1);
      gpio_put(PIN_DISPLAY_CS, 1);
    }
  }
  return 0;
}
