#pragma once

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#define CS_PORT GPIOA
#define CS_PIN  GPIO4

#define ADXL345_REG_POWER_CTL  0x2D
#define ADXL345_REG_DATA_X0    0x32
#define ADXL345_MEASURE        0x08

static void adxl345_cs_low(void)  { gpio_clear(CS_PORT, CS_PIN); }
static void adxl345_cs_high(void) { gpio_set(CS_PORT, CS_PIN); }

/// @brief Write given register with given value through given spi
/// @param spi 
/// @param reg 
/// @param val 
static void adxl345_write_reg(uint32_t spi, uint8_t reg, uint8_t val)
 {
    adxl345_cs_low();
    spi_send(spi, reg & 0x3F);
    spi_read(spi);
    spi_send(spi, val);
    spi_read(spi);
    adxl345_cs_high();
}

/// @brief Set measure mode
/// @param spi 
static void adxl345_init(uint32_t spi) 
{
    // POWER_CTL = 0x08 (Measurement mode)
    adxl345_write_reg(spi, ADXL345_REG_POWER_CTL, ADXL345_MEASURE);
}