#pragma once

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

static void adxl345_cs_low(void)  { gpio_clear(GPIOA, GPIO4); }
static void adxl345_cs_high(void) { gpio_set(GPIOA, GPIO4); }

//static uint8_t adxl345_read_reg(uint32_t spi, uint8_t reg) 
//{
//    adxl345_cs_low();
//    spi_send(spi, 0x80 | reg);           // Read flag
//    uint8_t val = spi_read(spi);
//    spi_send(spi, 0x00);                 // Dummy write
//    val = spi_read(spi);
//    adxl345_cs_high();
//    return val;
//}

/// @brief Write given register with given value through given spi
/// @param spi 
/// @param reg 
/// @param val 
static void adxl345_write_reg(uint32_t spi, uint8_t reg, uint8_t val)
 {
    adxl345_cs_low();
    spi_send(spi, reg & 0x3F);           // Write op
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
    adxl345_write_reg(spi, 0x2D, 0x08);
}

/// @brief Read xyz accelations through the given spi 
/// @param spi SPI identifier
/// @param x[out] Acceleration in x-diemension
/// @param y[out] Acceleration in y-diemension
/// @param z[out] Acceleration in z-diemension
static void adxl345_read_xyz(uint32_t spi, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t raw[6];

    adxl345_cs_low();
    spi_send(spi, 0xC0 | 0x32); // Read multiple starting at 0x32
    spi_read(spi);

    for (int i = 0; i < 6; i++) {
        spi_send(spi, 0x00);
        raw[i] = spi_read(spi);
    }
    adxl345_cs_high();

    *x = (int16_t)((raw[1] << 8) | raw[0]);
    *y = (int16_t)((raw[3] << 8) | raw[2]);
    *z = (int16_t)((raw[5] << 8) | raw[4]);
}
