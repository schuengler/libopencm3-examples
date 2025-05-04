#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

#include "adxl345.h"

#define ADXL_ADDRESS (0x53)

///
/// Function declarations
///
static void clock_setup(void);
static void usart2_setup(void);
static void spi1_setup(void);
static void gpio_setup(void);
static void setup_periphery(void);
static void wait(uint32_t cycles);
static void usart2_printf(const char *fmt, ...);

///
/// Function definitions
///
/// @brief Setup clocks for gpios and usart
void clock_setup(void)
{
	// gpio
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);

	// i2c
	//rcc_periph_clock_enable(RCC_I2C1);

	// usart
	rcc_periph_clock_enable(RCC_USART2);
}

/// @brief Setup usart2
void usart2_setup(void)
{
	// setup usart
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	// enable usart
	usart_enable(USART2);
}

/// @brief Setup gpios
void gpio_setup(void)
{
	// gpio A5 (onboard led)
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

	// gpio A2 
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
}

/// @brief Setup spi1
void spi1_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_SPI1);

    // SPI1: PA5=SCK, PA6=MISO, PA7=MOSI, PA4=CS
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

    // CS: PA4 als GPIO output
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
    gpio_set(GPIOA, GPIO4); // CS high (deaktiviert)

    //spi_reset(SPI1);
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_16,
                    SPI_CR1_CPOL, SPI_CR1_CPHA,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);
    spi_enable(SPI1);
}

/// @brief 
/// @param fmt 
/// @param  
void usart2_printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    for (char *p = buf; *p; p++)
	{
        if (*p == '\n') usart_send_blocking(USART2, '\r');
        usart_send_blocking(USART2, *p);
    }
}

/// @brief Setup periphery
void setup_periphery(void)
{
	clock_setup();
	gpio_setup();
	spi1_setup();
	usart2_setup();

	// sensor
	adxl345_init(SPI1);
}

/// @brief Main function
/// @return 0
int main(void)
{
	setup_periphery();

	int16_t x, y, z;
	while (1)
	{
		// toggle led
		gpio_toggle(GPIOA, GPIO5);

		adxl345_read_xyz(SPI1, &x, &y, &z);

		usart2_printf("X=%4d, Y=%4d, Z=%4d\n", x, y, z);

		wait(1000000);
	}

	return 0;
}

/// @brief Wait a given amount of cycles
void wait(uint32_t cycles)
{
	uint32_t countCycles = 0;
	for (countCycles = 0; countCycles < cycles; countCycles++) 
	{
		__asm__("nop");
	}
}