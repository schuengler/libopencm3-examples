#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include <stdio.h>
#include <stdint.h>

///
/// Function declarations
///
static void clock_setup(void);
static void usart2_setup(void);
static void gpio_setup(void);
static void setup_periphery(void);
static void wait(uint32_t cycles);

///
/// Function definitions
///
/// @brief Setup clocks for gpios and usart
void clock_setup(void)
{
	// gpio
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOA);

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

/// @brief Setup periphery
void setup_periphery(void)
{
	clock_setup();
	gpio_setup();
	usart2_setup();
}

/// @brief Main function
/// @return 0
int main(void)
{
	uint8_t counter = 0;

	setup_periphery();

	while (1)
	{
		// toggle led
		gpio_toggle(GPIOA, GPIO5);

		// transmit usart
		usart_send_blocking(USART2, counter + '0');

		// count from 0 to 9
		counter = (counter == 9) ? 0 : counter + 1;

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