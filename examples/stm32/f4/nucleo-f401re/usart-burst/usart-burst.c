#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include <stdio.h>
#include <stdint.h>

//
// Globals variables
//
uint32_t startCycles = 0;
uint32_t endCycles = 0;

///
/// Function declarations
///
static void clock_setup(void);
static void usart2_setup(void);
static void gpio_setup(void);
static void setup_periphery(void);

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

/// @brief 
/// @param data 
/// @param size 
void setup_random_data(uint8_t* data, size_t size)
{
	for(size_t i = 0; i < size; i++)
	{
		data[i] = (i % 9) + '0';
	}
}


/// @brief Main function
/// @return 0
int main(void)
{
	dwt_enable_cycle_counter();
	startCycles = dwt_read_cycle_counter();

	// large amount of data
	size_t size = 1024;
	uint8_t data[size]; // 1 kB
	setup_random_data(data, size);

	setup_periphery();

	for(size_t i = 0; i < size; i++)
	{
		usart_send_blocking(USART2, data[i]);
	}

	endCycles = dwt_read_cycle_counter();
	uint32_t usedCycles = endCycles - startCycles;
	__asm__("nop");

	while(1) 
	{
		__asm__("nop");
	}

	return 0;
}
