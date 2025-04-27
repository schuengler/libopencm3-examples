#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include <stdio.h>
#include <stdint.h>

#define DMA_CIRCULAR

// DMA1 request mapping (table 43, STM32F4x reference manual) 
// specifies channel 4 stream 6 for USART2_TX
#define DMA_DEMO_STREAM DMA_STREAM6
#define DMA_DEMO_STREAM_IRQ NVIC_DMA1_STREAM6_IRQ
#define DMA_DEMO_CHANNEL DMA_SxCR_CHSEL_4

typedef uint8_t DmaDataType;

///
/// Function declarations
///
static void clock_setup(void);
static void dma_setup(DmaDataType* data, size_t size);
static void usart2_setup(void);
static void gpio_setup(void);
static void setup_periphery(DmaDataType* data);
static void wait(uint32_t cycles);

///
/// Function definitions
///
/// @brief Setup clocks for gpios and usart
void clock_setup(void)
{
	// gpio
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);

	// usart
	rcc_periph_clock_enable(RCC_USART2);

	// dma
	rcc_periph_clock_enable(RCC_DMA1);
}

/// @brief Setup dma
/// @param data Address of data source
/// @param size Number of data blocks
void dma_setup(DmaDataType* data, size_t size)
{
	// enable interrupt
	nvic_enable_irq(DMA_DEMO_STREAM_IRQ);

	dma_stream_reset(DMA1, DMA_DEMO_STREAM);
	dma_disable_stream(DMA1, DMA_DEMO_STREAM);

	dma_set_priority(DMA1, DMA_DEMO_STREAM, DMA_SxCR_PL_HIGH);
	dma_set_memory_size(DMA1, DMA_DEMO_STREAM, DMA_SxCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_DEMO_STREAM, DMA_SxCR_PSIZE_8BIT);
	dma_disable_memory_increment_mode(DMA1, DMA_DEMO_STREAM);

	// dma mode
#ifdef DMA_CIRCULAR
	dma_enable_circular_mode(DMA1, DMA_DEMO_STREAM);
#else
	dma_enable_direct_mode(DMA1, DMA_DEMO_STREAM);
#endif
	dma_set_transfer_mode(DMA1, DMA_DEMO_STREAM, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

	// dest address (usart)
	dma_set_peripheral_address(DMA1, DMA_DEMO_STREAM, (uint32_t)&USART_DR(USART2));
	
	// src address 
	dma_set_memory_address(DMA1, DMA_DEMO_STREAM, (uint32_t)data);
	dma_set_number_of_data(DMA1, DMA_DEMO_STREAM, size);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_DEMO_STREAM);

	dma_channel_select(DMA1, DMA_DEMO_STREAM, DMA_SxCR_CHSEL_4);
	dma_enable_stream(DMA1, DMA_DEMO_STREAM);

	usart_enable_tx_dma(USART2);
}

/// @brief Interrupt handler for dma1 stream 6
void dma1_stream6_isr(void)
{
	if (dma_get_interrupt_flag(DMA1, DMA_DEMO_STREAM, DMA_TCIF)) 
	{
		dma_clear_interrupt_flags(DMA1, DMA_DEMO_STREAM, DMA_TCIF);
		
		// toggle led
		gpio_toggle(GPIOA, GPIO5);
	}
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
	usart_enable_tx_dma(USART2);
	usart_enable(USART2);

	nvic_set_priority(DMA_DEMO_STREAM_IRQ, 0);
	nvic_enable_irq(DMA_DEMO_STREAM_IRQ);
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
void setup_periphery(DmaDataType* data)
{
	clock_setup();
	gpio_setup();
	usart2_setup();
	dma_setup(data, 1);
}

/// @brief Main function
/// @return 0
int main(void)
{
	uint8_t counter = 0;
	uint8_t data = 0 + '0'; 

	setup_periphery(&data);

	while (1)
	{
#ifdef DMA_CIRCULAR
		// nothing to do
#else
		dma_setup(&data, 1);
#endif

		// count from 0 to 9
		counter = (counter == 9) ? 0 : counter + 1;

		// set data (value pointed by dma src address)
		data = counter + '0';

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