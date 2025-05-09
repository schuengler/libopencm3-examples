#include <libopencmsis/core_cm3.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

#include "adxl345.h"

// DMA1 for usart2
// DMA1 request mapping (table 43, STM32F4x reference manual) 
// specifies channel 4 stream 6 for USART2_TX
#define DMA_USART DMA1
#define DMA_DEMO_STREAM DMA_STREAM6
#define DMA_DEMO_STREAM_IRQ NVIC_DMA1_STREAM6_IRQ
#define DMA_DEMO_CHANNEL DMA_SxCR_CHSEL_4

// DMA2 for spi1
#define DMA_SPI DMA2
#define DMA2_DEMO_STREAM_TX DMA_STREAM3
#define DMA2_DEMO_STREAM_RX DMA_STREAM0
#define DMA2_DEMO_STREAM_IRQ NVIC_DMA2_STREAM0_IRQ
#define DMA2_DEMO_CHANNEL DMA_SxCR_CHSEL_3

typedef char DmaDataType;

///
/// Function declarations
///
static void clock_setup(void);
static void gpio_setup(void);
static void usart2_setup(void);
static void spi1_setup(void);
static void dma1_usart2_setup(DmaDataType* data, size_t size);
static void dma2_spi1_rx_tx_setup(uint8_t *tx, uint8_t *rx, size_t size);
static void adxl345_read_xyz_dma(void);
static void usart2_printf(const char *fmt, ...);
static void setup_periphery(void);
static void wait(uint32_t cycles);

///
/// Globals variables
///
uint32_t startCycles = 0;
uint32_t endCycles = 0;
uint32_t endCyclesTotal = 0;

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

	// spi
    rcc_periph_clock_enable(RCC_SPI1);

	// usart
	rcc_periph_clock_enable(RCC_USART2);

	// dma
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_DMA2);
}

/// @brief Setup gpios
void gpio_setup(void)
{
	// gpio A5 (onboard led)
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

	// usart2
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2);

	// spi1
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
    gpio_mode_setup(CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CS_PIN);
    gpio_set(CS_PORT, CS_PIN);
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

/// @brief Setup dma1 for usart2
/// @param data Address of data source
/// @param size Number of data blocks
void dma1_usart2_setup(DmaDataType* data, size_t size)
{
	// enable interrupt
	nvic_enable_irq(DMA_DEMO_STREAM_IRQ);

	dma_stream_reset(DMA_USART, DMA_DEMO_STREAM);
	dma_disable_stream(DMA_USART, DMA_DEMO_STREAM);

	dma_set_priority(DMA_USART, DMA_DEMO_STREAM, DMA_SxCR_PL_HIGH);
	dma_set_memory_size(DMA_USART, DMA_DEMO_STREAM, DMA_SxCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA_USART, DMA_DEMO_STREAM, DMA_SxCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA_USART, DMA_DEMO_STREAM);

	// dma mode
	dma_enable_direct_mode(DMA_USART, DMA_DEMO_STREAM);
	dma_set_transfer_mode(DMA_USART, DMA_DEMO_STREAM, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

	// dest address (usart)
	dma_set_peripheral_address(DMA_USART, DMA_DEMO_STREAM, (uint32_t)&USART_DR(USART2));
	
	// src address 
	dma_set_memory_address(DMA_USART, DMA_DEMO_STREAM, (uint32_t)data);
	dma_set_number_of_data(DMA_USART, DMA_DEMO_STREAM, size);
	dma_enable_transfer_complete_interrupt(DMA_USART, DMA_DEMO_STREAM);

	dma_channel_select(DMA_USART, DMA_DEMO_STREAM, DMA_DEMO_CHANNEL);
	dma_enable_stream(DMA_USART, DMA_DEMO_STREAM);

	usart_enable_tx_dma(USART2);
}

/// @brief Interrupt handler for dma1 stream 6
void dma1_stream6_isr(void)
{
	if (dma_get_interrupt_flag(DMA_USART, DMA_DEMO_STREAM, DMA_TCIF)) 
	{
		dma_clear_interrupt_flags(DMA_USART, DMA_DEMO_STREAM, DMA_TCIF);

		// benchmark: number used cycles to update display
		endCyclesTotal = dwt_read_cycle_counter();
		volatile uint32_t usedCycles = endCyclesTotal - startCycles; // 35962, 35974, 35949, 35920, 35987
		__asm__("nop");
	}
}

/// @brief Format string with xyz values and send through dma (usart2)
/// @param fmt 
/// @param  
void usart2_printf(const char *fmt, ...) 
{
	const size_t maxStringLength = 24;
    DmaDataType buf[maxStringLength];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

	dma1_usart2_setup(buf, maxStringLength);
}

/// @brief Setup spi1
void spi1_setup(void)
{
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_16,
		SPI_CR1_CPOL, SPI_CR1_CPHA,
		SPI_CR1_DFF_8BIT,
		SPI_CR1_MSBFIRST);
    spi_set_full_duplex_mode(SPI1);
    spi_enable(SPI1);

	nvic_set_priority(DMA2_DEMO_STREAM_IRQ, 0);
	nvic_enable_irq(DMA2_DEMO_STREAM_IRQ);
}

/// @brief Setup dma2 for spi tx and rx
/// @param tx Address of data source to be send through spi
/// @param rx Address of data destination to be received through spi
/// @param size 
void dma2_spi1_rx_tx_setup(uint8_t *tx, uint8_t *rx, size_t size)
{	
	//
    // RX
	//
	// enable interrupt
	nvic_enable_irq(DMA2_DEMO_STREAM_RX);
    dma_stream_reset(DMA_SPI, DMA2_DEMO_STREAM_RX);
    dma_channel_select(DMA_SPI, DMA2_DEMO_STREAM_RX, DMA2_DEMO_CHANNEL);
    dma_set_transfer_mode(DMA_SPI, DMA2_DEMO_STREAM_RX, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA_SPI, DMA2_DEMO_STREAM_RX, (uint32_t)&SPI_DR(SPI1));
    dma_set_memory_address(DMA_SPI, DMA2_DEMO_STREAM_RX, (uint32_t)rx);
    dma_set_number_of_data(DMA_SPI, DMA2_DEMO_STREAM_RX, size);
    dma_enable_memory_increment_mode(DMA_SPI, DMA2_DEMO_STREAM_RX);
    dma_disable_peripheral_increment_mode(DMA_SPI, DMA2_DEMO_STREAM_RX);
    dma_set_memory_size(DMA_SPI, DMA2_DEMO_STREAM_RX, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA_SPI, DMA2_DEMO_STREAM_RX, DMA_SxCR_PSIZE_8BIT);
    dma_set_priority(DMA_SPI, DMA2_DEMO_STREAM_RX, DMA_SxCR_PL_HIGH);
	dma_enable_transfer_complete_interrupt(DMA_SPI, DMA2_DEMO_STREAM_RX);

	//
    // TX
	//
    dma_stream_reset(DMA_SPI, DMA2_DEMO_STREAM_TX);
    dma_channel_select(DMA_SPI, DMA2_DEMO_STREAM_TX, DMA2_DEMO_CHANNEL);
    dma_set_transfer_mode(DMA_SPI, DMA2_DEMO_STREAM_TX, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_peripheral_address(DMA_SPI, DMA2_DEMO_STREAM_TX, (uint32_t)&SPI_DR(SPI1));
    dma_set_memory_address(DMA_SPI, DMA2_DEMO_STREAM_TX, (uint32_t)tx);
    dma_set_number_of_data(DMA_SPI, DMA2_DEMO_STREAM_TX, size);
    dma_enable_memory_increment_mode(DMA_SPI, DMA2_DEMO_STREAM_TX);
    dma_disable_peripheral_increment_mode(DMA_SPI, DMA2_DEMO_STREAM_TX);
    dma_set_memory_size(DMA_SPI, DMA2_DEMO_STREAM_TX, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA_SPI, DMA2_DEMO_STREAM_TX, DMA_SxCR_PSIZE_8BIT);
    dma_set_priority(DMA_SPI, DMA2_DEMO_STREAM_TX, DMA_SxCR_PL_HIGH);
}

/// @brief Global recieve buffer for spi 
uint8_t rx[7];

/// @brief Interrupt handler for dma2 stream 0 -> spi rx
void dma2_stream0_isr(void)
{
	uint32_t startCyclesIsr = 0;
	uint32_t endCyclesIsr = 0;

	startCyclesIsr = dwt_read_cycle_counter();
	if (dma_get_interrupt_flag(DMA_SPI, DMA2_DEMO_STREAM_RX, DMA_TCIF)) 
	{
		dma_clear_interrupt_flags(DMA_SPI, DMA2_DEMO_STREAM_RX, DMA_TCIF);
		dma_clear_interrupt_flags(DMA_SPI, DMA2_DEMO_STREAM_TX, DMA_TCIF);

		spi_disable_rx_dma(SPI1);
		spi_disable_tx_dma(SPI1);
		gpio_set(CS_PORT, CS_PIN);

		int16_t x = (int16_t)((rx[2] << 8) | rx[1]);
		int16_t y = (int16_t)((rx[4] << 8) | rx[3]);
		int16_t z = (int16_t)((rx[6] << 8) | rx[5]);
		usart2_printf("X=%4d, Y=%4d, Z=%4d\n", x, y, z);
	}

	// benchmark: number of cycles used by processing adxl345 data 
	endCyclesIsr = dwt_read_cycle_counter();
	volatile uint32_t usedCycles = endCyclesIsr - startCyclesIsr; // 3853, 3736, 3736, 3736, 3736
	__asm__("nop");
}

/// @brief Read sensor values through dma
void adxl345_read_xyz_dma(void) 
{
    uint8_t tx[7] = { 0x80 | 0x40 | ADXL345_REG_DATA_X0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    dma2_spi1_rx_tx_setup(tx, rx, 7);
    gpio_clear(CS_PORT, CS_PIN);
    dma_enable_stream(DMA_SPI, DMA2_DEMO_STREAM_RX);
    dma_enable_stream(DMA_SPI, DMA2_DEMO_STREAM_TX);
    spi_enable_rx_dma(SPI1);
    spi_enable_tx_dma(SPI1);

	// energy saving
	pwr_set_standby_mode();
	pwr_clear_wakeup_flag();
	// sleep and wait for interrupt
	__WFI();
}

/// @brief Setup periphery
void setup_periphery(void)
{
	clock_setup();
	gpio_setup();
    spi1_setup();
    usart2_setup();

	// Initialize ADXL345 -> put it into measuring mode
	adxl345_init(SPI1);
}

/// @brief Main function
/// @return 0
int main(void) 
{
	setup_periphery();
	dwt_enable_cycle_counter();

    while (1)
	{	
		startCycles = dwt_read_cycle_counter();

        adxl345_read_xyz_dma();

		// benchmark: number of cycles used by a single main iteration
		endCycles = dwt_read_cycle_counter();
		uint32_t usedCycles = endCycles - startCycles; // 590, 590, 590, 590 | with sleep 5405, 5406, 5406, 5406, 5289
		__asm__("nop");

        wait(1000000);
    }
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