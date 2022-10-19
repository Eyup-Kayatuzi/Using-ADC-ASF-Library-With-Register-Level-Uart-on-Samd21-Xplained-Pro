/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * This is a bare minimum user application template.
 *
 * For documentation of the board, go \ref group_common_boards "here" for a link
 * to the board-specific documentation.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# Basic usage of on-board LED and button
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <string.h> // for memset
#include "SERCOMforUSART.h"

// for adc
struct adc_module adc_instance;#define ADC_SAMPLES 10
uint16_t adc_result_buffer[ADC_SAMPLES];
void adc_complete_callback(const struct adc_module *const module);
void configure_adc(void);
void configure_adc_callbacks(void);

// uint16_t to str for writing adc values to data visualizer
uint8_t str[7] = {0, 0, 0, 0, 0, '\r', '\n'};
uint8_t GelenData[100];
uint8_t i = 0;


int main (void)
{
	system_init();
	delay_init();
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM3; 
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM3_CORE;
	PORT->Group[0].PINCFG[22].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[11].bit.PMUXE = 0x2; // pa22 is config as sercom3 pad[0]
	PORT->Group[0].PINCFG[23].bit.PMUXEN = 1;
	PORT->Group[0].PMUX[11].bit.PMUXO = 0x2; // pa23 is config as sercom3 pad[1]
	uartInit(115200);
	system_interrupt_enable_global();
	system_interrupt_set_priority(SYSTEM_INTERRUPT_MODULE_SERCOM3, SYSTEM_INTERRUPT_PRIORITY_LEVEL_0);
	system_interrupt_set_priority(SYSTEM_INTERRUPT_MODULE_ADC, SYSTEM_INTERRUPT_PRIORITY_LEVEL_1);
	configure_adc();
	configure_adc_callbacks();
	volatile uint32_t deneme = system_cpu_clock_get_hz();
	uint8_t string[] = "Hello World!\r\n";
	uartWrite(string, sizeof(string));
	delay_ms(50);
	while (1) {
		adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_SAMPLES);
	
	}
}

void USART_EYUP(){
	
	GelenData[i] = SERCOM3->USART.DATA.bit.DATA; // This flag is cleared by reading the Data register (DATA) or by disabling the receiver.
	 
	if (GelenData[i] == '\n')
	{
		uartWrite(GelenData + i, 1);
		i = 0;
	}
	else{
		uartWrite(GelenData + i, 1);
		i += 1;
	}
	
}

void adc_complete_callback(const struct adc_module *const module){
	
	sprintf(str,"%d",*adc_result_buffer);
	uartWrite(str, 7);
	memset(str, ' ', 5);
	delay_ms(500);
}

void configure_adc(void)
{
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);
	config_adc.clock_source = GCLK_GENERATOR_2; // for 8 Mhz
	config_adc.gain_factor = ADC_GAIN_FACTOR_DIV2;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV512;
	config_adc.reference = ADC_REFERENCE_INTVCC1;
	config_adc.positive_input = ADC_POSITIVE_INPUT_PIN0;
	config_adc.resolution = ADC_RESOLUTION_10BIT;
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
	
}void configure_adc_callbacks(void)
{
	adc_register_callback(&adc_instance, adc_complete_callback, ADC_CALLBACK_READ_BUFFER);
	adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
}


