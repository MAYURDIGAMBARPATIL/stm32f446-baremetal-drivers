/**
 * @file    main.c
 * @brief   Read analog voltage from pin PA1 using ADC1 on STM32F446RE.
 * @details Configures ADC1 in continuous conversion mode, polls result via software.
 * @author  Mayur
 */

#include "stm32f446xx.h"

/* === Clock and Peripheral Defines === */
#define GPIOA_EN           (1U << 0)
#define ADC1_EN            (1U << 8)

/* === ADC Defines === */
#define ADC_CH1            (1U << 0)      // Channel 1 corresponds to PA1
#define ADC_SEQ_LEN_1      (0x00)         // Sequence length 1 conversion
#define CR2_ADON           (1U << 0)      // ADC ON
#define CR2_CONT           (1U << 1)      // Continuous mode
#define CR2_SWSTART        (1U << 30)     // Start conversion
#define SR_EOC             (1U << 1)      // End of conversion flag

/* === Function Prototypes === */
void pa1_adc_init(void);
void start_conversion(void);
uint32_t adc_read(void);

/* === Global Variable === */
uint32_t adc_result = 0;

/**
 * @brief Main function
 */
int main(void)
{
    pa1_adc_init();         // Initialize ADC1 on PA1
    start_conversion();     // Start continuous conversions

    while (1)
    {
        adc_result = adc_read();  // Read ADC value (12-bit result: 0–4095)
    }
}

/**
 * @brief Initialize PA1 as analog input and configure ADC1
 */
void pa1_adc_init(void)
{
    /* === GPIO Config === */

    // Enable clock access to GPIOA
    RCC->AHB1ENR |= GPIOA_EN;

    // Set PA1 (pin 1) to analog mode: MODER1[1:0] = 0b11
    GPIOA->MODER |= (3U << (1 * 2));  // Set bits 2 and 3

    /* === ADC1 Config === */

    // Enable clock for ADC1
    RCC->APB2ENR |= ADC1_EN;

    // Set channel 1 as the 1st in sequence (SQR3[4:0] = 1)
    ADC1->SQR3 = 1U;

    // Set conversion sequence length to 1 (default is 0x00 = 1 conversion)
    ADC1->SQR1 = ADC_SEQ_LEN_1;

    // Enable ADC1 module
    ADC1->CR2 |= CR2_ADON;
}

/**
 * @brief Start ADC1 conversion in continuous mode
 */
void start_conversion(void)
{
    // Enable continuous conversion
    ADC1->CR2 |= CR2_CONT;

    // Start conversion by setting SWSTART
    ADC1->CR2 |= CR2_SWSTART;
}

/**
 * @brief Wait for ADC conversion to complete and return result
 * @return 12-bit ADC result
 */
uint32_t adc_read(void)
{
    // Wait until End of Conversion (EOC) flag is set
    while (!(ADC1->SR & SR_EOC)) {}

    // Read result from Data Register
    return ADC1->DR;
}

