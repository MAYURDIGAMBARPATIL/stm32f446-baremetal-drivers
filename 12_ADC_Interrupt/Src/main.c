#include "stm32f446xx.h"

/* --- Macro Definitions --- */
#define GPIOAEN         (1U << 0)   // RCC AHB1: GPIOA clock enable
#define ADC1EN          (1U << 8)   // RCC APB2: ADC1 clock enable

#define SR_EOC          (1U << 1)   // ADC SR: End of conversion flag
#define CR1_EOCIE       (1U << 5)   // ADC CR1: End of conversion interrupt enable
#define CR2_ADON        (1U << 0)   // ADC CR2: ADC enable
#define CR2_CONT        (1U << 1)   // ADC CR2: Continuous conversion
#define CR2_SWSTART     (1U << 30)  // ADC CR2: Start conversion

#define ADC_CH1         1           // Channel 1 corresponds to PA1
#define ADC_SEQ_LEN_1   0x00        // 1 conversion in sequence

/* --- Global Variables --- */
volatile uint32_t sensor_value = 0;

/* --- Function Prototypes --- */
void ADC1_PA1_Init_Interrupt(void);
void ADC1_Start_Conversion(void);
uint32_t ADC1_Read(void);
static void ADC1_Callback(void);

/* --- Main Function --- */
int main(void)
{
    ADC1_PA1_Init_Interrupt();
    ADC1_Start_Conversion();

    while (1)
    {
        // Main loop can do other tasks, sensor_value is updated in ISR
    }
}

/* --- ADC Initialization for PA1 (Channel 1) with Interrupt --- */
void ADC1_PA1_Init_Interrupt(void)
{
    /** Enable clock access to GPIOA */
    RCC->AHB1ENR |= GPIOAEN;

    /** Set PA1 as analog mode */
    GPIOA->MODER |= (3U << (1 * 2));  // MODER1[1:0] = 11 (Analog)

    /** Enable clock for ADC1 */
    RCC->APB2ENR |= ADC1EN;

    /** Enable EOC interrupt */
    ADC1->CR1 |= CR1_EOCIE;

    /** Enable ADC1 interrupt in NVIC */
    NVIC_EnableIRQ(ADC_IRQn);

    /** Set ADC channel sequence */
    ADC1->SQR3 = ADC_CH1;         // 1st conversion in regular sequence = channel 1
    ADC1->SQR1 = ADC_SEQ_LEN_1;   // Only 1 conversion

    /** Enable ADC module */
    ADC1->CR2 |= CR2_ADON;
}

/* --- Start ADC Continuous Conversion --- */
void ADC1_Start_Conversion(void)
{
    ADC1->CR2 |= CR2_CONT;        // Enable continuous mode
    ADC1->CR2 |= CR2_SWSTART;     // Start conversions
}

/* --- ADC Read Function (used by callback) --- */
uint32_t ADC1_Read(void)
{
    // Wait until conversion is complete
    while (!(ADC1->SR & SR_EOC));

    // Read and return data
    return ADC1->DR;
}

/* --- ADC Callback (called from ISR) --- */
static void ADC1_Callback(void)
{
    sensor_value = ADC1_Read();
}

/* --- ADC Interrupt Handler --- */
void ADC_IRQHandler(void)
{
    if (ADC1->SR & SR_EOC)
    {
        ADC1->SR &= ~SR_EOC;     // Clear interrupt flag
        ADC1_Callback();         // Handle data read
    }
}

