#include "stm32f446xx.h"

/* --- Function Prototypes --- */
void ADC1_Init(void);
void ADC2_Init(void);
void ADC3_Init(void);

int  ADC1_Read(void);
int  ADC2_Read(void);
int  ADC3_Read(void);

void delayMS(int delay);

/* --- Global Variables --- */
int ADC1_data, ADC2_data, ADC3_data;
double Temp_LM35_1, Temp_LM35_2, Temp_LM35_3;

int main(void)
{
    ADC1_Init();
    ADC2_Init();
    ADC3_Init();

    while (1)
    {
        ADC1_data = ADC1_Read();
        ADC2_data = ADC2_Read();
        ADC3_data = ADC3_Read();

        // Convert ADC value to LM35 temperature in °C
        Temp_LM35_1 = (ADC1_data * (3.3 / 4095.0) * 1000) / 10.0;
        Temp_LM35_2 = (ADC2_data * (3.3 / 4095.0) * 1000) / 10.0;
        Temp_LM35_3 = (ADC3_data * (3.3 / 4095.0) * 1000) / 10.0;

        delayMS(500);  // Optional delay for readability
    }
}

/* --- ADC1 Initialization on PA4 (Channel 4) --- */
void ADC1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;      // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;       // Enable ADC1 clock

    // Configure PA4 as analog mode
    GPIOA->MODER |= (3U << (4 * 2));          // PA4 -> Analog (MODER4 = 11)

    // Configure ADC1: 12-bit resolution, channel 4
    ADC1->CR1 &= ~((1 << 24) | (1 << 25));    // 12-bit resolution
    ADC1->SQR3 = 4;                           // Channel 4 in first rank
    ADC1->CR2 |= ADC_CR2_ADON;                // Turn on ADC1
}

/* --- ADC2 Initialization on PA1 (Channel 1) --- */
void ADC2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;      // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;       // Enable ADC2 clock

    // Configure PA1 as analog mode
    GPIOA->MODER |= (3U << (1 * 2));          // PA1 -> Analog

    ADC2->CR1 &= ~((1 << 24) | (1 << 25));    // 12-bit resolution
    ADC2->SQR3 = 1;                           // Channel 1
    ADC2->CR2 |= ADC_CR2_ADON;                // Turn on ADC2
}

/* --- ADC3 Initialization on PA0 (Channel 0) --- */
void ADC3_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;      // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;       // Enable ADC3 clock

    // Configure PA0 as analog mode
    GPIOA->MODER |= (3U << (0 * 2));          // PA0 -> Analog

    ADC3->CR1 &= ~((1 << 24) | (1 << 25));    // 12-bit resolution
    ADC3->SQR3 = 0;                           // Channel 0
    ADC3->CR2 |= ADC_CR2_ADON;                // Turn on ADC3
}

/* --- Read ADC1 Channel 4 --- */
int ADC1_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;                     // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));                 // Wait until conversion complete
    return ADC1->DR;                                   // Read data
}

/* --- Read ADC2 Channel 1 --- */
int ADC2_Read(void)
{
    ADC2->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC2->SR & ADC_SR_EOC));
    return ADC2->DR;
}

/* --- Read ADC3 Channel 0 --- */
int ADC3_Read(void)
{
    ADC3->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC3->SR & ADC_SR_EOC));
    return ADC3->DR;
}

/* --- Delay in milliseconds --- */
void delayMS(int delay)
{
    for (int i = 0; i < delay * 3195; i++) {
        __NOP(); // Wait
    }
}

