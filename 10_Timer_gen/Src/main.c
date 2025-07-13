#include "stm32f446xx.h"

/* --- Macro Definitions --- */
#define TIM2EN      (1U << 0)    // Enable TIM2 clock
#define GPIOAEN     (1U << 0)    // Enable GPIOA clock
#define CR1_CEN     (1U << 0)    // TIM2 Control Register: Counter Enable
#define SR_UIF      (1U << 0)    // TIM2 Status Register: Update Interrupt Flag
#define LED_PIN     (1U << 5)    // PA5 (On-board LED on some Nucleo boards)

/* --- Function Prototypes --- */
void TIM2_Init_1Hz(void);
void GPIOA_LED_Init(void);

/* --- Main Function --- */
int main(void)
{
    GPIOA_LED_Init();     // Configure PA5 as output
    TIM2_Init_1Hz();      // Initialize TIM2 to generate 1Hz delay

    while (1)
    {
        // Wait for timer update event (1 second)
        while (!(TIM2->SR & SR_UIF));

        // Clear the update flag
        TIM2->SR &= ~SR_UIF;

        // Toggle the LED
        GPIOA->ODR ^= LED_PIN;
    }
}

/* --- Initialize PA5 as Output (LED) --- */
void GPIOA_LED_Init(void)
{
    RCC->AHB1ENR |= GPIOAEN;            // Enable GPIOA clock

    // Set PA5 as General Purpose Output Mode (MODER5 = 01)
    GPIOA->MODER &= ~(3U << (5 * 2));   // Clear MODER5 bits
    GPIOA->MODER |=  (1U << (5 * 2));   // Set MODER5 to 01 (Output)
}

/* --- Configure TIM2 for 1Hz Time Base --- */
void TIM2_Init_1Hz(void)
{
    RCC->APB1ENR |= TIM2EN;             // Enable TIM2 clock

    TIM2->PSC = 1600 - 1;               // Prescaler: 16 MHz / 1600 = 10 kHz
    TIM2->ARR = 10000 - 1;              // Auto-reload: 10,000 counts = 1 second

    TIM2->CNT = 0;                      // Reset timer counter
    TIM2->CR1 |= CR1_CEN;              // Enable the timer
}

