#include "stm32f446xx.h"

/* --- Macro Definitions --- */
#define TIM2EN      (1U << 0)    // RCC APB1ENR: TIM2 clock enable
#define GPIOAEN     (1U << 0)    // RCC AHB1ENR: GPIOA clock enable

#define CR1_CEN     (1U << 0)    // TIM2 CR1: Counter enable
#define DIER_UIE    (1U << 0)    // TIM2 DIER: Update interrupt enable
#define SR_UIF      (1U << 0)    // TIM2 SR: Update interrupt flag

#define LED_PIN     (1U << 5)    // GPIOA Pin 5 (PA5)

volatile int var = 0;

/* --- Function Prototypes --- */
void TIM2_Init_1Hz_Interrupt(void);
void GPIOA_LED_Init(void);

/* --- Main Function --- */
int main(void)
{
    GPIOA_LED_Init();               // Initialize PA5 as output for LED
    TIM2_Init_1Hz_Interrupt();      // Setup TIM2 to generate 1Hz interrupts

    while (1)
    {
        // Main loop can perform other tasks (non-blocking)
        // LED is toggled inside the TIM2 interrupt
    }
}

/* --- TIM2 Configuration for 1Hz Interrupt --- */
void TIM2_Init_1Hz_Interrupt(void)
{
    // 1. Enable TIM2 clock
    RCC->APB1ENR |= TIM2EN;

    // 2. Set prescaler and auto-reload for 1Hz: 16MHz / 1600 = 10kHz, ARR = 10000 -> 1Hz
    TIM2->PSC = 1600 - 1;       // Prescaler
    TIM2->ARR = 10000 - 1;      // Auto-reload

    // 3. Reset counter
    TIM2->CNT = 0;

    // 4. Enable update interrupt
    TIM2->DIER |= DIER_UIE;

    // 5. Enable TIM2 IRQ in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);

    // 6. Enable TIM2 counter
    TIM2->CR1 |= CR1_CEN;
}

/* --- GPIOA Configuration for PA5 Output (LED) --- */
void GPIOA_LED_Init(void)
{
    RCC->AHB1ENR |= GPIOAEN;                // Enable GPIOA clock

    GPIOA->MODER &= ~(3U << (5 * 2));       // Clear mode bits for PA5
    GPIOA->MODER |=  (1U << (5 * 2));       // Set PA5 to output mode
}

/* --- TIM2 Interrupt Handler --- */
void TIM2_IRQHandler(void)
{
    // Check update interrupt flag (should always be set here)
    if (TIM2->SR & SR_UIF)
    {
        TIM2->SR &= ~SR_UIF;        // Clear interrupt flag (important!)
        GPIOA->ODR ^= LED_PIN;      // Toggle LED on PA5
        var++;                      // Optional counter for debug
    }
}

