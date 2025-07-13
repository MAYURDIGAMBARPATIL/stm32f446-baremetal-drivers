#include "stm32f446xx.h"

/* --- Function Prototypes --- */
void delayMS(int delay);
void PWM_Configure(void);

/* --- Main Function --- */
int main(void)
{
    PWM_Configure();

    while (1)
    {
        // PWM signals generated on:
        // PA5 (TIM2_CH1), PA6 (TIM3_CH1), PB6 (TIM4_CH1), PB14 (TIM12_CH1)
        // You can vary duty cycles by changing CCRx registers
    }
}

/* --- Configure All 4 PWM Outputs --- */
void PWM_Configure(void)
{
    /************** PWM 1: TIM2_CH1 on PA5 (H1) **************/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     // Enable TIM2 clock

    GPIOA->MODER &= ~(3U << (5 * 2));
    GPIOA->MODER |=  (2U << (5 * 2));       // Alternate function mode
    GPIOA->AFR[0] |= (1U << 20);            // AF1 for TIM2_CH1 on PA5

    TIM2->PSC = 16 - 1;                     // 1 MHz timer clock
    TIM2->ARR = 1000 - 1;                   // 1 kHz PWM frequency
    TIM2->CCR1 = 500;                       // 50% Duty Cycle
    TIM2->CCMR1 |= (6U << 4);               // PWM mode 1 on CH1
    TIM2->CCER |= (1U << 0);                // Enable CH1 output
    TIM2->CR1 |= (1U << 0);                 // Enable TIM2

    /************** PWM 2: TIM3_CH1 on PA6 (H2) **************/
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;     // Enable TIM3 clock

    GPIOA->MODER &= ~(3U << (6 * 2));
    GPIOA->MODER |=  (2U << (6 * 2));       // Alternate function mode
    GPIOA->AFR[0] |= (2U << 24);            // AF2 for TIM3_CH1 on PA6

    TIM3->PSC = 16 - 1;
    TIM3->ARR = 1000 - 1;
    TIM3->CCR1 = 500;
    TIM3->CCMR1 |= (6U << 4);
    TIM3->CCER |= (1U << 0);
    TIM3->CR1 |= (1U << 0);

    /************** PWM 3: TIM4_CH1 on PB6 (H3) **************/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // Enable GPIOB clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;     // Enable TIM4 clock

    GPIOB->MODER &= ~(3U << (6 * 2));
    GPIOB->MODER |=  (2U << (6 * 2));       // Alternate function mode
    GPIOB->AFR[0] |= (2U << 24);            // AF2 for TIM4_CH1 on PB6

    TIM4->PSC = 16 - 1;
    TIM4->ARR = 1000 - 1;
    TIM4->CCR1 = 500;
    TIM4->CCMR1 |= (6U << 4);
    TIM4->CCER |= (1U << 0);
    TIM4->CR1 |= (1U << 0);

    /************** PWM 4: TIM12_CH1 on PB14 (FAN) **************/
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;    // Enable TIM12 clock

    GPIOB->MODER &= ~(3U << (14 * 2));
    GPIOB->MODER |=  (2U << (14 * 2));      // Alternate function mode

    // AF9 for TIM12_CH1 on PB14 (AFR[1] bits 24–27)
    GPIOB->AFR[1] &= ~(0xF << (14 * 4 - 32));  // Clear
    GPIOB->AFR[1] |=  (9U << (14 * 4 - 32));   // Set AF9

    TIM12->PSC = 16 - 1;
    TIM12->ARR = 1000 - 1;
    TIM12->CCR1 = 500;
    TIM12->CCMR1 |= (6U << 4);
    TIM12->CCER |= (1U << 0);
    TIM12->CR1 |= (1U << 0);
}

/* --- Blocking Delay in Milliseconds --- */
void delayMS(int delay)
{
    for (; delay > 0; delay--)
        for (int i = 0; i < 3195; i++)
            __NOP(); // Prevent optimization
}

