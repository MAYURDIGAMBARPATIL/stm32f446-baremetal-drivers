/**
 * @file    main.c
 * @brief   Turn on LED (PA5) when button (PC13) is not pressed on Nucleo-F446RE.
 *          Uses GPIO input and output in bare-metal style with CMSIS.
 * @author  Mayur
 */

#include "stm32f446xx.h"  // Device header file (CMSIS)

#define GPIOA_EN        (1U << 0)    // Enable clock for GPIOA
#define GPIOC_EN        (1U << 2)    // Enable clock for GPIOC

#define PIN5            (1U << 5)    // PA5 (LED)
#define PIN13           (1U << 13)   // PC13 (User Button)

#define LED_PIN         PIN5
#define BTN_PIN         PIN13

int main(void)
{
    /* 1. Enable clock access to GPIOA and GPIOC */
    RCC->AHB1ENR |= GPIOA_EN;
    RCC->AHB1ENR |= GPIOC_EN;

    /* 2. Configure PA5 as output (MODER5 = 0b01) */
    GPIOA->MODER &= ~(3U << (5 * 2));    // Clear mode bits for pin 5
    GPIOA->MODER |=  (1U << (5 * 2));    // Set to output mode

    /* 3. Configure PC13 as input (MODER13 = 0b00) */
    GPIOC->MODER &= ~(3U << (13 * 2));   // Clear mode bits for pin 13

    /* 4. Infinite loop to monitor button state and control LED */
    while (1)
    {
        // Note: PC13 button is active-low (pressed = 0)
        if (GPIOC->IDR & BTN_PIN)
        {
            // Button not pressed → Turn off LED (reset PA5)
            GPIOA->BSRR = (1U << (5 + 16));  // Reset bit (bit 21)
        }
        else
        {
            // Button pressed → Turn on LED (set PA5)
            GPIOA->BSRR = LED_PIN;           // Set bit 5
        }
    }

    // Code never reaches here
    return 0;
}

