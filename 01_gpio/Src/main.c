/**
 * @file    main.c
 * @brief   Toggle LED on PA5 using bare-metal STM32F446RE with CMSIS.
 * @author  Mayur
 * @note    This example toggles an LED connected to PA5 (on-board LED for Nucleo-F446RE)
 */

#include "stm32f446xx.h"  // CMSIS header for STM32F446RE

// Define bit position for GPIOA clock enable
#define GPIOA_EN        (1U << 0)

// Define bit positions for GPIOA Pin 5
#define PIN5            (1U << 5)
#define PIN5_MODE_BIT   (1U << 10)

int main(void)
{
    /* Enable clock access to GPIOA */
    RCC->AHB1ENR |= GPIOA_EN;

    /* Set PA5 as output mode */
    // First clear MODER bits for pin 5 (bits 10 and 11)
    GPIOA->MODER &= ~(3U << 10);
    // Set bit 10 to '1' and bit 11 to '0' => Output mode (0b01)
    GPIOA->MODER |= PIN5_MODE_BIT;

    while (1)
    {
        /* Toggle PA5 (LED) */
        GPIOA->ODR ^= PIN5;

        /* Simple software delay loop */
        for (volatile int i = 0; i < 1000000; i++) {
            // No operation
        }
    }

    // This point is never reached
    return 0;
}

