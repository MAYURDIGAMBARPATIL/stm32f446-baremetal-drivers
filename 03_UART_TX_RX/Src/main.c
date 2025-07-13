/**
 * @file    main.c
 * @brief   UART2 communication example: transmit text and receive characters into buffer.
 * @author  Mayur
 * @board   Nucleo STM32F446RE
 */

#include "stm32f446xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* --- Macro Definitions --- */

// RCC
#define GPIOA_EN        (1U << 0)
#define UART2_EN        (1U << 17)

// USART Control Bits
#define CR1_TE          (1U << 3)    // Transmitter enable
#define CR1_RE          (1U << 2)    // Receiver enable
#define CR1_UE          (1U << 13)   // USART enable

#define SR_TXE          (1U << 7)    // Transmit data register empty
#define SR_RXNE         (1U << 5)    // Read data register not empty

// Clocks
#define SYS_FREQ        16000000U
#define APB1_CLK        SYS_FREQ

// Baud rate
#define UART_BAUDRATE   115200U

/* --- Function Prototypes --- */

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);

void uart2_init(void);
void uart2_write(int ch);
char uart2_read(void);
void uart2_write_text(char *text);

/* --- Global Variables --- */

char rx_data[5] = {0};
char buffer[5] = {0};
int i = 0;

int main(void)
{
    /* Initialize UART2 */
    uart2_init();

    /* Send welcome string */
    uart2_write_text("Mayur_patil\r\n");

    /* Receive 5 characters and store into buffer */
    while (1)
    {
        while (i < 5)
        {
            rx_data[i] = uart2_read();
            i++;

            if (i == 5)
            {
                strcpy(buffer, rx_data);     // Copy received data
                memset(rx_data, 0, sizeof(rx_data)); // Clear receive buffer
                i = 0;                        // Reset index
            }
        }
    }
}

/**
 * @brief Configure UART2 TX (PA2) and RX (PA3) using AF7.
 */
void uart2_init(void)
{
    /******** GPIO Configuration for UART2 ********/

    // Enable GPIOA clock
    RCC->AHB1ENR |= GPIOA_EN;

    // Set PA2 and PA3 to alternate function mode (MODER = 0b10)
    GPIOA->MODER &= ~((3U << 4) | (3U << 6)); // Clear MODER2, MODER3
    GPIOA->MODER |=  ((2U << 4) | (2U << 6)); // Set to AF mode

    // Set alternate function AF7 for PA2 (TX) and PA3 (RX)
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12));
    GPIOA->AFR[0] |=  ((7U << 8) | (7U << 12)); // AF7 for USART2

    /******** UART2 Peripheral Configuration ********/

    // Enable USART2 clock
    RCC->APB1ENR |= UART2_EN;

    // Configure baud rate
    uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

    // Enable Transmitter and Receiver
    USART2->CR1 |= (CR1_TE | CR1_RE);

    // Enable UART2
    USART2->CR1 |= CR1_UE;
}

/**
 * @brief Write a single character to USART2.
 */
void uart2_write(int ch)
{
    // Wait until transmit data register is empty
    while (!(USART2->SR & SR_TXE)) {}

    // Send data
    USART2->DR = (ch & 0xFF);
}

/**
 * @brief Read a single character from USART2.
 * @return Received character.
 */
char uart2_read(void)
{
    // Wait until receive data register is not empty
    while (!(USART2->SR & SR_RXNE)) {}

    // Read and return received data
    return USART2->DR;
}

/**
 * @brief Write a null-terminated string over USART2.
 */
void uart2_write_text(char *text)
{
    while (*text)
    {
        uart2_write(*text++);
    }
}

/**
 * @brief Configure USART BRR (baud rate register).
 */
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
    USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);
}

/**
 * @brief Compute baud rate register value.
 * @return BRR value
 */
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
    return (PeriphClk + (BaudRate / 2U)) / BaudRate;
}

