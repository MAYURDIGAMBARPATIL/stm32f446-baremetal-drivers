/**
 * @file    main.c
 * @brief   USART2 interrupt-based UART communication with string receive on STM32F446RE.
 * @details RX interrupt is used to receive 5 characters into a buffer, TX is polled.
 * @author  Mayur
 */

#include "stm32f446xx.h"
#include <string.h>

/* === Clock and Peripheral Defines === */
#define GPIOA_EN        (1U << 0)
#define UART2_EN        (1U << 17)

/* === GPIO Defines === */
#define LED_PIN         (1U << 5)   // PA5

/* === USART Defines === */
#define CR1_TE          (1U << 3)
#define CR1_RE          (1U << 2)
#define CR1_UE          (1U << 13)
#define CR1_RXNEIE      (1U << 5)
#define SR_TXE          (1U << 7)
#define SR_RXNE         (1U << 5)

#define SYS_FREQ        16000000U
#define APB1_CLK        SYS_FREQ
#define UART_BAUDRATE   115200U

/* === Function Prototypes === */
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);

void uart2_interrupt_init(void);
void uart2_write_char(int ch);
void uart2_write_text(char *text);
static void uart_callback(void);

/* === Global Buffers === */
char rx_data[5] = {0};
char buffer[5]  = {0};
volatile int i = 0;

/**
 * @brief Main function
 */
int main(void)
{
    /* Enable GPIOA clock */
    RCC->AHB1ENR |= GPIOA_EN;

    /* Configure PA5 as output (LED) */
    GPIOA->MODER &= ~(3U << (5 * 2));
    GPIOA->MODER |=  (1U << (5 * 2));

    /* Initialize UART2 with RX interrupt */
    uart2_interrupt_init();

    /* Main loop: periodically send message */
    while (1)
    {
        uart2_write_text("Mayur_patil\r\n");
        for (volatile int d = 0; d < 100000; d++);  // Software delay
    }
}

/**
 * @brief Initialize UART2 with RX interrupt
 */
void uart2_interrupt_init(void)
{
    /* === GPIO Config for UART2 TX (PA2) and RX (PA3) === */
    RCC->AHB1ENR |= GPIOA_EN;

    // Set PA2 and PA3 to alternate function mode (AF7)
    GPIOA->MODER &= ~((3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA->MODER |=  ((2U << (2 * 2)) | (2U << (3 * 2)));

    GPIOA->AFR[0] &= ~((0xF << (4 * 2)) | (0xF << (4 * 3)));
    GPIOA->AFR[0] |=  ((7U << (4 * 2)) | (7U << (4 * 3))); // AF7 for USART2

    /* === USART2 Config === */
    RCC->APB1ENR |= UART2_EN;

    uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

    // Enable TX, RX, RXNE interrupt
    USART2->CR1 |= (CR1_TE | CR1_RE | CR1_RXNEIE);

    // Enable USART2 interrupt in NVIC
    NVIC_EnableIRQ(USART2_IRQn);

    // Enable USART2 module
    USART2->CR1 |= CR1_UE;
}

/**
 * @brief USART2 interrupt handler (RX interrupt)
 */
void USART2_IRQHandler(void)
{
    if (USART2->SR & SR_RXNE)
    {
        uart_callback();
    }
}

/**
 * @brief Called on every received character via RXNE interrupt
 */
static void uart_callback(void)
{
    rx_data[i] = USART2->DR;  // Read received character

    i++;
    if (i == 5)
    {
        strcpy(buffer, rx_data);        // Copy to persistent buffer
        memset(rx_data, 0, sizeof(rx_data));  // Clear receive array
        i = 0;
    }
}

/**
 * @brief Send a single character over USART2 (polling mode)
 */
void uart2_write_char(int ch)
{
    while (!(USART2->SR & SR_TXE)) {}   // Wait until TXE is set
    USART2->DR = (ch & 0xFF);
}

/**
 * @brief Send a string over USART2
 */
void uart2_write_text(char *text)
{
    while (*text)
    {
        uart2_write_char(*text++);
    }
}

/**
 * @brief Set USART2 baud rate
 */
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
    USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);
}

/**
 * @brief Compute USART2 BRR value
 */
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
    return ((PeriphClk + (BaudRate / 2U)) / BaudRate);
}

