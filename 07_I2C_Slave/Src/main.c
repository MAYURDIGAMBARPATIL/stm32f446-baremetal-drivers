/**
 * @file    main.c
 * @brief   STM32F446RE I2C1 Slave mode using interrupts (PB8=SCL, PB9=SDA)
 * @details Receives and sends a byte when addressed by an I2C master.
 * @author  Mayur
 */

#include "stm32f446xx.h"

/* === Function Prototypes === */
void I2C1_Init(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);

/* === Global Variables === */
volatile uint8_t rxBuffer[1];
volatile uint8_t txBuffer[1] = {0xAB};
volatile uint8_t rxIndex = 0;

int main(void)
{
    I2C1_Init();  // Initialize I2C1 in slave mode

    while (1)
    {
        // Place for main-loop logic (optional)
        // You can process rxBuffer or update txBuffer here
    }
}

/**
 * @brief Initialize I2C1 peripheral in slave mode with interrupts.
 */
void I2C1_Init(void)
{
    /* === Enable Clocks === */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable GPIOB clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;   // Enable I2C1 clock

    /* === Configure PB8 (SCL) & PB9 (SDA) as Alternate Function AF4 === */
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOB->MODER |=  (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);  // AF mode

    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);          // Open-drain

    GPIOB->AFR[1] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));       // Clear
    GPIOB->AFR[1] |=  ((4U << (0 * 4)) | (4U << (1 * 4)));         // AF4 for PB8, PB9

    /* === Configure I2C1 as Slave === */
    I2C1->CR1 &= ~I2C_CR1_PE;             // Disable I2C1 before configuring

    I2C1->OAR1 = (0x12 << 1);             // 7-bit slave address (0x12)
    I2C1->OAR1 |= I2C_OAR1_ADDMODE;       // 7-bit address mode
    I2C1->CR1 |= I2C_CR1_ACK;             // Enable ACK for communication

    I2C1->CR2 = 16;                       // APB1 clock in MHz (16 MHz)
    I2C1->CCR = 80;                       // Standard Mode, 100kHz
    I2C1->TRISE = 17;                     // Max rise time (per datasheet)

    /* === Enable I2C1 Peripheral and Interrupts === */
    I2C1->CR1 |= I2C_CR1_PE;             // Enable I2C1
    I2C1->CR2 |= (1 << 9);               // Event interrupt enable
    I2C1->CR2 |= (1 << 8);               // Error interrupt enable

    NVIC_EnableIRQ(I2C1_EV_IRQn);        // NVIC: Enable event IRQ
    NVIC_EnableIRQ(I2C1_ER_IRQn);        // NVIC: Enable error IRQ
}

/**
 * @brief I2C1 Event Interrupt Handler
 */
void I2C1_EV_IRQHandler(void)
{
    // Address matched
    if (I2C1->SR1 & I2C_SR1_ADDR)
    {
        (void)I2C1->SR1;     // Clear ADDR by reading SR1
        (void)I2C1->SR2;     // Followed by SR2
    }

    // Master writing to slave (RXNE)
    if (I2C1->SR1 & I2C_SR1_RXNE)
    {
        rxBuffer[rxIndex++] = I2C1->DR;  // Read data
    }

    // Master reading from slave (TXE)
    if (I2C1->SR1 & I2C_SR1_TXE)
    {
        I2C1->DR = txBuffer[0];  // Send fixed byte (0xAB) or dynamic response
    }

    // STOP condition received from master
    if (I2C1->SR1 & I2C_SR1_STOPF)
    {
        (void)I2C1->SR1;           // Clear STOPF
        I2C1->CR1 |= I2C_CR1_PE;   // Re-enable I2C1
        rxIndex = 0;               // Reset index for next transmission
    }
}

/**
 * @brief I2C1 Error Interrupt Handler
 */
void I2C1_ER_IRQHandler(void)
{
    if (I2C1->SR1 & I2C_SR1_AF)    // NACK received
        I2C1->SR1 &= ~I2C_SR1_AF;

    if (I2C1->SR1 & I2C_SR1_BERR)  // Bus error
        I2C1->SR1 &= ~I2C_SR1_BERR;

    if (I2C1->SR1 & I2C_SR1_ARLO)  // Arbitration lost
        I2C1->SR1 &= ~I2C_SR1_ARLO;

    if (I2C1->SR1 & I2C_SR1_OVR)   // Overrun/Underrun
        I2C1->SR1 &= ~I2C_SR1_OVR;

    if (I2C1->SR1 & I2C_SR1_TIMEOUT) // Timeout
        I2C1->SR1 &= ~I2C_SR1_TIMEOUT;
}

