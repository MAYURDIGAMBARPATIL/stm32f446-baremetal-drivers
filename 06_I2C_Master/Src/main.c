/**
 * @file    main.c
 * @brief   STM32F446RE I2C1 Master communication using PB8 (SCL) and PB9 (SDA).
 * @details Sends and receives a byte using I2C in polling mode with CMSIS support.
 * @author  Mayur
 */

#include "stm32f446xx.h"

/* === Definitions === */
#define GPIOBEN           (1U << 1)
#define I2C1EN            (1U << 21)

#define I2C_WRITE         0
#define I2C_READ          1

#define SLAVE_ADDR        0x12  // 7-bit slave address

/* === Function Prototypes === */
void I2C1_GPIO_Init(void);
void I2C1_Master_Init(void);
void I2C1_Generate_Start(void);
void I2C1_Send_Address(uint8_t address, uint8_t direction);
void I2C1_Write_Data(uint8_t data);
uint8_t I2C1_Read_Data(void);
void delay_ms(int ms);

/* === Global Variables === */
uint8_t data_received = 0;

int main(void)
{
    I2C1_GPIO_Init();       // Configure PB8/PB9 for I2C
    I2C1_Master_Init();     // Configure I2C1 peripheral

    /* === Transmit data === */
    I2C1_Generate_Start();
    I2C1_Send_Address(SLAVE_ADDR, I2C_WRITE);
    I2C1_Write_Data(25);

    /* === Read data === */
    I2C1_Generate_Start();
    I2C1_Send_Address(SLAVE_ADDR, I2C_READ);
    data_received = I2C1_Read_Data();

    while (1)
    {
        // Loop forever
    }
}

/**
 * @brief Configure PB8 (SCL) and PB9 (SDA) as I2C1 alternate function.
 */
void I2C1_GPIO_Init(void)
{
    // Enable clock to GPIOB
    RCC->AHB1ENR |= GPIOBEN;

    // Set PB8 & PB9 to alternate function mode (AF4)
    GPIOB->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOB->MODER |=  ((2U << (8 * 2)) | (2U << (9 * 2)));

    // Output type: open-drain
    GPIOB->OTYPER |= ((1U << 8) | (1U << 9));

    // High speed for both
    GPIOB->OSPEEDR |= ((3U << (8 * 2)) | (3U << (9 * 2)));

    // Pull-up resistors
    GPIOB->PUPDR &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOB->PUPDR |=  ((1U << (8 * 2)) | (1U << (9 * 2)));  // Pull-up

    // Select AF4 (I2C1) for PB8 and PB9
    GPIOB->AFR[1] &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
    GPIOB->AFR[1] |=  ((4U << ((8 - 8) * 4)) | (4U << ((9 - 8) * 4)));
}

/**
 * @brief Initialize I2C1 in master mode (standard mode, 100 kHz).
 */
void I2C1_Master_Init(void)
{
    // Enable clock for I2C1
    RCC->APB1ENR |= I2C1EN;

    // Reset I2C1 peripheral
    I2C1->CR1 |= (1U << 15);
    I2C1->CR1 &= ~(1U << 15);

    // Set peripheral clock frequency (must match APB1 clock in MHz)
    I2C1->CR2 = 16;  // 16 MHz

    // Set clock control register (CCR) for 100kHz SCL
    I2C1->CCR = 80;

    // Set maximum rise time
    I2C1->TRISE = 17;

    // Enable I2C1 peripheral
    I2C1->CR1 |= (1U << 0);
}

/**
 * @brief Generate START condition and wait for SB flag.
 */
void I2C1_Generate_Start(void)
{
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
}

/**
 * @brief Send 7-bit address with direction (read/write).
 * @param address   7-bit slave address.
 * @param direction 0 = write, 1 = read.
 */
void I2C1_Send_Address(uint8_t address, uint8_t direction)
{
    uint8_t addr_byte = (address << 1) | (direction & 0x1);
    I2C1->DR = addr_byte;

    // Wait until address is acknowledged
    while (!(I2C1->SR1 & I2C_SR1_ADDR));

    // Clear ADDR flag by reading SR1 then SR2
    volatile uint32_t temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void)temp;
}

/**
 * @brief Send one byte to slave.
 */
void I2C1_Write_Data(uint8_t data)
{
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;

    while (!(I2C1->SR1 & I2C_SR1_BTF));  // Wait for byte transfer finished
}

/**
 * @brief Read one byte from slave and generate NACK.
 * @return Received byte.
 */
uint8_t I2C1_Read_Data(void)
{
    // Disable ACK for 1-byte reception
    I2C1->CR1 &= ~I2C_CR1_ACK;

    // Wait until RXNE is set (data received)
    while (!(I2C1->SR1 & I2C_SR1_RXNE));

    // Generate STOP condition
    I2C1->CR1 |= I2C_CR1_STOP;

    return I2C1->DR;
}

/**
 * @brief Simple delay (blocking, not accurate).
 */
void delay_ms(int ms)
{
    for (volatile int i = 0; i < ms * 3195; i++);
}

