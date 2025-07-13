#include "stm32f446xx.h"

/* --- Configuration Macros --- */
#define DATA_SIZE       16
#define SPI1EN          (1 << 12)   // SPI1 clock enable bit in RCC->APB2ENR
#define GPIOBEN         (1 << 1)    // GPIOB clock enable bit in RCC->AHB1ENR

/* --- Function Prototypes --- */
void GPIOB_SPI1_Init(void);
void SPI1_Master_Init(void);
void SPI1_Transmit(uint8_t *data, uint16_t size);
void SPI1_Receive(uint8_t *data, uint16_t size);
void ChipSelect_Enable(uint8_t pin);
void ChipSelect_Disable(uint8_t pin);
void Error_Handler(void);

/* Optional Test Data */
// uint8_t txData[DATA_SIZE] = {0x01, 0x02, ..., 0x10};
// uint8_t rxData[DATA_SIZE] = {0};

/* --- Main Function --- */
int main(void)
{
    GPIOB_SPI1_Init();
    SPI1_Master_Init();

    ChipSelect_Enable(3);             // Enable chip select on PB3 (example)
    SPI1_Transmit((uint8_t *)"\x0C", 1);  // Transmit a single byte (0x0C)
    ChipSelect_Disable(3);            // Disable chip select

    while (1); // Infinite loop
}

/* --- GPIO Initialization for SPI1 on GPIOB --- */
void GPIOB_SPI1_Init(void)
{
    // Enable GPIOB clock
    RCC->AHB1ENR |= GPIOBEN;

    // Configure PB3 (SCK), PB4 (MISO), PB5 (MOSI) as alternate function (AF5)
    GPIOB->MODER &= ~((3U << 6) | (3U << 8) | (3U << 10));   // Clear MODER bits
    GPIOB->MODER |=  ((2U << 6) | (2U << 8) | (2U << 10));   // Set to AF mode

    // Set AF5 (SPI1) for PB3, PB4, PB5
    GPIOB->AFR[0] &= ~((0xF << 12) | (0xF << 16) | (0xF << 20)); // Clear bits
    GPIOB->AFR[0] |=  ((5U << 12) | (5U << 16) | (5U << 20));    // AF5

    // Configure PB0, PB1, PB2 as general purpose output for Chip Select (CS)
    GPIOB->MODER &= ~((3U << 0) | (3U << 2) | (3U << 4));  // Clear MODER bits
    GPIOB->MODER |=  ((1U << 0) | (1U << 2) | (1U << 4));  // Set as output
}

/* --- SPI1 Initialization as Master (Mode 0) --- */
void SPI1_Master_Init(void)
{
    // Enable SPI1 clock
    RCC->APB2ENR |= SPI1EN;

    // Configure SPI1: 
    // CR1:
    // - Bidirectional disabled (full duplex)
    // - Master mode
    // - Baud rate = fPCLK/16 (BR[2:0] = 0b011)
    // - CPOL = 0, CPHA = 0 (Mode 0)
    // - DFF = 0 (8-bit data)
    // - SSM = 1, SSI = 1 (Software NSS management)
    SPI1->CR1 = (1 << 2) |  // Master mode
                (3 << 3) |  // Baud rate (fPCLK/16)
                (1 << 8) |  // SSI = 1
                (1 << 9) |  // SSM = 1
                (1 << 6);   // SPE = SPI enable

    // CR2: default (Motorola frame format)
    SPI1->CR2 = 0;
}

/* --- Chip Select Control (Active Low) --- */
void ChipSelect_Enable(uint8_t pin)
{
    GPIOB->ODR &= ~(1U << pin);
}

void ChipSelect_Disable(uint8_t pin)
{
    GPIOB->ODR |= (1U << pin);
}

/* --- SPI1 Transmit Function --- */
void SPI1_Transmit(uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        // Wait for TXE (Transmit buffer empty)
        while (!(SPI1->SR & SPI_SR_TXE));

        // Write data to the data register
        SPI1->DR = data[i];

        // Wait until RXNE (Receive buffer not empty) to clear it
        while (!(SPI1->SR & SPI_SR_RXNE));
        (void)SPI1->DR;
    }

    // Wait until not busy (BSY cleared)
    while (SPI1->SR & SPI_SR_BSY);
}

/* --- SPI1 Receive Function --- */
void SPI1_Receive(uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        // Wait for TXE
        while (!(SPI1->SR & SPI_SR_TXE));

        // Send dummy byte to initiate SPI clock
        SPI1->DR = 0xFF;

        // Wait for RXNE
        while (!(SPI1->SR & SPI_SR_RXNE));

        // Read received data
        data[i] = SPI1->DR;
    }

    // Wait until not busy
    while (SPI1->SR & SPI_SR_BSY);
}

/* --- Simple Error Handler --- */
void Error_Handler(void)
{
    while (1)
    {
        // Hang in infinite loop (optional: toggle LED here)
    }
}

