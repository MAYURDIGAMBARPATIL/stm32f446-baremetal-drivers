# STM32F446RE Bare-Metal Drivers 🚀

This repository contains a collection of bare-metal (register-level) embedded C projects for the **STM32F446RE** microcontroller, developed without using STM32 HAL or CMSIS-Driver libraries. It is focused on learning, testing, and demonstrating the use of on-chip peripherals directly via register programming.

---

## 📁 Folder Structure & Contents

| Folder                        | Description                                      |
|------------------------------|--------------------------------------------------|
| `01_gpio`                    | Basic GPIO output (LED blinking)                |
| `02_GPIO_INPUT`              | Reading digital input from a push button        |
| `03_UART_TX_RX`             | UART transmit and receive example               |
| `04_UART_TX_RX_Interrupt`   | UART communication using interrupt handling     |
| `05_ADC_Contineous_Conversion` | Continuous analog data acquisition with ADC   |
| `06_I2C_Master`              | I2C Master communication with start/stop        |
| `07_I2C_Slave`               | I2C Slave mode implementation                   |
| `08_SPI`                     | SPI Master communication                        |
| `09_ADC1_ADC2_ADC3`         | Reading from ADC1, ADC2, ADC3 simultaneously    |
| `10_Timer_gen`              | Basic timer configuration for delay generation  |
| `11_Timer_interrupt_1s`     | Timer interrupt every 1 second                  |
| `12_ADC_Interrupt`          | ADC data acquisition using interrupt            |
| `13_PWM`                    | Generating PWM signals on multiple channels     |
| `14_Empty`                  | Reserved for new experiments                    |
| `Chip_header`               | Common register definitions for STM32F446       |

---

## 🔧 Microcontroller Info

- **Target MCU:** STM32F446RE
- **Architecture:** ARM Cortex-M4
- **Toolchain:** ARM GCC / STM32CubeIDE / Makefile compatible
- **Debugging Interface:** SWD (OpenOCD / ST-Link)

---

## 📚 Learning Goals

✅ Write low-level peripheral drivers without HAL  
✅ Understand STM32 register maps and bit manipulation  
✅ Gain hands-on experience with UART, SPI, I2C, ADC, Timers, PWM, and GPIO  
✅ Build embedded systems confidence for interviews and real projects

---

## 🔨 How to Build & Flash

You can use:
- **STM32CubeIDE**
- **Makefile + arm-none-eabi-gcc**
- **OpenOCD / ST-Link Utility** for flashing

Make sure to configure:
- Target: STM32F446RE
- Startup file and linker script (if using Makefiles)

---

## ✍️ Author

**Mayur Patil**  
Assistant Embedded Engineer @ Discrete Circuit Pvt Ltd  
GitHub: [@MAYURDIGAMBARPATIL](https://github.com/MAYURDIGAMBARPATIL)

---

## 🧠 Future Plans

- Add Makefiles for each folder  
- Add README for each individual project  
- Create board schematics and simulation support (Proteus / KiCAD)  
- Port examples to STM32F103 & STM32F767

---

## 📄 License

This project is open-source and available under the MIT License. Feel free to use and modify for learning or commercial projects.

