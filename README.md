# STM32F446 Bare‑Metal Drivers 🚀

A collection of **bare-metal (register-level) C examples** for the **STM32F446RE** microcontroller — no HAL or high-level libraries used. Learn by working directly with registers, exploring key peripherals, and building embedded systems skills.

---

## 📁 Project Structure

| Folder                        | Description                                      |
|------------------------------|--------------------------------------------------|
| `01_gpio`                    | Basic GPIO output – LED blinking                |
| `02_GPIO_INPUT`              | Reading digital input from a push-button        |
| `03_UART_TX_RX`             | UART transmit and receive example               |
| `04_UART_TX_RX_Interrupt`   | UART using interrupt-driven communication       |
| `05_ADC_Continuous_Conversion` | Continuous ADC sampling                     |
| `06_I2C_Master`              | I2C Master communication                        |
| `07_I2C_Slave`               | I2C Slave configuration                         |
| `08_SPI`                     | SPI Master communication                        |
| `09_ADC1_ADC2_ADC3`         | Simultaneous ADC data from ADC1/2/3             |
| `10_Timer_gen`              | Basic timer – delay generation                  |
| `11_Timer_interrupt_1s`     | Timer interrupt every 1 second                  |
| `12_ADC_Interrupt`          | ADC sampling using interrupts                   |
| `13_PWM`                    | Multi-channel PWM outputs                       |
| `14_Empty`                  | Reserved for future examples                    |
| `Chip_header`               | Common register definitions for STM32F446       |

---

## ⚙️ Toolchain & Debugging

- **MCU:** STM32F446RE (Cortex-M4, 180 MHz)
- **Toolchain Options:**
  - STM32CubeIDE  
  - ARM GCC / Makefiles  
  - VS Code + Cortex-Debug + OpenOCD  
- **Debug Interface:** SWD (ST-LINK or OpenOCD)

---

## 🔨 Build & Flash

1. **STM32CubeIDE**:  
   - Import each folder as a project  
   - Ensure the correct startup file and linker script  
2. **Makefile + GCC**:  
   - Use `arm-none-eabi-gcc`  
   - Flash with `st-flash` or `OpenOCD`

(*Let me know if you'd like sample Makefiles inserted*)

---

## 📚 Learning Objectives

- Register-level access without abstraction  
- Practical use of UART, I2C, SPI, ADC, Timers, and PWM  
- Techniques for both polling and interrupt-based I/O  
- Essential knowledge for embedded interviews and product development

---

## 🔧 Next Steps

- Add individual `README.md` files per folder  
- Create Makefiles for each example  
- Include `.gitignore` and `LICENSE` (MIT recommended)  
- Expand to other STM32 MCUs (e.g., F103, F767)

---

## 👤 Author

**Mayur Patil**  
— Embedded Engineer | Hardware Enthusiast  
GitHub: [MAYURDIGAMBARPATIL](https://github.com/MAYURDIGAMBARPATIL)

---

## 📄 License

Distributed under the **MIT License** – see `LICENSE` for details.
