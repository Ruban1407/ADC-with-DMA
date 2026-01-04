# STM32 Timer → ADC → DMA → UART (STM32G491RE)

## Description
This project demonstrates a timer-triggered ADC conversion using DMA.
A 100 Hz TIM6 update event triggers ADC1 conversion on the internal VREFINT channel.
The ADC result is transferred via DMA and sent as a floating-point voltage over USART2.

## Hardware
- STM32G491RE (Nucleo)
- No external sensors used (internal VREFINT)

## Peripherals Used
- TIM6 (100 Hz)
- ADC1 (VREFINT)
- DMA
- USART2 (115200 baud)

## Output
ADC raw value and converted voltage displayed on serial terminal (PuTTY).

## Author
D S Ruban
