# STM32F446xx-Drivers
STM32f446xx drivers for GPIO, I2C, USART/UART, SPI peripherals written by an electrical engineering student 

Some notes:
- RCC Drivers are unfinished.
- I might want to add more drivers later (RCC+PLL+Low Power Modes+Low Voltage Domain, Timers (Basic, General, Advanced, PWM, Input Capture, Output Compare, Watchdog, etc), CAN, RTC, More generalized IRQ drivers, DAC, ADC, improve or add more functionality to the existing drivers, add examples of usage, etc.) 
- Might be benefitial to upload some Unit Tests.
- Methods expect correct inputs (No assertions (assert()) are used). Might want to add some optional asserts within #if directives.
