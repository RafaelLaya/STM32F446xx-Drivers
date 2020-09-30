# STM32F446xx-Drivers

Drivers for the STM32F446xx Family of Microcontrollers. This family from ST Microelectronics uses an ARM Cortex M4 processor with a floating point unit (ARM Cortex M4F). The drivers include interrupt and polling methods for:

* I2C
* GPIO
* SPI
* UART
* Dynamic IRQ System with Support for Opaque Data

# Dependencies
This is a bare-metal layer and everything is self-contained in this repository. 

# Usage

## UART
* Enable the clock to the peripheral with `USART_PeriClockControl()`
* Declare a `USART_Handle_t` structure and fill with initialization information, and initialize with a call to `USART_Init()`
* If using interrupts, use `USART_IRQPriorityConfig()` to configure priority for this UART. Then `USART_IRQInterruptConfig()` to enable or disable the interrupt line.
* Call `USART_PeripheralControl()` to enable or disable the peripheral. Call `USART_IRQHandling()` as soon as possible in the ISR 
* Send data with `USART_SendData()` or `USART_SendDataIT()`. The former polls and the latter uses interrupts
* Receive data with `USART_ReceiveData()` or `USART_ReceiveDataIT()`, the former polls and the latter uses interrupts
* If using interrupts, you might want to implement the callback `USART_ApplicationEventCallback()`
* You can call `USART_DeInit()` to reset the peripheral
* Calls to `USART_GetFlagStatus()` and `USART_ClearFlag()` allow you to read and clear flags, respectively

## SPI
* Enable the clock to the peripheral with `SPI_PeriClockControl()`
* Declare a `SPI_Handle_t` structure and fill with initialization information, then initialize with a call to `SPI_Init()`
* `SPI_PeripheralControl()` can be used to enable or disable the peripheral
* Send data with `SPI_SendData()` or `SPI_SendDataWithIT()`. The former polls and the latter uses interrupts
* Receive data with `SPI_ReadData()` or `SPI_ReadDataWithIT()`. The former polls and the latter uses interrupts
* If using interrupts, call `SPI_IRQPriorityConfig()` to configure priority. Then call `SPI_IRQInterruptConfig()` to enable or disable the interrupt line. The function `SPI_IRQHandling()` must be called as soon as possible in the ISR. 
* If using interrupts, you might want to implement the callback `SPI_ApplicationEventCallback()`
* You can call `SPI_DeInit()` to reset the peripheral
* Calls to `SPI_GetFlagStatus()` allow you to read flags
* SSI and SSOE are supported and enabled/disabled through `SPI_SSIConfig()` and `SPI_SSOEConfig()`

## GPIO
* Enable the clock to the peripheral with `GPIO_PeriClockControl()`
* Declare a `GPIO_Handle_t` structure and fill with initialization information, then initialize with a call to `GPIO_Init()`
* Read a pin with `GPIO_ReadFromInputPin()` or a whole port with `GPIO_ReadFromInputPort()`
* Write to a pin with `GPIO_WriteToOutputPin()` or a port with `GPIO_WriteToOutputPort()`. You can also toggle an output pin with `GPIO_ToggleOutputPin()`
* If using interrupts, call `GPIO_IRQPriorityConfig()` to configure priority. Then call `GPIO_IRQInterruptConfig()` to enable or disable the interrupt line. The function `GPIO_IRQHandling()` must be called as soon as possible within the ISR
* You can call `GPIO_DeInit()` to reset the peripheral

## I2C
* Enable the clock to the peripheral with `I2C_PeripheralClockControl()`
* Declare an `I2C_Handle_t` structure and fill with initialization information, then initialize with a call to `I2C_Init()`
* Use `I2C_ManageAcking()` to configure acknowledge control
* As a master device, you can send data with `I2C_MasterSendData()` or `I2C_MasterSendDataIT()`. The former polls while the latter uses interrupts
* Use `I2C_PeripheralControl()` to enable or disable the peripheral
* You can receive data with `I2C_MasterReceiveDataIT()` or `I2C_SlaveReceiveData()`. The former polls while the latter uses interrupts.
* As a slave device, you can respond with `I2C_SlaveSendData()` or `I2C_SlaveReceiveData()`. 
* If using interrupts, call `I2C_IRQPriorityConfig()` to configure priority. Then call `I2C_IRQInterruptConfig()` to enable or disable the interrupt line. The funciton `I2C_EV_IRQhandling()` must be called as soon as possible within the ISR. Optionally, call `I2C_ERR_IRQHandling()` to get a callback when an error is detected. You might want to implement the callback `I2C_ApplicationEventCallback()`
* You can call `I2C_DeInit()` to reset the peripheral

## Dynamic IRQ
* Always call `IRQ_Init()` to initialize the dynamic IRQ system
* An ISR must have the following type (`irq` is the IRQ Number and `data` is the opaque data): `typedef void (*IRQ_Handler)(IRQn_Type irq, void *data);` 
* Register the ISR with `IRQ_Register()` and unregister with `IRQ_Unregister()`
* Configure the priority with `IRQ_SetPriority()`. The current priority can be retrieved with `IRQ_GetPriority()`.
* Enable the interrupt line with `IRQ_Enable()` and disable with `IRQ_Disable()`
* Change the opaque data with `IRQ_SetData()`
* Retrieve the opaque data outside of the ISR with `IRQ_GetData()`

# Applied Example
Let's say you want to make a simple application where you'd like to turn on an LED whenever a button is pressed, but you want the LED off
whenever the button is not being pressed. Let's use the on-board LED and user button for this example.

The first thing is to design the circuitry. This is done for us since we are using on-board components. Second, we need to select which pins to use. Since we are using on-board components, we need to look at the schematics. The schematics can be found in the user manual which can be downloaded from ST's website <a href="https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html"> click here to go to ST's website </a>. From the schematics, we know that PA5 (Port A, Pin 5) is connected to the LED, and PC13 (Port C, Pin 13) is connected to the push-button. From the schematics, we can also see that PC13 is pulled to high by a pull-up resistor. PC 13 is driven to low when the button is pressed. When PC13 is Low (pressed), we will turn on the LED (PA5). When PC13 is High (not pressed), we will turn off the LED (PA5).

<img src="images/LED.PNG?raw=true"/>	

<img src="images/Button.PNG?raw=true"/>

Now let's dive into the code. The general procedure for these drivers is to:

1. Create a handler. The handler keeps track of configurations and it keeps a pointer to the base address of the peripheral. Pointers to the base address of the peripherals have already been defined for you. 
2. Fill the configurations.
3. Enable the clock, and initialize the desired configuration.
4. If using interrupts, configure the priority, register the handler, and then enable the interrupt.

```C
// 1. Create Handles
GPIO_Handle_t GPIOHandle_C13;
GPIO_Handle_t GPIOHandle_A5;
  
// 1. Point to GPIO Port C and A
GPIOHandle_C13.pGPIO = GPIOC;
GPIOHandle_A5.pGPIO = GPIOA;
  
// 2. Configurations
GPIOHandle_C13.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
GPIOHandle_C13.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
GPIOHandle_C13.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  
GPIOHandle_A5.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
GPIOHandle_A5.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
GPIOHandle_A5.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
GPIOHandle_A5.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
GPIOHandle_A5.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  
// 3. Enable Clock
GPIO_PeriClockControl(GPIOHandle_C13.pGPIO, ENABLE);
GPIO_PeriClockControl(GPIOHandle_A5.pGPIO, ENABLE);
  
// 3. Initialize
GPIO_Init(&GPIOHandle_C13);
GPIO_Init(&GPIOHandle_A5);
```

Now, let's add the logic:

```C
while(1) {
    // if !High -> if Low -> If pressed -> turn LED on
    if(!GPIO_ReadFromInputPin(GPIOHandle_C13.pGPIO, GPIOHandle_C13.GPIO_PinConfig.GPIO_PinNumber)) {
        GPIO_WriteToOutputPin(GPIOHandle_A5.pGPIO, GPIOHandle_A5.GPIO_PinConfig.GPIO_PinNumber, SET);
    } else {
        GPIO_WriteToOutputPin(GPIOHandle_A5.pGPIO, GPIOHandle_A5.GPIO_PinConfig.GPIO_PinNumber, RESET);
    }
}
```

Thus the whole program can be written like this:

```C
#include "stm32f446xx.h"

int main(void) {
    // Create Handles
    GPIO_Handle_t GPIOHandle_C13;
    GPIO_Handle_t GPIOHandle_A5;
    
    // Point to GPIO Port C and A
    GPIOHandle_C13.pGPIO = GPIOC;
    GPIOHandle_A5.pGPIO = GPIOA;
  
    // Configurations
    GPIOHandle_C13.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOHandle_C13.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIOHandle_C13.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  
    GPIOHandle_A5.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIOHandle_A5.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIOHandle_A5.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIOHandle_A5.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIOHandle_A5.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  
    // Enable Clock
    GPIO_PeriClockControl(GPIOHandle_C13.pGPIO, ENABLE);
    GPIO_PeriClockControl(GPIOHandle_A5.pGPIO, ENABLE);
  
    // Initialize
    GPIO_Init(&GPIOHandle_C13);
    GPIO_Init(&GPIOHandle_A5);
  
    while(1) {
        // if !High -> if Low -> If pressed -> turn LED on
 	if(!GPIO_ReadFromInputPin(GPIOHandle_C13.pGPIO, GPIOHandle_C13.GPIO_PinConfig.GPIO_PinNumber)) {
   	    GPIO_WriteToOutputPin(GPIOHandle_A5.pGPIO, GPIOHandle_A5.GPIO_PinConfig.GPIO_PinNumber, SET);
	} else {
	    GPIO_WriteToOutputPin(GPIOHandle_A5.pGPIO, GPIOHandle_A5.GPIO_PinConfig.GPIO_PinNumber, RESET);
	}
    }
}
```
