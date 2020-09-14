/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Sep 9, 2020
 *      Author: yash
 */

#include "stm32f446xx_gpio_driver.h"


/*
 * Peripheral Clock Setup
 */

//This function enables or disables peripheral clock for given GPIO port
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init and DeInit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	//configure mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig->GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinMode << (2 * (pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	}else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
			//clear corresponding RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
			//configure corresponding FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//configure FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
		}
		//configure GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
	}
	temp = 0;

	//configure speed
	temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinSpeed << (2 * (pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;
	temp = 0;

	//configure pupd
	temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinPuPdControl << (2 * (pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//configure optype
	temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinOPType << ( pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//configure alt functionality
	if(pGPIOHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure alt fun registers
		uint32_t temp1,temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig->GPIO_PinAltFunMode << (4 * temp2));
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data Read and Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint8_t) pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber%64);
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber%32);
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber%64);
		}
	}
}

void GPIO_IRQPriorityConfig(uint32_t IRQPriority, uint8_t IRQNumber)
{
	//find ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
