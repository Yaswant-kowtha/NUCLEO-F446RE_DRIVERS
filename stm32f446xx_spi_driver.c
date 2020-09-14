/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Sep 10, 2020
 *      Author: yash
 */

#include "stm32f446xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral Clock Setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}

	}
}
/*
 * Init and DeInit
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//configure SPI_CR1 register
	uint32_t tempreg = 0;

	//configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//configure bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &=(1<<15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_HD)
	{
		//bidimode should be set
		tempreg |= (1<<15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &=(1<<15);
		//RXONLY bit must be set
		tempreg |= (1 << 10);
	}

	//configure spi serial clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed <<3;

	//configure DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	//configure CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL <<1;

	//configure CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA <<0;

	pSPIHandle->pSPIx->CR1 = tempreg;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

/*
 * Data send and receive
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}else
	{
	return FLAG_RESET;
	}
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//check DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//load data into DRR
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}


}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//check DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//load data from DR to Rxbuffer
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}else
		{
			//8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}

}

/*
 * IRQ Configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQNumber){
	//find ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1,temp2;
	//check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if( temp1 & temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}
	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if( temp1 & temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pHandle);
	}

	//check for OVR
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if( temp1 & temp2)
	{
		//handle TXE
		spi_ovr_err_interrupt_handle(pHandle);
	}
}
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//check DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit DFF
		//load data into DRR
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(!pSPIHandle->TxLen)
	{
		//txlen is zero. so close spi transmission
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//check DFF bit in CR1
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		//16 bit DFF
		//load data into DRR
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pRxBuffer);
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pRxBuffer);
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen)
	{
		//txlen is zero. so close spi transmission
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	//clear ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

	}
	(void)temp;
	//inform application
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t * pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//Save Tx buffer address and Len
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//mark spi state as busy in transmission
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Enable TXEIE control bit
		pSPIHandle->pSPIx->CR2 |= (1<< SPI_CR2_TXEIE);

		//data transmission will be handled by isr code
	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//Save Tx buffer address and Len
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//mark spi state as busy in transmission
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//Enable TXEIE control bit
		pSPIHandle->pSPIx->CR2 |= (1<< SPI_CR2_RXNEIE);

		//data transmission will be handled by isr code
	}

	return state;
}

__attribute__((weak)) void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEv);
/*
 * Other peripheral control APIs
 */


