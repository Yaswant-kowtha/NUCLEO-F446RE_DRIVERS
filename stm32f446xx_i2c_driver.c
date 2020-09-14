/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Sep 11, 2020
 *      Author: yash
 */

#include "stm32f446xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr <<1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr <<1;
	SlaveAddr |= (1);
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//device in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//disable ack
				I2C_ManageAcking(pI2CHandle->pI2Cx , DISABLE);

				//clear ADDR flag
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			//clear ADDR flag
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}else{
		//device in slave mode
	}
}
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

uint32_t RCC_GetPLLOutputClock(void){
	uint32_t pll;
	return pll;
}
uint16_t AHB_PreScaler[8] = {2,4,8,16,32,64,128,256};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clksrc,temp,ahbp,apb1p;
	clksrc = (RCC->CFGR >> 2)& 0x3;
	if(clksrc==0){
		SystemClk=16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc ==2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}
	//ahb
	temp = (RCC->CFGR >>4) & 0xF;
	if(temp <8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}
	//apb1
	temp = (RCC->CFGR >>4) & 0xF;
	if(temp <4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}
	pclk1 = (SystemClk /ahbp)/apb1p;
	return pclk1;
}
/*
 * Peripheral Clock Setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

/*
 * Init and DeInit
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//enable clock for i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure FREQ of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U;
	pI2CHandle->pI2Cx->CR2 = tempreg&0x3F;

	//program device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = RCC_GetPCLK1Value()/(2 *pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= ccr_value & 0xFFF;
	}else
	{
		//mode is fast mode
		tempreg |= (1 <<15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			//duty 0
			ccr_value = RCC_GetPCLK1Value()/(3 *pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}else
		{
			//duty 1
			ccr_value = RCC_GetPCLK1Value()/(25 *pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= ccr_value & 0xFFF;
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE config
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		tempreg = (RCC_GetPCLK1Value()/1000000U) + 1;
	}else
	{
		//mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300)/1000000U) + 1;

	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/*
 * Data send and receive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr, uint8_t Sr)
{
	//generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//confirm that start generation is completed by checking SB flag in CR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//send address of slave with r/w bit set to 0
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//confirm that address phase is completed by checking the ADDR flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//clear ADDR flag
	I2C_ClearADDRFlag(pI2CHandle);

	//send data until len becomes 0

	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); //wait till txe is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//when len becomes zero wait for txe=1 and btf=1 before generating stop condition
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//generate stop condition
	if(Sr == I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len,uint8_t SlaveAddr, uint8_t Sr)
{
	//generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//confirm that start generation is completed by checking SB flag in CR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//send address of slave with r/w bit set to 0
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//confirm that address phase is completed by checking the ADDR flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//procedure to read only 1 byte
	if(Len ==1 )
	{
		//disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//generate stop condition
		if(Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//procedure to read data from slave when len>1
	if(Len > 1)
	{
		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read data until len becomes 0
		for(uint32_t i=Len; i>0; i--)
		{
			//wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i==2) //if last 2 bytes are remaining
			{
				//clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//generate stop condition
				if(Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}
			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}

	}

	//re-enable ack
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}


}


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len,uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_RX) & (busystate != I2C_BUSY_IN_TX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//implement code to generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//implement code to enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//implement the code to enable ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//implement the code to enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;

}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len,uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_RX) & (busystate != I2C_BUSY_IN_TX))
	{
		pI2CHandle->pTxBuffer = pRxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//implement code to generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//implement code to enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//implement the code to enable ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//implement the code to enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}





/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQNumber){
	//find ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//implement code to disable ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//implement code to disable ITEVFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//implement code to disable ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//implement code to disable ITEVFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}

}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		//load data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//decrement TxLen
		pI2CHandle->TxLen--;

		//increment buffer address
		pI2CHandle->pTxBuffer++;
	}


}
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen ==2)
		{
			//clear ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		//read dr
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;

	}
	if(pI2CHandle->RxLen == 0)
	{
		//generate stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//close I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//notify the application
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t) pI2C->DR;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//interrupt handling for both master and slave mode of a device
	uint32_t temp1,temp2,temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	//Handle for interrupt generated by SB flag
	if(temp1 && temp3)
	{
		//interrupt generated because of sb
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//Handle for interrupt generated by ADDR event
	//when master mode: address is sent
	//when slave mode: address matched with own address
	if(temp1 && temp3)
	{
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//handle for interrupt generated by BTF event
	if(temp1 && temp3)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				if(pI2CHandle->TxLen == 0)//generate stop condition
				{
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					//reset all member elements of handle structure
					I2C_CloseSendData(pI2CHandle);
					//notify application about transmission complete
					I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CMPLT);

				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{

		}
	}

	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//handle for interrupt generated by STOPF event
	if(temp1 && temp3)
	{
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//notify application that stop is detected
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP);
	}


	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	//Handle for interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3= pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//Handle for interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{

			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else
		{
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}

}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	//status of ITERREN control bit
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	//check for bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		//clear bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		//code to notify application about error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_BERR);
	}

	//check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if(temp1 && temp2)
	{
		//clear bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//code to notify application about error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_ARLO);

	}

	//check for AF error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if(temp1 && temp2)
	{
		//clear bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//code to notify application about error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_AF);

	}

	//check for OVR error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if(temp1 && temp2)
	{
		//clear bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//code to notify application about error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_OVR);

	}

	//check for TIMEOUT error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2)
	{
		//clear bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//code to notify application about error
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_ERROR_TIMEOUT);

	}

}



/*
 * Other peripheral control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi==I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

//Application call back
void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

