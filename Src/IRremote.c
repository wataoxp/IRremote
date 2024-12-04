/*
 * IRremote.c
 *
 *  Created on: Nov 16, 2024
 *      Author: wataoxp
 */
#include "main.h"
#include "IRremote.h"

static inline void DisableIR(void)
{
	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_3);
	__NVIC_DisableIRQ(EXTI2_3_IRQn);
	__NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
}
void Sleep200ms(TIM_TypeDef *TIMsleep)
{
	LL_TIM_EnableIT_UPDATE(TIMsleep);

	__WFI();													//wait for interrupt
	while(LL_TIM_IsActiveFlag_UPDATE(TIMsleep) == 0);
	LL_TIM_ClearFlag_UPDATE(TIMsleep);
}
uint32_t RecieveIR(TIM_TypeDef *TIMx)
{
	uint32_t Binary = 0;
	uint8_t numBits = 0;

	while((LL_GPIO_IsInputPinSet(IR_p_GPIO_Port, IR_p_Pin)));	//wait HighPulse

	while(!(LL_GPIO_IsInputPinSet(IR_p_GPIO_Port, IR_p_Pin)));	//Start LeaderCode
	TIMx->CNT = 0;
	while((LL_GPIO_IsInputPinSet(IR_p_GPIO_Port, IR_p_Pin)));
	if(TIMx->CNT < IR_P_REPEAT_TIMING)
	{
		return IR_P_REPEAT;
	}															//End LeaderCode

	while(numBits < 32)											//Check DataBit
	{
		TIMx->CNT = 0;
		while(!(LL_GPIO_IsInputPinSet(IR_p_GPIO_Port, IR_p_Pin)));
		while((LL_GPIO_IsInputPinSet(IR_p_GPIO_Port, IR_p_Pin)));
		if(TIMx->CNT > NEC_HIGH)
		{
			Binary |= 1 << numBits;
		}
		numBits++;
	}
	return Binary;
}
void RecieveIR_IT(TIM_TypeDef *TIMx,uint32_t *Binary,uint8_t *Flag,uint8_t mode)
{
	static uint32_t *pBinary;
	static uint8_t *pFlag;
	static uint8_t Status = StartLeader;
	static uint8_t numBits = 0;

	if(mode == Init)
	{
		pBinary = Binary;
		pFlag = Flag;
		return;
	}
	__disable_irq();
	switch(Status)
	{
	case StartLeader:
		TIMx->CNT = 0;
		Status = EndLeader;
		break;

	case EndLeader:
		if(TIMx->CNT < IR_MIN_TIMING || TIMx->CNT > IR_MAX_TIMING)
		{
			Status = StartLeader;
			*pFlag = IR_NACK;
			DisableIR();
		}
		else if(TIMx->CNT > IR_LEADER_TIMING)
		{
			Status = ReadTime;
			TIMx->CNT = 0;
		}
		else if(TIMx->CNT > IR_REPEAT_TIMING)
		{
			Status = StartLeader;
			*pFlag = IR_REPEAT;
			DisableIR();
		}
		break;

	case ReadTime:
		if(TIMx->CNT > NEC_HIGH)
		{
			*pBinary |= 1 << numBits;
		}
		numBits++;
		TIMx->CNT = 0;
		if(numBits > 31)
		{
			Status = StartLeader;
			*pFlag = IR_ACK;
			numBits = 0;
			DisableIR();
		}
		break;

	default:
		break;
	}
	__enable_irq();
}
void BinaryToHex(ConvertLSB *LSB,uint32_t Binary)
{
	LSB->bit8 = (uint8_t)(Binary & 0xFF);
	LSB->bit16 = (uint8_t)(Binary >> 8) & 0xFF;
	LSB->bit24 = (uint8_t)(Binary >> 16) & 0xFF;
	LSB->bit31 = (uint8_t)(Binary >> 24) & 0xFF;

}
void DataReset(ConvertLSB *LSB)
{
	LSB->bit8 = 0;
	LSB->bit16 = 0;
	LSB->bit24 = 0;
	LSB->bit31 = 0;
}
