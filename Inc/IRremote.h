/*
 * IRremote.h
 *
 *  Created on: Nov 16, 2024
 *      Author: wataoxp
 */

#ifndef INC_IRREMOTE_H_
#define INC_IRREMOTE_H_

#include "main.h"

#define IR_DECODE_READY 0
#define IR_ACK 0x06
#define IR_NACK 0x15
#define IR_REPEAT 0x20

/* Falling Edge Exti Timing */
#define IR_MIN_TIMING 10000 //10000us.RepeatCode Limit
#define IR_MAX_TIMING 14000 //14000us.LeaderCode Limit
#define IR_LEADER_TIMING 13000 //13000us.LeaderCode
#define IR_REPEAT_TIMING 11000 //11000us.RepeatCode
#define NEC_HIGH 2000
#define NEC_LOW 1400

/* Polling Status */
#define IR_P_REPEAT 1
#define IR_P_REPEAT_TIMING 4000 //4000us

typedef enum{
	StartLeader,
	EndLeader,
	ErrorLeader,
	ReadTime,
}IRstatus;

typedef enum{
	Init,
	Save,
}INTmode;

typedef struct{
	uint8_t bit31;
	uint8_t bit24;
	uint8_t bit16;
	uint8_t bit8;
}ConvertLSB;

void Sleep200ms(TIM_TypeDef *TIMsleep);
void RecieveIR_IT(TIM_TypeDef *TIMx,uint32_t *Binary,uint8_t *Flag,uint8_t mode);
uint32_t RecieveIR(TIM_TypeDef *TIMx);
void BinaryToHex(ConvertLSB *LSB,uint32_t *Binary);
void DataReset(ConvertLSB *LSB);

#endif /* INC_IRREMOTE_H_ */
