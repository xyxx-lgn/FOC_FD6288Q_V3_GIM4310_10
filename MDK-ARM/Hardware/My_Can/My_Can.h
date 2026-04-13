#ifndef __MY_CAN_H_
#define __MY_CAN_H_

#include "ALL_H.h"

extern uint8_t Motor_Can_ID[6];

void CANFD_Init_Config(void);
uint8_t CANFD_Send(uint8_t motor_id,uint8_t command_type,uint32_t target_value,uint8_t direction,uint16_t rotate_speed);
void CANFD_ReceiveDate(uint8_t * recbuf,uint16_t Identifier,uint16_t len);
void CAN_State(uint16_t Identifier);
#endif
