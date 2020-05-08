/*
 * can.h
 *
 *  Created on: May 8, 2020
 *      Author: nightworker
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"

extern int iCAN1_Prescaler;
extern int iCAN2_Prescaler;

void CAN_Init(void);
void CAN_Loop(void);
void CAN_Read_Param(void);

#endif /* INC_CAN_H_ */
