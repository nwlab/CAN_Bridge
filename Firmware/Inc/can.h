/*
 * can.h
 *
 *  Created on: May 8, 2020
 *      Author: nightworker
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* a popular industrial application has optional settings of 125 kbps, 250 kbps, or 500 kbps  */
#define CAN_42MHZ_1000KBPS_PRE  3
#define CAN_42MHZ_1000KBPS_TS1  CAN_BS1_11TQ
#define CAN_42MHZ_1000KBPS_TS2  CAN_BS2_2TQ

#define CAN_42MHZ_500KBPS_PRE   6
#define CAN_42MHZ_500KBPS_TS1   CAN_BS1_11TQ
#define CAN_42MHZ_500KBPS_TS2   CAN_BS2_2TQ

#define CAN_42MHZ_250KBPS_PRE   12
#define CAN_42MHZ_250KBPS_TS1   CAN_BS1_11TQ
#define CAN_42MHZ_250KBPS_TS2   CAN_BS2_2TQ

#define CAN_42MHZ_125KBPS_PRE   21
#define CAN_42MHZ_125KBPS_TS1   CAN_BS1_13TQ
#define CAN_42MHZ_125KBPS_TS2   CAN_BS2_2TQ

#define CAN_MSG_RX_BUF_MAX  8

#define REPLACEMENT_MAX 32

typedef struct can_msg
{
  union
  {
    CAN_TxHeaderTypeDef TxHeader;
    CAN_RxHeaderTypeDef RxHeader;
  };
  uint8_t Data[8];
} can_msg_t;

typedef struct replace
{
  unsigned int IDMask;
  unsigned int IDFilter;
  unsigned int NewIDMask;
  unsigned int NewIDValue;
  unsigned int DataMaskHigh;
  unsigned int DataMaskLow;
  unsigned int DataFilterHigh;
  unsigned int DataFilterLow;
  unsigned int NewDataMaskHigh;
  unsigned int NewDataMaskLow;
  unsigned int NewDataValueHigh;
  unsigned int NewDataValueLow;
} replace_t;

extern int iCAN1_Prescaler;
extern int iCAN2_Prescaler;
extern uint32_t iCAN1_FilterId;
extern uint32_t iCAN1_FilterMaskId;
extern uint32_t iCAN2_FilterId;
extern uint32_t iCAN2_FilterMaskId;
extern int iReplace_Count;
extern replace_t iReplace[];
extern uint32_t iCAN1_Received;
extern uint32_t iCAN2_Received;
extern uint32_t iCAN1_Transmited;
extern uint32_t iCAN2_Transmited;

void CAN_Init(void);
void CAN_Loop(void);
void CAN_Read_Param(void);
void CAN_Write_Param(void);
uint8_t CAN_GetErrCount(CAN_HandleTypeDef *hcan);
uint32_t CAN_GetError(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* INC_CAN_H_ */
