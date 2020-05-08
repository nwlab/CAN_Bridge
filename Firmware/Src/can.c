#include "can.h"
#include "logger.h"
#include "nvs.h"

#define MAX_SEND_TRY_COUNT 15000 // this is about 20ms TX timeout (measured by scope)

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_TxHeaderTypeDef Tx1Message;
CAN_RxHeaderTypeDef Rx1Message;
CAN_TxHeaderTypeDef Tx2Message;
CAN_RxHeaderTypeDef Rx2Message;
uint8_t Tx1Data[8];
uint8_t Rx1Data[8];
uint8_t Tx2Data[8];
uint8_t Rx2Data[8];

uint32_t Tx1Mailbox;
uint32_t Tx2Mailbox;

static uint8_t bCAN1_TxReq = 0;
static uint8_t bCAN2_TxReq = 0;

static uint32_t iCAN1_Timeout = 0;
static uint32_t iCAN2_Timeout = 0;

int iCAN1_Prescaler = 6;
int iCAN2_Prescaler = 6;
int iCAN1_FilterIdHigh = 0;
int iCAN1_FilterIdLow = 0;
int iCAN1_FilterMaskIdHigh = 0;
int iCAN1_FilterMaskIdLow = 0;
int iCAN2_FilterIdHigh = 0;
int iCAN2_FilterIdLow = 0;
int iCAN2_FilterMaskIdHigh = 0;
int iCAN2_FilterMaskIdLow = 0;
int iReplace_Count = 0;

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
#define REPLACEMENT_SIZE (12)
#define REPLACEMENT_MAX 32

replace_t iReplace[REPLACEMENT_MAX] =
{ 0 };

void ProcessLogging(CAN_RxHeaderTypeDef *pTxMsg, uint8_t aData[]);

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
void CAN_Init()
{
  HAL_StatusTypeDef res;
  /* Start the CAN peripheral */
  res = HAL_CAN_Start(&hcan1);
  if (res != HAL_OK)
  {
    /* Start Error */
    INFO_MSG("CAN1 Start Error : 0x%X, ErrorCode : 0x%X", (int )res, (int )hcan1.ErrorCode);
    Error_Handler();
  }

  /* Activate CAN RX notification */
  res = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (res != HAL_OK)
  {
    /* Notification Error */
    INFO_MSG("CAN1 Notification Error : 0x%X", (int )res);
    Error_Handler();
  }

  /* Start the CAN peripheral */
  res = HAL_CAN_Start(&hcan2);
  if (res != HAL_OK)
  {
    /* Start Error */
    INFO_MSG("CAN2 Start Error : 0x%X, ErrorCode : 0x%X", (int )res, (int )hcan2.ErrorCode);
    Error_Handler();
  }

  /* Activate CAN RX notification */
  res = HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (res != HAL_OK)
  {
    /* Notification Error */
    INFO_MSG("CAN2 Notification Error : 0x%X", (int )res);
    Error_Handler();
  }
}

/**
 * @brief  Rx Fifo 0 message pending callback
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t bAccept = 0;

  if (hcan == &hcan1)
  {
    DEBUG_MSG("Receive CAN1 packet");
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx1Message, Rx1Data) != HAL_OK)
    {
      /* Reception Error */
      Error_Handler();
    }

    ProcessLogging(&Rx1Message, Rx1Data);

    // software packet filter
    if (Rx1Message.IDE == CAN_ID_STD)
    {
      // standard ID
      if ((Rx1Message.StdId & iCAN1_FilterMaskIdLow) == iCAN1_FilterIdLow)
        bAccept = 1;
    }
    else
    {
      // extended ID
      if ((Rx1Message.ExtId & iCAN1_FilterMaskIdLow) == iCAN1_FilterIdLow)
        bAccept = 1;
    }

    if (bAccept)
    {
      // CAN1 reception
      toggle_time_g = 2;

      // copy all stuff from RX CAN1 to TX CAN2
      Tx2Message.StdId = Rx1Message.StdId;
      Tx2Message.RTR = Rx1Message.RTR;
      Tx2Message.IDE = Rx1Message.IDE;
      Tx2Message.ExtId = Rx1Message.ExtId;
      Tx2Message.DLC = Rx1Message.DLC;
      Tx2Data[0] = Rx1Data[0];
      Tx2Data[1] = Rx1Data[1];
      Tx2Data[2] = Rx1Data[2];
      Tx2Data[3] = Rx1Data[3];
      Tx2Data[4] = Rx1Data[4];
      Tx2Data[5] = Rx1Data[5];
      Tx2Data[6] = Rx1Data[6];
      Tx2Data[7] = Rx1Data[7];

      bCAN2_TxReq = 1;  // requesting transmission for CAN2
    }
  }
  else
  {
    DEBUG_MSG("Receive CAN2 packet");
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx2Message, Rx2Data) != HAL_OK)
    {
      /* Reception Error */
      Error_Handler();
    }
    // software packet filter
    if (Rx2Message.IDE == CAN_ID_STD)
    {
      // standard ID
      if ((Rx2Message.StdId & iCAN2_FilterMaskIdLow) == iCAN2_FilterIdLow)
        bAccept = 1;
    }
    else
    {
      // extended ID
      if ((Rx2Message.ExtId & iCAN2_FilterMaskIdLow) == iCAN2_FilterIdLow)
        bAccept = 1;
    }

    if (bAccept)
    {
      // CAN1 reception
      toggle_time_b = 2;

      // copy all stuff from RX CAN1 to TX CAN2
      Tx1Message.StdId = Rx2Message.StdId;
      Tx1Message.RTR = Rx2Message.RTR;
      Tx1Message.IDE = Rx2Message.IDE;
      Tx1Message.ExtId = Rx2Message.ExtId;
      Tx1Message.DLC = Rx2Message.DLC;
      Tx1Data[0] = Rx2Data[0];
      Tx1Data[1] = Rx2Data[1];
      Tx1Data[2] = Rx2Data[2];
      Tx1Data[3] = Rx2Data[3];
      Tx1Data[4] = Rx2Data[4];
      Tx1Data[5] = Rx2Data[5];
      Tx1Data[6] = Rx2Data[6];
      Tx1Data[7] = Rx2Data[7];

      bCAN1_TxReq = 1;  // requesting transmission for CAN2
    }
  }
}

//----------------------------------------------------------------------------------------
// CAN packets data modification procedure
//----------------------------------------------------------------------------------------

void ProcessModification(CAN_TxHeaderTypeDef *pTxMsg, uint8_t aData[])
{
  unsigned int *iID;
  int i;
  unsigned int *iDataLow = 0;
  unsigned int *iDataHigh = 0;

  if (pTxMsg->IDE == CAN_ID_STD)
    iID = (unsigned int*) &pTxMsg->StdId;
  else
    iID = (unsigned int*) &pTxMsg->ExtId;

  iDataLow = (unsigned int*) (&aData[0]);
  iDataHigh = (unsigned int*) (&aData[4]);

  for (i = 0; i < iReplace_Count; i++)
  {
    // if bit in "ID Mask" =1 AND bit in recieved ID = bit in "ID Filter", then packed ID is accepted for modification
    // AND
    // if bit in "Data Mask" =1 AND bit in recieved ID = bit in "Data Filter", then packed data is accepted for modification

    if ((*iID & iReplace[i].IDMask) != (iReplace[i].IDFilter & iReplace[i].IDMask))
      continue;
    if ((*iDataLow & iReplace[i].DataMaskLow) != (iReplace[i].DataFilterLow & iReplace[i].DataMaskLow))
      continue;
    if ((*iDataHigh & iReplace[i].DataMaskHigh) != (iReplace[i].DataFilterHigh & iReplace[i].DataMaskHigh))
      continue;

    // here is data packed accepted by filter

    // if bit =1 in "New ID Mask" then the value of this bit is being replaced by bit from "New ID Value"
    *iID = (*iID & ~(iReplace[i].NewIDMask)) | iReplace[i].NewIDValue;

    // if bit =1 in "New Data Mask" then the value of this bit is being replaced by bit from "New Data Value"
    *iDataLow = (*iDataLow & ~(iReplace[i].NewDataMaskLow))
        | (iReplace[i].NewDataValueLow & iReplace[i].NewDataMaskLow);
    *iDataHigh = (*iDataHigh & ~(iReplace[i].NewDataMaskHigh))
        | (iReplace[i].NewDataValueHigh & iReplace[i].NewDataMaskHigh);
  }

}

/*
 *
 */
void CAN_Loop()
{
  HAL_StatusTypeDef res;

  if (bCAN2_TxReq)
  {
    LedR(1);
    ProcessModification(&Tx2Message, Tx2Data);
    LedR(0);

    DEBUG_MSG("Transmit CAN2 packet");

    res = HAL_CAN_AddTxMessage(&hcan2, &Tx2Message, Tx2Data, &Tx2Mailbox);

    if (res != HAL_OK)
    {
      /* Transmition Error */
      LedR_Toggle();

      INFO_MSG("CAN1 transmission request Error : 0x%X", (int )res);
      // if we tried hard so many times -- restart CAN
      iCAN2_Timeout++;
      if (iCAN2_Timeout > MAX_SEND_TRY_COUNT)
      {
        iCAN2_Timeout = 0;
        //CAN_CancelTransmit(&hcan2);
        HAL_NVIC_SystemReset();
      }
    }
    else
    {
      bCAN2_TxReq = 0;
      iCAN2_Timeout = 0;
    }
  }

  if (bCAN1_TxReq)
  {
    //ProcessModification(&Tx1Message, Tx1Data); // TODO: maybe add CAN instance selector in replacement data?

    DEBUG_MSG("Transmit CAN1 packet");
    res = HAL_CAN_AddTxMessage(&hcan1, &Tx1Message, Tx1Data, &Tx1Mailbox);

    if (res != HAL_OK)
    {
      /* Transmition Error */
      LedR_Toggle();

      INFO_MSG("CAN2 transmission request Error : 0x%X", (int )res);

      // if we tried hard so many times -- restart CAN
      iCAN1_Timeout++;
      if (iCAN1_Timeout > MAX_SEND_TRY_COUNT)
      {
        iCAN1_Timeout = 0;
        //CAN_CancelTransmit(&hcan1);
        HAL_NVIC_SystemReset();
      }
    }
    else
    {
      bCAN1_TxReq = 0;
      iCAN1_Timeout = 0;
    }
  }

  write_log();
}

//----------------------------------------------------------------------------------------
// Read parameters from NVS
//----------------------------------------------------------------------------------------
void CAN_Read_Param()
{
  uint16_t size = 0;

  if (nvs_get("c1_pre", (uint8_t*) &iCAN1_Prescaler, &size, sizeof(iCAN1_Prescaler)) == KEY_NOT_FOUND
      || nvs_get("c2_pre", (uint8_t*) &iCAN2_Prescaler, &size, sizeof(iCAN2_Prescaler)) == KEY_NOT_FOUND
      || nvs_get("c1_fih", (uint8_t*) &iCAN1_FilterIdHigh, &size, sizeof(iCAN1_FilterIdHigh)) == KEY_NOT_FOUND
      || nvs_get("c1_fil", (uint8_t*) &iCAN1_FilterIdLow, &size, sizeof(iCAN1_FilterIdLow)) == KEY_NOT_FOUND
      || nvs_get("c1_fmih", (uint8_t*) &iCAN1_FilterMaskIdHigh, &size, sizeof(iCAN1_FilterMaskIdHigh)) == KEY_NOT_FOUND
      || nvs_get("c1_fmil", (uint8_t*) &iCAN1_FilterMaskIdLow, &size, sizeof(iCAN1_FilterMaskIdLow)) == KEY_NOT_FOUND
      || nvs_get("c2_fih", (uint8_t*) &iCAN2_FilterIdHigh, &size, sizeof(iCAN2_FilterIdHigh)) == KEY_NOT_FOUND
      || nvs_get("c2_fil", (uint8_t*) &iCAN2_FilterIdLow, &size, sizeof(iCAN2_FilterIdLow)) == KEY_NOT_FOUND
      || nvs_get("c2_fmih", (uint8_t*) &iCAN2_FilterMaskIdHigh, &size, sizeof(iCAN2_FilterMaskIdHigh)) == KEY_NOT_FOUND
      || nvs_get("c2_fmil", (uint8_t*) &iCAN2_FilterMaskIdLow, &size, sizeof(iCAN2_FilterMaskIdLow)) == KEY_NOT_FOUND
      || nvs_get("repl_cnt", (uint8_t*) &iReplace_Count, &size, sizeof(iReplace_Count)) == KEY_NOT_FOUND
      || nvs_get("replace", (uint8_t*) &iReplace, &size, sizeof(iReplace)) == KEY_NOT_FOUND)
  {
    DEBUG_MSG("No parameters in NVS, Set to default");
    iCAN1_Prescaler = 6;
    iCAN2_Prescaler = 6;
    iCAN1_FilterIdHigh = 0;
    iCAN1_FilterIdLow = 0;
    iCAN1_FilterMaskIdHigh = 0;
    iCAN1_FilterMaskIdLow = 0;
    iCAN2_FilterIdHigh = 0;
    iCAN2_FilterIdLow = 0;
    iCAN2_FilterMaskIdHigh = 0;
    iCAN2_FilterMaskIdLow = 0;
    iReplace_Count = 0;
    nvs_put("c1_pre", (uint8_t*) &iCAN1_Prescaler, sizeof(iCAN1_Prescaler), sizeof(iCAN1_Prescaler));
    nvs_put("c2_pre", (uint8_t*) &iCAN2_Prescaler, sizeof(iCAN2_Prescaler), sizeof(iCAN2_Prescaler));
    nvs_put("c1_fih", (uint8_t*) &iCAN1_FilterIdHigh, sizeof(iCAN1_FilterIdHigh), sizeof(iCAN1_FilterIdHigh));
    nvs_put("c1_fil", (uint8_t*) &iCAN1_FilterIdLow, sizeof(iCAN1_FilterIdLow), sizeof(iCAN1_FilterIdLow));
    nvs_put("c1_fmih", (uint8_t*) &iCAN1_FilterMaskIdHigh, sizeof(iCAN1_FilterMaskIdHigh),
        sizeof(iCAN1_FilterMaskIdHigh));
    nvs_put("c1_fmil", (uint8_t*) &iCAN1_FilterMaskIdLow, sizeof(iCAN1_FilterMaskIdLow), sizeof(iCAN1_FilterMaskIdLow));
    nvs_put("c2_fih", (uint8_t*) &iCAN2_FilterIdHigh, sizeof(iCAN2_FilterIdHigh), sizeof(iCAN2_FilterIdHigh));
    nvs_put("c2_fil", (uint8_t*) &iCAN2_FilterIdLow, sizeof(iCAN2_FilterIdLow), sizeof(iCAN2_FilterIdLow));
    nvs_put("c2_fmih", (uint8_t*) &iCAN2_FilterMaskIdHigh, sizeof(iCAN2_FilterMaskIdHigh),
        sizeof(iCAN2_FilterMaskIdHigh));
    nvs_put("c2_fmil", (uint8_t*) &iCAN2_FilterMaskIdLow, sizeof(iCAN2_FilterMaskIdLow), sizeof(iCAN2_FilterMaskIdLow));
    nvs_put("repl_cnt", (uint8_t*) &iReplace_Count, sizeof(iReplace_Count), sizeof(iReplace_Count));
    nvs_put("replace", (uint8_t*) &iReplace, sizeof(iReplace), sizeof(iReplace));
    if (nvs_commit() != NVS_OK)
    {
      DEBUG_MSG("Flash commit failed");
    }
  }
}
