#include "can.h"
#include "logger.h"
#include "nvs.h"
#include "ring.h"

#define MAX_SEND_TRY_COUNT 15000 // this is about 20ms TX timeout (measured by scope)

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint32_t Tx1Mailbox;
uint32_t Tx2Mailbox;

static uint32_t iCAN1_Timeout = 0;
static uint32_t iCAN2_Timeout = 0;

static ring_node_t ring1_msg;
static ring_node_t ring2_msg;

static can_msg_t can1_msg[CAN_MSG_RX_BUF_MAX];
static can_msg_t can2_msg[CAN_MSG_RX_BUF_MAX];

int iCAN1_Prescaler = CAN_42MHZ_500KBPS_PRE;
int iCAN2_Prescaler = CAN_42MHZ_500KBPS_PRE;
uint32_t iCAN1_FilterId = 0;
uint32_t iCAN1_FilterMaskId = 0;
uint32_t iCAN2_FilterId = 0;
uint32_t iCAN2_FilterMaskId = 0;
int iReplace_Count = 0;

replace_t iReplace[REPLACEMENT_MAX] = { 0 };

uint32_t iCAN1_Received = 0;
uint32_t iCAN2_Received = 0;
uint32_t iCAN1_Transmited = 0;
uint32_t iCAN2_Transmited = 0;

void ProcessLogging(CAN_RxHeaderTypeDef *pTxMsg, uint8_t aData[]);

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
void CAN_Init()
{
  HAL_StatusTypeDef res;

  ringCreate(&ring1_msg, CAN_MSG_RX_BUF_MAX);
  ringCreate(&ring2_msg, CAN_MSG_RX_BUF_MAX);

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

uint8_t CAN_GetErrCount(CAN_HandleTypeDef *hcan)
{
  return (hcan->Instance->ESR) >> 24;
}

uint32_t CAN_GetError(CAN_HandleTypeDef *hcan)
{
  return HAL_CAN_GetError(hcan);
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
  CAN_RxHeaderTypeDef RxMessage;
  uint8_t RxData[8];

  if (hcan == &hcan1)
  {
    //DEBUG_MSG("Receive CAN1 packet");
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxData) != HAL_OK)
    {
      /* Reception Error */
      Error_Handler();
    }

    iCAN1_Received++;

    ProcessLogging(&RxMessage, RxData);

    // software packet filter
    if (RxMessage.IDE == CAN_ID_STD)
    {
      // standard ID
      if ((RxMessage.StdId & iCAN1_FilterMaskId) == iCAN1_FilterId)
        bAccept = 1;
    }
    else
    {
      // extended ID
      if ((RxMessage.ExtId & iCAN1_FilterMaskId) == iCAN1_FilterId)
        bAccept = 1;
    }

    if (bAccept)
    {
      // CAN1 reception
      toggle_time_g = 2;

      // copy all stuff from RX CAN1 to TX CAN2
      uint8_t msg_idx = ringGetWriteIndex(&ring2_msg);
      can_msg_t *rx_buf = &can2_msg[msg_idx];

      rx_buf->TxHeader.StdId = RxMessage.StdId;
      rx_buf->TxHeader.RTR = RxMessage.RTR;
      rx_buf->TxHeader.IDE = RxMessage.IDE;
      rx_buf->TxHeader.ExtId = RxMessage.ExtId;
      rx_buf->TxHeader.DLC = RxMessage.DLC;
      rx_buf->Data[0] = RxData[0];
      rx_buf->Data[1] = RxData[1];
      rx_buf->Data[2] = RxData[2];
      rx_buf->Data[3] = RxData[3];
      rx_buf->Data[4] = RxData[4];
      rx_buf->Data[5] = RxData[5];
      rx_buf->Data[6] = RxData[6];
      rx_buf->Data[7] = RxData[7];

      ringWriteUpdate(&ring2_msg);
    }
  }
  else
  {
    //DEBUG_MSG("Receive CAN2 packet");
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, RxData) != HAL_OK)
    {
      /* Reception Error */
      Error_Handler();
    }

    iCAN2_Received++;

    // software packet filter
    if (RxMessage.IDE == CAN_ID_STD)
    {
      // standard ID
      if ((RxMessage.StdId & iCAN2_FilterMaskId) == iCAN2_FilterId)
        bAccept = 1;
    }
    else
    {
      // extended ID
      if ((RxMessage.ExtId & iCAN2_FilterMaskId) == iCAN2_FilterId)
        bAccept = 1;
    }

    if (bAccept)
    {
      // CAN1 reception
      toggle_time_b = 2;

      // copy all stuff from RX CAN2 to TX CAN1
      uint8_t msg_idx = ringGetWriteIndex(&ring1_msg);
      can_msg_t *rx_buf = &can1_msg[msg_idx];

      rx_buf->TxHeader.StdId = RxMessage.StdId;
      rx_buf->TxHeader.RTR = RxMessage.RTR;
      rx_buf->TxHeader.IDE = RxMessage.IDE;
      rx_buf->TxHeader.ExtId = RxMessage.ExtId;
      rx_buf->TxHeader.DLC = RxMessage.DLC;
      rx_buf->Data[0] = RxData[0];
      rx_buf->Data[1] = RxData[1];
      rx_buf->Data[2] = RxData[2];
      rx_buf->Data[3] = RxData[3];
      rx_buf->Data[4] = RxData[4];
      rx_buf->Data[5] = RxData[5];
      rx_buf->Data[6] = RxData[6];
      rx_buf->Data[7] = RxData[7];

      ringWriteUpdate(&ring1_msg);
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

  //Transmit to CAN2
  if (ringReadAvailable(&ring2_msg))
  {
    can_msg_t *p_ret = &can2_msg[ringGetReadIndex(&ring2_msg)];

    ringReadUpdate(&ring2_msg);

    LedR(1);
    ProcessModification(&p_ret->TxHeader, p_ret->Data);
    LedR(0);

    //DEBUG_MSG("Transmit CAN2 packet");

    res = HAL_CAN_AddTxMessage(&hcan2, &p_ret->TxHeader, p_ret->Data, &Tx2Mailbox);

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
      while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3)
      {
      }
      iCAN2_Transmited++;
      iCAN2_Timeout = 0;
    }
  }

  //Transmit to CAN1
  if (ringReadAvailable(&ring1_msg))
  {
    can_msg_t *p_ret = &can1_msg[ringGetReadIndex(&ring1_msg)];

    ringReadUpdate(&ring1_msg);

    //ProcessModification(&Tx1Message, Tx1Data); // TODO: maybe add CAN instance selector in replacement data?

    //DEBUG_MSG("Transmit CAN1 packet");

    res = HAL_CAN_AddTxMessage(&hcan1, &p_ret->TxHeader, p_ret->Data, &Tx1Mailbox);

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
      while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3)
      {
      }
      iCAN1_Transmited++;
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
      || nvs_get("c1_fid", (uint8_t*) &iCAN1_FilterId, &size, sizeof(iCAN1_FilterId)) == KEY_NOT_FOUND
      || nvs_get("c1_fmid", (uint8_t*) &iCAN1_FilterMaskId, &size, sizeof(iCAN1_FilterMaskId)) == KEY_NOT_FOUND
      || nvs_get("c2_fid", (uint8_t*) &iCAN2_FilterId, &size, sizeof(iCAN2_FilterId)) == KEY_NOT_FOUND
      || nvs_get("c2_fmid", (uint8_t*) &iCAN2_FilterMaskId, &size, sizeof(iCAN2_FilterMaskId)) == KEY_NOT_FOUND
      || nvs_get("repl_cnt", (uint8_t*) &iReplace_Count, &size, sizeof(iReplace_Count)) == KEY_NOT_FOUND
      || nvs_get("replace", (uint8_t*) &iReplace, &size, sizeof(iReplace)) == KEY_NOT_FOUND)
  {
    DEBUG_MSG("No parameters in NVS, Set to default");
    iCAN1_Prescaler = CAN_42MHZ_500KBPS_PRE;
    iCAN2_Prescaler = CAN_42MHZ_500KBPS_PRE;
    iCAN1_FilterId = 0;
    iCAN1_FilterMaskId = 0;
    iCAN2_FilterId = 0;
    iCAN2_FilterMaskId = 0;
    iReplace_Count = 0;

    CAN_Write_Param();

    if (nvs_commit() != NVS_OK)
    {
      DEBUG_MSG("Flash commit failed");
    }
  }
}

void CAN_Write_Param()
{
  nvs_put("c1_pre", (uint8_t*) &iCAN1_Prescaler, sizeof(iCAN1_Prescaler), sizeof(iCAN1_Prescaler));
  nvs_put("c2_pre", (uint8_t*) &iCAN2_Prescaler, sizeof(iCAN2_Prescaler), sizeof(iCAN2_Prescaler));
  nvs_put("c1_fid", (uint8_t*) &iCAN1_FilterId, sizeof(iCAN1_FilterId), sizeof(iCAN1_FilterId));
  nvs_put("c1_fmid", (uint8_t*) &iCAN1_FilterMaskId, sizeof(iCAN1_FilterMaskId), sizeof(iCAN1_FilterMaskId));
  nvs_put("c2_fid", (uint8_t*) &iCAN2_FilterId, sizeof(iCAN2_FilterId), sizeof(iCAN2_FilterId));
  nvs_put("c2_fmid", (uint8_t*) &iCAN2_FilterMaskId, sizeof(iCAN2_FilterMaskId), sizeof(iCAN2_FilterMaskId));
  nvs_put("repl_cnt", (uint8_t*) &iReplace_Count, sizeof(iReplace_Count), sizeof(iReplace_Count));
  nvs_put("replace", (uint8_t*) &iReplace, sizeof(iReplace), sizeof(iReplace));
}
