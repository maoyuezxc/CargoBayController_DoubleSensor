#include "main.h"
#include "can.h"
#include "string.h"
#include "door_control.h"
#include "detector.h"

// ----------- CAN handle ----------------------------
extern CAN_HandleTypeDef hcan;

// ------------ CAN ID define ------------------------
#define CAN_ID_VCU_TO_CARGO 0x31A
#define CAN_ID_CARGO_TO_VCU 0x31B

/* ------------- CAN message struct define ------------ */
#define CAN_MSG_LEN 8
typedef struct
{
    CAN_RxHeaderTypeDef rxMsgHead;
    uint8_t payload[CAN_MSG_LEN];
} AppCanMsg_t;

typedef struct
{
    CAN_TxHeaderTypeDef txMsgHead;
    uint8_t payload[CAN_MSG_LEN];
} AppCanSendMsg_t;

/* ------------------- Data type definitions ---------------- */
typedef struct
{
    uint8_t doorId; // 1 - Front door 2 - Rear door
    uint8_t action; // 0 - Close 1 - Open 2 - Reset
} CargoDoorControl_t;

/* -------------- Static variables --------------------- */
// Event flags definitions
#define CAN_MSG_RECEIVED 0x01
#define SEND_CAN_MSG 0x02
static uint32_t CanEventFlags = 0x00000000U;
// Received CAN message
static AppCanMsg_t receivedCanMessage;
// Send buffer and sending length
static uint8_t sendBuffer[CAN_MSG_LEN];
static uint16_t sendLen;

/**
 * @brief  Configure the CAN bus, configure the CAN ID filter, start CAN1 and enable the callback function.
 * @param  None
 * @retval None
 */
void CanConfigAndStart(void)
{
    // Filter 0 go fifo 1
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (uint16_t)CAN_ID_VCU_TO_CARGO << 5;
    sFilterConfig.FilterIdLow = CAN_ID_STD | CAN_RTR_DATA;
    sFilterConfig.FilterMaskIdHigh = 0xFF00;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;

    HAL_StatusTypeDef result;

    result = HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
    result = HAL_CAN_Start(&hcan);
    result = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief  Rx FIFO_1 Pending callback function.
 *         Rx FIFO_1 Pending callback function, get a can message at canRxMessage
 *         and the can head at rxMsgHead. Set the os event flags to notify the
 *         thread app_can.
 * @param  hcan CAN_HandleTypeDef*
 * @retval None
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &receivedCanMessage.rxMsgHead, receivedCanMessage.payload) == HAL_OK)
    {
        if (CAN_ID_VCU_TO_CARGO == receivedCanMessage.rxMsgHead.StdId)
        {
            CanEventFlags |= CAN_MSG_RECEIVED;
        }
    }
}

/**
 * @brief  Transmit message by CAN
 * @param  id CAN message ID
 * @param  message 8 bytes message send by CAN
 * @retval none
 */
void CanTransmit(uint32_t id, uint8_t *message, uint32_t size)
{
    AppCanSendMsg_t canSendMsg;
    canSendMsg.txMsgHead.IDE = CAN_ID_STD;
    canSendMsg.txMsgHead.RTR = CAN_RTR_DATA;
    canSendMsg.txMsgHead.TransmitGlobalTime = DISABLE;
    canSendMsg.txMsgHead.DLC = size;
    canSendMsg.txMsgHead.StdId = id;
    memcpy(canSendMsg.payload, message, 8);
    uint32_t txMailBox;
    HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan, &canSendMsg.txMsgHead, canSendMsg.payload, &txMailBox);
    if (HAL_OK == result)
        HAL_Delay(2);
    else
        HAL_Delay(2);
}

/**
 * @brief Send message to RCU by CAN
 * @param message 8 bytes can message array
 * @return none
 */
void CanSendMessageToRcu(uint8_t *message, uint32_t size)
{
    memcpy(sendBuffer, message, size);
    sendLen = size;
    CanEventFlags |= SEND_CAN_MSG;
}

/**
 * @brief Always running function that handle the CAN transmition
 * @param None
 * @retval None
 */
void CanFunction(void)
{
    if (CAN_MSG_RECEIVED & CanEventFlags)
    {
        CanEventFlags &= ~(uint32_t)CAN_MSG_RECEIVED;
        CargoDoorControl_t cargoDoorControl;
        memcpy(&cargoDoorControl, receivedCanMessage.payload, sizeof(cargoDoorControl));
        InformDoorControl(cargoDoorControl.doorId, cargoDoorControl.action);
    }
    if (SEND_CAN_MSG & CanEventFlags)
    {
        CanEventFlags &= ~(uint32_t)SEND_CAN_MSG;
        CanTransmit(CAN_ID_CARGO_TO_VCU, sendBuffer, sendLen);
    }
}

/**
 * @brief Initialize can function
 * @param None
 * @retval None
 */
void InitCan(void)
{
    CanConfigAndStart();
}
