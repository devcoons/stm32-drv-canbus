/*!
	@file   drv_canbus.c
	@brief  <brief description here>
	@t.odo	-
	---------------------------------------------------------------------------

	MIT License
	Copyright (c) 2019 Io. D

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/
/******************************************************************************
* Preprocessor Definitions & Macros
******************************************************************************/


/******************************************************************************
* Includes
******************************************************************************/

#include "drv_canbus.h"

#ifdef DRV_CANBUS_ENABLED

/******************************************************************************
* Enumerations, structures & Variables
******************************************************************************/

static FDCAN_TxHeaderTypeDef TxHeader;
static canbus_t* canbus_interfaces[8] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
static uint32_t canbus_interfaces_cnt = 0;

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

static void canbus_remove_callbacks(canbus_t* canbus);

/******************************************************************************
* Definition  | Static Functions
******************************************************************************/

static void canbus_remove_callbacks(canbus_t* canbus)
{
	if(canbus->callbacks == NULL)
		return;
	__disable_irq();

	canbus_callback_t* next = NULL;
	canbus_callback_t *current = canbus->callbacks;

	while (current != NULL)
	{
		next = current->next;
		#if __has_include("FreeRTOS.h")
				vPortFree(current);
		#else
				free(current);
		#endif
		current = next;
	}

	canbus->callbacks = NULL;
	__enable_irq();
}

/******************************************************************************
* Definition  | Public Functions
******************************************************************************/


i_status canbus_initialize(canbus_t* canbus)
{
	__disable_irq();

	(void)HAL_FDCAN_DeactivateNotification(canbus->hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	(void)HAL_FDCAN_DeInit(canbus->hcan);

	__enable_irq();
	canbus->mx_init();

	if(canbus->filters_cnt != 0)
	{
		for(int i=0;i<canbus->filters_cnt;i++)
			if (HAL_FDCAN_ConfigFilter(canbus->hcan, &canbus->filters[i] ) != HAL_OK) goto canbus_initialize_error;
	}

	if (HAL_FDCAN_ConfigGlobalFilter(canbus->hcan,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE) != HAL_OK) goto canbus_initialize_error;
	if (HAL_FDCAN_Start(canbus->hcan) != HAL_OK) goto canbus_initialize_error;
	if (HAL_FDCAN_ActivateNotification(canbus->hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) goto canbus_initialize_error;
	if (HAL_FDCAN_ActivateNotification(canbus->hcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK) goto canbus_initialize_error;
	for(register uint32_t i=0;i<canbus_interfaces_cnt;i++)
		if(canbus_interfaces[i] == canbus)
			return I_OK;

	canbus_interfaces[canbus_interfaces_cnt] = canbus;
	canbus_interfaces_cnt++;

	return I_OK;
	canbus_initialize_error:
		return I_ERROR;
}

i_status canbus_send_plain(canbus_t* canbus, uint16_t fr_format, uint32_t id_type, uint32_t id, uint8_t dlc, uint8_t* data)
{
	uint32_t timeout = HAL_GetTick();
	uint8_t iterrations = 0;
	__disable_irq();

	TxHeader.Identifier = id;
	TxHeader.IdType = id_type == CBUS_ID_T_EXTENDED ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	if(dlc<=8)
	{
		TxHeader.DataLength = dlc * 0x00010000U;
	}
	else
	{
		if (dlc <= 12)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_12;
		}
		else if (dlc <= 16)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_16;
		}
		else if (dlc <= 20)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_20;
		}
		else if (dlc <= 24)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_24;
		}
		else if (dlc <= 32)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_32;
		}
		else if (dlc <= 48)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_48;
		}
		else
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_64;
		}
	}
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = fr_format == CBUS_FR_FRM_FD ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = TxHeader.MessageMarker > 3 ? 0 : TxHeader.MessageMarker+1;
	HAL_StatusTypeDef result;
	do
	{
		result = HAL_FDCAN_AddMessageToTxFifoQ(canbus->hcan, &TxHeader, data);
		iterrations++;
		if(iterrations == 0xFF)
		{
			__enable_irq();
			return I_ERROR;
		}
	}
	while(result != HAL_OK);
	__enable_irq();
	while( canbus->hcan->Instance->TXBRP!=0 )
	{
		__NOP();
		if((timeout + 3)< HAL_GetTick())
			return I_ERROR;
	}

	return result == HAL_OK ? I_OK :I_ERROR;
}

i_status canbus_send(canbus_t* canbus,canbus_frame_t* frame)
{
	uint32_t timeout = HAL_GetTick();
	uint8_t iterrations = 0;
	__disable_irq();
	TxHeader.Identifier = frame->id;
	TxHeader.IdType = frame->id_type == CBUS_ID_T_EXTENDED ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;

	if(frame->dlc<=8)
	{
		TxHeader.DataLength = frame->dlc * 0x00010000U;
	}
	else
	{
		if (frame->dlc <= 12)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_12;
		}
		else if (frame->dlc <= 16)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_16;
		}
		else if (frame->dlc <= 20)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_20;
		}
		else if (frame->dlc <= 24)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_24;
		}
		else if (frame->dlc <= 32)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_32;
		}
		else if (frame->dlc <= 48)
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_48;
		}
		else
		{
			TxHeader.DataLength = FDCAN_DLC_BYTES_64;
		}
	}

	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = frame->fr_format == CBUS_FR_FRM_FD ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = TxHeader.MessageMarker > 3 ? 0 : TxHeader.MessageMarker+1;
	HAL_StatusTypeDef result;
	do
	{
		result = HAL_FDCAN_AddMessageToTxFifoQ(canbus->hcan, &TxHeader, frame->dt);
		iterrations++;
		if(iterrations == 0xFF)
		{
			 __enable_irq();
			return I_ERROR;
		}
	}
	while(result != HAL_OK);
	__enable_irq();

	while( canbus->hcan->Instance->TXBRP!=0 )
	{
		__NOP();
		if((timeout + 3)< HAL_GetTick())
			return I_ERROR;
	}

	return result == HAL_OK ? I_OK :I_ERROR;
}

i_status canbus_callback_add(canbus_t* canbus,uint32_t id,uint32_t mask,uint32_t type, void(*cb)(canbus_frame_t*))
{
	__disable_irq();
	canbus_callback_t* node = (canbus_callback_t*)malloc(sizeof(canbus_callback_t));
	
	#if __has_include("FreeRTOS.h")
	if(node == NULL)
			node = (canbus_callback_t*)pvPortMalloc(sizeof(canbus_callback_t));
	#else
	if(node == NULL)
				node = (canbus_callback_t*)malloc(sizeof(canbus_callback_t));
	#endif

	if(node == NULL)
		goto canbus_callback_add_error;
	node->id = id;
	node->mask = mask;
	node->type = type;
	node->callback = cb;
	node->next = canbus->callbacks;
	canbus->callbacks = node;
	__enable_irq();
	return I_OK;
	canbus_callback_add_error:
	__enable_irq();
	return I_ERROR;
}

i_status canbus_callback_remove(canbus_t* canbus,canbus_callback_t* clb)
{
	if(canbus->callbacks == NULL)
		return I_NOTEXISTS;
	__disable_irq();

	canbus_callback_t *current;
	canbus_callback_t *to_remove;

	if(canbus->callbacks == clb)
	{
		current = canbus->callbacks;
		canbus->callbacks = current->next;
		#if __has_include("FreeRTOS.h")
				vPortFree(current);
		#else
				free(current);
		#endif
		__enable_irq();
		return I_OK;
	}

	current = canbus->callbacks;

	while (current != NULL)
	{
		if(current->next == clb)
		{
			to_remove = current->next;
			current->next = to_remove->next;
		#if __has_include("FreeRTOS.h")
				vPortFree(current);
		#else
				free(current);
		#endif
			__enable_irq();
			return I_OK;
		}
		current = current->next;
	}
	__enable_irq();
	return I_NOTEXISTS;
}

i_status canbus_callback_exists(canbus_t* canbus,canbus_callback_t* clb)
{
	if(canbus->callbacks == NULL)
		return I_NOTEXISTS;
	__disable_irq();

	canbus_callback_t *current = canbus->callbacks;

	while (current != NULL)
	{
		if(current == clb)
		{
			__enable_irq();
			return I_EXISTS;
		}
		current = current->next;
	}

	__enable_irq();
	return I_NOTEXISTS;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	static canbus_t* current_canbus = NULL;
	static FDCAN_RxHeaderTypeDef pRxHeader;
	static canbus_frame_t frame;
	static canbus_callback_t* callback_item = NULL;

	for(register uint32_t i=0;i<canbus_interfaces_cnt;i++)
		if(canbus_interfaces[i]->hcan == hfdcan)
			current_canbus = canbus_interfaces[i];

	if(current_canbus == NULL)
	{
		while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, frame.dt) == HAL_OK);
		return;
	}

	if(current_canbus->callbacks == NULL)
	{
		while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, frame.dt) == HAL_OK);
		return;
	}

	while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, frame.dt) == HAL_OK)
	{
		callback_item = current_canbus->callbacks;
		frame.id = pRxHeader.Identifier;
		switch(pRxHeader.DataLength)
		{
		case FDCAN_DLC_BYTES_0:
			frame.dlc = 0;
			break;
		case FDCAN_DLC_BYTES_1:
			frame.dlc = 1;
			break;
		case FDCAN_DLC_BYTES_2:
			frame.dlc = 2;
			break;
		case FDCAN_DLC_BYTES_3:
			frame.dlc = 3;
			break;
		case FDCAN_DLC_BYTES_4:
			frame.dlc = 4;
			break;
		case FDCAN_DLC_BYTES_5:
			frame.dlc = 5;
			break;
		case FDCAN_DLC_BYTES_6:
			frame.dlc = 6;
			break;
		case FDCAN_DLC_BYTES_7:
			frame.dlc = 7;
			break;
		case FDCAN_DLC_BYTES_8:
			frame.dlc = 8;
			break;
		case FDCAN_DLC_BYTES_12:
			frame.dlc = 12;
			break;
		case FDCAN_DLC_BYTES_16:
			frame.dlc = 16;
			break;
		case FDCAN_DLC_BYTES_20:
			frame.dlc = 20;
			break;
		case FDCAN_DLC_BYTES_24:
			frame.dlc = 24;
			break;
		case FDCAN_DLC_BYTES_32:
			frame.dlc = 32;
			break;
		case FDCAN_DLC_BYTES_48:
			frame.dlc = 48;
			break;
		case FDCAN_DLC_BYTES_64:
			frame.dlc = 64;
			break;
		}

		frame.id_type = pRxHeader.IdType == FDCAN_EXTENDED_ID ? CBUS_ID_T_EXTENDED : CBUS_ID_T_STANDARD;
		frame.fr_format = pRxHeader.FDFormat == FDCAN_FD_CAN ? CBUS_FR_FRM_FD : CBUS_FR_FRM_STD;

		while(callback_item!=NULL)
		{
			if(callback_item->type == pRxHeader.IdType)
			{
				if((callback_item->mask == 0 && (callback_item->id == pRxHeader.Identifier)) || (callback_item->mask!=0 && (callback_item->id & callback_item->mask) == (pRxHeader.Identifier & callback_item->mask)))
					callback_item->callback(&frame);
			}
			callback_item = callback_item->next;
		}
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	static canbus_t* current_canbus = NULL;

	for(register uint32_t i=0;i<canbus_interfaces_cnt;i++)
		if(canbus_interfaces[i]->hcan == hfdcan)
			current_canbus = canbus_interfaces[i];

	if(current_canbus != NULL)
	{
		__HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_BUS_OFF);
		canbus_initialize(current_canbus);
	}
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif
