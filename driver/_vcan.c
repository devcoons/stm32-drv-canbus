/*!
	@file   drv_canbus_leg.c
	@brief  <brief description here>
	@t.odo	-
	---------------------------------------------------------------------------

	MIT License
	Copyright (c) 2022 Federico Carnevale, Ioannis Deligiannis

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
#ifdef CANBUS_HAL_CAN
#ifdef DRV_CANBUS_ENABLED

/******************************************************************************
* Enumerations, structures & Variables
******************************************************************************/

static CAN_TxHeaderTypeDef TxHeader;
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

	(void)HAL_CAN_DeactivateNotification(canbus->hcan, CAN_IT_RX_FIFO0_FULL);
	(void)HAL_CAN_DeInit(canbus->hcan);

	__enable_irq();
	canbus->mx_init();

	if(canbus->filters_cnt != 0)
	{
		for(int i=0;i<canbus->filters_cnt;i++)
			if (HAL_CAN_ConfigFilter(canbus->hcan, &canbus->filters[i] ) != HAL_OK) goto canbus_initialize_error;
	}


	if (HAL_CAN_Start(canbus->hcan) != HAL_OK) goto canbus_initialize_error;
	if (HAL_CAN_ActivateNotification(canbus->hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) goto canbus_initialize_error;
	if (HAL_CAN_ActivateNotification(canbus->hcan, CAN_IT_RX_FIFO0_FULL) != HAL_OK) goto canbus_initialize_error;
	if (HAL_CAN_ActivateNotification(canbus->hcan, CAN_IT_BUSOFF) != HAL_OK) goto canbus_initialize_error;
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
	uint32_t       TxMailbox;
	__disable_irq();

	if(id_type == CBUS_ID_T_EXTENDED)
	{
		TxHeader.ExtId = id;
		TxHeader.IDE = CAN_ID_EXT;
	}
	else
	{
		TxHeader.StdId = id;
		TxHeader.IDE = CAN_ID_STD;
	}

	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = dlc;

	if(dlc>8)
	{
		return I_ERROR;
	}

	HAL_StatusTypeDef result;
	do
	{
		result = HAL_CAN_AddTxMessage(canbus->hcan, &TxHeader, data,&TxMailbox);
		iterrations++;
		if(iterrations == 0xFF)
		{
			__enable_irq();
			return I_ERROR;
		}
	}
	while(result != HAL_OK);
	__enable_irq();

	return result == HAL_OK ? I_OK :I_ERROR;
}

i_status canbus_send(canbus_t* canbus,canbus_frame_t* frame)
{
	uint32_t timeout = HAL_GetTick();
	uint8_t iterrations = 0;
	uint32_t       TxMailbox;
	__disable_irq();

	if(frame->id_type == CBUS_ID_T_EXTENDED)
	{
		TxHeader.ExtId = frame->id;
		TxHeader.IDE = CAN_ID_EXT;
	}
	else
	{
		TxHeader.StdId = frame->id;
		TxHeader.IDE = CAN_ID_STD;
	}
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = frame->dlc;

	HAL_StatusTypeDef result;
	do
	{
		result = HAL_CAN_AddTxMessage(canbus->hcan, &TxHeader, frame->dt,&TxMailbox);
		iterrations++;
		if(iterrations == 0xFF)
		{
			 __enable_irq();
			return I_ERROR;
		}
	}
	while(result != HAL_OK);
	__enable_irq();


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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static canbus_t* current_canbus = NULL;
	static CAN_RxHeaderTypeDef pRxHeader;
	static canbus_frame_t frame;
	static canbus_callback_t* callback_item = NULL;

	for(register uint32_t i=0;i<canbus_interfaces_cnt;i++)
		if(canbus_interfaces[i]->hcan == hcan)
			current_canbus = canbus_interfaces[i];

	if(current_canbus == NULL)
	{
		while(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, frame.dt) == HAL_OK);
		return;
	}
	if(current_canbus->callbacks == NULL)
	{
		while(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, frame.dt) == HAL_OK);
		return;
	}
	while(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, frame.dt) == HAL_OK)
	{
		callback_item = current_canbus->callbacks;
		frame.id =pRxHeader.IDE == CAN_ID_STD ?  pRxHeader.StdId :  pRxHeader.ExtId;
		frame.dlc = pRxHeader.DLC;
		frame.id_type = pRxHeader.IDE == CAN_ID_EXT ? CBUS_ID_T_EXTENDED : CBUS_ID_T_STANDARD;
		frame.fr_format =  CBUS_FR_FRM_STD;

		while(callback_item!=NULL)
		{
   			if(callback_item->type == (pRxHeader.IDE == CAN_ID_EXT ? CBUS_ID_T_EXTENDED : CBUS_ID_T_STANDARD))
			{
				if((callback_item->mask == 0 && (callback_item->id == (pRxHeader.IDE == CAN_ID_STD ?  pRxHeader.StdId :  pRxHeader.ExtId)))
				  || (callback_item->mask!=0 && (callback_item->id & callback_item->mask) == ((pRxHeader.IDE == CAN_ID_STD ?  pRxHeader.StdId :  pRxHeader.ExtId) & callback_item->mask)))
					callback_item->callback(&frame);
			}
			callback_item = callback_item->next;
		}
	}
}


void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	static canbus_t* current_canbus = NULL;

	for(register uint32_t i=0;i<canbus_interfaces_cnt;i++)
		if(canbus_interfaces[i]->hcan == hcan)
			current_canbus = canbus_interfaces[i];

	if(current_canbus != NULL)
	{

		//__HAL_CAN_CLEAR_FLAG(hcan, FDCAN_FLAG_BUS_OFF);
		canbus_initialize(current_canbus);
	}
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif
#endif
