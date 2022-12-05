/*!
	@file   drv_canbus.h
	@brief  <brief description here>
	@t.odo	-
	---------------------------------------------------------------------------

	MIT License
	Copyright (c) 2019 Ioannis Deligiannis

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

#ifdef CANBUS_HAL_FDCAN

#ifndef DRV_CANBUS_VFDCAN_H_
#define DRV_CANBUS_VFDCAN_H_

/******************************************************************************
* Includes
******************************************************************************/

#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#if __has_include("FreeRTOS.h")
#include "FreeRTOS.h"
#endif

#if __has_include("task.h")
#include "task.h"
#endif

#if __has_include("cmsis_os.h")
#include "cmsis_os.h"
#endif

#if __has_include("fdcan.h")
#include "fdcan.h"
#define DRV_CANBUS_ENABLED

/******************************************************************************
* Enumerations, structures & Variables
******************************************************************************/

#if !defined(ENUM_I_STATUS)
#define ENUM_I_STATUS
typedef enum
{
	I_OK 			= 0x00,
	I_INVALID 		= 0x01,
	I_EXISTS 		= 0x02,
	I_NOTEXISTS 		= 0x03,
	I_FAILED 		= 0x04,
	I_EXPIRED 		= 0x05,
	I_UNKNOWN 		= 0x06,
	I_INPROGRESS 		= 0x07,
	I_IDLE			= 0x08,
	I_FULL			= 0x09,
	I_EMPTY			= 0x0A,
	I_YES			= 0x0B,
	I_NO			= 0x0C,
	I_SKIP			= 0x0D,
	I_LOCKED 		= 0x0E,
	I_INACTIVE 		= 0x0F,
	I_ACTIVE 		= 0x10,
	I_READY		 	= 0x11,
	I_WAIT 			= 0x12,
	I_OVERFLOW 		= 0x13,
	I_CONTINUE 		= 0x14,
	I_STOPPED 		= 0x15,
	I_WARNING 		= 0x16,
	I_SLEEP 		= 0x17,
	I_DEEPSLEEP 		= 0x18,
	I_STANDBY 		= 0x19,
	I_GRANTED 		= 0x1A,
	I_DENIED 		= 0x1B,
	I_DEBUG_01 		= 0xE0,
	I_DEBUG_02 		= 0xE1,
	I_DEBUG_03 		= 0xE2,
	I_DEBUG_04 		= 0xE3,
	I_DEBUG_05 		= 0xE4,
	I_DEBUG_06 		= 0xE5,
	I_DEBUG_07 		= 0xE6,
	I_DEBUG_08 		= 0xE7,
	I_DEBUG_09 		= 0xE8,
	I_DEBUG_10 		= 0xE9,
	I_DEBUG_11 		= 0xEA,
	I_DEBUG_12 		= 0xEB,
	I_DEBUG_13 		= 0xEC,
	I_DEBUG_14 		= 0xED,
	I_DEBUG_15 		= 0xEE,
	I_DEBUG_16 		= 0xEF,
	I_MEMALIGNED		= 0xFC,
	I_MEMUNALIGNED		= 0xFD,
	I_NOTIMPLEMENTED 	= 0xFE,
	I_ERROR 		= 0xFF
}i_status;
#endif

/* --- CANTP Mode - Frame Type (ref: x) ------------------------------------- */

#ifndef CBUS_FR_FORMAT
#define CBUS_FR_FORMAT
typedef enum
{
	CBUS_FR_FRM_STD = 0x01,		/* Standard CANBUS */
	CBUS_FR_FRM_FD  = 0x02		/* FD CANBUS       */
}cbus_fr_format;
#endif

/* --- CANBus Mode [ID Type] (ref: iso15765-2 p.8) ------------------------- */

#ifndef CBUS_ID_TYPE
#define CBUS_ID_TYPE
typedef enum
{
	CBUS_ID_T_STANDARD = 0x04U,	/* 11bits CAN Identifier */
	CBUS_ID_T_EXTENDED = 0x08U	/* 29bits CAN Identifier */
}cbus_id_type;
#endif

/* --- CANBus Frame (ref: iso15765-2 p.) ----------------------------------- */

#ifndef CANBUS_FRAME
#define CANBUS_FRAME
typedef struct
{
	uint32_t id;		/* CAN Frame Id */
	uint32_t id_type;	/* CAN Frame Id Type `cbus_id_type` */
	uint16_t fr_format;	/* CAN Frame Format `cbus_fr_format` */
	uint16_t dlc;		/* Size of data */
	uint8_t dt[64];		/* Actual data of the frame */
}canbus_frame_t;
#endif

struct canbus_callback
{
	uint32_t id;
	uint32_t mask;
	uint32_t type;
	void (*callback)(canbus_frame_t*);
	struct canbus_callback *next;
};

typedef struct canbus_callback canbus_callback_t;

typedef struct
{
	void (*mx_init)();
	FDCAN_HandleTypeDef *hcan;
	FDCAN_FilterTypeDef *filters;
	uint8_t filters_cnt;
	canbus_callback_t * callbacks;
}canbus_t;

/******************************************************************************
* Declaration | Public Functions
******************************************************************************/

i_status canbus_initialize(canbus_t* canbus);
i_status canbus_send(canbus_t* canbus, canbus_frame_t* frame);
i_status canbus_send_plain(canbus_t* canbus, uint16_t fr_format, uint32_t id_type, uint32_t id, uint8_t dlc, uint8_t* data);
i_status canbus_callback_add(canbus_t* canbus, uint32_t id, uint32_t mask,uint32_t type, void(*cb)(canbus_frame_t*));
i_status canbus_callback_remove(canbus_t* canbus, canbus_callback_t* clb);
i_status canbus_callback_exists(canbus_t* canbus, canbus_callback_t* clb);
void canbus_recover_if_needs(canbus_t* canbus);

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#else
	#warning "Missing: `fdcan.h` file in the parent folder. Please use the example"
#endif
#endif
#endif
