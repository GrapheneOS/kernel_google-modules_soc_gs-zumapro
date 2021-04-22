/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS GS201 ITP Interrupt And Event Defines
 *
 * Copyright (c) 2021 Google, LLC
 */

#ifndef DT_BINDINGS_LWIS_PLATFORM_GS201_ITP_H_
#define DT_BINDINGS_LWIS_PLATFORM_GS201_ITP_H_

#include <dt-bindings/lwis/platform/common.h>

/* clang-format off */

#define ITP_ITP_INT0_BASE (HW_EVENT_MASK + 0)

#define ITP_ITP_INT0_FRAME_START 0
#define ITP_ITP_INT0_FRAME_END 1
#define ITP_ITP_INT0_CMDQ_HOLD 2
#define ITP_ITP_INT0_SETTING_DONE 3
#define ITP_ITP_INT0_C_LOADER_END 4
#define ITP_ITP_INT0_COREX_END0 5
#define ITP_ITP_INT0_COREX_END1 6
#define ITP_ITP_INT0_ROW_COL 7
#define ITP_ITP_INT0_FREEZE_ON_ROW_COL 8
#define ITP_ITP_INT0_TRANS_STOP_DONE 9
#define ITP_ITP_INT0_CMDQ_ERROR 10
#define ITP_ITP_INT0_C_LOADER_ERROR 11
#define ITP_ITP_INT0_COREX_ERROR 12
#define ITP_ITP_INT0_CINFIFO0_OVERFLOW_ERROR 13
#define ITP_ITP_INT0_CINFIFO0_OVERLAP_ERROR 14
#define ITP_ITP_INT0_CINFIFO0_PIXEL_CNT_ERROR 15
#define ITP_ITP_INT0_CINFIFO0_INPUT_PROTOCOL_ERROR 16
#define ITP_ITP_INT0_CINFIFO1_OVERFLOW_ERROR 17
#define ITP_ITP_INT0_CINFIFO1_OVERLAP_ERROR 18
#define ITP_ITP_INT0_CINFIFO1_PIXEL_CNT_ERROR 19
#define ITP_ITP_INT0_CINFIFO1_INPUT_PROTOCOL_ERROR 20
#define ITP_ITP_INT0_COUTFIFO0_PIXEL_CNT_ERROR 21
#define ITP_ITP_INT0_COUTFIFO0_INPUT_PROTOCOL_ERROR 22
#define ITP_ITP_INT0_COUTFIFO0_OVERFLOW_ERROR 23

#define ITP_ITP_CMDQ_INT_BASE (HW_EVENT_MASK + 32)

#define ITP_ITP_CMDQ_INT_STOP_CRPT_OFF_CMDQ_EN 0
#define ITP_ITP_CMDQ_INT_PRELOAD_FLUSH 1
#define ITP_ITP_CMDQ_INT_QUE0_OVERFLOW 2

#define ITP_DNS_INT0_BASE (HW_EVENT_MASK + 64)

#define ITP_DNS_INT0_FRAME_START 0
#define ITP_DNS_INT0_FRAME_END 1
#define ITP_DNS_INT0_CMDQ_HOLD 2
#define ITP_DNS_INT0_SETTING_DONE 3
#define ITP_DNS_INT0_C_LOADER_END 4
#define ITP_DNS_INT0_COREX_END0 5
#define ITP_DNS_INT0_COREX_END1 6
#define ITP_DNS_INT0_ROW_COL 7
#define ITP_DNS_INT0_FREEZE_ON_ROW_COL 8
#define ITP_DNS_INT0_TRANS_STOP_DONE 9
#define ITP_DNS_INT0_CMDQ_ERROR 10
#define ITP_DNS_INT0_C_LOADER_ERROR 11
#define ITP_DNS_INT0_COREX_ERROR 12
#define ITP_DNS_INT0_CINFIFO0_OVERFLOW_ERROR 13
#define ITP_DNS_INT0_CINFIFO0_OVERLAP_ERROR 14
#define ITP_DNS_INT0_CINFIFO0_PIXEL_CNT_ERROR 15
#define ITP_DNS_INT0_CINFIFO0_INPUT_PROTOCOL_ERROR 16
#define ITP_DNS_INT0_COUTFIFO0_PIXEL_CNT_ERROR 21
#define ITP_DNS_INT0_COUTFIFO0_INPUT_PROTOCOL_ERROR 22
#define ITP_DNS_INT0_COUTFIFO0_OVERFLOW_ERROR 23
#define ITP_DNS_INT0_COUTFIFO1_PIXEL_CNT_ERROR 24
#define ITP_DNS_INT0_COUTFIFO1_INPUT_PROTOCOL_ERROR 25
#define ITP_DNS_INT0_COUTFIFO1_OVERFLOW_ERROR 26
#define ITP_DNS_INT0_VOTF_GLOBAL_ERROR 27
#define ITP_DNS_INT0_VOTF_LOST_CONNECTION 28
#define ITP_DNS_INT0_OTF_SEQ_ID_ERROR 29

#define ITP_DNS_INT1_BASE (HW_EVENT_MASK + 96)

#define ITP_DNS_INT1_SBWC_ERR 0
#define ITP_DNS_INT1_VOTF_SLOW_RING 1
#define ITP_DNS_INT1_VOTF_LOST_CONNECTION 2
#define ITP_DNS_INT1_VOTF_LOST_FLUSH 3
#define ITP_DNS_INT1_COUTFIFO2_PIXEL_CNT_ERROR 4
#define ITP_DNS_INT1_COUTFIFO2_INPUT_PROTOCOL_ERROR 5
#define ITP_DNS_INT1_COUTFIFO2_OVERFLOW_ERROR 6

#define ITP_DNS_CMDQ_INT_BASE (HW_EVENT_MASK + 128)

#define ITP_DNS_CMDQ_INT_STOP_CRPT_OFF_CMDQ_EN 0
#define ITP_DNS_CMDQ_INT_PRELOAD_FLUSH 1
#define ITP_DNS_CMDQ_INT_QUE0_OVERFLOW 2

#define ITP_ITSC_INT0_BASE (HW_EVENT_MASK + 160)

#define ITP_ITSC_INT0_FRAME_START 0
#define ITP_ITSC_INT0_FRAME_END 1
#define ITP_ITSC_INT0_CMDQ_HOLD 2
#define ITP_ITSC_INT0_SETTING_DONE 3
#define ITP_ITSC_INT0_C_LOADER_END 4
#define ITP_ITSC_INT0_COREX_END0 5
#define ITP_ITSC_INT0_COREX_END1 6
#define ITP_ITSC_INT0_ROW_COL 7
#define ITP_ITSC_INT0_FREEZE_ON_ROW_COL 8
#define ITP_ITSC_INT0_TRANS_STOP_DONE 9
#define ITP_ITSC_INT0_CMDQ_ERROR 10
#define ITP_ITSC_INT0_C_LOADER_ERROR 11
#define ITP_ITSC_INT0_COREX_ERROR 12
#define ITP_ITSC_INT0_CINFIFO0_OVERFLOW_ERROR 13
#define ITP_ITSC_INT0_CINFIFO0_OVERLAP_ERROR 14
#define ITP_ITSC_INT0_CINFIFO0_PIXEL_CNT_ERROR 15
#define ITP_ITSC_INT0_CINFIFO0_INPUT_PROTOCOL_ERROR 16
#define ITP_ITSC_INT0_CINFIFO1_OVERFLOW_ERROR 17
#define ITP_ITSC_INT0_CINFIFO1_OVERLAP_ERROR 18
#define ITP_ITSC_INT0_CINFIFO1_PIXEL_CNT_ERROR 19
#define ITP_ITSC_INT0_CINFIFO1_INPUT_PROTOCOL_ERROR 20
#define ITP_ITSC_INT0_COUTFIFO0_PIXEL_CNT_ERROR 21
#define ITP_ITSC_INT0_COUTFIFO0_INPUT_PROTOCOL_ERROR 22
#define ITP_ITSC_INT0_COUTFIFO0_OVERFLOW_ERROR 23
#define ITP_ITSC_INT0_COUTFIFO1_PIXEL_CNT_ERROR 24
#define ITP_ITSC_INT0_COUTFIFO1_INPUT_PROTOCOL_ERROR 25
#define ITP_ITSC_INT0_COUTFIFO1_OVERFLOW_ERROR 26
#define ITP_ITSC_INT0_VOTF_GLOBAL_ERROR 27
#define ITP_ITSC_INT0_VOTF_LOST_CONNECTION 28
#define ITP_ITSC_INT0_OTF_SEQ_ID_ERROR 29

#define ITP_ITSC_INT1_BASE (HW_EVENT_MASK + 192)

#define ITP_ITSC_INT1_VOTF_LOST_FLUSH 0

#define ITP_ITSC_CMDQ_INT_BASE (HW_EVENT_MASK + 224)

#define ITP_ITSC_CMDQ_INT_STOP_CRPT_OFF_CMDQ_EN 0
#define ITP_ITSC_CMDQ_INT_PRELOAD_FLUSH 1
#define ITP_ITSC_CMDQ_INT_QUE0_OVERFLOW 2

/* clang-format on */

#define LWIS_PLATFORM_EVENT_ID_ITP_FRAME_START \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_FRAME_START)
#define LWIS_PLATFORM_EVENT_ID_ITP_FRAME_END \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_FRAME_END)
#define LWIS_PLATFORM_EVENT_ID_ITP_CMDQ_HOLD \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_CMDQ_HOLD)
#define LWIS_PLATFORM_EVENT_ID_ITP_SETTING_DONE \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_SETTING_DONE)
#define LWIS_PLATFORM_EVENT_ID_ITP_C_LOADER_END \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_C_LOADER_END)
#define LWIS_PLATFORM_EVENT_ID_ITP_COREX_END0 \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_COREX_END0)
#define LWIS_PLATFORM_EVENT_ID_ITP_COREX_END1 \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_COREX_END1)
#define LWIS_PLATFORM_EVENT_ID_ITP_ROW_COL \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		ITP_ITP_INT0_ROW_COL)
#define LWIS_PLATFORM_EVENT_ID_ITP_FREEZE_ON_ROW_COL \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_FREEZE_ON_ROW_COL)
#define LWIS_PLATFORM_EVENT_ID_ITP_TRANS_STOP_DONE \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_TRANS_STOP_DONE)
#define LWIS_PLATFORM_EVENT_ID_ITP_CMDQ_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_CMDQ_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_C_LOADER_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_C_LOADER_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COREX_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_COREX_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_OVERFLOW_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_CINFIFO0_OVERFLOW_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_OVERLAP_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_CINFIFO0_OVERLAP_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_CINFIFO0_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_CINFIFO0_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO1_OVERFLOW_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_CINFIFO1_OVERFLOW_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO1_OVERLAP_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_CINFIFO1_OVERLAP_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO1_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_CINFIFO1_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO1_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_CINFIFO1_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO0_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_COUTFIFO0_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO0_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_COUTFIFO0_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO0_OVERFLOW_ERROR \
	EVENT_ID(ITP_ITP_INT0_BASE, \
		 ITP_ITP_INT0_COUTFIFO0_OVERFLOW_ERROR)

#define LWIS_PLATFORM_EVENT_ID_ITP_STOP_CRPT_OFF_CMDQ_EN \
	EVENT_ID(ITP_ITP_CMDQ_INT_BASE, \
		 ITP_ITP_CMDQ_INT_STOP_CRPT_OFF_CMDQ_EN)
#define LWIS_PLATFORM_EVENT_ID_ITP_PRELOAD_FLUSH \
	EVENT_ID(ITP_ITP_CMDQ_INT_BASE, \
		 ITP_ITP_CMDQ_INT_PRELOAD_FLUSH)
#define LWIS_PLATFORM_EVENT_ID_ITP_QUE0_OVERFLOW \
	EVENT_ID(ITP_ITP_CMDQ_INT_BASE, \
		 ITP_ITP_CMDQ_INT_QUE0_OVERFLOW)

#define LWIS_PLATFORM_EVENT_ID_ITP_FRAME_START \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_FRAME_START)
#define LWIS_PLATFORM_EVENT_ID_ITP_FRAME_END \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_FRAME_END)
#define LWIS_PLATFORM_EVENT_ID_ITP_CMDQ_HOLD \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_CMDQ_HOLD)
#define LWIS_PLATFORM_EVENT_ID_ITP_SETTING_DONE \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_SETTING_DONE)
#define LWIS_PLATFORM_EVENT_ID_ITP_C_LOADER_END \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_C_LOADER_END)
#define LWIS_PLATFORM_EVENT_ID_ITP_COREX_END0 \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_COREX_END0)
#define LWIS_PLATFORM_EVENT_ID_ITP_COREX_END1 \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_COREX_END1)
#define LWIS_PLATFORM_EVENT_ID_ITP_ROW_COL \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_ROW_COL)
#define LWIS_PLATFORM_EVENT_ID_ITP_FREEZE_ON_ROW_COL \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_FREEZE_ON_ROW_COL)
#define LWIS_PLATFORM_EVENT_ID_ITP_TRANS_STOP_DONE \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_TRANS_STOP_DONE)
#define LWIS_PLATFORM_EVENT_ID_ITP_CMDQ_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_CMDQ_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_C_LOADER_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_C_LOADER_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COREX_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_COREX_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_OVERFLOW_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_CINFIFO0_OVERFLOW_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_OVERLAP_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_CINFIFO0_OVERLAP_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_CINFIFO0_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_CINFIFO0_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO0_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_COUTFIFO0_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO0_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_COUTFIFO0_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO0_OVERFLOW_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_COUTFIFO0_OVERFLOW_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO1_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_COUTFIFO1_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO1_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_COUTFIFO1_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO1_OVERFLOW_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_COUTFIFO1_OVERFLOW_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_VOTF_GLOBAL_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_VOTF_GLOBAL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_VOTF_LOST_CONNECTION \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_VOTF_LOST_CONNECTION)
#define LWIS_PLATFORM_EVENT_ID_ITP_OTF_SEQ_ID_ERROR \
	EVENT_ID(ITP_DNS_INT0_BASE, \
		 ITP_DNS_INT0_OTF_SEQ_ID_ERROR)

#define LWIS_PLATFORM_EVENT_ID_ITP_SBWC_ERR \
	EVENT_ID(ITP_DNS_INT1_BASE, \
		 ITP_DNS_INT1_SBWC_ERR)
#define LWIS_PLATFORM_EVENT_ID_ITP_VOTF_SLOW_RING \
	EVENT_ID(ITP_DNS_INT1_BASE, \
		 ITP_DNS_INT1_VOTF_SLOW_RING)
#define LWIS_PLATFORM_EVENT_ID_ITP_VOTF_LOST_CONNECTION \
	EVENT_ID(ITP_DNS_INT1_BASE, \
		 ITP_DNS_INT1_VOTF_LOST_CONNECTION)
#define LWIS_PLATFORM_EVENT_ID_ITP_VOTF_LOST_FLUSH \
	EVENT_ID(ITP_DNS_INT1_BASE, \
		 ITP_DNS_INT1_VOTF_LOST_FLUSH)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO2_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_DNS_INT1_BASE, \
		 ITP_DNS_INT1_COUTFIFO2_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO2_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_DNS_INT1_BASE, \
		 ITP_DNS_INT1_COUTFIFO2_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO2_OVERFLOW_ERROR \
	EVENT_ID(ITP_DNS_INT1_BASE, \
		 ITP_DNS_INT1_COUTFIFO2_OVERFLOW_ERROR)

#define LWIS_PLATFORM_EVENT_ID_ITP_STOP_CRPT_OFF_CMDQ_EN \
	EVENT_ID(ITP_DNS_CMDQ_INT_BASE, \
		 ITP_DNS_CMDQ_INT_STOP_CRPT_OFF_CMDQ_EN)
#define LWIS_PLATFORM_EVENT_ID_ITP_PRELOAD_FLUSH \
	EVENT_ID(ITP_DNS_CMDQ_INT_BASE, \
		 ITP_DNS_CMDQ_INT_PRELOAD_FLUSH)
#define LWIS_PLATFORM_EVENT_ID_ITP_QUE0_OVERFLOW \
	EVENT_ID(ITP_DNS_CMDQ_INT_BASE, \
		 ITP_DNS_CMDQ_INT_QUE0_OVERFLOW)

#define LWIS_PLATFORM_EVENT_ID_ITP_FRAME_START \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_FRAME_START)
#define LWIS_PLATFORM_EVENT_ID_ITP_FRAME_END \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_FRAME_END)
#define LWIS_PLATFORM_EVENT_ID_ITP_CMDQ_HOLD \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_CMDQ_HOLD)
#define LWIS_PLATFORM_EVENT_ID_ITP_SETTING_DONE \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_SETTING_DONE)
#define LWIS_PLATFORM_EVENT_ID_ITP_C_LOADER_END \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_C_LOADER_END)
#define LWIS_PLATFORM_EVENT_ID_ITP_COREX_END0 \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_COREX_END0)
#define LWIS_PLATFORM_EVENT_ID_ITP_COREX_END1 \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_COREX_END1)
#define LWIS_PLATFORM_EVENT_ID_ITP_ROW_COL \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_ROW_COL)
#define LWIS_PLATFORM_EVENT_ID_ITP_FREEZE_ON_ROW_COL \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_FREEZE_ON_ROW_COL)
#define LWIS_PLATFORM_EVENT_ID_ITP_TRANS_STOP_DONE \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_TRANS_STOP_DONE)
#define LWIS_PLATFORM_EVENT_ID_ITP_CMDQ_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_CMDQ_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_C_LOADER_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_C_LOADER_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COREX_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_COREX_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_OVERFLOW_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_CINFIFO0_OVERFLOW_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_OVERLAP_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_CINFIFO0_OVERLAP_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_CINFIFO0_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO0_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_CINFIFO0_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO1_OVERFLOW_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_CINFIFO1_OVERFLOW_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO1_OVERLAP_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_CINFIFO1_OVERLAP_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO1_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_CINFIFO1_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_CINFIFO1_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_CINFIFO1_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO0_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_COUTFIFO0_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO0_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_COUTFIFO0_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO0_OVERFLOW_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_COUTFIFO0_OVERFLOW_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO1_PIXEL_CNT_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_COUTFIFO1_PIXEL_CNT_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO1_INPUT_PROTOCOL_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_COUTFIFO1_INPUT_PROTOCOL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_COUTFIFO1_OVERFLOW_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_COUTFIFO1_OVERFLOW_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_VOTF_GLOBAL_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_VOTF_GLOBAL_ERROR)
#define LWIS_PLATFORM_EVENT_ID_ITP_VOTF_LOST_CONNECTION \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_VOTF_LOST_CONNECTION)
#define LWIS_PLATFORM_EVENT_ID_ITP_OTF_SEQ_ID_ERROR \
	EVENT_ID(ITP_ITSC_INT0_BASE, \
		 ITP_ITSC_INT0_OTF_SEQ_ID_ERROR)

#define LWIS_PLATFORM_EVENT_ID_ITP_VOTF_LOST_FLUSH \
	EVENT_ID(ITP_ITSC_INT1_BASE, \
		 ITP_ITSC_INT1_VOTF_LOST_FLUSH)

#define LWIS_PLATFORM_EVENT_ID_ITP_STOP_CRPT_OFF_CMDQ_EN \
	EVENT_ID(ITP_ITSC_CMDQ_INT_BASE, \
		 ITP_ITSC_CMDQ_INT_STOP_CRPT_OFF_CMDQ_EN)
#define LWIS_PLATFORM_EVENT_ID_ITP_PRELOAD_FLUSH \
	EVENT_ID(ITP_ITSC_CMDQ_INT_BASE, \
		 ITP_ITSC_CMDQ_INT_PRELOAD_FLUSH)
#define LWIS_PLATFORM_EVENT_ID_ITP_QUE0_OVERFLOW \
	EVENT_ID(ITP_ITSC_CMDQ_INT_BASE, \
		 ITP_ITSC_CMDQ_INT_QUE0_OVERFLOW)

#endif /* DT_BINDINGS_LWIS_PLATFORM_GS201_ITP_H_ */