/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2012-2015. All rights reserved.
 * foss@huawei.com
 *
 * If distributed as part of the Linux kernel, the following license terms
 * apply:
 *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 and
 * * only version 2 as published by the Free Software Foundation.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * * GNU General Public License for more details.
 * *
 * * You should have received a copy of the GNU General Public License
 * * along with this program; if not, write to the Free Software
 * * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Otherwise, the following license terms apply:
 *
 * * Redistribution and use in source and binary forms, with or without
 * * modification, are permitted provided that the following conditions
 * * are met:
 * * 1) Redistributions of source code must retain the above copyright
 * *    notice, this list of conditions and the following disclaimer.
 * * 2) Redistributions in binary form must reproduce the above copyright
 * *    notice, this list of conditions and the following disclaimer in the
 * *    documentation and/or other materials provided with the distribution.
 * * 3) Neither the name of Huawei nor the names of its contributors may
 * *    be used to endorse or promote products derived from this software
 * *    without specific prior written permission.
 *
 * * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MDRV_EICC_COMMON_H__
#define __MDRV_EICC_COMMON_H__
#ifdef __cplusplus
extern "C"
{
#endif

#define EICC_CHANNEL_ID_MAKEUP(channel_id, func_id) ((channel_id << 16) | (func_id))

#ifndef BSP_ERR_EICC_BASE
#define BSP_ERR_EICC_BASE               (int)(0x80000000 | 0x10180000)
#endif

/* C核发生复位 */
#ifndef BSP_ERR_EICC_CCORE_RESETTING
#define BSP_ERR_EICC_CCORE_RESETTING    (BSP_ERR_EICC_BASE + 0x12)
#endif

/* CPU ID 分配 */
enum EICC_CPU_ID
{
    EICC_CPU_MIN = 0,

    EICC_CPU_APP = 0,
    EICC_CPU_LPM3 = 1,
    EICC_CPU_NRCCPU = 2,
    EICC_CPU_R8HL1C0 = 3,
    EICC_CPU_R8HL1C1 = 4,
    EICC_CPU_L2HAC0 = 5,
    EICC_CPU_L2HAC1 = 6,
    EICC_CPU_LL1C0 = 7,
    EICC_CPU_LL1C1 = 8,
    EICC_CPU_LL1C2 = 9,
    EICC_CPU_LL1C3 = 10,
    EICC_CPU_LL1C4 = 11,
    EICC_CPU_LL1C5 = 12,
    EICC_CPU_LL1C6 = 13,
    EICC_CPU_LL1C7 = 14,
    EICC_CPU_LRMCCPU = 15,

    EICC_CPU_MAX
};

/* 错误码定义 */
enum EICC_ERR_NO
{
    EICC_CHN_INIT_FAIL = (0x80000000 + (0 << 16)),
    EICC_MALLOC_CHANNEL_FAIL,
    EICC_MALLOC_VECTOR_FAIL,
    EICC_CREATE_TASK_FAIL,
    EICC_DEBUG_INIT_FAIL,
    EICC_CREATE_SEM_FAIL,
    EICC_REGISTER_INT_FAIL,
    EICC_INVALID_PARA,
    EICC_WAIT_SEM_TIMEOUT,
    EICC_SEND_ERR,
    EICC_RECV_ERR,
    EICC_REGISTER_CB_FAIL,
    EICC_REGISTER_DPM_FAIL,
    EICC_MALLOC_MEM_FAIL,
    EICC_NULL_PTR,
    EICC_INIT_ADDR_TOO_BIG,
    EICC_INIT_SKIP,
    EICC_CHANNEL_OPEN_FAIL,
    EICC_CHANNEL_CLOSE_FAIL,
    EICC_REGISTER_DYNAMIC_CB_FAIL,
    EICC_DYNAMIC_SEND_FAIL,
    EICC_INVALID_NO_FIFO_SPACE  /* 通道满 */
};

typedef int (*eicc_msg_cb)(unsigned int channel_id, unsigned int len); /*len:接收消息长度,写回调时len无意义*/

enum EICC_READ_INT_LEV
{
    EICC_RD_FLDN_INT = 0x1,
    EICC_RD_FLUP_INT = 0x2,
    EICC_RD_DATA_ARV_INT = 0x4,
    EICC_RD_DATA_ARV_TIMEOUT_INT = 0x8,
    EICC_RD_PIPE_WAKEUP_INT = 0x10
};

enum EICC_WRITE_INT_LEV
{
    EICC_WR_FLDN_INT = 0x1,
    EICC_WR_FLUP_INT = 0x2,
    EICC_WR_DATA_DONE_INT = 0x4,
    EICC_WR_PIPE_ERR_INT = 0x8,
    EICC_WR_TRANS_CNT_OVF_INT = 0x10
};

enum CHANL_TYPE
{
    CHANNEL_OUTSIDE_SEND = 0x10, /*外部发送通道*/
    CHANNEL_OUTSIDE_RECV = 0x11, /*外部接收通道*/
    CHANNEL_INSIDE_SEND = 0x12,  /*内部发送通道*/
    CHANNEL_INSIDE_RECV = 0x13,  /*内部接收通道*/  
    CHANNEL_OUTDMA_SEND = 0x14,  /*外部DMA发送通道*/
    CHANNEL_OUTDMA_RECV = 0x15,  /*外部DMA接收通道*/
    CHANNEL_INSIDE_DMA = 0x16    /*内部DMA通道*/
};

/* 通道id分配 */
/* nx EICC DEV 枚举 DEV0~DEV3*/
enum EICC_DEV0_CHN_ID
{
    EICC_SDR_CHN_MIN = 0,            /* EICC_SDR最小通道数 */

    EICC_SDR_SEND_LL1C_DL_0 = 0,     /* LL1C DL与SDR的原语交互 */
    EICC_SDR_SEND_LL1C_DL_1,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_SEND_LL1C_DL_2,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_SEND_LL1C_DL_3,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_SEND_HL1C,              /* HL1C与SDR的原语交互 */

    EICC_SDR_RECV_LL1C_DL_0,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_RECV_LL1C_DL_1,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_RECV_LL1C_DL_2,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_RECV_LL1C_DL_3,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_RECV_HL1C,              /* HL1C与SDR的原语交互 */

    EICC_SDR_RECV_HL1C_OM,           /* HL1C与SDR的OM消息*/

    EICC_SDR_SEND_LL1C_DL_4,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_SEND_LL1C_DL_5,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_SEND_LL1C_DL_6,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_SEND_LL1C_DL_7,         /* LL1C DL与SDR的原语交互 */
    
    EICC_SDR_RECV_LL1C_DL_4,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_RECV_LL1C_DL_5,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_RECV_LL1C_DL_6,         /* LL1C DL与SDR的原语交互 */
    EICC_SDR_RECV_LL1C_DL_7,         /* LL1C DL与SDR的原语交互 */

    EICC_SDR_SEND_HL1C_1,            /* HL1C与SDR的原语交互 */
    EICC_SDR_RECV_HL1C_1,            /* HL1C与SDR的原语交互 */

#ifdef UPHY_DRV_EICC_TEST_MODE
    EICC_SDR_DMA_TEST,
    EICC_SDR_OUTSIDE_DMA_TEST,
#endif
    EICC_SDR_CHN_ID_MAX
};

enum EICC_DEV1_CHN_ID
{
    EICC_HL1C_CHN_MIN = 0x1000,  /* EICC_HL1C_DL最小通道数 */

    EICC_HL1C_SEND_LL1C_DL_0 = 0x1000,/* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_SEND_LL1C_DL_1,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_SEND_LL1C_DL_2,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_SEND_LL1C_DL_3,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_SEND_SDR,              /* HL1C与SDR的原语交互 */

    EICC_HL1C_RECV_LL1C_DL_0,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_RECV_LL1C_DL_1,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_RECV_LL1C_DL_2,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_RECV_LL1C_DL_3,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_RECV_SDR,              /* HL1C与SDR的原语交互 */

    EICC_HL1C_SEND_NRCCPU_ULMBX,
    EICC_HL1C_RECV_NRCCPU_DLMBX,
    EICC_HL1C_RECV_NRCCPU_OM,

    EICC_HL1C_SEND_SDR_OM,           /* HL1C与SDR的OM消息*/
    EICC_HL1C_SEND_LL1C_DL_OM_0,     /* HL1C与LL1C DL的OM消息*/
    EICC_HL1C_SEND_LL1C_DL_OM_1,     /* HL1C与LL1C DL的OM消息*/
    EICC_HL1C_SEND_LL1C_DL_OM_2,     /* HL1C与LL1C DL的OM消息*/
    EICC_HL1C_SEND_LL1C_DL_OM_3,     /* HL1C与LL1C DL的OM消息*/
    EICC_HL1C_SEND_LL1C_UL_OM_0,     /* HL1C与LL1C UL的OM消息*/
    EICC_HL1C_SEND_LL1C_UL_OM_1,     /* HL1C与LL1C UL的OM消息*/

    EICC_HL1C_SEND_LL1C_UL_0,        /* HL1C发消息到两个ULL1C核，共2*2=4个*/
    EICC_HL1C_SEND_LL1C_UL_1,        /* HL1C发消息到两个ULL1C核，共2*2=4个*/
    EICC_HL1C_SEND_LL1C_UL_2,        /* HL1C发消息到两个ULL1C核，共2*2=4个*/
    EICC_HL1C_SEND_LL1C_UL_3,        /* HL1C发消息到两个ULL1C核，共2*2=4个*/
    
    EICC_HL1C_RECV_LL1C_UL_0,        /* HL1C接收消息到两个ULL1C核，共2*2=4个(松/紧时序) */
    EICC_HL1C_RECV_LL1C_UL_1,        /* HL1C接收消息到两个ULL1C核，共2*2=4个(松/紧时序) */
    EICC_HL1C_RECV_LL1C_UL_2,        /* HL1C接收消息到两个ULL1C核，共2*2=4个(松/紧时序) */
    EICC_HL1C_RECV_LL1C_UL_3,        /* HL1C接收消息到两个ULL1C核，共2*2=4个(松/紧时序) */

    EICC_HL1C_SEND_LL1C_UL_SP_0,     /* 专有邮箱做Beam Pool镜像，两个ULL1C核，共2个 */
    EICC_HL1C_SEND_LL1C_UL_SP_1,     /* 专有邮箱做Beam Pool镜像，两个ULL1C核，共2个 */

    EICC_HL1C_SEND_BBIC_SEQ_DMA,     /* HL1C读写BBIC SEQ, EICC DMA方式，每个核两个，共1*2个 */
    EICC_HL1C_RECV_BBIC_SEQ_DMA,     /* HL1C读写BBIC SEQ, EICC DMA方式，每个核两个，共1*2个 */

    EICC_HL1C_SEND_LL1C_DL_4,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_SEND_LL1C_DL_5,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_SEND_LL1C_DL_6,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_SEND_LL1C_DL_7,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_RECV_LL1C_DL_4,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_RECV_LL1C_DL_5,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_RECV_LL1C_DL_6,        /* LL1C DL与HL1C的原语交互 */
    EICC_HL1C_RECV_LL1C_DL_7,        /* LL1C DL与HL1C的原语交互 */

    EICC_HL1C_SEND_SDR_1,            /* HL1C与SDR的原语交互 */
    EICC_HL1C_RECV_SDR_1,            /* HL1C与SDR的原语交互 */
    
#ifdef UPHY_DRV_EICC_TEST_MODE
    EICC_HL1C_OUTSIDE_DMA_TEST,
#endif
    EICC_HL1C_CHN_ID_MAX
};

enum EICC_DEV2_CHN_ID
{
    EICC_LL1C_DL_CHN_MIN = 0x2000,   /* EICC_LL1C最小通道数 */

    EICC_LL1C_DL_SEND_SDR_0 = 0x2000,/* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_SEND_SDR_1,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_SEND_SDR_2,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_SEND_SDR_3,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_SEND_HL1C_0,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_SEND_HL1C_1,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_SEND_HL1C_2,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_SEND_HL1C_3,       /* LL1C DL与HL1C的原语交互 */

    EICC_LL1C_DL_RECV_SDR_0,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_RECV_SDR_1,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_RECV_SDR_2,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_RECV_SDR_3,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_RECV_HL1C_0,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_RECV_HL1C_1,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_RECV_HL1C_2,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_RECV_HL1C_3,       /* LL1C DL与HL1C的原语交互 */

    EICC_LL1C_DL_RECV_HL1C_OM_0,    /* HL1C与LL1C DL的OM消息*/
    EICC_LL1C_DL_RECV_HL1C_OM_1,    /* HL1C与LL1C DL的OM消息*/
    EICC_LL1C_DL_RECV_HL1C_OM_2,    /* HL1C与LL1C DL的OM消息*/
    EICC_LL1C_DL_RECV_HL1C_OM_3,    /* HL1C与LL1C DL的OM消息*/

    EICC_LL1C_DL_0_SEND_LL1C_UL_0,  /* 4个DL1C发消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_1_SEND_LL1C_UL_0,  /* 4个DL1C发消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_2_SEND_LL1C_UL_0,  /* 4个DL1C发消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_3_SEND_LL1C_UL_0,  /* 4个DL1C发消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_0_SEND_LL1C_UL_1,  /* 4个DL1C发消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_1_SEND_LL1C_UL_1,  /* 4个DL1C发消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_2_SEND_LL1C_UL_1,  /* 4个DL1C发消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_3_SEND_LL1C_UL_1,  /* 4个DL1C发消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_0_RECV_LL1C_UL_0,  /* 4个DL1C接收消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_1_RECV_LL1C_UL_0,  /* 4个DL1C接收消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_2_RECV_LL1C_UL_0,  /* 4个DL1C接收消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_3_RECV_LL1C_UL_0,  /* 4个DL1C接收消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_0_RECV_LL1C_UL_1,  /* 4个DL1C接收消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_1_RECV_LL1C_UL_1,  /* 4个DL1C接收消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_2_RECV_LL1C_UL_1,  /* 4个DL1C接收消息到两个ULL1C核，共8个 */
    EICC_LL1C_DL_3_RECV_LL1C_UL_1,  /* 4个DL1C接收消息到两个ULL1C核，共8个 */

    EICC_LL1C_DL_SEND_SDR_4,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_SEND_SDR_5,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_SEND_SDR_6,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_SEND_SDR_7,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_RECV_SDR_4,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_RECV_SDR_5,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_RECV_SDR_6,        /* LL1C DL与SDR的原语交互 */
    EICC_LL1C_DL_RECV_SDR_7,        /* LL1C DL与SDR的原语交互 */

    EICC_LL1C_DL_SEND_HL1C_4,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_SEND_HL1C_5,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_SEND_HL1C_6,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_SEND_HL1C_7,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_RECV_HL1C_4,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_RECV_HL1C_5,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_RECV_HL1C_6,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_DL_RECV_HL1C_7,       /* LL1C DL与HL1C的原语交互 */
    EICC_LL1C_UL_1_RECV_LL1C_UL_0,  /* 两个ULL1C核相互接收消息，共2个 */
    EICC_LL1C_UL_0_RECV_LL1C_UL_1,  /* 两个ULL1C核相互接收消息，共2个 */

    EICC_LL1C_DL_CHN_ID_MAX
};

enum EICC_DEV3_CHN_ID
{
    EICC_LL1C_UL_CHN_MIN = 0x3000,        /* EICC_LL1C_UL最小通道数 */

    EICC_NRCCPU_RECV_HL1C_ULMBX = 0x3000, /* PC模拟临时增加 */
    EICC_NRCCPU_SEND_HL1C_DLMBX,      /* PC模拟临时增加 */
    EICC_NRCCPU_SEND_HL1C_OM,         /* PC模拟临时增加 */

    EICC_LL1C_UL_RECV_HL1C_OM_0,      /* LL1C UL与HL1C的OM消息*/
    EICC_LL1C_UL_RECV_HL1C_OM_1,      /* LL1C UL与HL1C的OM消息*/

    EICC_LL1C_UL_SEND_HL1C_0,         /* 两个ULL1C核发消息到HL1C，共2*2=4个*/
    EICC_LL1C_UL_SEND_HL1C_1,         /* 两个ULL1C核发消息到HL1C，共2*2=4个*/
    EICC_LL1C_UL_SEND_HL1C_2,         /* 两个ULL1C核发消息到HL1C，共2*2=4个*/
    EICC_LL1C_UL_SEND_HL1C_3,         /* 两个ULL1C核发消息到HL1C，共2*2=4个*/

    EICC_LL1C_UL_RECV_HL1C_0,         /* 两个ULL1C核接收消息到HL1C，共2*2=4个*/
    EICC_LL1C_UL_RECV_HL1C_1,         /* 两个ULL1C核接收消息到HL1C，共2*2=4个*/
    EICC_LL1C_UL_RECV_HL1C_2,         /* 两个ULL1C核接收消息到HL1C，共2*2=4个*/
    EICC_LL1C_UL_RECV_HL1C_3,         /* 两个ULL1C核接收消息到HL1C，共2*2=4个*/

    EICC_LL1C_UL_0_SEND_LL1C_DL_0,    /* 两个ULL1C核发消息到4个DL1C，共8个 */
    EICC_LL1C_UL_0_SEND_LL1C_DL_1,    /* 两个ULL1C核发消息到4个DL1C，共8个 */
    EICC_LL1C_UL_0_SEND_LL1C_DL_2,    /* 两个ULL1C核发消息到4个DL1C，共8个 */
    EICC_LL1C_UL_0_SEND_LL1C_DL_3,    /* 两个ULL1C核发消息到4个DL1C，共8个 */
    EICC_LL1C_UL_1_SEND_LL1C_DL_0,    /* 两个ULL1C核发消息到4个DL1C，共8个 */
    EICC_LL1C_UL_1_SEND_LL1C_DL_1,    /* 两个ULL1C核发消息到4个DL1C，共8个 */
    EICC_LL1C_UL_1_SEND_LL1C_DL_2,    /* 两个ULL1C核发消息到4个DL1C，共8个 */
    EICC_LL1C_UL_1_SEND_LL1C_DL_3,    /* 两个ULL1C核发消息到4个DL1C，共8个 */
    EICC_LL1C_UL_0_RECV_LL1C_DL_0,    /* 两个ULL1C核接收消息到4个DL1C，共8个 */
    EICC_LL1C_UL_0_RECV_LL1C_DL_1,    /* 两个ULL1C核接收消息到4个DL1C，共8个 */
    EICC_LL1C_UL_0_RECV_LL1C_DL_2,    /* 两个ULL1C核接收消息到4个DL1C，共8个 */
    EICC_LL1C_UL_0_RECV_LL1C_DL_3,    /* 两个ULL1C核接收消息到4个DL1C，共8个 */
    EICC_LL1C_UL_1_RECV_LL1C_DL_0,    /* 两个ULL1C核接收消息到4个DL1C，共8个 */
    EICC_LL1C_UL_1_RECV_LL1C_DL_1,    /* 两个ULL1C核接收消息到4个DL1C，共8个 */
    EICC_LL1C_UL_1_RECV_LL1C_DL_2,    /* 两个ULL1C核接收消息到4个DL1C，共8个 */
    EICC_LL1C_UL_1_RECV_LL1C_DL_3,    /* 两个ULL1C核接收消息到4个DL1C，共8个 */

    EICC_LL1C_UL_0_SEND_LL1C_UL_1,    /* 两个ULL1C核相互发送消息，共2个 */
    EICC_LL1C_UL_1_SEND_LL1C_UL_0,    /* 两个ULL1C核相互发送消息，共2个 */

    EICC_LL1C_UL_0_RECV_HL1C_SP,      /* 专有邮箱做Beam Pool镜像，两个ULL1C核，共2个*/
    EICC_LL1C_UL_1_RECV_HL1C_SP,      /* 专有邮箱做Beam Pool镜像，两个ULL1C核，共2个*/

    EICC_LL1C_UL_0_SEND_BBIC_SEQ_DMA, /* LL1C读写BBIC SEQ, EICC DMA方式，每个核两个，共2*2个 */
    EICC_LL1C_UL_1_SEND_BBIC_SEQ_DMA, /* LL1C读写BBIC SEQ, EICC DMA方式，每个核两个，共2*2个 */
    EICC_LL1C_UL_0_RECV_BBIC_SEQ_DMA, /* LL1C读写BBIC SEQ, EICC DMA方式，每个核两个，共2*2个 */
    EICC_LL1C_UL_1_RECV_BBIC_SEQ_DMA, /* LL1C读写BBIC SEQ, EICC DMA方式，每个核两个，共2*2个 */
    
    EICC_LL1C_UL_CHN_ID_MAX
};

/* NRCCPU EICC DEV DEV0~DEV1*/
enum EICC_NRCCPU_DEV0_CHN_ID
{
    EICC_NRCCPU_DEV0_CHN_ID_MIN = 0x4000,

    EICC_NRCCPU_SEND_AP_IFC_0 = 0x4000,  /* nrccpu和ap之间的共享IFC的发送通道 */   
    EICC_NRCCPU_SEND_AP_IFC_1,           /* nrccpu和ap之间的共享IFC的发送通道 */    
    EICC_NRCCPU_SEND_AP_IFC_2,           /* nrccpu和ap之间的共享IFC的发送通道 */   
    EICC_NRCCPU_SEND_AP_IFC_3,           /* nrccpu和ap之间的共享IFC的发送通道 */   
    EICC_NRCCPU_RECV_AP_IFC_0,           /* nrccpu和ap之间的共享IFC的接收通道 */
    EICC_NRCCPU_RECV_AP_IFC_1,           /* nrccpu和ap之间的共享IFC的接收通道 */
    EICC_NRCCPU_RECV_AP_IFC_2,           /* nrccpu和ap之间的共享IFC的接收通道 */
    EICC_NRCCPU_RECV_AP_IFC_3,           /* nrccpu和ap之间的共享IFC的接收通道 */
    EICC_NRCCPU_SEND_LRCCPU_0,           /* nrccpu和lrccpu之间的专有发送通道 */
    EICC_NRCCPU_RECV_LRCCPU_0,           /* nrccpu和lrccpu之间的专有接收通道 */
    EICC_NRCCPU_SEND_LRCCPU_IFC_0,       /* nrccpu和lrccpu之间的共享发送通道 */
    EICC_NRCCPU_RECV_LRCCPU_IFC_0,       /* nrccpu和lrccpu之间的共享接收通道 */

    EICC_NRCCPU_DEV0_CHN_ID_MAX
};

enum EICC_NRCCPU_DEV1_CHN_ID
{
    EICC_NRCCPU_DEV1_CHN_ID_MIN     = 0x5000,

    EICC_NRCCPU_SEND_L2HAC_DTRANS_0 = 0x5000,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_1,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_2,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_3,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_4,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_5,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_6,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_7,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_8,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_9,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_10,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_11,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_12,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_13,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_14,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_15,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_16,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_17,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_18,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_19,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_20,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_21,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_22,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_23,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_24,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_25,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_26,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_27,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_28,
    EICC_NRCCPU_SEND_L2HAC_DTRANS_29,
    EICC_NRCCPU_SEND_L2HAC_NRLC_BSR,
    EICC_NRCCPU_RECV_L2HAC_NRLC_REL_PDU,
    EICC_NRCCPU_SEND_L2HAC_NPDCP_COUNT,

    EICC_NRCCPU_SEND_L2HAC_IFC_0,           /* 5G实时任务->非实时任务共享通道 */
    EICC_NRCCPU_RECV_L2HAC_IFC_0,           /* 5G非实时任务->实时任务共享通道 */

    EICC_NRCCPU_SEND_HL1C_OSA_0,
    EICC_NRCCPU_RECV_HL1C_OSA_0,
    EICC_NRCCPU_SEND_L2HAC_OSA_0,
    EICC_NRCCPU_RECV_L2HAC_OSA_0,

    EICC_NRCCPU_SEND_AP_OSA_0,
    EICC_NRCCPU_RECV_AP_OSA_0,
    EICC_NRCCPU_SEND_LRCCPU_OSA_0,
    EICC_NRCCPU_RECV_LRCCPU_OSA_0,

    EICC_NRCCPU_DEV1_CHN_ID_MAX
};

/* LRCCPU EICC DEV */
enum EICC_LRCCPU_DEV_CHN_ID
{
    EICC_LRCCPU_CHN_ID_MIN = 0x6000,        /* EICC LRCCPU最小通道数 */

    EICC_LRCCPU_SEND_NRCCPU_IFC_0 = 0x6000, /* lrccpu和nrccpu之间的共享发送通道 */ 
    EICC_LRCCPU_RECV_NRCCPU_IFC_0,          /* lrccpu和nrccpu之间的共享发送通道 */ 
    EICC_LRCCPU_SEND_NRCCPU_0,                /* lrccpu和nrccpu之间的专有发送通道 */
    EICC_LRCCPU_RECV_NRCCPU_0,                /* lrccpu和nrccpu之间的专有接收通道 */
    EICC_LRNX_SEND_L2HAC_IFC_0,             /* LTE<->NR调度信息交互共享通道 */
    EICC_LRNX_RECV_L2HAC_IFC_0,             /* LTE<->NR调度信息交互共享通道 */
    EICC_LRCCPU_SEND_NRCCPU_OSA_0,
    EICC_LRCCPU_RECV_NRCCPU_OSA_0,

    EICC_LRCCPU_CHN_ID_MAX
};

/* HAC EICC DEV DEV */
enum EICC_L2HAC_DEV0_ID_ENUM
{
    EICC_L2HAC_DEV0_CHN_ID_MIN      = 0x7000,

    EICC_L2HAC_RECV_NRCCPU_DTRANS_0 = 0x7000,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_1,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_2,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_3,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_4,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_5,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_6,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_7,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_8,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_9,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_10,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_11,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_12,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_13,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_14,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_15,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_16,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_17,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_18,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_19,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_20,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_21,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_22,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_23,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_24,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_25,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_26,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_27,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_28,
    EICC_L2HAC_RECV_NRCCPU_DTRANS_29,
    EICC_L2HAC_RECV_NRCCPU_NRLC_BSR,
    EICC_L2HAC_SEND_NRCCPU_NRLC_REL_PDU,
    EICC_L2HAC_RECV_NRCCPU_NPDCP_COUNT,
    
    EICC_L2HAC_SEND_LRNX_IFC_0,       /* LTE<->NR调度信息交互共享通道 */
    EICC_L2HAC_RECV_LRNX_IFC_0,       /* LTE<->NR调度信息交互共享通道 */
    EICC_L2HAC_SEND_NRCCPU_IFC_0,     /* 5G实时任务->非实时任务公共消息 */
    EICC_L2HAC_RECV_NRCCPU_IFC_0,     /* 5G非实时任务->实时任务公共消息 */

    EICC_L2HAC_SEND_NRCCPU_OSA_0,
    EICC_L2HAC_RECV_NRCCPU_OSA_0,

    EICC_L2HAC_DEV0_CHN_ID_MAX
};

enum EICC_L2HAC_DEV1_ID_ENUM
{
    EICC_L2HAC_DEV1_CHN_ID_MIN  = 0x8000,

    EICC_L2HAC_SEND_HL1C_URGENT = 0x8000,
    EICC_L2HAC_SEND_HL1C_NORMAL,
    EICC_L2HAC_RECV_HL1C_URGENT,
    EICC_L2HAC_RECV_HL1C_NORMAL,
    EICC_L2HAC_SEND_LL1C_UL_0,
    EICC_L2HAC_SEND_LL1C_UL_1,
    EICC_L2HAC_RECV_LL1C_UL_0_URGENT,
    EICC_L2HAC_RECV_LL1C_UL_0_NORMAL,
    EICC_L2HAC_RECV_LL1C_UL_1_URGENT,
    EICC_L2HAC_RECV_LL1C_UL_1_NORMAL,
    EICC_L2HAC_RECV_LL1C_DL_1,
    EICC_L2HAC_RECV_LL1C_DL_2,
    EICC_L2HAC_RECV_LL1C_DL_3,
    EICC_L2HAC_RECV_LL1C_DL_4,

    EICC_L2HAC_DEV1_CHN_ID_MAX
};

/* AP EICC DEV DEV */
enum EICC_AP_DEV_CHN_ID
{
    EICC_AP_CHN_ID_MIN = 0x9000,
    
    EICC_AP_SEND_NRCCPU_IFC_0 = 0x9000, /* nrccpu和ap之间的共享IFC的发送通道 */   
    EICC_AP_SEND_NRCCPU_IFC_1,          /* nrccpu和ap之间的共享IFC的发送通道 */    
    EICC_AP_SEND_NRCCPU_IFC_2,          /* nrccpu和ap之间的共享IFC的发送通道 */   
    EICC_AP_SEND_NRCCPU_IFC_3,          /* nrccpu和ap之间的共享IFC的发送通道 */   

    EICC_AP_RECV_NRCCPU_IFC_0,          /* nrccpu和ap之间的共享IFC的接收通道 */
    EICC_AP_RECV_NRCCPU_IFC_1,          /* nrccpu和ap之间的共享IFC的接收通道 */
    EICC_AP_RECV_NRCCPU_IFC_2,          /* nrccpu和ap之间的共享IFC的接收通道 */
    EICC_AP_RECV_NRCCPU_IFC_3,          /* nrccpu和ap之间的共享IFC的接收通道 */

    EICC_AP_SEND_NRCCPU_OSA_0,
    EICC_AP_RECV_NRCCPU_OSA_0,

    EICC_AP_CHN_ID_MAX
};

/* HAC EICC DEV */
enum EICC_HL1C_1_DEV_CHN_ID
{
    EICC_HL1C_1_CHN_ID_MIN = 0xa000,
    
    EICC_HL1C_SEND_NRCCPU_OSA_0 = 0xa000,
    EICC_HL1C_RECV_NRCCPU_OSA_0, 

    EICC_HL1C_1_CHN_ID_MAX
};

/* 说明: 接收回调函数ID,子通道必须放置在, "通道名称_xxx=0和通道名称_RECV_FUNC_ID_MAX之间
 */
enum EICC_LL1C_UL_SEND_LL1C_UL_FUNC_ID
{
    LL1C_UL_SEND_LL1C_UL_FUNC_TEST1 = 0,


    /* 若要在EICC_CHN_IFC物理通道上定义子通道,请在该注释行之前定义 */
    LL1C_UL_SEND_LL1C_UL_FUNC_ID_MAX,
};

enum FLOWLINE_UP
{
    FLUP_ONE_ARV = 0x0,        /*一个数据块或则一个消息*/
    FLUP_HALF = 0x1,           /*1/2深度*/
    FLUP_THREE_FOURTHS = 0x2,  /*3/4深度*/
    FLUP_SEVEN_EIGHTHS = 0x3,  /*7/8深度*/  
};

enum FLOWLINE_DN
{
    FLDN_EMPTY = 0x0,        /*空则报下水线*/
    FLDN_ONE_EIGHTHS = 0x1,  /*1/8深度*/
    FLDN_ONE_FOURTHS = 0x2,  /*1/4深度*/
    FLDN_HALF = 0x3,         /*1/2深度*/  
};

enum ALIGN_MODE
{
    ALIGN_4BYTES = 0x0,   /*4字节对齐*/
    ALIGN_16BYTES = 0x1,  /*16字节对齐*/
    ALIGN_32BYTES = 0x2,  /*32字节对齐*/
};

enum MSG_DISTRIBUTE
{
    IS_DISTRIBUTE = 0x0,   /*驱动按注册的子通道回调分发消息*/
    NOT_DISTRIBUTE = 0x1,  /*通道内所有消息报到子通道0回调上*/
};

struct eicc_chan_attr
{
    enum CHANL_TYPE     type;           /*通道类型*/
    enum MSG_DISTRIBUTE distribute;     /*是否按子通道分发消息*/
    enum ALIGN_MODE     align_mode;     /*消息头和消息体对齐方式，默认芯片约束4字节对齐，非默认配置时ram_addr和ram_size对齐方式必须保持一致*/
    eicc_msg_cb         eicc_cb;        /*发送完成或消息接收回调*/
    void *              ram_addr;       /*注册本地pipe空间地址，默认需配8字节对齐，align_mode非默认配置时需与其保持一致*/
    unsigned int        ram_size;       /*注册本地pipe空间大小，默认需配8字节对齐，align_mode非默认配置时需与其保持一致*/
    unsigned int        packet_len_max; /*接收包的最大长度(发送通道配置0)，默认需配8字节对齐，align_mode非默认配置时需与其保持一致*/
};

struct eicc_currmsg_info
{
    unsigned int channel_id;            /* 当前数据包记录的通道id（物理通道和子通道组成）*/
    unsigned int msg_len;               /* 当前数据包长度 */
    unsigned int time_stamp;            /* 当前数据包发送的时间戳 */
    unsigned int rptr_offset;           /* 当前数据包消息读指针偏移 */
    unsigned int is_continuous;         /* 消息是否连续。1:连续；0:不连续 */
};

/************************************************************************
 * 函 数 名  : mdrv_icc_open
 * 功能描述  :
 * 输入参数  :
 *            channel_id: ICC 逻辑通道号
 *            p_chan_attr: ICC 通道属性，函数内只用其值
 * 输出参数  : 无
 * 返 回 值  :  0          操作成功。
 *             其他        操作失败。
 **************************************************************************/
int mdrv_eicc_open(unsigned int channel_id, struct eicc_chan_attr *p_chan_attr);

/************************************************************************
 * 函 数 名  : mdrv_icc_read
 * 功能描述  :
 * 输入参数  :
 *            u32ChanId: ICC 逻辑通道号
 *            pData:     数据存放地址
 *            s32Size:   回调函数返回的数据长度
 * 输出参数  : pData       数据
 * 返 回 值  : 正值        数据长度。
 *             其他        操作失败。
 **************************************************************************/
int mdrv_eicc_read(unsigned int channel_id, unsigned char *msg_ptr, unsigned int msg_len);


/************************************************************************
 * 函 数 名  : mdrv_icc_write
 * 功能描述  :
 * 输入参数  :
 *            u32ChanId: ICC 逻辑通道号
 *            pData:     数据存放地址
 *            s32Size:   数据长度
 * 输出参数  : 无
 * 返 回 值  : 与s32Size相等  操作成功。
 *             其他           操作失败。
 **************************************************************************/
int mdrv_eicc_write(unsigned int channel_id, unsigned char *msg_ptr, unsigned int msg_len, unsigned int is_arv_int);

/************************************************************************
 * 函 数 名  : mdrv_icc_close
 * 功能描述  :
 * 输入参数  :
 *            channel_id: ICC 逻辑通道号
 * 输出参数  : 无
 * 返 回 值  :  0          操作成功。
 *             其他        操作失败。
 **************************************************************************/
int mdrv_eicc_close(unsigned int channel_id);

/************************************************************************
 * 函 数 名  : mdrv_icc_read_int_enable
 * 功能描述  :
 * 输入参数  :
 *            channel_id: ICC 逻辑通道号
 * 输出参数  : 无
 * 返 回 值  :  0          操作成功。
 *             其他        操作失败。
 **************************************************************************/
int mdrv_eicc_read_int_enable(unsigned int channel_id, enum EICC_READ_INT_LEV level);

/************************************************************************
 * 函 数 名  : mdrv_icc_read_int_disable
 * 功能描述  :
 * 输入参数  :
 *            channel_id: ICC 逻辑通道号
 * 输出参数  : 无
 * 返 回 值  :  0          操作成功。
 *             其他        操作失败。
 **************************************************************************/
int mdrv_eicc_read_int_disable(unsigned int channel_id, enum EICC_READ_INT_LEV level);


int mdrv_eicc_currmsg_header_get(unsigned int phy_channel_id, struct eicc_currmsg_info *currmsg_info);


int mdrv_eicc_rptr_get(unsigned int channel_id, unsigned int *rptr_offset);


int mdrv_eicc_read_done(unsigned int channel_id, unsigned int rptr_offset);

/* 协议栈eicc接口打桩 */
struct eicc_dtrans_bd
{
    unsigned int  msg_len     :16;/*[0-15]*/
    unsigned int  reserved1   :6; /*[16-21]*/
    unsigned int  I_pos       :1; /*[22]I*/
    unsigned int  reserved2   :9; /*[23-31]*/
};

enum ETRANS_INT_LEV
{
    ETRANS_RD_FLDN_INT = 0x1,
    ETRANS_RD_FLUP_INT = 0x2,
    ETRANS_RD_DATA_ARV_INT = 0x4,
    ETRANS_RD_DATA_ARV_TIMEOUT_INT = 0x8,
    ETRANS_RD_PIPE_WAKEUP_INT = 0x10,
    ETRANS_WR_FLDN_INT = 0x100,
    ETRANS_WR_FLUP_INT = 0x200,
    ETRANS_WR_DATA_DONE_INT = 0x400,
    ETRANS_WR_PIPE_ERR_INT = 0x800,
    ETRANS_WR_TRANS_CNT_OVF_INT = 0x1000
};

enum ETRANS_PIPE_STATUS
{
    ETRANS_PIPE_STATUS_IDLE = 0x0,
    ETRANS_PIPE_STATUS_BUSY = 0x1
};

typedef int (*eicc_dtrans_cb)(unsigned int channel_id, enum ETRANS_INT_LEV int_type, unsigned int len);
struct eicc_dtrans_chan_attr
{
    enum    CHANL_TYPE      type;            /*通道类型*/
    enum    FLOWLINE_UP     flup;            /*上水线配置*/
    enum    FLOWLINE_DN     fldn;            /*下水线配置*/
    enum    ETRANS_INT_LEV  etans_int;       /*中断配置*/
    eicc_dtrans_cb          eicc_cb;         /*发送完成或消息接收回调*/
    void *                  ram_addr;        /*注册本地pipe空间*/
    unsigned int            ram_size;
    unsigned int            packet_len_max;
};

int mdrv_eicc_dtrans_open(unsigned int channel_id, struct eicc_dtrans_chan_attr *p_chan_attr);

int mdrv_eicc_dtrans_close(unsigned int channel_id);

void mdrv_eicc_dtrans_optr_get(unsigned int channel_id,unsigned int  *wptr_offset,unsigned int *rptr_offset);

void mdrv_eicc_dtrans_wptr_set(unsigned int channel_id,unsigned int  wptr_offset);

void mdrv_eicc_dtrans_iptr_get(unsigned int channel_id,unsigned int  *wptr_offset, unsigned int  *rptr_offset);

void mdrv_eicc_dtrans_rptr_set(unsigned int channel_id,unsigned int  rptr_offset);

enum ETRANS_PIPE_STATUS mdrv_eicc_dtrans_pipe_status_get(unsigned int channel_id);

#ifdef __cplusplus
}
#endif
#endif
