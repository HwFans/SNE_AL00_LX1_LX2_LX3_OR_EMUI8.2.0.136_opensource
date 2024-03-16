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


#ifndef __MSP_DIAG_COMM_H__
#define __MSP_DIAG_COMM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "product_config.h"
#include "vos.h"
#include "mdrv.h"

/**************************************************************************
  �궨��
**************************************************************************/

/*���PS��MODID ��DIAG_AIR_MSG_LOG_ID�ĵط����滻��DIAG_ID*/
#define DIAG_ID(module_id, log_type)   (VOS_UINT32)(module_id | (log_type << 12))

#define DIAG_AIR_MSG_LOG_ID(module_id, is_up_link)  DIAG_ID(module_id, is_up_link) /*module_id��ӦPID*/

#define DIAG_GEN_LOG_MODULE(modemid, modetype, level)  \
               (((VOS_UINT32)(modemid & 0xff) << 24)  \
              | ((VOS_UINT32)(modetype & 0xf) << 16)  \
              | ((VOS_UINT32)(level    & 0xf ) << 12))

#define DIAG_GEN_MODULE(modemid, modetype)  \
               (((VOS_UINT32)(modemid & 0xff) << 24)  \
              | ((VOS_UINT32)(modetype & 0xf) << 16))


#define DIAG_GEN_MODULE_EX(modemid, modetype, groupid)  \
                   (((VOS_UINT32)(modemid & 0xff) << 24)  \
                  | ((VOS_UINT32)(modetype & 0xf) << 16) \
                  | ((VOS_UINT32)(groupid  & 0xf)  << 8))

#define DIAG_GEN_LOG_ID(filenum, linenum)   \
                ((((VOS_UINT32)(filenum & 0XFFFF)) << 16)   \
                | ((VOS_UINT32)(linenum & 0XFFFF)))

/*is_up_linkȡֵ*/
#define OS_MSG_UL                (0x01)/* ��ʾ������Ϣ*/
#define OS_MSG_DL                (0x02)/* ��ʾ������Ϣ*/


#define DIAG_SIDE_UE             (0x1)  /* ��ʾUE���յĿտ���Ϣ��NET-->UE*/
#define DIAG_SIDE_NET            (0x2)  /* ��ʾNET���յĿտ���Ϣ��UE-->NET*/

/* �¼���Ϣ��ӡ������*/
#define DIAG_LOG_TYPE_INFO            0x00000008UL
#define DIAG_LOG_TYPE_AUDIT_FAILURE   0x00000004UL
#define DIAG_LOG_TYPE_AUDIT_SUCCESS   0x00000002UL
#define DIAG_LOG_TYPE_ERROR           0x00000001UL
#define DIAG_LOG_TYPE_WARNING         0x00000010UL

/*ԭ������diag_message_id.h��,*/
#define ID_MSG_DIAG_HSO_DISCONN_IND                 (0x00010004)
#define ID_MSG_DIAG_CHR_REQ                         (0x00010010)

/*diag AGENT����PSģ���HSO�ط�����*/
#define ID_MSG_DIAG_CMD_REPLAY_TO_PS    			(0x00010100)

/* diag ���͸�������ϵͳ�Ľ������� */
#define ID_MSG_DIAG_CMD_CONNECT_REQ    		        (0x00010200)
#define ID_MSG_DIAG_CMD_CONNECT_CNF    		        (ID_MSG_DIAG_CMD_CONNECT_REQ)
/* diag ���͸�������ϵͳ�Ķ������� */
#define ID_MSG_DIAG_CMD_DISCONNECT_REQ    		    (0x00010202)
#define ID_MSG_DIAG_CMD_DISCONNECT_CNF    		    (ID_MSG_DIAG_CMD_DISCONNECT_REQ)
/* diag ���͸�������ϵͳ�Ķ������� */
#define ID_MSG_DIAG_CMD_CHANN_DISCONN_REQ    		(0x00010203)
#define ID_MSG_DIAG_CMD_CHANN_DISCONN_CNF    		(ID_MSG_DIAG_CMD_CHANN_DISCONN_REQ)

/* diag ���͸�TL-PHY����Ϣ�������� */
#define ID_MSG_DIAG_DSP_CONNECT_REQ    		        (0x30004903)
#define ID_MSG_DIAG_DSP_CONNECT_CNF    		        (ID_MSG_DIAG_DSP_CONNECT_REQ)
/* diag ���͸�TL-PHY����Ϣ�������� */
#define ID_MSG_DIAG_DSP_DISCONNECT_REQ    		    (0x30004904)
#define ID_MSG_DIAG_DSP_DISCONNECT_CNF    		    (ID_MSG_DIAG_DSP_DISCONNECT_REQ)

/* ������Ϣ�ķ�Χ�� DIAG_MESSAGE_TYPE_U32 ��ͨ */

/* 2000 - 2fff����PS����Ϣ��Χ */

/* ����TTF�Ĳ����Ϣ�ϱ��漰�ڴ濽�������԰ѿ���֪ͨTTF��TTF������鿪��δ��ʱ����ִ���ڴ濽���������CPU����Ч�� */
#define     DIAG_TTF_TRACECFG_REQ                   (0x00002201)    /* ��TTF���ò����Ϣ���� */
#define     DIAG_TTF_DISCONN_REQ                    (0x00002203)    /* ֪ͨTTF�����ѶϿ� */


/* 3000 - 3fff����PHY����Ϣ��Χ */
#define     DIAG_GUPHY_RELEASE_REQ                  (0x00003201)    /* ��ΪAPI��ʽ֪ͨ������Ϣ�ϳ� */


/* c000 - cfff����HIFI����Ϣ��Χ */
#define     DIAG_HIFI_RELEASE_REQ                   (0x0000c001)


/* ��HIFIԼ������ϢID */
#define     ID_HIFI_DIAG_CONNECT_CNF                (0x492A)
#define     ID_HIFI_DIAG_DISCONNECT_CNF             (0x492B)


/*****************************************************************************
  4 Enum
*****************************************************************************/

/* ==============�����ͷ�ṹö��ֵ����==================================== */

/* DIAG_SERVICE_HEAD_STRU:ssid4b */
enum DIAG_SSID_TYPE
{
#ifdef DIAG_SYSTEM_5G
    DIAG_SSID_DEFAULT       = 0x0,  /* 0x00��δָ�������·�REQʹ�ã���ʾ��UEȷ���ַ�Ŀ�꣬CNF��IND����ʹ�ø�ֵ�� */
    DIAG_SSID_APP_CPU       = 0x1,  /* 0x01��A-CPU */
    DIAG_SSID_MODEM_CPU     = 0x2,  /* 0x02��2G/3G/4.5G��ϵͳC-CPU��������PSʹ�ã� */
    DIAG_SSID_TLDSP_BBE_NX  = 0x3,  /* 0x03��BBE NX��TL DSPʹ�ã� */
    DIAG_SSID_BBP_DEBUG     = 0x4,  /* 0x04��2G/3G/4.5G BBP Debug	//ԭ����LTE BBP */
    DIAG_SSID_GUC_BBE_NX    = 0x5,  /* 0x05��BBE NX��GUC DSP SDRʹ�ã�	//��GUC PHY��TL PHYû���ںϣ����赥������ */
    DIAG_SSID_HIFI          = 0x6,
    DIAG_SSID_LTE_V_DSP     = 0x7,  /* 0x07��LTE-V DSP��Ԥ����//ԭ����TDS DSP */
    DIAG_SSID_RESERVE0      = 0x8,
    DIAG_SSID_MCU           = 0x9,
    DIAG_SSID_TEE           = 0xA,  /* 0x0A��TEE		//ԭ����GPU */
    DIAG_SSID_RESERVE1      = 0xB,  /* 0x0B������		//ԭ����GUX BBP */
    DIAG_SSID_IOM3          = 0xC,
    DIAG_SSID_EASYRF0       = 0xD,
    DIAG_SSID_X_DSP         = 0xE,
    DIAG_SSID_GUC_L1C       = 0xE,  /* 0x0E��2G/3G/4.5G��ϵͳC-CPU��GUC L1Cʹ�ã� 	//��GUC L1Cû��ͨ��C��MSP����OAM�����ϱ������赥������ */
    DIAG_SSID_RESERVE2      = 0xF,
    DIAG_SSID_5G_CCPU       = 0x10,
    DIAG_SSID_L2HAC         = 0x11,
    DIAG_SSID_HL1C          = 0x12,
    DIAG_SSID_LL1C_CORE0    = 0x13,
    DIAG_SSID_LL1C_CORE1    = 0x14,
    DIAG_SSID_LL1C_CORE2    = 0x15,
    DIAG_SSID_LL1C_CORE3    = 0x16,
    DIAG_SSID_LL1C_CORE4    = 0x17,
    DIAG_SSID_LL1C_CORE5    = 0x18,
    DIAG_SSID_SDR_CORE0     = 0x19,
    DIAG_SSID_SDR_CORE1     = 0x1A,
    DIAG_SSID_SDR_CORE2     = 0x1B,
    DIAG_SSID_SDR_CORE3     = 0x1C,
    DIAG_SSID_5G_BBP_DEBUG  = 0x1D,
    DIAG_SSID_EASYRF1       = 0x1E,
    DIAG_SSID_BBP_ACCESS_DEBUG= 0x1F,
    DIAG_SSID_BUTT,
#else
    DIAG_SSID_APP_CPU   = 0x1,
    DIAG_SSID_MODEM_CPU,
    DIAG_SSID_LTE_DSP,
    DIAG_SSID_LTE_BBP,
    DIAG_SSID_GU_DSP,
    DIAG_SSID_HIFI,
    DIAG_SSID_TDS_DSP,
    DIAG_SSID_TDS_BBP,
    DIAG_SSID_MCU,
    DIAG_SSID_GPU,
    DIAG_SSID_GU_BBP,
    DIAG_SSID_IOM3,
    DIAG_SSID_ISP,
    DIAG_SSID_X_DSP,
    DIAG_SSID_BUTT
#endif
};
typedef VOS_UINT32 DIAG_SSID_TYPE_U32;


/* DIAG_SERVICE_HEAD_STRU:sessionid8b */
#define MSP_SERVICE_SESSION_ID        (0x1) /* ��ʶService��Client֮�������,�̶�Ϊ1*/

/* DIAG_SERVICE_HEAD_STRU:mt2b */
enum DIAG_MSGTYPE_TYPE
{
    DIAG_MT_RSV   = 0x0,
    DIAG_MT_REQ   = 0x1,
    DIAG_MT_CNF   = 0x2,
    DIAG_MT_IND   = 0x3
};
typedef VOS_UINT32 DIAG_DIRECTION_TYPE_U32;

/* ======================================================================== */


/* ==============��Ϣ��ͷ�ṹö��ֵ����==================================== */

/* MSP_DIAG_STID_STRU:pri4b */
#ifdef DIAG_SYSTEM_5G
enum DIAG_MESSAGE_TYPE
{
    DIAG_MSG_TYPE_RSV       = 0x0,
    DIAG_MSG_TYPE_MSP       = 0x1,
    DIAG_MSG_TYPE_PS        = 0x2,
    DIAG_MSG_TYPE_PHY       = 0x3,
    DIAG_MSG_TYPE_BBP       = 0x4,
    DIAG_MSG_TYPE_HSO       = 0x5,
    DIAG_MSG_TYPE_BSP       = 0x9, /*MODEM BSP*/
    DIAG_MSG_TYPE_EASYRF    = 0xa,
    DIAG_MSG_TYPE_AP_BSP    = 0xb, /*AP BSP*/
    DIAG_MSG_TYPE_AUDIO     = 0xc,
    DIAG_MSG_TYPE_APP       = 0xe,
    DIAG_MSG_TYPE_BUTT
};
#else
enum DIAG_MESSAGE_TYPE
{
    DIAG_MSG_TYPE_RSV   = 0x0,
    DIAG_MSG_TYPE_MSP   = 0x1,
    DIAG_MSG_TYPE_PS    = 0x2,
    DIAG_MSG_TYPE_PHY   = 0x3,
    DIAG_MSG_TYPE_BBP   = 0x4,
    DIAG_MSG_TYPE_HSO   = 0x5,
    DIAG_MSG_TYPE_BSP   = 0x9,
    DIAG_MSG_TYPE_ISP   = 0xa,
    DIAG_MSG_TYPE_AUDIO = 0xc,
    DIAG_MSG_TYPE_APP   = 0xe,
    DIAG_MSG_TYPE_BUTT
};
#endif
typedef VOS_UINT32 DIAG_MESSAGE_TYPE_U32;

/* MSP_DIAG_STID_STRU:mode4b */
enum DIAG_MODE_TYPE
{
    DIAG_MODE_LTE  = 0x0,
    DIAG_MODE_TDS  = 0x1,
    DIAG_MODE_GSM  = 0x2,
    DIAG_MODE_UMTS = 0x3,
    DIAG_MODE_1X   = 0x4,
    DIAG_MODE_HRPD = 0x5,
#ifdef DIAG_SYSTEM_5G    
    DIAG_MODE_NR   = 0x6,
#endif
    DIAG_MODE_COMM = 0xf
};
typedef VOS_UINT32 DIAG_MODE_TYPE_U32;

/* MSP_DIAG_STID_STRU:pri4b */
enum DIAG_MSG_SUB_TYPE_TYPE
{
    DIAG_MSG_CMD        = 0x0,
    DIAG_MSG_PS_AIR     = 0x1,
    DIAG_MSG_PS_LAYER   = 0x2,
    DIAG_MSG_PS_PRINT   = 0x3,
    DIAG_MSG_PS_EVENT   = 0x4,
    DIAG_MSG_PS_USER    = 0x5,
    DIAG_MSG_PS_VOLTE   = 0x6,
    DIAG_MSG_PS_STRUCT  = 0x7,
    DIAG_MSG_PS_DOT     = 0x8,
    DIAG_MSG_DSP_PRINT  = 0x9,
    DIAG_MSG_RAE_CNF    = 0xa,
    DIAG_MSG_RAE_IND    = 0xb,
    DIAG_MSG_STAT       = 0x1f
};
typedef VOS_UINT32 DIAG_MSG_SUB_TYPE_U32;


enum DIAG_MODEM_ID
{
    DIAG_MODEM_0 = 0x0,
    DIAG_MODEM_1 = 0x1,
    DIAG_MODEM_2 = 0x2
};
typedef unsigned int DIAG_MODEM_ID_U32;


/* DIAG_SERVICE_HEAD_STRU:sid8b */
enum DIAG_SID_TYPE
{
    MSP_SID_DEFAULT   = 0x0,
    MSP_SID_AT_SERVICE,
    MSP_SID_DIAG_SERVICE,
    MSP_SID_DATA_SERVICE,
    MSP_SID_NV_SERVICE,
    MSP_SID_USIM_SERVICE,
    MSP_SID_DM_SERVICE,
    MSP_SID_CBT_SERVICE,
    MSP_SID_BUTT
};

typedef unsigned int MSP_SID_TYPE_U32;

typedef enum
{
    PS_LOG_LEVEL_OFF  = 0,
    PS_LOG_LEVEL_ERROR,
    PS_LOG_LEVEL_WARNING,
    PS_LOG_LEVEL_NORMAL,
    PS_LOG_LEVEL_INFO,
    PS_LOG_LEVEL_BUTT
}PS_LOG_LEVEL_EN;

enum DIAG_FRAME_VER_TYPE
{
    DIAG_FRAME_VER_4G = 0,
    DIAG_FRAME_VER_5G = 1,
    DIAG_FRAME_VER_BUTT
};
/**************************************************************************
  5 �ṹ����
**************************************************************************/

/* ==============֡�ṹ����================================================ */
#ifdef DIAG_SYSTEM_5G
/* ���� :5G һ��ͷ: serviceͷ */
typedef struct
{
    VOS_UINT32    sid4b       :4;   /* service id, value:DIAG_SID_TYPE */
    VOS_UINT32    ver4b       :4;   /* version , value:DIAG_SERVICE_HEAD_VER_TYPE */
    VOS_UINT32    mdmid3b     :3;   /* modem id dfd*/
    VOS_UINT32    rsv5b       :5;
    VOS_UINT32    ssid8b      :8;   /* sub system id , DIAG_SSID_TYPE, CCPU/ACPU/BBE NX/Audio Dsp/LTE-V DSP..... */
    VOS_UINT32    mt2b        :2;
    VOS_UINT32    index4b     :4;
    VOS_UINT32    eof1b       :1;
    VOS_UINT32    ff1b        :1;

    VOS_UINT16    ulMsgTransId;
    VOS_UINT16    ulSocpEncDstId;

    VOS_UINT8     aucTimeStamp[4];
}DIAG_SERVICE_HEAD_STRU;

#define SERVICE_HEAD_SID(pData)   ((VOS_UINT32)(((DIAG_SERVICE_HEAD_STRU *)pData)->sid4b))
#define DIAG_4G_FRAME_HEAD_LEN    (sizeof(DIAG_SERVICE_HEAD_STRU) + sizeof(VOS_UINT32))
#define DIAG_SERVICE_HEAD_VER(pData) (((DIAG_SERVICE_HEAD_STRU *)pData)->ver4b)
#else
/* ���� :4G һ��ͷ: serviceͷ */
typedef struct
{
    VOS_UINT32    sid8b       :8;   /* service id, value:DIAG_SID_TYPE */
    VOS_UINT32    mdmid3b     :3;
    VOS_UINT32    rsv1b       :1;
    VOS_UINT32    ssid4b      :4;
    VOS_UINT32    sessionid8b :8;
    VOS_UINT32    mt2b        :2;
    VOS_UINT32    index4b     :4;
    VOS_UINT32    eof1b       :1;
    VOS_UINT32    ff1b        :1;

    VOS_UINT32    ulMsgTransId;
    VOS_UINT8     aucTimeStamp[8];
}DIAG_SERVICE_HEAD_STRU;

#define SERVICE_HEAD_SID(pData)         ((VOS_UINT32)(((DIAG_SERVICE_HEAD_STRU *)pData)->sid8b))
#define DIAG_4G_FRAME_HEAD_LEN          (sizeof(DIAG_SERVICE_HEAD_STRU))
#define DIAG_SERVICE_HEAD_VER(pData)    ((((DIAG_SERVICE_HEAD_STRU *)pData)->sid8b)&0xF0)//sid8b��5G��Ϊsid4b ver4b ȡ
#endif

/* ���� :����ͷ: DIAG��Ϣͷ */
typedef struct
{
    VOS_UINT32    cmdid19b:19;
    VOS_UINT32    sec5b   :5;
    VOS_UINT32    mode4b  :4;
    VOS_UINT32    pri4b   :4;
} MSP_DIAG_STID_STRU;

typedef struct
{
    union
    {
        VOS_UINT32          ulID;           /* �ṹ��ID */
        MSP_DIAG_STID_STRU  stID;
    };

    VOS_UINT32          ulDataSize;     /* ucData�ĳ��� */
    VOS_UINT8           aucData[0];
}DIAG_MSG_INFO_STRU;

/* ���� :����ͷ: ����������Ϣͷ������REQ/CNF��Ϣ */
typedef struct
{
    VOS_UINT32 ulAuid;         /* ԭAUID*/
    VOS_UINT32 ulSn;           /* HSO�ַ�������������*/
    VOS_UINT8  ucData[0];      /* ����������*/
} MSP_DIAG_DATA_REQ_STRU;

typedef MSP_DIAG_DATA_REQ_STRU MSP_DIAG_DATA_CNF_STRU;


/* ���� :����֡�ṹ */
typedef struct
{
    DIAG_SERVICE_HEAD_STRU      stService;

    union
    {
        VOS_UINT32          ulCmdId;           /* �ṹ��ID */
        MSP_DIAG_STID_STRU  stID;
    };

    VOS_UINT32                  ulMsgLen;
#if (VOS_OS_VER == VOS_WIN32)
    VOS_UINT8                   aucData[4];
#else
    VOS_UINT8                   aucData[0];
#endif

}DIAG_FRAME_INFO_STRU;


/* �����Ϣ��Ϊ�ϲ��ṩ�Ĳ����ṹ */
typedef struct
{
    DIAG_SSID_TYPE_U32          ulSSId;         /* ���ݲ�����CPU ID */
    DIAG_MESSAGE_TYPE_U32       ulMsgType;      /* ������� */
    DIAG_MODE_TYPE_U32          ulMode;         /* ģʽ */
    DIAG_MSG_SUB_TYPE_U32       ulSubType;      /* �����ͣ�DIAG_MSG_SUB_TYPE_U32 */
    DIAG_DIRECTION_TYPE_U32     ulDirection;    /* �ϱ���Ϣ�ķ��� */
    VOS_UINT32                  ulModemid;
    VOS_UINT32                  ulMsgId;        /* ��16λ��Ч */
    VOS_UINT32                  ulTransId;      /* TransId */
} MSP_DIAG_CNF_INFO_STRU;

/* ======================================================================== */


typedef struct
{
    VOS_UINT32 ulId;
    VOS_UINT32 ulMessageID;  /* Specify the message id.*/
    VOS_UINT32 ulSideId;     /* �տ���Ϣ�ķ���*/
    VOS_UINT32 ulDestMod;    /* Ŀ��ģ��*/
    VOS_UINT32 ulDataSize;   /* the data size in bytes.*/
    VOS_VOID* pData;      /* Pointer to the data buffer.*/
} DIAG_AIR_MSG_LOG_STRU;

typedef DIAG_AIR_MSG_LOG_STRU DIAG_LAYER_MSG_STRU;

typedef struct
{
    VOS_UINT32 ulMessageID;    /* Specify the message id.*/
    VOS_UINT32 ulDataSize;     /* the data size in bytes.*/
    VOS_VOID* pData;        /* Pointer to the data buffer.*/
} DIAG_USERPLANE_MSG_STRU;

/* service��Ϊ�ϲ��ṩ�Ļ�ȡbuffer�ķ��ؽṹ */
typedef struct
{
    DIAG_SERVICE_HEAD_STRU      *pHeadService;
    DIAG_MSG_INFO_STRU          *pHeadMessage;
    VOS_VOID                    *pData;
    VOS_UINT32                  ulSize;         /* ���ݳ��� */
    VOS_UINT32                  ulLength;       /* ������ͷ��ʵ�ʳ��� */
}DIAG_SERVICE_REPORT_STRU;



/*****************************************************************************
���� : ��TTF��Ĳ����Ϣ���õĲ����ṹ(Ϊ�����ֽڶ������⣬TTF�������ô˽ṹ)
MSGID:DIAG_TTF_LAYERCFG_REQ
REQ : DIAG_TTF_TRACECFG_REQ_STRU(����Ϣ����Ҫ�ظ�)
*****************************************************************************/

typedef struct
{
    VOS_MSG_HEADER
    VOS_UINT32                  ulMsgId;

    VOS_UINT8                   aucSrcPid[VOS_PID_BUTT-VOS_PID_DOPRAEND];
    VOS_UINT8                   aucDstPid[VOS_PID_BUTT-VOS_PID_DOPRAEND];
}DIAG_TTF_TRACECFG_REQ_STRU;



enum OM_AT_MSG_ENUM
{
    AT_OM_PORT_SWITCH           = 0,    /* OM���л�������USB��HSIC֮���л� */
    AT_OM_HSIC_PORT_CONNECT,            /* OM��OM����HSIC������ */
    AT_OM_HSIC_PORT_DISCONNECT,         /* OM��OM����HSICȡ������ */
    AT_OM_MSG_BUTT
};
typedef VOS_UINT32 OM_AT_MSG_ENUM_UINT32;


enum OM_PORT_SWITCH_MODE_ENUM
{
    OM_PORT_SWITCH_MODEM2AP     = USB_SWITCH_OFF,
    OM_PORT_SWITCH_AP2MODEM     = USB_SWITCH_ON,
    OM_PORT_SWITCH_BUTT
};
typedef VOS_UINT32 OM_PORT_SWITCH_MODE_ENUM_UINT32;


typedef struct
{
   VOS_MSG_HEADER
   OM_AT_MSG_ENUM_UINT32            ulMsgName;      /*�л���Ϣ��*/
   OM_PORT_SWITCH_MODE_ENUM_UINT32  ulSwitchMode;   /*�л�ģʽ����AP��Modem��Modem��AP*/
}OM_PORT_SWITCH_MSG_STRU;


typedef struct
{
   VOS_MSG_HEADER
   OM_AT_MSG_ENUM_UINT32            ulMsgName;      /*�л���Ϣ��*/
}OM_HSIC_CONNECT_MSG_STRU;

/**************************************************************************
  ��������
**************************************************************************/


/* ==============�����ϱ��ӿڲ���========================================== */

typedef struct
{
    VOS_UINT32        ulModule;		/* DIAG_GEN_MODULE*/
    VOS_UINT32        ulPid;
    VOS_UINT32        ulEventId;		/* �¼�ID */
    VOS_UINT32        ulLength;
    VOS_VOID          *pData;
}DIAG_EVENT_IND_STRU;

typedef struct
{
    VOS_UINT32        ulModule;       /* DIAG_GEN_MODULE*/
    VOS_UINT32        ulPid;
    VOS_UINT32        ulMsgId;
    VOS_UINT32        ulDirection;
    VOS_UINT32        ulLength;
    VOS_VOID          *pData;
}DIAG_AIR_IND_STRU;

typedef struct
{
    VOS_UINT32        ulModule;       /* DIAG_GEN_MODULE*/
    VOS_UINT32        ulPid;
    VOS_UINT32        ulMsgId;
    VOS_UINT32        ulLength;
    VOS_VOID          *pData;
}DIAG_USER_IND_STRU;


typedef struct
{
    VOS_UINT32        ulId;
    VOS_UINT32        ulMessageID;
    VOS_UINT32        ulSideId;
    VOS_UINT32        ulDestMod;
    VOS_UINT32        ulDataSize;
    VOS_VOID          *pData;
} DIAG_VOLTE_LOG_STRU;

typedef struct
{
    VOS_UINT32        ulModule;
    VOS_UINT32        ulPid;
    VOS_UINT32        ulMsgId;
    VOS_UINT32        ulReserve;
    VOS_UINT32        ulLength;
    VOS_VOID          *pData;
} DIAG_TRANS_IND_STRU;

/*********************************���ӹ������********************************************/

typedef struct
{
    VOS_MSG_HEADER
    VOS_UINT32              ulMsgId;           /* ��Ϣ�� */
    VOS_UINT32              ulLen;             /* ���ݳ��� */
    VOS_UINT8               pContext[0];       /* ������ʵ��ַ*/
}DIAG_CFG_MSG_HEAD;


typedef struct{
    VOS_UINT32			ulChannelId;	/* ͨ��ID */
    VOS_UINT32			ulResult;		/*  ������� 0�ɹ�, 0x5C5C5C5Cͨ��δ����, 0x5A5A5A5Aͨ��δʹ��, ����ֵʧ��*/
}DIAG_CONNECT_RESULT;

typedef struct{
    VOS_UINT32				ulSn;
    DIAG_CONNECT_RESULT     pstResult[0];
} DIAG_CONN_CNF_MSG_STRU;

typedef struct{
    VOS_UINT32				ulSn;
}DIAG_CONN_REQ_MSG_STRU;


/* �����Ϣƥ��ӿڣ���κͷ���ֵΪ��׼��OSA��Ϣ��ʽ */
typedef VOS_VOID* (*DIAG_LayerMsgMatchFunc)(VOS_VOID *pMsg);
/* �����Ϣ���˽ӿڣ����Ϊ��׼��OSA��Ϣ��ʽ������ֵΪ�Ƿ����: 0-������,�ϱ�; ����-����,���ϱ� */
typedef VOS_UINT32 (*DIAG_TraceFilterFunc)(const VOS_VOID *pMsg);
/*****************************************************************************
 Function Name   : DIAG_TraceMatchFuncReg
 Description     : �����Ϣ����ע��ӿ�(�˽ӿڲ�֧���ظ�ע�ᣬ���ע�᷵��ʧ��)
 Input           : ���˴�������
 Output          : None
 Return          : ����ֵΪע����: 0-ע��ɹ�������-ע��ʧ��

 History         :
    1.c64416      2012-11-20  Draft Enact

*****************************************************************************/
VOS_UINT32 DIAG_LayerMsgMatchFuncReg(DIAG_LayerMsgMatchFunc pFun);

/* �����Ϣƥ����ɺ��notify�ӿ� */
typedef VOS_UINT32 (*DIAG_LayerMsgMatchNotifyFunc)(VOS_UINT32 sendPid, VOS_VOID *pMsg);
/* �����Ϣƥ��ӿڣ���κͷ���ֵΪ��׼��OSA��Ϣ��ʽ */
typedef VOS_VOID* (*DIAG_TraceMatchFunc)(VOS_VOID *pMsg);
/*****************************************************************************
 Function Name   : DIAG_TraceMatchFuncReg
 Description     : �����Ϣ���˵�notify�ӿ�ע��(�˽ӿڲ�֧���ظ�ע�ᣬ���ע�᷵��ʧ��)
 Input           : ���˴�������
 Output          : None
 Return          : ����ֵΪע����: 0-ע��ɹ�������-ע��ʧ��

 History         :
    1.c64416      2012-11-20  Draft Enact

*****************************************************************************/
VOS_UINT32 DIAG_LayerMsgMatchNotifyFuncReg(DIAG_LayerMsgMatchNotifyFunc pFun);

/*****************************************************************************
 �� �� ��  : DIAG_TransReport
 ��������  : �ṹ�������ϱ��ӿ�(�滻ԭ����DIAG_ReportCommand)
 �������  : DIAG_TRANS_IND_STRU->ulModule( 31-24:modemid,23-16:modeid,15-12:level,11-0:pid )
             DIAG_TRANS_IND_STRU->ulMsgId(͸������ID)
             DIAG_TRANS_IND_STRU->ulLength(͸����Ϣ�ĳ���)
             DIAG_TRANS_IND_STRU->pData(͸����Ϣ)
*****************************************************************************/
VOS_UINT32 DIAG_TransReport(DIAG_TRANS_IND_STRU *pstData);

/*****************************************************************************
 �� �� ��  : DIAG_TransReport_Ex
 ��������  : �ṹ�������ϱ�����չ�ӿ�(���DIAG_TransReport��������11-8bits �����Ϣ)
 �������  : DIAG_TRANS_IND_STRU->ulModule( 31-24:modemid,23-16:modeid,15-12:level,11-8:groupid )
 �������  : DIAG_TRANS_IND_STRU->ulPid(ģ���PID)
             DIAG_TRANS_IND_STRU->ulMsgId(͸������ID)
             DIAG_TRANS_IND_STRU->ulLength(͸����Ϣ�ĳ���)
             DIAG_TRANS_IND_STRU->pData(͸����Ϣ)
*****************************************************************************/
VOS_UINT32 DIAG_TransReport_Ex(DIAG_TRANS_IND_STRU *pstData);

/*****************************************************************************
 �� �� ��  : DIAG_LogReport
 ��������  : ��ӡ�ϱ��ӿ�
 �������  : ulModuleId( 31-24:modemid,23-16:modeid,15-12:level )
             ulPid( PID )
             pcFileName(�ϱ�ʱ����ļ�·��ɾ����ֻ�����ļ���)
             ulLineNum(�к�)
             pszFmt(�ɱ����)
ע������   : ���ڴ˽ӿڻᱻЭ��ջƵ�����ã�Ϊ��ߴ���Ч�ʣ����ӿ��ڲ���ʹ��1K�ľֲ����������ϱ����ַ�����Ϣ��
             �Ӷ��˽ӿڶ�Э��ջ���������ƣ�һ�ǵ��ô˽ӿڵ�����ջ�е��ڴ�Ҫ����Ϊ�˽ӿ�Ԥ��1K�ռ䣻
                                           ���ǵ��ô˽ӿ������log��Ҫ����1K���������ֻ��Զ�������
*****************************************************************************/
VOS_UINT32 DIAG_LogReport(VOS_UINT32 ulModuleId, VOS_UINT32 ulPid, VOS_CHAR *cFileName, VOS_UINT32 ulLineNum, VOS_CHAR* pszFmt, ...);


#define LTE_DIAG_PRINTF_PARAM_MAX_NUM       (6)
/******************************************************************************
��������: DIAG_LogIdReport
��������: ��ӡ�����͵Ĵ�ӡ�ӿں���
����˵��:
            ulModuleId[in]  : ( 31-24:modemid,23-16:modeid,15-12:level )
            ulPid[in]       : PID
            ulLogId[in]     : ���ļ��ź��кŸ���DIAG_LOG_ID����
            amount[in]      : �ɱ����������������ulModuleId/ulLevel/ulLogId/amout��
            ...             : �ɱ����
����Լ��:
            1. ���Խ�ֹ�Դ˺������ж��η�װ��ֻ��ת����
            2. ֧�ֿɱ�Ĳ����������������ڵ���ʱ�ɲ���amountָ����������
            3. �ɱ����ֻ֧��int����
            4. Ŀǰ�汾��֧�ֵ�������������6���������Ĳ���Ĭ�϶���
******************************************************************************/
VOS_INT32 DIAG_LogIdReport(VOS_UINT32 ulModuleId, VOS_UINT32 ulPid,
                        VOS_UINT32 ulLogId, VOS_UINT32 amount, ...);

/*****************************************************************************
 �� �� ��  : DIAG_EventReport
 ��������  : �¼��ϱ��ӿڣ���PSʹ��(�滻ԭ����DIAG_ReportEventLog)
 �������  : DIAG_EVENT_IND_STRU->ulModule( 31-24:modemid,23-16:modeid,15-12:level,11-0:pid )
             DIAG_EVENT_IND_STRU->ulEventId(event ID)
             DIAG_EVENT_IND_STRU->ulLength(event�ĳ���)
             DIAG_EVENT_IND_STRU->pData(event��Ϣ)
*****************************************************************************/
VOS_UINT32 DIAG_EventReport(DIAG_EVENT_IND_STRU *pstEvent);

/*****************************************************************************
 �� �� ��  : DIAG_AirMsgReport
 ��������  : �տ���Ϣ�ϱ��ӿڣ���PSʹ��(�滻ԭ����DIAG_ReportAirMessageLog)
 �������  : DIAG_AIR_IND_STRU->ulModule( 31-24:modemid,23-16:modeid,15-12:level,11-0:pid )
             DIAG_AIR_IND_STRU->ulMsgId(�տ���ϢID)
             DIAG_AIR_IND_STRU->ulDirection(�տ���Ϣ�ķ���)
             DIAG_AIR_IND_STRU->ulLength(�տ���Ϣ�ĳ���)
             DIAG_AIR_IND_STRU->pData(�տ���Ϣ��Ϣ)
*****************************************************************************/
VOS_UINT32 DIAG_AirMsgReport(DIAG_AIR_IND_STRU *pstAir);


/*****************************************************************************
 �� �� ��  : DIAG_TraceReport
 ��������  : �����Ϣ�ϱ��ӿڣ���PSʹ��(�滻ԭ����DIAG_ReportLayerMessageLog)
 �������  : pMsg(��׼��VOS��Ϣ�壬Դģ�顢Ŀ��ģ����Ϣ����Ϣ���л�ȡ)

*****************************************************************************/
extern VOS_VOID DIAG_TraceReport(VOS_VOID *pMsg);



VOS_VOID DIAG_LayerMsgReport(VOS_VOID *pMsg);

/*****************************************************************************
 �� �� ��  : DIAG_UserPlaneReport
 ��������  : �û����ϱ��ӿڣ���PSʹ��(�滻ԭ����DIAG_ReportUserPlaneMessageLog)
 �������  : DIAG_USER_IND_STRU->ulModule( 31-24:modemid,23-16:modeid,15-12:level,11-0:pid )
             DIAG_USER_IND_STRU->ulMsgId(�û�����ϢID)
             DIAG_USER_IND_STRU->ulLength(�û�����Ϣ�ĳ���)
             DIAG_USER_IND_STRU->pData(�û�����Ϣ)
*****************************************************************************/
VOS_UINT32 DIAG_UserPlaneReport(DIAG_USER_IND_STRU *pstUser);

/*****************************************************************************
 Function Name   : DIAG_ReportVoLTELog
 Description     : ��IMS�ṩ��VOLTE������Ϣ�ϱ��ӿ�
 Input           : DIAG_VoLTE_LOG_STRU* pRptMessage
 Output          : None
 Return          : VOS_UINT32

 History         :
    1.fuxin      2013-12-30  create
*****************************************************************************/
VOS_UINT32 DIAG_ReportVoLTELog(DIAG_VOLTE_LOG_STRU* pRptMessage);


VOS_UINT32 DIAG_GetConnState(VOS_VOID);


VOS_UINT32 DIAG_LogPortSwich(VOS_UINT32 ulPhyPort, VOS_BOOL ulEffect);


/*****************************************************************************
  �ⲿ�ӿڶ���
*****************************************************************************/
extern VOS_UINT32 DIAG_ErrorLog(VOS_CHAR * cFileName,VOS_UINT32 ulFileId, VOS_UINT32 ulLine,
                VOS_UINT32 ulErrNo, VOS_VOID * pBuf, VOS_UINT32 ulLen);

#define MNTN_RecordErrorLog(ulErrNo,pBuf,ulLen) \
    DIAG_ErrorLog(__FILE__,THIS_FILE_ID,__LINE__,ulErrNo,pBuf,ulLen)



extern VOS_UINT32 Diag_GetLogLevel(VOS_UINT32 ulPid);


#if(VOS_OS_VER == VOS_LINUX)

extern VOS_VOID DIAG_LogShowToFile(VOS_BOOL bIsSendMsg);
#endif

#ifdef __cplusplus
}
#endif
#endif
