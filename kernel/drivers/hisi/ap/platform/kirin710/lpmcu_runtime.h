#ifndef _LPMCU_RUNTIME_H_
#define _LPMCU_RUNTIME_H_ 
#include <m3_sram_map.h>
#ifndef BYTE_REF
#define BYTE_REF(address) (* ((unsigned char volatile * ) (address)))
#define HALFWORD_REF(address) (* ((unsigned short volatile * ) (address)))
#define WORD_REF(address) (* ((unsigned int volatile * ) (address)))
#define WORD_PTR(address) (* ((unsigned int volatile **) (address)))
#endif
enum {
 WITH_MODEM_OFFSET = 0,
 LOWPM_SHOW_OFFSET = 3,
 LOWPM_CFG_OFFSET = 4,
};
#define IS_MODEM ((u8)BIT(WITH_MODEM_OFFSET))
#define LOWPM_SHOW ((u8)BIT(LOWPM_SHOW_OFFSET))
#define LOWPM_CFG ((u8)BIT(LOWPM_CFG_OFFSET))
enum {
 TMP_OP_OFFSET = 0,
 TMP_STRENGTHEN_MONITOR_OFFSET = 1,
 TMP_CTRL_RUNNING_OFFSET = 2,
 TMP_TSEN_RST_ENABLE_OFFSET = 4,
 TMP_DDR_HIGH_LIM_OFFSET = 5,
 TMP_LOW_TEMP_CTRL_OFFSET = 6,
};
#define TMP_STRENGTHEN_MONITOR ((u8)BIT(TMP_STRENGTHEN_MONITOR_OFFSET))
#define TMP_CTRL_RUNNING ((u8)BIT(TMP_CTRL_RUNNING_OFFSET))
#define TMP_TSEN_RST_ENABLE ((u8)BIT(TMP_TSEN_RST_ENABLE_OFFSET))
#define TMP_DDR_HIGH_LIM ((u8)BIT(TMP_DDR_HIGH_LIM_OFFSET))
#define TMP_LOW_TEMP_CTRL ((u8)BIT(TMP_LOW_TEMP_CTRL_OFFSET))
enum {
 DDR_OP_OFFSET = 0,
 DDR_MNT_OFFSET = 1,
 DDR_DFS_OFFSET = 2,
 DDR_AFS_OFFSET = 3,
 DDR_TMP_OFFSET = 4,
 DDR_TMP_NREC_OFFSET = 5,
 DDR_TMP_NRST_OFFSET = 6,
 DDR_TMP_SHANRST_OFFSET = 7,
 AUTOFSGT_EXT_EN_OFFSET = 8,
 AUTOFSGT_MODE_OFFSET = 9,
 AUTOFSGT_INT_EN_OFFSET = 10,
 DDR_UCE_UART_DIS_OFFSET = 11,
 DDR_TRAIN_TRACK_DIS_OFFSET = 12,
};
#define DDR_OP_EN ((u16)BIT(DDR_OP_OFFSET))
#define DDR_MNT_EN ((u16)BIT(DDR_MNT_OFFSET))
#define DDR_DFS_EN ((u16)BIT(DDR_DFS_OFFSET))
#define DDR_AFS_EN ((u16)BIT(DDR_AFS_OFFSET))
#define DDR_TMP_EN ((u16)BIT(DDR_TMP_OFFSET))
#define DDR_TMP_NREC ((u16)BIT(DDR_TMP_NREC_OFFSET))
#define DDR_TMP_NRST ((u16)BIT(DDR_TMP_NRST_OFFSET))
#define DDR_TMP_SHANRST ((u16)BIT(DDR_TMP_SHANRST_OFFSET))
#define AUTOFSGT_EXT_EN ((u16)BIT(AUTOFSGT_EXT_EN_OFFSET))
#define AUTOFSGT_MODE ((u16)BIT(AUTOFSGT_MODE_OFFSET))
#define AUTOFSGT_INT_EN ((u16)BIT(AUTOFSGT_INT_EN_OFFSET))
#define DDR_UCE_UART_DIS ((u16)BIT(DDR_UCE_UART_DIS_OFFSET))
#define DDR_TRAIN_TRACK_DIS ((u16)BIT(DDR_TRAIN_TRACK_DIS_OFFSET))
enum {
 OP_ENABLE_OFFSET = 0,
 VDM_ENABLE_OFFSET = 1,
 DVFS_ENABLE_OFFSET = 2,
 AVS_ENABLE_OFFSET = 3,
 IDLE_DIV_OFFSET = 4,
 FREQ_DOWN_OFFSET = 5,
 DVFS_RUNNING_OFFSET = 6,
 VOL_SCALE_OFFSET = 7,
};
#define OP_ENABLE ((u8)BIT(OP_ENABLE_OFFSET))
#define VDM_ENABLE ((u8)BIT(VDM_ENABLE_OFFSET))
#define DVFS_ENABLE ((u8)BIT(DVFS_ENABLE_OFFSET))
#define AVS_ENABLE ((u8)BIT(AVS_ENABLE_OFFSET))
#define IDLE_DIV ((u8)BIT(IDLE_DIV_OFFSET))
#define FREQ_DOWN ((u8)BIT(FREQ_DOWN_OFFSET))
#define DVFS_RUNNING ((u8)BIT(DVFS_RUNNING_OFFSET))
#define VOL_SCALE ((u8)BIT(VOL_SCALE_OFFSET))
enum {
 GPU_LOG_EN_OFFSET = 0,
 GPU_DEBUG_MAX = 7
};
#define GPU_LOG_ENABLE ((u8)BIT(GPU_LOG_EN_OFFSET))
enum {
 PERI_VIVOBUS_OFFSET = 0,
 PERI_VCODECBUS_OFFSET = 1,
 PERI_SYSBUS_OFFSET = 2,
 PERI_LOWTEMP_FREQ_SET = 4,
 PERI_NOMTEMP_FREQ_SET = 5,
};
#define PERI_VIVOBUS_ENABLE ((u8)BIT(PERI_VIVOBUS_OFFSET))
#define PERI_CODECBUS_ENABLE ((u8)BIT(PERI_VCODECBUS_OFFSET))
#define PERI_SYSBUS_ENABLE ((u8)BIT(PERI_SYSBUS_OFFSET))
#define PERI_LOWTEMP_FREQ_ENABLE ((u8)BIT(PERI_LOWTEMP_FREQ_SET))
#define PERI_NOMTEMP_FREQ_ENABLE ((u8)BIT(PERI_NOMTEMP_FREQ_SET))
#define LOWTCTRL_LOWFREQ(x) ((u8)((x) & 0xF))
#define LOWTCTRL_HIGHFREQ(x) ((u8)(((x) >> 4) & 0xF))
#define LPMCU_SECKEY_ADDR (M3_SRAM_BASE + 0x408)
#define LPMCU_SECKEY_BL31_ADDR (SOC_ACPU_LP_RAM_BASE_ADDR + 0x84)
#define LPMCU_SECKEY_SIZE (32U)
#define RUNTIME_CPU_VAR_ADDR RUNTIME_VAR_BASE
#define RUNTIME_CPU_VAR_SIZE (0x40U)
#define RUNTIME_CPU_CURRENT_ADDR(n) (RUNTIME_VAR_BASE + ((n) * 4))
#define CPU_CURRENT(n) (BYTE_REF(RUNTIME_CPU_CURRENT_ADDR(n)))
#define RUNTIME_CPU_TARGET_ADDR(n) (RUNTIME_VAR_BASE + 0x01 + ((n) * 4))
#define CPU_TARGET(n) (BYTE_REF(RUNTIME_CPU_TARGET_ADDR(n)))
#define RUNTIME_CPU_TCTRL_ADDR(n) (RUNTIME_VAR_BASE + 0x02 + ((n) * 4))
#define CPU_TCTRL(n) (BYTE_REF(RUNTIME_CPU_TCTRL_ADDR(n)))
#define RUNTIME_CPU_LAST_ADDR(n) (RUNTIME_VAR_BASE + 0x03 + ((n) * 4))
#define CPU_LAST(n) (BYTE_REF(RUNTIME_CPU_LAST_ADDR(n)))
#define RUNTIME_CPU_MIN_ADDR(n) (RUNTIME_VAR_BASE + 0x08 + ((n) * 4))
#define CPU_MIN(n) (BYTE_REF(RUNTIME_CPU_MIN_ADDR(n)))
#define RUNTIME_CPU_MAX_ADDR(n) (RUNTIME_VAR_BASE + 0x09 + ((n) * 4))
#define CPU_MAX(n) (BYTE_REF(RUNTIME_CPU_MAX_ADDR(n)))
#define RUNTIME_CPU_LOW_TCTRL_ADDR(n) (RUNTIME_VAR_BASE + 0x0A + ((n) * 4))
#define CPU_LOW_TCTRL(n) (BYTE_REF(RUNTIME_CPU_LOW_TCTRL_ADDR(n)))
#define RUNTIME_CPU_STEP_ADDR(n) (RUNTIME_VAR_BASE + 0x0B + ((n) * 4))
#define CPU_STEP(n) (BYTE_REF(RUNTIME_CPU_STEP_ADDR(n)))
#define RUNTIME_CPU_STATUS_ADDR(n) (RUNTIME_VAR_BASE + 0x10 + ((n) * 4))
#define CPU_STATUS(n) (BYTE_REF(RUNTIME_CPU_STATUS_ADDR(n)))
#define RUNTIME_CPU_ONLINE_NUM_ADDR(n) (RUNTIME_VAR_BASE + 0x11 + ((n) * 4))
#define CPU_ONLINE_NUM(n) (BYTE_REF(RUNTIME_CPU_ONLINE_NUM_ADDR(n)))
#define RUNTIME_CPU_ONLINE_STATUS_ADDR(n) (RUNTIME_VAR_BASE + 0x12 + ((n) * 4))
#define CPU_ONLINE_STATUS(n) (BYTE_REF(RUNTIME_CPU_ONLINE_STATUS_ADDR(n)))
#define RUNTIME_CPU_PERI_LIMIT_ADDR(n) (RUNTIME_VAR_BASE + 0x13 + ((n) * 4))
#define CPU_PERI_LIMIT(n) BYTE_REF(RUNTIME_CPU_PERI_LIMIT_ADDR(n))
#define RUNTIME_CPU_LOWTCTRL_SUPPORT_ADDR(n) (RUNTIME_VAR_BASE + 0x18 + ((n) * 2))
#define CPU_LOWTCTRL_SUPPORT(n) (BYTE_REF(RUNTIME_CPU_LOWTCTRL_SUPPORT_ADDR(n)))
#define RUNTIME_CPU_LOWTEMP_VOLT_ADDR(n) (RUNTIME_VAR_BASE + 0x19 + ((n) * 2))
#define CPU_LOWTEMP_VOLT(n) (BYTE_REF(RUNTIME_CPU_LOWTEMP_VOLT_ADDR(n)))
#define RUNTIME_CPU_VAR_END (RUNTIME_CPU_LOWTEMP_VOLT_ADDR(1))
#if (RUNTIME_CPU_VAR_END >= (RUNTIME_CPU_VAR_ADDR + RUNTIME_CPU_VAR_SIZE))
#error "RUNTIME_CPU_VAR overflow!!!"
#endif
#define RUNTIME_GPU_VAR_ADDR (RUNTIME_CPU_VAR_ADDR + RUNTIME_CPU_VAR_SIZE)
#define RUNTIME_GPU_VAR_SIZE (0x20U)
#define RUNTIME_GPU_CURRENT_ADDR (RUNTIME_VAR_BASE + 0x40)
#define GPU_CURRENT (BYTE_REF(RUNTIME_GPU_CURRENT_ADDR))
#define RUNTIME_GPU_TARGET_ADDR (RUNTIME_VAR_BASE + 0x41)
#define GPU_TARGET (BYTE_REF(RUNTIME_GPU_TARGET_ADDR))
#define RUNTIME_GPU_TCTRL_ADDR (RUNTIME_VAR_BASE + 0x42)
#define GPU_TCTRL (BYTE_REF(RUNTIME_GPU_TCTRL_ADDR))
#define RUNTIME_GPU_LAST_PLL_ADDR (RUNTIME_VAR_BASE + 0x43)
#define GPU_LAST_PLL (BYTE_REF(RUNTIME_GPU_LAST_PLL_ADDR))
#define RUNTIME_GPU_MIN_ADDR (RUNTIME_VAR_BASE + 0x44)
#define GPU_MIN (BYTE_REF(RUNTIME_GPU_MIN_ADDR))
#define RUNTIME_GPU_MAX_ADDR (RUNTIME_VAR_BASE + 0x45)
#define GPU_MAX (BYTE_REF(RUNTIME_GPU_MAX_ADDR))
#define RUNTIME_GPU_SAFE_ADDR (RUNTIME_VAR_BASE + 0x46)
#define GPU_LOW_TCTRL (BYTE_REF(RUNTIME_GPU_SAFE_ADDR))
#define RUNTIME_GPU_STEP_ADDR (RUNTIME_VAR_BASE + 0x47)
#define GPU_STEP (BYTE_REF(RUNTIME_GPU_STEP_ADDR))
#define RUNTIME_GPU_STATUS_ADDR (RUNTIME_VAR_BASE + 0x48)
#define GPU_STATUS (BYTE_REF(RUNTIME_GPU_STATUS_ADDR))
#define RUNTIME_GPU_ONLINE_NUM_ADDR (RUNTIME_VAR_BASE + 0x49)
#define GPU_ONLINE_NUM (BYTE_REF(RUNTIME_GPU_ONLINE_NUM_ADDR))
#define RUNTIME_GPU_ONLINE_STATUS_ADDR (RUNTIME_VAR_BASE + 0x4A)
#define GPU_ONLINE_STATUS (BYTE_REF(RUNTIME_GPU_ONLINE_STATUS_ADDR))
#define RUNTIME_GPU_PLL_ADDR (RUNTIME_VAR_BASE + 0x4B)
#define GPU_PLL (BYTE_REF(RUNTIME_GPU_PLL_ADDR))
#define RUNTIME_GPU_LAST_ADDR (RUNTIME_VAR_BASE + 0x4C)
#define GPU_LAST (BYTE_REF(RUNTIME_GPU_LAST_ADDR))
#define RUNTIME_GPU_DEBUG_ADDR (RUNTIME_VAR_BASE + 0x4D)
#define GPU_DEBUG (BYTE_REF(RUNTIME_GPU_DEBUG_ADDR))
#define RUNTIME_GPU_VAR_END (RUNTIME_GPU_DEBUG_ADDR)
#if (RUNTIME_GPU_VAR_END >= (RUNTIME_GPU_VAR_ADDR + RUNTIME_GPU_VAR_SIZE))
#error "RUNTIME_GPU_VAR overflow!!!"
#endif
#define RUNTIME_TMP_VAR_ADDR (RUNTIME_GPU_VAR_ADDR + RUNTIME_GPU_VAR_SIZE)
#define RUNTIME_TMP_VAR_SIZE (0x20U)
#define RUNTIME_TMP_NOR_ADDR (RUNTIME_VAR_BASE + 0x60)
#define TMP_NOR (BYTE_REF(RUNTIME_TMP_NOR_ADDR))
#define RUNTIME_TMP_HIGH_ADDR (RUNTIME_VAR_BASE + 0x61)
#define TMP_HIGH (BYTE_REF(RUNTIME_TMP_HIGH_ADDR))
#define RUNTIME_TMP_RST_ADDR (RUNTIME_VAR_BASE + 0x62)
#define TMP_RST (BYTE_REF(RUNTIME_TMP_RST_ADDR))
#define RUNTIME_TMP_LAST_ADDR (RUNTIME_VAR_BASE + 0x63)
#define TMP_LAST (BYTE_REF(RUNTIME_TMP_LAST_ADDR))
#define RUNTIME_TMP_STATUS_ADDR (RUNTIME_VAR_BASE + 0x64)
#define TMP_STATUS (BYTE_REF(RUNTIME_TMP_STATUS_ADDR))
#define RUNTIME_TMP_PERIOD_ADDR (RUNTIME_VAR_BASE + 0x65)
#define TMP_PERIOD (BYTE_REF(RUNTIME_TMP_PERIOD_ADDR))
#define RUNTIME_TMP_HYSTERESIS_ADDR (RUNTIME_VAR_BASE + 0x66)
#define TMP_HYSTERESIS (BYTE_REF(RUNTIME_TMP_HYSTERESIS_ADDR))
#define RUNTIME_TMP_LOW_ADDR (RUNTIME_VAR_BASE + 0x67)
#define TMP_LOW (BYTE_REF(RUNTIME_TMP_LOW_ADDR))
#define RUNTIME_TMP_CUR_ADDR(n) (RUNTIME_VAR_BASE + 0x68 + n)
#define TMP_CUR(n) (BYTE_REF(RUNTIME_TMP_CUR_ADDR(n)))
#define RUNTIME_TSENSOR_ADDR(n) (RUNTIME_VAR_BASE + 0x6E + (n)*2)
#define EFUSE_FT_LOCAL_SENSOR(n) (HALFWORD_REF(RUNTIME_TSENSOR_ADDR(n)))
#define RUNTIME_TMP_VAR_END (RUNTIME_TSENSOR_ADDR(4))
#if (RUNTIME_TMP_VAR_END >= (RUNTIME_TMP_VAR_ADDR + RUNTIME_TMP_VAR_SIZE))
#error "RUNTIME_TMP_VAR overflow!!!"
#endif
#define RUNTIME_GEN_VAR_ADDR (RUNTIME_TMP_VAR_ADDR + RUNTIME_TMP_VAR_SIZE)
#define RUNTIME_GEN_VAR_SIZE (0x40U)
#define RUNTIME_LPMCU_LPNV_ADDR (RUNTIME_VAR_BASE + 0x80)
#define LPM3_FUNC_STATE (WORD_REF(RUNTIME_LPMCU_LPNV_ADDR))
#define RUNTIME_HISEE_DATA_ADDR (RUNTIME_VAR_BASE + 0x90)
#define HISEE_SHARE_DATA (WORD_REF(RUNTIME_HISEE_DATA_ADDR))
#define RUNTIME_OCLDO_CALI_ADDR(n) (RUNTIME_VAR_BASE + 0x94 + ((n) * 4))
#define OCLDO_CALI(n) (WORD_REF(RUNTIME_OCLDO_CALI_ADDR(n)))
#define RUNTIME_OCLDO_EFUSE_VOL_ADDR (RUNTIME_VAR_BASE + 0xA4)
#define OCLDO_EFUSE_VOL (WORD_REF(RUNTIME_OCLDO_EFUSE_VOL_ADDR))
#define RUNTIME_PERIL_DVFS_ADDR(n) (RUNTIME_VAR_BASE + 0xA8 + (n) * 2)
#define PERIL_DVFS_BIAS(n) (HALFWORD_REF(RUNTIME_PERIL_DVFS_ADDR(n)))
#define RUNTIME_PERIL_STAT_ADDR (RUNTIME_VAR_BASE + 0xB0)
#define PERIL_DVFS_BUSSTAT (BYTE_REF(RUNTIME_PERIL_STAT_ADDR))
#define RUNTIME_PERIL_VOL_ADAPT_ADDR (RUNTIME_VAR_BASE + 0xB4)
#define PERIL_VOL_ADAPT (BYTE_REF(RUNTIME_PERIL_VOL_ADAPT_ADDR))
#define RUNTIME_GEN_VAR_END (RUNTIME_PERIL_VOL_ADAPT_ADDR)
#if (RUNTIME_GEN_VAR_END >= (RUNTIME_GEN_VAR_ADDR + RUNTIME_GEN_VAR_SIZE))
#error "RUNTIME_GEN_VAR overflow!!!"
#endif
#define RUNTIME_DDR_VAR_ADDR (RUNTIME_GEN_VAR_ADDR + RUNTIME_GEN_VAR_SIZE)
#define RUNTIME_DDR_STATUS_ADDR (RUNTIME_DDR_VAR_ADDR)
#define DDR_STATUS (HALFWORD_REF(RUNTIME_DDR_STATUS_ADDR))
#define RUNTIME_DDR_INITFREQ_ADDR (RUNTIME_DDR_VAR_ADDR + 0x002)
#define INITFREQ (BYTE_REF(RUNTIME_DDR_INITFREQ_ADDR))
#define RUNTIME_DDR_TMP_PERIOD_ADDR (RUNTIME_DDR_VAR_ADDR + 0x003)
#define DDR_TMP_PERIOD (BYTE_REF(RUNTIME_DDR_TMP_PERIOD_ADDR))
#define RUNTIME_DDR_PD_PERIOD_ADDR (RUNTIME_DDR_VAR_ADDR + 0x004)
#define DDR_PD_PRD (HALFWORD_REF(RUNTIME_DDR_PD_PERIOD_ADDR))
#define RUNTIME_DDR_ASREF_PERIOD_ADDR (RUNTIME_DDR_VAR_ADDR + 0x006)
#define DDR_ASREF_PRD (HALFWORD_REF(RUNTIME_DDR_ASREF_PERIOD_ADDR))
#define RUNTIME_DDR_FREQ_LOAD_ADDR (RUNTIME_DDR_VAR_ADDR + 0x008)
#define DDR_FREQ_LOAD(n) (HALFWORD_REF(RUNTIME_DDR_FREQ_LOAD_ADDR + ((n) * 2)))
#define RUNTIME_DDR_MIN_ADDR (RUNTIME_DDR_VAR_ADDR + 0x018)
#define DDR_MIN (BYTE_REF(RUNTIME_DDR_MIN_ADDR))
#define RUNTIME_DDR_MAX_ADDR (RUNTIME_DDR_VAR_ADDR + 0x019)
#define DDR_MAX (BYTE_REF(RUNTIME_DDR_MAX_ADDR))
#define RUNTIME_DDR_LAST_ADDR (RUNTIME_DDR_VAR_ADDR + 0x01A)
#define DDR_LAST (BYTE_REF(RUNTIME_DDR_LAST_ADDR))
#define RUNTIME_DDR_CURRENT_ADDR (RUNTIME_DDR_VAR_ADDR + 0x01B)
#define DDR_CURRENT (BYTE_REF(RUNTIME_DDR_CURRENT_ADDR))
#define RUNTIME_DDR_TARGET_ADDR (RUNTIME_DDR_VAR_ADDR + 0x01C)
#define DDR_TARGET (BYTE_REF(RUNTIME_DDR_TARGET_ADDR))
#define RUNTIME_DDR_DN_LIMIT_ADDR (RUNTIME_DDR_VAR_ADDR + 0x01D)
#define DDR_DN_LIMIT (BYTE_REF(RUNTIME_DDR_DN_LIMIT_ADDR))
#define RUNTIME_DDR_UP_LIMIT_ADDR (RUNTIME_DDR_VAR_ADDR + 0x01E)
#define DDR_UP_LIMIT (BYTE_REF(RUNTIME_DDR_UP_LIMIT_ADDR))
#define RUNTIME_DDR_PLL_ADDR (RUNTIME_DDR_VAR_ADDR + 0x01F)
#define DDR_PLL (BYTE_REF(RUNTIME_DDR_PLL_ADDR))
#define RUNTIME_DDR_LAST_PLL_ADDR (RUNTIME_DDR_VAR_ADDR + 0x020)
#define DDR_LAST_PLL (BYTE_REF(RUNTIME_DDR_LAST_PLL_ADDR))
#define RUNTIME_CMD_CNT_ADDR (RUNTIME_DDR_VAR_ADDR + 0x024)
#define CMD_CNT (WORD_REF(RUNTIME_CMD_CNT_ADDR))
#define RUNTIME_DATA_CNT_ADDR (RUNTIME_DDR_VAR_ADDR + 0x034)
#define DATA_CNT (WORD_REF(RUNTIME_DATA_CNT_ADDR))
#define RUNTIME_DDR_RUNTIME_END_ADDR (RUNTIME_DATA_CNT_ADDR)
#define RUNTIME_SHARE_MEM_SIZE (0x100)
#define RUNTIME_SHARE_MEM_END_ADDR (RUNTIME_VAR_BASE + RUNTIME_SHARE_MEM_SIZE)
#if (RUNTIME_DDR_RUNTIME_END_ADDR >= RUNTIME_SHARE_MEM_END_ADDR)
 #error "runtime share memory overflow!!!"
#endif
#define RUNTIME_LPMCU_RUN_VAR_SIZE (0x40)
#define RUNTIME_LPM3_CFG_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x000)
#define LPM3_CFG (BYTE_REF(RUNTIME_LPM3_CFG_ADDR))
#define RUNTIME_TASK_CYCLE_STATUS_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x001)
#define TASK_CYCLE_STATUS (BYTE_REF(RUNTIME_TASK_CYCLE_STATUS_ADDR))
#define RUNTIME_MAX_IRQ_MAST_PERIOD_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x004)
#define MAX_IRQ_MAST_PERIOD (WORD_REF(RUNTIME_MAX_IRQ_MAST_PERIOD_ADDR))
#define RUNTIME_MAX_IRQ_MASK_FUNC_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x008)
#define MAX_IRQ_MASK_FUNC (WORD_REF(RUNTIME_MAX_IRQ_MASK_FUNC_ADDR))
#define RUNTIME_IRQ_MAST_PERIOD_SUM_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x00C)
#define IRQ_MAST_PERIOD_SUM (WORD_REF(RUNTIME_IRQ_MAST_PERIOD_SUM_ADDR))
#define RUNTIME_IRQ_MASK_TIMES_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x010)
#define IRQ_MASK_TIMES (WORD_REF(RUNTIME_IRQ_MASK_TIMES_ADDR))
#define RUNTIME_SYNC_IPC_TASK_MSG_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x014)
#define SYNC_IPC_TASK_MSG (WORD_REF(RUNTIME_SYNC_IPC_TASK_MSG_ADDR))
#define RUNTIME_ASYNC_IPC_TASK_MSG_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x018)
#define ASYNC_IPC_TASK_MSG (WORD_REF(RUNTIME_ASYNC_IPC_TASK_MSG_ADDR))
#define RUNTIME_LPM3_DDR_LOG_WP_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x01C)
#define LPM3_DDR_LOG_WP (WORD_REF(RUNTIME_LPM3_DDR_LOG_WP_ADDR))
#define RUNTIME_LPM3_DDR_LOG_RP_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x020)
#define LPM3_DDR_LOG_RP (WORD_REF(RUNTIME_LPM3_DDR_LOG_RP_ADDR))
#define RUNTIME_LPM3_DDR_LOG_FLAG_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x024)
#define LPM3_DDR_LOG_FLAG (WORD_REF(RUNTIME_LPM3_DDR_LOG_FLAG_ADDR))
#define RUNTIME_AUDIO_STATE_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x028)
#define AUDIO_STATE (WORD_REF(RUNTIME_AUDIO_STATE_ADDR))
#define RUNTIME_ASP_CLK_STATE_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x02C)
#define ASP_CLK_STATE (WORD_REF(RUNTIME_ASP_CLK_STATE_ADDR))
#define RUNTIME_PLL_UNLOCK_STAT_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x030)
#define PLL_UNLOCK_STAT (WORD_REF(RUNTIME_PLL_UNLOCK_STAT_ADDR))
#define RUNTIME_RTX_TIMER_CALLBACK_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x034)
#define RTX_TIMER_CALLBACK (WORD_REF(RUNTIME_RTX_TIMER_CALLBACK_ADDR))
#define RUNTIME_DDR_UNAVAILABLE_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x038)
#define DDR_UNAVAILABLE_STATE (WORD_REF(RUNTIME_DDR_UNAVAILABLE_ADDR))
#define RUNTIME_ASP_CLK_STATE2_ADDR (RUNTIME_SHARE_MEM_END_ADDR + 0x03C)
#define ASP_CLK_STATE2 (WORD_REF(RUNTIME_ASP_CLK_STATE2_ADDR))
#define RUNTIME_LPMCU_RUN_VAR_END_ADDR (RUNTIME_ASP_CLK_STATE2_ADDR)
#if (RUNTIME_LPMCU_RUN_VAR_END_ADDR >= (RUNTIME_SHARE_MEM_END_ADDR + RUNTIME_LPMCU_RUN_VAR_SIZE))
 #error "runtime lpmcu run memory overflow!!!"
#endif
#define RUNTIME_SR_VAR_BASE_ADDR (RUNTIME_SHARE_MEM_END_ADDR + RUNTIME_LPMCU_RUN_VAR_SIZE)
#define RUNTIME_WAKE_STATUS_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x000)
#define WAKE_STATUS (BYTE_REF(RUNTIME_WAKE_STATUS_ADDR))
#define RUNTIME_DEEP_SLEEP_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x004)
#define SYS_DSLEEP_CNT (WORD_REF(RUNTIME_DEEP_SLEEP_CNT_ADDR))
#define RUNTIME_DEEP_WAKE_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x008)
#define SYS_DWAKE_CNT (WORD_REF(RUNTIME_DEEP_WAKE_CNT_ADDR))
#define RUNTIME_SLEEP_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x00C)
#define SYS_SLEEP_CNT (WORD_REF(RUNTIME_SLEEP_CNT_ADDR))
#define RUNTIME_SLEEP_WAKE_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x010)
#define SYS_SLEEP_WAKE_CNT (WORD_REF(RUNTIME_SLEEP_WAKE_CNT_ADDR))
#define RUNTIME_SLOW_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x014)
#define SYS_SLOW_CNT (WORD_REF(RUNTIME_SLOW_CNT_ADDR))
#define RUNTIME_SLOW_WAKE_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x018)
#define SYS_SLOW_WAKE_CNT (WORD_REF(RUNTIME_SLOW_WAKE_CNT_ADDR))
#define RUNTIME_LIGHT_SLEEP_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x01C)
#define LIGHT_SLEEP_CNT (WORD_REF(RUNTIME_LIGHT_SLEEP_CNT_ADDR))
#define RUNTIME_LIGHT_SLEEP_RESUME_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x020)
#define LIGHT_SLEEP_RESUME_CNT (WORD_REF(RUNTIME_LIGHT_SLEEP_RESUME_CNT_ADDR))
#define RUNTIME_AP_WAKE_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x024)
#define AP_WAKE_CNT (WORD_REF(RUNTIME_AP_WAKE_CNT_ADDR))
#define RUNTIME_HIFI_WAKE_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x028)
#define HIFI_WAKE_CNT (WORD_REF(RUNTIME_HIFI_WAKE_CNT_ADDR))
#define RUNTIME_MODEM_WAKE_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x02C)
#define MODEM_WAKE_CNT (WORD_REF(RUNTIME_MODEM_WAKE_CNT_ADDR))
#define RUNTIME_IOMCU_WAKE_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x030)
#define IOM3_WAKE_CNT (WORD_REF(RUNTIME_IOMCU_WAKE_CNT_ADDR))
#define RUNTIME_LPMCU_WAKE_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x034)
#define LPM3_WAKE_CNT (WORD_REF(RUNTIME_LPMCU_WAKE_CNT_ADDR))
#define RUNTIME_HISEE_WAKE_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x038)
#define HISEE_WAKE_CNT (WORD_REF(RUNTIME_HISEE_WAKE_CNT_ADDR))
#define RUNTIME_AP_SUSPEND_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x03C)
#define AP_SUSPEND_CNT (WORD_REF(RUNTIME_AP_SUSPEND_CNT_ADDR))
#define RUNTIME_AP_RESUME_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x040)
#define AP_RESUME_CNT (WORD_REF(RUNTIME_AP_RESUME_CNT_ADDR))
#define RUNTIME_MODEM_SUSPEND_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x044)
#define MODEM_SUSPEND_CNT (WORD_REF(RUNTIME_MODEM_SUSPEND_CNT_ADDR))
#define RUNTIME_MODEM_RESUME_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x048)
#define MODEM_RESUME_CNT (WORD_REF(RUNTIME_MODEM_RESUME_CNT_ADDR))
#define RUNTIME_HIFI_SUSPEND_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x04C)
#define HIFI_SUSPEND_CNT (WORD_REF(RUNTIME_HIFI_SUSPEND_CNT_ADDR))
#define RUNTIME_HIFI_RESUME_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x050)
#define HIFI_RESUME_CNT (WORD_REF(RUNTIME_HIFI_RESUME_CNT_ADDR))
#define RUNTIME_IOMCU_SUSPEND_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x054)
#define IOM3_SUSPEND_CNT (WORD_REF(RUNTIME_IOMCU_SUSPEND_CNT_ADDR))
#define RUNTIME_IOMCU_RESUME_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x058)
#define IOM3_RESUME_CNT (WORD_REF(RUNTIME_IOMCU_RESUME_CNT_ADDR))
#define RUNTIME_HISEE_SUSPEND_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x05C)
#define HISEE_SUSPEND_CNT (WORD_REF(RUNTIME_HISEE_SUSPEND_CNT_ADDR))
#define RUNTIME_HISEE_RESUME_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x060)
#define HISEE_RESUME_CNT (WORD_REF(RUNTIME_HISEE_RESUME_CNT_ADDR))
#define RUNTIME_LIGHTSLEEP_WAKE_IRQ_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x064)
#define LIGHTSLEEP_WAKE_IRQ (WORD_REF(RUNTIME_LIGHTSLEEP_WAKE_IRQ_ADDR))
#define RUNTIME_LIGHTSLEEP_WAKE_IRQ1_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x068)
#define LIGHTSLEEP_WAKE_IRQ1 (WORD_REF(RUNTIME_LIGHTSLEEP_WAKE_IRQ1_ADDR))
#define RUNTIME_SLOW_WAKE_IRQ_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x06C)
#define SLOW_WAKE_IRQ (WORD_REF(RUNTIME_SLOW_WAKE_IRQ_ADDR))
#define RUNTIME_SLOW_WAKE_IRQ1_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x070)
#define SLOW_WAKE_IRQ1 (WORD_REF(RUNTIME_SLOW_WAKE_IRQ1_ADDR))
#define RUNTIME_SLEEP_WAKE_IRQ_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x074)
#define SLEEP_WAKE_IRQ (WORD_REF(RUNTIME_SLEEP_WAKE_IRQ_ADDR))
#define RUNTIME_SLEEP_WAKE_IRQ1_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x078)
#define SLEEP_WAKE_IRQ1 (WORD_REF(RUNTIME_SLEEP_WAKE_IRQ1_ADDR))
#define RUNTIME_WAKE_IRQ_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x07C)
#define WAKE_IRQ (WORD_REF(RUNTIME_WAKE_IRQ_ADDR))
#define RUNTIME_WAKE_IRQ1_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x080)
#define WAKE_IRQ1 (WORD_REF(RUNTIME_WAKE_IRQ1_ADDR))
#define RUNTIME_AP_WAKE_IRQ_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x084)
#define AP_WAKE_IRQ (WORD_REF(RUNTIME_AP_WAKE_IRQ_ADDR))
#define RUNTIME_MODEM_WAKE_IRQ_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x088)
#define MODEM_WAKE_IRQ (WORD_REF(RUNTIME_MODEM_WAKE_IRQ_ADDR))
#define RUNTIME_HIFI_WAKE_IRQ_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x08C)
#define HIFI_WAKE_IRQ (WORD_REF(RUNTIME_HIFI_WAKE_IRQ_ADDR))
#define RUNTIME_AP_NSIPC_IRQ_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x090)
#define AP_NSIPC_IRQ (WORD_REF(RUNTIME_AP_NSIPC_IRQ_ADDR))
#define RUNTIME_AP_AO_NSIPC_IRQ_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x094)
#define AP_AO_NSIPC_IRQ (WORD_REF(RUNTIME_AP_AO_NSIPC_IRQ_ADDR))
#define RUNTIME_IOMCU_WAKEIRQ_CNT_ADDR (RUNTIME_SR_VAR_BASE_ADDR + 0x098)
#define IOMCU_WAKEIRQ_CNT (WORD_REF(RUNTIME_IOMCU_WAKEIRQ_CNT_ADDR))
#define RUNTIME_SPACE_ADDR_END (RUNTIME_IOMCU_WAKEIRQ_CNT_ADDR)
#if (RUNTIME_SPACE_ADDR_END >= (RUNTIME_VAR_BASE + RUNTIME_VAR_SIZE))
 #error "runtime space overflow!!!"
#endif
#define WAKE_STATUS_OFFSET (RUNTIME_WAKE_STATUS_ADDR - RUNTIME_VAR_BASE)
#define SYS_DSLEEP_CNT_OFFSET (RUNTIME_DEEP_SLEEP_CNT_ADDR - RUNTIME_VAR_BASE)
#define SYS_DWAKE_CNT_OFFSET (RUNTIME_DEEP_WAKE_CNT_ADDR - RUNTIME_VAR_BASE)
#define SYS_SLEEP_CNT_OFFSET (RUNTIME_SLEEP_CNT_ADDR - RUNTIME_VAR_BASE)
#define LIGHT_SLEEP_CNT_OFFSET (RUNTIME_LIGHT_SLEEP_CNT_ADDR - RUNTIME_VAR_BASE)
#define LIGHT_SLEEP_RESUME_CNT_OFFSET (RUNTIME_LIGHT_SLEEP_RESUME_CNT_ADDR - RUNTIME_VAR_BASE)
#define AP_WAKE_CNT_OFFSET (RUNTIME_AP_WAKE_CNT_ADDR - RUNTIME_VAR_BASE)
#define MODEM_WAKE_CNT_OFFSET (RUNTIME_MODEM_WAKE_CNT_ADDR - RUNTIME_VAR_BASE)
#define HIFI_WAKE_CNT_OFFSET (RUNTIME_HIFI_WAKE_CNT_ADDR - RUNTIME_VAR_BASE)
#define IOMCU_WAKE_CNT_OFFSET (RUNTIME_IOMCU_WAKE_CNT_ADDR - RUNTIME_VAR_BASE)
#define LPM3_WAKE_CNT_OFFSET (RUNTIME_LPMCU_WAKE_CNT_ADDR - RUNTIME_VAR_BASE)
#define AP_SUSPEND_CNT_OFFSET (RUNTIME_AP_SUSPEND_CNT_ADDR - RUNTIME_VAR_BASE)
#define AP_RESUME_CNT_OFFSET (RUNTIME_AP_RESUME_CNT_ADDR - RUNTIME_VAR_BASE)
#define MODEM_SUSPEND_CNT_OFFSET (RUNTIME_MODEM_SUSPEND_CNT_ADDR - RUNTIME_VAR_BASE)
#define MODEM_RESUME_CNT_OFFSET (RUNTIME_MODEM_RESUME_CNT_ADDR - RUNTIME_VAR_BASE)
#define HIFI_SUSPEND_CNT_OFFSET (RUNTIME_HIFI_SUSPEND_CNT_ADDR - RUNTIME_VAR_BASE)
#define HIFI_RESUME_CNT_OFFSET (RUNTIME_HIFI_RESUME_CNT_ADDR - RUNTIME_VAR_BASE)
#define IOMCU_SUSPEND_CNT_OFFSET (RUNTIME_IOMCU_SUSPEND_CNT_ADDR - RUNTIME_VAR_BASE)
#define IOMCU_RESUME_CNT_OFFSET (RUNTIME_IOMCU_RESUME_CNT_ADDR - RUNTIME_VAR_BASE)
#define HISEE_SUSPEND_CNT_OFFSET (RUNTIME_HISEE_SUSPEND_CNT_ADDR - RUNTIME_VAR_BASE)
#define HISEE_RESUME_CNT_OFFSET (RUNTIME_HISEE_RESUME_CNT_ADDR - RUNTIME_VAR_BASE)
#define SLEEP_WAKE_IRQ_OFFSET (RUNTIME_SLEEP_WAKE_IRQ_ADDR - RUNTIME_VAR_BASE)
#define SLEEP_WAKE_IRQ1_OFFSET (RUNTIME_SLEEP_WAKE_IRQ1_ADDR - RUNTIME_VAR_BASE)
#define WAKE_IRQ_OFFSET (RUNTIME_WAKE_IRQ_ADDR - RUNTIME_VAR_BASE)
#define WAKE_IRQ1_OFFSET (RUNTIME_WAKE_IRQ1_ADDR - RUNTIME_VAR_BASE)
#define AP_WAKE_IRQ_OFFSET (RUNTIME_AP_WAKE_IRQ_ADDR - RUNTIME_VAR_BASE)
#define AP_NSIPC_IRQ_OFFSET (RUNTIME_AP_NSIPC_IRQ_ADDR - RUNTIME_VAR_BASE)
#define AP_AO_NSIPC_IRQ_OFFSET (RUNTIME_AP_AO_NSIPC_IRQ_ADDR - RUNTIME_VAR_BASE)
#define RUNTIME_SUBSYS_WAKE_CNT_ADDR(n) (RUNTIME_AP_WAKE_CNT_ADDR + ((n) << 2))
#define SUBSYS_WAKE_CNT(n) (WORD_REF(RUNTIME_SUBSYS_WAKE_CNT_ADDR(n)))
#define GET_OBJ(obj_feature) ((obj_feature) >> 16)
#define GET_FEATURE(obj_feature) ((obj_feature) & 0xFFFF)
typedef enum MODULE_IP
{
 LP_LIT_CPU = 0,
 LP_BIG_CPU = 1,
 LP_GPU = 2,
 LP_DDR = 3,
 LP_TMP = 4,
 LP_LPM3 = 5,
 LP_IOMCU,
 LP_ACPU,
 LP_PERI,
 LP_HIFI,
 LP_MODEM,
 LP_MMBUF,
 LP_MODULE_IP_MAX,
} MODULE_IP_T;
typedef enum MODULE_FUNC
{
 LP_LCPU_VDM = (LP_LIT_CPU << 16) | 0x0001,
 LP_LCPU_AVS = (LP_LIT_CPU << 16) | 0x0003,
 LP_BCPU_VDM = (LP_BIG_CPU << 16) | 0x0001,
 LP_BCPU_AVS = (LP_BIG_CPU << 16) | 0x0003,
 LP_GPU_VDM = (LP_GPU << 16) | 0x0001,
 LP_GPU_AVS = (LP_GPU << 16) | 0x0003,
 LP_DDR_TMP_ENABLE = (LP_DDR << 16) | DDR_TMP_OFFSET,
 LP_DDR_DFS = (LP_DDR << 16) | DDR_DFS_OFFSET,
 LP_DDR_AFS = (LP_DDR << 16) | DDR_AFS_OFFSET,
 LP_DDR_TMP_NREC = (LP_DDR << 16) | DDR_TMP_NREC_OFFSET,
 LP_DDR_TMP_NRST = (LP_DDR << 16) | DDR_TMP_NRST_OFFSET,
 LP_DDR_TMP_SHANRST = (LP_DDR << 16) | DDR_TMP_SHANRST_OFFSET,
 LP_DDR_UCE_UART_DIS = (LP_DDR << 16) | DDR_UCE_UART_DIS_OFFSET,
 LP_DDR_TRAIN_TRACK_DIS = (LP_DDR << 16) | DDR_TRAIN_TRACK_DIS_OFFSET,
 LP_TMP_TCTRL = (LP_TMP << 16) | TMP_OP_OFFSET,
 LP_TSENSOR_RESET = (LP_TMP << 16) | TMP_TSEN_RST_ENABLE_OFFSET,
 LP_LPM3_SYS_SR = (LP_LPM3 << 16) | 0x0000,
 LP_LPM3_ACPU_SR = (LP_LPM3 << 16) | 0x0001,
 LP_LPM3_IOMCU_SR = (LP_LPM3 << 16) | 0x0002,
 LP_LPM3_HIFI_SR = (LP_LPM3 << 16) | 0x0003,
 LP_LPM3_GPU_SR = (LP_LPM3 << 16) | 0x0004,
 LP_LPM3_MODEM_SR = (LP_LPM3 << 16) | 0x0005,
 LP_LPM3_SYS_S0 = (LP_LPM3 << 16) | 0x0006,
 LP_LPM3_SYS_S1 = (LP_LPM3 << 16) | 0x0007,
 LP_LPM3_SYS_S2 = (LP_LPM3 << 16) | 0x0008,
 LP_LPM3_SYS_S3 = (LP_LPM3 << 16) | 0x0009,
 LP_LPM3_BUS_DFS = (LP_LPM3 << 16) | 0x000A,
 LP_LPM3_PERI_AVS = (LP_LPM3 << 16) | 0x000B,
 LP_LPM3_HIFI_ULLP = (LP_LPM3 << 16) | 0x000C,
 LP_LPM3_MMBUF = (LP_LPM3 << 16) | 0x000D,
 LP_LPM3_PERIL_VOL = (LP_LPM3 << 16) | 0x000E,
 LP_LPM3_DDRFSGT = (LP_LPM3 << 16) | 0x000F,
 LP_LPM3_DDRFSGT_PP = (LP_LPM3 << 16) | 0x0010,
 LP_LPM3_DDRCRGGT = (LP_LPM3 << 16) | 0x0011,
 LP_LPM3_CPU_DDR_LINK = (LP_LPM3 << 16) | 0x0012,
 LP_LPM3_GPU_DDR_LINK = (LP_LPM3 << 16) | 0x0013,
} MODULE_FUNC_T;
#define LP_FEATURE_MAX 31
#define FUNCTION_ENABLE 1
#define FUNCTION_DISABLE 0
#endif
