# MD5: ac018c573938c70c2b4a8dad236ad0ac
CFG_BBP_MASTER_NONE                             := 0
CFG_BBP_MASTER_VER1                             := 1
CFG_BBP_MASTER_VER2                             := 2
CFG_BBP_MASTER_VER3                             := 3
CFG_BBP_MASTER_VER4                             := 4
CFG_BBP_MASTER_VER5                             := 5
CFG_BBP_MASTER_VER6                             := 6
CFG_FEATURE_BBP_MASTER_VER                      := (BBP_MASTER_VER5)
CFG_BBP_MASTER_ES                               := 0
CFG_BBP_MASTER_CS                               := 1
ifeq ($(CFG_CHIP_TYPE_CS),FEATURE_ON)
CFG_BBP_MASTER_CHIP_TYPE                        := BBP_MASTER_CS
else
CFG_BBP_MASTER_CHIP_TYPE                        := BBP_MASTER_ES
endif
CFG_FEATURE_POWER_TIMER                         := FEATURE_OFF
CFG_FEATURE_UE_UICC_MULTI_APP_SUPPORT           := FEATURE_ON
CFG_FEATURE_VSIM                                := FEATURE_ON
CFG_FEATURE_GUC_BBP_TRIG                        := FEATURE_ON
CFG_FEATURE_GUC_BBP_TRIG_NEWVERSION             := FEATURE_ON
CFG_FEATURE_VOS_REDUCE_MEM_CFG                  := FEATURE_OFF
CFG_FEATURE_RTC_TIMER_DBG                       := FEATURE_OFF
CFG_FEATURE_PHONE_SC                            := FEATURE_ON
CFG_FEATURE_SC_NETWORK_UPDATE                   := FEATURE_OFF
CFG_FEATURE_SC_SIGNATURE_UPDATE                 := FEATURE_ON
CFG_FEATURE_VSIM_ICC_SEC_CHANNEL                := FEATURE_ON
CFG_FEATURE_BOSTON_AFTER_FEATURE                := FEATURE_ON
CFG_FEATURE_XSIM                                := FEATURE_OFF
