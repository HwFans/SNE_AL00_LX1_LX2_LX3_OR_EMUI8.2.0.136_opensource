/*
	*drivers/power/huawei_charger.c
	*
	*huawei	charger	driver
	*
	*Copyright(C)2012-2015 HUAWEI, Inc.
	*Author: HUAWEI, Inc.
	*
	*This package is free software; you can	redistribute it and/or modify
	*it under the terms of the GNU General Public License version 2 as
	*published by the Free Software Foundation.
*/
#include <huawei_platform/power/direct_charger.h>
#include <huawei_platform/power/wired_channel_switch.h>


#define HWLOG_TAG direct_charge
HWLOG_REGIST();

static struct dc_volt_para_info orig_volt_para[DC_VOLT_LEVEL];
static struct direct_charge_device *g_di;
struct smart_charge_ops* g_scp_ops;
struct scp_power_supply_ops* g_scp_ps_ops;
struct direct_charge_cable_detect_ops* g_cable_detect_ops;

/*PT set adator test result*/
extern void chg_set_adaptor_test_result(enum adaptor_name charger_type, enum test_state result);
extern int pd_dpm_notify_direct_charge_status(bool dc);
static int battery_temp_handler(int temp);
int pmic_vbus_irq_is_enabled(void);

void direct_charge_get_g_scp_ops(struct smart_charge_ops **ops)
{
	*ops = g_scp_ops;
}

void direct_charge_get_g_scp_ps_ops(struct scp_power_supply_ops **ops)
{
	*ops = g_scp_ps_ops;
}

void direct_charge_get_g_cable_detect_ops(struct atomic_notifier_head **ops)
{
	*ops = g_cable_detect_ops;
}

void direct_charge_check(void)
{
	int adap_mode = 0;
	int local_mode= 0;
	int ret = 0;
	struct direct_charge_device *di = NULL;

	scp_adaptor_type_detect(&adap_mode);
	local_mode = direct_charge_get_local_mode();
	hwlog_info("direct_charge [adaptor mode] = [%d], [local mode] = [%d]!\n", adap_mode, local_mode);

	if ((adap_mode & local_mode) & SC_MODE)
	{
		ret = direct_charge_get_sc_di(&di);

		if (ret == 0)
		{
			di->adaptor_test_result_type = TYPE_SC;
			di->scp_ops->scp_set_direct_charge_mode(SC_MODE);
			direct_charge_set_di(di);
			chg_set_adaptor_test_result(TYPE_SC,DETECT_SUCC);
			direct_charge_sc_check();
		}
	}
	else if ((adap_mode & local_mode) & LVC_MODE)
	{
		ret = direct_charge_get_lvc_di(&di);

		if (ret == 0)
		{
			di->adaptor_test_result_type = TYPE_SCP;
			di->scp_ops->scp_set_direct_charge_mode(LVC_MODE);
			direct_charge_set_di(di);
			chg_set_adaptor_test_result(TYPE_SCP,DETECT_SUCC);
			direct_charge_lvc_check();
		}
	}
}
void direct_charge_set_di(struct direct_charge_device *di)
{
	if (di != NULL)
	{
		g_di = di;
	}
	else
	{
		hwlog_err("direct_charge_set_di fail!\n");
	}
}

int cable_detect_ops_register(struct direct_charge_cable_detect_ops* ops)
{
	int ret = 0;

	if (ops != NULL)
	{
		g_cable_detect_ops = ops;
	}
	else
	{
		hwlog_err("cable detect ops register fail!\n");
		ret = -EPERM;
	}
	return ret;
}
int scp_ops_register(struct smart_charge_ops* ops)
{
	int ret = 0;

	if (ops != NULL)
	{
		g_scp_ops = ops;
	}
	else
	{
		hwlog_err("scp ops register fail!\n");
		ret = -EPERM;
	}
	return ret;
}

int scp_power_supply_ops_register(struct scp_power_supply_ops* ops)
{
	int ret = 0;

	if (ops != NULL)
	{
		g_scp_ps_ops = ops;
	}
	else
	{
		hwlog_err("scp power ops register fail!\n");
		ret = -EPERM;
	}
	return ret;
}


int get_quick_charge_flag(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return 0;
	}
	di = g_di;

	return di->quick_charge_flag;
}
int get_super_charge_flag(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return 0;
	}
	di = g_di;

	return di->super_charge_flag;
}
static void direct_charge_send_quick_charge_uevent(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}
	di = g_di;

	di->quick_charge_flag = 1;
	di->super_charge_flag = 0;
	direct_charger_connect_send_uevent();
}
static void direct_charge_send_super_charge_uevent(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}

	di = g_di;

	di->quick_charge_flag = 0;
	di->super_charge_flag = 1;
	direct_charger_connect_send_uevent();
}
void direct_charge_send_normal_charge_uevent(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}

	di = g_di;

	di->quick_charge_flag = 0;
	di->super_charge_flag = 0;
	direct_charger_connect_send_uevent();
}

int direct_charge_get_cutoff_normal_flag(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return 0;
	}
	di = g_di;

	return di->cutoff_normal_flag;
}

/**********************************************************
*  Function:       direct_charge_wake_lock
*  Description:   apply direct_charge wake_lock
*  Parameters:   NULL
*  return value:  NULL
**********************************************************/
void direct_charge_wake_lock(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}
	di = g_di;

	if (!wake_lock_active(&di->direct_charge_lock)) {
		wake_lock(&di->direct_charge_lock);
		hwlog_info("direct_charge wake lock\n");
	}
}

/**********************************************************
*  Function:       direct_charge_wake_unlock
*  Description:   release direct_charge wake_lock
*  Parameters:   NULL
*  return value:  NULL
**********************************************************/
void direct_charge_wake_unlock(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}
	di = g_di;

	if (wake_lock_active(&di->direct_charge_lock)) {
		wake_unlock(&di->direct_charge_lock);
		hwlog_info("direct_charge wake unlock\n");
	}
}

int is_in_scp_charging_stage(void)
{
	int ret = 0;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return NOT_IN_SCP_CHARGING_STAGE;
	}
	di = g_di;

	if (SCP_STAGE_CHARGING == di->scp_stage)
	{
		hwlog_info("in direct charge progress!\n");
		return IN_SCP_CHARGING_STAGE;
	}
	return NOT_IN_SCP_CHARGING_STAGE;
}
/**********************************************************
*  Function:       scp_get_stage_status
*  Description:    get the stage of scp charge
*  Parameters:
*  return value:   stage
**********************************************************/
enum scp_stage_type scp_get_stage_status(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return SCP_STAGE_DEFAULT;
	}
	di = g_di;
	return di->scp_stage;
}

/**********************************************************
*  Function:       scp_set_stage_status
*  Description:    set the stage of scp charge
*  Parameters:     stage type
*  return value:   NULL
**********************************************************/
void scp_set_stage_status(enum scp_stage_type stage_type)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}
	di = g_di;
	di->scp_stage = stage_type;
}

int get_bat_voltage(void)
{
	int btb_vol = 0;
	int package_vol = 0;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return 0;
	}
	di = g_di;

	btb_vol = di->bi_ops->get_bat_btb_voltage();
	package_vol = di->bi_ops->get_bat_package_voltage();
	if (btb_vol < 0 && package_vol < 0)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
		hwlog_err("%s", tmp_buf);
		strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
		di->scp_stop_charging_flag_error = 1;
		return 0;
	}

	return btb_vol > package_vol ? btb_vol : package_vol;
}

int get_bat_current(void)
{
	int bat_curr = 0;
	int ret;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	ret = di->bi_ops->get_bat_current(&bat_curr);
	if (ret < 0)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
		hwlog_err("%s", tmp_buf);
		strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
		di->scp_stop_charging_flag_error = 1;
	}
	return bat_curr;
}

int get_ls_vbus(void)
{
	int vbus = 0;
	int ret;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	ret = di->bi_ops->get_vbus_voltage(&vbus);
	if (ret < 0)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
		hwlog_err("%s", tmp_buf);
		strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
		di->scp_stop_charging_flag_error = 1;
	}
	return vbus;
}

int get_ls_ibus(void)
{
	int ibus = 0;
	int ret;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	ret = di->bi_ops->get_ls_ibus(&ibus);
	if (ret < 0)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
		hwlog_err("%s", tmp_buf);
		strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
		di->scp_stop_charging_flag_error = 1;
	}

	return ibus;
}

int get_ls_temp(void)
{
	int temp = 0;
	int ret;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	ret = di->bi_ops->get_ls_temp(&temp);
	if (ret < 0)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
		hwlog_err("%s", tmp_buf);
		strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
		di->scp_stop_charging_flag_error = 1;
	}
	return temp;
}

int is_ls_close(void)
{
	int ret;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return 1;
	}
	di = g_di;

	ret = di->ls_ops->is_ls_close();
	if (ret)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:ls is close\n", __func__);
		hwlog_err("%s", tmp_buf);
		strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
		di->scp_stop_charging_flag_error = 1;
		return 1;
	}
	return 0;
}
void scp_set_stop_charging_flag(int flag)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}
	di = g_di;
	di->scp_stop_charging_flag_error = flag;
}

int is_scp_stop_charging_complete(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return 1;
	}
	di = g_di;
	return (1 == di->scp_stop_charging_complete_flag);
}

int get_adaptor_voltage(void)
{
	int adaptor_vol = -1;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	if(di->scp_stop_charging_flag_error)
	{
		hwlog_err("%s scp_stop_charging_flag_error\n", __func__);
		return -1;
	}
	if (di->scp_ops->scp_get_adaptor_voltage)
	{
		adaptor_vol = di->scp_ops->scp_get_adaptor_voltage();
		if (adaptor_vol < 0)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
			hwlog_err("%s", tmp_buf);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			di->scp_stop_charging_flag_error = 1;
		}
	}
	return adaptor_vol;
}

int get_adaptor_current(void)
{
	int adaptor_cur = -1;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	if(di->scp_stop_charging_flag_error)
		return -1;
	switch(di->adaptor_vendor_id)
	{
		case IWATT_ADAPTER:
			adaptor_cur = get_ls_ibus();
			return adaptor_cur;
		default:
			if (di->scp_ops->scp_get_adaptor_current)
			{
				adaptor_cur = di->scp_ops->scp_get_adaptor_current();
				if (adaptor_cur < 0)
				{
					snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
					hwlog_err("%s", tmp_buf);
					strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
					di->scp_stop_charging_flag_error = 1;
				}
			}
			return adaptor_cur;
	}
}
int get_adaptor_current_for_vbat_ovp(void)
{
	int adaptor_cur = -1;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	switch(di->adaptor_vendor_id)
	{
		case IWATT_ADAPTER:
			adaptor_cur = get_ls_ibus();
			return adaptor_cur;
		default:
			if (di->scp_ops->scp_get_adaptor_current)
			{
				adaptor_cur = di->scp_ops->scp_get_adaptor_current();
			}
			return adaptor_cur;
	}
}


int get_adaptor_current_set(void)
{
	int adaptor_cur_set = -1;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	if(di->scp_stop_charging_flag_error)
		return -1;
	if (di->scp_ops->scp_get_adaptor_current_set)
	{
 		adaptor_cur_set = di->scp_ops->scp_get_adaptor_current_set();
		if (adaptor_cur_set < 0)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
			hwlog_err("%s", tmp_buf);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			di->scp_stop_charging_flag_error = 1;
		}
	}
	return adaptor_cur_set;
}

int get_adaptor_max_current(void)
{
	int adaptor_max_cur = -1;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	if(di->scp_stop_charging_flag_error)
		return -1;
	if (di->scp_ops->scp_get_adaptor_max_current)
	{
		adaptor_max_cur = di->scp_ops->scp_get_adaptor_max_current();
		if (adaptor_max_cur < 0)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
			hwlog_err("%s", tmp_buf);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			di->scp_stop_charging_flag_error = 1;
		}
	}
	return adaptor_max_cur;
}

void set_adaptor_voltage(void)
{
	int ret = -1;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}
	di = g_di;

	if(di->scp_stop_charging_flag_error)
		return;
	if (di->scp_ops->scp_set_adaptor_voltage)
	{
		hwlog_info("set_adaptor_vol = %d!\n", di->adaptor_vset);
		if (di->adaptor_vset >= di->max_adaptor_vset)
		{
			di->adaptor_vset = di->max_adaptor_vset;
		}
		ret = di->scp_ops->scp_set_adaptor_voltage(di->adaptor_vset);
		if (ret)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
			hwlog_err("%s", tmp_buf);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			di->scp_stop_charging_flag_error = 1;
		}
	}
}

void set_adaptor_current(void)
{
	int ret = -1;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;

	if(di->scp_stop_charging_flag_error)
		return;
	if (di->scp_ops->scp_set_adaptor_current)
	{
		hwlog_info("set_adaptor_cur = %d!\n", di->adaptor_iset);
		if (di->adaptor_iset >= di->max_adaptor_iset)
		{
			di->adaptor_iset = di->max_adaptor_iset;
		}
		ret = di->scp_ops->scp_set_adaptor_current(di->adaptor_iset);
		if (ret)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
			hwlog_err("%s", tmp_buf);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			di->scp_stop_charging_flag_error = 1;
		}
	}
}

int get_adaptor_temp(void)
{
	int ret = -1;
	int temp = 0;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return 0;
	}
	di = g_di;

	if(di->scp_stop_charging_flag_error)
		return 0;
	if (di->scp_ops->scp_get_adaptor_temp)
	{
		ret = di->scp_ops->scp_get_adaptor_temp(&temp);
		if (ret)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "[%s]:error\n", __func__);
			hwlog_err("%s", tmp_buf);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			di->scp_stop_charging_flag_error = 1;
		}
	}
	return temp;
}
int can_battery_temp_do_direct_charge(void)
{
	int bat_temp = hisi_battery_temperature();
	int bat_temp_cur_max = battery_temp_handler(bat_temp);
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return 0;
	}
	di = g_di;

	if (0 == bat_temp_cur_max)
	{
		hwlog_info("%s : temp = %d, can not do direct charging \n", __func__, bat_temp);
		return 0;
	}
	return 1;
}
int can_battery_vol_do_direct_charge(void)
{
	int bat_vol = hisi_battery_voltage();
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return 0;
	}
	di = g_di;

	if (bat_vol < di->min_dc_bat_vol || bat_vol > di->max_dc_bat_vol)
	{
		hwlog_info("%s : vol = %d, can not do direct charging \n", __func__, bat_vol);
		return 0;
	}
	return	1;
}

int scp_retry_pre_operate(enum scp_retry_operate_type type, struct direct_charge_device *di)
{
	int ret	= -1;

	switch (type) {
	case SCP_RETRY_OPERATE_RESET_ADAPTER:
		if (NULL != di->scp_ops->scp_adaptor_reset)
		{
			hwlog_info("send scp adapter reset cmd \n");
			ret = di->scp_ops->scp_adaptor_reset();
		}
		else
		{
			ret = -1;
		}
		break;
	case SCP_RETRY_OPERATE_RESET_CHIP:
		if (NULL != di->scp_ops->scp_chip_reset)
		{
			hwlog_info("scp_chip_reset \n");
			ret = di->scp_ops->scp_chip_reset();
			msleep(2000);
		}
		else
		{
			ret = -1;
		}
		break;
	default:
		break;
	}
	return ret;
}

void scp_power_control(int enable)
{
	int ret = 0;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;
	/* power control for SCP communication */
	ret = di->scp_ps_ops->scp_power_enable(enable);
	if(ret)
	{
		hwlog_err("[%s]:fail, status = %d!\n", __func__,enable);
		return;
	}
	hwlog_err("[%s]: success, status = %d!\n", __func__,enable);
	return;
}

int restore_normal_charge(void)
{
	int ret = SUCC;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return FAIL;
	}
	di = g_di;
	hwlog_err("%s \n", __func__);
	msleep(WAIT_LS_DISCHARGE); /*need to wait loadswitch discharge*/
	/*no need to check the return val, here when ovp_en set fail ,we do note return*/
	dp_aux_ldo_supply_disable(DP_AUX_LDO_CTRL_DIRECT_CHARGE);

	ret = wired_chsw_set_wired_channel(WIRED_CHANNEL_RESTORE);
	if (ret)
		ret = FAIL;
	else
		ret = SUCC;

	if (pmic_vbus_irq_is_enabled()) {
		restore_pluggin_pluggout_interrupt();
	}
	return ret;
}
int cutoff_normal_charge(void)
{
	int ret = SUCC;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;
	if (pmic_vbus_irq_is_enabled()) {
		di->cutoff_normal_flag = 1;
		ignore_pluggin_and_pluggout_interrupt();
	}
	dp_aux_ldo_supply_enable(DP_AUX_LDO_CTRL_DIRECT_CHARGE);
	if (di->scp_work_on_charger) {
		scp_power_control(ENABLE);
		charge_set_hiz_enable(SET_HIZ_ENABLE);
	}
	msleep(100);
	ret = wired_chsw_set_wired_channel(WIRED_CHANNEL_CUTOFF);
	hwlog_err("%s \n", __func__);

	return ret;
}
int scp_adaptor_set_output_enable(int enable)
{
	int ret, i;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

    	for (i = 0; i < 3; i++) {
		ret = di->scp_ops->scp_adaptor_output_enable(enable);
		if (!ret)
			break;
	}
	if (ret) {
		hwlog_err("[%s]: direct charge set output voltage fail !!\n", __func__);
	}
	return ret;
}
int _scp_adaptor_detect(struct direct_charge_device *di)
{
	int ret;
	int i;

	if (NULL == di || NULL == di->scp_ops || NULL == di->scp_ops->scp_adaptor_detect)
	{
		hwlog_err("[%s]bad scp adaptor detect ops!\n", __func__);
		return -1;
	}
	ret = di->scp_ops->scp_adaptor_detect();
	if (SCP_ADAPTOR_DETECT_FAIL == ret)
	{
		for (i = 0; i < 3 && SCP_ADAPTOR_DETECT_FAIL == ret; ++i)
		{
			/*check if the adapter has been pulled out */
			if(SCP_STAGE_DEFAULT == scp_get_stage_status())
			{
				hwlog_info("[%s]the adapter has been pulled out,stop adapter detect!\n", __func__);
				return -1;
			}
			if ((scp_retry_pre_operate(SCP_RETRY_OPERATE_RESET_ADAPTER, di)) < 0)
			{
				hwlog_err("reset adapter failed	\n");
				break;
			}
			ret = di->scp_ops->scp_adaptor_detect();
		}
		if (SCP_ADAPTOR_DETECT_FAIL == ret)
		{
			/*check if the adapter has been pulled out */
			if(SCP_STAGE_DEFAULT == scp_get_stage_status())
			{
				hwlog_info("[%s]the adapter has been pulled out,stop adapter detect!\n", __func__);
				return -1;
			}

			/* reset scp chip and try again	*/
			if ((scp_retry_pre_operate(SCP_RETRY_OPERATE_RESET_CHIP, di)) == 0)
			{
				ret = di->scp_ops->scp_adaptor_detect();
			}
			else
			{
				hwlog_err("%s : scp_retry_pre_operate failed \n", __func__);
			}
		}
	}
	hwlog_info("%s : scp adaptor detect ret = %d \n", __func__, ret);
	return ret;
}
void direct_charge_double_56k_cable_detect(void)
{
	int ret;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;
	if (NULL == di || NULL == di->direct_charge_cable_detect || NULL == di->direct_charge_cable_detect->direct_charge_cable_detect) {
		hwlog_err("[%s]NULL direct charge cable detect ops!\n", __func__);
		scp_set_stage_status(SCP_STAGE_DEFAULT);
		return;
	}
	if (di->cc_cable_detect_ok) {
		return;
	}
	ret = di->direct_charge_cable_detect->direct_charge_cable_detect();
	if (ret) {
		di->cc_cable_detect_ok = 0;
		di->full_path_res_threshold = di->full_path_res_max;
		hwlog_info("%s:cable detect fail!\n",__func__);
	} else {
		di->cc_cable_detect_ok = 1;
		di->full_path_res_threshold = di->standard_cable_full_path_res_max;
		direct_charge_send_super_charge_uevent();
		hwlog_info("%s:cable detect ok!\n",__func__);
	}

}

int scp_adaptor_detect(void)
{
	int ret = 0;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;
	ret = _scp_adaptor_detect(di);
	/*try again in the next loop*/
	if (SCP_ADAPTOR_DETECT_SUCC != ret) {
		scp_set_stage_status(SCP_STAGE_DEFAULT);
		return -1;
	}
	chg_set_adaptor_test_result(di->adaptor_test_result_type,DETECT_SUCC);
	return 0;
}

int scp_adaptor_type_detect(int *mode)
{
	int adaptor_support = 0;
	int ret = 0;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	di->direct_charge_succ_flag = DIRECT_CHARGE_ERROR_ADAPTOR_DETECT;
	ret = _scp_adaptor_detect(di);

	if (ret != 0)
	{
		hwlog_info("%s:scp_adaptor_detect fail!\n",__func__);
		scp_set_stage_status(SCP_STAGE_DEFAULT);
		return -1;
	}

	adaptor_support = di->scp_ops->scp_get_adaptor_type();
	*mode = adaptor_support;
	return 0;
}


void scp_cable_detect(void)
{
	int ret = 0;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;
	direct_charge_double_56k_cable_detect();
	if (SUCC == ret)
	{
		ret = di->scp_ops->scp_get_adaptor_info(&(di->adp_info));
		if (ret)
		{
			hwlog_err("get adaptor info failed\n");
		}
		di->adaptor_vendor_id = di->scp_ops->scp_get_adapter_vendor_id();
		if (di->adaptor_vendor_id < 0)
		{
			hwlog_err("get adaptor vendor id failed\n");
			scp_set_stage_status(SCP_STAGE_DEFAULT);
			return;
		}
		hwlog_info("b_adp_type= 0x%x\n", di->adp_info.b_adp_type);
		hwlog_info("vendor_id_h= 0x%x\n", di->adp_info.vendor_id_h);
		hwlog_info("vendor_id_l= 0x%x\n", di->adp_info.vendor_id_l);
		hwlog_info("module_id_h= 0x%x\n", di->adp_info.module_id_h);
		hwlog_info("module_id_l= 0x%x\n", di->adp_info.module_id_l);
		hwlog_info("serrial_no_h= 0x%x\n", di->adp_info.serrial_no_h);
		hwlog_info("serrial_no_l= 0x%x\n", di->adp_info.serrial_no_l);
		hwlog_info("pchip_id= 0x%x\n", di->adp_info.pchip_id);
		hwlog_info("hwver= 0x%x\n", di->adp_info.hwver);
		hwlog_info("fwver_h= 0x%x\n", di->adp_info.fwver_h);
		hwlog_info("fwver_l= 0x%x\n", di->adp_info.fwver_l);
		hwlog_info("adaptor_vendor_id= 0x%x\n", di->adaptor_vendor_id);
		scp_set_stage_status(SCP_STAGE_SWITCH_DETECT);
	}
}


int do_adpator_voltage_accuracy_check(void)
{
	int adp_vol;
	int vol_err;
	int i;
	int ret;
	int bat_curr;
	char buf[1024] = { 0 };
	char dsm_buf[CHARGE_DMDLOG_SIZE] = { 0 };
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };

	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	di->adaptor_vset = di->init_adapter_vset;
	set_adaptor_voltage();
	usleep_range(500000,501000);
	ret = di->bi_ops->get_bat_current(&bat_curr);/*keep communication with the adaptor within 1 second*/
	if (ret)
	{
		hwlog_err("get_bat_current fail!\n");
	}
	hwlog_info("To keep communication with the adaptor:bat_curr = %d!\n",bat_curr);
	for (i = 0; i < 3; ++i)
	{
		adp_vol = get_adaptor_voltage();
		if (adp_vol < 0)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "%s: get adptor voltage(%dmV) fail!\n", __func__, adp_vol);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			hwlog_err("%s", tmp_buf);
			return -1;
		}
		vol_err = adp_vol - di->adaptor_vset;
		if (vol_err < 0)
			vol_err = -vol_err;
		snprintf(tmp_buf, sizeof(tmp_buf),
			"Verr = %d Verr_th = %d, Vset = %d, Vread = %d, Vbus = %d!\n",
			 vol_err, di->vol_err_th, di->adaptor_vset, adp_vol, get_charger_vbus_vol());
		strncat(dsm_buf, tmp_buf, strlen(tmp_buf));
		hwlog_info("%s", tmp_buf);
		if (vol_err > di->vol_err_th)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "Verr(%d) > Verr_th(%d)\n",
				vol_err, di->vol_err_th);
			strncat(dsm_buf, tmp_buf, strlen(tmp_buf));
			strncat(di->dc_err_dsm_buff, dsm_buf, strlen(dsm_buf));
			dsm_report(DSM_DIRECT_CHARGE_VOL_ACCURACY, dsm_buf);
			return -1;
		}
	}
	return 0;
}
int do_full_path_resistance_check(void)
{
	int adp_vol;
	int iadapt;
	int vbus_vol;
	int ibus;
	int delta_vol;
	int r;
	int ret;
	int i;
	int sum = 0;
	char dsm_buf[CHARGE_DMDLOG_SIZE] = { 0 };
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };

	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	usleep_range(500000,501000);
	adp_vol = get_adaptor_voltage(); /*keep communication with the adaptor within 1 second*/
	if (adp_vol < 0)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "%s: get adptor voltage fail, adp_vol = %d\n", __func__, adp_vol);
		hwlog_err("%s", tmp_buf);
		strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
		return -1;
	}
	ret = is_ls_close();/*keep communication with loadswitchwithin 1 second*/
	if (ret)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "%s: ls is close!\n", __func__);
		hwlog_err("%s", tmp_buf);
		strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
		return -1;
	}
	usleep_range(400000,401000);
	for (i = 0; i < 3; ++i)
	{
		ret = di->bi_ops->get_ls_ibus(&ibus);
		if (ret)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "%s:get ibus fail, ls_ibus = %d\n", __func__, ibus);
			hwlog_err("%s", tmp_buf);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			return -1;
		}
		ret = di->bi_ops->get_vbus_voltage(&vbus_vol);
		if (ret)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "%s:get vbus vol fail, vbus = %d\n", __func__, vbus_vol);
			hwlog_err("%s", tmp_buf);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			return -1;
		}
		adp_vol = get_adaptor_voltage();
		if (adp_vol < 0)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "%s: get adptor voltage fail, adp_vol = %d\n", __func__, adp_vol);
			hwlog_err("%s", tmp_buf);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			return -1;
		}
		iadapt = get_adaptor_current();
		if (iadapt < 0)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "%s: get adptor current fail, iadapt = %d\n", __func__, iadapt);
			hwlog_err("%s", tmp_buf);
			strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
			return -1;
		}
		delta_vol = adp_vol - vbus_vol;
		r = delta_vol* 1000 / ibus;
		snprintf(tmp_buf, sizeof(tmp_buf),
			"full_res[%d] = %d, vadapt =%d, iadapt = %d, vbus = %d, ibus = %d\n",
			i, r, adp_vol, iadapt, vbus_vol, ibus);
		strncat(dsm_buf, tmp_buf, strlen(tmp_buf));
		hwlog_info("%s", tmp_buf);
		sum += r;
	}
	r = sum / 3;
	di->full_path_resistance = r;
	hwlog_info("di->full_path_res_threshold = %d\n", di->full_path_res_threshold);
	if (r >= -di->full_path_res_threshold && r <= di->full_path_res_threshold)
	{
		if (0 == di->cc_cable_detect_ok)
		{
			direct_charge_send_quick_charge_uevent();
		}
		return 0;
	}
	hwlog_err("full path resistance = %d is out of[%d, %d]\n", r, -di->full_path_res_threshold, di->full_path_res_threshold);
	if (1 == di->cc_cable_detect_ok)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "full_res(%d) is out of[%d, %d]\n",
			r, -di->full_path_res_threshold, di->full_path_res_threshold);
		hwlog_err("%s", tmp_buf);
		strncat(dsm_buf, tmp_buf, strlen(tmp_buf));
		dsm_report(DSM_DIRECT_CHARGE_FULL_PATH_RESISTANCE, dsm_buf);
		strncat(di->dc_err_dsm_buff, dsm_buf, strlen(dsm_buf));
	}
	return -1;
}
int do_usb_port_leakage_current_check(void)
{
	int iadapt;
	int  leak_current;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };

	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	switch(di->adaptor_vendor_id)
	{
		case IWATT_ADAPTER:
			di->adaptor_iset = 400;
			set_adaptor_current();
			msleep(100);
			leak_current = di->scp_ops->scp_get_usb_port_leakage_current_info();
			if (leak_current)
			{
				snprintf(tmp_buf, sizeof(tmp_buf),"iwatt_adaptor usb port current leak, charger_vbus_vol = %d\n",
					get_charger_vbus_vol());
				goto FuncEnd;
			}
			return 0;
		default:
			iadapt = get_adaptor_current();
			hwlog_info("[%s]:iadapt = %d, charger_vbus_vol = %d, ls_ibus = %d\n", __func__, iadapt, get_charger_vbus_vol(), get_ls_ibus());
			if (iadapt < 0)
			{
				hwlog_err("get adptor current fail!\n");
				return -1;
			}
			if (iadapt > di->adaptor_leakage_current_th)
			{
				snprintf(tmp_buf, sizeof(tmp_buf), "%s: idapt(%d) > adaptor_leakage_current_th(%d),  charger_vbus_vol = %d\n",
					__func__, iadapt, di->adaptor_leakage_current_th, get_charger_vbus_vol());
				goto FuncEnd;
			}
			return 0;
	}

FuncEnd:
	hwlog_err("%s", tmp_buf);
	strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
	dsm_report(DSM_DIRECT_CHARGE_USB_PORT_LEAKAGE_CURRENT, tmp_buf);
	return -1;
}
int open_direct_charge_path(void)
{
	int bat_vol;
	int ls_ibus;
	int ret;
	int adjust_times = MAX_TIMES_FOR_SET_ADAPTER_VOL_20;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	int bat_capacity;

	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	bat_capacity = hisi_battery_capacity();
	bat_vol = get_bat_voltage();
	di->adaptor_vset = bat_vol * di->dc_volt_ratio + di->init_delt_vset;
	if (di->max_adaptor_vset < di->adaptor_vset)
	{
		di->adaptor_vset = di->max_adaptor_vset;
	}
	di->adaptor_iset = CURRENT_SET_FOR_RES_DETECT_1000_MA;
	set_adaptor_voltage();
	set_adaptor_current();
	msleep(50);
	ret = di->ls_ops->ls_enable(1);
	if (ret)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "%s: ls enable fail!\n", __func__);
		goto FuncEnd;
	}
	msleep(10);
	ls_ibus = get_ls_ibus();
	hwlog_info("%s:ls_ibus = %d!\n",__func__,ls_ibus);
	while (MIN_CURRENT_FOR_RES_DETECT_800_MA > ls_ibus)
	{
		bat_vol = get_bat_voltage();
		if (MAX_VOL_FOR_BATTERY_4360_MV < bat_vol)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "%s: adaptor_vset(%d) too high, ls_ibus = %d\n",
				__func__, bat_vol, ls_ibus);
			goto FuncEnd;
		}
		adjust_times--;
		hwlog_info("%s: adjust_times = %d!\n", __func__,adjust_times);
		if(0 == adjust_times)
		{
			snprintf(tmp_buf, sizeof(tmp_buf), "%s: try too many times, ls_ibus = %d\n",
				__func__, ls_ibus);
			goto FuncEnd;
		}
		di->adaptor_vset += di->vstep;
		hwlog_info("%s:adaptor_vset = %d!\n",__func__,di->adaptor_vset);
		set_adaptor_voltage();
		msleep(5);
		ls_ibus = get_ls_ibus();
		hwlog_info("%s:ls_ibus = %d!\n",__func__,ls_ibus);
	}
	return 0;

FuncEnd:
	hwlog_err("%s", tmp_buf);
	strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
	if(bat_capacity >= BATTERY_CAPACITY_HIGH_TH)
	{
		di->dc_open_retry_cnt += 1;
		hwlog_info("%s:open direct charge path fail, battery capacity is %d, over threshold! \n",__func__,bat_capacity);
	}
	return -1;
}
int scp_security_check(void)
{
	int ret;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;
	di->direct_charge_succ_flag = DIRECT_CHARGE_ERROR_ADAPTOR_VOLTAGE_ACCURACY;
	ret = do_adpator_voltage_accuracy_check();
	if (ret)
	{
		hwlog_err("adaptor voltage accuracy check fail!\n");
		return -1;
	}
	di->direct_charge_succ_flag = DIRECT_CHARGE_ERROR_USB_PORT_LEAKAGE_CURRENT;
	ret = do_usb_port_leakage_current_check();
	if (ret)
	{
		hwlog_err("usb port leakage current check fail!\n");
		return -1;
	}
	di->direct_charge_succ_flag = DIRECT_CHARGE_ERROR_OPEN_CHARGE_PATH;
	ret = open_direct_charge_path();
	if (ret)
	{
		hwlog_err("open direct charge path fail!\n");
		return -1;
	}
	di->direct_charge_succ_flag = DIRECT_CHARGE_ERROR_FULL_REISISTANCE;
	ret = do_full_path_resistance_check();
	if (ret)
	{
		hwlog_err("full path resiststance check fail!\n");
		return -1;
	}
	return	ret;
}


int is_support_scp(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;
	/*check	whether	support	scp detect*/
	if (di->scp_ops && di->scp_ops->is_support_scp)
	{
		/*return 0 means support scp*/
		if (di->scp_ops->is_support_scp())
		{
			hwlog_err("not support scp!\n");
			return 1;
		}
		scp_set_stage_status(SCP_STAGE_ADAPTER_DETECT);
		return 0;
	}
	return 1;
}
int scp_direct_charge_init(void)
{
	int ret;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;
	struct scp_init_data sid;
	sid.scp_mode_enable = 1;
	sid.vset_boundary = di->max_adaptor_vset;
	sid.iset_boundary = di->max_adaptor_iset;
	sid.init_adaptor_voltage = di->init_adapter_vset;
	sid.watchdog_timer = 3;
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };

	pd_dpm_notify_direct_charge_status(true);
	ret = di->scp_ops->scp_init(&sid);
	if (ret)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "%s: scp init fail!\n", __func__);
		goto FuncEnd;
	}
	ret = di->ls_ops->ls_init();
	if (ret)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "%s: ls init fail!\n", __func__);
		goto FuncEnd;
	}
	ret = di->bi_ops->init();
	if (ret)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "%s: bi init fail!\n", __func__);
		goto FuncEnd;
	}
	hwlog_info("direct charge init succ!\n");
	return SUCC;
FuncEnd:
	pd_dpm_notify_direct_charge_status(false);
	hwlog_err("%s", tmp_buf);
	strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
	return FAIL;
}

int set_direct_charger_disable_flags(int val, int type)
{
	int i;
	int disable = 0;
	struct direct_charge_device *di = NULL;

	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	if(!di) {
		hwlog_err("NULL direct_charge_device pointer found in %s.\n", __func__);
		return -1;
	}

	if(type < 0 || type >= __MAX_DISABLE_DIRECT_CHAGER){
		hwlog_err("set direct charger to %d with wrong type(%d) in %s.\n",
					val, type, __func__);
		return -1;
	}

	di->sysfs_disable_chatger[type] = val;
	for( i = 0; i < __MAX_DISABLE_DIRECT_CHAGER; i++){
		disable |= di->sysfs_disable_chatger[i];
	}
	di->sysfs_enable_charger = !disable;

	return 0;
}

void scp_stop_charging(void)
{
	int ret;
	int vbus_vol = 0;
	int vbat;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;

	di->scp_adaptor_detect_flag = SCP_ADAPTOR_NOT_DETECT;
	if (di->scp_stop_charging_flag_error)
	{
		di->error_cnt += 1;
	}
	if (di->scp_stop_charging_flag_error || di->scp_stop_charging_flag_info || (0 == di->sysfs_enable_charger) ||(0 == di->vbat_ovp_enable_charger))
	{
		scp_set_stage_status(SCP_STAGE_DEFAULT);
	}
	else
	{
		scp_set_stage_status(SCP_STAGE_CHARGE_DONE);
	}
	ret = di->ls_ops->ls_enable(0);
	if (ret)
	{
		hwlog_err("[%s]: ls enable fail!\n", __func__);
	}
	pd_dpm_notify_direct_charge_status(false);
	ret = di->scp_ops->scp_exit(di);
	if (ret)
	{
		hwlog_err("[%s]: scp exit fail!\n", __func__);
	}
	if (di->scp_work_on_charger)
	{
		scp_power_control(DISABLE);
		charge_set_hiz_enable(SET_HIZ_DISABLE);
	}
	if (di->ls_ops->ls_discharge)
	{
		ret = di->ls_ops->ls_discharge(1);
		if (ret)
		{
			hwlog_err("[%s]: ls discharge fail!\n", __func__);
		}
		else
		{
			hwlog_info("[%s]: ls discharge succ!\n", __func__);
		}
	}
	if (pmic_vbus_irq_is_enabled()) {
		direct_charger_disconnect_update_charger_type();
	}
	if (FAIL == restore_normal_charge())
	{
		hwlog_info("%s: restore normal charge fail!\n", __func__);
	}
	ret = di->ls_ops->ls_discharge(0);
	msleep(20);
	ret = di->bi_ops->get_vbus_voltage(&vbus_vol);
	if (ret)
	{
		hwlog_err("[%s]: get vbus vol fail!\n", __func__);
	}
	hwlog_info("%s: vbus_vol = %d!\n", __func__, vbus_vol);
	vbat = get_bat_voltage();
	hwlog_info("%s: vbat = %d!\n", __func__, vbat);

	if ((vbus_vol < VBUS_ON_THRESHOLD) || (vbat - vbus_vol) > VBAT_VBUS_DIFFERENCE)
	{
		hwlog_info("%s: vbat - vbus_vol = %d!\n", __func__, vbat - vbus_vol);
		if (!strstr(saved_command_line, "androidboot.swtype=factory")) {
			set_direct_charger_disable_flags(DIRECT_CHARGER_CLEAR_DISABLE_FLAGS,
                                             DIRECT_CHARGER_SYS_NODE);
		}
		di->error_cnt = 0;
		di->dc_open_retry_cnt = 0;
		hwlog_info("%s:direct charger disconnect!\n", __func__);
		di->full_path_resistance = ERROR_RESISTANCE;
		di->direct_charge_succ_flag = DIRECT_CHARGE_ERROR_ADAPTOR_DETECT;
		di->quick_charge_flag = 0;
		di->super_charge_flag = 0;
		direct_charger_disconnect_send_uevent();
		di->dc_err_report_flag = FALSE;
		memset(di->dc_err_dsm_buff, 0, sizeof(di->dc_err_dsm_buff));
#ifdef CONFIG_WIRELESS_CHARGER
		direct_charger_disconnect_event();
#endif
	}
	else
	{
		hwlog_info("%s:switch to normal charge!\n", __func__);
	}
	ret = di->ls_ops->ls_exit();
	if (ret)
	{
		hwlog_err("[%s]: ls exit fail!\n", __func__);
	}
	ret = di->bi_ops->exit();
	if (ret)
	{
		hwlog_err("[%s]: bi exit fail!\n", __func__);
	}
	ret = di->scp_ops->scp_chip_reset();
	if (ret)
	{
		hwlog_err("[%s]: scp_chip_reset fail!\n", __func__);
	}
	hrtimer_cancel(&di->threshold_caculation_timer);
	cancel_work_sync(&di->threshold_caculation_work);

	hrtimer_cancel(&di->kick_watchdog_timer);
	cancel_work_sync(&di->kick_watchdog_work);

	di->scp_stop_charging_flag_error = 0;
	di->scp_stop_charging_flag_info = 0;
	di->cur_stage = 0;
	di->pre_stage = 0;
	di->vbat = 0;
	di->ibat = 0;
	di->vadapt = 0;
	di->iadapt = 0;
	di->ls_vbus = 0;
	di->ls_ibus = 0;
	di->compensate_v = 0;
	di->ibat_abnormal_cnt = 0;
	di->max_adaptor_cur = 0;
	direct_charge_wake_unlock();
	di->cc_cable_detect_ok = 0;
	di->scp_stop_charging_complete_flag = 1;
}

/*lint -save -e* */

void direct_charge_parse_volt_para(struct device_node* np, struct direct_charge_device* di)
{
	int ret, i, array_len, idata;
	const char *volt_para_string = NULL;

	array_len = of_property_count_strings(np, "volt_para");
	di->stage_size = array_len / DC_PARA_TOTAL;
	hwlog_info("stage_size = %d\n", di->stage_size);
	if ((array_len <= 0) || (array_len % DC_PARA_TOTAL != 0)) {
		hwlog_err("volt_para is invaild,please check volt_para number!!\n");
		return;
	}
	if (array_len > DC_VOLT_LEVEL * DC_PARA_TOTAL) {
		array_len = DC_VOLT_LEVEL * DC_PARA_TOTAL;
		hwlog_err("volt_para is too long,use only front %d paras!!\n", array_len);
		return;
	}
	for (i = 0; i < array_len; i++) {
		ret = of_property_read_string_index(np, "volt_para", i, &volt_para_string);
		if (ret) {
			hwlog_err("get volt_para failed\n");
			return;
		}

		idata = simple_strtol(volt_para_string, NULL, STRTOL_MAX_LEN);
		switch (i % DC_PARA_TOTAL) {
		case DC_PARA_VOL_TH:
			if (idata < 0) {
				hwlog_err ("the volt_para vol_th is out of range!!\n");
				return;
			}
#ifdef CONFIG_SCHARGER_V300
			if (TRUE == is_hi6523_cv_limit())
				idata -= HI6523_CV_CUT;
#endif
			orig_volt_para[i / (DC_PARA_TOTAL)].vol_th = idata;
			break;
		case DC_PARA_CUR_TH_HIGH:
			if (idata < 0) {
				hwlog_err ("the volt_para cur_th_high is out of range!!\n");
				return;
			}
			orig_volt_para[i / (DC_PARA_TOTAL)].cur_th_high = idata;
			break;
		case DC_PARA_CUR_TH_LOW:
			if (idata < 0) {
				hwlog_err ("the volt_para cur_th_low is out of range!!\n");
				return;
			}
			orig_volt_para[i / (DC_PARA_TOTAL)].cur_th_low = idata;
			break;
		default:
			hwlog_err("get volt_para failed\n");
			return;
		}
	}

	for (i = 0; i < di->stage_size; i++) {
		di->volt_para[i].vol_th = orig_volt_para[i].vol_th;
		di->volt_para[i].cur_th_high = orig_volt_para[i].cur_th_high;
		di->volt_para[i].cur_th_low = orig_volt_para[i].cur_th_low;
		hwlog_info("orig_volt_para[%d], vol_th:%d, cur_th_high:%d, cur_th_low:%d\n", \
				i, di->volt_para[i].vol_th, di->volt_para[i].cur_th_high, di->volt_para[i].cur_th_low);
	}
}


int direct_charge_parse_dts(struct device_node* np, struct direct_charge_device* di)
{
	int ret = 0;
	int i = 0;
	int array_len = 0;
	int idata = 0;
	const char *chrg_data_string = NULL;

	ret = of_property_read_u32(np, "dc_volt_ratio", &(di->dc_volt_ratio));
	if (ret)
	{
		di->dc_volt_ratio = 1;
		hwlog_err("dc_volt_ratio failed ! \n");
	}
	hwlog_info("dc_volt_ratio = %d\n", di->dc_volt_ratio);

	ret = of_property_read_u32(np, "init_adapter_vset", &(di->init_adapter_vset));
	if (ret)
	{
		hwlog_err("init_adapter_vset failed\n");
		di->init_adapter_vset = 4400;
	}
	hwlog_info("init_adapter_vset = %d\n", di->init_adapter_vset);

	ret = of_property_read_u32(np, "init_delt_vset", &(di->init_delt_vset));
	if (ret)
	{
		hwlog_err("init_delt_vset failed  \n");
		di->init_delt_vset = 300;
	}
	hwlog_info("init_delt_vset = %d\n", di->init_delt_vset);

	ret = of_property_read_u32(np, "scp_work_on_charger", &(di->scp_work_on_charger));
	if (ret)
	{
		hwlog_err("scp_work_on_charger failed\n");
		return -EINVAL;
	}
	hwlog_info("scp_work_on_charger = %d\n", di->scp_work_on_charger);

	ret = of_property_read_u32(np, "standard_cable_full_path_res_max", &(di->standard_cable_full_path_res_max));
	if (ret)
	{
		hwlog_err("get standard_cable_full_path_res_max failed\n");
		return -EINVAL;
	}
	hwlog_info("standard_cable_full_path_res_max = %d\n", di->standard_cable_full_path_res_max);
	ret = of_property_read_u32(np, "max_current_for_none_standard_cable", &(di->max_current_for_none_standard_cable));
	if (ret)
	{
		hwlog_err("get max_current_for_none_standard_cable failed\n");
		return -EINVAL;
	}
	hwlog_info("max_current_for_none_standard_cable = %d\n", di->max_current_for_none_standard_cable);
	ret = of_property_read_u32(np, "use_5A", &(di->use_5A));
	if (ret)
	{
		hwlog_err("get use_5A failed\n");
		di->use_5A= 0;
	}
	hwlog_info("use_5A = %d\n", di->use_5A);
	ret = of_property_read_u32(np, "max_tadapt", &(di->max_tadapt));
	if (ret)
	{
		hwlog_err("get max_tadapt failed\n");
		return -EINVAL;
	}
	hwlog_info("max_tadapt = %d\n", di->max_tadapt);
	ret = of_property_read_u32(np, "max_tls", &(di->max_tls));
	if (ret)
	{
		hwlog_err("get max_tls failed\n");
		return -EINVAL;
	}
	hwlog_info("max_tls = %d\n", di->max_tls);
	ret = of_property_read_u32(np, "ibat_abnormal_th", &(di->ibat_abnormal_th));
	if (ret)
	{
		hwlog_err("get ibat_abnormal_th failed\n");
		return -EINVAL;
	}
	hwlog_info("ibat_abnormal_th = %d\n", di->ibat_abnormal_th);
	ret = of_property_read_u32(np, "first_cc_stage_timer_in_min", &(di->first_cc_stage_timer_in_min));
	if (ret)
	{
		hwlog_err("get first_cc_stage_timer_in_min failed\n");
		return -EINVAL;
	}
	hwlog_info("first_cc_stage_timer_in_min = %d\n", di->first_cc_stage_timer_in_min);
	ret = of_property_read_u32(np, "vol_err_th", &(di->vol_err_th));
	if (ret)
	{
		hwlog_err("get vol_err_th failed\n");
		return -EINVAL;
	}
	hwlog_info("vol_err_th = %d\n", di->vol_err_th);
	ret = of_property_read_u32(np, "full_path_res_max", &(di->full_path_res_max));
	if (ret)
	{
		hwlog_err("get full_path_res_max failed\n");
		return -EINVAL;
	}
	hwlog_info("full_path_res_max = %d\n", di->full_path_res_max);
	ret = of_property_read_u32(np, "adaptor_leakage_current_th", &(di->adaptor_leakage_current_th));
	if (ret)
	{
		hwlog_err("get adaptor_leakage_current_th failed\n");
		return -EINVAL;
	}
	hwlog_info("adaptor_leakage_current_th = %d\n", di->adaptor_leakage_current_th);
	ret = of_property_read_u32(np, "compensate_r", &(di->compensate_r));
	if (ret)
	{
		hwlog_err("get compensate_r failed\n");
		return -EINVAL;
	}
	hwlog_info("compensate_r = %d\n", di->compensate_r);
	ret = of_property_read_u32(np, "max_dc_bat_vol", &(di->max_dc_bat_vol));
	if (ret)
	{
		hwlog_err("get max_dc_bat_vol failed\n");
		return -EINVAL;
	}
	#ifdef CONFIG_SCHARGER_V300
	if (TRUE == is_hi6523_cv_limit())
		di->max_dc_bat_vol -= HI6523_CV_CUT;
	#endif
	hwlog_info("max_dc_bat_vol = %d\n", di->max_dc_bat_vol);
	ret = of_property_read_u32(np, "min_dc_bat_vol", &(di->min_dc_bat_vol));
	if (ret)
	{
		hwlog_err("get min_dc_bat_vol failed\n");
		return -EINVAL;
	}
	hwlog_info("min_dc_bat_vol = %d\n", di->min_dc_bat_vol);
	ret = of_property_read_u32(np, "max_adaptor_vset", &(di->max_adaptor_vset));
	if (ret)
	{
		hwlog_err("get max_adaptor_vset failed\n");
		return -EINVAL;
	}
	hwlog_info("max_adaptor_vset = %d\n", di->max_adaptor_vset);
	ret = of_property_read_u32(np, "charge_control_interval", &(di->charge_control_interval));
	if (ret)
	{
		hwlog_err("get charge_control_interval failed\n");
		return -EINVAL;
	}
	hwlog_info("charge_control_interval = %d\n", di->charge_control_interval);
	ret = of_property_read_u32(np, "threshold_caculation_interval", &(di->threshold_caculation_interval));
	if (ret)
	{
		hwlog_err("get threshold_caculation_interval failed\n");
		return -EINVAL;
	}
	hwlog_info("threshold_caculation_interval = %d\n", di->threshold_caculation_interval);
	ret = of_property_read_u32(np, "vstep", &(di->vstep));
	if (ret)
	{
		hwlog_err("get vstep failed\n");
		return -EINVAL;
	}
	hwlog_info("vstep = %d\n", di->vstep);
	ret = of_property_read_u32(np, "delta_err", &(di->delta_err));
	if (ret)
	{
		hwlog_err("get delta_err failed\n");
		return -EINVAL;
	}
	hwlog_info("delta_err = %d\n", di->delta_err);
	ret = of_property_read_u32(np, "cc_cable_detect_enable", &(di->cc_cable_detect_enable));
	if (ret)
	{
		hwlog_err("get cc_cable_detect_enable failed\n");
	}
	hwlog_info("cc_cable_detect_enable = %d\n", di->cc_cable_detect_enable);
	direct_charge_parse_volt_para(np, di);
	array_len = of_property_count_strings(np, "temp_para");
	if ((array_len <= 0) || (array_len % DC_TEMP_TOTAL != 0))
	{
		hwlog_err("temp_para is invaild,please check temp_para number!!\n");
		return -EINVAL;
	}
	if (array_len > DC_TEMP_LEVEL * DC_TEMP_TOTAL)
	{
		array_len = DC_TEMP_LEVEL * DC_TEMP_TOTAL;
		hwlog_err("temp_para is too long,use only front %d paras!!\n", array_len);
		return -EINVAL;
	}
	for (i = 0; i < array_len; i++)
	{
		ret = of_property_read_string_index(np, "temp_para", i, &chrg_data_string);
		if (ret)
		{
			hwlog_err("get temp_para failed\n");
			return -EINVAL;
		}
		idata = simple_strtol(chrg_data_string, NULL, 10);
		switch (i % DC_TEMP_TOTAL) {
		case DC_TEMP_MIN:
			di->temp_para[i / (DC_TEMP_TOTAL)].temp_min = idata;
			break;
		case DC_TEMP_MAX:
			di->temp_para[i / (DC_TEMP_TOTAL)].temp_max = idata;
			break;
		case DC_CUR_MAX:
			di->temp_para[i / (DC_TEMP_TOTAL)].cur_max = idata;
			break;
		default:
			hwlog_err("get temp_para failed\n");
			return -EINVAL;
		}
		hwlog_info("di->temp_para[%d][%d] = %d\n", i / (DC_TEMP_TOTAL), i % (DC_TEMP_TOTAL), idata);
	}
	for (i = 0; i < 2*DC_VOLT_LEVEL; ++i)
	{
		di->stage_need_to_jump[i] = -1;
	}
	array_len = of_property_count_strings(np, "stage_need_to_jump");
	if ((array_len <= 0) || (array_len > 2*di->stage_size))
	{
		hwlog_err("invalid stage need to jump!!\n");
		return -EINVAL;
	}
	for (i = 0; i < array_len; i++)
	{
		ret = of_property_read_string_index(np, "stage_need_to_jump", i,&chrg_data_string);
		if (ret)
		{
			hwlog_err("get stage_need_to_jump error\n");
			return -EINVAL;
		}
		idata = simple_strtol(chrg_data_string, NULL, 10);
		if (idata < -1 || idata > 2*di->stage_size)
		{
			hwlog_err("stage_need_to_jump:[%d] is out of range!!\n", idata);
			return -EINVAL;
		}
		hwlog_info("stage_need_to_jump[%d] = %d\n", i, idata);
		di->stage_need_to_jump[i] = idata;
	}
	return ret;
}

/*lint -restore*/

int jump_stage_if_need(int cur_stage)
{
	int i;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	for (i = 0; i < 2*di->stage_size; ++i)
	{
		if (-1 == di->stage_need_to_jump[i])
		{
			return cur_stage;
		}
		else
		{
			if (cur_stage == di->stage_need_to_jump[i])
			{
				hwlog_info("jump stage %d\n", cur_stage);
				return jump_stage_if_need(cur_stage + 1);
			}
		}
	}
	return cur_stage;
}

/*lint -save -e* */
void select_direct_charge_stage(void)
{
	int i;
	int vbat_th;
	int cur_stage = 0;
	int stage_size = 0;
	int vbat = get_bat_voltage();
	int ibat = get_bat_current();
	int iadaptor = get_adaptor_current();
	char tmp_buf[ERR_NO_STRING_SIZE] = { 0 };
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;
	stage_size = di->stage_size;

	di->vbat = vbat;
	di->ibat = ibat;
	if (iadaptor < di->ibat_abnormal_th)
	{
		di->ibat_abnormal_cnt++;
	}
	else
	{
		di->ibat_abnormal_cnt = 0;
	}
	if (di->ibat_abnormal_cnt > 10)
	{
		di->scp_stop_charging_flag_error = 1;
		snprintf(tmp_buf, sizeof(tmp_buf),"%s: ibat abnormal, stop direct charge\n", __func__);
		hwlog_err("%s", tmp_buf);
		strncat(di->dc_err_dsm_buff, tmp_buf, strlen(tmp_buf));
		return;
	}
	di->pre_stage = di->cur_stage;
	for (i = stage_size - 1; i >=0; --i)
	{
		vbat_th = di->volt_para[i].vol_th + di->compensate_v;
		vbat_th = vbat_th > di->volt_para[stage_size - 1].vol_th ? di->volt_para[stage_size - 1].vol_th : vbat_th;
		if (vbat >= vbat_th && ibat <= di->volt_para[i].cur_th_low)
		{
			cur_stage = 2*i +2;
			break;
		}
		else if (vbat >= vbat_th)
		{
			cur_stage = 2*i +1;
			break;
		}
	}
	if (i < 0)
	{
		cur_stage = 0;
	}
	if (cur_stage < di->pre_stage)
	{
		cur_stage = di->pre_stage;
	}
	if (di->first_cc_stage_timer_in_min)
	{
		if (0 == cur_stage)
		{
			if (time_after(jiffies, di->first_cc_stage_timeout))
			{
				hwlog_info("first_cc_stage_timeout in %d min, stage++\n",di->first_cc_stage_timer_in_min);
				cur_stage += 1;
			}
		}
	}
	if (cur_stage != di->cur_stage)
	{
		di->cur_stage = jump_stage_if_need(cur_stage);
	}
}
/*lint -restore*/

int battery_temp_handler(int temp)
{
	int i;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	for (i = 0; i < DC_TEMP_LEVEL; ++i)
	{
		if  (temp >= di->temp_para[i].temp_min && temp < di->temp_para[i].temp_max)
		{
			return di->temp_para[i].cur_max;
		}
	}
	hwlog_err("error temp = %d\n",temp);
	return 0;
}

void select_direct_charge_param(void)
{
	int cur_th_high;
	int max_adaptor_cur;
	int vbat_th;
	int bat_temp_cur_max;
	int bat_temp = hisi_battery_temperature();

	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}
	di = g_di;

	direct_charge_double_56k_cable_detect();
	bat_temp_cur_max = battery_temp_handler(bat_temp);
	if (0 == bat_temp_cur_max)
	{
		hwlog_info("%s : temp = %d, can not do direct charging \n", __func__, bat_temp);
		di->scp_stop_charging_flag_info = 1;
	}
	max_adaptor_cur = get_adaptor_max_current();
	if (max_adaptor_cur < 0)
		return;
	if(di->use_5A)
	{
		if (max_adaptor_cur == 4500)
		{
			max_adaptor_cur = di->volt_para[0].cur_th_high;
		}
	}
	if (max_adaptor_cur != di->max_adaptor_cur)
	{
		hwlog_info("%s : max_adaptor_cur = %d\n", __func__, max_adaptor_cur);
		di->max_adaptor_cur = max_adaptor_cur;
	}
	di->compensate_v = di->ibat*di->compensate_r/1000;
	vbat_th = di->volt_para[di->cur_stage/2].vol_th + di->compensate_v;
	di->cur_vbat_th = vbat_th < di->volt_para[di->stage_size - 1].vol_th ? vbat_th: di->volt_para[di->stage_size - 1].vol_th;

	cur_th_high = di->volt_para[di->cur_stage/2].cur_th_high;
	if (di->cc_cable_detect_enable) {
		if (0 == di->cc_cable_detect_ok) {
			cur_th_high = di->volt_para[di->cur_stage/2].cur_th_high > di->max_current_for_none_standard_cable ? di->max_current_for_none_standard_cable : di->volt_para[di->cur_stage/2].cur_th_high;
		} else {
			cur_th_high = di->volt_para[di->cur_stage/2].cur_th_high;
		}
		hwlog_info("[%s]:cc_check_result = %d ,after_cc_check_cur_th_high = %d\n", __func__,di->cc_cable_detect_ok,cur_th_high);
	}
	cur_th_high = cur_th_high >  bat_temp_cur_max ? bat_temp_cur_max : cur_th_high;
	cur_th_high = cur_th_high >  di->dc_volt_ratio * max_adaptor_cur ? di->dc_volt_ratio * max_adaptor_cur : cur_th_high;
	di->cur_ibat_th_high = cur_th_high > di->sysfs_iin_thermal ? di->sysfs_iin_thermal: cur_th_high;
	di->cur_ibat_th_low = di->volt_para[di->cur_stage/2].cur_th_low;
}
void battery_aging_safe_policy(struct direct_charge_device *di)
{
	int ret, i, cur_level;
	static int last_level = BASP_PARA_LEVEL;
	AGING_SAFE_POLICY_TYPE basp = {0};

	if (NULL == di)
	{
		hwlog_err("%s direct_charge_device is NULL\n", __func__);
		return ;
	}

	ret = hisi_battery_aging_safe_policy(&basp);
	if (ret) {
		hwlog_err(BASP_TAG"[%s] get basp policy fail, ret:%d!\n", __func__, ret);
		return;
	}

	cur_level = basp.level;

	if (cur_level != last_level) {
		di->volt_para[di->stage_size -1].vol_th =
			orig_volt_para[di->stage_size -1].vol_th - basp.dc_volt_dec;
		for (i = 0; i < di->stage_size -1; i++) {
			di->volt_para[i].vol_th = orig_volt_para[i].vol_th < di->volt_para[di->stage_size -1].vol_th\
								? orig_volt_para[i].vol_th : di->volt_para[di->stage_size -1].vol_th;
		}
		switch (basp.cur_ratio_policy) {
		case BASP_RATIO_POLICY_ALL:
			di->volt_para[di->stage_size -1].cur_th_high =
						orig_volt_para[di->stage_size -1].cur_th_high * basp.cur_ratio/BASP_PARA_SCALE;
			di->volt_para[di->stage_size -1].cur_th_low =
						orig_volt_para[di->stage_size -1].cur_th_low;
			for (i = 0; i < di->stage_size -1; i++) {
				di->volt_para[i].cur_th_high = orig_volt_para[i].cur_th_high * basp.cur_ratio /BASP_PARA_SCALE;
				di->volt_para[i].cur_th_low = orig_volt_para[i].cur_th_low * basp.cur_ratio /BASP_PARA_SCALE;
			}
			break;
		case BASP_RATIO_POLICY_MAX:
			di->volt_para[0].cur_th_high = orig_volt_para[0].cur_th_high * basp.cur_ratio /BASP_PARA_SCALE;
			di->volt_para[0].cur_th_low = orig_volt_para[0].cur_th_low;
			for (i = 1; i < di->stage_size;i++) {
				di->volt_para[i].cur_th_high = orig_volt_para[i].cur_th_high <= di->volt_para[0].cur_th_high ?
							orig_volt_para[i].cur_th_high : di->volt_para[0].cur_th_high;
				di->volt_para[i].cur_th_low = orig_volt_para[i].cur_th_low;
			}
			break;
		default:
			break;
		}
		last_level = cur_level;
		hwlog_info(BASP_TAG"cur_level = %d\n", cur_level);
		for (i = 0; i < di->stage_size; i++) {
			hwlog_info(BASP_TAG"volt_para[%d], vol_th:%d, cur_th_high:%d, cur_th_low:%d\n",
				i, di->volt_para[i].vol_th, di->volt_para[i].cur_th_high, di->volt_para[i].cur_th_low);
		}
	}
	return;
}
void direct_charge_regulation(void)
{
	char buf[ERR_NO_STRING_SIZE] = { 0 };
	int ret;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;
	int vbat = di->vbat;
	int ibat = di->ibat;
	int vbat_sh = di->cur_vbat_th;
	int ibat_sh_high = di->cur_ibat_th_high;
	int ibat_sh_low = di->cur_ibat_th_low;
	int iadapt = get_adaptor_current();
	int iadapt_set = get_adaptor_current_set();

	di->vadapt = get_adaptor_voltage();
	di->tadapt = get_adaptor_temp();
	di->iadapt = iadapt;
	di->ls_ibus = get_ls_ibus();
	di->ls_vbus = get_ls_vbus();
	di->tls = get_ls_temp();

	hwlog_info("cur_stage = %d vbat = %d ibat = %d vbat_sh = %d ibat_sh_high = %d ibat_sh_low = %d vadp = %d iadap = %d ls_vbus = %d ls_ibus = %d iadapt_set = %d tadapt = %d tls = %d!\n",
			di->cur_stage, vbat, ibat, vbat_sh, ibat_sh_high, ibat_sh_low, di->vadapt, iadapt, di->ls_vbus, di->ls_ibus, iadapt_set, di->tadapt, di->tls);
	if (di->tls > (int)(di->max_tls))
	{
		snprintf(buf, sizeof(buf), "%s: tls = %d > %d, stop direct_charge!\n", __func__, di->tls, di->max_tls);
		hwlog_err("%s", buf);
		strncat(di->dc_err_dsm_buff, buf, strlen(buf));
		di->scp_stop_charging_flag_error = 1;
		return;
	}
	if (di->tadapt > (int)(di->max_tadapt))
	{
		snprintf(buf, sizeof(buf), "%s: tadapt(%d) > [%d], stop direct_charge!\n", __func__, di->tadapt, di->max_tadapt);
		hwlog_err("%s", buf);
		strncat(di->dc_err_dsm_buff, buf, strlen(buf));
		dsm_report(DSM_DIRECT_CHARGE_ADAPTER_OTP, buf);
		di->scp_stop_charging_flag_error = 1;
		return;
	}
	ret = is_ls_close();/*keep communication with loadswitchwithin 1 second*/
	if (ret)
	{
		hwlog_err("ls is close!\n");
		return;
	}
	if (di->cur_stage % 2)
	{
		if (vbat > vbat_sh)
		{
			di->adaptor_vset += di->dc_volt_ratio * (vbat_sh - vbat);
			set_adaptor_voltage();
			return;
		}
		if (iadapt > ibat_sh_high/di->dc_volt_ratio)
		{
			di->adaptor_vset -= di->vstep;
			set_adaptor_voltage();
			return;
		}
		if (ibat > ibat_sh_high)
		{
			di->adaptor_vset -= di->vstep;
			set_adaptor_voltage();
			return;
		}
		if (ibat > ibat_sh_high - di->delta_err)
		{
			hwlog_info("do nothing!\n");
			return;
		}
		if (di->adaptor_iset < (ibat_sh_high - 1000)/di->dc_volt_ratio)
		{
			di->adaptor_iset += 1000/di->dc_volt_ratio;
			set_adaptor_current();
			return;
		}
		else if (di->adaptor_iset < ibat_sh_high/di->dc_volt_ratio)
		{
			di->adaptor_iset = ibat_sh_high;
			set_adaptor_current();
			return;
		}
		else
		{
			if (iadapt < (ibat_sh_high - di->delta_err)/di->dc_volt_ratio)
			{
				di->adaptor_vset += di->vstep;
				set_adaptor_voltage();
				return;
			}
		}
	}
	else
	{
		if (iadapt > ibat_sh_high/di->dc_volt_ratio)
		{
			di->adaptor_vset -= di->vstep;
			set_adaptor_voltage();
			return;
		}
		if (ibat > ibat_sh_high)
		{
			di->adaptor_vset -= di->vstep;
			set_adaptor_voltage();
			return;
		}
		if (ibat > ibat_sh_high - di->delta_err)
		{
			hwlog_info("do nothing!\n");
			return;
		}
		if (di->adaptor_iset < (ibat_sh_high - 1000)/di->dc_volt_ratio)
		{
			di->adaptor_iset += 1000/di->dc_volt_ratio;
			set_adaptor_current();
			return;
		}
		else if (di->adaptor_iset < ibat_sh_high/di->dc_volt_ratio)
		{
			di->adaptor_iset = ibat_sh_high/di->dc_volt_ratio;
			set_adaptor_current();
			return;
		}
		else
		{
			if (iadapt < (ibat_sh_high - di->delta_err)/di->dc_volt_ratio)
			{
				di->adaptor_vset += di->vstep;
				set_adaptor_voltage();
				return;
			}
		}

	}
}

/*lint -save -e* */
void charge_control_work(struct work_struct *work)
{
	struct direct_charge_device *di = container_of(work,struct direct_charge_device, charge_control_work);
	int interval = di->charge_control_interval;

	if (di->scp_stop_charging_flag_error || di->scp_stop_charging_flag_info || (0 == di->sysfs_enable_charger) ||(0 == di->vbat_ovp_enable_charger))
	{
		hwlog_info("direct charge stop!\n");
		scp_stop_charging();
		return;
	}
	if (2*di->stage_size == di->cur_stage)
	{
		hwlog_info("cur_stage = %d vbat = %d ibat = %d\n", di->cur_stage, di->vbat, di->ibat);
		hwlog_info("direct charge done!\n");
		scp_stop_charging();
		return;
	}
	direct_charge_regulation();

	hrtimer_start(&di->charge_control_timer, ktime_set(interval/MSEC_PER_SEC, (interval % MSEC_PER_SEC) * USEC_PER_SEC), HRTIMER_MODE_REL);
}
/*lint -restore*/

/*lint -save -e* */
void threshold_caculation_work(struct work_struct *work)
{
	struct direct_charge_device *di = container_of(work,struct direct_charge_device, threshold_caculation_work);
	int interval = di->threshold_caculation_interval;

	if (di->scp_stop_charging_flag_error || di->scp_stop_charging_flag_info || (0 == di->sysfs_enable_charger) ||(0 == di->vbat_ovp_enable_charger))
	{
		hwlog_info("direct charge stop, stop threshold_caculation!\n");
		return;
	}
	battery_aging_safe_policy(di);
	select_direct_charge_stage();
	select_direct_charge_param();
	if (2*di->stage_size == di->cur_stage)
	{
		hwlog_info("direct charge done, stop threshold_caculation!\n");
		return;
	}
	hrtimer_start(&di->threshold_caculation_timer, ktime_set(interval/MSEC_PER_SEC, (interval % MSEC_PER_SEC) * USEC_PER_SEC), HRTIMER_MODE_REL);
}
/*lint -restore*/

void kick_watchdog_work(struct work_struct *work)
{
	struct direct_charge_device *di = container_of(work,struct direct_charge_device, kick_watchdog_work);
	int interval = KICK_WATCHDOG_TIME;//kich watchdog timer;
	int ret;
	int bat_curr;

	if (di->scp_stop_charging_flag_error || di->scp_stop_charging_flag_info || (0 == di->sysfs_enable_charger) ||(0 == di->vbat_ovp_enable_charger))
	{
		hwlog_info("direct charge stop, stop kick_watchdog!\n");
		return;
	}
	if (DOUBLE_SIZE * di->stage_size == di->cur_stage)
	{
		hwlog_info("direct charge done, stop kick_watchdog!\n");
		return;
	}

	ret = di->bi_ops->get_bat_current(&bat_curr);/*keep communication with the adaptor within 1 second*/
	if (ret)
	{
		hwlog_err("get_bat_current fail!\n");
	}
	hwlog_info("To keep communication with ls : bat_curr = %d!\n", bat_curr);
	hrtimer_start(&di->kick_watchdog_timer, ktime_set(interval/MSEC_PER_SEC, (interval % MSEC_PER_SEC) * USEC_PER_SEC), HRTIMER_MODE_REL);
}

/*lint -save -e* */
enum hrtimer_restart threshold_caculation_timer_func(struct hrtimer *timer)
{
	struct direct_charge_device *di;

	di = container_of(timer, struct direct_charge_device, threshold_caculation_timer);
	queue_work(di->direct_charge_wq, &di->threshold_caculation_work);
	return HRTIMER_NORESTART;
}

enum hrtimer_restart charge_control_timer_func(struct hrtimer *timer)
{
	struct direct_charge_device *di;

	di = container_of(timer, struct direct_charge_device, charge_control_timer);
	queue_work(di->direct_charge_wq, &di->charge_control_work);
	return HRTIMER_NORESTART;
}
/*lint -restore*/

enum hrtimer_restart kick_watchdog_timer_func(struct hrtimer *timer)
{
	struct direct_charge_device *di;

	di = container_of(timer, struct direct_charge_device, kick_watchdog_timer);
	queue_work(di->direct_charge_watchdog_wq, &di->kick_watchdog_work);
	return HRTIMER_NORESTART;
}

int direct_charge_fault_notifier_call(struct notifier_block *fault_nb, unsigned long event, void *data)
{
	struct direct_charge_device *di = container_of(fault_nb, struct direct_charge_device, fault_nb);
	enum scp_stage_type stage = scp_get_stage_status();

	if (stage < SCP_STAGE_SECURITY_CHECK || stage == SCP_STAGE_CHARGE_DONE)
	{
		hwlog_err("notify event:%d when direct charge not initialized, so ignored!\n", di->charge_fault);
		return NOTIFY_OK;
	}
	di->charge_fault = (enum charge_fault_type)event;
	di->fault_data = (struct nty_data*)data;
	schedule_work(&di->fault_work);
	return NOTIFY_OK;
}
/*lint -restore*/

void scp_start_charging(void)
{
	int interval;

	hwlog_info("%s \n",__func__);
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;
	direct_charge_wake_lock();
	if (di->first_cc_stage_timer_in_min)
	{
		/*8A maximum 5 min, ap will not suspend in direct charge mode, so use jiffies */
		hwlog_info("%s start timing\n",__func__);
		di->first_cc_stage_timeout = jiffies + msecs_to_jiffies(di->first_cc_stage_timer_in_min*60*MSEC_PER_SEC);
	}
	select_direct_charge_stage();
	chg_set_adaptor_test_result(di->adaptor_test_result_type,PROTOCOL_FINISH_SUCC);
	scp_set_stage_status(SCP_STAGE_CHARGING);
	select_direct_charge_param();
	interval = di->charge_control_interval;
	hrtimer_start(&di->charge_control_timer, ktime_set(interval/MSEC_PER_SEC, (interval % MSEC_PER_SEC) * USEC_PER_SEC), HRTIMER_MODE_REL);
	interval = di->threshold_caculation_interval;
	hrtimer_start(&di->threshold_caculation_timer, ktime_set(interval/MSEC_PER_SEC, (interval % MSEC_PER_SEC) * USEC_PER_SEC), HRTIMER_MODE_REL);
	interval = KICK_WATCHDOG_TIME;
	hrtimer_start(&di->kick_watchdog_timer, ktime_set(interval/MSEC_PER_SEC, (interval % MSEC_PER_SEC) * USEC_PER_SEC), HRTIMER_MODE_REL);
}
/*lint -save -e* */
void vbat_ovp_exit_direct_charge(int enable_charge)
{
	int i = 0;
	int ibus = 0;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;

	di->vbat_ovp_enable_charger = enable_charge;
	hwlog_info("%s: vbat_ovp_enable_charger = %d \n",__func__,di->vbat_ovp_enable_charger);
}

int vbat_ovp_scp_exit(void)
{
	int ret;
	int val;
	int vbus_vol = 0;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}
	di = g_di;
	val = di->ls_ops->ls_enable(0);
	if (val)
	{
		hwlog_err("[%s]: ls enable fail!\n", __func__);
	}
	if (di->ls_ops->ls_discharge)
	{
		val = di->ls_ops->ls_discharge(1);
		if (val)
		{
			hwlog_err("[%s]: ls discharge fail!\n", __func__);
		}
		else
		{
			hwlog_info("[%s]: ls discharge succ!\n", __func__);
		}
	}
	msleep(200);
	val = di->ls_ops->ls_discharge(0);
	msleep(20);
	val = di->bi_ops->get_vbus_voltage(&vbus_vol);
	if (val)
	{
		hwlog_err("[%s]: get vbus vol fail!\n", __func__);
	}
	hwlog_info("%s: vbus_vol = %d!\n", __func__, vbus_vol);
	if (vbus_vol < 3000)
	{
		hwlog_info("%s:direct charger disconnect!\n", __func__);
		ret = 0;
	}
	else
	{
		hwlog_info("%s:comunication fail!\n", __func__);
		ret = -1;
	}
	val = di->scp_ops->scp_exit(di);
	if (val)
	{
		hwlog_err("[%s]: scp exit fail!\n", __func__);
	}
	if (di->scp_work_on_charger)
	{
		scp_power_control(DISABLE);
		charge_set_hiz_enable(SET_HIZ_ENABLE);
	}
	val = di->ls_ops->ls_exit();
	if (val)
	{
		hwlog_err("[%s]: ls exit fail!\n", __func__);
	}
	val = di->bi_ops->exit();
	if (val)
	{
		hwlog_err("[%s]: bi exit fail!\n", __func__);
	}
	return ret;
}
int vbat_ovp_scp_handle(void)
{
	int timeout;
	int ret;
	int adaptor_vol;
	int iadaptor;
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return -1;
	}
	di = g_di;

	if (0 != is_direct_charge_ops_valid(di))
	{
		hwlog_err("%s:bad ops \n", __func__);
		return -1;
	}
	di->adaptor_vendor_id = di->scp_ops->scp_get_adapter_vendor_id();
	di->adaptor_vset = 4300;
	di->adaptor_iset = 2000;
	ret = scp_direct_charge_init();
	if (ret)
	{
		hwlog_err("%s:scp_direct_charge_init fail\n", __func__);
		ret = vbat_ovp_scp_exit();
		return ret;
	}
	ret = di->ls_ops->watchdog_config_ms(0);
	if (ret)
	{
		hwlog_err("%s:watchdog_config_ms fail\n", __func__);
		ret = vbat_ovp_scp_exit();
		return ret;
	}

	adaptor_vol = di->scp_ops->scp_get_adaptor_voltage();
	if (adaptor_vol < 0)
	{
		hwlog_err("%s:scp_get_adaptor_voltage fail\n", __func__);
		ret = vbat_ovp_scp_exit();
		return ret;
	}
	iadaptor= get_adaptor_current_for_vbat_ovp();
	if (iadaptor < 0)
	{
		hwlog_err("%s:scp_get_iadaptor fail\n", __func__);
		ret = vbat_ovp_scp_exit();
		return ret;
	}
	hwlog_info("%s:adaptor_vol1 = %d, adaptor_current1 = %d\n",__func__, adaptor_vol,iadaptor);
	ret = di->scp_ops->scp_set_adaptor_voltage(di->adaptor_vset);
	if (ret)
	{
		hwlog_err("%s:scp_set_adaptor_voltage fail\n", __func__);
		ret = vbat_ovp_scp_exit();
		return ret;
	}
	ret = di->scp_ops->scp_set_adaptor_current(di->adaptor_iset);
	if (ret)
	{
		hwlog_err("%s:scp_set_adaptor_current\n", __func__);
		ret = vbat_ovp_scp_exit();
		return ret;
	}
	ret = di->ls_ops->ls_enable(1);
	if (ret)
	{
		hwlog_err("%s:ls_enable fail\n", __func__);
		ret = vbat_ovp_scp_exit();
		return ret;
	}
	hwlog_info("%s:set_adaptor_vol = %d, set_adaptor_current = %d\n",__func__, di->adaptor_vset,di->adaptor_iset);
	while(1)
	{
		di->adaptor_vset = 4300;
		di->adaptor_iset = 2000;
		ret = di->scp_ops->scp_set_adaptor_voltage(di->adaptor_vset);
		if (ret)
		{
			hwlog_err("%s:scp_set_adaptor_voltage fail\n", __func__);
			ret = vbat_ovp_scp_exit();
			return ret;
		}
		ret = di->scp_ops->scp_set_adaptor_current(di->adaptor_iset);
		if (ret)
		{
			hwlog_err("%s:scp_set_adaptor_current fail\n", __func__);
			ret = vbat_ovp_scp_exit();
			return ret;
		}
		adaptor_vol = di->scp_ops->scp_get_adaptor_voltage();
		if (adaptor_vol < 0)
		{
			hwlog_err("%s:scp_get_adaptor_voltage fail\n", __func__);
			ret = vbat_ovp_scp_exit();
			return ret;
		}
		iadaptor= get_adaptor_current_for_vbat_ovp();
		if (iadaptor < 0)
		{
			hwlog_err("%s:scp_get_iadaptor fail\n", __func__);
			ret = vbat_ovp_scp_exit();
			return ret;
		}
		hwlog_info("%s:adaptor_vol = %d, adaptor_current = %d\n",__func__, adaptor_vol,iadaptor);
		msleep(500);
	}
}
int is_direct_charge_failed(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return 0;
	}
	di = g_di;
	if (di->error_cnt >= DC_ERR_CNT_MAX)
	{
		return 1;
	}
	return 0;
}

int is_direct_charge_ops_valid(struct direct_charge_device *di)
{
	if (NULL == di)
	{
		hwlog_err("direct_charge_device is null!\n");
		return	INVALID;
	}

	if ((NULL == di->scp_ops) || (NULL == di->scp_ops->is_support_scp)
		||(NULL	== di->scp_ops->scp_init) || (NULL == di->scp_ops->scp_adaptor_detect)
		||(NULL	== di->scp_ops->scp_set_adaptor_voltage) || (NULL == di->scp_ops->scp_get_adaptor_voltage)
		||(NULL	== di->scp_ops->scp_get_adaptor_current) || (NULL == di->scp_ops->scp_set_adaptor_current)
		||(NULL	== di->scp_ops->scp_adaptor_reset) || (NULL == di->scp_ops->scp_chip_reset)
		||(NULL	== di->scp_ops->scp_stop_charge_config)	|| (NULL == di->scp_ops->scp_get_adaptor_status)
		||(NULL	== di->scp_ops->scp_get_chip_status) || (NULL == di->scp_ops->scp_exit)
		||(NULL == di->scp_ops->scp_get_adaptor_max_current) || (NULL == di->scp_ops->scp_cable_detect)
		||(NULL == di->scp_ops->scp_get_adapter_vendor_id ) )
	{
		hwlog_err("scp ops is null!\n");
		return	INVALID;
	}

	if ((NULL == di->scp_ps_ops) || (NULL == di->scp_ps_ops->scp_power_enable))
	{
		hwlog_err("scp ps ops is null!\n");
		return	INVALID;
	}

	if ((NULL == di->ls_ops) || (NULL == di->ls_ops->ls_init) || (NULL == di->ls_ops->ls_exit)
		|| (NULL == di->ls_ops->is_ls_close) || (NULL == di->ls_ops->get_ls_id) || (NULL == di->ls_ops->ls_enable)
		||(NULL == di->ls_ops->watchdog_config_ms))
	{
		hwlog_err("ls ops is null!\n");
		return	INVALID;
	}

	if ((NULL == di->bi_ops) || (NULL == di->bi_ops->get_bat_current) || (NULL == di->bi_ops->exit)
		|| (NULL == di->bi_ops->get_bat_btb_voltage) || (NULL == di->bi_ops->get_vbus_voltage))
	{
		hwlog_err("bi ops is null!\n");
		return	INVALID;
	}
	return	VALID;
}

static int direct_charge_mode = 0;

void direct_charge_or_set_local_mode(int dc_mode)
{
	direct_charge_mode |=  dc_mode;
	return;
}

void direct_charge_and_set_local_mode(int dc_mode)
{
	direct_charge_mode &=  dc_mode;
	return;
}

int direct_charge_get_local_mode(void)
{
	return direct_charge_mode;
}

/*lint -save -e* */
void direct_charge_update_cutoff_flag(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return;
	}
	di = g_di;

	if (pmic_vbus_irq_is_enabled() && di->cutoff_normal_flag){
		hwlog_info("[%s]cutoff_normal_flag = %d ! \n", __func__, di->cutoff_normal_flag);
		di->cutoff_normal_flag = 0;
	} else {
		di->quick_charge_flag = 0;
		di->super_charge_flag = 0;
		di->error_cnt = 0;
	}
}
void direct_charge_set_scp_stage_default(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;
	scp_set_stage_status(SCP_STAGE_DEFAULT);
	hwlog_info("[%s]cutoff_normal_flag = %d, scp_stage = %d ! \n", __func__,di->cutoff_normal_flag, di->scp_stage);
}

void direct_charge_stop_charging(void)
{
	struct direct_charge_device *di = NULL;
	if (NULL == g_di)
	{
		hwlog_err("%s g_di is NULL\n", __func__);
		return ;
	}
	di = g_di;

	di->full_path_resistance = ERROR_RESISTANCE;
	di->direct_charge_succ_flag = DIRECT_CHARGE_ERROR_ADAPTOR_DETECT;
	di->scp_stop_charging_flag_error = 0;
	di->scp_stop_charging_flag_info = 0;
	di->cur_stage = 0;
	di->pre_stage = 0;
	di->vbat = 0;
	di->ibat = 0;
	di->vadapt = 0;
	di->iadapt = 0;
	di->ls_vbus = 0;
	di->ls_ibus = 0;
	di->compensate_v = 0;
	di->cc_cable_detect_ok = 0;
	if (!strstr(saved_command_line, "androidboot.swtype=factory")) {
		set_direct_charger_disable_flags(DIRECT_CHARGER_CLEAR_DISABLE_FLAGS,
                                         DIRECT_CHARGER_SYS_NODE);
	}
	di->ibat_abnormal_cnt = 0;
	di->max_adaptor_cur = 0;
	di->dc_open_retry_cnt = 0;
	di->dc_err_report_flag = FALSE;
	memset(di->dc_err_dsm_buff, 0, sizeof(di->dc_err_dsm_buff));
}
/*lint -restore*/
