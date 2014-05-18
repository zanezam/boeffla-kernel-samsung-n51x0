/*
 * Copyright (C) 2011 Samsung Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _SII8246_H_
#define _SII8246_H_

#ifndef __MHL_NEW_CBUS_MSC_CMD__
#define	__MHL_NEW_CBUS_MSC_CMD__
/*
 * Read DCAP for distinguish TA and USB
 */
#endif

#ifdef __KERNEL__
struct sii8246_platform_data {
	u8 power_state;
	u8	swing_level;
	u8	factory_test;
	int ddc_i2c_num;
	int (*get_irq)(void);
	void (*init)(void);
	void (*mhl_sel)(bool enable);
	void (*hw_onoff)(bool on);
	void (*hw_reset)(void);
	void (*enable_vbus)(bool enable);
#if defined(__MHL_NEW_CBUS_MSC_CMD__)
	void (*vbus_present)(bool on, int value);
#else
	void (*vbus_present)(bool on);
#endif
#ifdef CONFIG_SAMSUNG_MHL_UNPOWERED
	int (*get_vbus_status)(void);
	void (*sii8246_otg_control)(bool onoff);
#endif
	void (*sii8246_muic_cb)(bool otg_enable, int plim);
	struct i2c_client *mhl_tx_client;
	struct i2c_client *tpi_client;
	struct i2c_client *hdmi_rx_client;
	struct i2c_client *cbus_client;

	int (*reg_notifier)(struct notifier_block *nb);
	int (*unreg_notifier)(struct notifier_block *nb);
#ifdef CONFIG_EXTCON
	const char *extcon_name;
#endif
};

int acc_notify(int event);
extern u8 sii8246_mhl_onoff_ex(bool onoff);
#endif

#if defined(__MHL_NEW_CBUS_MSC_CMD__)
#if defined(CONFIG_MFD_MAX77693)
extern void max77693_muic_usb_cb(u8 usb_mode);
#endif
#endif

#ifdef	CONFIG_SAMSUNG_WORKAROUND_HPD_GLANCE
extern	void mhl_hpd_handler(bool onoff);
#endif

#ifdef	CONFIG_SAMSUNG_USE_11PIN_CONNECTOR
extern	int	max77693_muic_get_status1_adc1k_value(void);
#endif

#ifdef	CONFIG_SAMSUNG_SMARTDOCK
extern	int	max77693_muic_get_status1_adc_value(void);
#endif

#ifdef CONFIG_MACH_MIDAS
extern void sii8246_wake_lock(void);
extern void sii8246_wake_unlock(void);
#endif

#ifdef CONFIG_JACK_MON
extern void jack_event_handler(const char *name, int value);
#endif

#endif /* _SII8246_H_ */
