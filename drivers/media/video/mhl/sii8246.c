/*
 * Copyright (C) 2011 Samsung Electronics
 *
 * Authors: Adam Hampson <ahampson@sta.samsung.com>
 *          Erik Gilling <konkers@android.com>
 *
 * Additional contributions by : Shankar Bandal <shankar.b@samsung.com>
 *                               Dharam Kumar <dharam.kr@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sii8246.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#ifdef CONFIG_EXTCON
#include <linux/extcon.h>
#endif

/*////////////////////////////////////////////////////////////////////////////*/
/*/////////////////////////	definition area		//////////////////////*/
/*////////////////////////////////////////////////////////////////////////////*/

#define	__CONFIG_RSEN_LOST_PATCH__
/* #define __CONFIG_MHL_SWING_LEVEL__ */
#define	__CONFIG_SS_FACTORY__
#ifdef CONFIG_MACH_GC2PD /* M project */
#	define	__CONFIG_MHL_FORCE_ON_FACTORY__
#endif
#define	__CONFIG_MHL_DEBUG__
#if defined(CONFIG_MACH_T0) || defined(CONFIG_MACH_M3) \
	|| defined(CONFIG_MACH_M0_DUOSCTC) || defined(CONFIG_MACH_KONA) \
	|| defined(CONFIG_MACH_TAB3) || defined(CONFIG_MACH_ZEST) \
	|| defined(CONFIG_MACH_GC2PD)
#	define __CONFIG_MHL_VER_1_2__
#else
#	define __CONFIG_MHL_VER_1_1__
#endif
/* #define	__SII8246_MUTEX_DEBUG__ */
/*////////////////////////////////////////////////////////////////////////////*/
/*//////////////////     dependence hader file area     //////////////////////*/
/*////////////////////////////////////////////////////////////////////////////*/

#ifdef __CONFIG_MHL_SWING_LEVEL__
#include <linux/ctype.h>
#endif
#include "sii8246_driver.h"

/*////////////////////////////////////////////////////////////////////////////*/
/*//////////////////////////     mecro area     //////////////////////////////*/
/*////////////////////////////////////////////////////////////////////////////*/

#undef pr_debug
#ifdef __CONFIG_MHL_DEBUG__
int sii8246_mhl_dbg_flag;
#	define pr_debug(fmt, ...) \
	do { \
		if (likely(sii8246_mhl_dbg_flag == 1)) { \
			printk(KERN_INFO fmt, ##__VA_ARGS__); \
		} \
	} while (0)
#else
#	define pr_debug(fmt, ...)
#endif

#ifdef __SII8246_MUTEX_DEBUG__
	int g_mutex_cnt;
	int g_cbus_mutex_cnt;
#	define	sii8246_mutex_lock(prm)	\
	do { \
		printk(KERN_INFO"%s(%d) mutex++:%d\n", __func__,\
				__LINE__, ++g_mutex_cnt); \
		mutex_lock(prm); \
		printk(KERN_INFO"%s(%d) mutex--:%d\n", __func__,\
				__LINE__, g_mutex_cnt); \
	} while (0)
#	define	sii8246_mutex_unlock(prm) \
	do { \
		printk(KERN_INFO"%s(%d) mutex_unlock:%d\n", __func__, \
				__LINE__, --g_mutex_cnt); \
		mutex_unlock(prm); \
	} while (0)
#	define	sii8246_cbus_mutex_lock(prm) \
	do { \
		printk(KERN_INFO"%s(%d) cbus mutex++:%d\n", __func__, \
				__LINE__, ++g_cbus_mutex_cnt); \
		mutex_lock(prm); \
		printk(KERN_INFO"%s(%d) cbus mutex--:%d\n", __func__, \
				__LINE__, g_cbus_mutex_cnt); \
	} while (0)
#	define	sii8246_cbus_mutex_unlock(prm)	\
	do { \
		printk(KERN_INFO"%s(%d) cbus mutex_unlock:%d\n", __func__, \
				__LINE__, --g_cbus_mutex_cnt); \
		mutex_unlock(prm); \
	} while (0)
#else
#	define	sii8246_mutex_lock(prm)	mutex_lock(prm);
#	define	sii8246_mutex_unlock(prm)	mutex_unlock(prm);
#	define	sii8246_cbus_mutex_lock(prm)	mutex_lock(prm);
#	define	sii8246_cbus_mutex_unlock(prm)	mutex_unlock(prm);
#endif /*__SII8246_MUTEX_DEBUG__*/

static struct mutex sii8246_irq_lock;
#define	__SII8246_IRQ_DEBUG__
#ifdef __SII8246_IRQ_DEBUG__
int sii8246_en_irq;
#	define sii8246_enable_irq() \
	do { \
		mutex_lock(&sii8246_irq_lock); \
		if (atomic_read(&sii8246->is_irq_enabled) == false) { \
			atomic_set(&sii8246->is_irq_enabled, true); \
			enable_irq(sii8246->pdata->mhl_tx_client->irq); \
			printk(KERN_INFO"%s() : enable_irq(%d)\n", __func__, \
					++sii8246_en_irq); \
		} else { \
			printk(KERN_INFO"%s() : irq is already enabled(%d)\n" \
					, __func__, sii8246_en_irq); \
		} \
		mutex_unlock(&sii8246_irq_lock); \
	} while (0)

#	define sii8246_disable_irq() \
	do { \
		mutex_lock(&sii8246_irq_lock); \
		if (atomic_read(&sii8246->is_irq_enabled) == true) { \
			atomic_set(&sii8246->is_irq_enabled, false); \
			disable_irq_nosync(sii8246->pdata->mhl_tx_client->irq);\
			printk(KERN_INFO"%s() : disable_irq(%d)\n", \
					__func__, --sii8246_en_irq); \
		} else { \
			printk(KERN_INFO"%s() : irq is already disabled(%d)\n"\
					, __func__, sii8246_en_irq); \
		} \
		mutex_unlock(&sii8246_irq_lock); \
	} while (0)
#else
#	define sii8246_enable_irq() \
	do { \
		mutex_lock(&sii8246_irq_lock); \
		if (atomic_read(&sii8246->is_irq_enabled) == false) { \
			atomic_set(&sii8246->is_irq_enabled, true); \
			enable_irq(sii8246->pdata->mhl_tx_client->irq); \
		} \
		mutex_unlock(&sii8246_irq_lock); \
	} while (0)

#	define sii8246_disable_irq() \
	do { \
		mutex_lock(&sii8246_irq_lock); \
		if (atomic_read(&sii8246->is_irq_enabled) == true) { \
			atomic_set(&sii8246->is_irq_enabled, false); \
			disable_irq_nosync(sii8246->pdata->mhl_tx_client->irq);\
		} \
		mutex_unlock(&sii8246_irq_lock); \
	} while (0)
#endif /*__SII8246_IRQ_DEBUG__*/

/*////////////////////////////////////////////////////////////////////////////*/
/*////////////////////     global value area     /////////////////////////////*/
/*////////////////////////////////////////////////////////////////////////////*/

#ifdef __MHL_NEW_CBUS_MSC_CMD__
LIST_HEAD(sii8246_g_msc_packet_list);
static int g_list_cnt;
static struct workqueue_struct *sii8246_msc_wq;
#endif

static struct cbus_packet cbus_pkt_buf[CBUS_PKT_BUF_COUNT];

#ifdef __CONFIG_TMDS_OFFON_WORKAROUND__
static struct workqueue_struct *sii8246_tmds_offon_wq;
#endif

/*////////////////////////////////////////////////////////////////////////////*/
/*/////////////////     function declaration  area     ///////////////////////*/
/*////////////////////////////////////////////////////////////////////////////*/

static u8 sii8246_tmds_control(struct sii8246_data *sii8246, bool enable);
#ifdef __CONFIG_TMDS_OFFON_WORKAROUND__
static u8 sii8246_tmds_control2(struct sii8246_data *sii8246, bool enable);

static bool cbus_command_request(struct sii8246_data *sii8246,
				 enum cbus_command command, u8 offset, u8 data);
static void cbus_command_response(struct sii8246_data *sii8246);
#endif
static irqreturn_t sii8246_irq_thread(int irq, void *data);

static void goto_d3(void);

/*////////////////////////////////////////////////////////////////////////////*/
/*////////////////     function description  area     ////////////////////////*/
/*////////////////////////////////////////////////////////////////////////////*/

#ifdef CONFIG_MACH_MIDAS
void sii8246_wake_lock(void)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	if (!sii8246 || !sii8246->pdata) {
		pr_debug("[ERROR] %s() MHL driver has not initailized.\n",
			 __func__);
		return;
	}
	wake_lock(&sii8246->mhl_wake_lock);
	pr_debug("%s()\n", __func__);
}
EXPORT_SYMBOL(sii8246_wake_lock);

void sii8246_wake_unlock(void)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	if (!sii8246 || !sii8246->pdata) {
		pr_debug("[ERROR] %s() MHL driver has not initailized.\n",
			 __func__);
		return;
	}
	wake_unlock(&sii8246->mhl_wake_lock);
	pr_debug("%s()\n", __func__);
}
EXPORT_SYMBOL(sii8246_wake_unlock);
#endif

#ifdef __CONFIG_MHL_SWING_LEVEL__
static ssize_t sii8246_swing_test_show(struct class *class,
				       struct class_attribute *attr,
				       char *buf)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	return sprintf(buf, "mhl_show_value : 0%o(0x%x)\n",
	       sii8246->pdata->swing_level, sii8246->pdata->swing_level);

}

static ssize_t sii8246_swing_test_store(struct class *class,
					struct class_attribute *attr,
					const char *buf, size_t size)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	const char *p = buf;
	int data, clk, ret;
	unsigned int base, value;

	if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X'))
		base = 16;
	else
		base = 8;

	ret = kstrtouint(p, base, &value);
	pr_debug(KERN_INFO "\n%s(): ret = %d, value = %d(=0%o=0x%x)\n", __func__,
		 ret, value, value, value);
	if (ret != 0 || value < 0 || value > 0xff) {
		pr_debug(KERN_INFO "[ERROR] %s(): The value is invald!",
				__func__);
		return size;
	}
	data = value & 070;
	clk = value & 07;
	sii8246->pdata->swing_level = 0300 | data | clk;
	pr_debug(KERN_INFO "%s(): mhl_store_value : 0%o(0x%x)\n", __func__,
		 sii8246->pdata->swing_level, sii8246->pdata->swing_level);
	return size;
}

static CLASS_ATTR(swing, 0666,
		sii8246_swing_test_show, sii8246_swing_test_store);
#endif

#if defined(CONFIG_SAMSUNG_USE_11PIN_CONNECTOR) && !defined(CONFIG_MACH_P4NOTE) && !defined(CONFIG_MACH_SP7160LTE)
static int is_mhl_cable_connected(void)
{
#	ifdef CONFIG_SAMSUNG_SMARTDOCK
		if (max77693_muic_get_status1_adc_value() == ADC_SMARTDOCK)
			return 1;
		else
			return max77693_muic_get_status1_adc1k_value();
#	else
		return max77693_muic_get_status1_adc1k_value();
#	endif
}
#endif
#if defined(CONFIG_SAMSUNG_WORKAROUND_HPD_GLANCE) &&\
	!defined(CONFIG_SAMSUNG_MHL_9290)
/*
 *  This workaournd is for prevent of HDMI_HPD pin glitch.
 *  HDMI_HPD pin is from MHL IC's HPD pin to AP's HDMI HPD pin.
 *  When connect/disconnect the MHL Dongle, or turn on/off MHL IC,
 * or other cases, the glitch can be generated.
 *  - Cases
 *  1) 2012 Sept
 *    a. When HDMI is connected.
 *    b. Remove TA/HDMI cable.
 *    c. Remove MHL dongle about 0.5 sec(shorter then 1 sec) after b.
 *    d. Glitch is occured with HDMI connected/disconnected pop-up.
 *
 *  void mhl_hpd_handler(bool onoff) must be defined on HDMI driver.
 *  That fuction will control the HDMI HPD high/low intterpt on/off.
 *
 *
 *  int (*sii8246_hpd_intr_state)(void);
 *  This function pointer shows whether HDMI HPD low interrrupt was
 * handled on HDMI driver or not. It must be assigned in HDMI driver's probe
 * function.
 */
int (*sii8246_hpd_intr_state)(void);
/*
 *  sii8246_sched_hpd_handler_false means that mhl_hpd_handler(false) must be called.
 *  It is set on sii8246_mhl_hpd_handler_false(). It is read and cleared on
 * call_sched_mhl_hpd_handler().
 */
atomic_t sii8246_sched_hpd_handler_false;
/*
 *  void call_sched_mhl_hpd_handler(void);
 *  This function must be called by HDMI HPD LOW IRQ handler.
 *  If the sii8246_sched_hpd_handler_false is 1, then it clears the
 * sii8246_sched_hpd_handler_false to 0 and call mhl_hpd_handler(false).
 */
void sii8246_call_sched_mhl_hpd_handler(void)
{
	if (atomic_read(&sii8246_sched_hpd_handler_false) == 1) {
		atomic_set(&sii8246_sched_hpd_handler_false, 0);
		mhl_hpd_handler(false);
		pr_debug("%s : called mhl_hpd_handler(false)\n", __func__);
	}
}
/*
 *  void sii8246_mhl_hpd_handler_false(void);
 *  This function checks whether HDMI HPD LOW IRQ handler was handled
 * by sii8246_hpd_intr_state().
 *  If IRQ handler was handled, mhl_hpd_handler(false) is called directly,
 * if not, mhl_hpd_handler(false) is scheduled by setting the
 * sii8246_sched_hpd_handler_false to 1.
 */
void sii8246_mhl_hpd_handler_false(void)
{
	if (sii8246_hpd_intr_state != NULL) {
		if (sii8246_hpd_intr_state() == LOW) {
			mhl_hpd_handler(false);
		} else {
			atomic_set(&sii8246_sched_hpd_handler_false, 1);
			pr_debug("%s : mhl_hpd_handler(false)"
					" is scheduled\n", __func__);
		}
	} else {
		mhl_hpd_handler(false);
	}
}
#endif

u8 sii8246_mhl_onoff_ex(bool onoff)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	int ret;

	pr_debug("MHL %s(%s)\n", __func__, onoff ? "on" : "off");
		pr_debug("MHL sii8246_mhl_onoff_ex (): system_rev  = %d \n", system_rev );

	if (!sii8246 || !sii8246->pdata) {
		pr_debug("sii8246: sii8246_mhl_onoff_ex: getting resource is failed\n");
		return 2;
	}

	if (sii8246->pdata->power_state == onoff) {
		pr_debug("sii8246: sii8246_mhl_onoff_ex: mhl already %s\n",
			onoff ? "on" : "off");
		return 2;
	}

	sii8246->pdata->power_state = onoff;	/*save power state */

	if (sii8246->pdata->mhl_sel)
		sii8246->pdata->mhl_sel(onoff);

	if (onoff) {
		if (sii8246->pdata->hw_onoff)
			sii8246->pdata->hw_onoff(1);

		if (sii8246->pdata->hw_reset)
			sii8246->pdata->hw_reset();

		goto_d3();
		return 2;
	} else {
		if (sii8246->mhl_event_switch.state == 1) {
			pr_debug("%s: MHL switch event sent : 0\n", __func__);
			switch_set_state(&sii8246->mhl_event_switch, 0);
		}
		sii8246_cancel_callback();

#ifdef __MHL_NEW_CBUS_MSC_CMD__
		if (sii8246->pdata->sii8246_muic_cb)
			sii8246->pdata->sii8246_muic_cb(false, -1);
#endif
		if (sii8246->pdata->hw_onoff)
			sii8246->pdata->hw_onoff(0);

#if defined(CONFIG_SAMSUNG_WORKAROUND_HPD_GLANCE) &&\
	!defined(CONFIG_SAMSUNG_MHL_9290)
		sii8246_mhl_hpd_handler_false();
#endif

#ifdef CONFIG_SAMSUNG_USE_11PIN_CONNECTOR
#if !defined(CONFIG_MACH_P4NOTE) && !defined(CONFIG_MACH_SP7160LTE)
		ret = is_mhl_cable_connected();
#endif
		if (ret == 1) {
			pr_debug("sii8246: %s() mhl still inserted, "
				 "retry discovery\n", __func__);
			schedule_work(&sii8246->mhl_restart_work);
		} else if (ret == 0) {
			pr_debug("sii8246: %s() mhl cable is removed\n",
				__func__);
		} else {
			pr_debug
			    ("[ERROR] %s() is_mhl_cable_connected error : %d\n",
			     __func__, ret);
		}
#endif
	}
	return sii8246->rgnd;
}
EXPORT_SYMBOL(sii8246_mhl_onoff_ex);

static int mhl_tx_write_reg(struct sii8246_data *sii8246, unsigned int offset,
			    u8 value)
{
	int ret;
//	pr_debug("MHL %s %x %x\n",__func__,offset,value);
	ret = i2c_smbus_write_byte_data(sii8246->pdata->mhl_tx_client, offset,
					value);
	if (ret < 0)
		pr_debug("[ERROR] sii8246 : %s(0x%02x, 0x%02x)\n", __func__,  offset, value);
	return ret;
}

static int mhl_tx_read_reg(struct sii8246_data *sii8246, unsigned int offset,
			   u8 *value)
{
	int ret;

//		pr_debug("MHL %s(0x%02x)\n", __func__, offset);
	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(sii8246->pdata->mhl_tx_client, offset);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x)\n", __func__, offset);
		return ret;
	}

	ret = i2c_smbus_read_byte(sii8246->pdata->mhl_tx_client);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x)\n", __func__, offset);
		return ret;
	}

	*value = ret & 0x000000FF;

	return 0;
}

static int mhl_tx_set_reg(struct sii8246_data *sii8246, unsigned int offset,
			  u8 mask)
{
	int ret;
	u8 value;

//		pr_debug("MHL %s(0x%02x, 0x%02x)\n", __func__,
//		       offset, mask);
	ret = mhl_tx_read_reg(sii8246, offset, &value);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x, 0x%02x)\n", __func__,
		       offset, mask);
		return ret;
	}

	value |= mask;

	return mhl_tx_write_reg(sii8246, offset, value);
}

static int mhl_tx_clear_reg(struct sii8246_data *sii8246, unsigned int offset,
			    u8 mask)
{
	int ret;
	u8 value;

//		pr_debug("MHL %s(0x%02x, 0x%02x)\n", __func__,
//		       offset, mask);
	ret = mhl_tx_read_reg(sii8246, offset, &value);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x, 0x%02x)\n", __func__,
		       offset, mask);
		return ret;
	}

	value &= ~mask;

	ret = mhl_tx_write_reg(sii8246, offset, value);
	if (ret < 0)
		pr_debug("[ERROR] sii8246 : %s(0x%02x, 0x%02x)\n", __func__,
		       offset, mask);
	return ret;
}

static int tpi_write_reg(struct sii8246_data *sii8246, unsigned int offset,
			 u8 value)
{
	int ret = 0;
//	pr_debug("MHL %s %x %x\n",__func__,offset,value);
	ret = i2c_smbus_write_byte_data(sii8246->pdata->tpi_client, offset,
					value);
	if (ret < 0)
		pr_debug("[ERROR] sii8246 : %s(0x%02x, 0x%02x)\n", __func__,
		       offset, value);
	return ret;
}

static int tpi_read_reg(struct sii8246_data *sii8246, unsigned int offset,
			u8 *value)
{
	int ret;

//		pr_debug("MHL %s(0x%02x)\n", __func__, offset);
	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(sii8246->pdata->tpi_client, offset);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x)\n", __func__, offset);
		return ret;
	}

	ret = i2c_smbus_read_byte(sii8246->pdata->tpi_client);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x)\n", __func__, offset);
		return ret;
	}

	*value = ret & 0x000000FF;

	return 0;
}

static int hdmi_rx_read_reg(struct sii8246_data *sii8246, unsigned int offset,
			    u8 *value)
{
	int ret;

//		pr_debug("MHL %s(0x%02x)\n", __func__, offset);
	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(sii8246->pdata->hdmi_rx_client, offset);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x)\n", __func__, offset);
		return ret;
	}

	ret = i2c_smbus_read_byte(sii8246->pdata->hdmi_rx_client);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x)\n", __func__, offset);
		return ret;
	}

	*value = ret & 0x000000FF;

	return 0;
}

static int hdmi_rx_write_reg(struct sii8246_data *sii8246, unsigned int offset,
			     u8 value)
{
	int ret;
//	pr_debug("MHL %s %x %x\n",__func__,offset,value);
	ret = i2c_smbus_write_byte_data(sii8246->pdata->hdmi_rx_client, offset,
					value);
	if (ret < 0)
		pr_debug("[ERROR] sii8246 : %s(0x%02x, 0x%02x)\n", __func__,
		       offset, value);
	return ret;
}

static int cbus_write_reg(struct sii8246_data *sii8246, unsigned int offset,
			  u8 value)
{
//	pr_debug("MHL %s %x %x\n",__func__,offset,value);
	return i2c_smbus_write_byte_data(sii8246->pdata->cbus_client, offset,
					 value);
}

static int cbus_read_reg(struct sii8246_data *sii8246, unsigned int offset,
			 u8 *value)
{
	int ret;

	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(sii8246->pdata->cbus_client, offset);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x)\n", __func__, offset);
		return ret;
	}

	ret = i2c_smbus_read_byte(sii8246->pdata->cbus_client);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x)\n", __func__, offset);
		return ret;
	}

	*value = ret & 0x000000FF;

	return 0;
}

static int cbus_set_reg(struct sii8246_data *sii8246, unsigned int offset,
			u8 mask)
{
	int ret;
	u8 value;

	ret = cbus_read_reg(sii8246, offset, &value);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x, 0x%02x)\n", __func__,
		       offset, mask);
		return ret;
	}

	value |= mask;

	return cbus_write_reg(sii8246, offset, value);
}
static int cbus_clear_reg(struct sii8246_data *sii8246, unsigned int offset,
			    u8 mask)
{
	int ret;
	u8 value;

	ret = cbus_read_reg(sii8246, offset, &value);
	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s(0x%02x, 0x%02x)\n", __func__,
		       offset, mask);
		return ret;
	}

	value &= ~mask;

	ret = cbus_write_reg(sii8246, offset, value);
	if (ret < 0)
		pr_debug("[ERROR] sii8246 : %s(0x%02x, 0x%02x)\n", __func__,
		       offset, mask);
	return ret;
}


#ifdef __CONFIG_TMDS_OFFON_WORKAROUND__
void sii8246_tmds_offon_work(struct work_struct *work)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);

	pr_debug("MHL %s() \n", __func__);
	sii8246_tmds_control2(sii8246, false);
	sii8246_tmds_control2(sii8246, true);
}
#endif

static int sii8246_cbus_reset(struct sii8246_data *sii8246)
{
	int ret;
	u8 idx;
	pr_debug("MHL %s\n",__func__);
	/* Reset CBUS */
	ret = mhl_tx_set_reg(sii8246, MHL_TX_SRST, (1 << 3));
	if (ret < 0)
		return ret;

	msleep(T_SRC_CBUS_DEGLITCH);

	ret = mhl_tx_clear_reg(sii8246, MHL_TX_SRST, (1 << 3));
	if (ret < 0)
		return ret;

	for (idx = 0; idx < 4; idx++) {
		/* Enable WRITE_STAT interrupt for writes to all
		   4 MSC Status registers. */
		ret = cbus_write_reg(sii8246, 0xE0 + idx, 0xFF);
		if (ret < 0)
			return ret;

		/*Enable SET_INT interrupt for writes to all
		   4 MSC Interrupt registers. */
		ret = cbus_write_reg(sii8246, 0xF0 + idx, 0xFF);
		if (ret < 0)
			return ret;
	}

	return 0;

}

/* require to chek mhl imformation of samsung in cbus_init_register*/
static int sii8246_cbus_init(struct sii8246_data *sii8246)
{
	u8 value;
	int ret;
	pr_debug("MHL %s\n",__func__);

	ret = cbus_write_reg(sii8246, 0x07, 0xF2);
	if (ret < 0)
		goto i2c_error_exit;

	ret = cbus_write_reg(sii8246, 0x40, 0x03);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x42, 0x06);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x36, 0x0B);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x3D, 0xFD);
	if (ret < 0)
		goto i2c_error_exit;

	ret = cbus_write_reg(sii8246, 0x1C, 0x01);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x1D, 0x0F);
	if (ret < 0)
		goto i2c_error_exit;
/* --
	ret = cbus_write_reg(sii8246, 0x44, 0x02);
	if (ret < 0)
		goto i2c_error_exit;
*/
	/* Setup our devcap */
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_DEV_STATE, 0x00);
				/*To meet cts 6.3.10.1 spec */
	if (ret < 0)
		goto i2c_error_exit;

	/*mhl version 1.2 */
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_MHL_VERSION, 0x12);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_DEV_CAT, 0x02);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_ADOPTER_ID_H, 0x01);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_ADOPTER_ID_L, 0x41);
	if (ret < 0)
		goto i2c_error_exit;
	/*only RGB fmt*/
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_VID_LINK_MODE, 0x03);
	if (ret < 0)
		goto i2c_error_exit;
#ifdef CONFIG_VIDEO_TVOUT_5_1CH_AUDIO
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_AUD_LINK_MODE, 0x03);
				/* 8ch, 2ch */
#else
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_AUD_LINK_MODE, 0x01);
				/* 2ch */
#endif
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_VIDEO_TYPE, 0);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_LOG_DEV_MAP, (1 << 7));
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_BANDWIDTH, 0x0F);
	if (ret < 0)
		goto i2c_error_exit;
	ret =
	    cbus_write_reg(sii8246, 0x80 + DEVCAP_DEV_FEATURE_FLAG,
			   (1 << 0) | (1 << 1) | (1 << 2));
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_DEVICE_ID_H, 0x0);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_DEVICE_ID_L, 0x0);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_SCRATCHPAD_SIZE, 0x10);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_INT_STAT_SIZE, 0x33);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, 0x80 + DEVCAP_RESERVED, 0);
	if (ret < 0)
		goto i2c_error_exit;

	ret = cbus_read_reg(sii8246, 0x31, &value);
	if (ret < 0)
		goto i2c_error_exit;
	value |= 0x0C;
	ret = cbus_write_reg(sii8246, 0x31, value);
	if (ret < 0)
		goto i2c_error_exit;

	ret = cbus_read_reg(sii8246, 0x22, &value);
	if (ret < 0)
		goto i2c_error_exit;
	value &= ~0x0F;
	value |= 0x0F;

	ret = cbus_write_reg(sii8246, 0x22, (value&0x0F));
	if (ret < 0)
		goto i2c_error_exit;

	ret = cbus_write_reg(sii8246, 0x30, 0x01);
	if (ret < 0)
		goto i2c_error_exit;

	ret = cbus_write_reg(sii8246, 0x3C, 0xB4);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_read_reg(sii8246, 0x22, &value);
	if (ret < 0)
		goto i2c_error_exit;
	value &= ~0x0F;
	value |= 0x0D;
	ret = cbus_write_reg(sii8246, 0x22, value);
	if (ret < 0)
		goto i2c_error_exit;

	ret = cbus_read_reg(sii8246, 0x2E, &value);
	if (ret < 0)
		goto i2c_error_exit;
	value |= 0x14;
	ret = cbus_write_reg(sii8246, 0x2E, value);
	if (ret < 0)
		goto i2c_error_exit;
/*
	ret = cbus_write_reg(sii8246, CBUS_INTR1_ENABLE_REG, 0);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, CBUS_INTR2_ENABLE_REG, 0);
	if (ret < 0)
		goto i2c_error_exit;
*/
	return 0;
 i2c_error_exit:
	pr_debug("[ERROR] %s()\n", __func__);
	return ret;
}

static void cbus_req_abort_error(struct sii8246_data *sii8246)
{
	u8 abort_reason = 0;
	pr_debug("MHL %s\n",__func__);

	pr_debug("sii8246: MSC Request Aborted:");

	cbus_read_reg(sii8246, MSC_REQ_ABORT_REASON_REG, &abort_reason);

	if (abort_reason) {
		if (abort_reason & BIT_MSC_XFR_ABORT) {
			cbus_read_reg(sii8246, MSC_REQ_ABORT_REASON_REG,
				      &abort_reason);
			pr_cont("ABORT_REASON_REG = %d\n", abort_reason);
			cbus_write_reg(sii8246, MSC_REQ_ABORT_REASON_REG, 0xff);
		}
		if (abort_reason & BIT_MSC_ABORT) {
			cbus_read_reg(sii8246, BIT_MSC_ABORT, &abort_reason);
			pr_cont("BIT_MSC_ABORT = %d\n", abort_reason);
			cbus_write_reg(sii8246, BIT_MSC_ABORT, 0xff);
		}
		if (abort_reason & ABORT_BY_PEER)
			pr_cont(" Peer Sent an ABORT:");
		if (abort_reason & UNDEF_CMD)
			pr_cont(" Undefined Opcode:");
		if (abort_reason & TIMEOUT)
			pr_cont(" Requestor Translation layer Timeout:");
		if (abort_reason & PROTO_ERROR)
			pr_cont(" Protocol Error:");
		if (abort_reason & MAX_FAIL) {
			u8 msc_retry_thr_val = 0;
			pr_cont(" Retry Threshold exceeded:");
			cbus_read_reg(sii8246,
				      MSC_RETRY_FAIL_LIM_REG,
				      &msc_retry_thr_val);
			pr_cont("Retry Threshold value is:%d",
				msc_retry_thr_val);
		}
	}
	pr_cont("\n");
}

static void cbus_resp_abort_error(struct sii8246_data *sii8246)
{
	u8 abort_reason = 0;
	pr_debug("MHL %s\n",__func__);

	pr_debug("sii8246: MSC Response Aborted:");
	cbus_read_reg(sii8246, MSC_RESP_ABORT_REASON_REG, &abort_reason);
	cbus_write_reg(sii8246, MSC_RESP_ABORT_REASON_REG, 0xff);
	if (abort_reason) {
		if (abort_reason & ABORT_BY_PEER)
			pr_cont(" Peer Sent an ABORT");
		if (abort_reason & UNDEF_CMD)
			pr_cont(" Undefined Opcode");
		if (abort_reason & TIMEOUT)
			pr_cont(" Requestor Translation layer Timeout");
	}
	pr_cont("\n");
}

static void force_usb_id_switch_open(struct sii8246_data *sii8246)
{
	pr_debug("MHL %s\n",__func__);
	/*Disable CBUS discovery */
	mhl_tx_clear_reg(sii8246, MHL_TX_DISC_CTRL1_REG, (1 << 0));
	/*Force USB ID switch to open */
	mhl_tx_set_reg(sii8246, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);

	mhl_tx_set_reg(sii8246, MHL_TX_DISC_CTRL3_REG, 0x86);
	/*Force upstream HPD to 0 when not in MHL mode. */
 	mhl_tx_set_reg(sii8246, MHL_TX_INT_CTRL_REG, 1<<4);
	mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG, 1 << 5);
	//mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 4) | (1 << 5));
}

static void release_usb_id_switch_open(struct sii8246_data *sii8246)
{
	pr_debug("MHL %s\n",__func__);
	msleep(T_SRC_CBUS_FLOAT);
	/* clear USB ID switch to open */
	mhl_tx_clear_reg(sii8246, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);
	/* Enable CBUS discovery */
	mhl_tx_set_reg(sii8246, MHL_TX_DISC_CTRL1_REG, (1 << 0));
}

static bool cbus_ddc_abort_error(struct sii8246_data *sii8246)
{
	u8 val1, val2;
	pr_debug("MHL %s\n",__func__);

	/* clear the ddc abort counter */
	cbus_write_reg(sii8246, 0x29, 0xFF);
	cbus_read_reg(sii8246, 0x29, &val1);
	usleep_range(3000, 4000);
	cbus_read_reg(sii8246, 0x29, &val2);
	if (val2 > (val1 + 50)) {
		pr_debug("Applying DDC Abort Safety(SWA 18958)\n)");
		mhl_tx_set_reg(sii8246, MHL_TX_SRST, (1 << 3));
		mhl_tx_clear_reg(sii8246, MHL_TX_SRST, (1 << 3));
		force_usb_id_switch_open(sii8246);
		release_usb_id_switch_open(sii8246);
		mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL1_REG, 0xD0);
		sii8246_tmds_control(sii8246, false);
		/* Disconnect and notify to OTG */
		return true;
	}
	pr_debug("sii8246: DDC abort interrupt\n");

	return false;
}

#ifdef CONFIG_SII8246_RCP
static void rcp_uevent_report(struct sii8246_data *sii8246, u8 key)
{
	pr_debug("MHL %s\n",__func__);
	if (!sii8246->input_dev) {
		pr_debug("%s: sii8246->input_dev is NULL & "
		       "skip rcp_report\n", __func__);
		return;
	}

	pr_info("sii8246: rcp_uevent_report key: %d\n", key);
	input_report_key(sii8246->input_dev, (unsigned int)key + 1, 1);
	input_report_key(sii8246->input_dev, (unsigned int)key + 1, 0);
	input_sync(sii8246->input_dev);
}

/*
 * is_rcp_code_valid: Validdates the recevied RCP key,
 * valid key is 1 to 1  map to fwk keylayer file sii8246_rcp.kl
 * located at (/system/usr/keylayout/sii8246_rcp.kl).
 *
 * New key support needs to be update is_rcp_key_code_valid at
 * driver side and /system/usr/keylayout/sii8246_rcp.kl at fwk side.
 */

static int is_rcp_key_code_valid(u8 key)
{
	pr_debug("MHL %s\n",__func__);
	switch (key + 1) {
		/*should resemble /system/usr/keylayout/sii8246_rcp.kl */
	case 1:		/* ENTER                WAKE_DROPPED */
	case 2:		/* DPAD_UP              WAKE_DROPPED */
	case 3:		/* DPAD_DOWN            WAKE_DROPPED */
	case 4:		/* DPAD_LEFT            WAKE_DROPPED */
	case 5:		/* DPAD_RIGHT           WAKE_DROPPED */
	case 10:		/* MENU                 WAKE_DROPPED */
	case 14:		/* BACK                 WAKE_DROPPED */
	case 33:		/* 0    */
	case 34:		/* 1    */
	case 35:		/* 2    */
	case 36:		/* 3    */
	case 37:		/* 4    */
	case 38:		/* 5    */
	case 39:		/* 6    */
	case 40:		/* 7    */
	case 41:		/* 8    */
	case 42:		/* 9    */
	case 43:		/* ENTER */
	case 45:		/* DEL  */
	case 69:		/* MEDIA_PLAY_PAUSE         WAKE */
	case 70:		/* MEDIA_STOP               WAKE */
	case 71:		/* MEDIA_PAUSE              WAKE */
	case 73:		/* MEDIA_REWIND             WAKE */
	case 74:		/* MEDIA_FAST_FORWARD       WAKE */
	case 76:		/* MEDIA_NEXT               WAKE */
	case 77:		/* MEDIA_PREVIOUS           WAKE */
		return 1;
	default:
		return 0;
	}

}

static void cbus_process_rcp_key(struct sii8246_data *sii8246, u8 key)
{
	pr_debug("MHL %s\n",__func__);
	if (key == 0x7E) {
		pr_debug("sii8246: MHL switch event sent : 1\n");
		switch_set_state(&sii8246->mhl_event_switch, 1);
	}

	if (is_rcp_key_code_valid(key)) {
		/* Report the key */
		rcp_uevent_report(sii8246, key);
		/* Send the RCP ack */
#ifndef __MHL_NEW_CBUS_MSC_CMD__
		cbus_command_request(sii8246, CBUS_MSC_MSG, MSG_RCPK, key);
#else
		sii8246_enqueue_msc_work(sii8246, CBUS_MSC_MSG,
					 MSG_RCPK, key, 0x0);
#endif
	} else {
		sii8246->error_key = key;
		/*
		 * Send a RCPE(RCP Error Message) to Peer followed by
		 * RCPK with old key-code so that initiator(TV) can
		 * recognize failed key code.error code = 0x01 means
		 * Ineffective key code was received.
		 * See Table 21.(PRM)for details.
		 */
#ifndef __MHL_NEW_CBUS_MSC_CMD__
		cbus_command_request(sii8246, CBUS_MSC_MSG, MSG_RCPE, 0x01);
#else
		sii8246_enqueue_msc_work(sii8246, CBUS_MSC_MSG, MSG_RCPE, 0x01,
					 0x0);
#endif
	}
}
#endif

static void cbus_process_rap_key(struct sii8246_data *sii8246, u8 key)
{
	pr_debug("MHL %s\n",__func__);
	if (CBUS_MSC_RAP_CONTENT_ON == key)
		sii8246_tmds_control(sii8246, true);
	else if (CBUS_MSC_RAP_CONTENT_OFF == key)
		sii8246_tmds_control(sii8246, false);

#ifndef __MHL_NEW_CBUS_MSC_CMD__
	cbus_command_request(sii8246, CBUS_MSC_MSG, MSG_RAPK, 0x00);
#else
	sii8246_enqueue_msc_work(sii8246, CBUS_MSC_MSG, MSG_RAPK, 0x00, 0x0);
#endif
}

/*
 * Incoming MSC_MSG : RCP/RAP/RCPK/RCPE/RAPK commands
 *
 * Process RCP key codes and the send supported keys to userspace.
 * If a key is not supported then an error ack is sent to the peer.  Note
 * that by default all key codes are supported.
 *
 * An alternate method might be to decide the validity of the key in the
 * driver itself.  However, the driver does not have any criteria to which
 * to make this decision.
 */
static void cbus_handle_msc_msg(struct sii8246_data *sii8246)
{
	u8 cmd_code, key;

	sii8246_cbus_mutex_lock(&sii8246->cbus_lock);
	if (sii8246->state != STATE_ESTABLISHED) {
		pr_debug("sii8246: invalid MHL state\n");
		sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
		return;
	}
	pr_debug("MHL %s\n",__func__);

	cbus_read_reg(sii8246, CBUS_MSC_MSG_CMD_IN_REG, &cmd_code);
	cbus_read_reg(sii8246, CBUS_MSC_MSG_DATA_IN_REG, &key);

	pr_debug("sii8246: cmd_code:%d, key:%d\n", cmd_code, key);

	switch (cmd_code) {
	case MSG_RCP:
		pr_debug("sii8246: RCP Arrived. KEY CODE:%d\n", key);
		sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
		cbus_process_rcp_key(sii8246, key);
		return;
	case MSG_RAP:
		pr_debug("sii8246: RAP Arrived\n");
		sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
		cbus_process_rap_key(sii8246, key);
		return;
	case MSG_RCPK:
		pr_debug("sii8246: RCPK Arrived\n");
		break;
	case MSG_RCPE:
		pr_debug("sii8246: RCPE Arrived\n");
		break;
	case MSG_RAPK:
		pr_debug("sii8246: RAPK Arrived\n");
		break;
	default:
		pr_debug("sii8246: MAC error\n");
		sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
#ifndef __MHL_NEW_CBUS_MSC_CMD__
		cbus_command_request(sii8246, CBUS_GET_MSC_ERR_CODE, 0, 0);
#else
		sii8246_enqueue_msc_work(sii8246, CBUS_GET_MSC_ERR_CODE, 0, 0,
					 0x0);
#endif
		return;
	}
	sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
}

void sii8246_mhl_path_enable(struct sii8246_data *sii8246, bool path_en)
{
	pr_debug("sii8246: sii8246_mhl_path_enable MHL_STATUS_PATH_ENABLED,"
		 " path_en=%d !!!\n", path_en);
	pr_debug("MHL %s\n",__func__);

	if (path_en)
		sii8246->mhl_status_value.linkmode |= MHL_STATUS_PATH_ENABLED;
	else
		sii8246->mhl_status_value.linkmode &= ~MHL_STATUS_PATH_ENABLED;
#ifndef __MHL_NEW_CBUS_MSC_CMD__
	cbus_command_request(sii8246, CBUS_WRITE_STAT, CBUS_LINK_CONTROL_2_REG,
			     sii8246->mhl_status_value.linkmode);
#else
	sii8246_enqueue_msc_work(sii8246, CBUS_WRITE_STAT,
				 CBUS_LINK_CONTROL_2_REG,
				 sii8246->mhl_status_value.linkmode, 0x0);
#endif
}

static void cbus_handle_wrt_burst_recd(struct sii8246_data *sii8246)
{
	pr_debug("MHL %s\n",__func__);
	pr_debug("sii8246: CBUS WRT_BURST_RECD\n");
}

static void cbus_handle_wrt_stat_recd(struct sii8246_data *sii8246)
{
	u8 status_reg0, status_reg1, value;
	pr_debug("MHL %s\n",__func__);

	pr_debug("sii8246: CBUS WRT_STAT_RECD\n");

	/*
	 * The two MHL status registers need to read to ensure that the MSC is
	 * ready to receive the READ_DEVCAP command.
	 * The READ_DEVCAP command is need to determine the dongle power state
	 * and whether RCP, RCPE, RCPK, RAP, and RAPE are supported.
	 *
	 * Note that this is not documented properly in the PRM.
	 */

	cbus_read_reg(sii8246, CBUS_MHL_STATUS_REG_0, &status_reg0);
	cbus_write_reg(sii8246, CBUS_MHL_STATUS_REG_0, 0xFF);
	cbus_read_reg(sii8246, CBUS_MHL_STATUS_REG_1, &status_reg1);
	cbus_write_reg(sii8246, CBUS_MHL_STATUS_REG_1, 0xFF);

	pr_debug("sii8246: STATUS_REG0 : [%d];STATUS_REG1 : [%d]\n",
		 status_reg0, status_reg1);

	/* clear WRT_STAT_RECD intr */
	cbus_read_reg(sii8246, CBUS_MHL_STATUS_REG_0, &value);
	cbus_write_reg(sii8246, CBUS_MHL_STATUS_REG_0, value);

	cbus_read_reg(sii8246, CBUS_MHL_STATUS_REG_1, &value);
	cbus_write_reg(sii8246, CBUS_MHL_STATUS_REG_1, value);

	cbus_read_reg(sii8246, CBUS_MHL_STATUS_REG_2, &value);
	cbus_write_reg(sii8246, CBUS_MHL_STATUS_REG_2, value);

	cbus_read_reg(sii8246, CBUS_MHL_STATUS_REG_3, &value);
	cbus_write_reg(sii8246, CBUS_MHL_STATUS_REG_3, value);

	if (!(sii8246->mhl_status_value.linkmode & MHL_STATUS_PATH_ENABLED) &&
	    (MHL_STATUS_PATH_ENABLED & status_reg1)) {
		sii8246_mhl_path_enable(sii8246, true);
	} else if ((sii8246->mhl_status_value.linkmode
		    & MHL_STATUS_PATH_ENABLED) &&
		   !(MHL_STATUS_PATH_ENABLED & status_reg1)) {
		sii8246_mhl_path_enable(sii8246, false);
	}

	if (status_reg0 & MHL_STATUS_DCAP_READY) {
		pr_debug("sii8246: DEV CAP READY\n");
#ifndef __MHL_NEW_CBUS_MSC_CMD__
		cbus_command_request(sii8246, CBUS_READ_DEVCAP,
				     DEVCAP_DEV_CAT, 0x00);
		cbus_command_request(sii8246, CBUS_READ_DEVCAP,
				     DEVCAP_DEV_FEATURE_FLAG, 0x00);
#else
		sii8246_enqueue_msc_work(sii8246, CBUS_READ_DEVCAP,
					 DEVCAP_MHL_VERSION, 0x00, 0x0);
		sii8246_enqueue_msc_work(sii8246, CBUS_READ_DEVCAP,
					 DEVCAP_DEV_CAT, 0x00, 0x0);
		sii8246_enqueue_msc_work(sii8246, CBUS_READ_DEVCAP,
					 DEVCAP_DEV_FEATURE_FLAG, 0x00, 0x0);
		sii8246_enqueue_msc_work(sii8246, CBUS_READ_DEVCAP,
					 DEVCAP_DEVICE_ID_H, 0x0, 0x0);
		sii8246_enqueue_msc_work(sii8246, CBUS_READ_DEVCAP,
					 DEVCAP_DEVICE_ID_L, 0x0, 0x0);
		sii8246_enqueue_msc_work(sii8246, CBUS_READ_DEVCAP,
					 DEVCAP_RESERVED, 0x0, 0x0);
#endif
	}
}

static void cbus_handle_set_int_recd(struct sii8246_data *sii8246)
{
	u8 intr_reg0, intr_reg1, value;
	pr_debug("MHL %s\n",__func__);

	/* read and clear interrupt */
	cbus_read_reg(sii8246, CBUS_MHL_INTR_REG_0, &intr_reg0);
	cbus_write_reg(sii8246, CBUS_MHL_INTR_REG_0, intr_reg0);

	cbus_read_reg(sii8246, CBUS_MHL_INTR_REG_1, &intr_reg1);
	cbus_write_reg(sii8246, CBUS_MHL_INTR_REG_1, intr_reg1);

	pr_debug("sii8246: INTR_REG0 : [%d]; INTR_REG1 : [%d]\n",
		 intr_reg0, intr_reg1);

	if (intr_reg0 & MHL_INT_DCAP_CHG) {
#ifndef __MHL_NEW_CBUS_MSC_CMD__
		pr_debug("sii8246: MHL_INT_DCAP_CHG\n");
		cbus_command_request(sii8246, CBUS_READ_DEVCAP,
				     DEVCAP_DEV_CAT, 0x00);
		cbus_command_request(sii8246, CBUS_READ_DEVCAP,
				     DEVCAP_DEV_FEATURE_FLAG, 0x00);
#endif
	}

	if (intr_reg0 & MHL_INT_DSCR_CHG)
		pr_debug("sii8246:  MHL_INT_DSCR_CHG\n");

	if (intr_reg0 & MHL_INT_REQ_WRT) {
		pr_debug("sii8246:  MHL_INT_REQ_WRT\n");
#ifndef __MHL_NEW_CBUS_MSC_CMD__
		cbus_command_request(sii8246, CBUS_SET_INT,
				     MHL_RCHANGE_INT, MHL_INT_GRT_WRT);
#else
		sii8246_enqueue_msc_work(sii8246, CBUS_SET_INT,
					 MHL_RCHANGE_INT, MHL_INT_GRT_WRT, 0x0);
#endif
	}

	if (intr_reg0 & MHL_INT_GRT_WRT)
		pr_debug("sii8246:  MHL_INT_GRT_WRT\n");

	if (intr_reg1 & MHL_INT_EDID_CHG) {
		pr_debug("sii8246:  MHL_INT_EDID_CHG\n");
		/* Enable Overriding HPD OUT */
		mhl_tx_set_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 4));

		/*
		 * As per HDMI specification to indicate EDID change
		 * in TV (or sink), we need to toggle HPD line.
		 */

		/* HPD OUT = Low */
		mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 5));

		/* A SET_HPD command shall not follow a CLR_HPD command
		 * within less than THPD_WIDTH(50ms).
		 */
		msleep(T_HPD_WIDTH);

		/* HPD OUT = High */
		mhl_tx_set_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 5));

		/* Disable Overriding of HPD OUT */
		mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 4));
	}

	/* clear SET_INT_RECD interrupt */
	cbus_read_reg(sii8246, CBUS_MHL_INTR_REG_2, &value);
	cbus_write_reg(sii8246, CBUS_MHL_INTR_REG_2, value);

	cbus_read_reg(sii8246, CBUS_MHL_INTR_REG_3, &value);
	cbus_write_reg(sii8246, CBUS_MHL_INTR_REG_3, value);
}

static int sii8246_power_init(struct sii8246_data *sii8246)
{
	int ret;
	pr_debug("MHL %s\n",__func__);

	/* Force the SiI8246 into the D0 state. */
	ret = tpi_write_reg(sii8246, TPI_DPD_REG, 0x35);
	if (ret < 0)
		return ret;

	/* Enable TxPLL Clock */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_TMDS_CLK_EN_REG, 0x01);
	if (ret < 0)
		return ret;

	/* Enable Tx Clock Path & Equalizer */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_TMDS_CH_EN_REG, 0x15);
	if (ret < 0)
		return ret;

	/* Power Up TMDS */
	ret = mhl_tx_write_reg(sii8246, 0x08, 0x01);
	if (ret < 0)
		return ret;

	return ret;
}

static int sii8246_hdmi_init(struct sii8246_data *sii8246)
{
	int ret = 0;
	/* Analog PLL Control
	 * bits 5:4 = 2b00 as per characterization team.
	 */
	pr_debug("MHL %s\n",__func__);
	/*-
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_TMDS0_CCTRL1_REG, 0xC1);
	if (ret < 0)
		goto i2c_error_exit;
	*/
/*//+
	ret = hdmi_rx_write_reg(sii8246, 0x19, 0x07);
	if (ret < 0)
		goto i2c_error_exit;
//+
*/

	/* PLL Calrefsel */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_PLL_CALREFSEL_REG, 0x03);
	if (ret < 0)
		goto i2c_error_exit;

	/* VCO Cal */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_PLL_VCOCAL_REG, 0x20);
	if (ret < 0)
		goto i2c_error_exit;

	/* Auto EQ */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_EQ_DATA0_REG, 0xE0);
	if (ret < 0)
		goto i2c_error_exit;

	/* Auto EQ */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_EQ_DATA1_REG, 0xC0);
	if (ret < 0)
		goto i2c_error_exit;

	/* Auto EQ */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_EQ_DATA2_REG, 0xA0);
	if (ret < 0)
		goto i2c_error_exit;

	/* Auto EQ */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_EQ_DATA3_REG, 0x80);
	if (ret < 0)
		goto i2c_error_exit;

	/* Auto EQ */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_EQ_DATA4_REG, 0x60);
	if (ret < 0)
		goto i2c_error_exit;

	/* Auto EQ */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_EQ_DATA5_REG, 0x40);
	if (ret < 0)
		goto i2c_error_exit;

	/* Auto EQ */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_EQ_DATA6_REG, 0x20);
	if (ret < 0)
		goto i2c_error_exit;

	/* Auto EQ */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_EQ_DATA7_REG, 0x10);
	if (ret < 0)
		goto i2c_error_exit;	

	/* Manual zone */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_TMDS_ZONE_CTRL_REG, 0xD0);
	if (ret < 0)
		goto i2c_error_exit;

	/* PLL Mode Value */
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_TMDS_MODE_CTRL_REG, 0x02);
	if (ret < 0)
		goto i2c_error_exit;

	ret = mhl_tx_write_reg(sii8246, MHL_TX_TMDS_CCTRL, 0x3C);
	if (ret < 0)
		goto i2c_error_exit;

// +
	ret = mhl_tx_write_reg(sii8246, 0x85, 0x00);
	if (ret < 0)
		goto i2c_error_exit;
// +

	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_TMDS_TERMCTRL1_REG, 0x40);
	if (ret < 0)
		goto i2c_error_exit;	

	ret = hdmi_rx_write_reg(sii8246, 0x45, 0x06);
	if (ret < 0)
		goto i2c_error_exit;

//		py.oh	TMDS_CTRL7
	ret = hdmi_rx_write_reg(sii8246, 0x4B, 0x06);
	if (ret < 0)
		goto i2c_error_exit;	

	/* Rx PLL BW ~ 4MHz */
	ret = hdmi_rx_write_reg(sii8246, 0x31, 0x0A);
	if (ret < 0)
		goto i2c_error_exit;

	/* Analog PLL Control
	 * bits 5:4 = 2b00 as per characterization team.
	 */
/*--
	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_TMDS0_CCTRL1_REG, 0xC1);
	if (ret < 0)
		goto i2c_error_exit;
*/
	return ret;

 i2c_error_exit:
	pr_debug("[ERROR] %s()\n", __func__);
	return ret;
}

static int sii8246_mhl_tx_ctl_int(struct sii8246_data *sii8246)
{
	int ret = 0;
	pr_debug("MHL %s\n",__func__);

	ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL1_REG, 0xD0);
	if (ret < 0)
		goto i2c_error_exit;
#ifdef	__CONFIG_RSEN_LOST_PATCH__
	ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL2_REG, 0xBC);
	if (ret < 0)
		goto i2c_error_exit;
#else
	ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL2_REG, 0xFC);
	if (ret < 0)
		goto i2c_error_exit;
#endif
	ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL4_REG,
			       sii8246->pdata->swing_level);
	if (ret < 0)
		goto i2c_error_exit;
	ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL7_REG, 0x03);
	if (ret < 0)
		goto i2c_error_exit;
/*
	ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL8_REG, 0x08);
	if (ret < 0)
		goto i2c_error_exit;
*/
	return ret;

 i2c_error_exit:
	pr_debug("[ERROR] %s()\n", __func__);
	return ret;
}

static void sii8246_power_down(struct sii8246_data *sii8246)
{
	sii8246_disable_irq();
	printk("MHL %s\n",__func__);

//	if (sii8246->claimed) {
//	if (sii8246->pdata->sii8246_muic_cb)
//		sii8246->pdata->sii8246_muic_cb(false, sii8246->vbus_owner);
//	}

	sii8246->state = STATE_DISCONNECTED;
	sii8246->claimed = false;

	tpi_write_reg(sii8246, TPI_DPD_REG, 0);
	/*turn on&off hpd festure for only QCT HDMI */
}

int sii8246_rsen_state_timer_out(struct sii8246_data *sii8246)
{
	int ret = 0;
	u8 value;
	pr_debug("MHL %s\n",__func__);

	ret = mhl_tx_read_reg(sii8246, MHL_TX_SYSSTAT_REG, &value);
	if (ret < 0)
		goto err_exit;
	sii8246->rsen = value & RSEN_STATUS;

	if (value & RSEN_STATUS) {
		pr_debug("sii8246: MHL cable connected.. RSEN High\n");
	} else {
		pr_debug("sii8246: RSEN lost\n");
		msleep(T_SRC_RXSENSE_DEGLITCH);
		ret = mhl_tx_read_reg(sii8246, MHL_TX_SYSSTAT_REG, &value);
		if (ret < 0)
			goto err_exit;

		pr_debug("sii8246: sys_stat: %x ~\n", value);
		if ((value & RSEN_STATUS) == 0) {
			pr_debug("sii8246: RSEN Really LOW ~\n");
			/*To meet CTS 3.3.22.2 spec */
			sii8246_tmds_control(sii8246, false);
			force_usb_id_switch_open(sii8246);
			release_usb_id_switch_open(sii8246);
			ret = -1;
			goto err_exit;
		} else
			pr_debug("sii8246: RSEN recovery\n");

	}
	return ret;

 err_exit:
	/*turn off mhl and change usb_sel to usb */
	pr_debug("sii8246: %s() call sii8246_mhl_onoff_ex(off)\n", __func__);
	schedule_work(&sii8246->mhl_end_work);
	return ret;
}

int sii8246_callback_sched;
static void goto_d3(void)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	int ret;
	u8 value;
	pr_debug("MHL %s\n",__func__);

	pr_debug("sii8246: detection started d3\n");
	sii8246_callback_sched = 0;

	sii8246->mhl_status_value.linkmode = MHL_STATUS_CLK_MODE_NORMAL;
	sii8246->rgnd = RGND_UNKNOWN;

	sii8246->state = NO_MHL_STATUS;

	sii8246->rsen = false;

		sii8246_mhl_hpd_handler_false();

	memset(cbus_pkt_buf, 0x00, sizeof(cbus_pkt_buf));

//	pr_debug("sii8246 : INT Pin Status : %x\n", gpio_get_value(GPIO_MHL_INT));
	
	ret = sii8246_power_init(sii8246);
	if (ret < 0)
		goto exit;
	ret = sii8246_hdmi_init(sii8246);
	if (ret < 0)
		goto exit;
	ret = sii8246_mhl_tx_ctl_int(sii8246);
	if (ret < 0)
		goto exit;

	/* Enable HDCP Compliance safety */
	ret = mhl_tx_write_reg(sii8246, 0x2B, 0x01);
	if (ret < 0)
		goto exit;

	/* CBUS discovery cycle time for each drive and float = 150us */
	ret = mhl_tx_read_reg(sii8246, MHL_TX_DISC_CTRL1_REG, &value);
	if (ret < 0)
		goto exit;

	value &= ~(1 << 3);
	value |= (1 << 2);
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL1_REG, value);
	if (ret < 0)
		goto exit;

	/* Clear bit 6 (reg_skip_rgnd) */
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL2_REG,
			(1 << 7) /* Reserved Bit */ |
			2 << ATT_THRESH_SHIFT | DEGLITCH_TIME_50MS);
	if (ret < 0)
		goto exit;

	/* Changed from 66 to 65 for 94[1:0] = 01 = 5k reg_cbusmhl_pup_sel */
	/* 1.8V CBUS VTH & GND threshold */
	/*To meet CTS 3.3.7.2 spec */
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL5_REG, 0x77);
	if (ret < 0)
		goto exit;

	/* set bit 2 and 3, which is Initiator Timeout */
	ret = cbus_read_reg(sii8246, CBUS_LINK_CONTROL_2_REG, &value);
	if (ret < 0)
		goto exit;

	value |= 0x0C;

	ret = cbus_write_reg(sii8246, CBUS_LINK_CONTROL_2_REG, value);
	if (ret < 0)
		goto exit;

	ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL6_REG, 0xA0);
	if (ret < 0)
		goto exit;

	ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL8_REG, 0x08);
	if (ret < 0)
	  goto exit;

	/* RGND & single discovery attempt (RGND blocking) */
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL6_REG, 0x11/*BLOCK_RGND_INT |
			       DVRFLT_SEL | SINGLE_ATT*/);
	if (ret < 0)
		goto exit;

	/* Use VBUS path of discovery state machine */
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL8_REG, 0);
	if (ret < 0)
		goto exit;

	ret = mhl_tx_write_reg(sii8246, 0x92, 0x86);
	if (ret < 0)
		goto exit;
		ret = mhl_tx_write_reg(sii8246, 0x93, 0x8c);
	if (ret < 0)
		goto exit;
	
	ret = mhl_tx_clear_reg(sii8246, 0x79, (1 << 6)|(1 << 2)|(1 << 1));
	if (ret < 0)
	  goto exit;
	
	ret = mhl_tx_clear_reg(sii8246, 0x79, (1 << 5)); //hpd low... 
	if (ret < 0)
	  goto exit;
	
	ret = mhl_tx_set_reg(sii8246, 0x79, (1 << 4)); //hpd low... 
	if (ret < 0)
	  goto exit;
	
	msleep(25);

	ret = mhl_tx_clear_reg(sii8246, 0x95, (1 << 6)); 
  	if (ret < 0)
   	 goto exit;
  
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL9_REG, 0x34);
	if (ret < 0)
		goto exit;

	 ret = mhl_tx_write_reg(sii8246, 0x90, 0x26);  
  if (ret < 0)
    goto exit;

	ret = sii8246_cbus_reset(sii8246);
  	if (ret < 0)
    	goto exit;
  

  ret = sii8246_cbus_init(sii8246);
  if (ret < 0)
    goto exit;

  ret = mhl_tx_write_reg(sii8246, 0x05, 0x04);  
  if (ret < 0)
    goto exit;

  ret = mhl_tx_write_reg(sii8246, 0x0D, 0x1C);  
  if (ret < 0)
    goto exit;
  
  ///// mhl_init_func ported form daniel code
  
  //interrupt setting start...
  ret = mhl_tx_write_reg(sii8246, 0x75, 0x00);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x76, 0x00);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x77, 0x00);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x78, 0x40);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x63, 0x00);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x64, 0x00);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x65, 0x00);	
  if (ret < 0)
	goto exit;
  
  ret = cbus_write_reg(sii8246, 0x09, 0x00);
  if (ret < 0)
	goto exit;
  
  ret = cbus_write_reg(sii8246, 0x52, 0x00);
  if (ret < 0)
	goto exit;
  
  ret = cbus_write_reg(sii8246, 0x54, 0x00);
  if (ret < 0)
	goto exit;
  
  ret = cbus_write_reg(sii8246, 0x1F, 0x00);
  if (ret < 0)
	goto exit;
  //interrupt setting end ...
  
  force_usb_id_switch_open(sii8246);
  ret = mhl_tx_clear_reg(sii8246, 0x93, (1 << 7)|(1 << 6)|(1 << 5)|(1 << 4)); 
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_clear_reg(sii8246, 0x94, (1 << 1)|(1 << 0)); 
  if (ret < 0)
	goto exit;
  
  //interrupt clear start...
  ret = mhl_tx_write_reg(sii8246, 0x71, 0xFF);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x72, 0xFF);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x73, 0xFF);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x74, 0xFF);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x60, 0xFF);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x61, 0xFF);	
  if (ret < 0)
	goto exit;
  
  ret = mhl_tx_write_reg(sii8246, 0x62, 0xFF);	
  if (ret < 0)
	goto exit;
  
  ret = cbus_write_reg(sii8246, 0x08, 0xFF);
  if (ret < 0)
	goto exit;
  
  ret = cbus_write_reg(sii8246, 0x51, 0xFF);
  if (ret < 0)
	goto exit;
  
  ret = cbus_write_reg(sii8246, 0x53, 0xFF);
  if (ret < 0)
	goto exit;
  
  ret = cbus_write_reg(sii8246, 0x1E, 0xFF);
  if (ret < 0)
	goto exit;
  
  release_usb_id_switch_open(sii8246);

  
  ret = mhl_tx_set_reg(sii8246, 0x79, (1 << 4)); //hpd low... 
  if (ret < 0)
	goto exit;

  ret = hdmi_rx_write_reg(sii8246, 0x01, 0x03);
  if (ret < 0)
	goto exit;
  
  ret = tpi_read_reg(sii8246, 0x3D, &value);  
  
  value &= ~0x01;
  
  ret = tpi_write_reg(sii8246, 0x3D, value);  
  if (ret < 0)
	goto exit;

	pr_debug("sii8246 : go_to d3 mode!!!\n");
//	pr_debug("sii8246 : INT Pin Status : %x\n", gpio_get_value(GPIO_MHL_INT));
	sii8246->state = MHL_READY_RGND_DETECT;

	irq_set_irq_type(sii8246->pdata->mhl_tx_client->irq,
					 IRQ_TYPE_LEVEL_HIGH);
	sii8246_enable_irq();

/*	{
		int i = 0;
		for(i=0; i<50; i++)
		{
			pr_debug("sii8246 : INT Pin Status : %x\n", gpio_get_value(GPIO_MHL_INT));
			msleep(100);
		}
	}
*/

	return;
 exit:
	pr_debug("[ERROR] %s() error terminated!\n", __func__);
	schedule_work(&sii8246->mhl_end_work);
	return;
}

#ifdef __MHL_NEW_CBUS_MSC_CMD__
void sii8246_process_msc_work(struct work_struct *work)
{
	u8 value;
	int ret;
	struct msc_packet *p_msc_pkt, *scratch;
	struct sii8246_data *sii8246 = container_of(work,
						    struct sii8246_data,
						    msc_work);

		if (sii8246->cbus_abort == true) {
			sii8246->cbus_abort = false;
			pr_debug(KERN_INFO "%s() : sii8246->cbus_abort = %d\n", __func__, sii8246->cbus_abort);
			msleep(2000);
		}
	sii8246_cbus_mutex_lock(&sii8246->cbus_lock);
	sii8246_mutex_lock(&sii8246->lock);
	pr_debug("MHL %s\n",__func__);

	pr_debug("%s() - start\n", __func__);

	list_for_each_entry_safe(p_msc_pkt, scratch,
				 &sii8246_g_msc_packet_list, p_msc_packet_list) {
				 

		pr_debug("[MSC] %s() command(0x%x), offset(0x%x), "
				"data_1(0x%x), data_2(0x%x)\n",
				__func__, p_msc_pkt->command, p_msc_pkt->offset,
				p_msc_pkt->data_1, p_msc_pkt->data_2);

		/* msc request */
		ret = sii8246_msc_req_locked(sii8246, p_msc_pkt);
		if (ret < 0) {
			pr_info("%s(): msc_req_locked error %d\n",
				__func__, ret);
			goto exit;
		}

		/* MSC_REQ_DONE received */
		switch (p_msc_pkt->command) {
		case CBUS_MSC_MSG:
			if ((p_msc_pkt->offset == MSG_RCPE) &&
			    (p_msc_pkt->data_2 == 0x01)) {
				sii8246_enqueue_msc_work(sii8246, CBUS_MSC_MSG,
							 MSG_RCPK, MSG_RCPK,
							 0x0);
			}
			break;
		case CBUS_WRITE_STAT:
			pr_debug("sii8246: cbus_command_response"
					"CBUS_WRITE_STAT\n");
			cbus_read_reg(sii8246, CBUS_MSC_FIRST_DATA_IN_REG,
					&p_msc_pkt->data_1);
			break;
		case CBUS_SET_INT:
			if ((p_msc_pkt->offset == MHL_RCHANGE_INT) &&
				(p_msc_pkt->data_1 == MHL_INT_DSCR_CHG)) {
				/* Write burst final step...
				   Req->GRT->Write->DSCR */
				pr_debug("sii8246: MHL_RCHANGE_INT &"
						"MHL_INT_DSCR_CHG\n");
			} else if (p_msc_pkt->offset == MHL_RCHANGE_INT &&
				p_msc_pkt->data_1 == MHL_INT_DCAP_CHG) {
				sii8246_enqueue_msc_work(sii8246,
						CBUS_WRITE_STAT,
						MHL_STATUS_REG_CONNECTED_RDY,
						MHL_STATUS_DCAP_READY, 0x0);
			}
			break;
		case CBUS_WRITE_BURST:
			pr_debug("sii8246: cbus_command_response"
					"MHL_WRITE_BURST\n");
			p_msc_pkt->command = CBUS_IDLE;
			sii8246_enqueue_msc_work(sii8246, CBUS_SET_INT,
					MHL_RCHANGE_INT, MHL_INT_DSCR_CHG, 0x0);
			break;
		case CBUS_READ_DEVCAP:
			ret = cbus_read_reg(sii8246,
					    CBUS_MSC_FIRST_DATA_IN_REG, &value);
			if (ret < 0)
				break;
			switch (p_msc_pkt->offset) {
			case DEVCAP_DEV_STATE:
				pr_debug("sii8246: DEVCAP_DEV_STATE\n");
				break;
			case DEVCAP_MHL_VERSION:
				sii8246->devcap.mhl_ver = value;
				pr_debug("sii8246: MHL_VERSION: %X\n", value);
				break;
			case DEVCAP_DEV_CAT:
				sii8246->plim = (value >> 5) & 0x03;
				pr_debug("sii8246: PLIM : %d\n", sii8246->plim);
				if (value & MHL_DEV_CATEGORY_POW_BIT)
					pr_debug("sii8246: CAT=POWERED");
				else
					pr_debug("sii8246: CAT=UNPOWERED");
				break;
			case DEVCAP_ADOPTER_ID_H:
				sii8246->devcap.adopter_id =
				    (value & 0xFF) << 0x8;
				pr_debug("sii8246: DEVCAP_ADOPTER_ID_H = %X\n",
					 value);
				break;
			case DEVCAP_ADOPTER_ID_L:
				sii8246->devcap.adopter_id |= value & 0xFF;
				pr_debug("sii8246: DEVCAP_ADOPTER_ID_L = %X\n",
					 value);
				break;
			case DEVCAP_VID_LINK_MODE:
				sii8246->devcap.vid_link_mode = 0x3F & value;
				pr_debug
				    ("sii8246: MHL_CAP_VID_LINK_MODE = %d\n",
				     sii8246->devcap.vid_link_mode);
				break;
			case DEVCAP_AUD_LINK_MODE:
				sii8246->devcap.aud_link_mode = 0x03 & value;
				pr_debug("sii8246: DEVCAP_AUD_LINK_MODE =%d\n",
					 sii8246->devcap.aud_link_mode);
				break;
			case DEVCAP_VIDEO_TYPE:
				sii8246->devcap.video_type = 0x8F & value;
				pr_debug("sii8246: DEVCAP_VIDEO_TYPE =%d\n",
					 sii8246->devcap.video_type);
				break;
			case DEVCAP_LOG_DEV_MAP:
				sii8246->devcap.log_dev_map = value;
				pr_debug("sii8246: DEVCAP_LOG_DEV_MAP =%d\n",
					 sii8246->devcap.log_dev_map);
				break;
			case DEVCAP_BANDWIDTH:
				sii8246->devcap.bandwidth = value;
				pr_debug("sii8246: DEVCAP_BANDWIDTH =%d\n",
					 sii8246->devcap.bandwidth);
				break;
			case DEVCAP_DEV_FEATURE_FLAG:
				if ((value & MHL_FEATURE_RCP_SUPPORT) == 0)
					pr_debug("sii8246: FEATURE_FLAG=RCP");

				if ((value & MHL_FEATURE_RAP_SUPPORT) == 0)
					pr_debug("sii8246: FEATURE_FLAG=RAP\n");

				if ((value & MHL_FEATURE_SP_SUPPORT) == 0)
					pr_debug("sii8246: FEATURE_FLAG=SP\n");
				break;
			case DEVCAP_DEVICE_ID_H:
				sii8246->devcap.device_id =
				    (value & 0xFF) << 0x8;
				pr_debug("sii8246: DEVICE_ID_H=0x%x\n", value);
				break;
			case DEVCAP_DEVICE_ID_L:
				sii8246->devcap.device_id |= value & 0xFF;
				pr_debug("sii8246: DEVICE_ID_L=0x%x\n", value);
				break;
			case DEVCAP_SCRATCHPAD_SIZE:
				sii8246->devcap.scratchpad_size = value;
				pr_debug
				    ("sii8246: DEVCAP_SCRATCHPAD_SIZE =%d\n",
				     sii8246->devcap.scratchpad_size);
				break;
			case DEVCAP_INT_STAT_SIZE:
				sii8246->devcap.int_stat_size = value;
				pr_debug("sii8246: DEVCAP_INT_STAT_SIZE =%d\n",
					 sii8246->devcap.int_stat_size);
				break;
			case DEVCAP_RESERVED:
				sii8246->dcap_ready_status = 1;
				sii8246->devcap.reserved_data = value;
				pr_debug("sii8246: DEVCAP_RESERVED : %d\n",
					value);
				wake_up(&sii8246->wq);
				break;
			default:
				pr_debug("sii8246: DEVCAP DEFAULT\n");
				break;
			}
			break;
		default:
			break;
		}

		list_del(&p_msc_pkt->p_msc_packet_list);
		pr_debug("[MSC] %s() free item , addr = 0x%x, cnt=%d\n",
			 __func__, (unsigned int)p_msc_pkt, --g_list_cnt);
		kfree(p_msc_pkt);
	}
 exit:
	sii8246_mutex_unlock(&sii8246->lock);
	sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
}

static int sii8246_enqueue_msc_work(struct sii8246_data *sii8246, u8 command,
				    u8 offset, u8 data_1, u8 data_2)
{
	struct msc_packet *packet_item;

	packet_item = kmalloc(sizeof(struct msc_packet), GFP_KERNEL);
	pr_debug("MHL %s\n",__func__);
	if (!packet_item) {
		pr_debug("[ERROR] %s() kmalloc error\n", __func__);
		return -ENOMEM;
	} else
		pr_debug("[MSC] %s() add item, addr = 0x%x, cnt=%d\n",
			 __func__, (unsigned int)packet_item, ++g_list_cnt);

	packet_item->command = command;
	packet_item->offset = offset;
	packet_item->data_1 = data_1;
	packet_item->data_2 = data_2;

	pr_debug("[MSC] %s() command(0x%x), offset(0x%x), data_1(0x%x), "
		"data_2(0x%x)\n", __func__, command, offset, data_1, data_2);
	list_add_tail(&packet_item->p_msc_packet_list, &sii8246_g_msc_packet_list);

	pr_debug("[MSC] %s() msc work schedule\n", __func__);
	queue_work(sii8246_msc_wq, &(sii8246->msc_work));

	return 0;
}

/* Must call with sii8246->lock held */
static int sii8246_msc_req_locked(struct sii8246_data *sii8246,
				  struct msc_packet *msc_pkt)
{
	int ret;
	u8 start_command;
	pr_debug("MHL %s\n",__func__);

	if (sii8246->state != STATE_ESTABLISHED)
		return -ENOENT;

	init_completion(&sii8246->msc_complete);

	cbus_write_reg(sii8246, CBUS_MSC_OFFSET_REG, msc_pkt->offset);
	if (msc_pkt->command == CBUS_MSC_MSG)
		msc_pkt->data_1 = msc_pkt->offset;
	cbus_write_reg(sii8246, CBUS_MSC_FIRST_DATA_OUT_REG, msc_pkt->data_1);

	switch (msc_pkt->command) {
	case CBUS_SET_INT:
	case CBUS_WRITE_STAT:
		start_command = START_BIT_WRITE_STAT_INT;
		break;
	case CBUS_MSC_MSG:
		cbus_write_reg(sii8246, CBUS_MSC_SECOND_DATA_OUT_REG,
			       msc_pkt->data_2);
		cbus_write_reg(sii8246, CBUS_MSC_OFFSET_REG, msc_pkt->command);

		start_command = START_BIT_MSC_MSG;
		break;
	case CBUS_READ_DEVCAP:
		start_command = START_BIT_READ_DEVCAP;
		break;
	case CBUS_WRITE_BURST:
		start_command = START_BIT_WRITE_BURST;
		break;
	case CBUS_GET_STATE:
	case CBUS_GET_VENDOR_ID:
	case CBUS_SET_HPD:
	case CBUS_CLR_HPD:
	case CBUS_GET_MSC_ERR_CODE:
	case CBUS_GET_SC3_ERR_CODE:
	case CBUS_GET_SC1_ERR_CODE:
	case CBUS_GET_DDC_ERR_CODE:
		cbus_write_reg(sii8246, CBUS_MSC_OFFSET_REG, msc_pkt->command);
		start_command = START_BIT_MSC_RESERVED;
		break;
	default:
		pr_debug("[ERROR] %s() invalid msc command(%d)\n",
		       __func__, msc_pkt->command);
		return -EINVAL;
	}

	cbus_write_reg(sii8246, CBUS_MSC_COMMAND_START_REG, start_command);

	sii8246_mutex_unlock(&sii8246->lock);
	ret = wait_for_completion_timeout(&sii8246->msc_complete,
					  msecs_to_jiffies(300));
	if (ret == 0)
		printk(KERN_ERR "[ERROR] %s() MSC_REQ_DONE timeout\n",
		       __func__);

	sii8246_mutex_lock(&sii8246->lock);

	return ret ? 0 : -EIO;
}
#endif				/* __MHL_NEW_CBUS_MSC_CMD__ end */

static void sii8246_detection_callback_worker(struct work_struct *p)
{
	pr_debug("%s()\n", __func__);
	sii8246_detection_callback();
	return;
}

static void mhl_start_worker(struct work_struct *p)
{
	pr_debug("%s\n",__func__);
	sii8246_mhl_onoff_ex(1);
	return;
}

static void mhl_end_worker(struct work_struct *p)
{
	pr_debug("%s()\n", __func__);
	sii8246_mhl_onoff_ex(0);
	return;
}

static void mhl_goto_d3_worker(struct work_struct *p)
{
	pr_debug("MHL %s\n",__func__);
	goto_d3();
	return;
}

static void mhl_cbus_write_stat_worker(struct work_struct *p)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	pr_debug("MHL %s\n",__func__);
#ifndef __MHL_NEW_CBUS_MSC_CMD__

	cbus_command_request(sii8246, CBUS_WRITE_STAT, CBUS_LINK_CONTROL_2_REG,
			     sii8246->mhl_status_value.linkmode);
#else
	sii8246_enqueue_msc_work(sii8246, CBUS_WRITE_STAT,
				 CBUS_LINK_CONTROL_2_REG,
				 sii8246->mhl_status_value.linkmode, 0x0);
#endif
	return;
}

static int sii8246_detection_callback(void)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	int ret;
	u8 value;
	int handled = 0;
	pr_debug("MHL %s\n",__func__);
	sii8246_callback_sched = 1;

	sii8246_mutex_lock(&sii8246->lock);
	sii8246->mhl_status_value.linkmode = MHL_STATUS_CLK_MODE_NORMAL;
	sii8246->rgnd = RGND_UNKNOWN;
	sii8246->state = STATE_DISCONNECTED;
	sii8246->rsen = false;
#ifdef __MHL_NEW_CBUS_MSC_CMD__
	sii8246->dcap_ready_status = 0;
#endif
	memset(cbus_pkt_buf, 0x00, sizeof(cbus_pkt_buf));
	ret = sii8246_power_init(sii8246);
	if (ret < 0) {
		pr_debug("[ERROR] %s() - sii8246_power_init\n", __func__);
		goto unhandled;
	}

	ret = sii8246_cbus_reset(sii8246);
	if (ret < 0) {
		pr_debug("[ERROR] %s() - sii8246_cbus_reset\n", __func__);
		goto unhandled;
	}

	ret = hdmi_rx_write_reg(sii8246, HDMI_RX_TMDS0_CCTRL1_REG, 0xC1);
	if (ret < 0)
		goto unhandled;
	ret = hdmi_rx_write_reg(sii8246, 0x19, 0x07);
	if (ret < 0)
		goto unhandled;
	
	ret = sii8246_hdmi_init(sii8246);
	if (ret < 0) {
		pr_err("[ERROR] %s() - sii8246_hdmi_init\n", __func__);
		goto unhandled;
	}

	ret = sii8246_mhl_tx_ctl_int(sii8246);
	if (ret < 0) {
		pr_err("[ERROR] %s() - sii8246_mhl_tx_ctl_int\n", __func__);
		goto unhandled;
	}

	/* Enable HDCP Compliance safety */
	ret = mhl_tx_write_reg(sii8246, 0x2B, 0x01);
	if (ret < 0) {
		pr_err("[ERROR] %s() - mhl_tx_write_reg 0x2B\n", __func__);
		goto unhandled;
	}
#if 1
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL1_REG, 0x27);
	if (ret < 0) {
		pr_err("[ERROR] %s() - mhl_tx_write_reg MHL_TX_DISC_CTRL1_REG\n", __func__);
		goto unhandled;
	}
#else
	ret = mhl_tx_clear_reg(sii8246, MHL_TX_DISC_CTRL1_REG, (1 << 3));
	if (ret < 0) {
		pr_err("[ERROR] %s() - MHL_TX_DISC_CTRL1_REG\n", __func__);
		goto unhandled;
	}
	
	ret = mhl_tx_set_reg(sii8246, MHL_TX_DISC_CTRL1_REG, (1 << 2));
	if (ret < 0) {
		pr_err("[ERROR] %s() - MHL_TX_DISC_CTRL1_REG\n", __func__);
		goto unhandled;
	}
#endif
	/* Clear bit 6 (reg_skip_rgnd) */
	ret = mhl_tx_write_reg(sii8246,
			MHL_TX_DISC_CTRL2_REG, (1 << 7) /* Reserved Bit */ |
			2 << ATT_THRESH_SHIFT | DEGLITCH_TIME_50MS);
	if (ret < 0) {
		pr_err("[ERROR] %s() - Clear bit 6\n", __func__);
		goto unhandled;
	}

	/* Changed from 66 to 65 for 94[1:0] = 01 = 5k reg_cbusmhl_pup_sel */
	/* 1.8V CBUS VTH & GND threshold */
	/*To meet CTS 3.3.7.2 spec */
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL5_REG, 0x77);
	if (ret < 0) {
		pr_err("[ERROR] %s() - MHL_TX_DISC_CTRL5_REG\n", __func__);
		goto unhandled;
	}

	/* set bit 2 and 3, which is Initiator Timeout */
	ret = cbus_read_reg(sii8246, CBUS_LINK_CONTROL_2_REG, &value);
	if (ret < 0) {
		pr_err("[ERROR] %s() - read CBUS_LINK_CONTROL_2_REG\n",
		       __func__);
		goto unhandled;
	}

	value |= 0x0C;

	ret = cbus_write_reg(sii8246, CBUS_LINK_CONTROL_2_REG, value);
	if (ret < 0) {
		pr_err("[ERROR] %s() - write CBUS_LINK_CONTROL_2_REG\n",
		       __func__);
		goto unhandled;
	}

	ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL6_REG, 0xBC);
	if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_MHLTX_CTL6_REG\n",
		       __func__);
		goto unhandled;
	}

	
	ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL8_REG, 0x08);
	if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_MHLTX_CTL8_REG\n",
		       __func__);
		goto unhandled;
	}
	
	/* RGND & single discovery attempt (RGND blocking) */
    ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL6_REG, 
    			USB_ID_OVR | DVRFLT_SEL | BLOCK_RGND_INT| SINGLE_ATT);  
	if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_DISC_CTRL6_REG\n",
		       __func__);
		goto unhandled;
	}
	/* Use VBUS path of discovery state machine */
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL8_REG, 0);
	if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_DISC_CTRL8_REG\n",
		       __func__);
		goto unhandled;
	}

	
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL3_REG, 0x86);
	if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_DISC_CTRL3_REG\n",
		       __func__);
		goto unhandled;
	}

	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL4_REG, 0x8C);
	if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_DISC_CTRL4_REG\n",
		       __func__);
		goto unhandled;
	}

	ret = mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 6)|(1 << 2)|(1 << 1));
	 if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_INT_CTRL_REG\n",
		       __func__);
		goto unhandled;
	}
	
	 ret = mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 5)); //hpd low... 
	 if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_INT_CTRL_REG\n",
		       __func__);
		goto unhandled;
	}
	
	 ret = mhl_tx_set_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 4)); //hpd low... 
	 if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_INT_CTRL_REG\n",
		       __func__);
		goto unhandled;
	}
	
	 msleep(25);
	 

	/* 0x92[3] sets the CBUS / ID switch */
	ret = mhl_tx_set_reg(sii8246, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);
	if (ret < 0) {
		pr_err("[ERROR] %s() - set MHL_TX_DISC_CTRL6_REG\n", __func__);
		goto unhandled;
	}
	
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL9_REG, (0x20|0x10|0x04));  
	if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_DISC_CTRL9_REG\n",
		       __func__);
		goto unhandled;
	}
	
	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL1_REG, 0x27);  
	if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_DISC_CTRL1_REG\n",
		       __func__);
		goto unhandled;
	}
	
	ret = sii8246_cbus_init(sii8246);
	if (ret < 0) {
		pr_err("[ERROR] %s() - write sii8246_cbus_init\n",
		       __func__);
		goto unhandled;
	}

	
	ret = mhl_tx_write_reg(sii8246, 0x05, 0x04);  
	if (ret < 0) {
		pr_err("[ERROR] %s() - write 0x05\n",
		       __func__);
		goto unhandled;
	}
	ret = mhl_tx_write_reg(sii8246, 0x0D, 0x1C);  
	if (ret < 0) {
		pr_err("[ERROR] %s() - write 0x0D\n",
		       __func__);
		goto unhandled;
	}


	ret = mhl_tx_write_reg(sii8246, 0x90, 0x25);  
	if (ret < 0) {
		pr_err("[ERROR] %s() - write 0x90\n",
		       __func__);
		goto unhandled;
	}

	ret = mhl_tx_write_reg(sii8246, 0x74, 0x42);  
	if (ret < 0) {
		pr_err("[ERROR] %s() - write 0x74\n",
		       __func__);
		goto unhandled;
	}

	ret = mhl_tx_write_reg(sii8246, 0x99, 0x02);  
	if (ret < 0) {
		pr_err("[ERROR] %s() - write 0x99\n",
		       __func__);
		goto unhandled;
	}
	/*
	ret = mhl_tx_clear_reg(sii8246, MHL_TX_DISC_CTRL6_REG, (1 << 1)); 
	if (ret < 0) {
		pr_err("[ERROR] %s() - write MHL_TX_DISC_CTRL6_REG\n",
		       __func__);
		goto unhandled;
	}
*/
	/* To allow RGND engine to operate correctly.
	 * When moving the chip from D2 to D0 (power up, init regs)
	 * the values should be
	 * 94[1:0] = 01  reg_cbusmhl_pup_sel[1:0] should be set for 5k
	 * 93[7:6] = 10  reg_cbusdisc_pup_sel[1:0] should be
	 * set for 10k (default)
	 * 93[5:4] = 00  reg_cbusidle_pup_sel[1:0] = open (default)
	 */
 #if 0
	ret = mhl_tx_set_reg(sii8246, MHL_TX_DISC_CTRL3_REG, 0x86);
	if (ret < 0) {
		pr_err("[ERROR] %s() - set MHL_TX_DISC_CTRL3_REG\n", __func__);
		goto unhandled;
	}

	/* change from CC to 8C to match 5K */
	/*To meet CTS 3.3.72 spec */
	ret = mhl_tx_set_reg(sii8246, MHL_TX_DISC_CTRL4_REG, 0x8C);
	if (ret < 0) {
		pr_err("[ERROR] %s() - set MHL_TX_DISC_CTRL4_REG\n", __func__);
		goto unhandled;
	}
	/* Force upstream HPD to 0 when not in MHL mode */
	ret = mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 5));
	if (ret < 0) {
		pr_err("[ERROR] %s() - clear MHL_TX_INT_CTRL_REG\n", __func__);
		goto unhandled;
	}
	ret = mhl_tx_set_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 4));
	if (ret < 0) {
		pr_err("[ERROR] %s() - set MHL_TX_INT_CTRL_REG\n", __func__);
		goto unhandled;
	}

	/* Configure the interrupt as active high */
	ret =
	    mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG, (1 << 6) | (1 << 2) | (1 << 1));
	if (ret < 0) {
		pr_err("[ERROR] %s() - clear MHL_TX_INT_CTRL_REG\n", __func__);
		goto unhandled;
	}

	msleep(25);

	/* release usb_id switch : HW decides the switch setting */
	ret = mhl_tx_clear_reg(sii8246, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);
	if (ret < 0) {
		pr_err("[ERROR] %s() - clear MHL_TX_DISC_CTRL6_REG\n",
		       __func__);
		goto unhandled;
	}

	ret = sii8246_cbus_init(sii8246);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL9_REG, 0x34);
		if (ret < 0) {
			pr_err("[ERROR] %s() - write MHL_TX_DISC_CTRL9_REG\n",
				   __func__);
			goto unhandled;
		}

	ret = mhl_tx_write_reg(sii8246, MHL_TX_DISC_CTRL1_REG, 0x27);
	if (ret < 0)
		goto unhandled;

	/* Enable Auto soft reset on SCDT = 0 */
	ret = mhl_tx_write_reg(sii8246, 0x05, 0x04);
	if (ret < 0)
		goto unhandled;

	/* HDMI Transcode mode enable */
	ret = mhl_tx_write_reg(sii8246, 0x0D, 0x1C);
	if (ret < 0)
		goto unhandled;

#ifdef __CONFIG_TMDS_OFFON_WORKAROUND__
	ret = mhl_tx_write_reg(sii8246, MHL_TX_INTR4_ENABLE_REG,
			       RGND_READY_MASK | CBUS_LKOUT_MASK |
			       MHL_DISC_FAIL_MASK | MHL_EST_MASK | (1 << 0));
#else
	ret = mhl_tx_write_reg(sii8246, MHL_TX_INTR4_ENABLE_REG,
			       RGND_READY_MASK | CBUS_LKOUT_MASK |
			       MHL_DISC_FAIL_MASK | MHL_EST_MASK);
#endif
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii8246, MHL_TX_INTR1_ENABLE_REG,
			       (1 << 5) | (1 << 6));
	if (ret < 0)
		goto unhandled;
/* this point is very importand before megsure RGND impedance*/
	force_usb_id_switch_open(sii8246);
	ret = mhl_tx_clear_reg(sii8246, MHL_TX_DISC_CTRL4_REG,
			       (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4));
	if (ret < 0)
		goto unhandled;
	ret =
	    mhl_tx_clear_reg(sii8246, MHL_TX_DISC_CTRL5_REG,
			     (1 << 1) | (1 << 0));
	if (ret < 0)
		goto unhandled;
	release_usb_id_switch_open(sii8246);
	
/*end of this*/
#endif
	pr_debug("sii8246: waiting for RGND measurement\n");

	irq_set_irq_type(sii8246->pdata->mhl_tx_client->irq,
					IRQ_TYPE_LEVEL_HIGH);
	sii8246_enable_irq();

	/* SiI8246 Programmer's Reference Section 2.4.3
	 * State : RGND Ready
	 */
	sii8246_mutex_unlock(&sii8246->lock);

	pr_debug("sii8246: waiting for detection\n");
	ret = wait_event_timeout(sii8246->wq,
				 sii8246->state != STATE_DISCONNECTED,
				 msecs_to_jiffies(T_WAIT_TIMEOUT_DISC_INT * 2));
	sii8246_mutex_lock(&sii8246->lock);
	if (ret == 0) {
		pr_err("[ERROR] %s() - wait detection\n", __func__);
		goto unhandled;
	}

	if (sii8246->state == STATE_DISCOVERY_FAILED) {
		handled = -1;
		pr_err("[ERROR] %s() - state == STATE_DISCOVERY_FAILED\n",
		       __func__);
		goto unhandled;
	}

	if (sii8246->state == NO_MHL_STATUS) {
		handled = -1;
		pr_err("[ERROR] %s() - state == NO_MHL_STATUS\n", __func__);
		goto unhandled;
	}

	if (mhl_state_is_error(sii8246->state)) {
		pr_err("[ERROR] %s() -mhl_state_is_error\n", __func__);
		goto unhandled;
	}

	sii8246_mutex_unlock(&sii8246->lock);
	pr_info("sii8246: Established & start to moniter RSEN\n");
	/*CTS 3.3.14.3 Discovery;Sink Never Drives MHL+/- HIGH */
	/*MHL SPEC 8.2.1.1.1;Transition SRC4-SRC7 */
	if (sii8246_rsen_state_timer_out(sii8246) < 0)
		return handled;

	sii8246->claimed = true;

	ret = cbus_write_reg(sii8246,
			     CBUS_INTR1_ENABLE_REG,
			     MSC_RESP_ABORT_MASK |
			     MSC_REQ_ABORT_MASK |
			     MSC_REQ_DONE_MASK |
			     MSC_MSG_RECD_MASK | CBUS_DDC_ABORT_MASK);
	if (ret < 0)
		goto unhandled_nolock;

	ret = cbus_write_reg(sii8246,
			     CBUS_INTR2_ENABLE_REG,
			     WRT_STAT_RECD_MASK | SET_INT_RECD_MASK);
	if (ret < 0)
		goto unhandled_nolock;

#ifdef __MHL_NEW_CBUS_MSC_CMD__
	ret = wait_event_timeout(sii8246->wq,
				 sii8246->dcap_ready_status,
				 msecs_to_jiffies(500));
	if (ret == 0) {
		pr_debug("dcap_timeout err, dcap_staus:%d\n",
			 sii8246->dcap_ready_status);
		pr_debug("vbus_owner = USB\n");
		sii8246->vbus_owner = MHL_VBUS_USB;
	} else {
		switch (sii8246->devcap.mhl_ver & 0xf0) {
		case 0x10:
			pr_debug("%s() MHL dongle ver 1.0 ", __func__);
			/*SAMSUNG DEVICE_ID 0x1134:dongle, 0x1234:dock */
			if (sii8246->devcap.device_id == SS_MHL_DONGLE_DEV_ID ||
			  sii8246->devcap.device_id == SS_MHL_DOCK_DEV_ID) {
				switch (sii8246->devcap.reserved_data) {
				case 0: /* UNKNOWN */
				case 1: /* USB */
					pr_cont("vbus_owner = USB\n");
					sii8246->vbus_owner = MHL_VBUS_USB;
					break;
				case 2: /* TA */
					pr_cont("vbus_owner = TA 900mA\n");
					sii8246->vbus_owner = MHL_VBUS_TA_900mA;
					break;
				default:
					pr_cont("vbus_owner = USB\n");
					sii8246->vbus_owner = MHL_VBUS_USB;
					break;
				}
			} else {
				pr_cont("vbus_owner = USB\n");
				sii8246->vbus_owner = MHL_VBUS_USB;/* UNKNOWN */
			}
			break;
		case 0x20:
			pr_debug("%s() MHL dongle ver 2.0 ", __func__);
			pr_cont("vbus_owner = PLIM\n");
			sii8246->vbus_owner = sii8246->plim;
			break;
		default:
			pr_debug("%s() MHL dongle unknown version ", __func__);
			pr_cont("vbus_owner = USB\n");
			sii8246->vbus_owner = MHL_VBUS_USB; /* UNKNOWN */
			break;
		}
	}
	pr_debug("device_id:0x%4x, vbus_owner:%d\n",
		 sii8246->devcap.device_id, sii8246->vbus_owner);
	/*send some data for VBUS SRC such a TA or USB or UNKNOWN */
	if (sii8246->pdata->sii8246_muic_cb)
		sii8246->pdata->sii8246_muic_cb(false, sii8246->vbus_owner);
#endif

	return handled;

 unhandled:
	pr_info("sii8246: Detection failed");
	if (sii8246->state == STATE_DISCONNECTED) {
		pr_cont(" (timeout)");
		schedule_work(&sii8246->mhl_end_work);
	} else if (sii8246->state == STATE_DISCOVERY_FAILED)
		pr_cont(" (discovery failed)");
	else if (sii8246->state == NO_MHL_STATUS)
		pr_cont(" (already went to D3)");
	else if (sii8246->state == STATE_CBUS_LOCKOUT)
		pr_cont(" (cbus_lockout)");
	pr_cont("\n");

	/*mhl spec: 8.3.3, if discovery failed, must retry discovering */
#ifdef	CONFIG_SAMSUNG_USE_11PIN_CONNECTOR
	if ((sii8246->state == STATE_DISCOVERY_FAILED) &&
	    (sii8246->rgnd == RGND_1K)) {
#else
	if ((sii8246->state == STATE_DISCOVERY_FAILED)) {
#endif
		pr_cont("Discovery failed but RGND_1K impedence"
			" restart detection_callback");

		schedule_work(&sii8246->mhl_end_work);
	}

	sii8246_mutex_unlock(&sii8246->lock);

	return handled;

 unhandled_nolock:
	pr_info("sii8246: Detection failed");
	if (sii8246->state == STATE_DISCONNECTED) {
		pr_cont(" (timeout)");
		schedule_work(&sii8246->mhl_end_work);
	} else if (sii8246->state == STATE_DISCOVERY_FAILED)
		pr_cont(" (discovery failed)");
	else if (sii8246->state == NO_MHL_STATUS)
		pr_cont(" (already went to D3)");
	else if (sii8246->state == STATE_CBUS_LOCKOUT)
		pr_cont(" (cbus_lockout)");
	pr_cont("\n");

	/*mhl spec: 8.3.3, if discovery failed, must retry discovering */
#ifdef	CONFIG_SAMSUNG_USE_11PIN_CONNECTOR
	if ((sii8246->state == STATE_DISCOVERY_FAILED) &&
	    (sii8246->rgnd == RGND_1K)) {
#else
	if ((sii8246->state == STATE_DISCOVERY_FAILED)) {
#endif
		pr_cont("Discovery failed but RGND_1K impedence"
			" restart detection_callback");

		schedule_work(&sii8246->mhl_end_work);
	}

	return handled;
}

static void sii8246_cancel_callback(void)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	pr_debug("MHL %s\n",__func__);

	sii8246_mutex_lock(&sii8246->lock);
	sii8246_power_down(sii8246);
	sii8246_mutex_unlock(&sii8246->lock);
}


static void save_cbus_pkt_to_buffer(struct sii8246_data *sii8246)
{
	int index;
	pr_debug("MHL %s\n",__func__);

	for (index = 0; index < CBUS_PKT_BUF_COUNT; index++)
		if (sii8246->cbus_pkt_buf[index].status == false)
			break;

	if (index == CBUS_PKT_BUF_COUNT) {
		pr_debug("sii8246: Error save_cbus_pkt Buffer Full\n");
		index -= 1;	/*adjust index */
	}

	pr_debug("sii8246: save_cbus_pkt_to_buffer index = %d\n", index);
	memcpy(&sii8246->cbus_pkt_buf[index], &sii8246->cbus_pkt,
	       sizeof(struct cbus_packet));
	sii8246->cbus_pkt_buf[index].status = true;
}

static void cbus_command_response(struct sii8246_data *sii8246)
{
	u8 value, offset = 0;

	sii8246_cbus_mutex_lock(&sii8246->cbus_lock);
	pr_debug("sii8246: cbus_command_response\n");

	switch (sii8246->cbus_pkt.command) {
	case CBUS_MSC_MSG:
		pr_debug("sii8246: cbus_command_response Received"
			 " ACK for CBUS_MSC_MSG\n");
#ifdef CONFIG_SII8246_RCP
		if (sii8246->cbus_pkt.data[0] == MSG_RCPE &&
		    sii8246->cbus_pkt.data[1] == 0x01) {
			sii8246->cbus_pkt.command = CBUS_IDLE;
			sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
			cbus_command_request(sii8246, CBUS_MSC_MSG, MSG_RCPK,
					     sii8246->error_key);
			return;
		}
#endif
		break;
	case CBUS_WRITE_STAT:
		pr_debug("sii8246: cbus_command_response" "CBUS_WRITE_STAT\n");
		cbus_read_reg(sii8246, CBUS_MSC_FIRST_DATA_IN_REG,
			      &sii8246->cbus_pkt.data[0]);
		break;
	case CBUS_SET_INT:
		pr_debug("sii8246: cbus_command_response CBUS_SET_INT\n");
		if (sii8246->cbus_pkt.offset == MHL_RCHANGE_INT &&
		    sii8246->cbus_pkt.data[0] == MHL_INT_DSCR_CHG) {
			/*Write burst final step... Req->GRT->Write->DSCR */
			pr_debug("sii8246: MHL_RCHANGE_INT &"
				 "MHL_INT_DSCR_CHG\n");
		} else if (sii8246->cbus_pkt.offset == MHL_RCHANGE_INT &&
			   sii8246->cbus_pkt.data[0] == MHL_INT_DCAP_CHG) {
			pr_debug("sii8246: MHL_RCHANGE_INT &"
				 "MHL_INT_DCAP_CHG\n");
			sii8246->cbus_pkt.command = CBUS_IDLE;
			sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
			cbus_command_request(sii8246, CBUS_WRITE_STAT,
					     MHL_STATUS_REG_CONNECTED_RDY,
					     MHL_STATUS_DCAP_READY);
			return;
		}
		break;
	case CBUS_WRITE_BURST:
		pr_debug("sii8246: cbus_command_response" "MHL_WRITE_BURST\n");
		sii8246->cbus_pkt.command = CBUS_IDLE;
		sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
		cbus_command_request(sii8246, CBUS_SET_INT,
				     MHL_RCHANGE_INT, MHL_INT_DSCR_CHG);
		return;
	case CBUS_READ_DEVCAP:
		pr_debug("sii8246: cbus_command_response"
			 " CBUS_READ_DEVCAP\n");
		cbus_read_reg(sii8246, CBUS_MSC_FIRST_DATA_IN_REG, &value);
		switch (sii8246->cbus_pkt.offset) {
		case DEVCAP_MHL_VERSION:
			sii8246->devcap.mhl_ver = value;
			pr_debug("sii8246: MHL_VERSION: %X\n", value);
			break;
		case DEVCAP_DEV_CAT:
			if (value & MHL_DEV_CATEGORY_POW_BIT)
				pr_debug("sii8246: CAT=POWERED");
			else
				pr_debug("sii8246: CAT=UNPOWERED");
			break;
		case DEVCAP_ADOPTER_ID_H:
			sii8246->devcap.adopter_id = (value & 0xFF) << 0x8;
			pr_debug("sii8246: DEVCAP_ADOPTER_ID_H = %X\n", value);
			break;
		case DEVCAP_ADOPTER_ID_L:
			sii8246->devcap.adopter_id |= value & 0xFF;
			pr_debug("sii8246: DEVCAP_ADOPTER_ID_L = %X\n", value);
			break;
		case DEVCAP_VID_LINK_MODE:
			sii8246->devcap.vid_link_mode = 0x3F & value;
			pr_debug("sii8246: MHL_CAP_VID_LINK_MODE = %d\n",
				 sii8246->devcap.vid_link_mode);
			break;
		case DEVCAP_AUD_LINK_MODE:
			sii8246->devcap.aud_link_mode = 0x03 & value;
			pr_debug("sii8246: DEVCAP_AUD_LINK_MODE =%d\n",
				 sii8246->devcap.aud_link_mode);
			break;
		case DEVCAP_VIDEO_TYPE:
			sii8246->devcap.video_type = 0x8F & value;
			pr_debug("sii8246: DEVCAP_VIDEO_TYPE =%d\n",
				 sii8246->devcap.video_type);
			break;
		case DEVCAP_LOG_DEV_MAP:
			sii8246->devcap.log_dev_map = value;
			pr_debug("sii8246: DEVCAP_LOG_DEV_MAP =%d\n",
				 sii8246->devcap.log_dev_map);
			break;
		case DEVCAP_BANDWIDTH:
			sii8246->devcap.bandwidth = value;
			pr_debug("sii8246: DEVCAP_BANDWIDTH =%d\n",
				 sii8246->devcap.bandwidth);
			break;
		case DEVCAP_DEV_FEATURE_FLAG:
			if ((value & MHL_FEATURE_RCP_SUPPORT) == 0)
				pr_debug("sii8246: FEATURE_FLAG=RCP");

			if ((value & MHL_FEATURE_RAP_SUPPORT) == 0)
				pr_debug("sii8246: FEATURE_FLAG=RAP\n");

			if ((value & MHL_FEATURE_SP_SUPPORT) == 0)
				pr_debug("sii8246: FEATURE_FLAG=SP\n");
			break;
		case DEVCAP_DEVICE_ID_H:
			sii8246->devcap.device_id = (value & 0xFF) << 0x8;
			pr_info("sii8246: DEVICE_ID_H=0x%x\n", value);
			offset = DEVCAP_DEVICE_ID_L;
			break;
		case DEVCAP_DEVICE_ID_L:
			sii8246->devcap.device_id |= value & 0xFF;
			pr_info("sii8246: DEVICE_ID_L=0x%x\n", value);
			break;
		case DEVCAP_SCRATCHPAD_SIZE:
			sii8246->devcap.scratchpad_size = value;
			pr_debug("sii8246: DEVCAP_SCRATCHPAD_SIZE =%d\n",
				 sii8246->devcap.scratchpad_size);
			break;
		case DEVCAP_INT_STAT_SIZE:
			sii8246->devcap.int_stat_size = value;
			pr_debug("sii8246: DEVCAP_INT_STAT_SIZE =%d\n",
				 sii8246->devcap.int_stat_size);
			break;
		case DEVCAP_RESERVED:
			pr_info("sii8246: DEVCAP_RESERVED : %d\n", value);
			break;
		case DEVCAP_DEV_STATE:
			pr_debug("sii8246: DEVCAP_DEV_STATE\n");
			break;
		default:
			pr_debug("sii8246: DEVCAP DEFAULT\n");
			break;
		}

		break;
	default:
		pr_debug("sii8246: error: cbus_command_response"
			 "cannot handle...\n");
	}

	sii8246->cbus_pkt.command = CBUS_IDLE;
	sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);

	if (offset)
		cbus_command_request(sii8246, CBUS_READ_DEVCAP, offset, 0x00);
}

#ifdef DEBUG_MHL
static void cbus_command_response_dbg_msg(struct sii8246_data *sii8246,
					  u8 index)
{
	/*Added to debugcbus_pkt_buf */
	pr_info("sii8246: cbus_pkt_buf[index].command = %d,"
		"sii8246->cbus_pkt.command = %d\n",
		sii8246->cbus_pkt_buf[index].command,
		sii8246->cbus_pkt.command);
	pr_info("sii8246: cbus_pkt_buf[index].data[0] = %d,"
		"sii8246->cbus_pkt.data[0] =  %d\n",
		sii8246->cbus_pkt_buf[index].data[0],
		sii8246->cbus_pkt.data[0]);

	pr_info("sii8246: cbus_pkt_buf[index].data[1] = %d,"
		"sii8246->cbus_pkt.data[1] = %d\n",
		sii8246->cbus_pkt_buf[index].data[1],
		sii8246->cbus_pkt.data[1]);
	pr_info("sii8246: cbus_pkt_buf[index].offset = %d,"
		"sii8246->cbus_pkt.offset = %d\n",
		sii8246->cbus_pkt_buf[index].offset, sii8246->cbus_pkt.offset);
}
#endif

static void cbus_command_response_all(struct sii8246_data *sii8246)
{
	u8 index;
	struct cbus_packet cbus_pkt_process_buf[CBUS_PKT_BUF_COUNT];

	/*take bkp of cbus_pkt_buf */
	memcpy(cbus_pkt_process_buf, sii8246->cbus_pkt_buf,
	       sizeof(cbus_pkt_process_buf));

	/*clear cbus_pkt_buf  to hold next request */
	memset(sii8246->cbus_pkt_buf, 0x00, sizeof(sii8246->cbus_pkt_buf));

	/*process all previous requests */
	for (index = 0; index < CBUS_PKT_BUF_COUNT; index++) {
		if (cbus_pkt_process_buf[index].status == true) {
			memcpy(&sii8246->cbus_pkt, &cbus_pkt_process_buf[index],
			       sizeof(struct cbus_packet));
			cbus_command_response(sii8246);
#ifdef DEBUG_MHL
			/*print cbus_cmd messg */
			cbus_command_response_dbg_msg(sii8246, index);
#endif
		}
	}
}

static bool cbus_command_request(struct sii8246_data *sii8246,
				 enum cbus_command command, u8 offset, u8 data)
{
	u8 start_bit = 0;

	sii8246_cbus_mutex_lock(&sii8246->cbus_lock);
	if (sii8246->state != STATE_ESTABLISHED) {
		pr_debug("sii8246: cbus_command_request without establish\n");
		pr_debug("sii8246: ==> command:0x%X, offset:0x%X",
			 command, offset);
		sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
		return -EINVAL;
	}

	sii8246->cbus_pkt.command = command;
	sii8246->cbus_pkt.offset = offset;

	if (command == CBUS_MSC_MSG)
		sii8246->cbus_pkt.data[0] = offset;
	else
		sii8246->cbus_pkt.data[0] = data;

	pr_debug("sii8246: cbus_command_request Sending MSC_MSG SubCommand=%d"
		 ",key-code=%d\n", sii8246->cbus_pkt.offset,
		 sii8246->cbus_pkt.data[0]);

	cbus_write_reg(sii8246, CBUS_MSC_OFFSET_REG, sii8246->cbus_pkt.offset);
	cbus_write_reg(sii8246, CBUS_MSC_FIRST_DATA_OUT_REG,
		       sii8246->cbus_pkt.data[0]);

	switch (sii8246->cbus_pkt.command) {
	case CBUS_SET_INT:
		pr_debug("sii8246: cbus_command_request" "CBUS_SET_INT\n");
		start_bit = START_BIT_WRITE_STAT_INT;
		break;
	case CBUS_WRITE_STAT:
		pr_debug("sii8246: cbus_command_request" "CBUS_WRITE_STAT\n");
		start_bit = START_BIT_WRITE_STAT_INT;
		break;
	case CBUS_MSC_MSG:
		/*treat offset as data[0] in case of CBUS_MSC_MSG */
		sii8246->cbus_pkt.data[0] = offset;
		sii8246->cbus_pkt.data[1] = data;
		pr_debug("sii8246: cbus_command_request CBUS_MSC_MSG"
			 "SubCommand=%d,key-code=%d\n",
			 sii8246->cbus_pkt.data[0], sii8246->cbus_pkt.data[1]);

		cbus_write_reg(sii8246, CBUS_MSC_SECOND_DATA_OUT_REG,
			       sii8246->cbus_pkt.data[1]);
		cbus_write_reg(sii8246, CBUS_MSC_OFFSET_REG,
			       sii8246->cbus_pkt.command);
		start_bit = START_BIT_MSC_MSG;
		break;
	case CBUS_READ_DEVCAP:
		pr_debug("sii8246: cbus_command_request CBUS_READ_DEVCAP\n");
		start_bit = START_BIT_READ_DEVCAP;
		break;
	case CBUS_WRITE_BURST:
		pr_debug("sii8246: cbus_command_request CBUS_WRITE_BURST\n");
		start_bit = START_BIT_WRITE_BURST;
		break;
	case CBUS_GET_STATE:
	case CBUS_GET_VENDOR_ID:
	case CBUS_SET_HPD:
	case CBUS_CLR_HPD:
	case CBUS_GET_MSC_ERR_CODE:
	case CBUS_GET_SC3_ERR_CODE:
	case CBUS_GET_SC1_ERR_CODE:
	case CBUS_GET_DDC_ERR_CODE:
		cbus_write_reg(sii8246, CBUS_MSC_OFFSET_REG,
			       sii8246->cbus_pkt.command);
		start_bit = START_BIT_MSC_RESERVED;
		break;
	default:
		pr_debug("sii8246: error send cbus command fail\n");
		sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);
		return false;
	}

	pr_debug("sii8246: startbit = %d\n", start_bit);
	cbus_write_reg(sii8246, CBUS_MSC_COMMAND_START_REG, start_bit);
	save_cbus_pkt_to_buffer(sii8246);
	sii8246_cbus_mutex_unlock(&sii8246->cbus_lock);

	return true;
}

#ifdef __CONFIG_TMDS_OFFON_WORKAROUND__
static u8 sii8246_tmds_control(struct sii8246_data *sii8246, bool enable)
{
	int ret;
	pr_debug("MHL %s\n",__func__);
	if (sii8246->tmds_state == enable) {
		if (enable) {
			pr_debug("sii8246: MHL HPD High, already enabled TMDS\n");
			ret = mhl_tx_set_reg(sii8246, MHL_TX_INT_CTRL_REG,
					(1 << 4) | (1 << 5));
			if (ret < 0)
				return ret;
		} else {
			pr_debug("sii8246 MHL HPD low, already disabled TMDS\n");
			ret = mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG,
					(1 << 4) | (1 << 5));
			if (ret < 0)
				return ret;
		}
		return ret;
	} else {
		sii8246->tmds_state = enable;
	}

	if (enable) {
#ifdef	__CONFIG_RSEN_LOST_PATCH__
		ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL2_REG, 0xFC);
		if (ret < 0)
			return ret;
#endif
		ret = mhl_tx_set_reg(sii8246, MHL_TX_TMDS_CCTRL, (1 << 4));
		if (ret < 0)
			return ret;
		pr_debug("sii8246: MHL HPD High, enabled TMDS\n");
		ret = mhl_tx_set_reg(sii8246, MHL_TX_INT_CTRL_REG,
				     (1 << 4) | (1 << 5));
		if (ret < 0)
			return ret;
	} else {
		ret = mhl_tx_clear_reg(sii8246, MHL_TX_TMDS_CCTRL, (1 << 4));
		if (ret < 0)
			return ret;
		pr_debug("sii8246 MHL HPD low, disabled TMDS\n");
		ret = mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG,
				       (1 << 4) | (1 << 5));
		if (ret < 0)
			return ret;
#ifdef	__CONFIG_RSEN_LOST_PATCH__
		ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL2_REG, 0xC0);
		if (ret < 0)
			return ret;
#endif
	}

	return ret;
}

static u8 sii8246_tmds_control2(struct sii8246_data *sii8246, bool enable)
{
	int ret;

	pr_debug("MHL %s() \n", __func__);
	if (sii8246->tmds_state == enable) {
		pr_debug("%s(): already %s TMDS!!\n", __func__,
				enable ? "enabled" : "disabled");
		return 0;
	} else {
		sii8246->tmds_state = enable;
	}

	if (enable) {
#ifdef	__CONFIG_RSEN_LOST_PATCH__
		ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL2_REG, 0xFC);
		if (ret < 0)
			return ret;
#endif
		ret = mhl_tx_set_reg(sii8246, MHL_TX_TMDS_CCTRL, (1 << 4));
		if (ret < 0)
			return ret;
		pr_debug("sii8246: enabled TMDS\n");
	} else {
		ret = mhl_tx_clear_reg(sii8246, MHL_TX_TMDS_CCTRL, (1 << 4));
		if (ret < 0)
			return ret;
		pr_debug("sii8246: disabled TMDS\n");
#ifdef	__CONFIG_RSEN_LOST_PATCH__
		ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL2_REG, 0xC0);
		if (ret < 0)
			return ret;
#endif
	}

	return ret;
}
#else
static u8 sii8246_tmds_control(struct sii8246_data *sii8246, bool enable)
{
	int ret;

	if (enable) {
#ifdef	__CONFIG_RSEN_LOST_PATCH__
		ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL2_REG, 0xFC);
		if (ret < 0)
			return ret;
#endif
		ret = mhl_tx_set_reg(sii8246, MHL_TX_TMDS_CCTRL, (1 << 4));
		if (ret < 0)
			return ret;
		pr_debug("sii8246: MHL HPD High, enabled TMDS\n");
		ret = mhl_tx_set_reg(sii8246, MHL_TX_INT_CTRL_REG,
				     (1 << 4) | (1 << 5));
		if (ret < 0)
			return ret;
	} else {
		ret = mhl_tx_clear_reg(sii8246, MHL_TX_TMDS_CCTRL, (1 << 4));
		if (ret < 0)
			return ret;
		pr_debug("sii8246 MHL HPD low, disabled TMDS\n");
		ret = mhl_tx_clear_reg(sii8246, MHL_TX_INT_CTRL_REG,
				       (1 << 4) | (1 << 5));
		if (ret < 0)
			return ret;
#ifdef	__CONFIG_RSEN_LOST_PATCH__
		ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL2_REG, 0xC0);
		if (ret < 0)
			return ret;
#endif
	}

	return ret;
}
#endif

static irqreturn_t sii8246_irq_thread(int irq, void *data)
{
	struct sii8246_data *sii8246 = data;
	int ret;
	u8 intr1, intr4, value;
	u8 intr1_en, intr4_en;
	u8 cbus_intr0, cbus_intr1, cbus_intr2;
//	u8 cbus_intr1, cbus_intr2;
	u8 mhl_poweroff = 0;
	void (*cbus_resp_callback) (struct sii8246_data *) = NULL;
	pr_debug("MHL %s\n",__func__);

	pr_debug("MHL %s() \n", __func__);
	if (!sii8246) {
		pr_debug("[ERROR] %s: sii8246 is NULL & skipp this rutine\n",
		       __func__);
		return IRQ_HANDLED;
	}

	msleep(30);

	sii8246_mutex_lock(&sii8246->lock);

	ret = mhl_tx_read_reg(sii8246, MHL_TX_INTR1_REG, &intr1);
	if (ret < 0) {
		pr_debug(KERN_ERR
		       "[ERROR] %s():%d read MHL_TX_INTR1_REG failed !\n",
		       __func__, __LINE__);
		goto i2c_error_exit;
	}
	ret = mhl_tx_read_reg(sii8246, MHL_TX_INTR4_REG, &intr4);
	if (ret < 0) {
		pr_debug(KERN_ERR
		       "[ERROR] %s():%d read MHL_TX_INTR4_REG failed !\n",
		       __func__, __LINE__);
		goto i2c_error_exit;
	}

	ret = mhl_tx_read_reg(sii8246, MHL_TX_INTR1_ENABLE_REG, &intr1_en);
	if (ret < 0) {
		pr_debug(KERN_ERR
		       "[ERROR] %s():%d read MHL_TX_INTR1_ENABLE_REG failed !\n",
		       __func__, __LINE__);
		goto i2c_error_exit;
	}
	ret = mhl_tx_read_reg(sii8246, MHL_TX_INTR4_ENABLE_REG, &intr4_en);
	if (ret < 0) {
		pr_debug(KERN_ERR
		       "[ERROR] %s():%d read MHL_TX_INTR4_ENABLE_REG failed !\n",
		       __func__, __LINE__);
		goto i2c_error_exit;
	}
  ret = cbus_read_reg(sii8246, 0x51, &cbus_intr0);
  if (ret < 0) {
		pr_debug(KERN_ERR
           "[ERROR] %s():%d read CBUS_INT_STATUS_1_REG failed !\n",
           __func__, __LINE__);
    goto i2c_error_exit;
  }

	ret = cbus_read_reg(sii8246, CBUS_INT_STATUS_1_REG, &cbus_intr1);
	if (ret < 0) {
		pr_debug(KERN_ERR
		       "[ERROR] %s():%d read CBUS_INT_STATUS_1_REG failed !\n",
		       __func__, __LINE__);
		goto i2c_error_exit;
	}
	ret = cbus_read_reg(sii8246, CBUS_INT_STATUS_2_REG, &cbus_intr2);
	if (ret < 0) {
		pr_debug(KERN_ERR
		       "[ERROR] %s():%d read CBUS_INT_STATUS_2_REG failed !\n",
		       __func__, __LINE__);
		goto i2c_error_exit;
	}

	pr_debug("sii8246: irq %02x/%02x %02x/%02x %02x/%02x\n",
		 intr1, intr1_en, intr4, intr4_en, cbus_intr1, cbus_intr2);

#ifdef __CONFIG_TMDS_OFFON_WORKAROUND__
	if (intr4 & (1 << 0)) {
		ret = mhl_tx_read_reg(sii8246, 0x81, &value);
		if (ret < 0) {
			pr_debug("[ERROR] %s() read 0x81\n", __func__);
			goto i2c_error_exit;
		} else
			pr_debug("%s() 0x81 = 0x%x\n", __func__, value);

		queue_work(sii8246_tmds_offon_wq, &(sii8246->tmds_offon_work));
	}
#endif

	if (intr4 & RGND_READY_INT) {
		if (sii8246_callback_sched == 0) {
			pr_debug("%s() RGND_READY_INT\n", __func__);

			sii8246_disable_irq();

			if (sii8246->pdata->hw_reset)
				sii8246->pdata->hw_reset();

			schedule_work(&sii8246->rgnd_work);
			goto err_exit;
		}
		ret = mhl_tx_read_reg(sii8246, MHL_TX_STAT2_REG, &value);
		if (ret < 0) {
			dev_err(&sii8246->pdata->mhl_tx_client->dev,
				"STAT2 reg, err %d\n", ret);
			goto err_exit;
		}

		switch (value & RGND_INTP_MASK) {
		case RGND_INTP_OPEN:
			pr_debug("sii8246: RGND Open\n");
			sii8246->rgnd = RGND_OPEN;
			break;
		case RGND_INTP_1K:
			pr_debug("sii8246: RGND 1K!!\n");
			/* After applying RGND patch, there is some issue
			   about discovry failure
			   This point is add to fix that problem */
			ret = mhl_tx_set_reg(sii8246, MHL_TX_DISC_CTRL4_REG,
					     0x8C);
			if (ret < 0) {
				pr_debug(KERN_ERR "[ERROR] %s():%d write "
					"MHL_TX_DISC_CTRL4_REG failed !\n",
					__func__, __LINE__);
				goto i2c_error_exit;
			}
			ret = mhl_tx_write_reg(sii8246,
					       MHL_TX_DISC_CTRL5_REG, 0x77);
			if (ret < 0) {
				pr_debug(KERN_ERR "[ERROR] %s():%d write"
					" MHL_TX_DISC_CTRL5_REG failed !\n",
					__func__, __LINE__);
				goto i2c_error_exit;
			}

			ret = mhl_tx_set_reg(sii8246,
					     MHL_TX_DISC_CTRL6_REG, 0x05);
			if (ret < 0) {
				pr_debug(KERN_ERR "[ERROR] %s():%d write "
					"MHL_TX_DISC_CTRL6_REG failed !\n",
					__func__, __LINE__);
				goto i2c_error_exit;
			}

			usleep_range(T_SRC_VBUS_CBUS_TO_STABLE * USEC_PER_MSEC,
				     T_SRC_VBUS_CBUS_TO_STABLE * USEC_PER_MSEC);
			/* end of this */
			sii8246->rgnd = RGND_1K;
			break;

		case RGND_INTP_2K:
			pr_debug("sii8246: RGND 2K\n");
			sii8246->rgnd = RGND_2K;
			break;
		case RGND_INTP_SHORT:
			pr_debug("sii8246: RGND Short\n");
			sii8246->rgnd = RGND_SHORT;
			break;
		};

		if (sii8246->rgnd != RGND_1K) {
			mhl_poweroff = 1;	/*Power down mhl chip */
			pr_debug("sii8246: RGND is not 1k\n");
			sii8246->state = STATE_DISCOVERY_FAILED;
			goto err_exit;
		}
	}

	if (intr4 & CBUS_LKOUT_INT) {	/* 1<<4 */
		pr_debug("%s(): CBUS Lockout Interrupt\n", __func__);
		sii8246->state = STATE_CBUS_LOCKOUT;
	}

	if (intr4 & MHL_DISC_FAIL_INT) {	/* 1<<3 */
		pr_debug(KERN_ERR "[ERROR] %s(): MHL_DISC_FAIL_INT\n", __func__);
		sii8246->state = STATE_DISCOVERY_FAILED;
		goto err_exit;
	}

	if (intr4 & MHL_EST_INT) {	/* 1<<2 */
		pr_debug("%s(): mhl est interrupt\n", __func__);

		/* discovery override */
		ret = mhl_tx_write_reg(sii8246, MHL_TX_MHLTX_CTL1_REG, 0x10);
		if (ret < 0) {
			pr_debug(KERN_ERR
			       "[ERROR] %s():%d write MHL_TX_MHLTX_CTRL1_REG failed !\n",
			       __func__, __LINE__);
			goto i2c_error_exit;
		}

		/* increase DDC translation layer timer (byte mode) */
		ret = cbus_write_reg(sii8246, 0x07, 0xF2);
		if (ret < 0) {
			pr_debug(KERN_ERR
			       "[ERROR] %s():%d cbus_write_reg failed !\n",
			       __func__, __LINE__);
			goto i2c_error_exit;
		}

		ret = cbus_write_reg(sii8246, 0x66, 0x00);
		if (ret < 0) {
			pr_debug(KERN_ERR
			       "[ERROR] %s():%d cbus_write_reg failed !\n",
			       __func__, __LINE__);
			goto i2c_error_exit;
		}

		/* Keep the discovery enabled. Need RGND interrupt */
		ret = mhl_tx_set_reg(sii8246, MHL_TX_DISC_CTRL1_REG, (1 << 0));
		if (ret < 0) {
			pr_debug(KERN_ERR
			       "[ERROR] %s():%d mhl_tx_set_reg failed !\n",
			       __func__, __LINE__);
			goto i2c_error_exit;
		}

		sii8246->state = STATE_ESTABLISHED;

#ifndef __MHL_NEW_CBUS_MSC_CMD__
		cbus_command_request(sii8246, CBUS_SET_INT,
				     MHL_RCHANGE_INT, MHL_INT_DCAP_CHG);
#else
		sii8246_enqueue_msc_work(sii8246, CBUS_SET_INT,
					 MHL_RCHANGE_INT, MHL_INT_DCAP_CHG,
					 0x0);
#endif

		ret = mhl_tx_write_reg(sii8246, MHL_TX_INTR1_ENABLE_REG,
				       RSEN_CHANGE_INT_MASK |
				       HPD_CHANGE_INT_MASK);
	}

	if (intr1 & HPD_CHANGE_INT) {	/* 1<<6 */
		pr_debug("MHL %s %s\n",__func__,"HPD_CHANGE_INT");
		ret = cbus_read_reg(sii8246, MSC_REQ_ABORT_REASON_REG, &value);
		if (ret < 0) {
			pr_debug(KERN_ERR
			       "[ERROR] %s():%d cbus_read_reg failed !\n",
			       __func__, __LINE__);
			goto i2c_error_exit;
		}

		if (value & SET_HPD_DOWNSTREAM) {
			pr_debug("%s() hpd high\n", __func__);
			/* Downstream HPD High */

			/* Do we need to send HPD upstream using
			 * Register 0x79(page0)? Is HPD need to be overriden??
			 *      TODO: See if we need code for overriding HPD OUT
			 *      as per Page 0,0x79 Register
			 */
			sii8246->mhl_status_value.sink_hpd = true;
#ifndef __MHL_NEW_CBUS_MSC_CMD__
				cbus_command_request(sii8246, CBUS_WRITE_STAT,
						CBUS_LINK_CONTROL_2_REG,
						sii8246->mhl_status_value.
						linkmode);
#else
				sii8246_enqueue_msc_work(sii8246,
						CBUS_WRITE_STAT,
						CBUS_LINK_CONTROL_2_REG,
						sii8246->
						mhl_status_value.
						linkmode, 0x0);
#endif
			/* Enable TMDS */
			sii8246_tmds_control(sii8246, true);
			/*turn on&off hpd festure for only QCT HDMI */
#ifdef CONFIG_SAMSUNG_WORKAROUND_HPD_GLANCE
			mhl_hpd_handler(true);
#endif

		} else {
			pr_debug("sii8246: hpd low\n");
			/*Downstream HPD Low */

			if (sii8246->mhl_event_switch.state == 1) {
				pr_info("sii8246:MHL switch event sent : 0\n");
				switch_set_state(&sii8246->mhl_event_switch, 0);
			}
			/* Similar to above comments.
			 * TODO:Do we need to override HPD OUT value
			 * and do we need to disable TMDS here?
			 */
			sii8246->mhl_status_value.sink_hpd = false;
			/* Disable TMDS */
			sii8246_tmds_control(sii8246, false);

		}
	}

	if (intr1 & RSEN_CHANGE_INT) {
		/* work_around code to handle worng interrupt */
		if (sii8246->rgnd != RGND_1K) {
			pr_debug("[ERROR] sii8246: Err RSEN_HIGH"
			       " without RGND_1K rgnd=%d\n", sii8246->rgnd);
			goto err_exit2;
		}
		ret = mhl_tx_read_reg(sii8246, MHL_TX_SYSSTAT_REG, &value);
		if (ret < 0) {
			pr_debug("[ERROR] sii8246: "
					"MHL_TX_SYSSTAT_REG read error\n");
			goto err_exit2;
		}
		sii8246->rsen = value & RSEN_STATUS;

		if (value & RSEN_STATUS) {
			pr_debug("%s(): MHL cable connected.. RSEN High\n",
				__func__);
		} else {
			pr_debug("%s(): RSEN lost -\n", __func__);
			/* Once RSEN loss is confirmed,we need to check
			 * based on cable status and chip power status,whether
			 * it is SINK Loss(HDMI cable not connected, TV Off)
			 * or MHL cable disconnection
			 * TODO: Define the below mhl_disconnection()
			 */
			msleep(T_SRC_RXSENSE_DEGLITCH);
			ret = mhl_tx_read_reg(sii8246, MHL_TX_SYSSTAT_REG,
					      &value);
			if (ret < 0) {
				pr_debug(KERN_ERR
				       "[ERROR] %s() read MHL_TX_SYSSTAT_REG\n",
				       __func__);
				goto i2c_error_exit;
			}
			pr_info("sii8246:  sys_stat: %x\n", value);

			if ((value & RSEN_STATUS) == 0) {
				pr_debug(KERN_INFO
				       "%s() RSEN Really LOW ~\n", __func__);
				/*To meet CTS 3.3.22.2 spec */
				sii8246_tmds_control(sii8246, false);
				force_usb_id_switch_open(sii8246);
				release_usb_id_switch_open(sii8246);
				mhl_poweroff = 1;	/*Power down mhl chip */
				goto err_exit;
			} else
				pr_debug("sii8246: RSEN recovery ~\n");
		}
	}

	/*
	 * Process CBUS interrupts only when MHL connection has been
	 * established
	 */
	if (sii8246->state == STATE_ESTABLISHED) {

		if (cbus_intr1 & MSC_RESP_ABORT){
				sii8246->cbus_abort = true;
				printk(KERN_INFO "%s() : MSC_RESP_ABORT sii8246->cbus_abort = %d\n", __func__, sii8246->cbus_abort);
			cbus_resp_abort_error(sii8246);
			}
		if (cbus_intr1 & MSC_REQ_ABORT) {
				sii8246->cbus_abort = true;
				printk(KERN_INFO "%s() : MSC_REQ_ABORT sii8246->cbus_abort = %d\n", __func__, sii8246->cbus_abort);
			cbus_write_reg(sii8246, CBUS_INTR1_ENABLE_REG, 0);
			cbus_req_abort_error(sii8246);
			cbus_write_reg(sii8246, CBUS_INTR1_ENABLE_REG, 0xFF);
		}
		if ((cbus_intr1 & CBUS_DDC_ABORT) ||
		    (cbus_intr1 & MSC_RESP_ABORT)) {
				sii8246->cbus_abort = true;
				pr_debug("sii8246: CBUS DDC abort\n");
				printk(KERN_INFO "%s() : CBUS_DDC_ABORT or MSC_RESP_ABORT sii8246->cbus_abort = %d\n", __func__, sii8246->cbus_abort);
			if (cbus_ddc_abort_error(sii8246))
				schedule_work(&sii8246->mhl_end_work);
		}

		if (cbus_intr1 & MSC_REQ_DONE) {
			pr_debug("sii8246: CBUS cmd ACK Received\n");
#ifndef __MHL_NEW_CBUS_MSC_CMD__
			cbus_resp_callback = cbus_command_response_all;
#else
			complete(&sii8246->msc_complete);
#endif
		}
#ifdef CONFIG_SII8246_RCP
		if (cbus_intr1 & MSC_MSG_RECD) {
			pr_debug("sii8246: MSC MSG Received\n");
			cbus_handle_msc_msg(sii8246);
		}
#endif
    if(cbus_intr0&0x04){
      ret = cbus_read_reg(sii8246, 0x51, &value);
      if (ret < 0) {
        goto i2c_error_exit;
      }      

      if(value & 0x40){
  			sii8246->mhl_status_value.sink_hpd = true;
				sii8246_enqueue_msc_work(sii8246,
						CBUS_WRITE_STAT,
						CBUS_LINK_CONTROL_2_REG,
						sii8246->
						mhl_status_value.
						linkmode, 0x0);        
  			/* Enable TMDS */
  			sii8246_tmds_control(sii8246, true);
  			/*turn on&off hpd festure for only QCT HDMI */  
			} else {
					sii8246->mhl_status_value.sink_hpd = false;
					/* Disable TMDS */
					sii8246_tmds_control(sii8246, false);
      }
    }

		/* ignore WRT_STAT_RECD interrupt when we get HPD CHANGE */
		if (cbus_intr2 & WRT_STAT_RECD && intr1 == 0)
			cbus_handle_wrt_stat_recd(sii8246);

		if (cbus_intr2 & SET_INT_RECD)
			cbus_handle_set_int_recd(sii8246);
	}

 err_exit:
	pr_debug("sii8246: wake_up\n");
	wake_up(&sii8246->wq);
 err_exit2:
	ret = mhl_tx_write_reg(sii8246, MHL_TX_INTR1_REG, intr1);
	if (ret < 0)
		goto i2c_error_exit;
	ret = mhl_tx_write_reg(sii8246, MHL_TX_INTR4_REG, intr4);
	if (ret < 0)
		goto i2c_error_exit;
    
  ret = cbus_write_reg(sii8246, 0x51, cbus_intr0);
  if (ret < 0)
    goto i2c_error_exit;

	ret = cbus_write_reg(sii8246, CBUS_INT_STATUS_1_REG, cbus_intr1);
	if (ret < 0)
		goto i2c_error_exit;
	ret = cbus_write_reg(sii8246, CBUS_INT_STATUS_2_REG, cbus_intr2);
	if (ret < 0)
		goto i2c_error_exit;

	sii8246_mutex_unlock(&sii8246->lock);

	if (cbus_resp_callback)
		cbus_resp_callback(sii8246);

	if (mhl_poweroff) {
		if (sii8246_callback_sched != 0) {
			sii8246_disable_irq();
			if (sii8246->pdata->factory_test == 0) {
				schedule_work(&sii8246->mhl_d3_work);
				pr_debug("%s() normal goto_d3\n", __func__);
			} else {
				pr_debug("%s() skip goto_d3\n", __func__);
				sii8246_mhl_onoff_ex(0);
			}
		}
	}
	return IRQ_HANDLED;

 i2c_error_exit:
	sii8246_mutex_unlock(&sii8246->lock);
	pr_debug("%s(): i2c error exit\n", __func__);
	pr_debug("sii8246: wake_up\n");
	wake_up(&sii8246->wq);
	return IRQ_HANDLED;
}

static void mhl_cbus_command_timer(unsigned long data)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	pr_debug("MHL %s() \n", __func__);
	schedule_work(&sii8246->mhl_cbus_write_stat_work);
}

#ifdef __CONFIG_SS_FACTORY__
#define SII_ID 0x82
static ssize_t sysfs_check_mhl_command(struct class *class,
				       struct class_attribute *attr, char *buf)
{
	int size;
	int ret;
	u8 sii_id = 0;
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	pr_debug("MHL %s\n",__func__);

	if (sii8246->pdata->hw_onoff)
		sii8246->pdata->hw_onoff(1);

	if (sii8246->pdata->hw_reset)
		sii8246->pdata->hw_reset();

	ret = mhl_tx_read_reg(sii8246, MHL_TX_IDH_REG, &sii_id);
	if (unlikely(ret < 0))
		pr_debug("[ERROR] %s(): Failed to read register"
			" MHL_TX_IDH_REG!\n", __func__);

	pr_info("sii8246 : sel_show sii_id: %X\n", sii_id);

	if (sii8246->pdata->hw_onoff)
		sii8246->pdata->hw_onoff(0);

	size = snprintf(buf, 10, "%d\n", sii_id == SII_ID ? 1 : 0);

	return size;
}

static ssize_t sysfs_check_factory_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t size)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	const char *p = buf;
	u8 value = 0;
	pr_debug("MHL %s\n",__func__);

	if (p[0] == '1')
		value = 1;
	else
		value = 0;

	sii8246->pdata->factory_test = value;

	if (sii8246->pdata->factory_test == 1)
		pr_info("sii8246: in factory mode\n");
	else
		pr_info("sii8246: not factory mode\n");

	return size;

}

static CLASS_ATTR(test_result, 0664, sysfs_check_mhl_command,
		sysfs_check_factory_store);
#endif /*__CONFIG_SS_FACTORY__*/

#ifdef __CONFIG_MHL_FORCE_ON_FACTORY__ 
static ssize_t sysfs_mhl_on_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t size)
{
	const char *p = buf;
	if (p[0] == '1') {
		pr_info("%s() MHL Attached !!\n", __func__);
#ifdef CONFIG_MACH_MIDAS
		sii8246_wake_lock();
#endif
		sii8246_mhl_onoff_ex(1);
	} else {
		pr_info("%s() MHL Detached !!\n", __func__);
		sii8246_mhl_onoff_ex(false);
#ifdef CONFIG_MACH_MIDAS
		sii8246_wake_unlock();
#endif
	}
	return size;
}

static CLASS_ATTR(mhl_on, 0660, NULL, sysfs_mhl_on_store);
#endif /*__CONFIG_MHL_FORCE_ON_FACTORY__*/

static ssize_t sysfs_mhl_read_reg_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	pr_debug("MHL %s() \n", __func__);
	return sprintf(buf, "sysfs_mhl_read_reg_show\n");
}

static ssize_t sysfs_mhl_read_reg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct sii8246_data *sii8246 = dev_get_drvdata(sii8246_mhldev);
	enum page_num pn;
	unsigned int offset;
	int ret;
	u8 value = 0;
	char dest[10];
	char *buffer = (char *)buf;
	char *token;

	pr_debug("MHL %s() \n", __func__);
	if (size > 10) {
		pr_debug("sii8246: Error : Unsupported format\n");
		return size;
	}

	if (strnicmp(buf, "mhl", 3) == 0) {
		printk(KERN_INFO "sii8246: %s() - mhl start\n", __func__);
		schedule_work(&sii8246->mhl_restart_work);
	}

	token = strsep(&buffer, ":");
	if (token != NULL)
		strcpy(dest, token);
	else {
		pr_debug("sii8246: Error: command parsing error\n");
		return size;
	}

	ret = kstrtouint(dest, 0, &offset);
	if (ret != 0) {
		pr_debug("sii8246: Error : Page number\n");
		return size;
	}
	pn = (enum page_num)offset;

	strcpy(dest, buffer);
	ret = kstrtouint(dest, 0, &offset);
	if (ret || offset > 0xff) {
		pr_debug("sii8246: Error : Offset number\n");
		return size;
	}

	switch (pn) {
	case PAGE0:
		mhl_tx_read_reg(sii8246, offset, &value);
		break;
	case PAGE1:
		tpi_read_reg(sii8246, offset, &value);
		break;
	case PAGE2:
		hdmi_rx_read_reg(sii8246, offset, &value);
		break;
	case PAGE3:
		cbus_read_reg(sii8246, offset, &value);
		break;
	default:
		pr_debug("\nsii8246: Error : Out of the page number range\n");
		return size;
	}
	pr_debug("sii8246: MHL register Page%d:0x%02x = 0x%02x\n", pn, offset,
		value);

	return size;
}

static DEVICE_ATTR(mhl_read_reg, S_IRUGO | S_IWUSR,
		   sysfs_mhl_read_reg_show, sysfs_mhl_read_reg_store);

#ifdef __CONFIG_MHL_DEBUG__
module_param_named(sii8246_mhl_dbg_flag, sii8246_mhl_dbg_flag, uint, 0644);
#endif

#ifdef CONFIG_SLP
#ifdef CONFIG_PM_SLEEP
static int sii8246_mhl_tx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sii8246_data *sii8246 = i2c_get_clientdata(client);

	pr_debug("MHL %s() \n", __func__);
	if (!sii8246 || !sii8246->pdata)
		return -EFAULT;

	sii8246_mutex_lock(&sii8246->lock);

	if (!sii8246->pdata->power_state)
		goto out;

	sii8246->pdata->power_state = 0;

	if (sii8246->pdata->mhl_sel)
		sii8246->pdata->mhl_sel(0);

	sii8246_power_down(sii8246);

	if (sii8246->pdata->hw_onoff)
		sii8246->pdata->hw_onoff(0);
 out:
	sii8246_mutex_unlock(&sii8246->lock);
	return 0;
}

static int sii8246_mhl_tx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sii8246_data *sii8246 = i2c_get_clientdata(client);

	pr_debug("MHL %s() \n", __func__);
	if (!sii8246 || !sii8246->pdata)
		return -EFAULT;

	sii8246_mutex_lock(&sii8246->lock);

#ifdef	CONFIG_SAMSUNG_USE_11PIN_CONNECTOR
	if (!is_mhl_cable_connected()) {
		sii8246_mutex_unlock(&sii8246->lock);
		return 0;
	}
#endif

	schedule_work(&sii8246->mhl_restart_work);
	sii8246_mutex_unlock(&sii8246->lock);

	return 0;
}

static const struct dev_pm_ops sii8246_pm_ops = {
	.suspend = sii8246_mhl_tx_suspend,
	.resume = sii8246_mhl_tx_resume,
};
#endif
#endif

#ifdef CONFIG_EXTCON
static void sii8246_extcon_work(struct work_struct *work)
{
	struct sii8246_data *sii8246 =
		container_of(work, struct sii8246_data, extcon_wq);

	pr_debug("MHL %s() \n", __func__);
	pr_debug("%s: MHL is %s\n", __func__,
		sii8246->extcon_attached ? "attached" : "detached");

	if (sii8246->extcon_attached) {
#ifdef CONFIG_JACK_MON
		jack_event_handler("hdmi", 1);
#endif
#ifdef CONFIG_SAMSUNG_MHL
#ifdef CONFIG_MACH_MIDAS
		sii8246_wake_lock();
#endif
		mhl_onoff_ex(1);
#endif

	} else {
#ifdef CONFIG_JACK_MON
		jack_event_handler("hdmi", 0);
#endif
#ifdef CONFIG_SAMSUNG_MHL
		mhl_onoff_ex(false);
#ifdef CONFIG_MACH_MIDAS
		sii8246_wake_unlock();
#endif
#endif
	}
}

static int sii8246_extcon_notifier(struct notifier_block *self,
			unsigned long event, void *ptr)
{
	struct sii8246_data *sii8246 =
		container_of(self, struct sii8246_data, extcon_nb);

	pr_debug("MHL %s() \n", __func__);
	sii8246->extcon_attached = event;
	schedule_work(&sii8246->extcon_wq);

	return NOTIFY_DONE;
}
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static void sii8246_early_suspend(struct early_suspend *early_sus)
{
	struct sii8246_data *sii8246 = container_of(early_sus,
			struct sii8246_data, early_suspend);

	pr_debug("MHL %s() \n", __func__);
	pr_debug("%s()\n", __func__);
	if (!sii8246 || !sii8246->pdata)
		return;

	sii8246_mutex_lock(&sii8246->lock);
	sii8246->suspend_state = true;
	sii8246_mutex_unlock(&sii8246->lock);
}

static void sii8246_late_resume(struct early_suspend *early_sus)
{
	struct sii8246_data *sii8246 = container_of(early_sus,
			struct sii8246_data, early_suspend);

	pr_debug("MHL %s() \n", __func__);
	pr_debug("%s()\n", __func__);
	if (!sii8246 || !sii8246->pdata)
		return;

	sii8246_mutex_lock(&sii8246->lock);
	sii8246->suspend_state = false;
	sii8246_mutex_unlock(&sii8246->lock);

	sii8246_mhl_onoff_ex(false);
	sii8246_mhl_onoff_ex(true);

}
#endif

static int __devinit sii8246_mhl_tx_i2c_probe(struct i2c_client *client,
					      const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sii8246_data *sii8246;
	u8 deviceId_1, deviceId_2;
#ifdef CONFIG_SII8246_RCP
	struct input_dev *input;
#endif
	int ret;
#if defined(__CONFIG_SS_FACTORY__) || defined(__CONFIG_MHL_SWING_LEVEL__)
	struct class *sec_mhl;
#endif
#ifdef __CONFIG_MHL_DEBUG__
	sii8246_mhl_dbg_flag = 1;
#endif
	pr_debug("MHL %s\n",__func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	sii8246 = kzalloc(sizeof(struct sii8246_data), GFP_KERNEL);
	if (!sii8246) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}
#ifdef CONFIG_SII8246_RCP
	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device.\n");
		ret = -ENOMEM;
		goto err_exit0;
	}
#endif

	sii8246->pdata = client->dev.platform_data;
	if (!sii8246->pdata) {
		ret = -EINVAL;
		goto err_exit1;
	}
	sii8246->pdata->mhl_tx_client = client;
	sii8246->pdata->factory_test = 0;

	init_waitqueue_head(&sii8246->wq);
	mutex_init(&sii8246->lock);
	mutex_init(&sii8246_irq_lock);
	mutex_init(&sii8246->cbus_lock);

#ifdef __SII8246_MUTEX_DEBUG__
	g_mutex_cnt = 0;
	g_cbus_mutex_cnt = 0;
#endif
	INIT_WORK(&sii8246->mhl_restart_work, mhl_start_worker);

	INIT_WORK(&sii8246->mhl_end_work, mhl_end_worker);

	INIT_WORK(&sii8246->rgnd_work, sii8246_detection_callback_worker);

	INIT_WORK(&sii8246->mhl_d3_work, mhl_goto_d3_worker);

	INIT_WORK(&sii8246->mhl_cbus_write_stat_work,
		  mhl_cbus_write_stat_worker);

	sii8246->pdata->init();

	i2c_set_clientdata(client, sii8246);
	client->irq = gpio_to_irq(GPIO_MHL_INT);
	sii8246_mhldev = &client->dev;

#ifdef CONFIG_MACH_MIDAS
	wake_lock_init(&sii8246->mhl_wake_lock, WAKE_LOCK_SUSPEND,
		       "mhl_wake_lock");
	pr_debug("%s(): wake lock is initialized.\n", __func__);
#endif

#ifdef __SII8246_IRQ_DEBUG__
	sii8246_en_irq = 0;
#endif
	if (sii8246->pdata->swing_level == 0)
		sii8246->pdata->swing_level = 0x74;
#ifdef __MHL_NEW_CBUS_MSC_CMD__
	sii8246_msc_wq = create_singlethread_workqueue("sii8246_msc_wq");
	if (!sii8246_msc_wq) {
		printk(KERN_ERR
		       "[ERROR] %s() workqueue create fail\n", __func__);
		ret = -ENOMEM;
		goto	err_msc_wq;
	}
	INIT_WORK(&sii8246->msc_work, sii8246_process_msc_work);
#endif

#ifdef __CONFIG_TMDS_OFFON_WORKAROUND__
	sii8246_tmds_offon_wq =
		create_singlethread_workqueue("sii8246_tmds_offon_wq");
	if (!sii8246_tmds_offon_wq) {
		printk(KERN_ERR	"[ERROR] %s() tmds_offon"
				" workqueue create fail\n", __func__);
		ret = -ENOMEM;
		goto	err_tmds_offon_wq;
	}
	INIT_WORK(&sii8246->tmds_offon_work, sii8246_tmds_offon_work);
#endif

	ret = request_threaded_irq(client->irq, NULL, sii8246_irq_thread,
				   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				   "sii8246", sii8246);
	if (ret < 0)
		goto err_exit_after_irq;

	atomic_set(&sii8246->is_irq_enabled, false);
	disable_irq(client->irq);

#if defined(__CONFIG_SS_FACTORY__) || defined(__CONFIG_MHL_SWING_LEVEL__) \
	|| defined(__CONFIG_MHL_FORCE_ON_FACTORY__) 
	pr_debug("sii8246 : create mhl sysfile\n");

	sec_mhl = class_create(THIS_MODULE, "mhl");
	if (IS_ERR(sec_mhl)) {
		pr_debug("[ERROR] Failed to create class(sec_mhl)!\n");
		goto err_exit_after_irq;
	}
#endif

#ifdef __CONFIG_SS_FACTORY__
	ret = class_create_file(sec_mhl, &class_attr_test_result);
	if (ret) {
		pr_debug("[ERROR] Failed to create "
				"device file in sysfs entries!\n");
		goto err_exit2a;
	}
#endif

#ifdef __CONFIG_MHL_SWING_LEVEL__
	pr_debug("sii8246 : create mhl sysfile\n");

	ret = class_create_file(sec_mhl, &class_attr_swing);
	if (ret) {
		pr_debug("[ERROR] failed to create swing sysfs file\n");
		goto err_exit2b;
	}
#endif

#ifdef __CONFIG_MHL_FORCE_ON_FACTORY__
	ret = class_create_file(sec_mhl, &class_attr_mhl_on);
	if (ret) {
		pr_debug("[ERROR] Failed to create "
				"device file in sysfs entries!\n");
		goto err_exit2c;
	}
#endif

	sii8246->cbus_pkt.command = CBUS_IDLE;
	sii8246->cbus_pkt.offset = DEVCAP_DEV_STATE;
#ifdef CONFIG_SII8246_RCP
	/* indicate that we generate key events */
	set_bit(EV_KEY, input->evbit);
	bitmap_fill(input->keybit, KEY_MAX);

	sii8246->input_dev = input;
	input_set_drvdata(input, sii8246);
	input->name = "sii8246_rcp";
	input->id.bustype = BUS_I2C;

	ret = input_register_device(input);
	if (ret < 0) {
		dev_err(&client->dev, "fail to register input device\n");
		goto err_exit2d;
	}
#endif

#ifdef CONFIG_EXTCON
	/* Extcon */
	INIT_WORK(&sii8246->extcon_wq, sii8246_extcon_work);
	sii8246->extcon_nb.notifier_call = sii8246_extcon_notifier;
	ret = extcon_register_interest(&sii8246->extcon_dev,
					sii8246->pdata->extcon_name,
					"MHL",
					&sii8246->extcon_nb);
	if (ret < 0) {
		pr_debug("Cannot register extcon_dev for %s(cable: MHL).\n",
						sii8246->pdata->extcon_name);
		ret = -EINVAL;
		goto err_extcon;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	sii8246->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 5;
	sii8246->early_suspend.suspend = sii8246_early_suspend;
	sii8246->early_suspend.resume = sii8246_late_resume;
	register_early_suspend(&sii8246->early_suspend);
	sii8246->suspend_state = false;
#endif
#ifdef __CONFIG_TMDS_OFFON_WORKAROUND__
	sii8246->tmds_state = 0;
#endif
#if defined(CONFIG_SAMSUNG_WORKAROUND_HPD_GLANCE) &&\
	!defined(CONFIG_SAMSUNG_MHL_9290)
	atomic_set(&sii8246_sched_hpd_handler_false, 0);
#endif


	sii8246->mhl_event_switch.name = "mhl_event_switch";
	switch_dev_register(&sii8246->mhl_event_switch);

#if 0 //gihyuk.lee
	ret = mhl_tx_read_reg(sii8246, 0x03, &deviceId_1);
	pr_debug("sii8246_mhl_tx_i2c_probe : mhl_device_id = %x\n", deviceId_1);
	ret = mhl_tx_read_reg(sii8246, 0x02, &deviceId_2);
	pr_debug("sii8246_mhl_tx_i2c_probe : mhl_device_id = %x\n", deviceId_2);

	if (ret < 0) {
		pr_debug("[ERROR] sii8246 : %s mhl_tx_read_reg(ii8246, 0x03, &value)\n", __func__);
		return ret;
	}

	if(deviceId_1 != 0x92 || deviceId_2  != 0x34) {
		ret = -ENOMEM;
		kfree(sii8246);
		sii8246 = NULL;
	}
#endif
	return 0;

#ifdef CONFIG_EXTCON
err_extcon:
	extcon_unregister_interest(&sii8246->extcon_dev);
#endif
 err_exit2d:
#ifdef __CONFIG_MHL_FORCE_ON_FACTORY__
	class_remove_file(sec_mhl, &class_attr_mhl_on);
#endif
 err_exit2c:
#ifdef __CONFIG_MHL_SWING_LEVEL__
	class_remove_file(sec_mhl, &class_attr_swing);
#endif
 err_exit2b:
#ifdef __CONFIG_SS_FACTORY__
	class_remove_file(sec_mhl, &class_attr_test_result);
#endif
 err_exit2a:
#if defined(__CONFIG_SS_FACTORY__) || defined(__CONFIG_MHL_SWING_LEVEL__) \
	|| defined(__CONFIG_MHL_FORCE_ON_FACTORY__) 
	class_destroy(sec_mhl);
#endif
err_exit_after_irq:
	free_irq(client->irq, sii8246);
#ifdef __CONFIG_TMDS_OFFON_WORKAROUND__
err_tmds_offon_wq:
	destroy_workqueue(sii8246_tmds_offon_wq);
#endif

#ifdef __MHL_NEW_CBUS_MSC_CMD__
err_msc_wq:
	destroy_workqueue(sii8246_msc_wq);
#endif

err_exit1:
#ifdef CONFIG_SII8246_RCP
	input_free_device(input);
#endif
err_exit0:
	kfree(sii8246);
	sii8246 = NULL;
	return ret;
}

static int __devinit sii8246_tpi_i2c_probe(struct i2c_client *client,
					   const struct i2c_device_id *id)
{
	struct sii8246_platform_data *pdata = client->dev.platform_data;
	pr_debug("MHL %s\n",__func__);
	if (!pdata)
		return -EINVAL;
	pdata->tpi_client = client;
	return 0;
}

static int __devinit sii8246_hdmi_rx_i2c_probe(struct i2c_client *client,
					       const struct i2c_device_id *id)
{
	struct sii8246_platform_data *pdata = client->dev.platform_data;
	pr_debug("MHL %s\n",__func__);
	if (!pdata)
		return -EINVAL;

	pdata->hdmi_rx_client = client;
	return 0;
}

static int __devinit sii8246_cbus_i2c_probe(struct i2c_client *client,
					    const struct i2c_device_id *id)
{
	struct sii8246_platform_data *pdata = client->dev.platform_data;
	if (!pdata)
		return -EINVAL;

	pdata->cbus_client = client;
	return 0;
}

static int __devexit sii8246_mhl_tx_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit sii8246_tpi_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit sii8246_hdmi_rx_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit sii8246_cbus_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sii8246_mhl_tx_id[] = {
	{"sii8246_mhl_tx", 0},
	{}
};

static const struct i2c_device_id sii8246_tpi_id[] = {
	{"sii8246_tpi", 0},
	{}
};

static const struct i2c_device_id sii8246_hdmi_rx_id[] = {
	{"sii8246_hdmi_rx", 0},
	{}
};

static const struct i2c_device_id sii8246_cbus_id[] = {
	{"sii8246_cbus", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sii8246_mhl_tx_id);
MODULE_DEVICE_TABLE(i2c, sii8246_tpi_id);
MODULE_DEVICE_TABLE(i2c, sii8246_hdmi_rx_id);
MODULE_DEVICE_TABLE(i2c, sii8246_cbus_id);

static struct i2c_driver sii8246_mhl_tx_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sii8246_mhl_tx",
#ifdef CONFIG_SLP
		   .pm = &sii8246_pm_ops,
#endif
		   },
	.id_table = sii8246_mhl_tx_id,
	.probe = sii8246_mhl_tx_i2c_probe,
	.remove = __devexit_p(sii8246_mhl_tx_remove),
	.command = NULL,
};

static struct i2c_driver sii8246_tpi_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sii8246_tpi",
		   },
	.id_table = sii8246_tpi_id,
	.probe = sii8246_tpi_i2c_probe,
	.remove = __devexit_p(sii8246_tpi_remove),
};

static struct i2c_driver sii8246_hdmi_rx_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sii8246_hdmi_rx",
		   },
	.id_table = sii8246_hdmi_rx_id,
	.probe = sii8246_hdmi_rx_i2c_probe,
	.remove = __devexit_p(sii8246_hdmi_rx_remove),
};

static struct i2c_driver sii8246_cbus_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "sii8246_cbus",
		   },
	.id_table = sii8246_cbus_id,
	.probe = sii8246_cbus_i2c_probe,
	.remove = __devexit_p(sii8246_cbus_remove),
};

static int __init sii8246_init(void)
{
	int ret;
	pr_debug("MHL %s\n",__func__);

	ret = i2c_add_driver(&sii8246_mhl_tx_i2c_driver);
	if (ret < 0)
		return ret;

	ret = i2c_add_driver(&sii8246_tpi_i2c_driver);
	if (ret < 0)
		goto err_exit1;

	ret = i2c_add_driver(&sii8246_hdmi_rx_i2c_driver);
	if (ret < 0)
		goto err_exit2;

	ret = i2c_add_driver(&sii8246_cbus_i2c_driver);
	if (ret < 0)
		goto err_exit3;

	return 0;

 err_exit3:
	i2c_del_driver(&sii8246_hdmi_rx_i2c_driver);
 err_exit2:
	i2c_del_driver(&sii8246_tpi_i2c_driver);
 err_exit1:
	i2c_del_driver(&sii8246_mhl_tx_i2c_driver);
	pr_debug("[ERROR] i2c_add_driver fail\n");
	return ret;
}

static void __exit sii8246_exit(void)
{
	pr_debug("MHL %s\n",__func__);
	i2c_del_driver(&sii8246_cbus_i2c_driver);
	i2c_del_driver(&sii8246_hdmi_rx_i2c_driver);
	i2c_del_driver(&sii8246_tpi_i2c_driver);
	i2c_del_driver(&sii8246_mhl_tx_i2c_driver);
}

module_init(sii8246_init);
module_exit(sii8246_exit);

