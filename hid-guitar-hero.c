// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for Guitar Hero devices.
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2008 Jiri Slaby
 *  Copyright (c) 2012 David Dillow <dave@thedillows.org>
 *  Copyright (c) 2006-2013 Jiri Kosina
 *  Copyright (c) 2013 Colin Leitner <colin.leitner@gmail.com>
 *  Copyright (c) 2014-2016 Frank Praznik <frank.praznik@gmail.com>
 *  Copyright (c) 2018 Todd Kelner
 *  Copyright (c) 2020 Sanjay Govind
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/idr.h>
#include <linux/input/mt.h>
#include <linux/crc32.h>
#include <asm/unaligned.h>

#include "hid-ids.h"

static enum power_supply_property guitar_hero_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_STATUS,
};

enum guitar_hero_worker {
	SONY_WORKER_STATE,
	SONY_WORKER_HOTPLUG
};

#define GUITAR_TILT_USAGE 44

struct guitar_hero_sc {
	spinlock_t lock;
	struct list_head list_node;
	struct hid_device *hdev;
	struct input_dev *touchpad;
	struct input_dev *sensor_dev;
	struct work_struct hotplug_worker;
	struct work_struct state_worker;
	void (*send_output_report)(struct guitar_hero_sc *);
	struct power_supply *battery;
	struct power_supply_desc battery_desc;
	int device_id;
	unsigned fw_version;
	unsigned hw_version;
	u8 *output_report_dmabuf;
	u8 mac_address[6];
	u8 hotplug_worker_initialized;
	u8 state_worker_initialized;
	u8 defer_initialization;
	u8 cable_state;
	u8 battery_charging;
	u8 battery_capacity;
	bool timestamp_initialized;
	u16 prev_timestamp;
	unsigned int timestamp_us;
};

static inline void guitar_hero_schedule_work(struct guitar_hero_sc *sc,
				      enum guitar_hero_worker which)
{
	unsigned long flags;

	switch (which) {
	case SONY_WORKER_STATE:
		spin_lock_irqsave(&sc->lock, flags);
		if (!sc->defer_initialization && sc->state_worker_initialized)
			schedule_work(&sc->state_worker);
		spin_unlock_irqrestore(&sc->lock, flags);
		break;
	case SONY_WORKER_HOTPLUG:
		if (sc->hotplug_worker_initialized)
			schedule_work(&sc->hotplug_worker);
		break;
	}
}

static ssize_t guitar_hero_show_firmware_version(struct device *dev,
				struct device_attribute
				*attr, char *buf)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct guitar_hero_sc *sc = hid_get_drvdata(hdev);

	return snprintf(buf, PAGE_SIZE, "0x%04x\n", sc->fw_version);
}

static DEVICE_ATTR(firmware_version, 0444, guitar_hero_show_firmware_version, NULL);

static ssize_t guitar_hero_show_hardware_version(struct device *dev,
				struct device_attribute
				*attr, char *buf)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct guitar_hero_sc *sc = hid_get_drvdata(hdev);

	return snprintf(buf, PAGE_SIZE, "0x%04x\n", sc->hw_version);
}

static DEVICE_ATTR(hardware_version, 0444, guitar_hero_show_hardware_version, NULL);

static int guitar_mapping(struct hid_device *hdev, struct hid_input *hi,
			  struct hid_field *field, struct hid_usage *usage,
			  unsigned long **bit, int *max)
{
	if ((usage->hid & HID_USAGE_PAGE) == HID_UP_MSVENDOR) {
		unsigned int abs = usage->hid & HID_USAGE;

		if (abs == GUITAR_TILT_USAGE) {
			hid_map_usage_clear(hi, usage, bit, max, EV_ABS, ABS_RY);
			return 1;
		}
	}
	return 0;
}

static void sixaxis_parse_report(struct guitar_hero_sc *sc, u8 *rd, int size)
{
	static const u8 sixaxis_battery_capacity[] = { 0, 1, 25, 50, 75, 100 };
	unsigned long flags;
	int offset;
	u8 cable_state, battery_capacity, battery_charging;
	offset = 30;

	u8 index = rd[offset] <= 5 ? rd[offset] : 5;
	battery_capacity = sixaxis_battery_capacity[index];
	battery_charging = 0;
	cable_state = 0;

	spin_lock_irqsave(&sc->lock, flags);
	sc->cable_state = cable_state;
	sc->battery_capacity = battery_capacity;
	sc->battery_charging = battery_charging;
	spin_unlock_irqrestore(&sc->lock, flags);

	input_sync(sc->sensor_dev);
	
}

static int guitar_hero_raw_event(struct hid_device *hdev, struct hid_report *report,
		u8 *rd, int size)
{
	struct guitar_hero_sc *sc = hid_get_drvdata(hdev);

	/*
	 * Sixaxis HID report has acclerometers/gyro with MSByte first, this
	 * has to be BYTE_SWAPPED before passing up to joystick interface
	 */
	if (rd[0] == 0x01 && size == 49) {
		/*
		 * When connected via Bluetooth the Sixaxis occasionally sends
		 * a report with the second byte 0xff and the rest zeroed.
		 *
		 * This report does not reflect the actual state of the
		 * controller must be ignored to avoid generating false input
		 * events.
		 */
		if (rd[1] == 0xff)
			return -EINVAL;

		swap(rd[41], rd[42]);
		swap(rd[43], rd[44]);
		swap(rd[45], rd[46]);
		swap(rd[47], rd[48]);

		sixaxis_parse_report(sc, rd, size);
	} 

	if (sc->defer_initialization) {
		sc->defer_initialization = 0;
		guitar_hero_schedule_work(sc, SONY_WORKER_STATE);
	}

	return 0;
}

static int guitar_hero_mapping(struct hid_device *hdev, struct hid_input *hi,
			struct hid_field *field, struct hid_usage *usage,
			unsigned long **bit, int *max)
{
	return guitar_mapping(hdev, hi, field, usage, bit, max);
}

static void guitar_hero_state_worker(struct work_struct *work)
{
	struct guitar_hero_sc *sc = container_of(work, struct guitar_hero_sc, state_worker);

	sc->send_output_report(sc);
}

static int guitar_hero_battery_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct guitar_hero_sc *sc = power_supply_get_drvdata(psy);
	unsigned long flags;
	int ret = 0;
	u8 battery_charging, battery_capacity, cable_state;

	spin_lock_irqsave(&sc->lock, flags);
	battery_charging = sc->battery_charging;
	battery_capacity = sc->battery_capacity;
	cable_state = sc->cable_state;
	spin_unlock_irqrestore(&sc->lock, flags);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = battery_capacity;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (battery_charging)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			if (battery_capacity == 100 && cable_state)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int guitar_hero_battery_probe(struct guitar_hero_sc *sc, int append_dev_id)
{
	const char *battery_str_fmt = append_dev_id ?
		"guitar_hero_controller_battery_%pMR_%i" :
		"guitar_hero_controller_battery_%pMR";
	struct power_supply_config psy_cfg = { .drv_data = sc, };
	struct hid_device *hdev = sc->hdev;
	int ret;

	/*
	 * Set the default battery level to 100% to avoid low battery warnings
	 * if the battery is polled before the first device report is received.
	 */
	sc->battery_capacity = 100;

	sc->battery_desc.properties = guitar_hero_battery_props;
	sc->battery_desc.num_properties = ARRAY_SIZE(guitar_hero_battery_props);
	sc->battery_desc.get_property = guitar_hero_battery_get_property;
	sc->battery_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	sc->battery_desc.use_for_apm = 0;
	sc->battery_desc.name = devm_kasprintf(&hdev->dev, GFP_KERNEL,
					  battery_str_fmt, sc->mac_address, sc->device_id);
	if (!sc->battery_desc.name)
		return -ENOMEM;

	sc->battery = devm_power_supply_register(&hdev->dev, &sc->battery_desc,
					    &psy_cfg);
	if (IS_ERR(sc->battery)) {
		ret = PTR_ERR(sc->battery);
		hid_err(hdev, "Unable to register battery device\n");
		return ret;
	}

	power_supply_powers(sc->battery, &hdev->dev);
	return 0;
}

static inline void guitar_hero_init_output_report(struct guitar_hero_sc *sc,
				void (*send_output_report)(struct guitar_hero_sc *))
{
	sc->send_output_report = send_output_report;

	if (!sc->state_worker_initialized)
		INIT_WORK(&sc->state_worker, guitar_hero_state_worker);

	sc->state_worker_initialized = 1;
}

static inline void guitar_hero_cancel_work_sync(struct guitar_hero_sc *sc)
{
	unsigned long flags;

	if (sc->hotplug_worker_initialized)
		cancel_work_sync(&sc->hotplug_worker);
	if (sc->state_worker_initialized) {
		spin_lock_irqsave(&sc->lock, flags);
		sc->state_worker_initialized = 0;
		spin_unlock_irqrestore(&sc->lock, flags);
		cancel_work_sync(&sc->state_worker);
	}
}

static int guitar_hero_input_configured(struct hid_device *hdev,
					struct hid_input *hidinput)
{
	struct guitar_hero_sc *sc = hid_get_drvdata(hdev);
	int ret;	
	ret = guitar_hero_battery_probe(sc, false);
	if (ret < 0)
		goto err_stop;

	/* Open the device to receive reports with battery info */
	ret = hid_hw_open(hdev);
	if (ret < 0) {
		hid_err(hdev, "hw open failed\n");
		goto err_stop;
	}
	return 0;
err_stop:
	/* Piggy back on the default ds4_bt_ poll_interval to determine
	 * if we need to remove the file as we don't know for sure if we
	 * executed that logic.
	 */
	if (sc->fw_version)
		device_remove_file(&sc->hdev->dev, &dev_attr_firmware_version);
	if (sc->hw_version)
		device_remove_file(&sc->hdev->dev, &dev_attr_hardware_version);
	guitar_hero_cancel_work_sync(sc);
	return ret;
}

static int guitar_hero_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	struct guitar_hero_sc *sc;
	unsigned int connect_mask = HID_CONNECT_DEFAULT;

	sc = devm_kzalloc(&hdev->dev, sizeof(*sc), GFP_KERNEL);
	if (sc == NULL) {
		hid_err(hdev, "can't alloc guitar_hero descriptor\n");
		return -ENOMEM;
	}

	spin_lock_init(&sc->lock);

	hid_set_drvdata(hdev, sc);
	sc->hdev = hdev;

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, connect_mask);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		return ret;
	}

	/* guitar_hero_input_configured can fail, but this doesn't result
	 * in hid_hw_start failures (intended). Check whether
	 * the HID layer claimed the device else fail.
	 * We don't know the actual reason for the failure, most
	 * likely it is due to EEXIST in case of double connection
	 * of USB and Bluetooth, but could have been due to ENOMEM
	 * or other reasons as well.
	 */
	if (!(hdev->claimed & HID_CLAIMED_INPUT)) {
		hid_err(hdev, "failed to claim input\n");
		hid_hw_stop(hdev);
		return -ENODEV;
	}

	return ret;
}

static void guitar_hero_remove(struct hid_device *hdev)
{
	struct guitar_hero_sc *sc = hid_get_drvdata(hdev);

	hid_hw_close(hdev);

	if (sc->fw_version)
		device_remove_file(&sc->hdev->dev, &dev_attr_firmware_version);

	if (sc->hw_version)
		device_remove_file(&sc->hdev->dev, &dev_attr_hardware_version);

	guitar_hero_cancel_work_sync(sc);

	hid_hw_stop(hdev);
}

static const struct hid_device_id guitar_hero_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_ACTIVISION, USB_DEVICE_ID_ACTIVISION_GUITAR)},
	{ HID_USB_DEVICE(USB_VENDOR_ID_SONY2, USB_DEVICE_ID_SONY_GUITAR_CONTROLLER)},
	{ }
};
MODULE_DEVICE_TABLE(hid, guitar_hero_devices);

static struct hid_driver guitar_hero_driver = {
	.name             = "guitar_hero",
	.id_table         = guitar_hero_devices,
	.input_mapping    = guitar_hero_mapping,
	.input_configured = guitar_hero_input_configured,
	.probe            = guitar_hero_probe,
	.remove           = guitar_hero_remove,
	.raw_event        = guitar_hero_raw_event
};

static int __init guitar_hero_init(void)
{
	dbg_hid("Sony:%s\n", __func__);

	return hid_register_driver(&guitar_hero_driver);
}

static void __exit guitar_hero_exit(void)
{
	dbg_hid("Sony:%s\n", __func__);

	hid_unregister_driver(&guitar_hero_driver);
}
module_init(guitar_hero_init);
module_exit(guitar_hero_exit);

MODULE_LICENSE("GPL");
