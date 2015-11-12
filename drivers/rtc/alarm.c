/* drivers/rtc/alarm.c
 *
 * Copyright (C) 2007-2009 Google, Inc.
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

#include <linux/module.h>
#include <linux/android_alarm.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>

#include <asm/mach/time.h>
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
#include <linux/slab.h>
#endif
/*   2013-11-19 yuyi Add end for power up alarm */

#ifndef VENDOR_EDIT //mingqiang.guo@phone.bsp modify for power up at 40s before timing power up time
#define ALARM_DELTA 120
#else //VENDOR_EDIT
#define ALARM_DELTA 40
#endif //VENDOR_EDIT
#define ANDROID_ALARM_PRINT_ERROR (1U << 0)
#define ANDROID_ALARM_PRINT_INIT_STATUS (1U << 1)
#define ANDROID_ALARM_PRINT_TSET (1U << 2)
#define ANDROID_ALARM_PRINT_CALL (1U << 3)
#define ANDROID_ALARM_PRINT_SUSPEND (1U << 4)
#define ANDROID_ALARM_PRINT_INT (1U << 5)
#define ANDROID_ALARM_PRINT_FLOW (1U << 6)

static int debug_mask = ANDROID_ALARM_PRINT_ERROR | \
			ANDROID_ALARM_PRINT_INIT_STATUS;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define pr_alarm(debug_level_mask, args...) \
	do { \
		if (debug_mask & ANDROID_ALARM_PRINT_##debug_level_mask) { \
			pr_info(args); \
		} \
	} while (0)

/*   2013-11-19 yuyi modify begin for power up alarm */
#ifndef VENDOR_EDIT
#define ANDROID_ALARM_WAKEUP_MASK ( \
	ANDROID_ALARM_RTC_WAKEUP_MASK | \
	ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP_MASK)
#else
#define ANDROID_ALARM_WAKEUP_MASK ( \
	ANDROID_ALARM_RTC_WAKEUP_MASK | \
	ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP_MASK | \
	ANDROID_ALARM_RTC_POWERUP)
#endif
/*   2013-11-19 yuyi modify end for power up alarm */

/* support old usespace code */
#define ANDROID_ALARM_SET_OLD               _IOW('a', 2, time_t) /* set alarm */
#define ANDROID_ALARM_SET_AND_WAIT_OLD      _IOW('a', 3, time_t)

struct alarm_queue {
	struct rb_root alarms;
	struct rb_node *first;
	struct hrtimer timer;
	ktime_t delta;
	bool stopped;
	ktime_t stopped_time;
};

/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
enum RTC_CMD {
	RTC_CMD_CLEAR = 0x61,
	RTC_CMD_UPDATE,
};

struct rtc_cmd {
	struct list_head node;
	unsigned int cmd;
	struct timespec time;
};

struct rtc_alarm_work {
	struct list_head cmd_list;
	struct work_struct alarm_task;
	struct mutex mutex;
	spinlock_t slock;
	bool active;
};
#define DEBUG_PRINT_TIME
#endif
/*   2013-11-19 yuyi Add end for power up alarm */

static struct rtc_device *alarm_rtc_dev;
static DEFINE_SPINLOCK(alarm_slock);
static DEFINE_MUTEX(alarm_setrtc_mutex);
static DEFINE_MUTEX(power_on_alarm_mutex);
static struct wake_lock alarm_rtc_wake_lock;
static struct platform_device *alarm_platform_dev;
struct alarm_queue alarms[ANDROID_ALARM_TYPE_COUNT];
static bool suspended;
static long power_on_alarm;

static int set_alarm_time_to_rtc(const long);
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT

static struct rtc_alarm_work rtc_work;
static int rtc_alarm_update(struct timespec *alarm);
static void rtc_alarm_clear(void);

static void rtc_task(struct work_struct *work)
{
	unsigned long flags;
	struct rtc_cmd *cmd;
	struct list_head *list_pos, *list_pos_tmp;
	struct rtc_alarm_work *rwork =
		container_of(work, struct rtc_alarm_work, alarm_task);

	pr_alarm(FLOW, "handle rtc task...\n");
	if (!rwork->active) {
		pr_alarm(FLOW, "rtc_alarm_work is unactive\n");
		return;
	}

	/* We want to keep the cmd order and so the follow code is non-reentrant. */
	mutex_lock(&rwork->mutex);
	while(1) {
		cmd = NULL;

		spin_lock_irqsave(&rwork->slock, flags);
		if (list_empty(&rwork->cmd_list)) {
			spin_unlock_irqrestore(&rwork->slock, flags);
			pr_alarm(FLOW, "no rtc task in queue any more\n");
			break;
		}

		list_for_each_safe(list_pos, list_pos_tmp, &rwork->cmd_list) {
			cmd = list_entry(list_pos, struct rtc_cmd, node);

			if (list_is_last(list_pos, &rwork->cmd_list)) {
				list_del(list_pos);
				break;
			}
			list_del(list_pos);
			kzfree(cmd);
		}
		spin_unlock_irqrestore(&rwork->slock, flags);

		if (cmd == NULL) continue;

		pr_alarm(FLOW, "rtc cmd: 0x%x\n", cmd->cmd);
		switch (cmd->cmd) {
		case RTC_CMD_UPDATE:
			rtc_alarm_update(&cmd->time);
			break;
		case RTC_CMD_CLEAR:
			rtc_alarm_clear();
			break;
		}

		kzfree(cmd);
		cmd = NULL;
	}
	mutex_unlock(&rwork->mutex);

}
#endif
/*   2013-11-19 yuyi Add end for power up alarm */


void set_power_on_alarm(long secs, bool enable)
{
	/*  yuyi add begin just for analysis boot automaticly*/
	#ifdef VENDOR_EDIT
	printk("alarm  set_power_on_alarm time = %ld\n",secs);
	#endif
	/*  yuyi add end just for analysis boot automaticly*/
	mutex_lock(&power_on_alarm_mutex);
	if (enable) {
		power_on_alarm = secs;
	} else {
		if (power_on_alarm && power_on_alarm != secs) {
			pr_alarm(FLOW, "power-off alarm mismatch: \
				previous=%ld, now=%ld\n",
				power_on_alarm, secs);
		}
		else
			power_on_alarm = 0;
	}

	set_alarm_time_to_rtc(power_on_alarm);
	mutex_unlock(&power_on_alarm_mutex);
}


static void update_timer_locked(struct alarm_queue *base, bool head_removed)
{
	struct alarm *alarm;
/*   2013-11-19 yuyi modify begin for power up alarm */
#ifndef VENDOR_EDIT
	bool is_wakeup = base == &alarms[ANDROID_ALARM_RTC_WAKEUP] ||
			base == &alarms[ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP] ||
			base == &alarms[ANDROID_ALARM_RTC_POWEROFF_WAKEUP];
#else
	unsigned long flags;
	struct rtc_cmd *cmd;
	bool is_wakeup = base == &alarms[ANDROID_ALARM_RTC_WAKEUP] ||
			base ==  &alarms[ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP] ||
			base == &alarms[ANDROID_ALARM_RTC_POWERUP]||
			base == &alarms[ANDROID_ALARM_RTC_POWEROFF_WAKEUP];
#endif
/*   2013-11-19 yuyi modify end for power up alarm */

	if (base->stopped) {
		pr_alarm(FLOW, "changed alarm while setting the wall time\n");
		return;
	}

	if (is_wakeup && !suspended && head_removed)
		wake_unlock(&alarm_rtc_wake_lock);

/*   2013-11-19 yuyi modify begin for power up alarm */
#ifndef VENDOR_EDIT
	if (!base->first)
		return;
#else
	if (!base->first) {
		/* There is no more alarm */
		if (base == &alarms[ANDROID_ALARM_RTC_POWERUP]) {
			spin_lock_irqsave(&rtc_work.slock, flags);
			cmd = (struct rtc_cmd*)kzalloc(sizeof(*cmd), GFP_ATOMIC);
			if (cmd != NULL) {
				cmd->cmd = RTC_CMD_CLEAR;
				list_add_tail(&cmd->node, &rtc_work.cmd_list);
			}
			spin_unlock_irqrestore(&rtc_work.slock, flags);

			pr_alarm(FLOW, "schedule rtc alarm clear task\n");
			if (0 == schedule_work(&rtc_work.alarm_task)) {
				pr_alarm(FLOW, "alarm clear task is in queue\n");
			}
		}

		return;
	}
#endif
/*   2013-11-19 yuyi Add end for power up alarm */
	alarm = container_of(base->first, struct alarm, node);

	pr_alarm(FLOW, "selected alarm, type %d, func %pF at %lld\n",
		alarm->type, alarm->function, ktime_to_ns(alarm->expires));

	if (is_wakeup && suspended) {
		pr_alarm(FLOW, "changed alarm while suspened\n");
		wake_lock_timeout(&alarm_rtc_wake_lock, 1 * HZ);
		return;
	}

	hrtimer_try_to_cancel(&base->timer);
	base->timer.node.expires = ktime_add(base->delta, alarm->expires);
	base->timer._softexpires = ktime_add(base->delta, alarm->softexpires);
	hrtimer_start_expires(&base->timer, HRTIMER_MODE_ABS);

/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
	/* mwalker, update new expires time in rtc alarm */
	if (alarm->type == ANDROID_ALARM_RTC_POWERUP) {
		spin_lock_irqsave(&rtc_work.slock, flags);
		cmd = (struct rtc_cmd*)kzalloc(sizeof(*cmd), GFP_ATOMIC);
		if (cmd != NULL) {
			cmd->cmd = RTC_CMD_UPDATE;
			cmd->time = ktime_to_timespec(hrtimer_get_expires(&base->timer));
			list_add_tail(&cmd->node, &rtc_work.cmd_list);
		}
		spin_unlock_irqrestore(&rtc_work.slock, flags);

		pr_alarm(FLOW, "schedule rtc alarm update task\n");
		if (0 == schedule_work(&rtc_work.alarm_task)) {
			pr_alarm(FLOW, "alarm update task is in queue\n");
		}
	}
#endif
/*   2013-11-19 yuyi Add end for power up alarm */
}

static void alarm_enqueue_locked(struct alarm *alarm)
{
	struct alarm_queue *base = &alarms[alarm->type];
	struct rb_node **link = &base->alarms.rb_node;
	struct rb_node *parent = NULL;
	struct alarm *entry;
	int leftmost = 1;
	bool was_first = false;

	pr_alarm(FLOW, "added alarm, type %d, func %pF at %lld\n",
		alarm->type, alarm->function, ktime_to_ns(alarm->expires));

	if (base->first == &alarm->node) {
		base->first = rb_next(&alarm->node);
		was_first = true;
	}
	if (!RB_EMPTY_NODE(&alarm->node)) {
		rb_erase(&alarm->node, &base->alarms);
		RB_CLEAR_NODE(&alarm->node);
	}

	while (*link) {
		parent = *link;
		entry = rb_entry(parent, struct alarm, node);
		/*
		* We dont care about collisions. Nodes with
		* the same expiry time stay together.
		*/
		if (alarm->expires.tv64 < entry->expires.tv64) {
			link = &(*link)->rb_left;
		} else {
			link = &(*link)->rb_right;
			leftmost = 0;
		}
	}
	if (leftmost)
		base->first = &alarm->node;
	if (leftmost || was_first)
		update_timer_locked(base, was_first);

	rb_link_node(&alarm->node, parent, link);
	rb_insert_color(&alarm->node, &base->alarms);
}

/**
 * alarm_init - initialize an alarm
 * @alarm:	the alarm to be initialized
 * @type:	the alarm type to be used
 * @function:	alarm callback function
 */
void alarm_init(struct alarm *alarm,
	enum android_alarm_type type, void (*function)(struct alarm *))
{
	RB_CLEAR_NODE(&alarm->node);
	alarm->type = type;
	alarm->function = function;

	pr_alarm(FLOW, "created alarm, type %d, func %pF\n", type, function);
}


/**
 * alarm_start_range - (re)start an alarm
 * @alarm:	the alarm to be added
 * @start:	earliest expiry time
 * @end:	expiry time
 */
void alarm_start_range(struct alarm *alarm, ktime_t start, ktime_t end)
{
	unsigned long flags;

	spin_lock_irqsave(&alarm_slock, flags);
	alarm->softexpires = start;
	alarm->expires = end;
	alarm_enqueue_locked(alarm);
	spin_unlock_irqrestore(&alarm_slock, flags);
}

/**
 * alarm_try_to_cancel - try to deactivate an alarm
 * @alarm:	alarm to stop
 *
 * Returns:
 *  0 when the alarm was not active
 *  1 when the alarm was active
 * -1 when the alarm may currently be excuting the callback function and
 *    cannot be stopped (it may also be inactive)
 */
int alarm_try_to_cancel(struct alarm *alarm)
{
	struct alarm_queue *base = &alarms[alarm->type];
	unsigned long flags;
	bool first = false;
	int ret = 0;

	spin_lock_irqsave(&alarm_slock, flags);
	if (!RB_EMPTY_NODE(&alarm->node)) {
		pr_alarm(FLOW, "canceled alarm, type %d, func %pF at %lld\n",
			alarm->type, alarm->function,
			ktime_to_ns(alarm->expires));
		ret = 1;
		if (base->first == &alarm->node) {
			base->first = rb_next(&alarm->node);
			first = true;
		}
		rb_erase(&alarm->node, &base->alarms);
		RB_CLEAR_NODE(&alarm->node);
		if (first)
			update_timer_locked(base, true);
	} else
		pr_alarm(FLOW, "tried to cancel alarm, type %d, func %pF\n",
			alarm->type, alarm->function);
	spin_unlock_irqrestore(&alarm_slock, flags);
	if (!ret && hrtimer_callback_running(&base->timer))
		ret = -1;
	return ret;
}

/**
 * alarm_cancel - cancel an alarm and wait for the handler to finish.
 * @alarm:	the alarm to be cancelled
 *
 * Returns:
 *  0 when the alarm was not active
 *  1 when the alarm was active
 */
int alarm_cancel(struct alarm *alarm)
{
	for (;;) {
		int ret = alarm_try_to_cancel(alarm);
		if (ret >= 0)
			return ret;
		cpu_relax();
	}
}

/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
#ifdef DEBUG_PRINT_TIME
/**
 * print_rtc_time - print time in struct rtc_time as man readable
 * @purpose mwalker to debug
 */
static void inline print_rtc_time(struct rtc_time *rt)
{
	pr_alarm(FLOW, "\t--> hh:mm:ss = %02d:%02d:%02d mm/dd/yyyy = %02d/%02d/%04d\n",
		rt->tm_hour, rt->tm_min,
		rt->tm_sec, rt->tm_mon + 1,
		rt->tm_mday, rt->tm_year + 1900);
}
#endif

/**
 * rtc_alarm_update - update alarm time in rtc
 * @author mwalker
 */
static int rtc_alarm_update(struct timespec *alarm)
{
	int ret = 0;
	struct rtc_wkalrm   rtc_alarm;
	struct rtc_time     rtc_current_rtc_time;
	unsigned long       rtc_current_time;
	unsigned long       rtc_alarm_time;
	struct timespec     rtc_delta;
	struct timespec     wall_time;

	wake_lock(&alarm_rtc_wake_lock);
	ret = rtc_read_time(alarm_rtc_dev, &rtc_current_rtc_time);
	if (ret < 0) {
		pr_alarm(ERROR, "%s: Failed to read RTC time\n", __func__);
		goto err;
	}

	getnstimeofday(&wall_time);
	rtc_tm_to_time(&rtc_current_rtc_time, &rtc_current_time);
	set_normalized_timespec(&rtc_delta,
				wall_time.tv_sec - rtc_current_time,
				wall_time.tv_nsec);

	rtc_alarm_time = timespec_sub(*alarm, rtc_delta).tv_sec;

	rtc_time_to_tm(rtc_alarm_time, &rtc_alarm.time);
	rtc_alarm.enabled = 1;

	ret = rtc_set_alarm(alarm_rtc_dev, &rtc_alarm);
	if (ret < 0) {
		pr_alarm(ERROR, "%s: Failed to set rtc alarm\n", __func__);
		goto err;
	}

#ifdef DEBUG_PRINT_TIME
	pr_alarm(FLOW, "%s, new rtc alarm time set from: ", __func__);
	print_rtc_time(&rtc_current_rtc_time);
	pr_alarm(FLOW, "to: ");
	print_rtc_time(&rtc_alarm.time);
#endif

	wake_unlock(&alarm_rtc_wake_lock);
	return 0;

err:
	pr_alarm(ERROR, "%s: rtc alarm will lost!", __func__);
	wake_unlock(&alarm_rtc_wake_lock);
	return -1;
}

//#ifdef CONFIG_ _MODIFY
//use to get rtc times for other driver
int msmrtc_alarm_read_time(struct rtc_time *tm)
{
	int ret=0;

#ifndef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/02/28  Delete for sovle alarm can't sleep */
	wake_lock(&alarm_rtc_wake_lock);
#endif /*CONFIG_VENDOR_EDIT*/
	ret = rtc_read_time(alarm_rtc_dev, tm);
	if (ret < 0) {
		pr_alarm(ERROR, "%s: Failed to read RTC time\n", __func__);
		goto err;
	}

#ifndef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/02/28  Delete for sovle alarm can't sleep */
	wake_unlock(&alarm_rtc_wake_lock);
#endif /*CONFIG_VENDOR_EDIT*/
	return 0;
err:
	pr_alarm(ERROR, "%s: rtc alarm will lost!", __func__);
#ifndef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/02/28  Delete for sovle alarm can't sleep */
	wake_unlock(&alarm_rtc_wake_lock);
#endif /*CONFIG_VENDOR_EDIT*/
	return -1;

}
EXPORT_SYMBOL(msmrtc_alarm_read_time);
//#endif

/**
 * rtc_alarm_clear - clear alarm in rtc register.
 * @author mwalker
 */
static void rtc_alarm_clear(void)
{
	int ret;
	struct rtc_wkalrm rtc_wkalrm_time;

	//pr_alarm(FLOW, "%s\n", __func__);

	wake_lock(&alarm_rtc_wake_lock);

	ret	= rtc_alarm_irq_enable(alarm_rtc_dev, 0);
	if (ret < 0)
		pr_alarm(ERROR, "Disable rtc alarm irq failed!\n");

	memset(&rtc_wkalrm_time, 0, sizeof(rtc_wkalrm_time));
	ret = rtc_set_alarm(alarm_rtc_dev, &rtc_wkalrm_time);
	if (ret < 0)
		pr_alarm(ERROR, "%s(ret %d): "
			"Failed to clear powerup alarm\n", __func__, ret);

	wake_unlock(&alarm_rtc_wake_lock);
}
#endif
/*  2013-11-19 yuyi Add end for power up alarm */

/**
 * alarm_set_rtc - set the kernel and rtc walltime
 * @new_time:	timespec value containing the new time
 */
int alarm_set_rtc(struct timespec new_time)
{
	int i;
	int ret;
	unsigned long flags;
	struct rtc_time rtc_new_rtc_time;
	struct timespec tmp_time;

	rtc_time_to_tm(new_time.tv_sec, &rtc_new_rtc_time);

	pr_alarm(TSET, "set rtc %ld %ld - rtc %02d:%02d:%02d %02d/%02d/%04d\n",
		new_time.tv_sec, new_time.tv_nsec,
		rtc_new_rtc_time.tm_hour, rtc_new_rtc_time.tm_min,
		rtc_new_rtc_time.tm_sec, rtc_new_rtc_time.tm_mon + 1,
		rtc_new_rtc_time.tm_mday,
		rtc_new_rtc_time.tm_year + 1900);

	mutex_lock(&alarm_setrtc_mutex);
	spin_lock_irqsave(&alarm_slock, flags);
	wake_lock(&alarm_rtc_wake_lock);
	getnstimeofday(&tmp_time);
	for (i = 0; i < ANDROID_ALARM_SYSTEMTIME; i++) {
		hrtimer_try_to_cancel(&alarms[i].timer);
		alarms[i].stopped = true;
		alarms[i].stopped_time = timespec_to_ktime(tmp_time);
	}
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
	hrtimer_try_to_cancel(&alarms[ANDROID_ALARM_RTC_POWERUP].timer);
	alarms[ANDROID_ALARM_RTC_POWERUP].stopped = true;
	alarms[ANDROID_ALARM_RTC_POWERUP].stopped_time = timespec_to_ktime(tmp_time);
#endif
/*   2013-11-19 yuyi Add end for power up alarm */
	alarms[ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP].delta =
		alarms[ANDROID_ALARM_ELAPSED_REALTIME].delta =
		ktime_sub(alarms[ANDROID_ALARM_ELAPSED_REALTIME].delta,
			timespec_to_ktime(timespec_sub(tmp_time, new_time)));
	spin_unlock_irqrestore(&alarm_slock, flags);
	ret = do_settimeofday(&new_time);
	spin_lock_irqsave(&alarm_slock, flags);
	for (i = 0; i < ANDROID_ALARM_SYSTEMTIME; i++) {
		alarms[i].stopped = false;
		update_timer_locked(&alarms[i], false);
	}
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
	alarms[ANDROID_ALARM_RTC_POWERUP].stopped = false;
	update_timer_locked(&alarms[ANDROID_ALARM_RTC_POWERUP], false);
#endif
/*   2013-11-19 yuyi Add end for power up alarm */
	spin_unlock_irqrestore(&alarm_slock, flags);
	if (ret < 0) {
		pr_alarm(ERROR, "alarm_set_rtc: Failed to set time\n");
		goto err;
	}
	if (!alarm_rtc_dev) {
		pr_alarm(ERROR,
			"alarm_set_rtc: no RTC, time will be lost on reboot\n");
		goto err;
	}
	ret = rtc_set_time(alarm_rtc_dev, &rtc_new_rtc_time);
	if (ret < 0)
		pr_alarm(ERROR, "alarm_set_rtc: "
			"Failed to set RTC, time will be lost on reboot\n");
err:
	wake_unlock(&alarm_rtc_wake_lock);
	mutex_unlock(&alarm_setrtc_mutex);
	return ret;
}


void
alarm_update_timedelta(struct timespec tmp_time, struct timespec new_time)
{
	int i;
	unsigned long flags;

	spin_lock_irqsave(&alarm_slock, flags);
	for (i = 0; i < ANDROID_ALARM_SYSTEMTIME; i++) {
		hrtimer_try_to_cancel(&alarms[i].timer);
		alarms[i].stopped = true;
		alarms[i].stopped_time = timespec_to_ktime(tmp_time);
	}
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
	hrtimer_try_to_cancel(&alarms[ANDROID_ALARM_RTC_POWERUP].timer);
	alarms[ANDROID_ALARM_RTC_POWERUP].stopped = true;
	alarms[ANDROID_ALARM_RTC_POWERUP].stopped_time = timespec_to_ktime(tmp_time);
#endif
/*   2013-11-19 yuyi Add end for power up alarm */
	alarms[ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP].delta =
		alarms[ANDROID_ALARM_ELAPSED_REALTIME].delta =
		ktime_sub(alarms[ANDROID_ALARM_ELAPSED_REALTIME].delta,
			timespec_to_ktime(timespec_sub(tmp_time, new_time)));
	for (i = 0; i < ANDROID_ALARM_SYSTEMTIME; i++) {
		alarms[i].stopped = false;
		update_timer_locked(&alarms[i], false);
	}
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
	alarms[ANDROID_ALARM_RTC_POWERUP].stopped = false;
	update_timer_locked(&alarms[ANDROID_ALARM_RTC_POWERUP], false);
#endif
/*   2013-11-19 yuyi Add end for power up alarm */
	spin_unlock_irqrestore(&alarm_slock, flags);
}

/**
 * alarm_get_elapsed_realtime - get the elapsed real time in ktime_t format
 *
 * returns the time in ktime_t format
 */
ktime_t alarm_get_elapsed_realtime(void)
{
	ktime_t now;
	unsigned long flags;
	struct alarm_queue *base = &alarms[ANDROID_ALARM_ELAPSED_REALTIME];

	spin_lock_irqsave(&alarm_slock, flags);
	now = base->stopped ? base->stopped_time : ktime_get_real();
	now = ktime_sub(now, base->delta);
	spin_unlock_irqrestore(&alarm_slock, flags);
	return now;
}

static enum hrtimer_restart alarm_timer_triggered(struct hrtimer *timer)
{
	struct alarm_queue *base;
	struct alarm *alarm;
	unsigned long flags;
	ktime_t now;

	spin_lock_irqsave(&alarm_slock, flags);

	base = container_of(timer, struct alarm_queue, timer);
	now = base->stopped ? base->stopped_time : hrtimer_cb_get_time(timer);
	now = ktime_sub(now, base->delta);

	pr_alarm(INT, "alarm_timer_triggered type %d at %lld\n",
		base - alarms, ktime_to_ns(now));

	while (base->first) {
		alarm = container_of(base->first, struct alarm, node);
		if (alarm->softexpires.tv64 > now.tv64) {
			pr_alarm(FLOW, "don't call alarm, %pF, %lld (s %lld)\n",
				alarm->function, ktime_to_ns(alarm->expires),
				ktime_to_ns(alarm->softexpires));
			break;
		}
		base->first = rb_next(&alarm->node);
		rb_erase(&alarm->node, &base->alarms);
		RB_CLEAR_NODE(&alarm->node);
		pr_alarm(CALL, "call alarm, type %d, func %pF, %lld (s %lld)\n",
			alarm->type, alarm->function,
			ktime_to_ns(alarm->expires),
			ktime_to_ns(alarm->softexpires));
		spin_unlock_irqrestore(&alarm_slock, flags);
		alarm->function(alarm);
		spin_lock_irqsave(&alarm_slock, flags);
	}
	if (!base->first)
		pr_alarm(FLOW, "no more alarms of type %d\n", base - alarms);
	update_timer_locked(base, true);
	spin_unlock_irqrestore(&alarm_slock, flags);
	return HRTIMER_NORESTART;
}

static void alarm_triggered_func(void *p)
{
	struct rtc_device *rtc = alarm_rtc_dev;
	if (!(rtc->irq_data & RTC_AF))
		return;
	pr_alarm(INT, "rtc alarm triggered\n");
	wake_lock_timeout(&alarm_rtc_wake_lock, 1 * HZ);
}

static int alarm_suspend(struct platform_device *pdev, pm_message_t state)
{
	int                 err = 0;
	unsigned long       flags;
	struct rtc_wkalrm   rtc_alarm;
	struct rtc_time     rtc_current_rtc_time;
	unsigned long       rtc_current_time;
	unsigned long       rtc_alarm_time;
	struct timespec     rtc_delta;
	struct timespec     wall_time;
	struct alarm_queue *wakeup_queue = NULL;
	struct alarm_queue *tmp_queue = NULL;

	pr_alarm(SUSPEND, "alarm_suspend(%p, %d)\n", pdev, state.event);

	spin_lock_irqsave(&alarm_slock, flags);
	suspended = true;
	spin_unlock_irqrestore(&alarm_slock, flags);

	hrtimer_cancel(&alarms[ANDROID_ALARM_RTC_WAKEUP].timer);
	hrtimer_cancel(&alarms[
			ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP].timer);
/*   2013-11-19 yuyi Add end for power up alarm */
#ifdef VENDOR_EDIT
	hrtimer_cancel(&alarms[ANDROID_ALARM_RTC_POWERUP].timer); /* mwalker */
#endif
/*   2013-11-19 yuyi Add end for power up alarm */
	hrtimer_cancel(&alarms[
			ANDROID_ALARM_RTC_POWEROFF_WAKEUP].timer);

	tmp_queue = &alarms[ANDROID_ALARM_RTC_WAKEUP];
	if (tmp_queue->first)
		wakeup_queue = tmp_queue;

	tmp_queue = &alarms[ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP];
	if (tmp_queue->first && (!wakeup_queue ||
				hrtimer_get_expires(&tmp_queue->timer).tv64 <
				hrtimer_get_expires(&wakeup_queue->timer).tv64))
		wakeup_queue = tmp_queue;
/*   2013-11-19 yuyi Add begin for power up alarm */
	tmp_queue = &alarms[ANDROID_ALARM_RTC_POWERUP];
	if (tmp_queue->first && (!wakeup_queue ||
				hrtimer_get_expires(&tmp_queue->timer).tv64 <
				hrtimer_get_expires(&wakeup_queue->timer).tv64))
		wakeup_queue = tmp_queue;
/*   2013-11-19 yuyi Add end for power up alarm */
	tmp_queue = &alarms[ANDROID_ALARM_RTC_POWEROFF_WAKEUP];
	if (tmp_queue->first && (!wakeup_queue ||
				hrtimer_get_expires(&tmp_queue->timer).tv64 <
				hrtimer_get_expires(&wakeup_queue->timer).tv64))
		wakeup_queue = tmp_queue;

	if (wakeup_queue) {
		rtc_read_time(alarm_rtc_dev, &rtc_current_rtc_time);
		getnstimeofday(&wall_time);
		rtc_tm_to_time(&rtc_current_rtc_time, &rtc_current_time);
		set_normalized_timespec(&rtc_delta,
					wall_time.tv_sec - rtc_current_time,
					wall_time.tv_nsec);

		rtc_alarm_time = timespec_sub(ktime_to_timespec(
			hrtimer_get_expires(&wakeup_queue->timer)),
			rtc_delta).tv_sec;

		rtc_time_to_tm(rtc_alarm_time, &rtc_alarm.time);
		rtc_alarm.enabled = 1;
		rtc_set_alarm(alarm_rtc_dev, &rtc_alarm);
		rtc_read_time(alarm_rtc_dev, &rtc_current_rtc_time);
		rtc_tm_to_time(&rtc_current_rtc_time, &rtc_current_time);
		pr_alarm(SUSPEND,
			"rtc alarm set at %ld, now %ld, rtc delta %ld.%09ld\n",
			rtc_alarm_time, rtc_current_time,
			rtc_delta.tv_sec, rtc_delta.tv_nsec);
		if (rtc_current_time + 1 >= rtc_alarm_time) {
			pr_alarm(SUSPEND, "alarm about to go off\n");
			rtc_time_to_tm(0, &rtc_alarm.time);
			rtc_alarm.enabled = 0;
			rtc_set_alarm(alarm_rtc_dev, &rtc_alarm);

			spin_lock_irqsave(&alarm_slock, flags);
			suspended = false;
			wake_lock_timeout(&alarm_rtc_wake_lock, 2 * HZ);
			update_timer_locked(&alarms[ANDROID_ALARM_RTC_WAKEUP],
									false);
			update_timer_locked(&alarms[
				ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP], false);
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
			update_timer_locked(&alarms[ANDROID_ALARM_RTC_POWERUP],
									false);	/* mwalker */
#endif
/*   2013-11-19 yuyi Add end for power up alarm */
			update_timer_locked(&alarms[
					ANDROID_ALARM_RTC_POWEROFF_WAKEUP], false);
			err = -EBUSY;
			spin_unlock_irqrestore(&alarm_slock, flags);
		}
	}
	return err;
}

static int alarm_resume(struct platform_device *pdev)
{
	struct rtc_wkalrm alarm;
	unsigned long       flags;

	pr_alarm(SUSPEND, "alarm_resume(%p)\n", pdev);

	rtc_time_to_tm(0, &alarm.time);
	alarm.enabled = 0;
	rtc_set_alarm(alarm_rtc_dev, &alarm);

	spin_lock_irqsave(&alarm_slock, flags);
	suspended = false;
	update_timer_locked(&alarms[ANDROID_ALARM_RTC_WAKEUP], false);
	update_timer_locked(&alarms[ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP],
									false);
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
	update_timer_locked(&alarms[ANDROID_ALARM_RTC_POWERUP], false); /* mwalker */
#endif
/*   2013-11-19 yuyi Add end for power up alarm */
	update_timer_locked(&alarms[ANDROID_ALARM_RTC_POWEROFF_WAKEUP],
									false);
	spin_unlock_irqrestore(&alarm_slock, flags);

	set_alarm_time_to_rtc(power_on_alarm);
	return 0;
}

static int set_alarm_time_to_rtc(const long power_on_time)
{
	struct timespec wall_time;
	struct rtc_time rtc_time;
	struct rtc_wkalrm alarm;
	long rtc_secs, alarm_delta, alarm_time;
	int rc = -EINVAL;

	if (power_on_time <= 0) {
		goto disable_alarm;
	}

	rtc_read_time(alarm_rtc_dev, &rtc_time);
	getnstimeofday(&wall_time);
	rtc_tm_to_time(&rtc_time, &rtc_secs);
	alarm_delta = wall_time.tv_sec - rtc_secs;
	alarm_time = power_on_time - alarm_delta;

	/*
	 * Substract ALARM_DELTA from actual alarm time
	 * to powerup the device before actual alarm
	 * expiration.
	 */
	if ((alarm_time - ALARM_DELTA) > rtc_secs)
		alarm_time -= ALARM_DELTA;

	if (alarm_time <= rtc_secs)
		goto disable_alarm;

	rtc_time_to_tm(alarm_time, &alarm.time);
	alarm.enabled = 1;
	rc = rtc_set_alarm(alarm_rtc_dev, &alarm);
	if (rc){
		pr_alarm(ERROR, "Unable to set power-on alarm\n");
		goto disable_alarm;
	}
	else
		pr_alarm(FLOW, "Power-on alarm set to %lu\n",
				alarm_time);

	return 0;

disable_alarm:
	rtc_alarm_irq_enable(alarm_rtc_dev, 0);
	return rc;
}

static struct rtc_task alarm_rtc_task = {
	.func = alarm_triggered_func
};

static int rtc_alarm_add_device(struct device *dev,
				struct class_interface *class_intf)
{
	int err;
	struct rtc_device *rtc = to_rtc_device(dev);

	mutex_lock(&alarm_setrtc_mutex);

	if (alarm_rtc_dev) {
		err = -EBUSY;
		goto err1;
	}

	alarm_platform_dev =
		platform_device_register_simple("alarm", -1, NULL, 0);
	if (IS_ERR(alarm_platform_dev)) {
		err = PTR_ERR(alarm_platform_dev);
		goto err2;
	}
	err = rtc_irq_register(rtc, &alarm_rtc_task);
	if (err)
		goto err3;
	alarm_rtc_dev = rtc;
	pr_alarm(INIT_STATUS, "using rtc device, %s, for alarms", rtc->name);
	mutex_unlock(&alarm_setrtc_mutex);

/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
	// mwalker active rtc task work
	rtc_work.active = true;
#endif
/*   2013-11-19 yuyi Add end for power up alarm */

	return 0;

err3:
	platform_device_unregister(alarm_platform_dev);
err2:
err1:
	mutex_unlock(&alarm_setrtc_mutex);
	return err;
}

static void rtc_alarm_remove_device(struct device *dev,
				    struct class_interface *class_intf)
{
	if (dev == &alarm_rtc_dev->dev) {
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
		// mwalker clear rtc alarm queue
		rtc_work.active = false;
		cancel_work_sync(&rtc_work.alarm_task);
#endif
/*   2013-11-19 yuyi Add end for power up alarm */

		pr_alarm(INIT_STATUS, "lost rtc device for alarms");
		rtc_irq_unregister(alarm_rtc_dev, &alarm_rtc_task);
		platform_device_unregister(alarm_platform_dev);
		alarm_rtc_dev = NULL;
	}
}

static struct class_interface rtc_alarm_interface = {
	.add_dev = &rtc_alarm_add_device,
	.remove_dev = &rtc_alarm_remove_device,
};

static struct platform_driver alarm_driver = {
	.suspend = alarm_suspend,
	.resume = alarm_resume,
	.driver = {
		.name = "alarm"
	}
};

static int __init alarm_late_init(void)
{
	unsigned long   flags;
	struct timespec tmp_time, system_time;

	/* this needs to run after the rtc is read at boot */
	spin_lock_irqsave(&alarm_slock, flags);
	/* We read the current rtc and system time so we can later calulate
	 * elasped realtime to be (boot_systemtime + rtc - boot_rtc) ==
	 * (rtc - (boot_rtc - boot_systemtime))
	 */
	getnstimeofday(&tmp_time);
	ktime_get_ts(&system_time);
	alarms[ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP].delta =
		alarms[ANDROID_ALARM_ELAPSED_REALTIME].delta =
			timespec_to_ktime(timespec_sub(tmp_time, system_time));

	spin_unlock_irqrestore(&alarm_slock, flags);
	return 0;
}

static int __init alarm_driver_init(void)
{
	int err;
	int i;

	for (i = 0; i < ANDROID_ALARM_SYSTEMTIME; i++) {
		hrtimer_init(&alarms[i].timer,
				CLOCK_REALTIME, HRTIMER_MODE_ABS);
		alarms[i].timer.function = alarm_timer_triggered;
	}
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
	hrtimer_init(&alarms[ANDROID_ALARM_RTC_POWERUP].timer,
				CLOCK_REALTIME, HRTIMER_MODE_ABS);
	alarms[ANDROID_ALARM_RTC_POWERUP].timer.function = alarm_timer_triggered;
#endif
/*   2013-11-19 yuyi Add end for power up alarm */
	hrtimer_init(&alarms[ANDROID_ALARM_SYSTEMTIME].timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	alarms[ANDROID_ALARM_SYSTEMTIME].timer.function = alarm_timer_triggered;
	err = platform_driver_register(&alarm_driver);
	if (err < 0)
		goto err1;
	wake_lock_init(&alarm_rtc_wake_lock, WAKE_LOCK_SUSPEND, "alarm_rtc");

/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
	// mwalker
	INIT_LIST_HEAD(&rtc_work.cmd_list);
	INIT_WORK(&rtc_work.alarm_task, rtc_task);
	mutex_init(&rtc_work.mutex);
	spin_lock_init(&rtc_work.slock);
#endif
/*   2013-11-19 yuyi Add end for power up alarm */

	rtc_alarm_interface.class = rtc_class;
	err = class_interface_register(&rtc_alarm_interface);
	if (err < 0)
		goto err2;

	return 0;

err2:
	wake_lock_destroy(&alarm_rtc_wake_lock);
	platform_driver_unregister(&alarm_driver);
err1:
	return err;
}

static void  __exit alarm_exit(void)
{
/*   2013-11-19 yuyi Add begin for power up alarm */
#ifdef VENDOR_EDIT
	// mwalker make clean
	unsigned long flags;
	struct rtc_cmd *cmd;
	struct list_head *list_pos, *list_pos_tmp;

	rtc_work.active = false;
	cancel_work_sync(&rtc_work.alarm_task);

	spin_lock_irqsave(&rtc_work.slock, flags);
	list_for_each_safe(list_pos, list_pos_tmp, &rtc_work.cmd_list) {
		cmd = list_entry(list_pos, struct rtc_cmd, node);
		list_del(list_pos);
		kzfree(cmd);
	}
	spin_unlock_irqrestore(&rtc_work.slock, flags);

	mutex_destroy(&rtc_work.mutex);
#endif
/*   2013-11-19 yuyi Add end for power up alarm */

	class_interface_unregister(&rtc_alarm_interface);
	wake_lock_destroy(&alarm_rtc_wake_lock);
	platform_driver_unregister(&alarm_driver);
}

late_initcall(alarm_late_init);
module_init(alarm_driver_init);
module_exit(alarm_exit);

