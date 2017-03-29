/*
 * pps-gmtimer.c -- PPS client driver using OMAP Timers
 *
 * Copyright (C) 2014  Daniel Drown <dan-android@drown.org>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * ref1 - http://www.kunen.org/uC/beagle/omap_dmtimer.html
 * ref2 - [kernel] linux/samples/kobject/kobject-example.c
 * ref3 - [kernel] linux/arch/arm/mach-omap2/timer.c
 * ref4 - am335x trm on ti's website
 * ref5 - linux/drivers/pps/clients/pps-gpio.c
 *
 * required in kernel config - CONFIG_OF
 */

#define MODULE_NAME "pps-gmtimer"
#define pr_fmt(fmt) MODULE_NAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pps_kernel.h>
#include <linux/clocksource.h>
#include <linux/time.h>

#include <plat/dmtimer.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dan Drown");
MODULE_DESCRIPTION("PPS Client Driver using OMAP Timer hardware");
MODULE_VERSION("0.1.0");

/* Sanity checks */
#define MINIMUM_DT_FREQUENCY 10000000
#define MAXIMUM_DT_FREQUENCY 24000000

struct pps_gmtimer_platform_data {
  struct clocksource clksrc;  // First since it may be used frequently
  struct omap_dm_timer *capture_timer;
  uint32_t frequency;
  unsigned int capture;
  unsigned int overflow;
  unsigned int count_at_interrupt;
  unsigned int capture_at_interrupt;
  unsigned int capture_diff;
  unsigned int capture_spread;
  unsigned int interrupt_delay;
  unsigned int dt_frequency;
  const char *timer_name;
  struct pps_event_time ts;
  struct pps_event_time ts_last, ts_prev;
  struct timespec delta;
  struct pps_device *pps;
  struct pps_source_info info;
  int is_clocksource;  //FIXME: Create flags cell
};

/* helpers *******************/

/*
 * Read timer counter inline, ignoring "posting" (as clocksource does).
 * Since the "posted" check, when used, only checks for a posted write
 * to the same register, and since we never write the counter, the check
 * would just be a pointless (and time-wasting) extra register read.
 */
static inline unsigned int read_timer_counter(struct omap_dm_timer *timer)
{
  return __omap_dm_timer_read_counter(timer, 0);
}

/* kobject *******************/
static ssize_t timer_name_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%s\n", pdata->timer_name);
}

static DEVICE_ATTR(timer_name, S_IRUGO, timer_name_show, NULL);

static ssize_t stats_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "capture: %u\noverflow: %u\n", pdata->capture, pdata->overflow);
}

static DEVICE_ATTR(stats, S_IRUGO, stats_show, NULL);

static ssize_t interrupt_delta_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%lld.%09ld\n", (long long)pdata->delta.tv_sec, pdata->delta.tv_nsec);
}

static DEVICE_ATTR(interrupt_delta, S_IRUGO, interrupt_delta_show, NULL);

static ssize_t pps_ts_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%lld.%09ld\n", (long long)pdata->ts.ts_real.tv_sec, pdata->ts.ts_real.tv_nsec);
}

static DEVICE_ATTR(pps_ts, S_IRUGO, pps_ts_show, NULL);

static ssize_t count_at_interrupt_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%u\n", pdata->count_at_interrupt);
}

static DEVICE_ATTR(count_at_interrupt, S_IRUGO, count_at_interrupt_show, NULL);

static ssize_t capture_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%u\n", pdata->capture_at_interrupt);
}

static DEVICE_ATTR(capture, S_IRUGO, capture_show, NULL);

static ssize_t ctrlstatus_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%x\n",
      __omap_dm_timer_read(pdata->capture_timer, OMAP_TIMER_CTRL_REG, pdata->capture_timer->posted)
      );
}

static DEVICE_ATTR(ctrlstatus, S_IRUGO, ctrlstatus_show, NULL);

static ssize_t timer_counter_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  unsigned int current_count = 0;

  current_count = read_timer_counter(pdata->capture_timer);
  return sprintf(buf, "%u\n", current_count);
}

static DEVICE_ATTR(timer_counter, S_IRUGO, timer_counter_show, NULL);

static ssize_t frequency_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%u\n", pdata->frequency);
}

static DEVICE_ATTR(frequency, S_IRUGO, frequency_show, NULL);

static ssize_t capture_diff_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%u\n", pdata->capture_diff);
}

static DEVICE_ATTR(capture_diff, S_IRUGO, capture_diff_show, NULL);

static ssize_t capture_uncertainty_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  struct clocksource *clk = &pdata->clksrc;
  uint32_t nanos = clocksource_cyc2ns(pdata->capture_spread,
                                      clk->mult, clk->shift);
  return sprintf(buf, "0.%.09u\n", nanos);
}

static DEVICE_ATTR(capture_uncertainty, S_IRUGO, capture_uncertainty_show, NULL);

static int print_pps(struct pps_event_time *ts, char *buf)
{
#ifdef CONFIG_NTP_PPS
  return sprintf(buf, "%lld.%09ld, %lld.%09ld\n", (long long) ts->ts_real.tv_sec, ts->ts_real.tv_nsec, (long long) ts->ts_raw.tv_sec, ts->ts_raw.tv_nsec);
#else /* CONFIG_NTP_PPS */
  return sprintf(buf, "%lld.%09ld\n", (long long) ts->ts_real.tv_sec, ts->ts_real.tv_nsec);
#endif /* CONFIG_NTP_PPS */
}

static ssize_t interrupt_ts_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return print_pps(&pdata->ts_last, buf);
}

static DEVICE_ATTR(interrupt_ts, S_IRUGO, interrupt_ts_show, NULL);

static ssize_t interrupt_prev_ts_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return print_pps(&pdata->ts_prev, buf);
}

static DEVICE_ATTR(interrupt_prev_ts, S_IRUGO, interrupt_prev_ts_show, NULL);

static ssize_t interrupt_delay_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%u\n", pdata->interrupt_delay);
}

static DEVICE_ATTR(interrupt_delay, S_IRUGO, interrupt_delay_show, NULL);

static ssize_t devtree_frequency_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%u\n", pdata->dt_frequency);
}

static DEVICE_ATTR(devtree_frequency, S_IRUGO, devtree_frequency_show, NULL);

static struct attribute *attrs[] = {
   &dev_attr_timer_counter.attr,
   &dev_attr_ctrlstatus.attr,
   &dev_attr_capture.attr,
   &dev_attr_count_at_interrupt.attr,
   &dev_attr_pps_ts.attr,
   &dev_attr_interrupt_delta.attr,
   &dev_attr_stats.attr,
   &dev_attr_timer_name.attr,
   &dev_attr_frequency.attr,
   &dev_attr_capture_diff.attr,
   &dev_attr_capture_uncertainty.attr,
   &dev_attr_interrupt_ts.attr,
   &dev_attr_interrupt_prev_ts.attr,
   &dev_attr_interrupt_delay.attr,
   &dev_attr_devtree_frequency.attr,
   NULL,
};

static struct attribute_group attr_group = {
   .attrs = attrs,
};

/* timers ********************/
static irqreturn_t pps_gmtimer_interrupt(int irq, void *data) {
  struct pps_gmtimer_platform_data *pdata;

  pdata = data;

  unsigned int irq_status;

  irq_status = omap_dm_timer_read_status(pdata->capture_timer);
  if(irq_status & OMAP_TIMER_INT_CAPTURE) {
    unsigned int count_at_capture, first, before, after, spread, delta;

    /*
     * The first read is just for cache warmup, but we might as well
     * use it for the latency measurement.
     */
    first = read_timer_counter(pdata->capture_timer);
    /* Do a throwaway pps_get_ts for cache warmup */
    pps_get_ts(&pdata->ts);

    /*
     * Now do a "sandwich read" of the counter and the system time,
     * with a warm cache to make it as tight as possible.
     */
    before = read_timer_counter(pdata->capture_timer);
    pps_get_ts(&pdata->ts);
    after = read_timer_counter(pdata->capture_timer);

    pdata->ts_prev = pdata->ts_last;
    pdata->ts_last = pdata->ts;

    /*
     * If C is the captured value, B is before, and A is after, then:
     * Min offset = B - (C + 1)
     * Max offset = (A + 1) - C
     * Total variation = max - min = A - B + 2
     * Mean offset = (min + max) / 2 = (A + B) / 2 - C
     */
    spread = after - before;
    pdata->capture_spread = spread + 2;
    pdata->count_at_interrupt = before + ((spread + 1) >> 1);

    count_at_capture = __omap_dm_timer_read(pdata->capture_timer,
                                            OMAP_TIMER_CAPTURE_REG, 0);
    pdata->capture_diff = count_at_capture - pdata->capture_at_interrupt;
    pdata->capture_at_interrupt = count_at_capture;

    pdata->interrupt_delay = first - count_at_capture;

    delta = pdata->count_at_interrupt - count_at_capture;

    pdata->delta.tv_sec = 0;
    pdata->delta.tv_nsec = clocksource_cyc2ns(delta,
                                              pdata->clksrc.mult,
                                              pdata->clksrc.shift);

    pps_sub_ts(&pdata->ts, pdata->delta);
    pps_event(pdata->pps, &pdata->ts, PPS_CAPTUREASSERT, NULL);

    pdata->capture++;

    __omap_dm_timer_write_status(pdata->capture_timer, OMAP_TIMER_INT_CAPTURE);
  }
  if(irq_status & OMAP_TIMER_INT_OVERFLOW) {
    pdata->overflow++;
    __omap_dm_timer_write_status(pdata->capture_timer, OMAP_TIMER_INT_OVERFLOW);
  }

  return IRQ_HANDLED; // TODO: shared interrupts?
}

static void omap_dm_timer_setup_capture(struct omap_dm_timer *timer) {
  u32 ctrl;

  omap_dm_timer_set_source(timer, OMAP_TIMER_SRC_SYS_CLK);

  omap_dm_timer_enable(timer);

  ctrl = __omap_dm_timer_read(timer, OMAP_TIMER_CTRL_REG, timer->posted);

  // disable prescaler
  ctrl &= ~(OMAP_TIMER_CTRL_PRE | (0x07 << 2));

  // autoreload
  ctrl |= OMAP_TIMER_CTRL_AR;
  __omap_dm_timer_write(timer, OMAP_TIMER_LOAD_REG, 0, timer->posted);

  // start timer
  ctrl |= OMAP_TIMER_CTRL_ST;

  // set capture
  ctrl |= OMAP_TIMER_CTRL_TCM_LOWTOHIGH | OMAP_TIMER_CTRL_GPOCFG; // TODO: configurable direction

  __omap_dm_timer_load_start(timer, ctrl, 0, timer->posted);

  /* Save the context */
  timer->context.tclr = ctrl;
  timer->context.tldr = 0;
  timer->context.tcrr = 0;
}

/* if tclkin has no clock, writes to the timer registers will stall and you will get a message like:
 * Unhandled fault: external abort on non-linefetch (0x1028) at 0xfa044048
 */
static int omap_dm_timer_use_tclkin(struct pps_gmtimer_platform_data *pdata) {
  struct clk *gt_fclk;
  unsigned int frequency;
  int badfreq = 0;

  omap_dm_timer_set_source(pdata->capture_timer, OMAP_TIMER_SRC_EXT_CLK);
  frequency = pdata->dt_frequency;
  if (frequency < MINIMUM_DT_FREQUENCY || frequency > MAXIMUM_DT_FREQUENCY) {
    gt_fclk = omap_dm_timer_get_fclk(pdata->capture_timer);
    frequency = clk_get_rate(gt_fclk);
    badfreq = 1;
  }
  pdata->frequency = frequency;
  pr_info("timer(%s) switched to tclkin, %s rate=%uHz\n",
          pdata->timer_name, badfreq ? "assumed" : "device-tree", frequency);
  return badfreq;
}

static void pps_gmtimer_enable_irq(struct pps_gmtimer_platform_data *pdata) {
  unsigned int interrupt_mask;

  interrupt_mask = OMAP_TIMER_INT_CAPTURE|OMAP_TIMER_INT_OVERFLOW;
  __omap_dm_timer_int_enable(pdata->capture_timer, interrupt_mask);
  pdata->capture_timer->context.tier = interrupt_mask;
  pdata->capture_timer->context.twer = interrupt_mask;
}

static int pps_gmtimer_init_timer(struct device_node *timer_dn, struct pps_gmtimer_platform_data *pdata) {
  struct clk *gt_fclk;

  of_property_read_string_index(timer_dn, "ti,hwmods", 0, &pdata->timer_name);
  if (!pdata->timer_name) {
    pr_err("ti,hwmods property missing?\n");
    return -ENODEV;
  }

  pdata->capture_timer = omap_dm_timer_request_by_node(timer_dn);
  if(!pdata->capture_timer) {
    pr_err("request_by_node failed\n");
    return -ENODEV;
  }

  // TODO: use devm_request_irq?
  if(request_irq(pdata->capture_timer->irq, pps_gmtimer_interrupt, IRQF_TIMER, MODULE_NAME, pdata)) {
    pr_err("cannot register IRQ %d\n", pdata->capture_timer->irq);
    return -EIO;
  }

  omap_dm_timer_setup_capture(pdata->capture_timer);

  gt_fclk = omap_dm_timer_get_fclk(pdata->capture_timer);
  pdata->frequency = clk_get_rate(gt_fclk);

  pdata->capture_at_interrupt = 0;
  pdata->capture_diff = 0;
  pdata->capture_spread = 0;
  pdata->is_clocksource = 0;

  pr_info("timer name=%s rate=%uHz\n", pdata->timer_name, pdata->frequency);

  return 0;
}

static void pps_gmtimer_cleanup_timer(struct pps_gmtimer_platform_data *pdata) {
  if(pdata->capture_timer) {
    omap_dm_timer_set_source(pdata->capture_timer, OMAP_TIMER_SRC_SYS_CLK); // in case TCLKIN is stopped during boot
    omap_dm_timer_set_int_disable(pdata->capture_timer, OMAP_TIMER_INT_CAPTURE|OMAP_TIMER_INT_OVERFLOW);
    free_irq(pdata->capture_timer->irq, pdata);
    omap_dm_timer_stop(pdata->capture_timer);
    omap_dm_timer_free(pdata->capture_timer);
    pdata->capture_timer = NULL;
    pr_info("Exiting.\n");
  }
}

/* clocksource ***************/

static cycle_t pps_gmtimer_read_cycles(struct clocksource *cs) {
  struct pps_gmtimer_platform_data *pdata = container_of(cs, struct pps_gmtimer_platform_data, clksrc);
  return (cycle_t) read_timer_counter(pdata->capture_timer);
}

static void pps_gmtimer_clocksource_init(struct pps_gmtimer_platform_data *pdata, int badfreq) {
  pdata->clksrc.name = pdata->timer_name;

  pdata->clksrc.rating = 299;
  pdata->clksrc.read = pps_gmtimer_read_cycles;
  pdata->clksrc.mask = CLOCKSOURCE_MASK(32);
  pdata->clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;

  if (badfreq) {
    // Don't register it as a clocksource, but set up conversion factors.
    clocks_calc_mult_shift(&pdata->clksrc.mult, &pdata->clksrc.shift,
                           pdata->frequency, NSEC_PER_SEC, 1);
    return;
  }
  if (clocksource_register_hz(&pdata->clksrc, pdata->frequency)) {
    pr_err("Could not register clocksource %s\n", pdata->clksrc.name);
  } else {
    pr_info("clocksource: %s at %u Hz\n", pdata->clksrc.name, pdata->frequency);
    pdata->is_clocksource = 1;
  }
}

static void pps_gmtimer_clocksource_cleanup(struct pps_gmtimer_platform_data *pdata) {
  if(pdata->is_clocksource) {
    clocksource_unregister(&pdata->clksrc);
    pdata->is_clocksource = 0;
  }
}

/* module ********************/
static struct pps_gmtimer_platform_data *of_get_pps_gmtimer_pdata(struct platform_device *pdev) {
  struct device_node *np = pdev->dev.of_node, *timer_dn;
  struct pps_gmtimer_platform_data *pdata;
  const __be32 *timer_phandle;

  pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
  if (!pdata)
    return NULL;

  timer_phandle = of_get_property(np, "timer", NULL);
  if(!timer_phandle) {
    pr_err("timer property in devicetree null\n");
    goto fail;
  }

  timer_dn = of_find_node_by_phandle(be32_to_cpup(timer_phandle));
  if(!timer_dn) {
    pr_err("find_node_by_phandle failed\n");
    goto fail;
  }

  if(pps_gmtimer_init_timer(timer_dn, pdata) < 0) {
    goto fail2;
  }

  of_node_put(timer_dn);

  return pdata;

fail2:
  of_node_put(timer_dn);
fail:
  devm_kfree(&pdev->dev, pdata);
  return NULL;
}

static const struct of_device_id pps_gmtimer_dt_ids[] = {
	{ .compatible = "pps-gmtimer", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pps_gmtimer_dt_ids);

static int pps_gmtimer_probe(struct platform_device *pdev) {
  const struct of_device_id *match;
  struct pps_gmtimer_platform_data *pdata;
  struct pinctrl *pinctrl;
  const __be32 *use_tclkin, *of_freq;
  int badfreq = 0;

  match = of_match_device(pps_gmtimer_dt_ids, &pdev->dev);
  if (match) {
    pdev->dev.platform_data = of_get_pps_gmtimer_pdata(pdev);
  } else {
    pr_err("of_match_device failed\n");
  }
  pdata = pdev->dev.platform_data;
  if(!pdata)
    return -ENODEV;

  if(sysfs_create_group(&pdev->dev.kobj, &attr_group)) {
    pr_err("sysfs_create_group failed\n");
  }

  pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
  if (IS_ERR(pinctrl))
    pr_warning("pins are not configured from the driver\n");

  of_freq = of_get_property(pdev->dev.of_node, "frequency", NULL);
  if (of_freq) {
    pdata->dt_frequency = be32_to_cpup(of_freq);
  } else {
    pdata->dt_frequency  = 0;
  }

  use_tclkin = of_get_property(pdev->dev.of_node, "use-tclkin", NULL);
  if(use_tclkin && be32_to_cpup(use_tclkin) == 1) {
    badfreq = omap_dm_timer_use_tclkin(pdata);
  } else {
    pr_info("using system clock\n");
  }

  pdata->info.mode = PPS_CAPTUREASSERT | PPS_ECHOASSERT | PPS_CANWAIT | PPS_TSFMT_TSPEC;
  pdata->info.owner = THIS_MODULE;
  snprintf(pdata->info.name, PPS_MAX_NAME_LEN - 1, "%s", pdata->timer_name);

  pdata->pps = pps_register_source(&pdata->info, PPS_CAPTUREASSERT);
  if (pdata->pps == NULL) {
    pr_err("failed to register %s as PPS source\n", pdata->timer_name);
  } else {

    pps_gmtimer_clocksource_init(pdata, badfreq);

    pps_gmtimer_enable_irq(pdata);
  }

  return 0;
}

static int pps_gmtimer_remove(struct platform_device *pdev) {
  struct pps_gmtimer_platform_data *pdata;
  pdata = pdev->dev.platform_data;

  if(pdata) {
    pps_gmtimer_clocksource_cleanup(pdata);

    pps_gmtimer_cleanup_timer(pdata);

    if(pdata->pps) {
      pps_unregister_source(pdata->pps);
      pdata->pps = NULL;
    }

    devm_kfree(&pdev->dev, pdata);
    pdev->dev.platform_data = NULL;

    sysfs_remove_group(&pdev->dev.kobj, &attr_group);
  }

  platform_set_drvdata(pdev, NULL);

  return 0;
}

static struct platform_driver pps_gmtimer_driver = {
	.probe		= pps_gmtimer_probe,
	.remove		= pps_gmtimer_remove,
	.driver		= {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(pps_gmtimer_dt_ids),
	},
};

module_platform_driver(pps_gmtimer_driver);
