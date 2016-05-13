/*
 * Copyright (C) 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/drmP.h>
#include <drm/tinydrm/tinydrm.h>
#include <linux/backlight.h>
#include <linux/pm.h>
#include <linux/spi/spi.h>

void tinydrm_merge_clips(struct drm_clip_rect *dst,
			 struct drm_clip_rect *src, unsigned num_clips,
			 unsigned flags, u32 width, u32 height)
{
	unsigned int i;

	if (!src || !num_clips) {
		dst->x1 = 0;
		dst->x2 = width;
		dst->y1 = 0;
		dst->y2 = height;
		return;
	}

	dst->x1 = dst->y1 = ~0;
	dst->x2 = dst->y2 = 0;

	for (i = 0; i < num_clips; i++) {
		if (flags & DRM_MODE_FB_DIRTY_ANNOTATE_COPY)
			i++;
		dst->x1 = min(dst->x1, src[i].x1);
		dst->x2 = max(dst->x2, src[i].x2);
		dst->y1 = min(dst->y1, src[i].y1);
		dst->y2 = max(dst->y2, src[i].y2);
	}

	if (dst->x2 > width || dst->y2 > height ||
	    dst->x1 >= dst->x2 || dst->y1 >= dst->y2) {
		DRM_DEBUG_KMS("Illegal clip: x1=%u, x2=%u, y1=%u, y2=%u\n",
			      dst->x1, dst->x2, dst->y1, dst->y2);
		dst->x1 = dst->y1 = 0;
		dst->x2 = width;
		dst->y2 = height;
	}
}
EXPORT_SYMBOL(tinydrm_merge_clips);

struct backlight_device *tinydrm_of_find_backlight(struct device *dev)
{
	struct backlight_device *backlight;
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, "backlight", 0);
	if (!np)
		return NULL;

	backlight = of_find_backlight_by_node(np);
	of_node_put(np);

	if (!backlight)
		return ERR_PTR(-EPROBE_DEFER);

	return backlight;
}
EXPORT_SYMBOL(tinydrm_of_find_backlight);

int tinydrm_enable_backlight(struct tinydrm_device *tdev)
{
	if (tdev->backlight) {
		if (tdev->backlight->props.brightness == 0)
			tdev->backlight->props.brightness =
					tdev->backlight->props.max_brightness;
		tdev->backlight->props.state &= ~BL_CORE_SUSPENDED;
		backlight_update_status(tdev->backlight);
	}

	return 0;
}
EXPORT_SYMBOL(tinydrm_enable_backlight);

int tinydrm_disable_backlight(struct tinydrm_device *tdev)
{
	if (tdev->backlight) {
		tdev->backlight->props.state |= BL_CORE_SUSPENDED;
		backlight_update_status(tdev->backlight);
	}

	return 0;
}
EXPORT_SYMBOL(tinydrm_disable_backlight);

static int __maybe_unused tinydrm_pm_suspend(struct device *dev)
{
	struct tinydrm_device *tdev = dev_get_drvdata(dev);

	tinydrm_disable(tdev);
	tinydrm_unprepare(tdev);

	return 0;
}

static int __maybe_unused tinydrm_pm_resume(struct device *dev)
{
	struct tinydrm_device *tdev = dev_get_drvdata(dev);

	tinydrm_prepare(tdev);
	/* Will be enabled after the first display update */

	return 0;
}

const struct dev_pm_ops tinydrm_simple_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tinydrm_pm_suspend, tinydrm_pm_resume)
};
EXPORT_SYMBOL(tinydrm_simple_pm_ops);

void tinydrm_spi_shutdown(struct spi_device *spi)
{
	struct tinydrm_device *tdev = spi_get_drvdata(spi);

	tinydrm_disable(tdev);
	tinydrm_unprepare(tdev);
}
EXPORT_SYMBOL(tinydrm_spi_shutdown);
