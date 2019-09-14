// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for Sitronix ST7789 panels
 *
 * Copyright 2019 Limor Fried <ladyada@adafruit.com>
 *
 * Based on mi0283qt.c/ili9341.c:
 * Copyright 2018 David Lechner <david@lechnology.com>
 * Copyright 2016 Noralf Trønnes
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/tinydrm/mipi-dbi.h>
#include <drm/tinydrm/tinydrm-helpers.h>
#include <video/mipi_display.h>

#define ST77XX_MADCTL_MY  0x80
#define ST77XX_MADCTL_MX  0x40
#define ST77XX_MADCTL_MV  0x20
#define ST77XX_MADCTL_ML  0x10
#define ST77XX_MADCTL_BGR 0x08
#define ST77XX_MADCTL_RGB 0x00


static void st7789_enable(struct drm_simple_display_pipe *pipe,
			  struct drm_crtc_state *crtc_state,
			  struct drm_plane_state *plane_state)
{
        struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct mipi_dbi *mipi = mipi_dbi_from_tinydrm(tdev);
	struct drm_framebuffer *fb = pipe->plane.fb;
	u8 addr_mode;

	DRM_DEBUG_KMS("\n");

	mipi_dbi_hw_reset(mipi);

	mipi_dbi_command(mipi, MIPI_DCS_SET_DISPLAY_OFF);

	mipi_dbi_command(mipi, MIPI_DCS_SOFT_RESET);
	msleep(150);
	mipi_dbi_command(mipi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(10);
	mipi_dbi_command(mipi, MIPI_DCS_SET_PIXEL_FORMAT, 0x55); // 16 bit color
	msleep(10);
	mipi_dbi_command(mipi, MIPI_DCS_SET_ADDRESS_MODE, ST77XX_MADCTL_BGR);
	mipi_dbi_command(mipi, MIPI_DCS_SET_COLUMN_ADDRESS, 0, 0, 0, 240);
	mipi_dbi_command(mipi, MIPI_DCS_SET_PAGE_ADDRESS, 0, 0, 320>>8, 320&0xFF);
	mipi_dbi_command(mipi, MIPI_DCS_ENTER_INVERT_MODE); // odd hack, displays are inverted
	mipi_dbi_command(mipi, MIPI_DCS_ENTER_NORMAL_MODE);
	msleep(10);
	mipi_dbi_command(mipi, MIPI_DCS_SET_DISPLAY_ON);
	msleep(10);

	switch (mipi->rotation) {
	default:
		addr_mode = ST77XX_MADCTL_MX;
		break;
	case 90:
		addr_mode = ST77XX_MADCTL_MV;
		break;
	case 180:
		addr_mode = ST77XX_MADCTL_MY;
		break;
	case 270:
		addr_mode = ST77XX_MADCTL_MV | ST77XX_MADCTL_MY |
			    ST77XX_MADCTL_MX;
		break;
	}
	addr_mode |= ST77XX_MADCTL_BGR;
	mipi_dbi_command(mipi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);

	mipi_dbi_command(mipi, MIPI_DCS_SET_DISPLAY_ON);

	mipi->enabled = true;
	fb->funcs->dirty(fb, NULL, 0, 0, NULL, 0);

	backlight_enable(mipi->backlight);
}

static void st7789_disable(struct drm_simple_display_pipe *pipe)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct mipi_dbi *mipi = mipi_dbi_from_tinydrm(tdev);

	DRM_DEBUG_KMS("\n");

	mipi->enabled = false;
	backlight_disable(mipi->backlight);
}

static const struct drm_simple_display_pipe_funcs st7789_funcs = {
	.enable = st7789_enable,
	.disable = st7789_disable,
	.update = tinydrm_display_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode st7789_mode = {
	TINYDRM_MODE(240, 320, 37, 49),
};

static struct drm_driver st7789_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME |
	                          DRIVER_ATOMIC,
	TINYDRM_GEM_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "st7789",
	.desc			= "Sitronix ST7789",
	.date			= "20190913",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id st7789_of_match[] = {
	{ .compatible = "adafruit,st7789" },
	{ }
};
MODULE_DEVICE_TABLE(of, st7789_of_match);

static const struct spi_device_id st7789_id[] = {
	{ "generic st7789", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, st7789_id);

static int st7789_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct tinydrm_device *tdev;
	struct mipi_dbi *mipi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

	mipi = devm_kzalloc(dev, sizeof(*mipi), GFP_KERNEL);
	if (!mipi)
		return -ENOMEM;

	mipi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(mipi->reset)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(mipi->reset);
	}

	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(dc);
	}

	mipi->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(mipi->backlight))
		return PTR_ERR(mipi->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, mipi, dc);
	if (ret)
		return ret;

	ret = mipi_dbi_init(dev, mipi, &st7789_funcs, &st7789_driver, &st7789_mode, rotation);
	if (ret)
		return ret;

	/* Reading is not supported */
	mipi->read_commands = NULL;

	tdev = &mipi->tinydrm;

	ret = devm_tinydrm_register(tdev);
	if (ret)
		return ret;

	spi_set_drvdata(spi, mipi);

	DRM_DEBUG_DRIVER("Initialized %s:%s @%uMHz on minor %d\n",
			 tdev->drm->driver->name, dev_name(dev),
			 spi->max_speed_hz / 1000000,
			 tdev->drm->primary->index);

	return 0;
}

static void st7789_shutdown(struct spi_device *spi)
{
	struct mipi_dbi *mipi = spi_get_drvdata(spi);

	tinydrm_shutdown(&mipi->tinydrm);
}

static struct spi_driver st7789_spi_driver = {
	.driver = {
		.name = "st7789",
		.owner = THIS_MODULE,
		.of_match_table = st7789_of_match,
	},
	.id_table = st7789_id,
	.probe = st7789_probe,
	.shutdown = st7789_shutdown,
};
module_spi_driver(st7789_spi_driver);

MODULE_DESCRIPTION("Sitronix ST7789 DRM driver");
MODULE_AUTHOR("Limor Fried <ladyada@adafruit.com>");
MODULE_LICENSE("GPL");
