// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2025 Bharadwaj Raju <bharadwaj.raju777@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/property.h>

#include "inv_icm20948.h"

static int inv_icm20948_probe(struct spi_device *spi)
{
	struct regmap *regmap;
	int ret;

	spi->mode |= SPI_MODE_3;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	regmap = devm_regmap_init_spi(spi, &inv_icm20948_regmap_config_spi);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return inv_icm20948_core_probe(regmap);
}

static const struct spi_device_id inv_icm20948_id[] = { { "icm20948" }, {} };
MODULE_DEVICE_TABLE(spi, inv_icm20948_id);

static const struct of_device_id inv_icm20948_of_matches[] = {
	{ .compatible = "invensense,icm20948" },
	{}
};
MODULE_DEVICE_TABLE(of, inv_icm20948_of_matches);

static struct spi_driver inv_icm20948_driver = {
	.driver = {
		.name = "icm20948",
		.of_match_table = inv_icm20948_of_matches,
	},
	.probe = inv_icm20948_probe,
	.id_table = inv_icm20948_id,
};
module_spi_driver(inv_icm20948_driver);

MODULE_AUTHOR("Bharadwaj Raju <bharadwaj.raju777@gmail.com>");
MODULE_DESCRIPTION("InvenSense ICM-20948 device driver (SPI)");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_ICM20948");
