// SPDX-License-Identifier: GPL-2.0-or-later
/*
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>

#include "inv_icm20948.h"

/*
 * IIO acceleration scale is m/s^2 per LSB, expressed as
 * IIO_VAL_INT_PLUS_NANO (val = 0, val2 = nano m/s^2 per LSB).
 * e.g. +/-2g: 9.80665 / 16384 = 0.000598550 m/s^2 per LSB.
 *
 * ICM-20948 accelerometer sensitivity:
 *   +/-2g  = 16384 LSB/g
 *   +/-4g  =  8192 LSB/g
 *   +/-8g  =  4096 LSB/g
 *   +/-16g =  2048 LSB/g
 *
 * 1 g = 9.80665 m/s^2
 */
static const int inv_icm20948_accel_scale[] = {
	/* +/-2g */
	[2 * INV_ICM20948_ACCEL_FS_2G] = 0,
	[2 * INV_ICM20948_ACCEL_FS_2G + 1] = 598550,

	/* +/-4g */
	[2 * INV_ICM20948_ACCEL_FS_4G] = 0,
	[2 * INV_ICM20948_ACCEL_FS_4G + 1] = 1197100,

	/* +/-8g */
	[2 * INV_ICM20948_ACCEL_FS_8G] = 0,
	[2 * INV_ICM20948_ACCEL_FS_8G + 1] = 2394200,

	/* +/-16g */
	[2 * INV_ICM20948_ACCEL_FS_16G] = 0,
	[2 * INV_ICM20948_ACCEL_FS_16G + 1] = 4788400,
};

#define INV_ICM20948_ACCEL_CHAN(_dir) \
	{ \
		.type = IIO_ACCEL, \
		.modified = 1, \
		.channel2 = IIO_MOD_##_dir, \
		.info_mask_separate = \
			BIT(IIO_CHAN_INFO_RAW), \
		.info_mask_shared_by_type = \
			BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_shared_by_type_available = \
			BIT(IIO_CHAN_INFO_SCALE), \
		.ext_info = inv_icm20948_ext_info, \
		.scan_index = INV_ICM20948_ACCEL_SCAN_##_dir, \
		.scan_type = { \
			.sign = 's', \
			.realbits = 16, \
			.storagebits = 16, \
			.endianness = IIO_BE, \
		}, \
	}

enum inv_icm20948_accel_scan {
	INV_ICM20948_ACCEL_SCAN_X,
	INV_ICM20948_ACCEL_SCAN_Y,
	INV_ICM20948_ACCEL_SCAN_Z,
};

static const struct iio_chan_spec inv_icm20948_accel_channels[] = {
	INV_ICM20948_ACCEL_CHAN(X),
	INV_ICM20948_ACCEL_CHAN(Y),
	INV_ICM20948_ACCEL_CHAN(Z),
};

static int inv_icm20948_accel_apply_config(
	struct inv_icm20948_state *state)
{
	int ret;

	ret = pm_runtime_resume_and_get(state->dev);
	if (ret < 0)
		return ret;

	guard(mutex)(&state->lock);

	ret = regmap_write_bits(state->regmap,
				INV_ICM20948_REG_ACCEL_CONFIG,
				INV_ICM20948_ACCEL_CONFIG_FULLSCALE,
				FIELD_PREP(INV_ICM20948_ACCEL_CONFIG_FULLSCALE,
					   state->accel_conf->fsr));

	pm_runtime_put_autosuspend(state->dev);

	return ret;
}

static int inv_icm20948_accel_read_sensor(
	struct inv_icm20948_state *state,
	const struct iio_chan_spec *chan,
	s16 *val)
{
	unsigned int reg;
	__be16 raw;
	int ret;

	switch (chan->channel2) {
	case IIO_MOD_X:
		reg = INV_ICM20948_REG_ACCEL_DATA_X;
		break;
	case IIO_MOD_Y:
		reg = INV_ICM20948_REG_ACCEL_DATA_Y;
		break;
	case IIO_MOD_Z:
		reg = INV_ICM20948_REG_ACCEL_DATA_Z;
		break;
	default:
		return -EINVAL;
	}

	ret = pm_runtime_resume_and_get(state->dev);
	if (ret < 0)
		return ret;

	ret = regmap_bulk_read(state->regmap, reg, &raw, sizeof(raw));
	if (!ret)
		*val = (s16)be16_to_cpu(raw);

	pm_runtime_put_autosuspend(state->dev);

	return ret;
}

static int inv_icm20948_accel_read_raw(
	struct iio_dev *accel_dev,
	const struct iio_chan_spec *chan,
	int *val, int *val2, long mask)
{
	struct inv_icm20948_state *state =
		iio_device_get_drvdata(accel_dev);
	s16 raw;
	int ret;

	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(accel_dev))
			return -EBUSY;

		ret = inv_icm20948_accel_read_sensor(state, chan, &raw);

		iio_device_release_direct(accel_dev);

		if (ret)
			return ret;

		*val = raw;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = inv_icm20948_accel_scale[
			2 * state->accel_conf->fsr + 1];

		return IIO_VAL_INT_PLUS_NANO;

	default:
		return -EINVAL;
	}
}

static int inv_icm20948_accel_write_scale(
	struct inv_icm20948_state *state,
	int val, int val2)
{
	int idx, old_fsr, ret;

	if (val != 0)
		return -EINVAL;

	for (idx = 0;
	     idx < ARRAY_SIZE(inv_icm20948_accel_scale);
	     idx += 2) {
		if (val2 == inv_icm20948_accel_scale[idx + 1])
			break;
	}

	if (idx >= ARRAY_SIZE(inv_icm20948_accel_scale))
		return -EINVAL;

	old_fsr = state->accel_conf->fsr;
	state->accel_conf->fsr = idx / 2;

	ret = inv_icm20948_accel_apply_config(state);
	if (ret)
		state->accel_conf->fsr = old_fsr;

	return ret;
}

static int inv_icm20948_accel_write_raw_get_fmt(
	struct iio_dev *accel_dev,
	const struct iio_chan_spec *chan, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return -EINVAL;
	}
}

static int inv_icm20948_accel_write_raw(
	struct iio_dev *accel_dev,
	const struct iio_chan_spec *chan,
	int val, int val2, long mask)
{
	struct inv_icm20948_state *state =
		iio_device_get_drvdata(accel_dev);
	int ret;

	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (!iio_device_claim_direct(accel_dev))
			return -EBUSY;

		ret = inv_icm20948_accel_write_scale(
			state, val, val2);

		iio_device_release_direct(accel_dev);

		return ret;

	default:
		return -EINVAL;
	}
}

static int inv_icm20948_accel_read_avail(
	struct iio_dev *accel_dev,
	const struct iio_chan_spec *chan,
	const int **vals, int *type,
	int *length, long mask)
{
	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	if (mask != IIO_CHAN_INFO_SCALE)
		return -EINVAL;

	*vals = inv_icm20948_accel_scale;
	*type = IIO_VAL_INT_PLUS_NANO;
	*length = ARRAY_SIZE(inv_icm20948_accel_scale);

	return IIO_AVAIL_LIST;
}

static const struct iio_info inv_icm20948_accel_info = {
	.read_raw = inv_icm20948_accel_read_raw,
	.write_raw = inv_icm20948_accel_write_raw,
	.write_raw_get_fmt = inv_icm20948_accel_write_raw_get_fmt,
	.read_avail = inv_icm20948_accel_read_avail,
};

struct iio_dev *inv_icm20948_accel_init(
	struct inv_icm20948_state *state)
{
	struct iio_dev *accel_dev;
	int ret;

	accel_dev = devm_iio_device_alloc(state->dev, 0);
	if (!accel_dev)
		return ERR_PTR(-ENOMEM);

	iio_device_set_drvdata(accel_dev, state);

	accel_dev->name = "icm20948-accel";
	accel_dev->info = &inv_icm20948_accel_info;
	accel_dev->modes = INDIO_DIRECT_MODE;
	accel_dev->channels = inv_icm20948_accel_channels;
	accel_dev->num_channels =
		ARRAY_SIZE(inv_icm20948_accel_channels);

	state->accel_conf =
		devm_kzalloc(state->dev,
			     sizeof(*state->accel_conf),
			     GFP_KERNEL);
	if (!state->accel_conf)
		return ERR_PTR(-ENOMEM);

	state->accel_conf->fsr = INV_ICM20948_ACCEL_FS_2G;

	ret = inv_icm20948_accel_apply_config(state);
	if (ret)
		return ERR_PTR(ret);

	ret = devm_iio_device_register(state->dev, accel_dev);
	if (ret)
		return ERR_PTR(ret);

	return accel_dev;
}
