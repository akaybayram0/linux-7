// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ICM-20948 magnetometer (AK09916) support.
 *
 * The AK09916 sits behind the ICM-20948's internal I2C master, so this works
 * for both the I2C and the SPI host interface.
 *
 *  - SLV4 is used for one-shot register access (setup).
 *  - SLV0 is used for continuous reads: the I2C master copies
 *    AK09916 HXL..ST2 (8 bytes) into EXT_SLV_SENS_DATA_00..07 every cycle.
 *
 * The channels are reported in the accel/gyro coordinate frame (see
 * inv_icm20948_magn_read_sensor()).
 *
 * Register addresses are the regmap virtual addresses from inv_icm20948.h
 * (bank << 12 | reg).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>	/* <asm/unaligned.h> on kernels < 6.12 */

#include <linux/iio/iio.h>

#include "inv_icm20948.h"

#define ICM_I2C_MST_CLK_400KHZ		0x07	/* ~345.6 kHz */
#define ICM_I2C_MST_ODR_DIV		3	/* 1.1 kHz / 2^3 ~= 137 Hz */

/* AK09916 */
#define AK09916_I2C_ADDR		0x0c
#define AK09916_REG_WIA2		0x01
#define AK09916_WIA2_VALUE		0x09
#define AK09916_REG_HXL			0x11
#define AK09916_REG_CNTL2		0x31
#define   AK09916_MODE_POWER_DOWN	0x00
#define   AK09916_MODE_CONT_100HZ	0x08
#define AK09916_REG_CNTL3		0x32
#define   AK09916_SRST			BIT(0)
#define AK09916_ST2_HOFL		BIT(3)

/* 0.15 uT/LSB = 0.0015 Gauss/LSB (IIO magn scale is Gauss) */
#define AK09916_SCALE_MICRO		1500

#define AK09916_DATA_LEN		8	/* HXL..HZH (6) + TMPS + ST2 */

enum inv_icm20948_magn_scan {
	INV_ICM20948_MAGN_SCAN_X,
	INV_ICM20948_MAGN_SCAN_Y,
	INV_ICM20948_MAGN_SCAN_Z,
};

#define INV_ICM20948_MAGN_CHAN(_dir) \
	{ \
		.type = IIO_MAGN, \
		.modified = 1, \
		.channel2 = IIO_MOD_##_dir, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
		.ext_info = inv_icm20948_ext_info, \
		.scan_index = INV_ICM20948_MAGN_SCAN_##_dir, \
		.scan_type = { \
			.sign = 's', \
			.realbits = 16, \
			.storagebits = 16, \
			.endianness = IIO_LE, \
		}, \
	}

static const struct iio_chan_spec inv_icm20948_magn_channels[] = {
	INV_ICM20948_MAGN_CHAN(X),
	INV_ICM20948_MAGN_CHAN(Y),
	INV_ICM20948_MAGN_CHAN(Z),
};

/* ---- AK09916 access through the I2C master (SLV4), lock must be held ---- */

/*
 * Run one SLV4 transaction. I2C_MST_STATUS clears on read, so read it once
 * first to drop stale DONE/NACK bits. SLV4 is disabled again afterwards so
 * it does not keep repeating every master cycle.
 */
static int inv_icm20948_slv4_run(struct inv_icm20948_state *state)
{
	unsigned int status;
	int ret, ret2;

	ret = regmap_read(state->regmap, INV_ICM20948_REG_I2C_MST_STATUS,
			  &status);
	if (ret)
		return ret;

	ret = regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV4_CTRL,
			   INV_ICM20948_I2C_SLV_EN);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(state->regmap,
				       INV_ICM20948_REG_I2C_MST_STATUS, status,
				       status & INV_ICM20948_I2C_MST_STATUS_SLV4_DONE,
				       1000, 100000);

	ret2 = regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV4_CTRL, 0);

	if (ret)
		return ret;
	if (ret2)
		return ret2;

	if (status & INV_ICM20948_I2C_MST_STATUS_SLV4_NACK)
		return -EIO;

	return 0;
}

static int inv_icm20948_ak_read(struct inv_icm20948_state *state,
				u8 reg, unsigned int *val)
{
	int ret;

	ret = regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV4_ADDR,
			   INV_ICM20948_I2C_SLV_RNW | AK09916_I2C_ADDR);
	if (ret)
		return ret;

	ret = regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV4_REG, reg);
	if (ret)
		return ret;

	ret = inv_icm20948_slv4_run(state);
	if (ret)
		return ret;

	return regmap_read(state->regmap, INV_ICM20948_REG_I2C_SLV4_DI, val);
}

static int inv_icm20948_ak_write(struct inv_icm20948_state *state,
				 u8 reg, u8 val)
{
	int ret;

	ret = regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV4_ADDR, AK09916_I2C_ADDR);
	if (ret)
		return ret;

	ret = regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV4_REG, reg);
	if (ret)
		return ret;

	ret = regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV4_DO, val);
	if (ret)
		return ret;

	return inv_icm20948_slv4_run(state);
}

static int inv_icm20948_magn_setup(struct inv_icm20948_state *state)
{
	unsigned int wia;
	int ret;

	/* Turn on the internal I2C master and set its clock / rate. */
	ret = regmap_write(state->regmap, INV_ICM20948_REG_I2C_MST_CTRL,
			   ICM_I2C_MST_CLK_400KHZ);
	if (ret)
		return ret;

	ret = regmap_write(state->regmap, INV_ICM20948_REG_I2C_MST_ODR_CONFIG,
			   ICM_I2C_MST_ODR_DIV);
	if (ret)
		return ret;

	ret = regmap_set_bits(state->regmap, INV_ICM20948_REG_USER_CTRL,
			      INV_ICM20948_USER_CTRL_I2C_MST_EN);
	if (ret)
		return ret;

	/* Identify the chip. */
	ret = inv_icm20948_ak_read(state, AK09916_REG_WIA2, &wia);
	if (ret)
		return ret;
	if (wia != AK09916_WIA2_VALUE)
		return dev_err_probe(state->dev, -ENODEV,
				     "unexpected AK09916 WIA2: 0x%02x\n", wia);

	/* Soft reset, then continuous measurement at 100 Hz. */
	ret = inv_icm20948_ak_write(state, AK09916_REG_CNTL3, AK09916_SRST);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	ret = inv_icm20948_ak_write(state, AK09916_REG_CNTL2,
				    AK09916_MODE_CONT_100HZ);
	if (ret)
		return ret;

	/* SLV0: continuously read HXL..ST2 into EXT_SLV_SENS_DATA_00.. */
	ret = regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV0_ADDR,
			   INV_ICM20948_I2C_SLV_RNW | AK09916_I2C_ADDR);
	if (ret)
		return ret;

	ret = regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV0_REG, AK09916_REG_HXL);
	if (ret)
		return ret;

	return regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV0_CTRL,
			    INV_ICM20948_I2C_SLV_EN | AK09916_DATA_LEN);
}

/* Undo magn_setup() so a failed init does not leave the I2C master running. */
static void inv_icm20948_magn_disable(struct inv_icm20948_state *state)
{
	regmap_write(state->regmap, INV_ICM20948_REG_I2C_SLV0_CTRL, 0);
	regmap_clear_bits(state->regmap, INV_ICM20948_REG_USER_CTRL,
			  INV_ICM20948_USER_CTRL_I2C_MST_EN);
}

/* ---- IIO ---- */

static int inv_icm20948_magn_read_sensor(struct inv_icm20948_state *state,
					 const struct iio_chan_spec *chan,
					 int *val)
{
	u8 buf[AK09916_DATA_LEN];
	bool negate = false;
	int ret, off;

	/*
	 * On the ICM-20948 the AK09916 X axis points the same way as the
	 * accel/gyro X axis, while its Y and Z axes point the opposite way:
	 *   accel/gyro X =  mag X
	 *   accel/gyro Y = -mag Y
	 *   accel/gyro Z = -mag Z
	 * Report the magnetometer in the accel/gyro frame so that all three
	 * sensors share one coordinate system.
	 */
	switch (chan->channel2) {
	case IIO_MOD_X:
		off = 0;	/* HXL */
		break;
	case IIO_MOD_Y:
		off = 2;	/* HYL */
		negate = true;
		break;
	case IIO_MOD_Z:
		off = 4;	/* HZL */
		negate = true;
		break;
	default:
		return -EINVAL;
	}

	ret = pm_runtime_resume_and_get(state->dev);
	if (ret < 0)
		return ret;

	scoped_guard(mutex, &state->lock)
		ret = regmap_bulk_read(state->regmap, INV_ICM20948_REG_EXT_SLV_SENS_DATA_00,
				       buf, sizeof(buf));

	pm_runtime_put_autosuspend(state->dev);

	if (ret)
		return ret;

	/* buf[7] = ST2; the sensor saturated on this sample. */
	if (buf[AK09916_DATA_LEN - 1] & AK09916_ST2_HOFL)
		return -EOVERFLOW;

	*val = (s16)get_unaligned_le16(&buf[off]);
	if (negate)
		*val = -*val;

	return 0;
}

static int inv_icm20948_magn_read_raw(struct iio_dev *magn_dev,
				      const struct iio_chan_spec *chan,
				      int *val, int *val2, long mask)
{
	struct inv_icm20948_state *state = iio_device_get_drvdata(magn_dev);
	int raw;
	int ret;

	if (chan->type != IIO_MAGN)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(magn_dev))
			return -EBUSY;

		ret = inv_icm20948_magn_read_sensor(state, chan, &raw);

		iio_device_release_direct(magn_dev);

		if (ret)
			return ret;

		*val = raw;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = AK09916_SCALE_MICRO;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}
}

static const struct iio_info inv_icm20948_magn_info = {
	.read_raw = inv_icm20948_magn_read_raw,
};

struct iio_dev *inv_icm20948_magn_init(struct inv_icm20948_state *state)
{
	struct iio_dev *magn_dev;
	int ret;

	magn_dev = devm_iio_device_alloc(state->dev, 0);
	if (!magn_dev)
		return ERR_PTR(-ENOMEM);

	iio_device_set_drvdata(magn_dev, state);

	magn_dev->name = "icm20948-magn";
	magn_dev->info = &inv_icm20948_magn_info;
	magn_dev->modes = INDIO_DIRECT_MODE;
	magn_dev->channels = inv_icm20948_magn_channels;
	magn_dev->num_channels = ARRAY_SIZE(inv_icm20948_magn_channels);

	ret = pm_runtime_resume_and_get(state->dev);
	if (ret < 0)
		return ERR_PTR(ret);

	scoped_guard(mutex, &state->lock) {
		ret = inv_icm20948_magn_setup(state);
		if (ret)
			inv_icm20948_magn_disable(state);
	}

	pm_runtime_put_autosuspend(state->dev);

	if (ret)
		return ERR_PTR(ret);

	ret = devm_iio_device_register(state->dev, magn_dev);
	if (ret)
		return ERR_PTR(ret);

	return magn_dev;
}
