// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Sieć Badawcza Łukasiewicz
 * - Przemysłowy Instytut Automatyki i Pomiarów PIAP
 * Written by Krzysztof Hałasa
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "tc358748_i2c.h"

/* External clock (extclk) frequencies */
#define AR0521_EXTCLK_MIN		(10 * 1000 * 1000)
#define AR0521_EXTCLK_MAX		(48 * 1000 * 1000)

/* PLL and PLL2 */
#define AR0521_PLL_MIN			(320 * 1000 * 1000)
#define AR0521_PLL_MAX			(1280 * 1000 * 1000)

/* Effective pixel sample rate on the pixel array. */
#define AR0521_PIXEL_CLOCK_RATE		(184 * 1000 * 1000)
#define AR0521_PIXEL_CLOCK_MIN		(168 * 1000 * 1000)
#define AR0521_PIXEL_CLOCK_MAX		(414 * 1000 * 1000)

#define AR0521_NATIVE_WIDTH		2604u
#define AR0521_NATIVE_HEIGHT		1964u
#define AR0521_MIN_X_ADDR_START		0u
#define AR0521_MIN_Y_ADDR_START		0u
#define AR0521_MAX_X_ADDR_END		2603u
#define AR0521_MAX_Y_ADDR_END		1955u

#define AR0521_WIDTH_MIN		8u
#define AR0521_WIDTH_MAX		2592u
#define AR0521_HEIGHT_MIN		8u
#define AR0521_HEIGHT_MAX		1944u

#define AR0521_WIDTH_BLANKING_MIN	572u
#define AR0521_HEIGHT_BLANKING_MIN	38u /* must be even */
#define AR0521_TOTAL_HEIGHT_MAX		65535u /* max_frame_length_lines */
#define AR0521_TOTAL_WIDTH_MAX		65532u /* max_line_length_pck */

#define AR0521_ANA_GAIN_MIN		0x00
#define AR0521_ANA_GAIN_MAX		0x3f
#define AR0521_ANA_GAIN_STEP		0x01
#define AR0521_ANA_GAIN_DEFAULT		0x00

/* AR0521 registers */
// #define AR0521_REG_VT_PIX_CLK_DIV		0x0300
// #define AR0521_REG_FRAME_LENGTH_LINES		0x0340

// #define AR0521_REG_CHIP_ID			0x3000
// #define AR0521_REG_COARSE_INTEGRATION_TIME	0x3012
// #define AR0521_REG_ROW_SPEED			0x3016
// #define AR0521_REG_EXTRA_DELAY			0x3018
// #define AR0521_REG_RESET			0x301A
// #define   AR0521_REG_RESET_DEFAULTS		  0x0238
// #define   AR0521_REG_RESET_GROUP_PARAM_HOLD	  0x8000
// #define   AR0521_REG_RESET_STREAM		  BIT(2)
// #define   AR0521_REG_RESET_RESTART		  BIT(1)
// #define   AR0521_REG_RESET_INIT			  BIT(0)

// #define AR0521_REG_ANA_GAIN_CODE_GLOBAL		0x3028

// #define AR0521_REG_GREEN1_GAIN			0x3056
// #define AR0521_REG_BLUE_GAIN			0x3058
// #define AR0521_REG_RED_GAIN			0x305A
// #define AR0521_REG_GREEN2_GAIN			0x305C
// #define AR0521_REG_GLOBAL_GAIN			0x305E

// #define AR0521_REG_HISPI_TEST_MODE		0x3066
// #define AR0521_REG_HISPI_TEST_MODE_LP11		  0x0004

// #define AR0521_REG_TEST_PATTERN_MODE		0x3070

// #define AR0521_REG_SERIAL_FORMAT		0x31AE
// #define AR0521_REG_SERIAL_FORMAT_MIPI		  0x0200

// #define AR0521_REG_HISPI_CONTROL_STATUS		0x31C6
// #define AR0521_REG_HISPI_CONTROL_STATUS_FRAMER_TEST_MODE_ENABLE 0x80

#define be		cpu_to_be16

static const char * const ar0521_supply_names[] = {
	"vdd_io",	/* I/O (1.8V) supply */
	"vdd",		/* Core, PLL and MIPI (1.2V) supply */
	"vaa",		/* Analog (2.7V) supply */
};

static const s64 ar0521_link_frequencies[] = {
	184000000,
};

struct ar0521_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *gain;
		struct v4l2_ctrl *red_balance;
		struct v4l2_ctrl *blue_balance;
	};
	struct {
		struct v4l2_ctrl *hblank;
		struct v4l2_ctrl *vblank;
	};
	struct v4l2_ctrl *pixrate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *test_pattern;
};

struct ar0521_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct clk *extclk;
	u32 extclk_freq;

	struct regulator *supplies[ARRAY_SIZE(ar0521_supply_names)];
	struct gpio_desc *reset_gpio;

	/* lock to protect all members below */
	struct mutex lock;

	struct v4l2_mbus_framefmt fmt;
	struct ar0521_ctrls ctrls;
	unsigned int lane_count;
	struct {
		u16 pre;
		u16 mult;
		u16 pre2;
		u16 mult2;
		u16 vt_pix;
	} pll;

	bool streaming;
};

static inline struct ar0521_dev *to_ar0521_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0521_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ar0521_dev,
			     ctrls.handler)->sd;
}

static int ar0521_set_geometry(struct ar0521_dev *sensor)
{
	/* Center the image in the visible output window. */
	// u16 x = clamp((AR0521_WIDTH_MAX - sensor->fmt.width) / 2,
	// 	       AR0521_MIN_X_ADDR_START, AR0521_MAX_X_ADDR_END);
	// u16 y = clamp(((AR0521_HEIGHT_MAX - sensor->fmt.height) / 2) & ~1,
	// 	       AR0521_MIN_Y_ADDR_START, AR0521_MAX_Y_ADDR_END);

	/* All dimensions are unsigned 12-bit integers */
	// __be16 regs[] = {
	// 	be(AR0521_REG_FRAME_LENGTH_LINES),
	// 	be(sensor->fmt.height + sensor->ctrls.vblank->val),
	// 	be(sensor->fmt.width + sensor->ctrls.hblank->val),
	// 	be(x),
	// 	be(y),
	// 	be(x + sensor->fmt.width - 1),
	// 	be(y + sensor->fmt.height - 1),
	// 	be(sensor->fmt.width),
	// 	be(sensor->fmt.height)
	// };

	// return ar0521_write_regs(sensor, regs, ARRAY_SIZE(regs));
  return 0;
}

static int ar0521_set_gains(struct ar0521_dev *sensor)
{
	// int green = sensor->ctrls.gain->val;
	// int red = max(green + sensor->ctrls.red_balance->val, 0);
	// int blue = max(green + sensor->ctrls.blue_balance->val, 0);
	// unsigned int gain = min(red, min(green, blue));
	// unsigned int analog = min(gain, 64u); /* range is 0 - 127 */
	// __be16 regs[5];

	// red   = min(red   - analog + 64, 511u);
	// green = min(green - analog + 64, 511u);
	// blue  = min(blue  - analog + 64, 511u);
	// regs[0] = be(AR0521_REG_GREEN1_GAIN);
	// regs[1] = be(green << 7 | analog);
	// regs[2] = be(blue  << 7 | analog);
	// regs[3] = be(red   << 7 | analog);
	// regs[4] = be(green << 7 | analog);

	// return ar0521_write_regs(sensor, regs, ARRAY_SIZE(regs));
  return 0;
}

static int ar0521_set_stream(struct ar0521_dev *sensor, bool on)
{
	int ret;

	if (on) {
		ret = pm_runtime_resume_and_get(&sensor->i2c_client->dev);
		if (ret < 0)
			return ret;

		/* Stop streaming for just a moment */
		// ret = ar0521_write_reg(sensor, AR0521_REG_RESET,
		// 		       AR0521_REG_RESET_DEFAULTS);
		// if (ret)
		// 	return ret;

		ret = ar0521_set_geometry(sensor);
		if (ret)
			return ret;

		ret =  __v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
		if (ret)
			goto err;

		/* Exit LP-11 mode on clock and data lanes */
		// ret = ar0521_write_reg(sensor, AR0521_REG_HISPI_CONTROL_STATUS,
		// 		       0);
		// if (ret)
		// 	goto err;

		// /* Start streaming */
		// ret = ar0521_write_reg(sensor, AR0521_REG_RESET,
		// 		       AR0521_REG_RESET_DEFAULTS |
		// 		       AR0521_REG_RESET_STREAM);
		// if (ret)
		// 	goto err;

		return 0;

err:
		pm_runtime_put(&sensor->i2c_client->dev);
		return ret;

	} else {
		/*
		 * Reset gain, the sensor may produce all white pixels without
		 * this
		 */
		// ret = ar0521_write_reg(sensor, AR0521_REG_GLOBAL_GAIN, 0x2000);
		// if (ret)
		// 	return ret;

		// /* Stop streaming */
		// ret = ar0521_write_reg(sensor, AR0521_REG_RESET,
		// 		       AR0521_REG_RESET_DEFAULTS);
		// if (ret)
		// 	return ret;

		pm_runtime_put(&sensor->i2c_client->dev);
		return 0;
	}
}

static void ar0521_adj_fmt(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = clamp(ALIGN(fmt->width, 4), AR0521_WIDTH_MIN,
			   AR0521_WIDTH_MAX);
	fmt->height = clamp(ALIGN(fmt->height, 4), AR0521_HEIGHT_MIN,
			    AR0521_HEIGHT_MAX);
	fmt->code = MEDIA_BUS_FMT_SGRBG8_1X8;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int ar0521_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct ar0521_dev *sensor = to_ar0521_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(&sensor->sd, sd_state, 0
						 /* pad */);
	else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);
	return 0;
}

static int ar0521_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct ar0521_dev *sensor = to_ar0521_dev(sd);
	int max_vblank, max_hblank;
  // int exposure_max;
	int ret;

	ar0521_adj_fmt(&format->format);

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *fmt;

		fmt = v4l2_subdev_get_try_format(sd, sd_state, 0 /* pad */);
		*fmt = format->format;

		mutex_unlock(&sensor->lock);

// pr_info("----------------------- set_fmt 1");
		return 0;
	}

	sensor->fmt = format->format;

	/*
	 * Update the exposure and blankings limits. Blankings are also reset
	 * to the minimum.
	 */
	max_hblank = AR0521_TOTAL_WIDTH_MAX - sensor->fmt.width;
	ret = __v4l2_ctrl_modify_range(sensor->ctrls.hblank,
				       sensor->ctrls.hblank->minimum,
				       max_hblank, sensor->ctrls.hblank->step,
				       sensor->ctrls.hblank->minimum);
	if (ret)
	{
// pr_info("----------------------- set_fmt 2");
		goto unlock;
	}

	ret = __v4l2_ctrl_s_ctrl(sensor->ctrls.hblank,
				 sensor->ctrls.hblank->minimum);
	if (ret)
	{
// pr_info("----------------------- set_fmt 3");
		goto unlock;
	}

	max_vblank = AR0521_TOTAL_HEIGHT_MAX - sensor->fmt.height;
	ret = __v4l2_ctrl_modify_range(sensor->ctrls.vblank,
				       sensor->ctrls.vblank->minimum,
				       max_vblank, sensor->ctrls.vblank->step,
				       sensor->ctrls.vblank->minimum);
	if (ret)
	{
// pr_info("----------------------- set_fmt 4");
		goto unlock;
	}

	ret = __v4l2_ctrl_s_ctrl(sensor->ctrls.vblank,
				 sensor->ctrls.vblank->minimum);
	if (ret)
	{
// pr_info("----------------------- set_fmt 5");
		goto unlock;
	}

	// exposure_max = sensor->fmt.height + AR0521_HEIGHT_BLANKING_MIN - 4;
// pr_info("----------------------- set_fmt   exposure_max = %d, min = %d, step = %d, exposure_default = %d",
// 		exposure_max, sensor->ctrls.exposure->minimum, sensor->ctrls.exposure->step,
// 		sensor->ctrls.exposure->default_value);
// 	ret = __v4l2_ctrl_modify_range(sensor->ctrls.exposure,
// 				       sensor->ctrls.exposure->minimum,
// 				       exposure_max,
// 				       sensor->ctrls.exposure->step,
// 				       sensor->ctrls.exposure->default_value);
// pr_info("----------------------- set_fmt 6 = %d", ret);
ret=0;//$$
unlock:
	mutex_unlock(&sensor->lock);

	return ret;
}

static int ar0521_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct ar0521_dev *sensor = to_ar0521_dev(sd);
	int exp_max;
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		exp_max = sensor->fmt.height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(sensor->ctrls.exposure,
					 sensor->ctrls.exposure->minimum,
					 exp_max, sensor->ctrls.exposure->step,
					 sensor->ctrls.exposure->default_value);
		break;
	}

	/* access the sensor only if it's powered up */
	if (!pm_runtime_get_if_in_use(&sensor->i2c_client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_HBLANK:
	case V4L2_CID_VBLANK:
		ret = ar0521_set_geometry(sensor);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		// ret = ar0521_write_reg(sensor, AR0521_REG_ANA_GAIN_CODE_GLOBAL,
		// 		       ctrl->val);
		break;
	case V4L2_CID_GAIN:
	case V4L2_CID_RED_BALANCE:
	case V4L2_CID_BLUE_BALANCE:
		ret = ar0521_set_gains(sensor);
		break;
	case V4L2_CID_EXPOSURE:
		// ret = ar0521_write_reg(sensor,
		// 		       AR0521_REG_COARSE_INTEGRATION_TIME,
				      //  ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		// ret = ar0521_write_reg(sensor, AR0521_REG_TEST_PATTERN_MODE,
		// 		       ctrl->val);
		break;
	default:
		dev_err(&sensor->i2c_client->dev,
			"Unsupported control %x\n", ctrl->id);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&sensor->i2c_client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops ar0521_ctrl_ops = {
	.s_ctrl = ar0521_s_ctrl,
};

static const char * const test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"Color bars",
	"Faded color bars"
};

static int ar0521_init_controls(struct ar0521_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &ar0521_ctrl_ops;
	struct ar0521_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int max_vblank, max_hblank, exposure_max;
	struct v4l2_ctrl *link_freq;
	int ret;

	v4l2_ctrl_handler_init(hdl, 32);

	/* We can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Analog gain */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
			  AR0521_ANA_GAIN_MIN, AR0521_ANA_GAIN_MAX,
			  AR0521_ANA_GAIN_STEP, AR0521_ANA_GAIN_DEFAULT);

	/* Manual gain */
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN, 0, 511, 1, 0);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       -512, 511, 1, 0);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						-512, 511, 1, 0);
	v4l2_ctrl_cluster(3, &ctrls->gain);

	/* Initialize blanking limits using the default 2592x1944 format. */
	max_hblank = AR0521_TOTAL_WIDTH_MAX - AR0521_WIDTH_MAX;
	ctrls->hblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
					  AR0521_WIDTH_BLANKING_MIN,
					  max_hblank, 1,
					  AR0521_WIDTH_BLANKING_MIN);

	max_vblank = AR0521_TOTAL_HEIGHT_MAX - AR0521_HEIGHT_MAX;
	ctrls->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					  AR0521_HEIGHT_BLANKING_MIN,
					  max_vblank, 2,
					  AR0521_HEIGHT_BLANKING_MIN);
	v4l2_ctrl_cluster(2, &ctrls->hblank);

	/* Read-only */
	ctrls->pixrate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE,
					   AR0521_PIXEL_CLOCK_MIN,
					   AR0521_PIXEL_CLOCK_MAX, 1,
					   AR0521_PIXEL_CLOCK_RATE);

	/* Manual exposure time: max exposure time = visible + blank - 4 */
	exposure_max = AR0521_HEIGHT_MAX + AR0521_HEIGHT_BLANKING_MIN - 4;
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE, 0,
					    exposure_max, 1, 0x70);

	link_freq = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(ar0521_link_frequencies) - 1,
					0, ar0521_link_frequencies);
	if (link_freq)
		link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrls->test_pattern = v4l2_ctrl_new_std_menu_items(hdl, ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(test_pattern_menu) - 1,
					0, 0, test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

#define REGS_ENTRY(a)	{(a), ARRAY_SIZE(a)}
#define REGS(...)	REGS_ENTRY(((const __be16[]){__VA_ARGS__}))

static int ar0521_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0521_dev *sensor = to_ar0521_dev(sd);
	int i;

	clk_disable_unprepare(sensor->extclk);

	if (sensor->reset_gpio)
		gpiod_set_value(sensor->reset_gpio, 1); /* assert RESET signal */

	for (i = ARRAY_SIZE(ar0521_supply_names) - 1; i >= 0; i--) {
		if (sensor->supplies[i])
			regulator_disable(sensor->supplies[i]);
	}
	return 0;
}

static int ar0521_power_on(struct device *dev)
{
  return 0;
}

static int ar0521_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0521_dev *sensor = to_ar0521_dev(sd);

	if (code->index)
		return -EINVAL;

	code->code = sensor->fmt.code;
	return 0;
}

static int ar0521_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG8_1X8)
		return -EINVAL;

	fse->min_width = AR0521_WIDTH_MIN;
	fse->max_width = AR0521_WIDTH_MAX;
	fse->min_height = AR0521_HEIGHT_MIN;
	fse->max_height = AR0521_HEIGHT_MAX;

	return 0;
}

static int ar0521_pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct ar0521_dev *sensor = to_ar0521_dev(sd);
	int ret;

	if (!(flags & V4L2_SUBDEV_PRE_STREAMON_FL_MANUAL_LP))
		return -EACCES;

	ret = pm_runtime_resume_and_get(&sensor->i2c_client->dev);
	if (ret < 0)
		return ret;

	/* Set LP-11 on clock and data lanes */
	// ret = ar0521_write_reg(sensor, AR0521_REG_HISPI_CONTROL_STATUS,
	// 		AR0521_REG_HISPI_CONTROL_STATUS_FRAMER_TEST_MODE_ENABLE);
	// if (ret)
	// 	goto err;

	/* Start streaming LP-11 */
	// ret = ar0521_write_reg(sensor, AR0521_REG_RESET,
	// 		       AR0521_REG_RESET_DEFAULTS |
	// 		       AR0521_REG_RESET_STREAM);
	// if (ret)
	// 	goto err;
	return 0;

// err:
// 	pm_runtime_put(&sensor->i2c_client->dev);
// 	return ret;
}

static int ar0521_post_streamoff(struct v4l2_subdev *sd)
{
	struct ar0521_dev *sensor = to_ar0521_dev(sd);

	pm_runtime_put(&sensor->i2c_client->dev);
	return 0;
}

static int ar0521_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0521_dev *sensor = to_ar0521_dev(sd);
	int ret;

	mutex_lock(&sensor->lock);

	ret = ar0521_set_stream(sensor, enable);
	if (!ret)
		sensor->streaming = enable;

	mutex_unlock(&sensor->lock);
	return ret;
}

static int ar0521_s_power(struct v4l2_subdev *sd, int on)
{
  return 0;
}

static const struct v4l2_subdev_core_ops ar0521_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
  .s_power = ar0521_s_power,
};

static const struct v4l2_subdev_video_ops ar0521_video_ops = {
	.s_stream = ar0521_s_stream,
	.pre_streamon = ar0521_pre_streamon,
	.post_streamoff = ar0521_post_streamoff,
};

static const struct v4l2_subdev_pad_ops ar0521_pad_ops = {
	.enum_mbus_code = ar0521_enum_mbus_code,
	.enum_frame_size = ar0521_enum_frame_size,
	.get_fmt = ar0521_get_fmt,
	.set_fmt = ar0521_set_fmt,
};

static const struct v4l2_subdev_ops ar0521_subdev_ops = {
	.core = &ar0521_core_ops,
	.video = &ar0521_video_ops,
	.pad = &ar0521_pad_ops,
};

static int __maybe_unused ar0521_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0521_dev *sensor = to_ar0521_dev(sd);

	if (sensor->streaming)
		ar0521_set_stream(sensor, 0);

	return 0;
}

static int __maybe_unused ar0521_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar0521_dev *sensor = to_ar0521_dev(sd);

	if (sensor->streaming)
		return ar0521_set_stream(sensor, 1);

	return 0;
}

static int ar0521_probe(struct i2c_client *client)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct ar0521_dev *sensor;
	unsigned int cnt;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;
	sensor->fmt.width = AR0521_WIDTH_MAX;
	sensor->fmt.height = AR0521_HEIGHT_MAX;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
						   FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "could not parse endpoint\n");
		return ret;
	}

	if (ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(dev, "invalid bus type, must be MIPI CSI2\n");
		return -EINVAL;
	}

	sensor->lane_count = ep.bus.mipi_csi2.num_data_lanes;
	switch (sensor->lane_count) {
	case 1:
	case 2:
	case 4:
		break;
	default:
		dev_err(dev, "invalid number of MIPI data lanes\n");
		return -EINVAL;
	}

	/* Get master clock (extclk) */
	sensor->extclk = devm_clk_get(dev, "ext");
	if (IS_ERR(sensor->extclk)) {
		dev_err(dev, "failed to get extclk\n");
		return PTR_ERR(sensor->extclk);
	}

	sensor->extclk_freq = clk_get_rate(sensor->extclk);

	if (sensor->extclk_freq < AR0521_EXTCLK_MIN ||
	    sensor->extclk_freq > AR0521_EXTCLK_MAX) {
		dev_err(dev, "extclk frequency out of range: %u Hz\n",
			sensor->extclk_freq);
		return -EINVAL;
	}

	/* Request optional reset pin (usually active low) and assert it */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	v4l2_i2c_subdev_init(&sensor->sd, client, &ar0521_subdev_ops);

	sensor->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	for (cnt = 0; cnt < ARRAY_SIZE(ar0521_supply_names); cnt++) {
		struct regulator *supply = devm_regulator_get(dev,
						ar0521_supply_names[cnt]);

		if (IS_ERR(supply)) {
			dev_info(dev, "no %s regulator found: %li\n",
				 ar0521_supply_names[cnt], PTR_ERR(supply));
			return PTR_ERR(supply);
		}
		sensor->supplies[cnt] = supply;
	}

	mutex_init(&sensor->lock);

	ret = ar0521_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ar0521_adj_fmt(&sensor->fmt);

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret)
		goto free_ctrls;

	/* Turn on the device and enable runtime PM */
	ret = ar0521_power_on(&client->dev);
	if (ret)
		goto disable;
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);
	return 0;

disable:
	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int ar0521_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0521_dev *sensor = to_ar0521_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ar0521_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	mutex_destroy(&sensor->lock);
  return 0;
}

static const struct dev_pm_ops ar0521_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ar0521_suspend, ar0521_resume)
	SET_RUNTIME_PM_OPS(ar0521_power_off, ar0521_power_on, NULL)
};
static const struct of_device_id ar0521_dt_ids[] = {
	{.compatible = "onsemi,ar5"},
	{}
};
MODULE_DEVICE_TABLE(of, ar0521_dt_ids);

static struct i2c_driver ar0521_i2c_driver = {
	.driver = {
		.name  = "ar5",
		.pm = &ar0521_pm_ops,
		.of_match_table = ar0521_dt_ids,
	},
	.probe_new = ar0521_probe,
	.remove = ar0521_remove,
};

module_i2c_driver(ar0521_i2c_driver);

MODULE_DESCRIPTION("AR0521 MIPI Camera subdev driver");
MODULE_AUTHOR("Krzysztof Hałasa <khalasa@piap.pl>");
MODULE_LICENSE("GPL");