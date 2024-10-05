// SPDX-License-Identifier: GPL-2.0

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "tc358748_i2c.h"

#define TC358748_WIDTH_MIN		64u
#define TC358748_WIDTH_MAX		640u
#define TC358748_HEIGHT_MIN		64u
#define TC358748_HEIGHT_MAX		480u

#define TC358748_FORMAT       MEDIA_BUS_FMT_RGB888_1X24
// #define TC358748_FORMAT       MEDIA_BUS_FMT_Y8_1X8
// #define TC358748_FORMAT       MEDIA_BUS_FMT_UYVY8_1X16  // YCbCr 422 16-bit
// #define TC358748_FORMAT       MEDIA_BUS_FMT_RGB888_3X8
// #define TC358748_FORMAT       MEDIA_BUS_FMT_RGB888_1X32_PADHI

struct tc358748_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct clk *extclk;

	struct gpio_desc *reset_gpio;

	/* lock to protect all members below */
	struct mutex lock;

	struct v4l2_mbus_framefmt fmt;
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

static inline struct tc358748_dev *to_tc358748_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358748_dev, sd);
}

static int tc358748_set_stream(struct tc358748_dev *sensor, bool on)
{
	int ret;

	if (on)
	{
			/**
			 * Resume @dev synchronously and if that is successful, increment its runtime
			 * PM usage counter. Return 0 if the runtime PM usage counter of @dev has been
			 * incremented or a negative error code otherwise.
			 */
		ret = pm_runtime_resume_and_get(&sensor->i2c_client->dev);
		if (ret < 0)
			return ret;

		return 0;
	}
	else
	{
		pm_runtime_put(&sensor->i2c_client->dev);
		return 0;
	}
}

static void tc358748_adj_fmt(struct v4l2_mbus_framefmt *fmt)
{
pr_info("----------------------- adj_fmt 1");
	fmt->width = clamp(ALIGN(fmt->width, 4), TC358748_WIDTH_MIN, TC358748_WIDTH_MAX);
	fmt->height = clamp(ALIGN(fmt->height, 4), TC358748_HEIGHT_MIN, TC358748_HEIGHT_MAX);
	fmt->code = TC358748_FORMAT;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	// fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
pr_info("----------------------- adj_fmt %d %d 0x%04x", fmt->width, fmt->height, fmt->code);
}

static int tc358748_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct tc358748_dev *sensor = to_tc358748_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	mutex_lock(&sensor->lock);

pr_info("----------------------- get_fmt");
// pr_info("----------------------- get_fmt 0x%04x", format->format.code);


// if (format->which == V4L2_SUBDEV_FORMAT_TRY)
// {
// mutex_unlock(&sensor->lock);
// return 0;
// }

	// if (format->which == V4L2_SUBDEV_FORMAT_TRY)
	// 	fmt = v4l2_subdev_get_try_format(&sensor->sd, sd_state, 0 /* pad */);
	// else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);
	return 0;
}

static int tc358748_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct tc358748_dev *sensor = to_tc358748_dev(sd);
	int ret;

	tc358748_adj_fmt(&format->format);

	mutex_lock(&sensor->lock);

if (format->format.code != TC358748_FORMAT)
{
	mutex_unlock(&sensor->lock);
	return -EINVAL;
}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *fmt;

		fmt = v4l2_subdev_get_try_format(sd, sd_state, 0 /* pad */);
		*fmt = format->format;

		mutex_unlock(&sensor->lock);

pr_info("----------------------- set_fmt 1");
		return 0;
	}

	sensor->fmt = format->format;

pr_info("----------------------- set_fmt 6 = %d", ret);
ret=0;//$$
// unlock:
	mutex_unlock(&sensor->lock);

	return ret;
}

static int tc358748_s_ctrl(struct v4l2_ctrl *ctrl)
{
return 0;
}

static const struct v4l2_ctrl_ops tc358748_ctrl_ops = {
	.s_ctrl = tc358748_s_ctrl,
};

static int tc358748_init_controls(struct tc358748_dev *sensor)
{
	return 0;
}

#define REGS_ENTRY(a)	{(a), ARRAY_SIZE(a)}
#define REGS(...)	REGS_ENTRY(((const __be16[]){__VA_ARGS__}))

static int tc358748_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct tc358748_dev *sensor = to_tc358748_dev(sd);

	// clk_disable_unprepare(sensor->extclk);

	if (sensor->reset_gpio)
		gpiod_set_value(sensor->reset_gpio, 1); /* assert RESET signal */

	return 0;
}

static int tc358748_power_on(struct device *dev)
{
	// struct v4l2_subdev *sd = dev_get_drvdata(dev);
	// struct tc358748_dev *sensor = to_tc358748_dev(sd);
	return 0;
}

static int tc358748_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct tc358748_dev *sensor = to_tc358748_dev(sd);

	if (code->index)
		return -EINVAL;

	code->code = sensor->fmt.code;
	return 0;
}

static int tc358748_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index)
		return -EINVAL;

	if (fse->code != TC358748_FORMAT)
		return -EINVAL;

	fse->min_width = TC358748_WIDTH_MIN;
	fse->max_width = TC358748_WIDTH_MAX;
	fse->min_height = TC358748_HEIGHT_MIN;
	fse->max_height = TC358748_HEIGHT_MAX;

	return 0;
}

static int tc358748_pre_streamon(struct v4l2_subdev *sd, u32 flags)
{
	struct tc358748_dev *sensor = to_tc358748_dev(sd);
	int ret;

	if (!(flags & V4L2_SUBDEV_PRE_STREAMON_FL_MANUAL_LP))
		return -EACCES;

	ret = pm_runtime_resume_and_get(&sensor->i2c_client->dev);
	if (ret < 0)
		return ret;

	return 0;
}

static int tc358748_post_streamoff(struct v4l2_subdev *sd)
{
	struct tc358748_dev *sensor = to_tc358748_dev(sd);

	pm_runtime_put(&sensor->i2c_client->dev);
	return 0;
}

static int tc358748_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tc358748_dev *sensor = to_tc358748_dev(sd);
	int ret;

	mutex_lock(&sensor->lock);

	ret = tc358748_set_stream(sensor, enable);
	if (!ret)
		sensor->streaming = enable;

	mutex_unlock(&sensor->lock);
	return ret;
}

static int tc358748_s_power(struct v4l2_subdev *sd, int on)
{
  return 0;
}

static const struct v4l2_subdev_core_ops tc358748_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
  .s_power = tc358748_s_power,
};

static const struct v4l2_subdev_video_ops tc358748_video_ops = {
	.s_stream = tc358748_s_stream,
	.pre_streamon = tc358748_pre_streamon,
	.post_streamoff = tc358748_post_streamoff,
};

static const struct v4l2_subdev_pad_ops tc358748_pad_ops = {
	.enum_mbus_code = tc358748_enum_mbus_code,
	.enum_frame_size = tc358748_enum_frame_size,
	.get_fmt = tc358748_get_fmt,
	.set_fmt = tc358748_set_fmt,
};

static const struct v4l2_subdev_ops tc358748_subdev_ops = {
	.core = &tc358748_core_ops,
	.video = &tc358748_video_ops,
	.pad = &tc358748_pad_ops,
};

static int __maybe_unused tc358748_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct tc358748_dev *sensor = to_tc358748_dev(sd);

	if (sensor->streaming)
		tc358748_set_stream(sensor, 0);

	return 0;
}

static int __maybe_unused tc358748_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct tc358748_dev *sensor = to_tc358748_dev(sd);

	if (sensor->streaming)
		return tc358748_set_stream(sensor, 1);

	return 0;
}

static int tc358748_probe(struct i2c_client *client)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct tc358748_dev *sensor;
	int ret;
pr_info("----------------------- probe 1");

	if (!tc358748_setup(client))
		return false;

pr_info("----------------------- probe 2");
	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;
	sensor->fmt.width = TC358748_WIDTH_MAX;
	sensor->fmt.height = TC358748_HEIGHT_MAX;

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

pr_info("----------------------- probe data lanes %d", sensor->lane_count);
if (ep.bus.mipi_csi2.flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK) {
	pr_info("----------------------- probe clock noncontinuous");
}
if (ep.bus.mipi_csi2.flags & V4L2_MBUS_CSI2_CONTINUOUS_CLOCK) {
	pr_info("----------------------- probe clock continuous");
}

	/* Get master clock (extclk) */
	sensor->extclk = devm_clk_get(dev, "ext");
	if (IS_ERR(sensor->extclk)) {
		dev_err(dev, "failed to get extclk\n");
		return PTR_ERR(sensor->extclk);
	}

	/* Request optional reset pin (usually active low) and assert it */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	v4l2_i2c_subdev_init(&sensor->sd, client, &tc358748_subdev_ops);

	sensor->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	ret = tc358748_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	tc358748_adj_fmt(&sensor->fmt);

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret)
		goto entity_cleanup;
		// goto free_ctrls;

	/* Turn on the device and enable runtime PM */
	ret = tc358748_power_on(&client->dev);
	if (ret)
		goto disable;
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);
pr_info("----------------------- probe end");
	return 0;

disable:
	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int tc358748_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tc358748_dev *sensor = to_tc358748_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		tc358748_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	mutex_destroy(&sensor->lock);
  return 0;
}

static const struct dev_pm_ops tc358748_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tc358748_suspend, tc358748_resume)
	SET_RUNTIME_PM_OPS(tc358748_power_off, tc358748_power_on, NULL)
};
static const struct of_device_id tc358748_dt_ids[] = {
	{.compatible = "onsemi,ar5"},
	{}
};
MODULE_DEVICE_TABLE(of, tc358748_dt_ids);

static struct i2c_driver tc358748_i2c_driver = {
	.driver = {
		.name  = "ar5",
		.pm = &tc358748_pm_ops,
		.of_match_table = tc358748_dt_ids,
	},
	.probe_new = tc358748_probe,
	.remove = tc358748_remove,
};

module_i2c_driver(tc358748_i2c_driver);

MODULE_DESCRIPTION("TC358748 Toshiba Parallel to CSI-2 driver");
MODULE_AUTHOR("Jarosław Sułkowski, p2119 <jaroslaw.sulkowski@pcosa.com.pl>");
MODULE_LICENSE("GPL");
