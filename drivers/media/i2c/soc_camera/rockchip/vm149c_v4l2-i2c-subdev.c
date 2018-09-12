/*
 * drivers/media/i2c/soc_camera/xgold/vm149c.c
 *
 * vm149c auto focus controller driver
 *
 * Copyright (C) 2014
 *
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Note:
 *    09/04/2014: initial version
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/platform_data/rk_isp10_platform_camera_module.h>
#include <linux/platform_data/rk_isp10_platform.h>


#define VM149C_DRIVER_NAME    "vm149c"
/*!<
 * Focus position values:
 * 65 logical positions ( 0 - 64 )
 * where 64 is the setting for infinity and 0 for macro
 * corresponding to
 * 1024 register settings (0 - 1023)
 * where 0 is the setting for infinity and 1023 for macro
 */
#define MAX_LOG   64U
#define MAX_REG 1023U

#define MAX_VCMDRV_CURRENT      100U
#define MAX_VCMDRV_REG          1023U

#define VCMDRV_START_CURRENT	0
#define VCMDRV_RATED_CURRENT	100
#define VCMDRV_STEP_MODE		4

/* ======================================================================== */

struct vm149c_dev {
	u16 current_lens_pos;
	uint32_t StartCurrent;
    uint32_t RatedCurrent;
    uint32_t Step;
    uint32_t StepMode;
	struct v4l2_subdev sd;
};

/* ======================================================================== */

int vm149c_read_msg(
	struct i2c_client *client,
	u8* msb, u8* lsb)
{
	int ret = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retries;

	if (!client->adapter) {
		dev_err(&client->dev, "client->adapter NULL\n");
		return -ENODEV;
	}

	for (retries = 0; retries < 5; retries++) {
		msg->addr = client->addr;
		msg->flags = 1;
		msg->len = 2;
		msg->buf = data;

		ret = i2c_transfer(client->adapter, msg, 1);
		usleep_range(50, 100);

		if (ret == 1) {
			dev_info(&client->dev,
				"%s: vcm i2c ok, addr 0x%x, data 0x%x, 0x%x\n",
				__func__, msg->addr, data[0], data[1]);

			*msb = data[0];
			*lsb = data[1];
			return 0;
		}

		dev_info(&client->dev,
			"retrying I2C... %d\n", retries);
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
	}
	dev_err(&client->dev,
		"i2c write to failed with error %d\n", ret);
	return ret;
}


int vm149c_write_msg(
	struct i2c_client *client,
	u8 msb, u8 lsb)
{
	int ret = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retries;

	if (!client->adapter) {
		dev_err(&client->dev, "client->adapter NULL\n");
		return -ENODEV;
	}

	for (retries = 0; retries < 5; retries++) {
		msg->addr = client->addr;
		msg->flags = 0;
		msg->len = 2;
		msg->buf = data;

		data[0] = msb;
		data[1] = lsb;

		ret = i2c_transfer(client->adapter, msg, 1);
		usleep_range(50, 100);

		if (ret == 1) {
			dev_info(&client->dev,
				"%s: vcm i2c ok, addr 0x%x, data 0x%x, 0x%x\n",
				__func__, msg->addr, data[0], data[1]);
			return 0;
		}

		dev_info(&client->dev,
			"retrying I2C... %d\n", retries);
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
	}
	dev_err(&client->dev,
		"i2c write to failed with error %d\n", ret);
	return ret;
}

/* ======================================================================== */

static int vm149c_g_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vm149c_dev *dev = container_of(sd, struct vm149c_dev, sd);
	int ret;
	u8 lsb;
	u8 msb;
	uint32_t AbsStep;

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		ret = vm149c_read_msg(client, &msb, &lsb);
		if (IS_ERR_VALUE(ret))
			goto err;

		AbsStep = ( ((uint32_t)(msb & 0x3FU)) << 4U ) | ( ((uint32_t)lsb) >> 4U );
		if( AbsStep <= dev->StartCurrent)
	    {
	        AbsStep = MAX_LOG;
	    }
	    else if((AbsStep>dev->StartCurrent) && (AbsStep<=dev->RatedCurrent))
	    {
	        AbsStep = (dev->RatedCurrent - AbsStep ) / dev->Step;
	    }
		else
		{
			AbsStep = 0;
		}

		ctrl->value = AbsStep;
		dev_info(&client->dev,
			"%s V4L2_CID_FOCUS_ABSOLUTE %d\n", __func__, ctrl->value);
		return 0;
	}

	ret = -EINVAL;

err:
	dev_err(&client->dev,
		"failed with error %d\n", ret);
	return ret;
}

/* ======================================================================== */

static int vm149c_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int ret;
	u8 lsb;
	u8 msb;
	uint32_t nPosition;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vm149c_dev *dev = container_of(sd, struct vm149c_dev, sd);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		dev_info(&client->dev,
			"%s V4L2_CID_FOCUS_ABSOLUTE %d\n", __func__, ctrl->value);

		if (ctrl->value > MAX_LOG) {
			dev_err(&client->dev,
				"value out of range, must be in [0..1023]\n");
			ret = -ERANGE;
			goto err;
		}

	    if ( ctrl->value >= MAX_LOG )
	        nPosition = dev->StartCurrent;
	    else
	        nPosition = dev->StartCurrent + (dev->Step*(MAX_LOG-ctrl->value));
	    if (nPosition > MAX_VCMDRV_REG)
	        nPosition = MAX_VCMDRV_REG;

		dev->current_lens_pos = nPosition;
		msb = (0x00U | ((dev->current_lens_pos & 0x3F0U) >> 4U));
		lsb = (((dev->current_lens_pos & 0x0FU) << 4U) | 0x04);
		ret = vm149c_write_msg(client, msb, lsb);
		if (IS_ERR_VALUE(ret))
			goto err;
	} else {
		dev_info(&client->dev,
			"ctrl ID %d not supported\n", ctrl->id);
		return -EINVAL;
	}

	return 0;
err:
	dev_err(&client->dev,
		"failed with error %d\n", ret);
	return ret;
}

static long vm149c_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg)
{
	int ret = 0;
	uint32_t vcm_movefull_t;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vm149c_dev *dev = container_of(sd, struct vm149c_dev, sd);

	if (cmd == PLTFRM_CIFCAM_VCM_INFO) {
		uint32_t *max_step = (uint32_t *)arg;

		*max_step = 0;
		if ((dev->StepMode & 0x0c) != 0) {
			vcm_movefull_t = 64* (1<<(dev->StepMode & 0x03)) *1024/
					((1 << (((dev->StepMode & 0x0c)>>2)-1))*1000);
		}else{
			vcm_movefull_t =64*1023/1000;
		}

		*max_step = (MAX_LOG|(vcm_movefull_t<<16));
	} else {
		dev_info(&client->dev,
			"cmd %d not supported\n", cmd);
		return -EINVAL;
	}

	return ret;
}


/* ======================================================================== */

static struct v4l2_subdev_core_ops vm149c_core_ops = {
	.g_ctrl = vm149c_g_ctrl,
	.s_ctrl = vm149c_s_ctrl,
	.ioctl = vm149c_ioctl
};

static struct v4l2_subdev_ops vm149c_ops = {
	.core = &vm149c_core_ops,
};

static int vm149c_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct vm149c_dev *dev;
	int32_t current_distance;
	uint32_t VcmStartCurrent = VCMDRV_START_CURRENT;
    uint32_t VcmRatedCurrent = VCMDRV_RATED_CURRENT;
    uint32_t VcmStepMode = VCMDRV_STEP_MODE;

	dev_info(&client->dev, "probing...\n");

	dev = devm_kzalloc(&client->dev, sizeof(struct vm149c_dev), GFP_KERNEL);
	if (NULL == dev)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&dev->sd, client, &vm149c_ops);
	dev_info(&client->dev, "probing successful\n");

    current_distance = VcmRatedCurrent - VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;
    dev->Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    dev->StartCurrent   = VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;
    dev->RatedCurrent   = dev->StartCurrent + MAX_LOG * dev->Step;
    dev->StepMode       = VcmStepMode;

	return 0;
}

/* ======================================================================== */

static int __exit vm149c_remove(
	struct i2c_client *client)
{
	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id vm149c_id[] = {
	{ VM149C_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id vm149c_of_match[] = {
	{.compatible = "silicon touch,vm149c-v4l2-i2c-subdev"},
	{ }
};

MODULE_DEVICE_TABLE(i2c, vm149c_id);

static struct i2c_driver vm149c_i2c_driver = {
	.driver = {
		.name = VM149C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vm149c_of_match
	},
	.probe = vm149c_probe,
	.remove = __exit_p(vm149c_remove),
	.id_table = vm149c_id,
};

module_i2c_driver(vm149c_i2c_driver);

MODULE_DESCRIPTION("vm149c auto focus controller driver");
MODULE_AUTHOR("George");
MODULE_LICENSE("GPL");
