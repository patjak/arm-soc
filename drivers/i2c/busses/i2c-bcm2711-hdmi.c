// SPDX-License-Identifier: GPL-2.0
/*
 * BCM2711 HDMI I2C device driver
 *
 * Copyright (C) 2019 SUSE LLC
 * Author: Patrik Jakobsson <pjakobsson@suse.de>
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#define DRIVER_NAME "i2c-bcm2711-hdmi"

#define HI2C_REG_CHAN_START	0x260
#define HI2C_REG_CHAN_RESET	0x264
#define HI2C_REG_CHAN_DONE	0x268

#define HI2C_CHAN_LEN		0x98
#define HI2C_FIFO_BASE		0x74
#define HI2C_CMD_SIZE		8
#define HI2C_NUM_CMDS		14

#define HI2C_CHAN_CTRL_READ	0x190C0001
#define HI2C_CHAN_CTRL_WRITE	0x180C0005

#define HI2C_CMD_SLAVE_ADDR	0x0
#define HI2C_CMD_RD_OFFSET	0x1
#define HI2C_CMD_XFER_LEN	0x9
#define HI2C_CMD_XFER_LEN_MASK	0x3f
#define HI2C_CMD_CFG		0xa
#define HI2C_CMD_CFG_DEF	(0xd << 4)
#define HI2C_CMD_CFG_RD		(0x3 << 0)
#define HI2C_CMD_CFG_WR		(0x0 << 0)
#define HI2C_CMD_COND		0xb
#define HI2C_CMD_COND_START	0x0
#define HI2C_CMD_COND_STOP	0x1
#define HI2C_CMD_FLUSH		0x14
#define HI2C_CMD_FLUSH_DEF	0x40

struct hi2c_dev {
	struct device *dev;
	void __iomem *regs;
	struct i2c_adapter adap;
	struct mutex hw_lock;
};

#define REG_READ(addr) ioread32(hdev->regs + (addr))
#define REG_WRITE(val, addr) iowrite32((val), hdev->regs + (addr))

#define CHAN_BASE(chan) (HI2C_CHAN_LEN * (chan))
#define CMDLIST_BASE(chan) (CHAN_BASE(chan) + 4)

static void clear_chan_status(struct hi2c_dev *hdev, int chan)
{
	REG_WRITE(1 << chan, HI2C_REG_CHAN_RESET);
}

static void clear_chan(struct hi2c_dev *hdev, int chan)
{
	int base, i;

	/* Clear control register */
	REG_WRITE(0, CHAN_BASE(chan));

	/* Clear command list */
	base = CMDLIST_BASE(chan);
	for (i = 0; i < HI2C_NUM_CMDS; i++) {
		REG_WRITE(0, base + i * HI2C_CMD_SIZE);
		REG_WRITE(0, base + i * HI2C_CMD_SIZE + 4);
	}
}

static int wait_chan_fence(struct hi2c_dev *hdev, int chan)
{
	int retries;
	u32 status;
	int i;

	/* Normally finishes in < 4 msecs so set to 8 msec */
	retries = 800;

	/*
	 * We don't know how to detect a failed transfer other than looking at
	 * the response time. If the response is faster than 250 usec we assume
	 * that no slave exists on that address.
	 */
	for (i = 0; i < retries; i++) {
		status = REG_READ(HI2C_REG_CHAN_DONE);
		if (status & (1 << chan)) {
			if (i < 25)
				return -EREMOTEIO;

			return 0;
		}
		udelay(10);
	}

	return -ETIMEDOUT;
}

static void start_chan(struct hi2c_dev *hdev, int chan)
{
	REG_WRITE(1 << chan, HI2C_REG_CHAN_START);
}

static void emit_cmd(struct hi2c_dev *hdev, int chan, int offset, u32 cmd,
		     u32 arg)
{
	int base;

	if (offset >= HI2C_NUM_CMDS)
		return;

	cmd = (cmd & 0x1f) | 0x100;

	base = CMDLIST_BASE(chan);
	base += offset * HI2C_CMD_SIZE;
	REG_WRITE(cmd, base);
	REG_WRITE(arg, base + 4);
}

static int write_i2c_data(struct hi2c_dev *hdev, int chan, u8 slave_addr,
			  u8 len, u8 *data)
{
	int ret, i, j, n = 0;
	u32 val;

	if (len > 8)
		return -EINVAL;

	/* Don't know how to do zero length writes for bus probing */
	if (len == 0)
		return -EINVAL;

	mutex_lock(&hdev->hw_lock);

	clear_chan(hdev, chan);
	clear_chan_status(hdev, chan);

	/* Set start condition */
	emit_cmd(hdev, chan, n++, HI2C_CMD_COND, HI2C_CMD_COND_START);

	/* Set slave address */
	emit_cmd(hdev, chan, n++, HI2C_CMD_SLAVE_ADDR, slave_addr);

	/* Set direction */
	emit_cmd(hdev, chan, n++, HI2C_CMD_CFG,
		 HI2C_CMD_CFG_DEF | HI2C_CMD_CFG_WR);

	/* Set number of bytes to write */
	emit_cmd(hdev, chan, n++, HI2C_CMD_XFER_LEN,
		 len & HI2C_CMD_XFER_LEN_MASK);

	/* Flush command list??? */
	emit_cmd(hdev, chan, n++, HI2C_CMD_FLUSH, HI2C_CMD_FLUSH_DEF);

	/* Data to send (stuff u8 into little endian u32 )*/
	for (i = 0; i < len; i += 4) {
		val = 0;
		for (j = 0; j < 4 && (i + j) < len; j++)
			val |= data[i + j] << (8 * j);

		emit_cmd(hdev, chan, n++, (i  / 4) + 1, val);
	}

	/* Set stop condition */
	emit_cmd(hdev, chan, n++, HI2C_CMD_COND, HI2C_CMD_COND_STOP);

	/* Setup transfer */
	REG_WRITE(HI2C_CHAN_CTRL_WRITE, CHAN_BASE(chan));

	/* Start transfer */
	start_chan(hdev, chan);

	/* Wait for response */
	ret = wait_chan_fence(hdev, chan);

	mutex_unlock(&hdev->hw_lock);

	if (ret) {
		dev_dbg(hdev->dev, "Write failed: slave=0x%x, len=%d\n",
			slave_addr, len);
	}

	return ret;
}

static int read_i2c_data(struct hi2c_dev *hdev, int chan, u16 slave_addr,
			 u8 offset, u8 len, u8 *data)
{
	int ret, n = 0;
	u32 buf[8];

	if (len > 32)
		return -EINVAL;

	mutex_lock(&hdev->hw_lock);

	clear_chan_status(hdev, chan);

	/* Set start condition */
	emit_cmd(hdev, chan, n++, HI2C_CMD_COND, HI2C_CMD_COND_START);

	/* Set slave address */
	emit_cmd(hdev, chan, n++, HI2C_CMD_SLAVE_ADDR, slave_addr);

	/* Set direction */
	emit_cmd(hdev, chan, n++, HI2C_CMD_CFG, HI2C_CMD_CFG_DEF | HI2C_CMD_CFG_RD);

	/* Set offset */
	emit_cmd(hdev, chan, n++, HI2C_CMD_RD_OFFSET, offset);

	/* Set number of bytes to read */
	emit_cmd(hdev, chan, n++, HI2C_CMD_XFER_LEN,
				(len << 6) | (1 & HI2C_CMD_XFER_LEN_MASK));

	/* Flush command list??? */
	emit_cmd(hdev, chan, n++, HI2C_CMD_FLUSH, HI2C_CMD_FLUSH_DEF);

	/* Set stop condition */
	emit_cmd(hdev, chan, n++, HI2C_CMD_COND, HI2C_CMD_COND_STOP);

	/* Setup transfer */
	REG_WRITE(HI2C_CHAN_CTRL_READ, CHAN_BASE(chan));

	/* Start transfer */
	start_chan(hdev, chan);

	/* Wait for response */
	ret = wait_chan_fence(hdev, chan);

	/* Copy data from FIFO registers */
	for (n = 0; n < len; n += 4)
		buf[n / 4] = REG_READ(CHAN_BASE(chan) + HI2C_FIFO_BASE + n);

	mutex_unlock(&hdev->hw_lock);
	memcpy(data + offset, buf, len);

	if (ret) {
		dev_dbg(hdev->dev, "Read failed: slave=0x%x, len=%d\n",
			slave_addr, len);
	}

	return ret;
}

static int bcm2711_hi2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			    int num)
{
	struct hi2c_dev *hdev = i2c_get_adapdata(adap);
	struct i2c_msg *msg;
	int i, j, ret = 0, len;

	for (i = 0; i < num; i++) {
		msg = &msgs[i];
		if (msg->flags & I2C_M_RD) {
			for (j = 0; j < msg->len; j += 32) {
				len = msg->len - j;
				if (len > 32)
					len = 32;

				ret = read_i2c_data(hdev, 2, msg->addr << 1, j,
						    len, msg->buf);
				if (ret)
					break;
			}
		} else {
			ret = write_i2c_data(hdev, 2, msg->addr << 1,
					     msg->len, msg->buf);

			if (ret)
				break;
		}

		if (ret)
			break;
	}

	if (ret == 0)
		ret = num;

	return ret;
}

static u32 bcm2711_hi2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm bcm2711_hi2c_algo = {
	.master_xfer	= bcm2711_hi2c_xfer,
	.functionality	= bcm2711_hi2c_func,
};

static int bcm2711_hi2c_probe(struct platform_device *pdev)
{
	struct i2c_adapter *adap;
	struct hi2c_dev *hdev;
	struct resource *mem;

	hdev = devm_kzalloc(&pdev->dev, sizeof(*hdev), GFP_KERNEL);
	if (!hdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, hdev);
	hdev->dev = &pdev->dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENOENT;

        hdev->regs = devm_ioremap_resource(&pdev->dev, mem);
        if (IS_ERR(hdev->regs))
                return PTR_ERR(hdev->regs);

	mutex_init(&hdev->hw_lock);

	adap = &hdev->adap;
	i2c_set_adapdata(adap, hdev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	snprintf(adap->name, sizeof(adap->name), "bcm2711 (%s)",
		 of_node_full_name(pdev->dev.of_node));
	adap->algo = &bcm2711_hi2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;
	adap->quirks = NULL;

	dev_dbg(hdev->dev, "%s loaded\n", of_node_full_name(pdev->dev.of_node));

	return i2c_add_adapter(adap);
}

static int bcm2711_hi2c_remove(struct platform_device *pdev)
{
	struct hi2c_dev *hdev = platform_get_drvdata(pdev);
	i2c_del_adapter(&hdev->adap);

	return 0;
}

static const struct of_device_id bcm2711_hi2c_of_match[] = {
	{ .compatible = "brcm,bcm2711-i2c-hdmi" },
	{},
};

MODULE_DEVICE_TABLE(of, bcm2711_hi2c_of_match);

static struct platform_driver bcm2711_hi2c_driver = {
	.probe		= bcm2711_hi2c_probe,
	.remove		= bcm2711_hi2c_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = bcm2711_hi2c_of_match,
	},
};

module_platform_driver(bcm2711_hi2c_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patrik Jakobsson <pjakobsson@suse.de>");
MODULE_DESCRIPTION("Driver for the BCM2711 HDMI I2C controller");
