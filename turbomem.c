/*
 * Driver for Intel(R) Turbo Memory flash memory
 *
 * Copyright (c) 2013-2015 Erik Ekman <erik@kryo.se>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/debugfs.h>

#define DRIVER_NAME "turbomem"
#define NAME_SIZE 16

#define STATUS_REGISTER (0x18)
#define STATUS_INTERRUPT_MASK (0x1F)
#define STATUS_BOOTING (0x00010000)

#define INTERRUPT_CTRL_REGISTER (0x20)
#define INTERRUPT_CTRL_ENABLE_BITS (0x3)

enum xfer_status {
	XFER_QUEUED = 0,
	XFER_DONE,
	XFER_FAILED,
};

struct transferbuf_handle {
	/* Virtual address of struct transfer_command buffer */
	void *buf;
	/* DMA address to the same buffer, for writing to HW */
	dma_addr_t busaddr;
	/* Linked list item */
	struct list_head list;
	/* Operation status */
	enum xfer_status status;
	struct completion completion;
};

struct transfer_command {
	/* Offset 0000 */
	__le32 result;
	/* Offset 0004 */
	__le32 transfer_flags;
	/* Offset 0008 */
	__le64 next_command;
	/* Offset 0010 */
	u8 mode;
	u8 transfer_size;
	u16 reserved1;
	/* Offset 0014 */
	__le32 sector_addr;
	/* Offset 0018 */
	__le32 sector_addr2;
	/* Offset 001C */
	__le64 data_buffer;
	u64 reserved2;
	/* Offset 002C */
	__le64 metadata_buffer;
	/* Offset 0034 */
	u8 first_transfer;
	u8 last_transfer;
	u8 data_buffer_valid;
	u8 metadata_buffer_valid;
	u64 reserved3;
	/* Offset 0040 */
	u64 virtual_ptr1;
	u32 reserved4;
	/* Offset 004C */
	__le32 sector_addr3;
	/* Offset 0050 */
	u64 virtual_ptr2;
	u64 reserved5;
	u64 reserved6;
	u32 reserved7;
	/* Offset 006C */
	u64 virtual_ptr3;
	/* Offset 0074 */
	u64 virtual_ptr4;
	/* Offset 007C */
	u8 cmd_one;
	u8 reserved8;
	u16 reserved9;
} __attribute__((packed));

struct turbomem_info {
	struct device *dev;
	struct dentry *debugfs_dir;
	struct dentry *debugfs_f_orom;
	char name[NAME_SIZE];
	void __iomem *mem;
	struct dma_pool *dmapool_cmd;
	struct dma_pool *dmapool_data;
	spinlock_t lock;
	struct list_head transfer_queue;
	struct transferbuf_handle *curr_transfer;
	struct transferbuf_handle *idle_transfer;
	struct tasklet_struct tasklet;
	atomic_t irq_statusword;
	unsigned characteristics;
	unsigned flash_sectors;
};

static int major_nr;
static atomic_t cardid_allocator = ATOMIC_INIT(-1);
static struct dentry *debugfs_root = NULL;

static void turbomem_enable_interrupts(struct turbomem_info *turbomem,
	bool active)
{
	u32 reg;

	reg = le32_to_cpu(ioread32(turbomem->mem + INTERRUPT_CTRL_REGISTER));
	if (active)
		reg |= INTERRUPT_CTRL_ENABLE_BITS;
	else
		reg &= ~INTERRUPT_CTRL_ENABLE_BITS;

	iowrite32(cpu_to_le32(reg), turbomem->mem + INTERRUPT_CTRL_REGISTER);
}

static unsigned turbomem_calc_sectors(u32 reg0x38, unsigned chars)
{
	unsigned limit8, limit14;
	unsigned d = (reg0x38 >> 0xC) & 0xF;
	unsigned c = 1;
	unsigned i = 0;

	do {
		c = c * 2;
		i++;
	} while (i < d);
	limit8 = i << 10;

	d = (reg0x38 >> 16) & 0xF;
	limit14 = d + 1;

	d = 0x400 << ((chars >> 0xC) & 0xF);
	if (d > limit8)
		limit8 = d;

	d = ((chars >> 0x16) & 0xF);
	if (d > limit14)
		limit14 = d;

	return (limit8 * limit14) * 64;
}

static irqreturn_t turbomem_isr(int irq, void *dev)
{
	struct turbomem_info *turbomem = dev;
	u32 status;
	u32 reg;

	status = le32_to_cpu(ioread32(turbomem->mem + STATUS_REGISTER));
	if (status == 0xFFFFFFFF || (status & STATUS_INTERRUPT_MASK) == 0)
		return IRQ_NONE;

	dev_info(turbomem->dev, "Got IRQ on line %d, status is %08X\n",
		irq, status);

	atomic_set(&turbomem->irq_statusword, status);
	turbomem_enable_interrupts(turbomem, 0);

	reg = le32_to_cpu(ioread32(turbomem->mem + STATUS_REGISTER));
	iowrite32(cpu_to_le32(reg & STATUS_INTERRUPT_MASK),
		turbomem->mem + STATUS_REGISTER);

	tasklet_schedule(&turbomem->tasklet);

	return IRQ_HANDLED;
}

static struct transferbuf_handle *turbomem_transferbuf_alloc(
	struct turbomem_info *turbomem)
{
	struct transferbuf_handle *transferbuf;

	transferbuf = kzalloc(sizeof(*transferbuf), GFP_KERNEL);
	if (!transferbuf)
		return NULL;

	transferbuf->buf = dma_pool_alloc(turbomem->dmapool_cmd, GFP_KERNEL,
		&transferbuf->busaddr);
	if (!transferbuf->buf) {
		kfree(transferbuf);
		return NULL;
	}

	INIT_LIST_HEAD(&transferbuf->list);
	init_completion(&transferbuf->completion);
	return transferbuf;
}

/* Both the transfer_command and the transferbuf_handle will be freed. */
static void turbomem_transferbuf_free(struct turbomem_info *turbomem,
	struct transferbuf_handle *transferbuf)
{
	dma_pool_free(turbomem->dmapool_cmd, transferbuf->buf,
		transferbuf->busaddr);

	kfree(transferbuf);
}

static void turbomem_write_transfer_to_hw(struct turbomem_info *turbomem,
	struct transferbuf_handle *transfer)
{
	dma_addr_t busaddr = transfer->busaddr;
	/* Since we use 32-bit DMA mask upper should be zero, but.. */
	u32 upper = (busaddr >> 32) & 0xFFFFFFFF;
	u32 lower = busaddr & 0xFFFFFFFF;
	iowrite32(cpu_to_le32(upper), turbomem->mem + 4);
	iowrite32(cpu_to_le32(lower), turbomem->mem);
}

static void turbomem_setup_idle_transfer(struct turbomem_info *turbomem)
{
	struct transfer_command *idle_cmd = turbomem->idle_transfer->buf;
	memset(idle_cmd, 0, sizeof(struct transfer_command));

	idle_cmd->transfer_flags = cpu_to_le32(0x7FFFFFFE);
	idle_cmd->mode = 0x35; /* NOP command */
	idle_cmd->last_transfer = 1;
	idle_cmd->cmd_one = 0;
}

static void turbomem_start_idle_transfer(struct turbomem_info *turbomem)
{
	turbomem_setup_idle_transfer(turbomem);
	turbomem_write_transfer_to_hw(turbomem, turbomem->idle_transfer);
	turbomem_enable_interrupts(turbomem, 1);
}

static void turbomem_queue_transfer(struct turbomem_info *turbomem,
	struct transferbuf_handle *transfer)
{
	if (transfer == turbomem->idle_transfer) {
		dev_warn(turbomem->dev,
			"Blocked attempt to queue the idle transfer");
		WARN_ON(transfer == turbomem->idle_transfer);
		return;
	}

	spin_lock_bh(&turbomem->lock);
	list_add_tail(&transfer->list, &turbomem->transfer_queue);
	spin_unlock_bh(&turbomem->lock);
}

static void turbomem_start_next_transfer(struct turbomem_info *turbomem)
{
	spin_lock_bh(&turbomem->lock);

	if (!list_empty(&turbomem->transfer_queue)
		&& !turbomem->curr_transfer) {

		struct transfer_command *cmd;
		struct transferbuf_handle *handle;
		struct list_head *elem;
		elem = turbomem->transfer_queue.next;
		list_del(elem);
		handle= list_entry(elem, struct transferbuf_handle, list);

		/* Chain idle transfer as next item */
		cmd = handle->buf;
		turbomem_setup_idle_transfer(turbomem);
		cmd->next_command = cpu_to_le64(
			turbomem->idle_transfer->busaddr);

		/* Mark first job */
		cmd = handle->buf;
		cmd->first_transfer = 1;
		turbomem->curr_transfer = handle;

		/* Write addr, enable IRQ and DMA */
		turbomem_write_transfer_to_hw(turbomem, handle);
		turbomem_enable_interrupts(turbomem, 1);
		iowrite32(cpu_to_le32(1), turbomem->mem + 0x10);
	}

	/* If nothing queued, start idle transfer instead */
	if (!turbomem->curr_transfer)
		turbomem_start_idle_transfer(turbomem);

	spin_unlock_bh(&turbomem->lock);
}

static void turbomem_tasklet(unsigned long privdata)
{
	int status;
	u64 curr_transfer;
	struct turbomem_info *turbomem = (struct turbomem_info *) privdata;

	status = atomic_read(&turbomem->irq_statusword);
	curr_transfer = le64_to_cpu(ioread32(turbomem->mem) |
		((u64) ioread32(turbomem->mem + 4)) << 32);

	spin_lock_bh(&turbomem->lock);
	if (turbomem->curr_transfer) {
		if (status == 1) {
			/* Transfer completed */
			turbomem->curr_transfer->status = XFER_DONE;
		} else {
			/* Transfer failed */
			turbomem->curr_transfer->status = XFER_FAILED;
		}
		complete_all(&turbomem->curr_transfer->completion);
		turbomem->curr_transfer = NULL;
	}
	spin_unlock_bh(&turbomem->lock);

	/* This will re-enable interrupts, directly or via idle transfer */
	turbomem_start_next_transfer(turbomem);
}

#define HW_RESET_ATTEMPTS 50

static int turbomem_hw_init(struct turbomem_info *turbomem)
{
	u32 initregs;
	u32 reg;
	unsigned i;
	initregs = 0;
	for (i = 0; i < 4; i++) {
		initregs |= le32_to_cpu(ioread32(turbomem->mem + i*8));
	}
	if (initregs) {
		for (i = 0; i < 4; i++) {
			reg = 0;
			if (i == 3) reg = 0x1F;
			iowrite32(cpu_to_le32(reg), turbomem->mem + i*8);
		}
		initregs = 0;
		for (i = 0; i < 4; i++) {
			initregs |= le32_to_cpu(ioread32(turbomem->mem + i*8));
		}
		if (initregs) {
			u32 reg8 = 1 | le32_to_cpu(ioread32(turbomem->mem + 8));
			u32 reg16 = 0x100;
			iowrite32(cpu_to_le32(reg16), turbomem->mem + 16);
			for (i = 0; i < HW_RESET_ATTEMPTS; i++) {
				if (i) msleep(100);
				iowrite32(cpu_to_le32(reg8), turbomem->mem + 8);
				reg = le32_to_cpu(ioread32(
					turbomem->mem + STATUS_REGISTER));
				if ((reg & STATUS_BOOTING) == 0) break;
			}
			if (i >= HW_RESET_ATTEMPTS)
				return -EIO;
		}
	}

	reg = le32_to_cpu(ioread32(turbomem->mem + 8));
	reg = (reg & 0xFFFFFFFB) | 1;
	iowrite32(cpu_to_le32(reg), turbomem->mem + 8);
	for (i = 0; i < HW_RESET_ATTEMPTS; i++) {
		if (i) msleep(100);
		reg = le32_to_cpu(ioread32(turbomem->mem + STATUS_REGISTER));
		if ((reg & STATUS_BOOTING) == 0) break;
	}
	if (i >= HW_RESET_ATTEMPTS)
		return -EIO;

	/* Get device characteristics */
	reg = le32_to_cpu(ioread32(turbomem->mem + 0x38));
	turbomem->characteristics =
		(((reg& 0xFFFF0000) + 0x10000) & 0xF0000) | (reg & 0xFFFF);

	turbomem->flash_sectors = turbomem_calc_sectors(reg,
		turbomem->characteristics);

	return 0;
}

static ssize_t read_orom(struct file *file, char __user *userbuf,
	size_t count, loff_t *ppos)
{
	struct transferbuf_handle *xfer;
	struct transfer_command *cmd;
	struct turbomem_info *turbomem = file->f_inode->i_private;
	dma_addr_t bus4k;
	u8 *buf4k;
	loff_t offset_backup;
	ssize_t retval;
	int addr = 0x20 + 2 * (*ppos / 512);

	xfer = turbomem_transferbuf_alloc(turbomem);
	if (!xfer)
		return -ENOMEM;

	buf4k = dma_pool_alloc(turbomem->dmapool_data, GFP_KERNEL, &bus4k);
	if (!buf4k) {
		turbomem_transferbuf_free(turbomem, xfer);
		return -ENOMEM;
	}
	memset(buf4k, 0, 4096);

	cmd = xfer->buf;
	cmd->result = 3;
	cmd->transfer_flags = cpu_to_le32(0x80000001);
	cmd->mode = 1; // Read
	cmd->transfer_size = 8; // sectors *512 = 4k
	cmd->sector_addr = addr;
	cmd->data_buffer = cpu_to_le64(bus4k);
	cmd->data_buffer_valid = 1;

	turbomem_queue_transfer(turbomem, xfer);
	turbomem_start_next_transfer(turbomem);

	wait_for_completion_interruptible(&xfer->completion);

	if (xfer->status == XFER_FAILED) {
		/* Got error on transfer, end of OROM */
		retval = 0;
		goto out;
	}

	offset_backup = *ppos & 0xFFFF000;
	*ppos &= 0xFFF; /* Read within 4k-buf */
	retval = simple_read_from_buffer(userbuf, count, ppos,
		buf4k, 4096);
	*ppos += offset_backup;
out:
	dma_pool_free(turbomem->dmapool_data, buf4k, bus4k);
	turbomem_transferbuf_free(turbomem, xfer);
	return retval;
}

static const struct file_operations debugfs_orom_fops = {
	.read	= read_orom,
};

static void turbomem_debugfs_dev_add(struct turbomem_info *turbomem)
{
	if (IS_ERR_OR_NULL(debugfs_root))
		return;
	turbomem->debugfs_dir = debugfs_create_dir(turbomem->name,
		debugfs_root);

	if (IS_ERR_OR_NULL(turbomem->debugfs_dir))
		return;
	debugfs_create_file("orom", 0400, turbomem->debugfs_dir, turbomem,
		&debugfs_orom_fops);
}

static void turbomem_debugfs_dev_remove(struct turbomem_info *turbomem)
{
	if (IS_ERR_OR_NULL(turbomem->debugfs_dir))
		return;
	debugfs_remove_recursive(turbomem->debugfs_dir);
}

static int turbomem_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret;
	struct turbomem_info *turbomem;

	dev_info(&dev->dev, "Found Intel Turbo Memory Controller (rev %02X)\n",
		dev->revision);

	turbomem = kzalloc(sizeof(*turbomem), GFP_KERNEL);
	if (!turbomem)
		return -ENOMEM;

	ret = pci_enable_device(dev);
	if (ret) {
		dev_err(&dev->dev, "Unable to request memory region\n");
		goto fail_enable;
	}

	pci_set_master(dev);
	turbomem->dev = &dev->dev;

	ret = pci_request_regions(dev, DRIVER_NAME);
	if (ret) {
		dev_err(&dev->dev, "Unable to request memory region\n");
		goto fail_req_region;
	}

	turbomem->mem = ioremap_nocache(pci_resource_start(dev, 0),
				pci_resource_len(dev, 0));
	if (!turbomem->mem) {
		dev_err(&dev->dev, "Unable to remap memory area\n");
		goto fail_ioremap;
	}

	ret = dma_set_mask(turbomem->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&dev->dev, "No usable DMA configuration\n");
		goto fail_init;
	}

	ret = turbomem_hw_init(turbomem);
	if (ret) {
		dev_err(&dev->dev, "Unable to initialize device\n");
		goto fail_init;
	}

	tasklet_init(&turbomem->tasklet, turbomem_tasklet,
		(unsigned long) turbomem);

	ret = request_irq(dev->irq, turbomem_isr, IRQF_SHARED,
			DRIVER_NAME, turbomem);
	if (ret) {
		dev_err(&dev->dev, "Unable to request IRQ\n");
		goto fail_init;
	}

	turbomem->dmapool_cmd = dma_pool_create(DRIVER_NAME "_cmd", &dev->dev,
		sizeof(struct transfer_command), 8, 0);
	if (!turbomem->dmapool_cmd) {
		dev_err(&dev->dev, "Unable to create DMA pool for commands\n");
		ret = -ENOMEM;
		goto fail_irq;
	}

	turbomem->dmapool_data = dma_pool_create(DRIVER_NAME "_data", &dev->dev,
		4096, 8, 0);
	if (!turbomem->dmapool_data) {
		dev_err(&dev->dev, "Unable to create DMA pool for data\n");
		ret = -ENOMEM;
		goto fail_dmapool_cmd;
	}

	turbomem->idle_transfer = turbomem_transferbuf_alloc(turbomem);
	if (ret) {
		dev_err(&dev->dev, "Unable to allocate idle transfer job\n");
		goto fail_dmapool_data;
	}
	turbomem_start_idle_transfer(turbomem);

	spin_lock_init(&turbomem->lock);
	INIT_LIST_HEAD(&turbomem->transfer_queue);

	/* Generate unique card name */
	snprintf(turbomem->name, NAME_SIZE - 1, DRIVER_NAME "%c",
		atomic_inc_return(&cardid_allocator) + 'a');

	dev_info(&dev->dev, "Device characteristics: %05X, flash size: %d MB\n",
		turbomem->characteristics, turbomem->flash_sectors >> 8);

	pci_set_drvdata(dev, turbomem);

	turbomem_debugfs_dev_add(turbomem);

	return 0;

fail_dmapool_data:
	dma_pool_destroy(turbomem->dmapool_data);
fail_dmapool_cmd:
	dma_pool_destroy(turbomem->dmapool_cmd);
fail_irq:
	free_irq(dev->irq, turbomem);
	tasklet_kill(&turbomem->tasklet);
fail_init:
	iounmap(turbomem->mem);
fail_ioremap:
	pci_release_regions(dev);
fail_req_region:
	pci_disable_device(dev);
fail_enable:
	kfree(turbomem);
	return ret;
}

static void turbomem_remove(struct pci_dev *dev)
{
	struct turbomem_info *turbomem = pci_get_drvdata(dev);

	turbomem_debugfs_dev_remove(turbomem);
	if (turbomem->idle_transfer)
		turbomem_transferbuf_free(turbomem, turbomem->idle_transfer);
	dma_pool_destroy(turbomem->dmapool_data);
	dma_pool_destroy(turbomem->dmapool_cmd);
	free_irq(dev->irq, turbomem);
	tasklet_kill(&turbomem->tasklet);
	iounmap(turbomem->mem);
	pci_release_regions(dev);
	pci_disable_device(dev);
	pci_set_drvdata(dev, NULL);
	kfree(turbomem);
}

static void turbomem_debugfs_init(void)
{
	debugfs_root = debugfs_create_dir(DRIVER_NAME, NULL);
}

static void turbomem_debugfs_cleanup(void)
{
	debugfs_remove_recursive(debugfs_root);
	debugfs_root = NULL;
}

#define PCI_DEVICE_ID_INTEL_TURBOMEMORY (0x444e)

static const struct pci_device_id turbomem_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_TURBOMEMORY), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, turbomem_ids);

static struct pci_driver pci_driver = {
	.name = DRIVER_NAME,
	.id_table = turbomem_ids,
	.probe = turbomem_probe,
	.remove = turbomem_remove,
};

static int __init turbomem_init(void)
{
	int retval;
	int err;

	turbomem_debugfs_init();

	BUILD_BUG_ON(sizeof(struct transfer_command) != 0x80);
	retval = pci_register_driver(&pci_driver);
	if (retval)
		return -ENOMEM;

	err = major_nr = register_blkdev(0, DRIVER_NAME);
	if (err < 0) {
		retval = -EIO;
		goto fail_register_driver;
	}

	return 0;

fail_register_driver:
	pci_unregister_driver(&pci_driver);
	return retval;
}

static void __exit turbomem_exit(void)
{
	turbomem_debugfs_cleanup();
	unregister_blkdev(major_nr, DRIVER_NAME);
	pci_unregister_driver(&pci_driver);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel(R) Turbo Memory PCIe flash block device driver");
MODULE_AUTHOR("Erik Ekman <erik@kryo.se>");

module_init(turbomem_init);
module_exit(turbomem_exit);
