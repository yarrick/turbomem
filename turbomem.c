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

/*

Observed flash layout of Turbo Memory 2GB board:

It is addressed by 512 byte sectors. 4kB of flash is available
every 8kB. After 512 sectors (256kB), next available sector
starts at next even 0x1000 address.

Read can be done on individual sectors. Writes have to be done using
a full 4kB block. Erase is done per 256 sectors.

Example:
00000000-00000007 = 4kB
00000010-00000017 = 4kB
00000020-00000027 = 4kB
00000030-00000037 = 4kB
...
000003D0-000003D7 = 4kB
000003E0-000003E7 = 4kB
000003F0-000003F7 = 4kB

00001000-00001007 = 4kB
00001010-00001017 = 4kB
...
000013E0-000013E7 = 4kB
000013F0-000013F7 = 4kB

00002000-00002007 = 4kB
00002010-00002017 = 4kB
...
000023E0-000023E7 = 4kB
000023F0-000023F7 = 4kB
and so on.

Erase can be done at address 0x0, 0x200, 0x1000, 0x1200, 0x2000, 0x2200,
0x3000, 0x3200 and so on.

The first 256kB contain serial number, option ROM and other data and is kept
as reserved. There is probably no problem in using it as normal flash though.

*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/mtd/mtd.h>

#define DRIVER_NAME "turbomem"
#define NAME_SIZE 32
#define RESERVED_SECTORS 0x200

#define STATUS_REGISTER (0x18)
#define STATUS_INTERRUPT_MASK (0x1F)
#define STATUS_BOOTING (0x00010000)

#define COMMAND_REGISTER (0x10)
#define COMMAND_START_DMA (1)
#define COMMAND_RESET (0x100)

#define INTERRUPT_CTRL_REGISTER (0x20)
#define INTERRUPT_CTRL_ENABLE_BITS (0x3)

/*
 * There are also other modes:
 * Some kind of read:  2 3 (mode 3 maybe better at reading sectors
 * 	written to multiple times?)
 * Some kind of write: 5 6
 * Unknown: 0x31 +more?
 */
enum iomode {
	MODE_READ = 1,
	MODE_WRITE = 7,
	MODE_ERASE = 0x11,
	MODE_NOP = 0x35,
};

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

/* Value from transfer_command->result */
enum command_result {
	RESULT_READ_BAD_ADDRESS   = 0x8003,
	RESULT_ERASE_FAILED       = 0x8004,
	RESULT_READ_FAILED        = 0x8012, /* Returned after reading a sector
		that has been written twice after erase */
	RESULT_READ_ERASED_SECTOR = 0x8FF2,
};

struct turbomem_info {
	struct device *dev;
	struct dentry *debugfs_dir;
	struct mtd_info mtd;
	char name[NAME_SIZE];
	void __iomem *mem;
	struct dma_pool *dmapool_cmd;
	spinlock_t lock;
	struct list_head transfer_queue;
	struct transferbuf_handle *curr_transfer;
	struct transferbuf_handle *idle_transfer;
	struct tasklet_struct tasklet;
	atomic_t irq_statusword;
	unsigned characteristics;
	unsigned flash_sectors;
	unsigned usable_flash_sectors;
};

static struct dentry *debugfs_root = NULL;

static u32 readle32(struct turbomem_info *turbomem, u32 offset)
{
	return le32_to_cpu(ioread32(turbomem->mem + offset));
}

static void writele32(struct turbomem_info *turbomem, u32 offset, u32 value)
{
	iowrite32(cpu_to_le32(value), turbomem->mem + offset);
}

static void turbomem_enable_interrupts(struct turbomem_info *turbomem,
	bool active)
{
	u32 reg;

	reg = readle32(turbomem, INTERRUPT_CTRL_REGISTER);
	if (active)
		reg |= INTERRUPT_CTRL_ENABLE_BITS;
	else
		reg &= ~INTERRUPT_CTRL_ENABLE_BITS;

	writele32(turbomem, INTERRUPT_CTRL_REGISTER, reg);
}

static void turbomem_calc_sectors(struct turbomem_info *turbomem)
{
	unsigned sectors;
	unsigned reg;
	unsigned limit8, limit14;
	unsigned d;
	unsigned c = 1;
	unsigned i = 0;

	/* Get device characteristics */
	reg = readle32(turbomem, 0x38);
	turbomem->characteristics = ((reg & 0xFFFFF) + 0x10000) & 0xFFFFF;

	d = (reg >> 0xC) & 0xF;
	do {
		c = c * 2;
		i++;
	} while (i < d);
	limit8 = i << 10;

	d = (reg >> 16) & 0xF;
	limit14 = d + 1;

	d = 0x400 << ((turbomem->characteristics >> 0xC) & 0xF);
	if (d > limit8)
		limit8 = d;

	d = ((turbomem->characteristics >> 0x16) & 0xF);
	if (d > limit14)
		limit14 = d;

	sectors = (limit8 * limit14) * 512;

	turbomem->flash_sectors = sectors;
	/* First three 256-sector blocks are reserved */
	turbomem->usable_flash_sectors = sectors - RESERVED_SECTORS;
}

static irqreturn_t turbomem_isr(int irq, void *dev)
{
	struct turbomem_info *turbomem = dev;
	u32 status;
	u32 reg;

	status = readle32(turbomem, STATUS_REGISTER);
	if (status == 0xFFFFFFFF || (status & STATUS_INTERRUPT_MASK) == 0)
		return IRQ_NONE;

	atomic_set(&turbomem->irq_statusword, status);
	turbomem_enable_interrupts(turbomem, 0);

	reg = readle32(turbomem, STATUS_REGISTER);
	writele32(turbomem, STATUS_REGISTER, reg & STATUS_INTERRUPT_MASK);

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

	memset(transferbuf->buf, 0, sizeof(struct transfer_command));
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
	writele32(turbomem, 4, (busaddr >> 32) & 0xFFFFFFFF);
	writele32(turbomem, 0, busaddr & 0xFFFFFFFF);
}

static void turbomem_setup_idle_transfer(struct turbomem_info *turbomem)
{
	struct transfer_command *idle_cmd = turbomem->idle_transfer->buf;
	memset(idle_cmd, 0, sizeof(struct transfer_command));

	idle_cmd->transfer_flags = cpu_to_le32(0x7FFFFFFE);
	idle_cmd->mode = MODE_NOP;
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
			"Blocked attempt to queue the idle transfer\n");
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

	/* TODO start more than one transfer at a time */
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
		writele32(turbomem, COMMAND_REGISTER, COMMAND_START_DMA);
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
		if (status == 1)
			/* Transfer completed */
			turbomem->curr_transfer->status = XFER_DONE;
		else
			/* Transfer failed */
			turbomem->curr_transfer->status = XFER_FAILED;
		complete_all(&turbomem->curr_transfer->completion);
		turbomem->curr_transfer = NULL;
	}
	spin_unlock_bh(&turbomem->lock);

	/* This will re-enable interrupts, directly or via idle transfer */
	turbomem_start_next_transfer(turbomem);
}

static sector_t turbomem_translate_lba(sector_t lba)
{
	/* Every other 4kB area is not used */
	sector_t lower = 2 * (lba & 0x1FF);
	/* 512 usable sectors appear at even intervals */
	sector_t upper = 0x1000 * (lba >> 9);
	return upper | lower;
}

static int turbomem_do_io(struct turbomem_info *turbomem, sector_t lba,
	int sectors, struct transferbuf_handle *xfer,
	dma_addr_t busaddr, enum iomode mode)
{
	struct transfer_command *cmd = xfer->buf;
	lba = turbomem_translate_lba(lba);

	cmd = xfer->buf;
	cmd->result = cpu_to_le32(3);
	cmd->transfer_flags = cpu_to_le32(0x80000001);
	cmd->mode = mode;
	cmd->transfer_size = cpu_to_le32(sectors);
	cmd->sector_addr = cpu_to_le32(lba);
	if (mode != MODE_READ)
		cmd->sector_addr2 = cpu_to_le32(lba);
	cmd->data_buffer = cpu_to_le64(busaddr);
	cmd->data_buffer_valid = 1;

	turbomem_queue_transfer(turbomem, xfer);
	turbomem_start_next_transfer(turbomem);

	wait_for_completion_interruptible(&xfer->completion);

	/* Check error on transfer */
	if (xfer->status != XFER_DONE)
		return -EIO;

	return 0;
}

#define HW_RESET_ATTEMPTS 50

static int turbomem_hw_init(struct turbomem_info *turbomem)
{
	u32 initregs;
	u32 reg;
	unsigned i;
	initregs = 0;
	for (i = 0; i < 4; i++) {
		initregs |= readle32(turbomem, i*8);
	}
	if (initregs) {
		for (i = 0; i < 4; i++) {
			reg = 0;
			if (i == 3) reg = 0x1F;
			writele32(turbomem, i*8, reg);
		}
		initregs = 0;
		for (i = 0; i < 4; i++) {
			initregs |= readle32(turbomem, i*8);
		}
		if (initregs) {
			u32 reg8 = 1 | readle32(turbomem, 8);
			writele32(turbomem, COMMAND_REGISTER, COMMAND_RESET);
			for (i = 0; i < HW_RESET_ATTEMPTS; i++) {
				if (i) msleep(100);
				writele32(turbomem, 8, reg8);
				reg = readle32(turbomem, STATUS_REGISTER);
				if ((reg & STATUS_BOOTING) == 0) break;
			}
			if (i >= HW_RESET_ATTEMPTS)
				return -EIO;
		}
	}

	reg = readle32(turbomem, 8);
	reg = (reg & 0xFFFFFFFB) | 1;
	writele32(turbomem, 8, reg);
	for (i = 0; i < HW_RESET_ATTEMPTS; i++) {
		if (i) msleep(100);
		reg = readle32(turbomem, STATUS_REGISTER);
		if ((reg & STATUS_BOOTING) == 0) break;
	}
	if (i >= HW_RESET_ATTEMPTS)
		return -EIO;

	turbomem_calc_sectors(turbomem);

	return 0;
}

static ssize_t turbomem_debugfs_read_orom(struct file *file,
	char __user *userbuf, size_t count, loff_t *ppos)
{
	struct transferbuf_handle *xfer;
	struct turbomem_info *turbomem = file->f_inode->i_private;
	dma_addr_t bus4k;
	u8 *buf4k;
	loff_t offset_backup;
	ssize_t retval;
	sector_t addr = 0x10 + (*ppos / 512);

	xfer = turbomem_transferbuf_alloc(turbomem);
	if (!xfer)
		return -ENOMEM;

	buf4k = dma_alloc_coherent(turbomem->dev, 4096, &bus4k, GFP_KERNEL);
	if (!buf4k) {
		turbomem_transferbuf_free(turbomem, xfer);
		return -ENOMEM;
	}
	memset(buf4k, 0, 4096);

	retval = turbomem_do_io(turbomem, addr, 4096/512, xfer, bus4k,
			MODE_READ);
	if (xfer->status == XFER_FAILED) {
		/* Found erased page, end of OROM */
		retval = 0;
		goto out;
	}

	offset_backup = *ppos & 0xFFFF000;
	*ppos &= 0xFFF; /* Read within 4k-buf */
	retval = simple_read_from_buffer(userbuf, count, ppos,
		buf4k, 4096);
	*ppos += offset_backup;
out:
	dma_free_coherent(turbomem->dev, 4096, buf4k, bus4k);
	turbomem_transferbuf_free(turbomem, xfer);
	return retval;
}

static const struct file_operations debugfs_orom_fops = {
	.read	= turbomem_debugfs_read_orom,
};

static ssize_t turbomem_debugfs_wipe_flash(struct file *file,
	const char __user *user_buf, size_t size, loff_t *ppos)
{
	sector_t lba;
	int errors;
	struct turbomem_info *turbomem = file->f_inode->i_private;

	dev_info(turbomem->dev, "Wiping flash!");

	lba = RESERVED_SECTORS;
	errors = 0;
	do {
		struct transferbuf_handle *xfer;
		int ret;

		xfer = turbomem_transferbuf_alloc(turbomem);
		if (!xfer)
			break;

		ret = turbomem_do_io(turbomem, lba, 0, xfer, 0, MODE_ERASE);
		if (ret) {
			errors++;
		}

		turbomem_transferbuf_free(turbomem, xfer);
		lba += 0x100;
	} while (lba < turbomem->flash_sectors);

	dev_info(turbomem->dev, "Erase complete: %d of %d blocks failed.\n",
		errors, turbomem->flash_sectors / 256);
	return size;
}

static const struct file_operations debugfs_wipe_flash_fops = {
	.write	= turbomem_debugfs_wipe_flash,
};

static void turbomem_debugfs_dev_add(struct turbomem_info *turbomem)
{
	if (IS_ERR_OR_NULL(debugfs_root))
		return;
	turbomem->debugfs_dir = debugfs_create_dir(dev_name(turbomem->dev),
		debugfs_root);

	if (IS_ERR_OR_NULL(turbomem->debugfs_dir))
		return;
	debugfs_create_file("orom", 0400, turbomem->debugfs_dir, turbomem,
		&debugfs_orom_fops);
	debugfs_create_file("wipe_flash", 0200, turbomem->debugfs_dir,
		turbomem, &debugfs_wipe_flash_fops);
}

static void turbomem_debugfs_dev_remove(struct turbomem_info *turbomem)
{
	if (IS_ERR_OR_NULL(turbomem->debugfs_dir))
		return;
	debugfs_remove_recursive(turbomem->debugfs_dir);
}

static int turbomem_mtd_exec(struct turbomem_info *turbomem, enum iomode mode,
	sector_t lba, int sectors, u_char *buf)
{
	struct transferbuf_handle *xfer;
	int result;
	int length;
	dma_addr_t busaddr = 0;
	enum dma_data_direction dir = DMA_NONE;

	lba += RESERVED_SECTORS;
	length = sectors * 512;
	xfer = turbomem_transferbuf_alloc(turbomem);
	if (!xfer)
		return -ENOMEM;

	if (buf) {
		if (mode == MODE_WRITE)
			dir = DMA_TO_DEVICE;
		else
			dir = DMA_FROM_DEVICE;

		busaddr = dma_map_single(turbomem->dev, buf, length, dir);
		if (!busaddr) {
			result = -ENOMEM;
			goto out;
		}
	}
	result = turbomem_do_io(turbomem, lba, sectors, xfer, busaddr, mode);
	if (busaddr)
		dma_unmap_single(turbomem->dev, busaddr, length, dir);

	if (result) {
		struct transfer_command *cmd = xfer->buf;
		dev_warn(turbomem->dev, "Error result %d (lba %08lX op %d)\n",
			le32_to_cpu(cmd->result), lba, mode);
		if (mode == MODE_READ && le32_to_cpu(cmd->result) ==
						RESULT_READ_ERASED_SECTOR) {
			/* Make up erased sector */
			memset(buf, 0xFF, length);
			result = 0;
		}
	}
out:
	turbomem_transferbuf_free(turbomem, xfer);
	return result;
}

static int turbomem_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct turbomem_info *turbomem = mtd->priv;
	int result;
	u64 start = instr->addr >> mtd->erasesize_shift;
	u64 end = (instr->addr + instr->len - 1) >> mtd->erasesize_shift;

	while (start <= end) {
		result = turbomem_mtd_exec(turbomem, MODE_ERASE,
			(start * 0x100), 0, NULL);
		if (result) {
			instr->state = MTD_ERASE_FAILED;
			instr->fail_addr = start << mtd->erasesize_shift;
			return result;
		}
		start++;
	}

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);
	return 0;
}

static int turbomem_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
                      size_t *retlen, u_char *buf)
{
	struct turbomem_info *turbomem = mtd->priv;
	int result;
	sector_t lba = from / 512;
	int sectors = len / 512;
	if (sectors > 8)
		sectors = 8;
	result = turbomem_mtd_exec(turbomem, MODE_READ,
			lba, sectors, buf);
	if (result)
		return result;
	*retlen = sectors * 512;
	return 0;
}

static int turbomem_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
                      size_t *retlen, const u_char *buf)
{
	struct turbomem_info *turbomem = mtd->priv;
	int result;
	sector_t lba = to / 512;
	int sectors = len / 512;
	if (sectors > 8)
		sectors = 8;
	result = turbomem_mtd_exec(turbomem, MODE_WRITE,
			lba, sectors, (u_char *) buf);
	if (result)
		return result;
	*retlen = sectors * 512;
	return 0;
}

static int turbomem_setup_mtd(struct turbomem_info *turbomem)
{
	struct mtd_info *mtd = &turbomem->mtd;
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->size = turbomem->usable_flash_sectors * 512;
	mtd->erasesize = 256*1024;
	mtd->writesize = 4096;
	mtd->writebufsize = 4096;

	mtd->_erase = turbomem_mtd_erase;
	mtd->_read = turbomem_mtd_read;
	mtd->_write = turbomem_mtd_write;

	mtd->owner = THIS_MODULE;
	mtd->name = turbomem->name;
	mtd->priv = turbomem;

	return mtd_device_register(mtd, NULL, 0);
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
		goto fail_have_struct;
	}

	pci_set_drvdata(dev, turbomem);
	pci_set_master(dev);
	turbomem->dev = &dev->dev;

	ret = pci_request_regions(dev, DRIVER_NAME);
	if (ret) {
		dev_err(&dev->dev, "Unable to request memory region\n");
		goto fail_enabled;
	}

	turbomem->mem = ioremap_nocache(pci_resource_start(dev, 0),
				pci_resource_len(dev, 0));
	if (!turbomem->mem) {
		dev_err(&dev->dev, "Unable to remap memory area\n");
		goto fail_have_regions;
	}

	ret = dma_set_mask(turbomem->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&dev->dev, "No usable DMA configuration\n");
		goto fail_have_iomap;
	}

	ret = turbomem_hw_init(turbomem);
	if (ret) {
		dev_err(&dev->dev, "Unable to initialize device\n");
		goto fail_have_iomap;
	}

	tasklet_init(&turbomem->tasklet, turbomem_tasklet,
		(unsigned long) turbomem);

	ret = request_irq(dev->irq, turbomem_isr, IRQF_SHARED,
			DRIVER_NAME, turbomem);
	if (ret) {
		dev_err(&dev->dev, "Unable to request IRQ\n");
		goto fail_have_iomap;
	}

	turbomem->dmapool_cmd = dma_pool_create(DRIVER_NAME "_cmd", &dev->dev,
		sizeof(struct transfer_command), 8, 0);
	if (!turbomem->dmapool_cmd) {
		dev_err(&dev->dev, "Unable to create DMA pool for commands\n");
		ret = -ENOMEM;
		goto fail_have_irq;
	}

	turbomem->idle_transfer = turbomem_transferbuf_alloc(turbomem);
	if (ret) {
		dev_err(&dev->dev, "Unable to allocate idle transfer job\n");
		goto fail_have_dmapool;
	}
	turbomem_start_idle_transfer(turbomem);

	spin_lock_init(&turbomem->lock);
	INIT_LIST_HEAD(&turbomem->transfer_queue);

	snprintf(turbomem->name, NAME_SIZE - 1, "TurboMemory@%s",
		dev_name(turbomem->dev));

	dev_info(&dev->dev, "Device characteristics: %05X, flash size: %d MB\n",
		turbomem->characteristics, turbomem->flash_sectors >> 11);

	ret = turbomem_setup_mtd(turbomem);
	if (ret) {
		dev_err(&dev->dev, "Unable to register to MTD layer\n");
		goto fail_have_idle_transfer;
	}

	turbomem_debugfs_dev_add(turbomem);

	return 0;

fail_have_idle_transfer:
	if (turbomem->idle_transfer)
		turbomem_transferbuf_free(turbomem, turbomem->idle_transfer);
fail_have_dmapool:
	dma_pool_destroy(turbomem->dmapool_cmd);
fail_have_irq:
	free_irq(dev->irq, turbomem);
	tasklet_kill(&turbomem->tasklet);
fail_have_iomap:
	iounmap(turbomem->mem);
fail_have_regions:
	pci_release_regions(dev);
fail_enabled:
	pci_disable_device(dev);
fail_have_struct:
	kfree(turbomem);
	return ret;
}

static void turbomem_remove(struct pci_dev *dev)
{
	struct turbomem_info *turbomem = pci_get_drvdata(dev);

	turbomem_debugfs_dev_remove(turbomem);
	mtd_device_unregister(&turbomem->mtd);
	if (turbomem->idle_transfer)
		turbomem_transferbuf_free(turbomem, turbomem->idle_transfer);
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

	BUILD_BUG_ON(sizeof(struct transfer_command) != 0x80);

	turbomem_debugfs_init();

	retval = pci_register_driver(&pci_driver);
	if (retval < 0) {
		turbomem_debugfs_cleanup();
		return retval;
	}

	return 0;
}

static void __exit turbomem_exit(void)
{
	pci_unregister_driver(&pci_driver);
	turbomem_debugfs_cleanup();
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel(R) Turbo Memory Controller NAND flash driver");
MODULE_AUTHOR("Erik Ekman <erik@kryo.se>");

module_init(turbomem_init);
module_exit(turbomem_exit);
