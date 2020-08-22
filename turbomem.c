/*
 * Driver for Intel(R) Turbo Memory Controller
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
a full 4kB block. Erase is done per 512 sectors.

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

Erase can be done at address 0x0, 0x1000, 0x2000, 0x3000 and so on.

The first 256kB contain serial number, option ROM and other data and is kept
as reserved.

The flash chips mounted on the board are Intel SD74 SLC NAND, which has
2kB page size, 64byte OOB and rated at 26 MB/s read and 7.5 MB/s write speed.
The 2GB board has 2x JS29F08G08CANC1 while the 1GB board has 2x JS29F04G08AANB1.
The controller chip only allows 4kB pages and manages the OOB data.

*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/mtd/mtd.h>
#include <linux/pm.h>
#include <linux/version.h>

static int debug;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug mode (1=log all I/O)");

#define DBG(fmt, args...) do { if (debug) printk(KERN_DEBUG "turbomem: " fmt, ##args); } while (0)

#define DRIVER_NAME "turbomem"
#define NAME_SIZE 32

/* Addressable unit */
#define NAND_SECTOR_SIZE 512
/* Unit for reads/writes */
#define NAND_SECTORS_PER_PAGE 8
#define NAND_PAGE_SIZE ((NAND_SECTORS_PER_PAGE)*(NAND_SECTOR_SIZE))
#define NAND_PAGE_OFFSET(x) ((x) % (NAND_PAGE_SIZE))
/* Unit for erasing */
#define NAND_SECTORS_PER_BLOCK 512
#define NAND_BLOCK_SIZE ((NAND_SECTORS_PER_BLOCK)*(NAND_SECTOR_SIZE))

#define NUM_SECTORS(x) ((x)/(NAND_SECTOR_SIZE))

#define BBT_BLOCKS 4
/* First block is for OROM and things, then bad block tables */
#define RESERVED_SECTORS ((NAND_SECTORS_PER_BLOCK) * (1 + BBT_BLOCKS))


#define TRANSFER_CMD_ADDR_LOWER_REGISTER (0)
#define TRANSFER_CMD_ADDR_UPPER_REGISTER (4)

#define COMMAND_REGISTER (0x10)
#define COMMAND_START_DMA (1)
#define COMMAND_RESET (0x100)

#define STATUS_REGISTER (0x18)
#define STATUS_INTERRUPT_MASK (0x1F)
#define STATUS_BOOTING (0x00010000)

#define INTERRUPT_CTRL_REGISTER (0x20)
#define INTERRUPT_CTRL_ENABLE_BITS (0x3)

/*
 * There are also other modes:
 * Some kind of read:  2 3 (mode 3 maybe better at reading sectors
 *     written to multiple times?)
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
	/* Operation status */
	enum xfer_status status;
	/* IRQ completion */
	struct completion completion;
};

/*
 * This struct is given to the device to initiate a transfer.
 * The bus address of it is written to the 64bit register at offset 0.
 * Set the lowest bit in register 0x10 to start the transfer.
 * A NOP command is usually put at the end of each chain, but this
 * is not required.
 */
struct transfer_command {
	/* Offset 0000 */
	/* 3 written here initially. After transfer an errorcode will be
	 * written */
	__le32 result;
	/* Offset 0004 */
	/* Flags. Highest bit and lowest bit normally used. Highest bit
	 * means not executed? */
	__le32 transfer_flags;
	/* Offset 0008 */
	/* Bus address of next struct transfer_command buffer. Zero if last. */
	__le64 next_command;
	/* Offset 0010 */
	/* Type of operation. Read, write, erase etc. */
	u8 mode;
	/* Number of 512b sectors to transfer. For reading: 1-8, writing: 8.
	 * Set to zero when erasing */
	u8 transfer_size;
	u16 reserved1;
	/* Offset 0014 */
	/* Sector to start transfer/erase at. */
	__le32 sector_addr;
	/* Offset 0018 */
	/* Write same sector here as 'sector_addr' for writing/erasing. */
	__le32 sector_addr2;
	/* Offset 001C */
	/* Bus address of databuffer. Used for reads/writes. */
	__le64 data_buffer;
	u64 reserved2;
	/* Offset 002C */
	/* Bus address of metadata buffer. Unknown contents/size */
	__le64 metadata_buffer;
	/* Offset 0034 */
	/* Set to 1 if this is first transfer in the chain */
	u8 first_transfer;
	/* Set to 1 if this is last transfer in the chain */
	u8 last_transfer;
	/* Set to 1 if data buffer address at offset 001C above is valid */
	u8 data_buffer_valid;
	/* Set to 1 if metadata buffer address at offset 002C above is valid */
	u8 metadata_buffer_valid;
	u64 reserved3;
	/* Offset 0040 */
	/* The windows driver uses this field to link the transfer command
	 * structs. Addresses are virtual, not used by hardware. */
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
} __packed;

/* Value from transfer_command->result */
enum command_result {
	/* When doing IO to nonexisting address */
	RESULT_BAD_ADDRESS        = 0x8003,
	/* When writing/erasing bad block */
	RESULT_BAD_BLOCK          = 0x8004,
	/* When reading sector written more than once */
	RESULT_READ_FAILED        = 0x8012,
	/* When reading erased sector */
	RESULT_READ_ERASED_SECTOR = 0x8FF2,
};

/*
 * Bad block table will contain 4096 entries for 1GB board, 8192 for 2GB
 * and 16384 for 4GB version. It will be stored in two eraseblocks, with one
 * primary and the other as mirror. The bbt[] uses one bit per eraseblock,
 * so all sizes of the struct will fit in one page (4096 bytes).
 */
struct turbomem_bbt {
	u32 magic;
	u32 version;
	u32 eraseblocks;
	u8 bbt[0];
};

struct turbomem_info {
	struct device *dev;
	struct dentry *debugfs_dir;
	struct mtd_info mtd;
	char name[NAME_SIZE];
	void __iomem *mem;
	struct dma_pool *dmapool_cmd;
	struct mutex lock;
	struct transferbuf_handle *curr_transfer;
	struct transferbuf_handle *idle_transfer;
	u32 irq_statusword;
	unsigned characteristics;
	unsigned flash_sectors;
	unsigned usable_flash_sectors;
	struct turbomem_bbt *bbt;
};

static struct dentry *debugfs_root;

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
	unsigned i = 0;

	/* Get device characteristics */
	reg = readle32(turbomem, 0x38);
	turbomem->characteristics = ((reg & 0xFFFFF) + 0x10000) & 0xFFFFF;

	d = (reg >> 0xC) & 0xF;
	do {
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
	/* First 512-sector block is reserved */
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

	turbomem->irq_statusword = status;
	turbomem_enable_interrupts(turbomem, 0);

	reg = readle32(turbomem, STATUS_REGISTER);
	writele32(turbomem, STATUS_REGISTER, reg & STATUS_INTERRUPT_MASK);

	if (turbomem->curr_transfer)
		complete_all(&turbomem->curr_transfer->completion);

	return IRQ_HANDLED;
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

	turbomem->curr_transfer = transfer;
	writele32(turbomem, TRANSFER_CMD_ADDR_UPPER_REGISTER,
					(busaddr >> 16 >> 16) & 0xFFFFFFFF);
	writele32(turbomem, TRANSFER_CMD_ADDR_LOWER_REGISTER,
					busaddr & 0xFFFFFFFF);
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
	init_completion(&transferbuf->completion);
	return transferbuf;
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
	struct transfer_command *cmd;

	lba = turbomem_translate_lba(lba);

	/* We must have the lock here */
	BUG_ON(mutex_is_locked(&turbomem->lock) == 0);

	/* Setup transfer command */
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

	/* Chain idle transfer as next item */
	turbomem_setup_idle_transfer(turbomem);
	cmd->next_command = cpu_to_le64(
		turbomem->idle_transfer->busaddr);

	/* Mark first job */
	cmd->first_transfer = 1;

	/* Write addr, enable IRQ and DMA */
	turbomem_write_transfer_to_hw(turbomem, xfer);
	turbomem_enable_interrupts(turbomem, 1);
	writele32(turbomem, COMMAND_REGISTER, COMMAND_START_DMA);

	/* Wait for interrupt completion */
	wait_for_completion_io(&xfer->completion);

	if (turbomem->irq_statusword == 1)
		/* Transfer completed */
		turbomem->curr_transfer->status = XFER_DONE;
	else
		/* Transfer failed */
		turbomem->curr_transfer->status = XFER_FAILED;

	/* Setup new idle transfer, will enable interrupts again */
	turbomem_start_idle_transfer(turbomem);

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
	for (i = 0; i < 4; i++)
		initregs |= readle32(turbomem, i*8);
	if (initregs) {
		for (i = 0; i < 4; i++) {
			reg = 0;
			if (i == 3)
				reg = 0x1F;
			writele32(turbomem, i*8, reg);
		}
		initregs = 0;
		for (i = 0; i < 4; i++)
			initregs |= readle32(turbomem, i*8);
		if (initregs) {
			u32 reg8 = 1 | readle32(turbomem, 8);

			writele32(turbomem, COMMAND_REGISTER, COMMAND_RESET);
			for (i = 0; i < HW_RESET_ATTEMPTS; i++) {
				if (i)
					msleep(100);
				writele32(turbomem, 8, reg8);
				reg = readle32(turbomem, STATUS_REGISTER);
				if ((reg & STATUS_BOOTING) == 0)
					break;
			}
			if (i >= HW_RESET_ATTEMPTS)
				return -EIO;
		}
	}

	reg = readle32(turbomem, 8);
	reg = (reg & 0xFFFFFFFB) | 1;
	writele32(turbomem, 8, reg);
	for (i = 0; i < HW_RESET_ATTEMPTS; i++) {
		if (i)
			msleep(100);
		reg = readle32(turbomem, STATUS_REGISTER);
		if ((reg & STATUS_BOOTING) == 0)
			break;
	}
	if (i >= HW_RESET_ATTEMPTS)
		return -EIO;

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
	sector_t addr = 0x10 + NUM_SECTORS(*ppos);

	xfer = turbomem_transferbuf_alloc(turbomem);
	if (!xfer)
		return -ENOMEM;

	buf4k = dma_alloc_coherent(turbomem->dev, NAND_PAGE_SIZE, &bus4k,
			GFP_KERNEL);
	if (!buf4k) {
		turbomem_transferbuf_free(turbomem, xfer);
		return -ENOMEM;
	}
	memset(buf4k, 0, NAND_PAGE_SIZE);

	mutex_lock(&turbomem->lock);

	retval = turbomem_do_io(turbomem, addr, NUM_SECTORS(NAND_PAGE_SIZE),
			xfer, bus4k, MODE_READ);

	mutex_unlock(&turbomem->lock);
	if (xfer->status == XFER_FAILED) {
		/* Found erased page, end of OROM */
		retval = 0;
		goto out;
	}

	offset_backup = *ppos & 0xFFFF000;
	*ppos = NAND_PAGE_OFFSET(*ppos);
	retval = simple_read_from_buffer(userbuf, count, ppos,
		buf4k, NAND_PAGE_SIZE);
	*ppos += offset_backup;
out:
	dma_free_coherent(turbomem->dev, NAND_PAGE_SIZE, buf4k, bus4k);
	turbomem_transferbuf_free(turbomem, xfer);
	return retval;
}

static const struct file_operations debugfs_orom_fops = {
	.read	= turbomem_debugfs_read_orom,
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

	/* We must have the lock here */
	BUG_ON(mutex_is_locked(&turbomem->lock) == 0);

	length = sectors * NAND_SECTOR_SIZE;
	xfer = turbomem_transferbuf_alloc(turbomem);
	if (!xfer)
		return -ENOMEM;

	if (buf) {
		if (mode == MODE_WRITE)
			dir = DMA_TO_DEVICE;
		else
			dir = DMA_FROM_DEVICE;

		busaddr = dma_map_single(turbomem->dev, buf, length, dir);
		if (dma_mapping_error(turbomem->dev, busaddr)) {
			dev_err(turbomem->dev, "Failed to map DMA buffer\n");
			result = -EIO;
			goto out;
		}
	} else if (mode != MODE_ERASE) {
		/* Buffer required if not erasing */
		return -EINVAL;
	}
	result = turbomem_do_io(turbomem, lba, sectors, xfer, busaddr, mode);
	if (busaddr)
		dma_unmap_single(turbomem->dev, busaddr, length, dir);

	if (result) {
		struct transfer_command *cmd = xfer->buf;

		if (mode == MODE_READ && le32_to_cpu(cmd->result) ==
						RESULT_READ_ERASED_SECTOR) {
			/* Make up erased sector */
			memset(buf, 0xFF, length);
			result = 0;
		} else {
			dev_warn(turbomem->dev,
				"IO error: result %08X (lba %08llX op %d)\n",
				le32_to_cpu(cmd->result),
				(unsigned long long) lba, mode);
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
	u64 pos = instr->addr / NAND_SECTOR_SIZE;
	u64 end = (instr->addr + instr->len - 1) / NAND_SECTOR_SIZE;

	DBG("Erase from addr %08llX (sector %08llX) len %llu\n", instr->addr,
		pos, instr->len);
	mutex_lock(&turbomem->lock);
	while (pos <= end) {
		DBG("Suberase lba %08llX\n", RESERVED_SECTORS + pos);
		result = turbomem_mtd_exec(turbomem, MODE_ERASE,
				RESERVED_SECTORS + pos, 0, NULL);
		if (result) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0)
			instr->state = MTD_ERASE_FAILED;
#else
			result = -EIO;
#endif
			instr->fail_addr = pos * NAND_SECTOR_SIZE;
			mutex_unlock(&turbomem->lock);
			return result;
		}
		pos += NAND_SECTORS_PER_BLOCK;
	}
	mutex_unlock(&turbomem->lock);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0)
	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);
	return 0;
#else
	return result;
#endif
}

static int turbomem_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
				size_t *retlen, u_char *buf)
{
	struct turbomem_info *turbomem = mtd->priv;
	u_char *tempbuf = NULL;
	u_char *readbuf;
	size_t bytes_read = 0;
	int result = 0;
	/* Round to even page */
	unsigned offset = NAND_PAGE_OFFSET(from);
	sector_t lba = NUM_SECTORS(from) & 0xFFFFFFF8;

	DBG("Read len %zu from addr %08llX, sector %08llX\n", len, from,
		(unsigned long long) lba);
	mutex_lock(&turbomem->lock);
	if (offset || NAND_PAGE_OFFSET(len)) {
		/* Uneven offset or length, need a bounce buffer */
		tempbuf = kmalloc(NAND_PAGE_SIZE, GFP_KERNEL | GFP_DMA);
		if (!tempbuf) {
			result = -ENOMEM;
			goto out;
		}
	}
	while (bytes_read < len) {
		size_t to_read = len - bytes_read;

		if (tempbuf)
			readbuf = tempbuf;
		else
			readbuf = buf;
		DBG("Subread %zu bytes to lba %08llX offset %u to %p\n",
			to_read, (unsigned long long) lba, offset, readbuf);
		/* Read from flash */
		result = turbomem_mtd_exec(turbomem, MODE_READ,
				RESERVED_SECTORS + lba,
				NUM_SECTORS(NAND_PAGE_SIZE), readbuf);
		if (result) {
			goto out;
		}
		if (to_read > NAND_PAGE_SIZE - offset)
			to_read = NAND_PAGE_SIZE - offset;
		/* Return read data, handle partial request */
		if (tempbuf) {
			memcpy(buf, readbuf + offset, to_read);
			DBG("Copied %zu bytes to buf %p\n", to_read, buf);
		} else {
			DBG("Read %zu bytes to buf %p\n", to_read, buf);
		}
		buf += to_read;
		lba += NUM_SECTORS(NAND_PAGE_SIZE);
		bytes_read += to_read;
		offset = 0; /* Only first read can be misaligned */
	}
out:
	kfree(tempbuf);
	mutex_unlock(&turbomem->lock);
	*retlen = bytes_read;
	return result;
}

static int turbomem_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
				size_t *retlen, const u_char *buf)
{
	struct turbomem_info *turbomem = mtd->priv;
	int result;
	sector_t lba = NUM_SECTORS(to);
	size_t bytes_written = 0;

	DBG("Write len %zu to addr %08llX, sector %08llX\n",
		len, to, (unsigned long long) lba);
	mutex_lock(&turbomem->lock);
	/* Write max one page at a time */
	while (bytes_written < len) {
		int sectors = NUM_SECTORS(len - bytes_written);

		if (sectors > NUM_SECTORS(NAND_PAGE_SIZE))
			sectors = NUM_SECTORS(NAND_PAGE_SIZE);
		DBG("Subwrite %d sectors to lba %08llX\n", sectors,
			(unsigned long long) lba);
		result = turbomem_mtd_exec(turbomem, MODE_WRITE,
			RESERVED_SECTORS + lba, sectors, (u_char *) buf);
		if (result)
			goto out;
		buf += NAND_SECTOR_SIZE * sectors;
		bytes_written += NAND_SECTOR_SIZE * sectors;
		lba += sectors;
	}
out:
	mutex_unlock(&turbomem->lock);
	*retlen = bytes_written;
	return result;
}

#define BBT_MAGIC_PRIMARY (0xbadb10c0)
#define BBT_MAGIC_MIRROR  (0xbadb10c1)

static int turbomem_read_bbt(struct turbomem_info *turbomem)
{
	unsigned i;
	int result;
	u8 *buf = kzalloc(NAND_PAGE_SIZE, GFP_KERNEL | GFP_DMA);
	struct turbomem_bbt *bbt = NULL;

	if (!buf)
		return -ENOMEM;

	mutex_lock(&turbomem->lock);
	for (i = NAND_SECTORS_PER_BLOCK; i < RESERVED_SECTORS;
						i += NAND_SECTORS_PER_BLOCK) {
		result = turbomem_mtd_exec(turbomem, MODE_READ,
			i, NAND_SECTORS_PER_PAGE, buf);
		if (result)
			break;
		bbt = (struct turbomem_bbt *) buf;
		/* Found a table */
		if (bbt->magic == BBT_MAGIC_PRIMARY ||
				bbt->magic == BBT_MAGIC_MIRROR) {
			turbomem->bbt = bbt;
			break;
		}
	}
	mutex_unlock(&turbomem->lock);

	if (turbomem->bbt)
		return 0;

	kfree(buf);
	if (result)
		return result; /* Read error */
	else
		return -ENODATA; /* Found no bbt data */
}

/* Convert user addr to eraseblock index */
static unsigned turbomem_get_eraseblock(loff_t addr)
{
	unsigned sector = RESERVED_SECTORS + (addr / NAND_SECTOR_SIZE);

	return sector / (NAND_SECTORS_PER_BLOCK);
}

static void turbomem_markbad(struct turbomem_bbt *bbt, unsigned eb)
{
	if (eb > bbt->eraseblocks)
		return;

	bbt->bbt[eb / 8] |= (1 << (eb & 7));
}

static int turbomem_isbad(struct turbomem_bbt *bbt, unsigned eb)
{
	if (eb > bbt->eraseblocks)
		return 0;

	if (bbt->bbt[eb / 8] & (1 << (eb & 7)))
		return 1;
	return 0;
}

static int turbomem_count_bad(struct turbomem_bbt *bbt)
{
	int i = 0;
	int count = 0;
	u8 *table = bbt->bbt;

	/* Count number of bad blocks in table */
	while (i < bbt->eraseblocks) {
		int pos;

		for (pos = 0; pos < 8; pos++) {
			if (*table & (1 << pos))
				count++;
			i++;
		}
		table++;
	}
	return count;
}

static int turbomem_format_build_bbt(struct turbomem_info *turbomem)
{
	struct turbomem_bbt *bbt = NULL;
	unsigned i;

	bbt = kzalloc(NAND_PAGE_SIZE, GFP_KERNEL | GFP_DMA);
	if (!bbt)
		return -ENOMEM;

	bbt->magic = BBT_MAGIC_PRIMARY;
	bbt->version = 1;
	bbt->eraseblocks = turbomem->flash_sectors / NAND_SECTORS_PER_BLOCK;
	mutex_lock(&turbomem->lock);
	for (i = 0; i < turbomem->usable_flash_sectors;
						i += NAND_SECTORS_PER_BLOCK) {
		int result;
		struct transferbuf_handle *xfer;

		xfer = turbomem_transferbuf_alloc(turbomem);
		if (!xfer) {
			mutex_unlock(&turbomem->lock);
			return -ENOMEM;
		}
		/* Erase usable flash looking for bad blocks */
		result = turbomem_do_io(turbomem, RESERVED_SECTORS + i,
			0, xfer, 0, MODE_ERASE);
		if (result) {
			/* Bad block */
			loff_t addr = i * (NAND_SECTOR_SIZE);
			unsigned eb = turbomem_get_eraseblock(addr);

			turbomem_markbad(bbt, eb);
		}
		turbomem_transferbuf_free(turbomem, xfer);
	}
	mutex_unlock(&turbomem->lock);
	turbomem->bbt = bbt;
	return 0;
}

static int turbomem_save_bbt(struct turbomem_info *turbomem)
{
	struct turbomem_bbt *bbt_out;
	unsigned i;
	int result;

	bbt_out = kzalloc(NAND_PAGE_SIZE, GFP_KERNEL | GFP_DMA);
	if (!bbt_out)
		return -ENOMEM;

	memcpy(bbt_out, turbomem->bbt, sizeof(struct turbomem_bbt) +
		((turbomem->flash_sectors / (NAND_SECTORS_PER_BLOCK)) + 7) / 8);
	mutex_lock(&turbomem->lock);
	for (i = NAND_SECTORS_PER_BLOCK; i < RESERVED_SECTORS;
						i += NAND_SECTORS_PER_BLOCK) {
		/* Erase new BBT spot */
		result = turbomem_mtd_exec(turbomem, MODE_ERASE,
			i, 0, NULL);
		if (result == 0) {
			/* Write BBT */
			result = turbomem_mtd_exec(turbomem, MODE_WRITE, i,
				NAND_SECTORS_PER_PAGE, (u_char *) bbt_out);
		}
		if (result) {
			/* Mark bad in main bbt and this copy */
			turbomem_markbad(turbomem->bbt,
				i / NAND_SECTORS_PER_BLOCK);
			turbomem_markbad(bbt_out, i / NAND_SECTORS_PER_BLOCK);
			continue;
		}
		/* Erase and write successful */
		if (bbt_out->magic == BBT_MAGIC_PRIMARY) {
			dev_info(turbomem->dev,
				"Main BBT v%d written at sector %08X\n",
				bbt_out->version, i);
			bbt_out->magic = BBT_MAGIC_MIRROR;
		} else if (bbt_out->magic == BBT_MAGIC_MIRROR) {
			dev_info(turbomem->dev,
				"Spare BBT v%d written at sector %08X\n",
				bbt_out->version, i);
			/* All done. */
			mutex_unlock(&turbomem->lock);
			return 0;
		}
	}
	mutex_unlock(&turbomem->lock);

	/* Failed to write two copies of BBT */
	return -EIO;
}

static int turbomem_init_bbt(struct turbomem_info *turbomem)
{
	int ret;

	ret = turbomem_read_bbt(turbomem);
	if (ret == -ENODATA) {
		dev_warn(turbomem->dev,
			"No bad block data found, will format and create new!\n");
		ret = turbomem_format_build_bbt(turbomem);
		if (ret)
			return ret;
		ret = turbomem_save_bbt(turbomem);
		if (ret) {
			kfree(turbomem->bbt);
			return ret;
		}
	} else if (ret) {
		dev_err(turbomem->dev, "Failed looking for bad block data\n");
		return ret;
	}
	dev_info(turbomem->dev,
		"Using bad block table version %d, with %d bad blocks\n",
		turbomem->bbt->version, turbomem_count_bad(turbomem->bbt));
	return ret;
}

static int turbomem_mtd_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	struct turbomem_info *turbomem = mtd->priv;

	return turbomem_isbad(turbomem->bbt, turbomem_get_eraseblock(ofs));
}

static int turbomem_mtd_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct turbomem_info *turbomem = mtd->priv;

	turbomem_markbad(turbomem->bbt, turbomem_get_eraseblock(ofs));
	turbomem->bbt->version++;
	return turbomem_save_bbt(turbomem);
}

static int turbomem_setup_mtd(struct turbomem_info *turbomem)
{
	struct mtd_info *mtd = &turbomem->mtd;

	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->size = turbomem->usable_flash_sectors * NAND_SECTOR_SIZE;
	mtd->erasesize = NAND_BLOCK_SIZE;
	mtd->writesize = NAND_PAGE_SIZE;
	mtd->writebufsize = NAND_PAGE_SIZE;

	mtd->_erase = turbomem_mtd_erase;
	mtd->_read = turbomem_mtd_read;
	mtd->_write = turbomem_mtd_write;

	mtd->_block_isbad = turbomem_mtd_block_isbad;
	mtd->_block_markbad = turbomem_mtd_block_markbad;

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

	turbomem->mem = pci_iomap(dev, 0, pci_resource_len(dev, 0));
	if (!turbomem->mem) {
		dev_err(&dev->dev, "Unable to remap BAR0\n");
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

	turbomem_calc_sectors(turbomem);

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

	mutex_init(&turbomem->lock);

	snprintf(turbomem->name, NAME_SIZE - 1, "TurboMemory@%s",
		dev_name(turbomem->dev));

	dev_info(&dev->dev, "Device characteristics: %05X, flash size: %d MB\n",
		turbomem->characteristics, turbomem->flash_sectors >> 11);

	if (turbomem->flash_sectors >> 11 <= 1024) {
		dev_warn(turbomem->dev, "1GB board not supported yet\n"
					"Skipping further initialization to "
					"avoid bricking the card.\n");
		return 0;
	}

	ret = turbomem_init_bbt(turbomem);
	if (ret) {
		dev_err(&dev->dev, "Unable to initialize bad blocks table\n");
		goto fail_have_idle_transfer;
	}

	ret = turbomem_setup_mtd(turbomem);
	if (ret) {
		dev_err(&dev->dev, "Unable to register to MTD layer\n");
		goto fail_have_bbt;
	}

	turbomem_debugfs_dev_add(turbomem);

	DBG("Loaded turbomem driver with debug enabled\n");

	return 0;

fail_have_bbt:
	kfree(turbomem->bbt);
fail_have_idle_transfer:
	if (turbomem->idle_transfer)
		turbomem_transferbuf_free(turbomem, turbomem->idle_transfer);
fail_have_dmapool:
	dma_pool_destroy(turbomem->dmapool_cmd);
fail_have_irq:
	free_irq(dev->irq, turbomem);
fail_have_iomap:
	pci_iounmap(dev, turbomem->mem);
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
	kfree(turbomem->bbt);
	if (turbomem->idle_transfer)
		turbomem_transferbuf_free(turbomem, turbomem->idle_transfer);
	dma_pool_destroy(turbomem->dmapool_cmd);
	free_irq(dev->irq, turbomem);
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

#ifdef CONFIG_PM
static void turbomem_pm_complete(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct turbomem_info *turbomem = pci_get_drvdata(pdev);
	int result;

	result = turbomem_hw_init(turbomem);

	if (result)
		dev_warn(dev, "Failed to reset board on resume, result = %d\n",
			result);
}

static const struct dev_pm_ops turbomem_pm_ops = {
	.complete = turbomem_pm_complete,
};
#define TURBOMEM_PM_OPS (&turbomem_pm_ops)
#else
#define TURBOMEM_PM_OPS (NULL)
#endif

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
	.driver.pm = TURBOMEM_PM_OPS,
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
MODULE_DESCRIPTION("Intel(R) Turbo Memory Controller driver");
MODULE_AUTHOR("Erik Ekman <erik@kryo.se>");

module_init(turbomem_init);
module_exit(turbomem_exit);
