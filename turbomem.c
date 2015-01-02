#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>

#define DRIVER_NAME "turbomem"

#define STATUS_REGISTER (0x18)
#define STATUS_INTERRUPT_MASK (0x1F)
#define STATUS_DMA_ERROR (0x100)

#define INTERRUPT_CTRL_REGISTER (0x20)
#define INTERRUPT_CTRL_ENABLE_BITS (0x3)

struct dma_buf {
	void *buf;
	dma_addr_t busaddr;
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
	void __iomem *mem;
	struct dma_pool *dmapool_cmd;
	struct dma_pool *dmapool_data;
	struct dma_buf idle_transfer;
	struct tasklet_struct tasklet;
	unsigned characteristics;
	unsigned flash_sectors;
};

static int major_nr;

static void turbomem_enable_interrupts(struct turbomem_info *turbomem, bool active)
{
	u32 reg;

	reg = le32_to_cpu(ioread32(turbomem->mem + INTERRUPT_CTRL_REGISTER));
	if (active)
		reg |= INTERRUPT_CTRL_ENABLE_BITS;
	else
		reg &= ~INTERRUPT_CTRL_ENABLE_BITS;

	iowrite32(cpu_to_le32(reg), turbomem->mem + INTERRUPT_CTRL_REGISTER);
}

static void turbomem_tasklet(unsigned long privdata)
{
	struct turbomem_info *turbomem = (struct turbomem_info *) privdata;

	turbomem_enable_interrupts(turbomem, 1);
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

	turbomem_enable_interrupts(turbomem, 0);

	reg = le32_to_cpu(ioread32(turbomem->mem + STATUS_REGISTER));
	iowrite32(cpu_to_le32(reg & STATUS_INTERRUPT_MASK), turbomem->mem + STATUS_REGISTER);

	tasklet_schedule(&turbomem->tasklet);

	return IRQ_HANDLED;
}

static void turbomem_set_idle_transfer(struct turbomem_info *turbomem)
{
	dma_addr_t busaddr;
	struct transfer_command *idle_cmd;

	idle_cmd = dma_pool_alloc(turbomem->dmapool_cmd, GFP_KERNEL, &busaddr);
	if (!idle_cmd) {
		dev_err(turbomem->dev, "Failed to alloc dma buf");
		return;
	}

	memset(idle_cmd, 0, sizeof(struct transfer_command));

	idle_cmd->transfer_flags = cpu_to_le32(0x7FFFFFFE);
	idle_cmd->mode = 0x35; /* NOP command */
	idle_cmd->last_transfer = 1;
	idle_cmd->cmd_one = 0;

	iowrite32(cpu_to_le32(busaddr & 0xFFFFFFFF), turbomem->mem);

	turbomem_enable_interrupts(turbomem, 1);

	turbomem->idle_transfer.buf = idle_cmd;
	turbomem->idle_transfer.busaddr = busaddr;
}

#define HW_RESET_ATTEMPTS 50

static int turbomem_hw_init(struct turbomem_info *turbomem)
{
	u32 regs[4];
	u32 reg;
	unsigned i;
	for (i = 0; i < 4; i++) {
		regs[i] = le32_to_cpu(ioread32(turbomem->mem + i*8));
	}
	reg = regs[0] | regs[1] | regs[2] | regs[3];
	if (reg) {
		for (i = 0; i < 4; i++) {
			reg = 0;
			if (i == 3) reg = 0x1F;
			iowrite32(cpu_to_le32(reg), turbomem->mem + i*8);
		}
		for (i = 0; i < 4; i++) {
			regs[i] = le32_to_cpu(ioread32(turbomem->mem + i*8));
		}
		reg = regs[0] | regs[1] | regs[2] | regs[3];
		if (reg) {
			reg = 0x100;
			i = 16;
			iowrite32(cpu_to_le32(reg), turbomem->mem + i);
			regs[1] |= 1;
			for (i = 0; i < HW_RESET_ATTEMPTS; i++) {
				if (i) msleep(100);
				iowrite32(cpu_to_le32(regs[1]), turbomem->mem + 8);
				regs[3] = reg = le32_to_cpu(ioread32(turbomem->mem + 24));
				if ((reg & 0x00010000) == 0) break;
			}
			if (i >= HW_RESET_ATTEMPTS)
				return -EIO;
		}
	}

	regs[1] = (regs[1] & 0xFFFFFFFB) | 1;
	iowrite32(cpu_to_le32(regs[1]), turbomem->mem + 8);
	for (i = 0; i < HW_RESET_ATTEMPTS; i++) {
		if (i) msleep(100);
		regs[3] = reg = le32_to_cpu(ioread32(turbomem->mem + 24));
		if ((reg & 0x00010000) == 0) break;
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

	turbomem_set_idle_transfer(turbomem);
	if (!turbomem->idle_transfer.buf) {
		dev_err(&dev->dev, "Unable to allocate idle transfer job\n");
		ret = -ENOMEM;
		goto fail_dmapool_data;
	}

	dev_info(&dev->dev, "Device characteristics: %05X, flash size: %d MB\n",
		turbomem->characteristics, turbomem->flash_sectors >> 8);

	pci_set_drvdata(dev, turbomem);

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

	dma_pool_free(turbomem->dmapool_cmd, turbomem->idle_transfer.buf,
		turbomem->idle_transfer.busaddr);
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
	unregister_blkdev(major_nr, DRIVER_NAME);
	pci_unregister_driver(&pci_driver);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel(R) Turbo Memory PCIe flash block device driver");
MODULE_AUTHOR("Erik Ekman <erik@kryo.se>");

module_init(turbomem_init);
module_exit(turbomem_exit);
