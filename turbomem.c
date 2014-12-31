#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#define PCI_DEVICE_ID_INTEL_TURBOMEMORY (0x444e)

#define DRIVER_NAME "turbomem"

#define STATUS_REGISTER (0x18)
#define STATUS_INTERRUPT_MASK (0x1F)
#define STATUS_DMA_ERROR (0x100)

#define INTERRUPT_CTRL_REGISTER (0x20)
#define INTERRUPT_CTRL_DISABLE_MASK (0xFFFFFFFC)
#define INTERRUPT_CTRL_ENABLE (0x3)

static const struct pci_device_id turbomem_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_TURBOMEMORY), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, turbomem_ids);

struct dma_buf {
	void *buf;
	dma_addr_t busaddr;
};

struct turbomem_info {
	struct device *dev;
	void __iomem *mem;
	struct dma_pool *dmapool;
	struct dma_buf idle_transfer;
	unsigned characteristics;
	unsigned flash_sectors;
};

static void turbomem_enable_interrupts(struct turbomem_info *turbomem, bool active)
{
	u32 reg;

	reg = ioread32(turbomem->mem + INTERRUPT_CTRL_REGISTER);
	if (active)
		reg |= INTERRUPT_CTRL_ENABLE;
	else
		reg &= INTERRUPT_CTRL_DISABLE_MASK;

	iowrite32(reg, turbomem->mem + INTERRUPT_CTRL_REGISTER);
}

static unsigned turbomem_calc_sectors(unsigned reg0x38, unsigned chars)
{
	int limit8, limit14;
	int d = (reg0x38 >> 0xC) & 0xF;
	int c = 1;
	int i = 0;

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
	unsigned status;
	unsigned reg;

	status = ioread32(turbomem->mem + STATUS_REGISTER);
	if (status == 0xFFFFFFFF || (status & STATUS_INTERRUPT_MASK) == 0)
		return IRQ_NONE;

	dev_info(turbomem->dev, "Got IRQ on line %d, status is %08X\n",
		irq, status);

	turbomem_enable_interrupts(turbomem, 0);

	reg = ioread32(turbomem->mem + STATUS_REGISTER);
	iowrite32(reg & STATUS_INTERRUPT_MASK, turbomem->mem + STATUS_REGISTER);

	return IRQ_HANDLED;
}

static void turbomem_set_idle_transfer(struct turbomem_info *turbomem)
{
	dma_addr_t busaddr;
	void *buf;
	u8 *u8buf;
	u32 *u32buf;

	buf = dma_pool_alloc(turbomem->dmapool, GFP_KERNEL, &busaddr);
	if (!buf) {
		dev_err(turbomem->dev, "Failed to alloc dma buf");
		return;
	}

	memset(buf, 0, 128);
	u8buf = (u8 *) buf;
	u32buf = (u32 *) buf;

	u32buf[1] = 0x7FFFFFFE;
	u8buf[0x10] = 0x35;
	u8buf[0x7c] = 0;
	u8buf[0x35] = 1;

	iowrite32(busaddr, turbomem->mem);

	turbomem_enable_interrupts(turbomem, 1);

	turbomem->idle_transfer.buf = buf;
	turbomem->idle_transfer.busaddr = busaddr;
}

#define HW_RESET_ATTEMPTS 50

static int turbomem_hw_init(struct turbomem_info *turbomem)
{
	unsigned regs[4];
	unsigned reg, i;
	for (i = 0; i < 4; i++) {
		regs[i] = ioread32(turbomem->mem + i*8);
	}
	reg = regs[0] | regs[1] | regs[2] | regs[3];
	if (reg) {
		for (i = 0; i < 4; i++) {
			reg = 0;
			if (i == 3) reg = 0x1F;
			iowrite32(reg, turbomem->mem + i*8);
		}
		for (i = 0; i < 4; i++) {
			regs[i] = ioread32(turbomem->mem + i*8);
		}
		reg = regs[0] | regs[1] | regs[2] | regs[3];
		if (reg) {
			reg = 0x100;
			i = 16;
			iowrite32(reg, turbomem->mem + i);
			regs[1] |= 1;
			for (i = 0; i < HW_RESET_ATTEMPTS; i++) {
				if (i) msleep(100);
				iowrite32(regs[1], turbomem->mem + 8);
				regs[3] = reg = ioread32(turbomem->mem + 24);
				if ((reg & 0x00010000) == 0) break;
			}
			if (i >= HW_RESET_ATTEMPTS)
				return -EIO;
		}
	}

	regs[1] = (regs[1] & 0xFFFFFFFB) | 1;
	iowrite32(regs[1], turbomem->mem + 8);
	for (i = 0; i < HW_RESET_ATTEMPTS; i++) {
		if (i) msleep(100);
		regs[3] = reg = ioread32(turbomem->mem + 24);
		if ((reg & 0x00010000) == 0) break;
	}
	if (i >= HW_RESET_ATTEMPTS)
		return -EIO;

	/* Get device characteristics */
	reg = ioread32(turbomem->mem + 0x38);
	turbomem->characteristics =
		(((reg& 0xFFFF0000) + 0x10000) & 0xF0000) | (reg & 0xFFFF);

	turbomem->flash_sectors = turbomem_calc_sectors(reg, turbomem->characteristics);

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

	ret = request_irq(dev->irq, turbomem_isr, IRQF_SHARED,
			DRIVER_NAME, turbomem);
	if (ret) {
		dev_err(&dev->dev, "Unable to request IRQ\n");
		goto fail_init;
	}

	turbomem->dmapool = dma_pool_create(DRIVER_NAME "_128", &dev->dev,
		128, 8, 0);
	if (!turbomem->dmapool) {
		dev_err(&dev->dev, "Unable to create DMA pool\n");
		ret = -ENOMEM;
		goto fail_irq;
	}

	turbomem_set_idle_transfer(turbomem);
	if (!turbomem->idle_transfer.buf) {
		dev_err(&dev->dev, "Unable to allocate idle transfer command\n");
		ret = -ENOMEM;
		goto fail_dmapool;
	}

	dev_info(&dev->dev, "Device characteristics: %05X, flash size: %d MB\n",
		turbomem->characteristics, turbomem->flash_sectors >> 8);

	pci_set_drvdata(dev, turbomem);

	return 0;

fail_dmapool:
	dma_pool_destroy(turbomem->dmapool);
fail_irq:
	free_irq(dev->irq, turbomem);
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

	dma_pool_free(turbomem->dmapool, turbomem->idle_transfer.buf, turbomem->idle_transfer.busaddr);
	dma_pool_destroy(turbomem->dmapool);
	free_irq(dev->irq, turbomem);
	iounmap(turbomem->mem);
	pci_release_regions(dev);
	pci_disable_device(dev);
	pci_set_drvdata(dev, NULL);
	kfree(turbomem);
}

static struct pci_driver pci_driver = {
	.name = DRIVER_NAME,
	.id_table = turbomem_ids,
	.probe = turbomem_probe,
	.remove = turbomem_remove,
};

static int __init turbomem_init(void)
{
	return pci_register_driver(&pci_driver);
}

static void __exit turbomem_exit(void)
{
	pci_unregister_driver(&pci_driver);
}

MODULE_LICENSE("GPL");

module_init(turbomem_init);
module_exit(turbomem_exit);
