#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>

#define PCI_DEVICE_ID_INTEL_TURBOMEMORY (0x444e)

#define DRIVER_NAME "turbomem"

static struct pci_device_id turbomem_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_TURBOMEMORY), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, turbomem_ids);

static u8 turbomem_get_revision(struct pci_dev *dev)
{
	u8 revision;

	pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
	return revision;
}

static int do_reset(unsigned io_base)
{
	unsigned regs[25];
	unsigned reg, i;
	printk(KERN_INFO "Start reset, IO addr %04X\n", io_base);
	for (i = 0; i <= 24; i+= 8) {
		reg = inl(io_base + i);
		printk(KERN_INFO "IO read  %2d: %08X\n", i, reg);
	}
	for (i = 0; i <= 24; i+= 8) {
		reg = 0;
		if (i == 24) reg = 0x1F;
		outl(reg, io_base + i);
		printk(KERN_INFO "IO write %2d: %08X\n", i, reg);
	}
	for (i = 0; i <= 24; i+= 8) {
		reg = inl(io_base + i);
		regs[i] = reg;
		printk(KERN_INFO "IO read  %2d: %08X\n", i, reg);
	}
	reg = 0x100;
	i = 16;
	outl(reg, io_base + i);
	printk(KERN_INFO "IO write %2d: %08X\n", i, reg);
	regs[8] |= 1;
	i = 0;
	do {
		if (i) msleep(100);
		printk(KERN_INFO "IO write %2d: %08X\n", 8, regs[8]);
		outl(regs[8], io_base + 8);
		regs[24] = reg = inl(io_base + 16);
		printk(KERN_INFO "IO read  %2d: %08X\n", 16, reg);
		if ((reg & 0x00010000) == 0) break;
		i++;
	} while ( i < 20);
	if (i >= 20) return -EIO;

	regs[8] = (regs[8] & 0xFFFFFFFB) | 1;
	printk(KERN_INFO "IO write %2d: %08X\n", 8, regs[8]);
	outl(regs[8], io_base + 8);
	i = 0;
	do {
		if (i) msleep(100);
		regs[24] = reg = inl(io_base + 16);
		printk(KERN_INFO "IO read  %2d: %08X\n", 16, reg);
		if ((reg & 0x00010000) == 0) break;
		i++;
	} while ( i < 20);
	if (i >= 20) return -EIO;

	return 0;
}

static int turbomem_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret;
	u8 revision;

	ret = pci_enable_device(dev);
	if (ret)
		return ret;

	pci_set_master(dev);

	if (pci_set_dma_mask(dev, DMA_BIT_MASK(64)) &&
	    pci_set_dma_mask(dev, DMA_BIT_MASK(32))) {
		dev_warn(&dev->dev, "No suitable DMA found\n");
		return -ENOMEM;
	}

	ret = pci_request_regions(dev, DRIVER_NAME);
	if (ret) {
		dev_err(&dev->dev, "Unable to request memory region\n");
		goto fail_req_region;
	}

	ret = do_reset(pci_resource_start(dev, 2));
	if (ret) {
		dev_err(&dev->dev, "Unable to initialize device\n");
		goto fail_reset;
	}

	revision = turbomem_get_revision(dev);
	dev_info(&dev->dev, "Found Intel Turbo Memory Controller (rev %02X)\n", revision);

	return 0;

fail_reset:
	pci_release_regions(dev);
fail_req_region:
	pci_disable_device(dev);
	return ret;
}

static void turbomem_remove(struct pci_dev *dev)
{
	pci_release_regions(dev);
	pci_disable_device(dev);
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
