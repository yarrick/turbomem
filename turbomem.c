#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#define PCI_DEVICE_ID_INTEL_TURBOMEMORY (0x444e)

#define DRIVER_NAME "turbomem"

static struct pci_device_id ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_TURBOMEMORY), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

static u8 turbomem_get_revision(struct pci_dev *dev)
{
	u8 revision;

	pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
	return revision;
}

static int probe(struct pci_dev *dev, const struct pci_device_id *id)
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
		return ret;
	}

	revision = turbomem_get_revision(dev);
	dev_info(&dev->dev, "Found Intel Turbo Memory Controller (rev %02X)\n", revision);

	return 0;
}

static void remove(struct pci_dev *dev)
{
	pci_release_regions(dev);
}

static struct pci_driver pci_driver = {
	.name = DRIVER_NAME,
	.id_table = ids,
	.probe = probe,
	.remove = remove,
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
