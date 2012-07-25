/*
 * SAMSUNG EXYNOS USB HOST OHCI Controller
 *
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Author: Jingoo Han <jg1.han@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <plat/ohci.h>
#include <plat/usb-phy.h>

#define INSNREG00 0xec200090
#define APP_START_CLK (1<<15)
static struct s5p_ohci_platdata* ohci_pdata;
struct exynos_ohci_hcd {
	struct device *dev;
	struct usb_hcd *hcd;
	struct clk *clk;
};

static void ohci_exynos_set_clock(int start_clk)
{
	void __iomem* insnreg;
        insnreg = ioremap(INSNREG00,4);

	if(start_clk) {
		__raw_writel(APP_START_CLK,insnreg);
	}
	else {
		__raw_writel(~APP_START_CLK,insnreg);
	}		
}
#ifdef CONFIG_PM
static int exynos_ohci_bus_suspend(struct usb_hcd *hcd)
{
	int ret;
	ohci_exynos_set_clock(1);
	ret = ohci_bus_suspend(hcd);
	ohci_exynos_set_clock(0);
	return ret;
}
static int exynos_ohci_bus_resume(struct usb_hcd *hcd)
{
	int ret;
	ohci_exynos_set_clock(1);
	ret = ohci_bus_resume(hcd);
	ohci_exynos_set_clock(0);
	return ret;
}
#endif
static int ohci_exynos_start(struct usb_hcd *hcd)
{
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);
	int ret;

	ohci_dbg(ohci, "ohci_exynos_start, ohci:%p", ohci);

	ret = ohci_init(ohci);
	if (ret < 0)
		return ret;

	ret = ohci_run(ohci);
	if (ret < 0) {
		err("can't start %s", hcd->self.bus_name);
		ohci_stop(hcd);
		return ret;
	}

	return 0;
}

/* ohci_s5p_hub_control
 *
 * look at control requests to the hub, and see if we need
 * to take any action or over-ride the results from the
 * request.
*/

static int ohci_exynos_hub_control(
        struct usb_hcd  *hcd,
        u16             typeReq,
        u16             wValue,
        u16             wIndex,
        char            *buf,
        u16             wLength)
{
        struct ohci_hcd *ohci = hcd_to_ohci(hcd);
        int ret = -EINVAL;
        int temp;

        /* if we are only an humble host without any special capabilities
          * process the request straight away and exit */
        ret = ohci_hub_control(hcd, typeReq, wValue,
                                       wIndex, buf, wLength);

        /* check the request to see if it needs handling */
        switch (typeReq) {
        case GetPortStatus:
                /* check port status */
                if (wIndex == 1) {
                        temp = roothub_portstatus(hcd_to_ohci(hcd), wIndex - 1);
                        if((temp & RH_PS_LSDA))
                                ohci_pdata->port_status(1);
                        else
                                ohci_pdata->port_status(0);
                }
        }

        return ret;
}

static const struct hc_driver exynos_ohci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "EXYNOS OHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ohci_hcd),

	.irq			= ohci_irq,
	.flags			= HCD_MEMORY|HCD_USB11,

	.start			= ohci_exynos_start,
	.stop			= ohci_stop,
	.shutdown		= ohci_shutdown,

	.get_frame_number	= ohci_get_frame,

	.urb_enqueue		= ohci_urb_enqueue,
	.urb_dequeue		= ohci_urb_dequeue,
	.endpoint_disable	= ohci_endpoint_disable,

	.hub_status_data	= ohci_hub_status_data,
	.hub_control		= ohci_exynos_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend		= exynos_ohci_bus_suspend,
	.bus_resume		= exynos_ohci_bus_resume,
#endif
	.start_port_reset	= ohci_start_port_reset,
};

static int __devinit exynos_ohci_probe(struct platform_device *pdev)
{
	struct s5p_ohci_platdata *pdata;
	struct exynos_ohci_hcd *exynos_ohci;
	struct usb_hcd *hcd;
	struct ohci_hcd *ohci;
	struct resource *res;
	int irq;
	int err;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data defined\n");
		return -EINVAL;
	}
	ohci_pdata = pdata;
	exynos_ohci = kzalloc(sizeof(struct exynos_ohci_hcd), GFP_KERNEL);
	if (!exynos_ohci)
		return -ENOMEM;

	exynos_ohci->dev = &pdev->dev;

	hcd = usb_create_hcd(&exynos_ohci_hc_driver, &pdev->dev,
					dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		err = -ENOMEM;
		goto fail_hcd;
	}

	exynos_ohci->hcd = hcd;
	exynos_ohci->clk = clk_get(&pdev->dev, "usbhost");

	if (IS_ERR(exynos_ohci->clk)) {
		dev_err(&pdev->dev, "Failed to get usbhost clock\n");
		err = PTR_ERR(exynos_ohci->clk);
		goto fail_clk;
	}

	err = clk_enable(exynos_ohci->clk);
	if (err)
		goto fail_clken;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get I/O memory\n");
		err = -ENXIO;
		goto fail_io;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = ioremap(res->start, resource_size(res));
	if (!hcd->regs) {
		dev_err(&pdev->dev, "Failed to remap I/O memory\n");
		err = -ENOMEM;
		goto fail_io;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		err = -ENODEV;
		goto fail;
	}

	if (pdata->phy_init)
		pdata->phy_init(pdev, S5P_USB_PHY_HOST);

	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	err = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (err) {
		dev_err(&pdev->dev, "Failed to add USB HCD\n");
		goto fail;
	}

	platform_set_drvdata(pdev, exynos_ohci);

	return 0;

fail:
	iounmap(hcd->regs);
fail_io:
	clk_disable(exynos_ohci->clk);
fail_clken:
	clk_put(exynos_ohci->clk);
fail_clk:
	usb_put_hcd(hcd);
fail_hcd:
	kfree(exynos_ohci);
	return err;
}

static int __devexit exynos_ohci_remove(struct platform_device *pdev)
{
	struct s5p_ohci_platdata *pdata = pdev->dev.platform_data;
	struct exynos_ohci_hcd *exynos_ohci = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = exynos_ohci->hcd;

	usb_remove_hcd(hcd);

	if (pdata && pdata->phy_exit)
		pdata->phy_exit(pdev, S5P_USB_PHY_HOST);

	iounmap(hcd->regs);

	clk_disable(exynos_ohci->clk);
	clk_put(exynos_ohci->clk);

	usb_put_hcd(hcd);
	kfree(exynos_ohci);

	return 0;
}

static void exynos_ohci_shutdown(struct platform_device *pdev)
{
	struct exynos_ohci_hcd *exynos_ohci = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = exynos_ohci->hcd;

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

#ifdef CONFIG_PM
static int exynos_ohci_suspend(struct device *dev)
{
	struct exynos_ohci_hcd *exynos_ohci = dev_get_drvdata(dev);
	struct usb_hcd *hcd = exynos_ohci->hcd;
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);
	struct platform_device *pdev = to_platform_device(dev);
	struct s5p_ohci_platdata *pdata = pdev->dev.platform_data;
	unsigned long flags;
	int rc = 0;

	/*
	 * Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave(&ohci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED && hcd->state != HC_STATE_HALT) {
		rc = -EINVAL;
		goto fail;
	}

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

fail:
	spin_unlock_irqrestore(&ohci->lock, flags);
	if (pdata && pdata->phy_exit)
		pdata->phy_exit(pdev, S5P_USB_PHY_HOST);

	return rc;
}

static int exynos_ohci_resume(struct device *dev)
{
	struct exynos_ohci_hcd *exynos_ohci = dev_get_drvdata(dev);
	struct usb_hcd *hcd = exynos_ohci->hcd;
	struct platform_device *pdev = to_platform_device(dev);
	struct s5p_ohci_platdata *pdata = pdev->dev.platform_data;

	if (pdata && pdata->phy_init)
		pdata->phy_init(pdev, S5P_USB_PHY_HOST);

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	ohci_finish_controller_resume(hcd);

	return 0;
}
#else
#define exynos_ohci_suspend	NULL
#define exynos_ohci_resume	NULL
#endif

static const struct dev_pm_ops exynos_ohci_pm_ops = {
	.suspend	= exynos_ohci_suspend,
	.resume		= exynos_ohci_resume,
};

static struct platform_driver exynos_ohci_driver = {
	.probe		= exynos_ohci_probe,
	.remove		= __devexit_p(exynos_ohci_remove),
	.shutdown	= exynos_ohci_shutdown,
	.driver = {
		.name	= "s5p-ohci",
		.owner	= THIS_MODULE,
		.pm	= &exynos_ohci_pm_ops,
	}
};

MODULE_ALIAS("platform:exynos-ohci");
MODULE_AUTHOR("Jingoo Han <jg1.han@samsung.com>");
