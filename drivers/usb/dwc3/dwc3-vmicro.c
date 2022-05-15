#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/extcon.h>
#include <linux/usb/typec.h>

struct extcon_nb {
	struct extcon_dev *edev;
	struct dwc3_vmicro *dwc3_vmicro;
	int idx;
	struct notifier_block vbus_nb;
};

struct dwc3_vmicro {
	struct device *dev;
	struct typec_capability	typec_caps;
	struct typec_port *typec_port;
	struct typec_partner *partner;
	struct typec_partner_desc partner_desc;
	struct extcon_nb *extcon;
	int extcon_cnt;
};

static int dwc3_vmicro_parse_dts(struct platform_device *pdev,
				 struct dwc3_vmicro *dwc3_vmicro)
{
	return 0;
}

static int dwc3_vmicro_vbus_notifier(struct notifier_block *nb,
				     unsigned long event, void *ptr)
{
	struct extcon_dev *edev = ptr;
	struct extcon_nb *enb = container_of(nb, struct extcon_nb, vbus_nb);
	struct dwc3_vmicro *dwc3_vmicro = enb->dwc3_vmicro;
	const char *edev_name;

	if (!edev || !dwc3_vmicro)
		return NOTIFY_DONE;

	edev_name = extcon_get_edev_name(edev);
	pr_info("%s: %d: idx=%d event=%d received\n", __func__, __LINE__, enb->idx, event);

	if (event) {
		if (!dwc3_vmicro->partner) {
			dwc3_vmicro->partner = typec_register_partner(dwc3_vmicro->typec_port,
								      &dwc3_vmicro->partner_desc);
			pr_info("%s: %d: usb gadget connect\n", __func__, __LINE__);
		}
	} else {
		if (dwc3_vmicro->partner) {
			typec_unregister_partner(dwc3_vmicro->partner);
			dwc3_vmicro->partner = NULL;
			pr_info("%s: %d: usb gadget disconnect\n", __func__, __LINE__);
		}
	}

	return NOTIFY_DONE;
}

static int dwc3_vmicro_extcon_register(struct dwc3_vmicro *dwc3_vmicro)
{
	struct device_node *np = dwc3_vmicro->dev->of_node;
	struct extcon_dev *edev;
	int idx, ret = 0;
	bool check_vbus_state, phandle_found = false;

	dwc3_vmicro->extcon_cnt = of_count_phandle_with_args(np, "extcon", NULL);
	if (dwc3_vmicro->extcon_cnt < 0) {
		pr_err("%s: %d: of_count_phandle_with_args failed\n", __func__, __LINE__);
		return -ENODEV;
	}

	dwc3_vmicro->extcon = devm_kcalloc(dwc3_vmicro->dev, dwc3_vmicro->extcon_cnt,
					   sizeof(*dwc3_vmicro->extcon), GFP_KERNEL);
	if (!dwc3_vmicro->extcon)
		return -ENOMEM;
	memset(dwc3_vmicro->extcon, 0, sizeof(*dwc3_vmicro->extcon) * dwc3_vmicro->extcon_cnt);

	for (idx = 0; idx < dwc3_vmicro->extcon_cnt; idx++) {
		edev = extcon_get_edev_by_phandle(dwc3_vmicro->dev, idx);
		if (IS_ERR(edev) && PTR_ERR(edev) != -ENODEV)
			return PTR_ERR(edev);

		if (IS_ERR_OR_NULL(edev))
			continue;

		check_vbus_state = true;
		phandle_found = true;

		dwc3_vmicro->extcon[idx].dwc3_vmicro = dwc3_vmicro;
		dwc3_vmicro->extcon[idx].edev = edev;
		dwc3_vmicro->extcon[idx].idx = idx;

		dwc3_vmicro->extcon[idx].vbus_nb.notifier_call =
			dwc3_vmicro_vbus_notifier;
		ret = extcon_register_notifier(edev, EXTCON_USB,
					       &dwc3_vmicro->extcon[idx].vbus_nb);
		if (ret < 0)
			check_vbus_state = false;

		if (check_vbus_state && extcon_get_state(edev, EXTCON_USB))
			dwc3_vmicro_vbus_notifier(&dwc3_vmicro->extcon[idx].vbus_nb,
						  true, edev);
	}

	if (!phandle_found) {
		dev_err(dwc3_vmicro->dev, "no extcon device found\n");
		return -ENODEV;
	}

	return 0;
}

static int dwc3_vmicro_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dwc3_vmicro *dwc3_vmicro;
	int ret = 0;

	if (!np)
		return -EINVAL;

	dwc3_vmicro = devm_kzalloc(dev, sizeof(*dwc3_vmicro), GFP_KERNEL);
	if (!dwc3_vmicro)
		return -ENOMEM;
	dwc3_vmicro->dev = dev;

	ret = dwc3_vmicro_parse_dts(pdev, dwc3_vmicro);
	if (ret) {
		pr_err("%s: %d: fail to parse dwc3_vmicro dts: %d\n",
		       __func__, __LINE__, ret);
		goto err;
	}

	dwc3_vmicro->typec_caps.type = TYPEC_PORT_SNK;
	dwc3_vmicro->typec_caps.data = TYPEC_PORT_UFP;
	dwc3_vmicro->typec_caps.revision = 0x0130;
	dwc3_vmicro->partner_desc.usb_pd = false;
	dwc3_vmicro->partner_desc.accessory = TYPEC_ACCESSORY_NONE;

	dwc3_vmicro->typec_port = typec_register_port(dwc3_vmicro->dev, &dwc3_vmicro->typec_caps);
	if (IS_ERR(dwc3_vmicro->typec_port)) {
		ret = PTR_ERR(dwc3_vmicro->typec_port);
		pr_err("%s: %d: failed to register typec_port: %d\n", __func__, __LINE__, ret);
		goto err;
	}

	if (of_property_read_bool(np, "extcon")) {
		ret = dwc3_vmicro_extcon_register(dwc3_vmicro);
		if (ret) {
			pr_err("%s: %d: fail to register extcon: %d\n",
			       __func__, __LINE__, ret);
			goto err1;
		}
	} else {
		pr_err("%s: %d: no extcon node\n", __func__, __LINE__);
		goto err1;
	}

	platform_set_drvdata(pdev, dwc3_vmicro);
	pr_info("%s: %d: finish: %d\n", __func__, __LINE__, ret);

	return 0;
err1:
	typec_unregister_port(dwc3_vmicro->typec_port);
err:
	return ret;
}

static int dwc3_vmicro_remove(struct platform_device *pdev)
{
	struct dwc3_vmicro *dwc3_vmicro = platform_get_drvdata(pdev);
	int idx;

	for (idx = 0; idx < dwc3_vmicro->extcon_cnt; idx++) {
		if (dwc3_vmicro->extcon[idx].edev)
			extcon_unregister_notifier(dwc3_vmicro->extcon[idx].edev, EXTCON_USB,
						   &dwc3_vmicro->extcon[idx].vbus_nb);
	}
	if (dwc3_vmicro->partner) {
		typec_unregister_partner(dwc3_vmicro->partner);
		dwc3_vmicro->partner = NULL;
	};
	typec_unregister_port(dwc3_vmicro->typec_port);

	return 0;
}

static const struct of_device_id dwc3_vmicro_dt_ids[] = {
	{ .compatible = "v,dwc3-vmicro", },
	{ },
};
MODULE_DEVICE_TABLE(of, dwc3_vmicro_dt_ids);

static struct platform_driver dwc3_vmicro_driver = {
	.probe		= dwc3_vmicro_probe,
	.remove		= dwc3_vmicro_remove,
	.driver		= {
		.name	= "dwc3_vmicro",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(dwc3_vmicro_dt_ids),
	},
};

static int __init dwc3_vmicro_init(void)
{
	return platform_driver_register(&dwc3_vmicro_driver);
}

static void __init dwc3_vmicro_exit(void)
{
	platform_driver_unregister(&dwc3_vmicro_driver);
}

module_init(dwc3_vmicro_init);
module_exit(dwc3_vmicro_exit);

MODULE_DESCRIPTION("DWC3 VMICRO");
MODULE_LICENSE("GPL");
