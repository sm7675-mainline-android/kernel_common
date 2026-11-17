// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Linaro Ltd
 */
#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>

#define PMIC_GLINK_SEND_TIMEOUT (5 * HZ)
#define PMIC_GLINK_PRIMARY_CHANNEL "PMIC_RTR_ADSP_APPS"
#define PMIC_GLINK_LOG_CHANNEL "PMIC_LOGS_ADSP_APPS"

enum {
	PMIC_GLINK_CLIENT_BATT = 0,
	PMIC_GLINK_CLIENT_ALTMODE,
	PMIC_GLINK_CLIENT_UCSI,
};

struct pmic_glink {
	struct device *dev;
	struct pdr_handle *pdr;
	struct list_head node;
	const char *channel_name;

	struct rpmsg_endpoint *ept;

	unsigned long client_mask;

	struct auxiliary_device altmode_aux;
	struct auxiliary_device ps_aux;
	struct auxiliary_device ucsi_aux;
	struct platform_device *battery_pdev;

	/* serializing client_state and pdr_state updates */
	struct mutex state_lock;
	unsigned int client_state;
	unsigned int pdr_state;

	/* serializing clients list updates */
	spinlock_t client_lock;
	struct list_head clients;
};

static LIST_HEAD(pmic_glink_devices);
static DEFINE_MUTEX(__pmic_glink_lock);

struct pmic_glink_client {
	struct list_head node;

	struct pmic_glink *pg;
	unsigned int id;

	void (*cb)(const void *data, size_t len, void *priv);
	void (*pdr_notify)(void *priv, int state);
	void *priv;
};

static void _devm_pmic_glink_release_client(struct device *dev, void *res)
{
	struct pmic_glink_client *client = (struct pmic_glink_client *)res;
	struct pmic_glink *pg = client->pg;
	unsigned long flags;

	spin_lock_irqsave(&pg->client_lock, flags);
	if (!list_empty(&client->node))
		list_del_init(&client->node);
	spin_unlock_irqrestore(&pg->client_lock, flags);
}

static void _pmic_glink_client_unregister(struct pmic_glink_client *client)
{
	struct pmic_glink *pg = client->pg;
	unsigned long flags;

	spin_lock_irqsave(&pg->client_lock, flags);
	if (!list_empty(&client->node))
		list_del_init(&client->node);
	spin_unlock_irqrestore(&pg->client_lock, flags);
}

void pmic_glink_client_unregister(struct pmic_glink_client *client)
{
	if (!client)
		return;

	_pmic_glink_client_unregister(client);
}
EXPORT_SYMBOL_GPL(pmic_glink_client_unregister);

struct pmic_glink_client *devm_pmic_glink_client_alloc(struct device *dev,
						       unsigned int id,
						       void (*cb)(const void *, size_t, void *),
						       void (*pdr)(void *, int),
						       void *priv)
{
	struct pmic_glink_client *client;
	struct pmic_glink *pg = dev_get_drvdata(dev->parent);

	client = devres_alloc(_devm_pmic_glink_release_client, sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->pg = pg;
	client->id = id;
	client->cb = cb;
	client->pdr_notify = pdr;
	client->priv = priv;
	INIT_LIST_HEAD(&client->node);

	devres_add(dev, client);

	return client;
}
EXPORT_SYMBOL_GPL(devm_pmic_glink_client_alloc);

void pmic_glink_client_register(struct pmic_glink_client *client)
{
	struct pmic_glink *pg = client->pg;
	unsigned long flags;

	guard(mutex)(&pg->state_lock);
	spin_lock_irqsave(&pg->client_lock, flags);

	list_add(&client->node, &pg->clients);
	client->pdr_notify(client->priv, pg->client_state);

	spin_unlock_irqrestore(&pg->client_lock, flags);
}
EXPORT_SYMBOL_GPL(pmic_glink_client_register);

int pmic_glink_send(struct pmic_glink_client *client, void *data, size_t len)
{
	struct pmic_glink *pg = client->pg;
	bool timeout_reached = false;
	unsigned long start;
	int ret;

	guard(mutex)(&pg->state_lock);
	if (!pg->ept) {
		return -ECONNRESET;
	}

	start = jiffies;
	for (;;) {
		ret = rpmsg_send(pg->ept, data, len);
		if (ret != -EAGAIN)
			break;

		if (timeout_reached) {
			ret = -ETIMEDOUT;
			break;
		}

		usleep_range(1000, 5000);
		timeout_reached = time_after(jiffies, start + PMIC_GLINK_SEND_TIMEOUT);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(pmic_glink_send);

static int pmic_glink_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
				     int len, void *priv, u32 addr)
{
	struct pmic_glink_client *client;
	struct pmic_glink_hdr *hdr;
	struct pmic_glink *pg = dev_get_drvdata(&rpdev->dev);
	unsigned long flags;

	if (!pg)
		return 0;

	if (len < sizeof(*hdr)) {
		dev_warn(pg->dev, "ignoring truncated message\n");
		return 0;
	}

	hdr = data;

	spin_lock_irqsave(&pg->client_lock, flags);
	list_for_each_entry(client, &pg->clients, node) {
		if (client->id == le32_to_cpu(hdr->owner))
			client->cb(data, len, client->priv);
	}
	spin_unlock_irqrestore(&pg->client_lock, flags);

	return 0;
}

static void pmic_glink_aux_release(struct device *dev)
{
	of_node_put(dev->of_node);
}

static int pmic_glink_add_aux_device(struct pmic_glink *pg,
				     struct auxiliary_device *aux,
				     const char *name)
{
	struct device *parent = pg->dev;
	struct device_node *node = NULL;
	int ret;

	aux->name = name;
	aux->dev.parent = parent;
	aux->dev.release = pmic_glink_aux_release;
	if (!strcmp(name, "ucsi"))
		node = of_get_compatible_child(parent->of_node, "qcom,ucsi-glink");
	aux->dev.of_node = node ? node : of_node_get(parent->of_node);
	aux->dev.of_node_reused = true;
	ret = auxiliary_device_init(aux);
	if (ret) {
		of_node_put(aux->dev.of_node);
		return ret;
	}

	ret = auxiliary_device_add(aux);
	if (ret)
		auxiliary_device_uninit(aux);

	return ret;
}

static void pmic_glink_del_aux_device(struct pmic_glink *pg,
				      struct auxiliary_device *aux)
{
	auxiliary_device_delete(aux);
	auxiliary_device_uninit(aux);
}

static void pmic_glink_state_notify_clients(struct pmic_glink *pg)
{
	struct pmic_glink_client *client;
	unsigned int new_state = pg->client_state;
	unsigned long flags;

	if (pg->client_state != SERVREG_SERVICE_STATE_UP) {
		if (pg->pdr_state == SERVREG_SERVICE_STATE_UP && pg->ept)
			new_state = SERVREG_SERVICE_STATE_UP;
	} else {
		if (pg->pdr_state == SERVREG_SERVICE_STATE_DOWN || !pg->ept)
			new_state = SERVREG_SERVICE_STATE_DOWN;
	}

	if (new_state != pg->client_state) {
		dev_dbg(pg->dev, "USB DIAG glink channel=%s state=%u pdr=%u endpoint=%u\n",
			 pg->channel_name, new_state, pg->pdr_state, !!pg->ept);
		spin_lock_irqsave(&pg->client_lock, flags);
		list_for_each_entry(client, &pg->clients, node)
			client->pdr_notify(client->priv, new_state);
		spin_unlock_irqrestore(&pg->client_lock, flags);
		pg->client_state = new_state;
	}
}

static void pmic_glink_pdr_callback(int state, char *svc_path, void *priv)
{
	struct pmic_glink *pg = priv;

	guard(mutex)(&pg->state_lock);
	pg->pdr_state = state;

	pmic_glink_state_notify_clients(pg);
}

static int pmic_glink_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct pmic_glink *pg, *candidate;

	guard(mutex)(&__pmic_glink_lock);
	pg = NULL;
	list_for_each_entry(candidate, &pmic_glink_devices, node) {
		if (!strcmp(candidate->channel_name, rpdev->id.name)) {
			pg = candidate;
			break;
		}
	}
	if (!pg)
		return dev_err_probe(&rpdev->dev, -EPROBE_DEFER,
				     "no pmic_glink transport for %s\n", rpdev->id.name);

	if (!device_link_add(&rpdev->dev, pg->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -EINVAL;

	dev_set_drvdata(&rpdev->dev, pg);

	guard(mutex)(&pg->state_lock);
	pg->ept = rpdev->ept;
	dev_dbg(pg->dev, "USB DIAG glink endpoint attached channel=%s\n", pg->channel_name);
	pmic_glink_state_notify_clients(pg);

	return 0;
}

static void pmic_glink_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct pmic_glink *pg = dev_get_drvdata(&rpdev->dev);

	if (!pg)
		return;

	guard(mutex)(&pg->state_lock);
	pg->ept = NULL;
	pmic_glink_state_notify_clients(pg);
}

static const struct rpmsg_device_id pmic_glink_rpmsg_id_match[] = {
	{ PMIC_GLINK_PRIMARY_CHANNEL },
	{ PMIC_GLINK_LOG_CHANNEL },
	{}
};

static struct rpmsg_driver pmic_glink_rpmsg_driver = {
	.probe = pmic_glink_rpmsg_probe,
	.remove = pmic_glink_rpmsg_remove,
	.callback = pmic_glink_rpmsg_callback,
	.id_table = pmic_glink_rpmsg_id_match,
	.drv  = {
		.name  = "qcom_pmic_glink_rpmsg",
	},
};

static int pmic_glink_probe(struct platform_device *pdev)
{
	const unsigned long *match_data;
	struct device_node *charger_node __free(device_node) = NULL;
	struct pdr_service *service;
	struct pmic_glink *pg;
	const char *service_name = "tms/servreg";
	const char *service_path = "msm/adsp/charger_pd";
	int ret;

	pg = devm_kzalloc(&pdev->dev, sizeof(*pg), GFP_KERNEL);
	if (!pg)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, pg);

	pg->dev = &pdev->dev;
	pg->channel_name = PMIC_GLINK_PRIMARY_CHANNEL;
	if (of_property_present(pdev->dev.of_node, "qcom,pmic-glink-channel")) {
		ret = of_property_read_string(pdev->dev.of_node, "qcom,pmic-glink-channel",
					      &pg->channel_name);
		if (ret)
			return ret;
	}
	if (strcmp(pg->channel_name, PMIC_GLINK_PRIMARY_CHANNEL) &&
	    strcmp(pg->channel_name, PMIC_GLINK_LOG_CHANNEL))
		return dev_err_probe(&pdev->dev, -EINVAL, "unsupported glink channel %s\n",
				     pg->channel_name);

	INIT_LIST_HEAD(&pg->clients);
	spin_lock_init(&pg->client_lock);
	mutex_init(&pg->state_lock);

	match_data = (unsigned long *)of_device_get_match_data(&pdev->dev);
	if (!match_data)
		return -EINVAL;

	pg->client_mask = *match_data;
	if (strcmp(pg->channel_name, PMIC_GLINK_PRIMARY_CHANNEL))
		pg->client_mask = 0;
	charger_node = of_get_child_by_name(pdev->dev.of_node, "qcom,battery_charger");
	if (of_device_is_available(charger_node))
		pg->client_mask &= ~BIT(PMIC_GLINK_CLIENT_BATT);

	if (of_property_present(pdev->dev.of_node, "qcom,protection-domain")) {
		ret = of_property_read_string_index(pdev->dev.of_node,
				"qcom,protection-domain", 0, &service_name);
		if (ret)
			return ret;
		ret = of_property_read_string_index(pdev->dev.of_node,
				"qcom,protection-domain", 1, &service_path);
		if (ret)
			return ret;
	} else if (!pg->client_mask) {
		service_name = NULL;
	}

	if (service_name) {
		pg->pdr = pdr_handle_alloc(pmic_glink_pdr_callback, pg);
		if (IS_ERR(pg->pdr))
			return dev_err_probe(&pdev->dev, PTR_ERR(pg->pdr),
					     "failed to initialize pdr\n");
	} else {
		pg->pdr_state = SERVREG_SERVICE_STATE_UP;
	}

	if (pg->client_mask & BIT(PMIC_GLINK_CLIENT_UCSI)) {
		ret = pmic_glink_add_aux_device(pg, &pg->ucsi_aux, "ucsi");
		if (ret)
			goto out_release_pdr_handle;
	}
	if (pg->client_mask & BIT(PMIC_GLINK_CLIENT_ALTMODE)) {
		ret = pmic_glink_add_aux_device(pg, &pg->altmode_aux, "altmode");
		if (ret)
			goto out_release_ucsi_aux;
	}
	if (pg->client_mask & BIT(PMIC_GLINK_CLIENT_BATT)) {
		ret = pmic_glink_add_aux_device(pg, &pg->ps_aux, "power-supply");
		if (ret)
			goto out_release_altmode_aux;
	}

	if (pg->pdr) {
		service = pdr_add_lookup(pg->pdr, service_name, service_path);
		if (IS_ERR(service)) {
			ret = dev_err_probe(&pdev->dev, PTR_ERR(service),
					    "failed adding pdr lookup for %s\n", service_path);
			goto out_release_aux_devices;
		}
	}

	if (of_device_is_available(charger_node)) {
		pg->battery_pdev = of_platform_device_create(charger_node, NULL, &pdev->dev);
		if (!pg->battery_pdev) {
			ret = -ENODEV;
			goto out_release_aux_devices;
		}
		dev_dbg(pg->dev, "CHG DIAG created charger device %s\n",
			 dev_name(&pg->battery_pdev->dev));
	}

	mutex_lock(&__pmic_glink_lock);
	list_add_tail(&pg->node, &pmic_glink_devices);
	mutex_unlock(&__pmic_glink_lock);
	dev_dbg(pg->dev, "USB DIAG glink transport ready channel=%s clients=%lx pdr=%u\n",
		 pg->channel_name, pg->client_mask, !!pg->pdr);

	return 0;

out_release_aux_devices:
	if (pg->client_mask & BIT(PMIC_GLINK_CLIENT_BATT))
		pmic_glink_del_aux_device(pg, &pg->ps_aux);
out_release_altmode_aux:
	if (pg->client_mask & BIT(PMIC_GLINK_CLIENT_ALTMODE))
		pmic_glink_del_aux_device(pg, &pg->altmode_aux);
out_release_ucsi_aux:
	if (pg->client_mask & BIT(PMIC_GLINK_CLIENT_UCSI))
		pmic_glink_del_aux_device(pg, &pg->ucsi_aux);
out_release_pdr_handle:
	if (pg->pdr)
		pdr_handle_release(pg->pdr);

	return ret;
}

static void pmic_glink_remove(struct platform_device *pdev)
{
	struct pmic_glink *pg = dev_get_drvdata(&pdev->dev);

	if (pg->battery_pdev)
		of_platform_device_destroy(&pg->battery_pdev->dev, NULL);

	mutex_lock(&__pmic_glink_lock);
	list_del(&pg->node);
	mutex_unlock(&__pmic_glink_lock);

	if (pg->pdr)
		pdr_handle_release(pg->pdr);

	if (pg->client_mask & BIT(PMIC_GLINK_CLIENT_BATT))
		pmic_glink_del_aux_device(pg, &pg->ps_aux);
	if (pg->client_mask & BIT(PMIC_GLINK_CLIENT_ALTMODE))
		pmic_glink_del_aux_device(pg, &pg->altmode_aux);
	if (pg->client_mask & BIT(PMIC_GLINK_CLIENT_UCSI))
		pmic_glink_del_aux_device(pg, &pg->ucsi_aux);

}

static const unsigned long pmic_glink_sm8450_client_mask = BIT(PMIC_GLINK_CLIENT_BATT) |
							   BIT(PMIC_GLINK_CLIENT_ALTMODE) |
							   BIT(PMIC_GLINK_CLIENT_UCSI);

static const struct of_device_id pmic_glink_of_match[] = {
	{ .compatible = "qcom,pmic-glink", .data = &pmic_glink_sm8450_client_mask },
	{}
};
MODULE_DEVICE_TABLE(of, pmic_glink_of_match);

static struct platform_driver pmic_glink_driver = {
	.probe = pmic_glink_probe,
	.remove = pmic_glink_remove,
	.driver = {
		.name = "qcom_pmic_glink",
		.of_match_table = pmic_glink_of_match,
	},
};

static int pmic_glink_init(void)
{
	int ret;

	ret = platform_driver_register(&pmic_glink_driver);
	if (ret < 0)
		return ret;

	ret = register_rpmsg_driver(&pmic_glink_rpmsg_driver);
	if (ret < 0) {
		platform_driver_unregister(&pmic_glink_driver);
		return ret;
	}

	return 0;
}
module_init(pmic_glink_init);

static void pmic_glink_exit(void)
{
	unregister_rpmsg_driver(&pmic_glink_rpmsg_driver);
	platform_driver_unregister(&pmic_glink_driver);
}
module_exit(pmic_glink_exit);

MODULE_DESCRIPTION("Qualcomm PMIC GLINK driver");
MODULE_LICENSE("GPL");
