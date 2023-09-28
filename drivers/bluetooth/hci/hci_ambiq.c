/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Ambiq SPI based Bluetooth HCI driver.
 */

#define DT_DRV_COMPAT ambiq_bt_hci_spi

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/bluetooth/hci_driver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_ambiq.h>
#include <zephyr/bluetooth/hci.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_hci_driver);

#include <am_mcu_apollo.h>
#include <am_devices_ble_ctrl.h>

#define SPI_DEV_NODE DT_NODELABEL(iom4)
#define HCI_SPI_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ambiq_bt_hci_spi)
#define CLK_32M_NODE DT_NODELABEL(xo32m)
#define CLK_32K_NODE DT_NODELABEL(xo32k)

#define HCI_CMD 0x01
#define HCI_ACL 0x02
#define HCI_SCO 0x03
#define HCI_EVT 0x04

/* Command/response for SPI operation */
#define SPI_WRITE   0x80
#define SPI_READ    0x04
#define READY_BYTE0 0x68
#define READY_BYTE1 0xA8

/* Offset of special item */
#define PACKET_TYPE         0
#define EVT_HEADER_TYPE     0
#define EVT_HEADER_EVENT    1
#define EVT_HEADER_SIZE     2
#define EVT_VENDOR_CODE_LSB 3
#define EVT_VENDOR_CODE_MSB 4
#define CMD_OGF             1
#define CMD_OCF             2

/* Max SPI buffer length for transceive operations.
 * The maximum TX packet number is 512 bytes data + 12 bytes header.
 * The maximum RX packet number is 255 bytes data + 3 header.
 */
#define SPI_MAX_TX_MSG_LEN 524
#define SPI_MAX_RX_MSG_LEN 258

#define SPI_WRITE_TIMEOUT 200

static uint8_t g_hciRxMsg[SPI_MAX_RX_MSG_LEN];
static uint8_t g_nop_packet[6] = {0xe, 0x4, 0x5, 0x0, 0x0, 0x0};

static const struct gpio_dt_spec irq_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, irq_gpios);
static const struct gpio_dt_spec rst_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, reset_gpios);
static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, cs_gpios);
static const struct gpio_dt_spec clkreq_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, clkreq_gpios);

static struct gpio_callback irq_gpio_cb;
static struct gpio_callback clkreq_gpio_cb;

static K_SEM_DEFINE(sem_irq, 0, 1);
static K_SEM_DEFINE(sem_spi_available, 1, 1);
#if defined(CONFIG_BT_HCI_SETUP)
static K_SEM_DEFINE(sem_initialised, 0, 1);
static bool g_preInitialising = false;
#endif /* defined(CONFIG_BT_HCI_SETUP) */

const struct device *clk32m_dev = DEVICE_DT_GET(CLK_32M_NODE);
const struct device *clk32k_dev = DEVICE_DT_GET(CLK_32K_NODE);

const struct device *spi_dev = DEVICE_DT_GET(SPI_DEV_NODE);
static struct spi_config spi_cfg = {
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA |
		     SPI_WORD_SET(8),
};
static K_KERNEL_STACK_DEFINE(spi_rx_stack, CONFIG_BT_DRV_RX_STACK_SIZE);
static struct k_thread spi_rx_thread_data;

static struct spi_buf spi_tx_buf;
static struct spi_buf spi_rx_buf;
static const struct spi_buf_set spi_tx = {.buffers = &spi_tx_buf, .count = 1};
static const struct spi_buf_set spi_rx = {.buffers = &spi_rx_buf, .count = 1};

static inline int bt_spi_transceive(void *tx, uint32_t tx_len, void *rx, uint32_t rx_len)
{
	spi_tx_buf.buf = tx;
	spi_tx_buf.len = (size_t)tx_len;
	spi_rx_buf.buf = rx;
	spi_rx_buf.len = (size_t)rx_len;
	return spi_transceive(spi_dev, &spi_cfg, &spi_tx, &spi_rx);
}

static inline uint16_t bt_spi_get_cmd(uint8_t *txmsg)
{
	return (txmsg[CMD_OCF] << 8) | txmsg[CMD_OGF];
}

static inline uint16_t bt_spi_get_evt(uint8_t *rxmsg)
{
	return (rxmsg[EVT_VENDOR_CODE_MSB] << 8) | rxmsg[EVT_VENDOR_CODE_LSB];
}

static bool irq_pin_high(void)
{
	int pin_state;
	pin_state = gpio_pin_get_dt(&irq_gpio);
	LOG_DBG("IRQ Pin: %d", pin_state);
	return pin_state > 0;
}

static bool clkreq_pin_status(void)
{
	int pin_state;
	pin_state = gpio_pin_get_dt(&clkreq_gpio);
	LOG_DBG("CLKREQ Pin: %d", pin_state);
	return pin_state > 0;
}

static void bt_packet_irq_isr(const struct device *unused1, struct gpio_callback *unused2,
			      uint32_t unused3)
{
	k_sem_give(&sem_irq);
}

static void bt_clkreq_isr(const struct device *unused1, struct gpio_callback *unused2,
			  uint32_t unused3)
{
	if (clkreq_pin_status()) {
		/* Enable XO32MHz */
		clock_control_on(clk32m_dev,
				 (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE);
		gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_FALLING);
	} else {
		/* Disable XO32MHz */
		clock_control_off(clk32m_dev,
				  (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_HFXTAL_BLE);
		gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_RISING);
	}
}

static void configure_cs(void)
{
	/* Configure CS GPIO to SPI CS function */
	PINCTRL_DT_DEFINE(SPI_DEV_NODE);
	pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(SPI_DEV_NODE), PINCTRL_STATE_DEFAULT);
}

static void release_cs(void)
{
	gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_HIGH);
}

static void bt_spi_rx_thread(void)
{
	bool discardable = false;
	k_timeout_t timeout = K_FOREVER;
	struct net_buf *buf = NULL;
	uint8_t command[1] = {SPI_READ};
	uint8_t response[2] = {0, 0};
	struct bt_hci_acl_hdr acl_hdr = {0};
	uint16_t read_size = 0;
	int ret = 0;
	int len = 0;

	while (true) {
		/* Wait for IRQ interrupt */
		k_sem_take(&sem_irq, K_FOREVER);

		/* Wait for SPI bus to be available */
		k_sem_take(&sem_spi_available, K_FOREVER);

		do {
			/* Skip if the IRQ pin is not in high state */
			if (!irq_pin_high()) {
				ret = -1;
				break;
			}

			/* Check the available packet bytes */
			ret = bt_spi_transceive(command, 1, response, 2);
			if (ret) {
				break;
			}

			/* Check if the read size is acceptable */
			read_size = (uint16_t)(response[0] | response[1] << 8);
			if ((read_size == 0) || (read_size > SPI_MAX_RX_MSG_LEN)) {
				ret = -1;
				break;
			}

			/* Read the HCI data from controller */
			ret = bt_spi_transceive(NULL, 0, &g_hciRxMsg, read_size);

			if (ret) {
				LOG_ERR("SPI read error %d", ret);
				break;
			}

			/* Free the SPI bus as soon as possible */
			k_sem_give(&sem_spi_available);

#if defined(CONFIG_BT_HCI_SETUP)
			if (g_preInitialising) {
				/* Handshake with BLE controller in initialization stage */
				am_devices_ble_ctrl_handshake_recv(&g_hciRxMsg[0]);
				if (am_devices_ble_ctrl_fw_update() ==
				    AM_DEVICES_BLE_CTRL_SBL_STATUS_OK) {
					g_preInitialising = false;
					k_sem_give(&sem_initialised);
				}
				break;
			}
#endif /* defined(CONFIG_BT_HCI_SETUP) */

			switch (g_hciRxMsg[PACKET_TYPE]) {
			case HCI_EVT:
				if (g_hciRxMsg[1] == BT_HCI_EVT_LE_META_EVENT &&
				    (g_hciRxMsg[3] == BT_HCI_EVT_LE_ADVERTISING_REPORT)) {
					discardable = true;
					timeout = K_NO_WAIT;
				}

				/* Ingore none-opcode packet */
				if (memcmp(&g_hciRxMsg[1], g_nop_packet, sizeof(g_nop_packet)) ==
				    0) {
					break;
				}

				buf = bt_buf_get_evt(g_hciRxMsg[EVT_HEADER_EVENT], discardable,
						     timeout);
				if (!buf) {
					LOG_DBG("Discard adv report due to insufficient "
						"buf");
					break;
				}

				len = sizeof(struct bt_hci_evt_hdr) + g_hciRxMsg[EVT_HEADER_SIZE];
				if (len > net_buf_tailroom(buf)) {
					LOG_ERR("Event too long: %d", len);
					net_buf_unref(buf);
				} else {
					net_buf_add_mem(buf, &g_hciRxMsg[1], len);
					/* Post the RX message to host stack to process */
					bt_recv(buf);
				}
				break;
			case HCI_ACL:
				buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
				memcpy(&acl_hdr, &g_hciRxMsg[1], sizeof(acl_hdr));
				len = sizeof(acl_hdr) + sys_le16_to_cpu(acl_hdr.len);
				if (len > net_buf_tailroom(buf)) {
					LOG_ERR("ACL too long: %d", len);
					net_buf_unref(buf);
				} else {
					net_buf_add_mem(buf, &g_hciRxMsg[1], len);
					/* Post the RX message to host stack to process */
					bt_recv(buf);
				}
				break;
			default:
				LOG_ERR("Unknown BT buf type %d", g_hciRxMsg[0]);
				break;
			}
		} while (0);

		if (ret) {
			/* Free the SPI bus */
			k_sem_give(&sem_spi_available);
		}
	}
}

int spi_blocking_send(uint8_t *data, uint32_t len)
{
	uint8_t command[1] = {SPI_WRITE};
	uint8_t response[2] = {0, 0};
	int ret = 0;
	uint16_t fail_count = 0;

	do {
		/* Check if the controller is ready to receive the HCI packets. */
		ret = bt_spi_transceive(command, 1, response, 2);
		// printk("");
		if ((response[0] != READY_BYTE0) || (response[1] != READY_BYTE1) || ret) {
			k_sleep(K_MSEC(1));
			release_cs();
			k_sleep(K_MSEC(1));
			configure_cs();
			k_sleep(K_MSEC(2));
		} else {
			/* Transmit the message */
			ret = bt_spi_transceive(data, len, NULL, 0);
			if (ret) {
				LOG_ERR("SPI write error %d", ret);
			}
			break;
		}
	} while (fail_count++ < SPI_WRITE_TIMEOUT);

	return ret;
}

static int bt_spi_send(struct net_buf *buf)
{
	int ret = 0;

	/* Buffer needs an additional byte for type */
	if (buf->len >= SPI_MAX_TX_MSG_LEN) {
		LOG_ERR("Message too long");
		return -EINVAL;
	}

	/* Wait for SPI bus to be available */
	k_sem_take(&sem_spi_available, K_FOREVER);

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
		net_buf_push_u8(buf, HCI_ACL);
		break;
	case BT_BUF_CMD:
		net_buf_push_u8(buf, HCI_CMD);
		break;
	default:
		LOG_ERR("Unsupported type");
		k_sem_give(&sem_spi_available);
		return -EINVAL;
	}

	/* Send the SPI packet */
	ret = spi_blocking_send(buf->data, buf->len);

	/* Free the SPI bus */
	k_sem_give(&sem_spi_available);

	net_buf_unref(buf);

	return ret;
}

static int bt_spi_open(void)
{
	int ret = 0;

	/* Configure RST pin */
	ret = gpio_pin_configure_dt(&rst_gpio, GPIO_OUTPUT_HIGH);
	if (ret) {
		return ret;
	}

	/* Configure IRQ pin and the IRQ callback */
	ret = gpio_pin_configure_dt(&irq_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&irq_gpio_cb, bt_packet_irq_isr, BIT(irq_gpio.pin));
	ret = gpio_add_callback(irq_gpio.port, &irq_gpio_cb);
	if (ret) {
		return ret;
	}

	/* Configure CLKREQ pin and the IRQ callback */
	ret = gpio_pin_configure_dt(&clkreq_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&clkreq_gpio_cb, bt_clkreq_isr, BIT(clkreq_gpio.pin));
	ret = gpio_add_callback(clkreq_gpio.port, &clkreq_gpio_cb);
	if (ret) {
		return ret;
	}

	/* Configure the interrupt edge for CLKREQ pin */
	gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_RISING);

	/* Reset the BLE controller */
	gpio_pin_set_dt(&rst_gpio, 0);

	/* Take BLE out of reset */
	k_sleep(K_MSEC(20));
	gpio_pin_set_dt(&rst_gpio, 1);

	/* Give the controller some time to boot */
	k_sleep(K_MSEC(700));

	/* Configure the interrupt edge for IRQ pin */
	gpio_pin_interrupt_configure_dt(&irq_gpio, GPIO_INT_EDGE_RISING);

	/* Start RX thread */
	k_thread_create(&spi_rx_thread_data, spi_rx_stack, K_KERNEL_STACK_SIZEOF(spi_rx_stack),
			(k_thread_entry_t)bt_spi_rx_thread, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_DRIVER_RX_HIGH_PRIO), 0, K_NO_WAIT);

	return 0;
}

#if defined(CONFIG_BT_HCI_SETUP)
static int bt_set_nvds(void)
{
	struct net_buf *buf;
	uint8_t *p = &am_ble_ctrl_nvds_data[0];

	buf = bt_hci_cmd_create(HCI_VSC_UPDATE_NVDS_CFG_CMD_OPCODE,
				HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH);
	if (!buf) {
		return -ENOBUFS;
	}

	p = net_buf_add(buf, HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH);
	return bt_hci_cmd_send_sync(HCI_VSC_UPDATE_NVDS_CFG_CMD_OPCODE, buf, NULL);
}

static int bt_spi_setup(void)
{
	int ret = 0;

	/* Configure the XO32M and XO32k clocks.*/
	clock_control_configure(clk32k_dev, NULL, NULL);
	clock_control_configure(clk32m_dev, NULL, NULL);

	/* Enable XO32kHz for Controller */
	clock_control_on(clk32k_dev, (clock_control_subsys_t)CLOCK_CONTROL_AMBIQ_TYPE_LFXTAL);

	/* Check if BLE Controller firmware update is needed */
	am_devices_ble_ctrl_fw_update_init();
	am_devices_ble_ctrl_fw_update();
	g_preInitialising = true;

	/* Device will let us know when it's ready */
	k_sem_take(&sem_initialised, K_FOREVER);

	/* Set the NVDS parameters to BLE Controller*/
	ret = bt_set_nvds();
	/* Give some time to make NVDS take effect in Controller */
	k_sleep(K_MSEC(2));

	return ret;
}
#endif /* defined(CONFIG_BT_HCI_SETUP) */

static const struct bt_hci_driver drv = {
	.name = "ambiq hci",
	.bus = BT_HCI_DRIVER_BUS_SPI,
	.open = bt_spi_open,
	.send = bt_spi_send,
#if defined(CONFIG_BT_HCI_SETUP)
	.setup = bt_spi_setup,
#endif /* defined(CONFIG_BT_HCI_SETUP) */
};

static int bt_spi_init(void)
{
	if (!device_is_ready(spi_dev)) {
		printk("SPI device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(irq_gpio.port)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(rst_gpio.port)) {
		LOG_ERR("Reset GPIO device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(clkreq_gpio.port)) {
		LOG_ERR("CLKREQ GPIO device not ready");
		return -ENODEV;
	}

	bt_hci_driver_register(&drv);

	LOG_DBG("BT SPI initialized");

	return 0;
}

SYS_INIT(bt_spi_init, POST_KERNEL, CONFIG_BT_SPI_INIT_PRIORITY);
