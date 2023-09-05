/* hci_ambiq.c - Ambiq SPI based Bluetooth driver */

#define DT_DRV_COMPAT ambiq_bt_hci_spi

/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/bluetooth/hci_driver.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_driver);

#include <am_mcu_apollo.h>

#define SPI_DEV_NODE    DT_NODELABEL(spi2)
#define HCI_SPI_NODE    DT_COMPAT_GET_ANY_STATUS_OKAY(ambiq_bt_hci_spi)

#define HCI_CMD			0x01
#define HCI_ACL			0x02
#define HCI_SCO			0x03
#define HCI_EVT			0x04

/* Special Values */
#define SPI_WRITE		0x80
#define SPI_READ		0x04
#define READY_BYTE0		0x68
#define READY_BYTE1		0xA8

#define PACKET_TYPE		    0
#define EVT_HEADER_TYPE		0
#define EVT_HEADER_EVENT	1
#define EVT_HEADER_SIZE		2
#define EVT_VENDOR_CODE_LSB	3
#define EVT_VENDOR_CODE_MSB	4

#define CMD_OGF			1
#define CMD_OCF			2

/* Max SPI buffer length for transceive operations. */
#define SPI_MAX_TX_MSG_LEN       524 //!<  the max packet of SBL to controller is 512 plus 12 bytes header
#define SPI_MAX_RX_MSG_LEN       258 //!<  255 data + 3 header

static uint8_t g_hciRxMsg[SPI_MAX_RX_MSG_LEN];
static uint8_t g_nop_packet[6] = {0xe, 0x4, 0x5, 0x0, 0x0, 0x0};

static const struct gpio_dt_spec irq_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, irq_gpios);
static const struct gpio_dt_spec rst_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, reset_gpios);
const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, cs_gpios);
static const struct gpio_dt_spec clkreq_gpio = GPIO_DT_SPEC_GET(HCI_SPI_NODE, clkreq_gpios);

static struct gpio_callback	irq_gpio_cb;
static struct gpio_callback	clkreq_gpio_cb;

static K_SEM_DEFINE(sem_irq, 0, 1);
static K_SEM_DEFINE(sem_spi_available, 1, 1);

const struct device *spi_dev = DEVICE_DT_GET(SPI_DEV_NODE);
static struct spi_config spi_cfg = 
{
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8),
};
static K_KERNEL_STACK_DEFINE(spi_rx_stack, CONFIG_BT_DRV_RX_STACK_SIZE);
static struct k_thread spi_rx_thread_data;

#if defined(CONFIG_BT_HCI_DRIVER_LOG_LEVEL_DBG)
#include <zephyr/sys/printk.h>
static inline void spi_dump_message(const uint8_t *pre, uint8_t *buf,
				    uint16_t size)
{
	uint8_t c;
	printk("%s : ");
	for (uint16_t i = 0; i < size; i++) {
		c = buf[i];
		printk("%02X ", c);
	}
	printk("\n");
}
#else
static inline
void spi_dump_message(const uint8_t *pre, uint8_t *buf, uint16_t size) {}
#endif

static struct spi_buf spi_tx_buf;
static struct spi_buf spi_rx_buf;
static const struct spi_buf_set spi_tx = {
	.buffers = &spi_tx_buf,
	.count = 1
};
static const struct spi_buf_set spi_rx = {
	.buffers = &spi_rx_buf,
	.count = 1
};

static inline int bt_spi_transceive(void *tx, uint32_t tx_len,
				    void *rx, uint32_t rx_len)
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

static void bt_packet_irq_isr(const struct device *unused1,
		       struct gpio_callback *unused2,
		       uint32_t unused3)
{
	LOG_DBG("");
	k_sem_give(&sem_irq);
}

static void bt_clkreq_isr(const struct device *unused1,
		       struct gpio_callback *unused2,
		       uint32_t unused3)
{
	LOG_DBG("");
	if (clkreq_pin_status()) 
	{
		// TODO: enable XTAL32MHz
        gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_FALLING);
	} 
	else
	{
		// TODO: disable XTAL32MHz
	    gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_RISING);
	}
}

static void configure_cs(void)
{
	/* Configure CS pin as to SPI CS function */
#if 0
	am_hal_gpio_pincfg_t g_AM_BLE_SPI_CS =
	{
		.GP.cfg_b.uFuncSel             = AM_HAL_PIN_72_NCE72,
		.GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
		.GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
		.GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
		.GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
		.GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X,
		.GP.cfg_b.uSlewRate            = 0,
		.GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
		.GP.cfg_b.uNCE                 = AM_HAL_GPIO_NCE_IOM2CE0,
		.GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
		.GP.cfg_b.uRsvd_0              = 0,
		.GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
		.GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
		.GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
		.GP.cfg_b.uRsvd_1              = 0,
	};
	am_hal_gpio_pinconfig(72, g_AM_BLE_SPI_CS);
#else
	PINCTRL_DT_DEFINE(SPI_DEV_NODE);
	pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(SPI_DEV_NODE), PINCTRL_STATE_DEFAULT);
#endif
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
	struct bt_hci_acl_hdr acl_hdr;
	uint16_t size = 0;
	int ret;
	int len;

	while (true) {
		/* Wait for IRQ interrupt */
		k_sem_take(&sem_irq, K_FOREVER);

		/* Wait for SPI bus to be available */
		k_sem_take(&sem_spi_available, K_FOREVER);
        
		do {
			ret = -1;

			/* Skip if the IRQ pin is not in high state */
		    if (!irq_pin_high()) {
				break;
			}

			/* Check the available packet bytes */
			ret = bt_spi_transceive(command, 1, response, 2);
			if (ret) {
				break;
			}

			size = (uint16_t)(response[0] | response[1] << 8);
			if ((size == 0) || (size > SPI_MAX_RX_MSG_LEN)) {
				break;
			}
			
			/* Read the HCI data from controller */
			ret = bt_spi_transceive(NULL, 0, &g_hciRxMsg, size);		

			if (ret) {
				LOG_ERR("Error %d", ret);
				break;
			}

			/* Free the SPI bus as soon as possible */
			k_sem_give(&sem_spi_available);

			spi_dump_message("HCI RX: ", g_hciRxMsg, size);

			switch (g_hciRxMsg[PACKET_TYPE]) {
			case HCI_EVT:
				if (g_hciRxMsg[1] == BT_HCI_EVT_LE_META_EVENT &&
					(g_hciRxMsg[3] == BT_HCI_EVT_LE_ADVERTISING_REPORT)) {
					discardable = true;
					timeout = K_NO_WAIT;
				}

				/* Ingore none-opcode packet */
				if (memcmp(&g_hciRxMsg[1], g_nop_packet, sizeof(g_nop_packet)) == 0) {
					break;
				}

				buf = bt_buf_get_evt(g_hciRxMsg[EVT_HEADER_EVENT],
								discardable, timeout);
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

static int bt_spi_send(struct net_buf *buf)
{
	uint8_t command[1] = {SPI_WRITE};
	uint8_t response[2] = {0, 0};
	int ret;
	uint16_t fail_count = 0;

	LOG_DBG("");

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

	do {
		/* Check if the controller is ready to receive the HCI packets. */		
		ret = bt_spi_transceive(command, 1, response, 2);
		if ((response[0] != READY_BYTE0) || (response[1] != READY_BYTE1) || ret)
		{
			k_sleep(K_MSEC(1));
			release_cs();
			k_sleep(K_MSEC(1));
			configure_cs();
			k_sleep(K_MSEC(2));
		}
		else
		{
			/* Transmit the message */
			ret = bt_spi_transceive(buf->data, buf->len, NULL, 0);
			spi_dump_message("HCI TX: ", buf->data, buf->len);
			if (ret) {
				LOG_ERR("Error %d", ret);
			}			
			break;
		}
	} while (fail_count++ < 200);

	/* Free the SPI bus */
	k_sem_give(&sem_spi_available);

	net_buf_unref(buf);

	return ret;
}

static int bt_spi_open(void)
{
	int err;

	/* Configure RST pin */
	err = gpio_pin_configure_dt(&rst_gpio, GPIO_OUTPUT_HIGH);
	if (err) {
		return err;
	}

	/* Configure IRQ pin and the IRQ call-back/handler */
	err = gpio_pin_configure_dt(&irq_gpio, GPIO_INPUT);
	if (err) {
		return err;
	}

	gpio_init_callback(&irq_gpio_cb, bt_packet_irq_isr, BIT(irq_gpio.pin));
	err = gpio_add_callback(irq_gpio.port, &irq_gpio_cb);
	if (err) {
		return err;
	}

	/* Configure CLKREQ pin and the IRQ call-back/handler */
	err = gpio_pin_configure_dt(&clkreq_gpio, GPIO_INPUT);
	if (err) {
		return err;
	}

	gpio_init_callback(&clkreq_gpio_cb, bt_clkreq_isr, BIT(clkreq_gpio.pin));
	err = gpio_add_callback(clkreq_gpio.port, &clkreq_gpio_cb);
	if (err) {
		return err;
	}
	gpio_pin_interrupt_configure_dt(&clkreq_gpio, GPIO_INT_EDGE_RISING);

	k_sleep(K_MSEC(20));
	gpio_pin_set_dt(&rst_gpio, 0);

	/* Take BLE out of reset */
	k_sleep(K_MSEC(20));
	gpio_pin_set_dt(&rst_gpio, 1);

	/* Give the controller some time to boot */
	k_sleep(K_MSEC(700));

	gpio_pin_interrupt_configure_dt(&irq_gpio, GPIO_INT_EDGE_RISING);
	/* Start RX thread */
	k_thread_create(&spi_rx_thread_data, spi_rx_stack,
			K_KERNEL_STACK_SIZEOF(spi_rx_stack),
			(k_thread_entry_t)bt_spi_rx_thread, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_DRIVER_RX_HIGH_PRIO),
			0, K_NO_WAIT);

	return 0;
}

#if defined(CONFIG_BT_HCI_SETUP)
static int bt_spi_setup(void)
{
    return 0;
}
#endif /* defined(CONFIG_BT_HCI_SETUP) */

static const struct bt_hci_driver drv = {
	.name		= "ambiq hci",
	.bus		= BT_HCI_DRIVER_BUS_SPI,
	.open		= bt_spi_open,
	.send		= bt_spi_send,
#if defined(CONFIG_BT_HCI_SETUP)
    .setup      = bt_spi_setup,
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
