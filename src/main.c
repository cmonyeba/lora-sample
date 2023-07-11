
/*
 * Copyright (c) 2023 Craig Peacock
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/gpio.h>

#define LOG_LEVEL CONFIG_LOG_INF_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lora);

#define SLEEP_TIME_MS 1000
#define THREAD0_STACKSIZE       512

/* STEP 3 - Set the priority of the two threads to have equal priority*/
#define THREAD0_PRIORITY 4 

int lora_configure(const struct device *dev, bool transmit);
void lora_recv_callback(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr);

#define TRANSMIT 1
#define RECEIVE 0

#define TOGGLE 0xFA

uint8_t data_tx = TOGGLE;
const struct device *dev_lora;

static struct gpio_callback button_callback_data;
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0});

static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

K_SEM_DEFINE(test_mutex, 0, 1);

void button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Unlocking Mutex\n");
	k_sem_give(&test_mutex);

}

void lora_recv_callback(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr)
{
	// When lora_recv_async is cancelled, may be called with 0 bytes.
	if (size != 0) {
		printk("RECV %d bytes: ",size);
		for (uint16_t i = 0; i < size; i++)
			printk("0x%02x ",data[i]);
		printk("RSSI = %ddBm, SNR = %ddBm\n", rssi, snr);

        if(data[0] == TOGGLE)
        {
            printk("toggling led");
            gpio_pin_toggle_dt(&led);
        }
	}
}

int lora_configure(const struct device *dev, bool transmit)
{
	int ret;
	struct lora_modem_config config;

	config.frequency = 916800000;
	config.bandwidth = BW_125_KHZ;
	config.datarate = SF_10;
	config.preamble_len = 8;
	config.coding_rate = CR_4_5;
	config.tx_power = 4;
	config.iq_inverted = false;
	config.public_network = false;
	config.tx = transmit;

	ret = lora_config(dev, &config);
	if (ret < 0) {
		LOG_ERR("LoRa device configuration failed");
		return false;
	}

	return(true);
}

int main(void)
{
	
	int ret;

	printk("LoRa Point to Point Communications Example\n");

	// Setup LoRa Radio Device:
	dev_lora = DEVICE_DT_GET(DT_ALIAS(lora0));
	if (!device_is_ready(dev_lora)) {
		printk("%s: device not ready", dev_lora->name);
		return 0;
	}

	if (lora_configure(dev_lora, RECEIVE)) {
		printk("LoRa Device Configured\n");
	} else {
		return 0;
	}

	// Setup SW1 Momentary Push Button:
	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n", button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_callback_data, button_callback, BIT(button.pin));
	gpio_add_callback(button.port, &button_callback_data);

    // Setup LED
    if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}

	// Start LoRa radio listening
	ret = lora_recv_async(dev_lora, lora_recv_callback);
	if (ret < 0) {
		LOG_ERR("LoRa recv_async failed %d\n", ret);
	}

    return 0;
}


void thread0(void)
{
    int ret, bytes;

	printk("Thread 0 started\n");
	while (1) 
    {
		// // Wait for SW1 to be pressed. Onced pressed, transmit some data
        printk("Locking Mutex\n");
        if(k_sem_take(&test_mutex, K_FOREVER) != 0)
        {
            printk("big errorrr");
        }
		
        // Cancel reception
        printk("Cancelling Reception\n");
        ret = lora_recv_async(dev_lora, NULL);
        if (ret < 0) {
            LOG_ERR("LoRa recv_async failed %d\n", ret);
        }

        // Reconfigure radio for transmit
        lora_configure(dev_lora, TRANSMIT);

        // Transmit data
        printk("Transmiting Data\n");
        ret = lora_send(dev_lora, &data_tx, sizeof(data_tx));
        if (ret < 0) {
            LOG_ERR("LoRa send failed");
        } else {
            bytes = sizeof(data_tx);
            printk("XMIT %d bytes: ", bytes);
            printk("0x%02x \n",data_tx);
        }

        // Restart reception
        printk("Restarting Reception\n");
        lora_configure(dev_lora, RECEIVE);
        ret = lora_recv_async(dev_lora, lora_recv_callback);
        if (ret < 0) {
            LOG_ERR("LoRa recv_async failed %d\n", ret);
            }

        // k_msleep(SLEEP_TIME_MS);
	}
}

// Define and initialize threads
K_THREAD_DEFINE(thread0_id, THREAD0_STACKSIZE, thread0, NULL, NULL, NULL, THREAD0_PRIORITY, 0, 5000);