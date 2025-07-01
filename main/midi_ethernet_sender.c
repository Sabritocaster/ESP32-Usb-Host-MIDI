/*
 * MIDI over Ethernet sender using W5500
 * Modified from original MIDI USB Host example
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"

// W5500 Configuration
#define W5500_CS_PIN 5
#define W5500_SCK_PIN 6
#define W5500_MOSI_PIN 7
#define W5500_MISO_PIN 3
#define W5500_RST_PIN 2
#define W5500_INT_PIN 4

// Network Configuration
#define MAC_ADDR {0x00, 0x08, 0xDC, 0x56, 0x34, 0x12}
#define IP_ADDR {192, 168, 1, 100} // Your ESP32's IP
#define SUBNET_MASK {255, 255, 255, 0}
#define GATEWAY_ADDR {192, 168, 1, 1}
#define TARGET_IP {192, 168, 1, 10} // Your Mac/PC's IP
#define TARGET_PORT 6666            // Port that will be listened on the Mac/PC
#define LOCAL_PORT 5005

// MIDI Configuration
#define ACTION_OPEN_DEV 0x01
#define ACTION_GET_DEV_INFO 0x02
#define ACTION_GET_DEV_DESC 0x04
#define ACTION_GET_CONFIG_DESC 0x08
#define ACTION_GET_STR_DESC 0x10
#define ACTION_CLAIM_INTERFACE 0x20
#define ACTION_START_READING_DATA 0x40
#define ACTION_CLOSE_DEV 0x80
#define ACTION_EXIT 0xA0

#define USB_CLIENT_NUM_EVENT_MSG 5
#define MIDI_MESSAGE_LENGTH 4

// W5500 Register Definitions
#define W5500_MR 0x0000
#define W5500_GAR 0x0001
#define W5500_SUBR 0x0005
#define W5500_SHAR 0x0009
#define W5500_SIPR 0x000F
#define W5500_S0_MR 0x0000
#define W5500_S0_CR 0x0001
#define W5500_S0_IR 0x0002
#define W5500_S0_SR 0x0003
#define W5500_S0_PORT 0x0004
#define W5500_S0_DIPR 0x000C
#define W5500_S0_DPORT 0x0010
#define W5500_S0_RXBUF_SIZE 0x001E
#define W5500_S0_TXBUF_SIZE 0x001F
#define W5500_S0_TX_FSR 0x0020
#define W5500_S0_TX_RD 0x0022
#define W5500_S0_TX_WR 0x0024
#define W5500_S0_RX_RSR 0x0026
#define W5500_S0_RX_RD 0x0028

// Control byte definitions
#define W5500_BSB_S0_REG 0x08
#define W5500_BSB_S0_TX 0x10
#define W5500_BSB_S0_RX 0x18
#define W5500_BSB_COMMON 0x00

// Socket commands
#define W5500_CR_OPEN 0x01
#define W5500_CR_CONNECT 0x04
#define W5500_CR_SEND 0x20
#define W5500_CR_RECV 0x40

// Socket modes
#define W5500_MR_UDP 0x02

typedef struct
{
    uint8_t interface_nmbr;
    uint8_t alternate_setting;
    uint8_t endpoint_address;
    uint8_t max_packet_size;
} interface_config_t;

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
    interface_config_t interface_conf;
} class_driver_t;

typedef struct
{
    uint8_t status;
    uint8_t data1;
    uint8_t data2;
    uint8_t velocity;
} midi_message_t;

static const char *DRIVER_TAG = "MIDI DRIVER";
static const char *ETH_TAG = "ETHERNET";
static spi_device_handle_t spi_handle;
static QueueHandle_t midi_queue;

// W5500 SPI Functions
static void w5500_write_reg(uint16_t addr, uint8_t cb, uint8_t data)
{
    spi_transaction_t trans = {0};
    uint8_t tx_data[4] = {(addr >> 8) & 0xFF, addr & 0xFF, cb | 0x04, data};

    trans.length = 32;
    trans.tx_buffer = tx_data;

    spi_device_transmit(spi_handle, &trans);
}

static uint8_t w5500_read_reg(uint16_t addr, uint8_t cb)
{
    spi_transaction_t trans = {0};
    uint8_t tx_data[4] = {(addr >> 8) & 0xFF, addr & 0xFF, cb, 0x00};
    uint8_t rx_data[4] = {0};

    trans.length = 32;
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;

    spi_device_transmit(spi_handle, &trans);
    return rx_data[3];
}

static void w5500_write_buf(uint16_t addr, uint8_t cb, uint8_t *buf, uint16_t len)
{
    spi_transaction_t trans = {0};
    uint8_t full_buf[3 + len];
    
    full_buf[0] = (addr >> 8) & 0xFF;
    full_buf[1] = addr & 0xFF;
    full_buf[2] = cb | 0x04;
    memcpy(&full_buf[3], buf, len);

    trans.length = (3 + len) * 8;
    trans.tx_buffer = full_buf;

    spi_device_transmit(spi_handle, &trans);
}

static void w5500_init_spi(void)
{
    esp_err_t ret;

    // Configure reset pin
    gpio_config_t rst_config = {
        .pin_bit_mask = (1ULL << W5500_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_config);

    // Reset W5500
    gpio_set_level(W5500_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(W5500_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure SPI bus
    spi_bus_config_t bus_config = {
        .miso_io_num = W5500_MISO_PIN,
        .mosi_io_num = W5500_MOSI_PIN,
        .sclk_io_num = W5500_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
    };

    ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Configure SPI device
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
        .mode = 0,
        .spics_io_num = W5500_CS_PIN,
        .queue_size = 7,
    };

    ret = spi_bus_add_device(SPI2_HOST, &dev_config, &spi_handle);
    ESP_ERROR_CHECK(ret);
}

static void w5500_init_network(void)
{
    uint8_t mac[6] = MAC_ADDR;
    uint8_t ip[4] = IP_ADDR;
    uint8_t subnet[4] = SUBNET_MASK;
    uint8_t gateway[4] = GATEWAY_ADDR;

    ESP_LOGI(ETH_TAG, "Initializing W5500 network settings");

    // Set MAC address
    for (int i = 0; i < 6; i++)
    {
        w5500_write_reg(W5500_SHAR + i, W5500_BSB_COMMON, mac[i]);
    }

    // Set IP address
    for (int i = 0; i < 4; i++)
    {
        w5500_write_reg(W5500_SIPR + i, W5500_BSB_COMMON, ip[i]);
    }

    // Set subnet mask
    for (int i = 0; i < 4; i++)
    {
        w5500_write_reg(W5500_SUBR + i, W5500_BSB_COMMON, subnet[i]);
    }

    // Set gateway
    for (int i = 0; i < 4; i++)
    {
        w5500_write_reg(W5500_GAR + i, W5500_BSB_COMMON, gateway[i]);
    }

    ESP_LOGI(ETH_TAG, "Network configured: IP %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
}

static void w5500_socket_init(void)
{
    uint8_t target_ip[4] = TARGET_IP;

    ESP_LOGI(ETH_TAG, "Initializing UDP socket");

    // Set socket mode to UDP
    w5500_write_reg(W5500_S0_MR, W5500_BSB_S0_REG, W5500_MR_UDP);

    // Set source port
    w5500_write_reg(W5500_S0_PORT, W5500_BSB_S0_REG, (LOCAL_PORT >> 8) & 0xFF);
    w5500_write_reg(W5500_S0_PORT + 1, W5500_BSB_S0_REG, LOCAL_PORT & 0xFF);

    // Set destination IP
    for (int i = 0; i < 4; i++)
    {
        w5500_write_reg(W5500_S0_DIPR + i, W5500_BSB_S0_REG, target_ip[i]);
    }

    // Set destination port
    w5500_write_reg(W5500_S0_DPORT, W5500_BSB_S0_REG, (TARGET_PORT >> 8) & 0xFF);
    w5500_write_reg(W5500_S0_DPORT + 1, W5500_BSB_S0_REG, TARGET_PORT & 0xFF);

    // Open socket
    w5500_write_reg(W5500_S0_CR, W5500_BSB_S0_REG, W5500_CR_OPEN);

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t status = w5500_read_reg(W5500_S0_SR, W5500_BSB_S0_REG);
    ESP_LOGI(ETH_TAG, "Socket status: 0x%02X", status);
}

static void send_midi_over_ethernet(midi_message_t *midi_msg)
{
    // Simple MIDI over UDP packet format
    uint8_t packet[8] = {
        0x80, // MIDI command byte
        0x61, // Timestamp high
        0x00, // Timestamp low
        0x00, // Reserved
        0x00, // Cable number and code index
        midi_msg->status,
        midi_msg->data1,
        midi_msg->velocity};

    // Get current TX write pointer
    uint16_t tx_wr = (w5500_read_reg(W5500_S0_TX_WR, W5500_BSB_S0_REG) << 8) |
                     w5500_read_reg(W5500_S0_TX_WR + 1, W5500_BSB_S0_REG);

    // Write packet to TX buffer
    w5500_write_buf(tx_wr, W5500_BSB_S0_TX, packet, sizeof(packet));
    ESP_LOG_BUFFER_HEX(ETH_TAG, packet, sizeof(packet));

    // Update TX write pointer
    tx_wr += sizeof(packet);
    w5500_write_reg(W5500_S0_TX_WR, W5500_BSB_S0_REG, (tx_wr >> 8) & 0xFF);
    w5500_write_reg(W5500_S0_TX_WR + 1, W5500_BSB_S0_REG, tx_wr & 0xFF);

    // Send command
    w5500_write_reg(W5500_S0_CR, W5500_BSB_S0_REG, W5500_CR_SEND);

    ESP_LOGI(ETH_TAG, "Sent MIDI: Status=0x%02X, Note=%d, Velocity=%d",
             midi_msg->status, midi_msg->data1, midi_msg->velocity);
}

static void ethernet_task(void *param)
{
    midi_message_t midi_msg;

    while (1)
    {
        if (xQueueReceive(midi_queue, &midi_msg, portMAX_DELAY) == pdTRUE)
        {
            send_midi_over_ethernet(&midi_msg);
        }
    }
}

static void midi_usb_host_callback(usb_transfer_t *transfer)
{
    int size = (int)transfer->actual_num_bytes;
    int num_messages = size / MIDI_MESSAGE_LENGTH;
    int offset = 0;

    if (num_messages)
    {
        for (int i = 0; i < num_messages; i++)
        {
            uint8_t status = transfer->data_buffer[1 + offset];
            uint8_t velocity = transfer->data_buffer[3 + offset];

            // Ignore Active Sensing (0xFE)
            if (status == 0xFE)
            {
                offset += MIDI_MESSAGE_LENGTH;
                continue;
            }

            uint8_t msg_type = status & 0xF0;

            // Only allow Note On and Note Off
            if (msg_type != 0x90 && msg_type != 0x80)
            {
                offset += MIDI_MESSAGE_LENGTH;
                continue;
            }

            // Ignore Note On with velocity 0 (treated as Note Off)
            if (msg_type == 0x90 && velocity == 0)
            {
                offset += MIDI_MESSAGE_LENGTH;
                continue;
            }
            ESP_LOGI(DRIVER_TAG, "Received new MIDI data:");


            // Create MIDI message for ethernet transmission
            midi_message_t midi_msg = {
                .status = status,
                .data1 = transfer->data_buffer[2 + offset], // Note number
                .data2 = 0,                                 // Not used for note messages
                .velocity = velocity};

            // Send to ethernet task
            xQueueSend(midi_queue, &midi_msg, 0);

            // Print for debugging
            for (int j = 0; j < MIDI_MESSAGE_LENGTH; j++)
            {
                printf("%d ", transfer->data_buffer[j + offset]);
            }
            printf("\n");
            offset += MIDI_MESSAGE_LENGTH;
        }
    }

    ESP_ERROR_CHECK(usb_host_transfer_submit(transfer));
}

// Include your existing MIDI USB host functions here...
// (get_midi_interface_settings, client_event_cb, action_* functions)

static void get_midi_interface_settings(const usb_config_desc_t *usb_conf, interface_config_t *interface_conf)
{
    assert(usb_conf != NULL);
    assert(interface_conf != NULL);

    ESP_LOGI(DRIVER_TAG, "Getting interface config");

    int offset = 0;
    uint16_t wTotalLength = usb_conf->wTotalLength;
    const usb_standard_desc_t *next_desc = (const usb_standard_desc_t *)usb_conf;

    do
    {
        if (next_desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE)
        {
            usb_intf_desc_t *interface_desc = (usb_intf_desc_t *)next_desc;

            // check if there are >0 endpoints
            if (interface_desc->bNumEndpoints > 0)
            {
                // use interface
                interface_conf->interface_nmbr = interface_desc->bInterfaceNumber;
                interface_conf->alternate_setting = interface_desc->bAlternateSetting;

                printf("Interface Number: %d, Alternate Setting: %d \n", interface_conf->interface_nmbr, interface_conf->alternate_setting);
            }
        }
        if (next_desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT)
        {
            usb_ep_desc_t *ep_desc = (usb_ep_desc_t *)next_desc;
            if (USB_EP_DESC_GET_EP_DIR(ep_desc))
            {
                // endpoint is IN
                interface_conf->endpoint_address = ep_desc->bEndpointAddress;
                interface_conf->max_packet_size = ep_desc->wMaxPacketSize;
                printf("endpoint address: %d , mps: %d\n", interface_conf->endpoint_address, interface_conf->max_packet_size);
            }
        }

        next_desc = usb_parse_next_descriptor(next_desc, wTotalLength, &offset);

    } while (next_desc != NULL);
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event)
    {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        if (driver_obj->dev_addr == 0)
        {
            driver_obj->dev_addr = event_msg->new_dev.address;
            driver_obj->actions |= ACTION_OPEN_DEV;
        }
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        if (driver_obj->dev_hdl != NULL)
        {
            driver_obj->actions = ACTION_CLOSE_DEV;
        }
        break;
    default:
        abort();
    }
}

static void action_open_dev(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_addr != 0);
    ESP_LOGI(DRIVER_TAG, "Opening device at address %d", driver_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(driver_obj->client_hdl, driver_obj->dev_addr, &driver_obj->dev_hdl));
    // Get the device's information next
    driver_obj->actions &= ~ACTION_OPEN_DEV;
    driver_obj->actions |= ACTION_GET_DEV_INFO;
}

static void action_get_info(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(DRIVER_TAG, "Getting device information");
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    ESP_LOGI(DRIVER_TAG, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
    ESP_LOGI(DRIVER_TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
    // Todo: Print string descriptors

    // Get the device descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_INFO;
    driver_obj->actions |= ACTION_GET_DEV_DESC;
}

static void action_get_dev_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(DRIVER_TAG, "Getting device descriptor");
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc));
    usb_print_device_descriptor(dev_desc);
    // Get the device's config descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_DESC;
    driver_obj->actions |= ACTION_GET_CONFIG_DESC;
}

static void action_get_config_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(DRIVER_TAG, "Getting config descriptor");
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
    usb_print_config_descriptor(config_desc, NULL);

    // save interface number & alternative setting for later use
    interface_config_t interface_config = {0};
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
    get_midi_interface_settings(config_desc, &interface_config);

    driver_obj->interface_conf = interface_config;

    // Get the device's string descriptors next
    driver_obj->actions &= ~ACTION_GET_CONFIG_DESC;
    driver_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    if (dev_info.str_desc_manufacturer)
    {
        ESP_LOGI(DRIVER_TAG, "Getting Manufacturer string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_manufacturer);
    }
    if (dev_info.str_desc_product)
    {
        ESP_LOGI(DRIVER_TAG, "Getting Product string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_product);
    }
    if (dev_info.str_desc_serial_num)
    {
        ESP_LOGI(DRIVER_TAG, "Getting Serial Number string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_serial_num);
    }
    // Claim interface next
    driver_obj->actions &= ~ACTION_GET_STR_DESC;
    driver_obj->actions |= ACTION_CLAIM_INTERFACE;
}

static void action_claim_interface(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(DRIVER_TAG, "Claiming Interface");
    ESP_ERROR_CHECK(usb_host_interface_claim(
        driver_obj->client_hdl,
        driver_obj->dev_hdl,
        driver_obj->interface_conf.interface_nmbr,
        driver_obj->interface_conf.alternate_setting));

    driver_obj->actions &= ~ACTION_CLAIM_INTERFACE;
    driver_obj->actions |= ACTION_START_READING_DATA;
}

static void action_start_reading_data(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(DRIVER_TAG, "Configuring usb-transfer object");

    // configure usb-transfer object
    usb_transfer_t *transfer_obj;

    usb_host_transfer_alloc(1024, 0, &transfer_obj);

    transfer_obj->num_bytes = driver_obj->interface_conf.max_packet_size;
    transfer_obj->callback = midi_usb_host_callback;
    transfer_obj->bEndpointAddress = driver_obj->interface_conf.endpoint_address;
    transfer_obj->device_handle = driver_obj->dev_hdl;
    printf("set transfer parameters\n");
    ESP_ERROR_CHECK(usb_host_transfer_submit(transfer_obj));

    // Nothing to do until the device disconnects
    driver_obj->actions &= ~ACTION_START_READING_DATA;
}

static void action_close_dev(class_driver_t *driver_obj)
{
    ESP_LOGI(DRIVER_TAG, "Releasing interface");
    ESP_ERROR_CHECK(usb_host_interface_release(
        driver_obj->client_hdl,
        driver_obj->dev_hdl,
        driver_obj->interface_conf.interface_nmbr));
    ESP_LOGI(DRIVER_TAG, "Closing device");
    ESP_ERROR_CHECK(usb_host_device_close(driver_obj->client_hdl, driver_obj->dev_hdl));
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    // We need to exit the event handler loop
    driver_obj->actions &= ~ACTION_CLOSE_DEV;
    driver_obj->actions |= ACTION_EXIT;
}

void class_driver_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
    class_driver_t driver_obj = {0};

    // Wait until daemon task has installed USB Host Library
    xSemaphoreTake(signaling_sem, portMAX_DELAY);

    ESP_LOGI(DRIVER_TAG, "Registering Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = USB_CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *)&driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj.client_hdl));

    // classic state machine
    while (1)
    {
        if (driver_obj.actions == 0)
        {
            usb_host_client_handle_events(driver_obj.client_hdl, portMAX_DELAY);
        }
        else
        {
            if (driver_obj.actions & ACTION_OPEN_DEV)
            {
                action_open_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_INFO)
            {
                action_get_info(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_DESC)
            {
                action_get_dev_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_CONFIG_DESC)
            {
                action_get_config_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_STR_DESC)
            {
                action_get_str_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_CLAIM_INTERFACE)
            {
                action_claim_interface(&driver_obj);
            }
            if (driver_obj.actions & ACTION_START_READING_DATA)
            {
                action_start_reading_data(&driver_obj);
            }
            if (driver_obj.actions & ACTION_CLOSE_DEV)
            {
                action_close_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_EXIT)
            {
                driver_obj.actions = 0;
            }
        }
    }
}

static void host_lib_daemon_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;

    ESP_LOGI(DRIVER_TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    xSemaphoreGive(signaling_sem);
    vTaskDelay(10);

    while (1)
    {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            ESP_LOGI(DRIVER_TAG, "no clients available");
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            ESP_LOGI(DRIVER_TAG, "no devices connected");
        }
    }
}

void app_main(void)
{
    // Create MIDI message queue
    midi_queue = xQueueCreate(10, sizeof(midi_message_t));

    // Initialize W5500
    w5500_init_spi();
    vTaskDelay(pdMS_TO_TICKS(100));
    w5500_init_network();
    vTaskDelay(pdMS_TO_TICKS(100));
    w5500_socket_init();

    ESP_LOGI(ETH_TAG, "W5500 initialized successfully");

    // Create ethernet task
    xTaskCreatePinnedToCore(ethernet_task, "ethernet", 4096, NULL, 2, NULL, 1);

    // Create existing USB tasks
    SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();

    TaskHandle_t daemon_task_hdl;
    TaskHandle_t class_driver_task_hdl;

    xTaskCreatePinnedToCore(host_lib_daemon_task,
                            "daemon",
                            4096,
                            (void *)signaling_sem,
                            2,
                            &daemon_task_hdl,
                            0);

    xTaskCreatePinnedToCore(class_driver_task,
                            "class",
                            4096,
                            (void *)signaling_sem,
                            3,
                            &class_driver_task_hdl,
                            0);

    vTaskDelay(10);
}