#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"

// Use the Kconfig settings
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD

#define BLINK_GPIO 2
#define MAXIMUM_RETRY 10

#define I2C_MASTER_SCL_IO    22  // Assign to your SCL pin
#define I2C_MASTER_SDA_IO    21  // Assign to your SDA pin
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000

#define SCD41_TIMEOUT_MS    1000
#define SCD41_ADDRESS        0x62  // SCD41 I2C address

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static uint8_t s_led_state = 0;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    const char* TAG = "wifi-event-handler";
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAXIMUM_RETRY) 
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } 
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    const char* TAG = "wifi-init";
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = 
    {
        .sta = 
        {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void blink_task(void *pvParameter)
{
    const char* TAG = "blink";
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    while(1)
    {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        gpio_set_level(BLINK_GPIO, s_led_state);
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void configure_led(void)
{
    const char* TAG = "LED-config";
    ESP_LOGI(TAG, "Configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void scd41_task(void *pvParameters)
{
    const char* TAG = "sdc41";
    uint8_t data[9];
    esp_err_t ret;

    ESP_LOGI(TAG, "Configuring ESP32 as I2C master.");

    i2c_master_bus_config_t i2c_mst_config =
    {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };


    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    ESP_LOGI(TAG, "ESP32 successfully configured as I2C master.");
    ESP_LOGI(TAG, "Initializing SCD41 as I2C slave.");

    // Initialize I2C device
    i2c_device_config_t i2c_dev_cfg =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD41_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    i2c_master_dev_handle_t scd41_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_dev_cfg, &scd41_handle));
    ESP_LOGI(TAG, "SCD41 initialized ad I2C slave.");

    while (1)
    {
        // Command to start measurement in periodic mode
        uint8_t start_measurement_cmd[] = {0x21, 0xB1};

        ESP_ERROR_CHECK(i2c_master_transmit(scd41_handle, start_measurement_cmd, sizeof(start_measurement_cmd), SCD41_TIMEOUT_MS));

        // Delay for measurement to be ready
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // SCD41 measurement interval

        // Command to read measurement
        uint8_t read_measurement_cmd[] = {0xEC, 0x05};
        ESP_ERROR_CHECK(i2c_master_transmit(scd41_handle, read_measurement_cmd, sizeof(read_measurement_cmd), SCD41_TIMEOUT_MS));

        // Read measurement data
        ret = i2c_master_receive(scd41_handle, data, sizeof(data), SCD41_TIMEOUT_MS);
        if (ret == ESP_OK)
        {
            // Data format: 9-bytes. b0b1 b3 b4b5 b6 b7b8 b9
            // b0b1 = CO2 in ppm
            // b4b5 = Temperatue = -45 + 175*b4b5/(2^16 - 1)
            // b7b8 = Rel Humidity = 100*b7b8/(2^16 - 1)
            // b3 b6 b9 = CRC bytes
            uint16_t word0 = (data[0] << 8) | data[1];
            uint16_t word1 = (data[3] << 8) | data[4];
            uint16_t word2 = (data[6] << 8) | data[7];

            uint16_t co2 = word0;
            float temperature = (float)word1 / 65536.0f * 175 - 45;
            float humidity = (float)word2 / 65536.0f * 100;

            ESP_LOGI(TAG, "CO2: %d ppm, Temperature: %.2f °C, Humidity: %.2f%%", co2, temperature, humidity);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read data");
        }

        // Delay before the next read
        vTaskDelay(10000 / portTICK_PERIOD_MS); // Adjust based on desired read interval
    }

    // Clean up
    i2c_master_bus_rm_device(scd41_handle);
}

void app_main(void) 
{
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    /* Configure the peripheral according to the LED type */
    configure_led();

    xTaskCreate(&blink_task, "blink_task", 4096, NULL, 5, NULL);
    xTaskCreate(&scd41_task, "scd41_task", 4096, NULL, 5, NULL);
}
