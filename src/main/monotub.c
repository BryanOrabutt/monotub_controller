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

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define I2C_MASTER_SCL_IO    22  // Assign to your SCL pin
#define I2C_MASTER_SDA_IO    21  // Assign to your SDA pin
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000

#define SCD41_TIMEOUT_MS    1000
#define SCD41_MEASURE_PERIOD_MS    5000
#define SCD41_ADDRESS        0x62  // SCD41 I2C address

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xff


typedef struct
{
    i2c_master_dev_handle_t* device_handle;
    int ready;
} scd41_cal_t;

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static uint8_t s_led_state = 0;
static uint16_t temp_off = 0;

//static QueueHandle_t sensor_queue;
//static QueueHandle_t tx_queue;
//static QueueHandle_t tx_queue;

//create task handles for manipulating tasks via RTOS functions.
static TaskHandle_t blink_task_h, scd41_read_task_h, scd41_cal_h;

/* 8-bit CRC alogirthm or sensirion devices.
 * data: pointer to data to generate CRC
 * count: number of bytes in the data buffer.
 *
 * returns the 8-bit CRC of the data in the buffer.
*/
uint8_t sensirion_common_generate_crc(const uint8_t* data, uint16_t count)
{
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte)
    {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/* Wifi event handler provided by esp-idf */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    const char* TAG = "wifi-event";
    //If there is a WIFI event and it's a startion start event, attempt to connect to wifi.
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        esp_wifi_connect();
    } 
    //if the event is a disconnection event attempt to reconnect
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAXIMUM_RETRY) 
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } 
        //max retries exceeded. assert fail bit and restart.
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } 
    //station is able to connect to wifi, get the IP and reset the retry counter. Set station connected bit
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* Initialize the Wifi station. Provided by esp-idf */
void wifi_init_sta(void)
{
    const char* TAG = "wifi-init";

    // Create an event group for Wi-Fi events
    s_wifi_event_group = xEventGroupCreate();

    // Initialize the network interface (for Wi-Fi driver)
    ESP_ERROR_CHECK(esp_netif_init());

    // Create the default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create a default Wi-Fi station network interface
    esp_netif_create_default_wifi_sta();

    // Get default Wi-Fi configuration parameters
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register Wi-Fi event handler
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

    // Define Wi-Fi connection credentials
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
    // Set Wi-Fi mode to station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    // Set Wi-Fi configuration
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    // Start Wi-Fi connection
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

/* 1 Hz heartbeat LED */
static void blink_task(void *pvParameter)
{
    const char* TAG = "blink";
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    while(1)
    {
        ESP_LOGI(TAG, "If you can see this, FreeRTOS is still working.");
        gpio_set_level(BLINK_GPIO, s_led_state);
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/* GPIO configuration for on board LED */
static void configure_led(void)
{
    const char* TAG = "led-conf";
    ESP_LOGI(TAG, "Configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}


/* Configuration for ESP I2C periperal
 * sets ESP32 as I2C master
 */
static i2c_master_bus_handle_t* configure_i2c_master(void)
{
    const char* TAG = "i2c-conf";
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


    static i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    ESP_LOGI(TAG, "ESP32 successfully configured as I2C master.");

    return &bus_handle;

}

/* Cofiguration for SCD41 I2C slave
 * conifigres a handle for a I2C slave using the SCD41's address
 */
void scd41_init(i2c_master_bus_handle_t* bh, i2c_master_dev_handle_t* scd41_handle)
{
    const char* TAG = "scd41-init";
    i2c_master_bus_handle_t bus_handle = *bh;

    ESP_LOGI(TAG, "Initializing SCD41 as I2C slave.");

    // Initialize I2C device
    i2c_device_config_t i2c_dev_cfg =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD41_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_dev_cfg, scd41_handle));
    ESP_LOGI(TAG, "SCD41 initialized ad I2C slave.");
}

/* Calibration task for the SCD41.
 * Waits 50 ms for the SCD41 to finish power up procedures, then does the following:
 * 1) Put the SCD41 into idle mode by sending the command to stop periodic measurements.
 * 2) Sets the altitude to correspond to the elevation in STL as per wiki.
 * 3) Reads the temperature offset used in RH calculation so it can be removed from the temperature
 *    before doing any calculations with it
 * 4) Waits in idle loop until main task deltes this calibration task and begins measurements.
 */
void scd41_cal(void* pvParameters)
{
    const char* TAG = "scd41-cal";
    uint8_t data[9];
    esp_err_t ret;
    i2c_master_dev_handle_t scd41_handle = *((scd41_cal_t*)pvParameters)->device_handle;

    vTaskDelay(50 / portTICK_PERIOD_MS);

    uint8_t stop_periodic_measurement_cmd[] = {0x3f, 0x86};

    //1: put SCD41 in idle mode.
    ESP_ERROR_CHECK(i2c_master_transmit(scd41_handle, stop_periodic_measurement_cmd, sizeof(stop_periodic_measurement_cmd), SCD41_TIMEOUT_MS));
    vTaskDelay(500 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Calibrating SCD41 for accurate measurement.");
    ESP_LOGI(TAG, "Setting altitude...");

    //2: Set altitude.
    uint8_t set_altitude_packet[] = {0x24, 0x27, 0x00, 0x8e, 0x00};
    uint8_t crc = sensirion_common_generate_crc(set_altitude_packet+2, 2);
    set_altitude_packet[4] = crc;

    ESP_ERROR_CHECK(i2c_master_transmit(scd41_handle, set_altitude_packet, sizeof(set_altitude_packet), SCD41_TIMEOUT_MS));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "...done!");
    ESP_LOGI(TAG, "Reading back altitude from SCD41...");

    uint8_t get_altitude_packet[] = {0x23, 0x22};
    ESP_ERROR_CHECK(i2c_master_transmit(scd41_handle, get_altitude_packet, sizeof(get_altitude_packet), SCD41_TIMEOUT_MS));

    ret = i2c_master_receive(scd41_handle, data, 3, SCD41_TIMEOUT_MS);

    uint16_t altitude;
    if (ret == ESP_OK)
    {
        altitude = (data[0] << 8) | data[1];
        ESP_LOGI(TAG, "...altitude set to %dm.", altitude);
    }
    else
    {
        ESP_LOGE(TAG, "...failed to read altitude back.");
    }

    ESP_LOGI(TAG, "Reading temperature offset from SCD41...");

    //3: get temp offset
    uint8_t get_temp_offset_packet[] = {0x23, 0x18};
    ESP_ERROR_CHECK(i2c_master_transmit(scd41_handle, get_temp_offset_packet, sizeof(get_temp_offset_packet), SCD41_TIMEOUT_MS));

    ret = i2c_master_receive(scd41_handle, data, 3, SCD41_TIMEOUT_MS);
    if (ret == ESP_OK)
    {
        temp_off = ((float)((data[0] << 8) | data[1]))/65535.0f*175;
        ESP_LOGI(TAG, "...temperature offset set to %d °C.", temp_off);
    }
    else
    {
        ESP_LOGE(TAG, "...failed to read temperature offset back.");
    }

    ((scd41_cal_t*)pvParameters)->ready = 1;

    //4: idle wait
    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/* Reads the SCD41 measurements every 5 seconds (min measure period) */
void scd41_read_task(void *pvParameters)
{
    const char* TAG = "scd41-read";
    uint8_t data[9];
    esp_err_t ret;

    i2c_master_dev_handle_t scd41_handle = *((i2c_master_dev_handle_t*)pvParameters);

    // Command to start measurement in periodic mode
    uint8_t start_measurement_cmd[] = {0x21, 0xB1};

    ESP_ERROR_CHECK(i2c_master_transmit(scd41_handle, start_measurement_cmd, sizeof(start_measurement_cmd), SCD41_TIMEOUT_MS));

    while (1)
    {
        // Delay before the read
        vTaskDelay(SCD41_MEASURE_PERIOD_MS / portTICK_PERIOD_MS); // Adjust based on desired read interval

        // Command to read measurement
        uint8_t read_measurement_cmd[] = {0xEC, 0x05};
        ESP_ERROR_CHECK(i2c_master_transmit(scd41_handle, read_measurement_cmd, sizeof(read_measurement_cmd), SCD41_TIMEOUT_MS));

        // Read measurement data
        ret = i2c_master_receive(scd41_handle, data, sizeof(data), SCD41_TIMEOUT_MS);
        if (ret == ESP_OK)
        {
            // Data format: 9-bytes. b0b1 b3 b4b5 b6 b7b8 b9. Where bx = byte X
            // CO2 in ppm = b0b1
            // Temperature in Celsius = -45 + 175*b4b5/(2^16 - 1)
            // % Rel Humidity = 100*b7b8/(2^16 - 1)
            // CRC bytes = b3, b6, b9
            uint16_t co2 = (data[0] << 8) | data[1];
            float temperature = ((float)((data[3] << 8) | data[4])) / 65535.0f * 175 - 45 - temp_off;
            float humidity = ((float)((data[6] << 8) | data[7])) / 65535.0f * 100;

            ESP_LOGI(TAG, "CO2: %d ppm, Temperature: %.2f °C, Humidity: %.2f%%", co2, temperature, humidity);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read data");
        }
    }

    // Clean up
    i2c_master_bus_rm_device(scd41_handle);
}

void app_main(void) 
{
    //Init nvs for wifi configuration
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //configure periperals and interfaces.
    wifi_init_sta();
    configure_led();
    i2c_master_bus_handle_t* bus_handle = configure_i2c_master();

    i2c_master_dev_handle_t dev_handle;
    scd41_init(bus_handle, &dev_handle);

    scd41_cal_t scd41_cal_data = {&dev_handle, 0};

    //Start the heartbeat and SCD41 calibration tasks
    xTaskCreate(&blink_task, "blink_task", 4096, NULL, 5, &blink_task_h);
    xTaskCreate(&scd41_cal, "scd41_cal", 4096, (void*)&scd41_cal_data, 5, &scd41_cal_h);

    //Wait for SCD41 to be calibrated.
    while(!scd41_cal_data.ready);
    vTaskDelete(scd41_cal_h); //remove calibration task from run queue as it's no longer needed
    //start measurement task
    xTaskCreate(&scd41_read_task, "scd41_read_task", 4096, (void*)&dev_handle, 5, &scd41_read_task_h);
}
