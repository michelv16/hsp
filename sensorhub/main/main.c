/*------------------------------------------------------------------------------

	Sensorhub implementation for ESP32

	Michel Verlaan

	Reads sensors and publishes information to MQTT broker

---------------------------------------------------------------------------------*/

#include <stdio.h>
#include <memory.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "DHT.h"
#include "ccs811.h"

#define ESP_PLATFORM     1
#define TASK_STACK_DEPTH 2048
#define I2C_BUS          0
#define I2C_SCL_PIN      22
#define I2C_SDA_PIN      21
#define I2C_FREQ         I2C_FREQ_100K
#define DHT22_DATA_PIN   GPIO_NUM_4

static const char *TAG = "sensorhub";
static ccs811_sensor_t* sensor;
QueueHandle_t data_queue;

uint8_t sensor_errors;

struct sensor_data {
	uint16_t tvoc;
	uint16_t eco2;
	float    tmp;
	float    hum;
};

void setup_system()
{
	sensor_errors = 0;
    uart_set_baud(0, 115200);

    // CCS811
    // init all I2C bus interfaces at which CCS811 sensors are connected
    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

    // init the sensor with slave address CCS811_I2C_ADDRESS_1 connected I2C_BUS.
    sensor = ccs811_init_sensor (I2C_BUS, CCS811_I2C_ADDRESS_1);

    if (!sensor)
    {
    	sensor_errors |= 0x01;
    	ESP_LOGE(TAG, "Could not initialize CCS811 sensor \n");
    }

    // DHT22
    setDHTgpio(DHT22_DATA_PIN);

    // MQTT


    vTaskDelay(1);
}



void read_sensor_task(void *pvParameter)
{
	struct sensor_data data;

	data_queue = xQueueCreate(10, sizeof(data));

	memset(&data, 0, sizeof(data));

	if (data_queue == NULL)
	{
		ESP_LOGE(TAG, "Failed to create sensor data queue\n");
	}

	if (sensor_errors != 0x01)
	{
		// start periodic measurement with one measurement per second
		ccs811_set_mode (sensor, ccs811_mode_1s);
	}

    ESP_LOGI(TAG, "Starting reading sensor data Task\n");

    while (1)
    {
    	int ret = 0;

    	if (sensor_errors != 0x01)
    	{
        	// be aware, return value of ccs811_get_results is 1 on success
            ret = ccs811_get_results (sensor, &data.tvoc, &data.eco2, 0, 0);
            if (!ret)
            {
            	ESP_LOGE(TAG, "Failed reading CCS811 sensor \n");
            	ESP_LOGI(TAG, "ret %d \n", ret);
            }
    	}

        ret = readDHT();
        if (!ret)
        {
        	data.tmp = getTemperature();
        	data.hum = getHumidity();
        }
        else
        {
            errorHandler(ret);
        }

        ESP_LOGI(TAG, "Sending to queue\n");
        xQueueSend(data_queue, &data, portMAX_DELAY);

        // -- wait at least 2 sec before reading again ------------
        // The interval of whole process must be beyond 2 seconds !!
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void get_sensor_task(void *pvParameter)
{
	struct sensor_data data;

	while (1)
	{
		if (xQueueReceive(data_queue, &data, portMAX_DELAY))
		{
			ESP_LOGI(TAG, "Receiving from queue:\n");
			ESP_LOGI(TAG, "TVOC: %d eCO2: %d\n", data.tvoc, data.eco2);
			ESP_LOGI(TAG, "Hum: %.1f Tmp: %.1f\n", data.hum, data.tmp);
		}

		vTaskDelay(2000 / portTICK_RATE_MS);
	}
}

void app_main(void)
{
	setup_system();

    // create a periodic task that reads the sensors data
    xTaskCreate(read_sensor_task, "read_sensor_task", TASK_STACK_DEPTH, NULL, 2, NULL);

    xTaskCreate(get_sensor_task, "get_sensor_task", TASK_STACK_DEPTH, NULL, 2, NULL);

    esp_log_level_set("*", ESP_LOG_INFO);
}
