/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include <driver/i2c.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "i2c-easy.h"
#include"mpu9250.h"


#define TAG "mpu9250"
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},
    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}
};


static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}


void run_imu(void)
{

  i2c_mpu9250_init(&cal);
  float temp;
  while (true)
  {
    vector_t va, vg, vm;

    // Get the Accelerometer, Gyroscope.
    //ESP_ERROR_CHECK(get_accel_gyro(&va, &vg));

    // Get the Accelerometer, Gyroscope and Magnetometer values.
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va,&vg,&vm));

    // Transform these values to the orientation of our device.
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);
    ESP_ERROR_CHECK(get_temperature_celsius(&temp));

    ESP_LOGI(TAG, "Accel X: %2.3f, Accel Y: %2.3f, Accel Z: %2.3f, Temp %2.3f°C", va.x, va.y, va.z, temp);
    ESP_LOGI(TAG, "Gyro X: %2.3f, Gyro Y: %2.3f, Gyro Z: %2.3f, Temp %2.3f°C", vg.x, vg.y, vg.z, temp);
    ESP_LOGI(TAG, "Magn X: %2.3f, Magn Y: %2.3f, Magn Z: %2.3f, Temp %2.3f°C", vm.x, vm.y, vm.z, temp);
    vTaskDelay(200 / portTICK_RATE_MS);
  }
}

static void imu_task(void *arg)
{

#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
#else
  run_imu();
#endif

  // Exit
  vTaskDelay(100 / portTICK_RATE_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}

void app_main(void){
      // initialize the I2C bus
    ESP_LOGI(TAG, "Main Task");
    xTaskCreate(imu_task, "imu_task", 2048, NULL, 10, NULL);
}
