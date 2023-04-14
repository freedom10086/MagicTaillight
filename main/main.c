#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "driver/i2c.h"
#include "bh1750.h"
#include "LIS3DH.h"
#include "ble.h"
#include "vl53l1x/vl53l1_api.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO           7
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          200000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */

#define VL53L1X_GPIO 0
#define VL53L1X_XSHUT 2
#define STORAGE_NAMESPACE "vl53l1Data"

//#define CALIBRATION_VL53L1 1

static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void nvs_calib_vl53l1_reset() //clear flag
{
    nvs_handle my_handle;
    esp_err_t err;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    ESP_ERROR_CHECK(err);
    err = nvs_erase_key(my_handle, "calibrationData");
    ESP_ERROR_CHECK(err);
    err = nvs_commit(my_handle);
    ESP_ERROR_CHECK(err);
    // Close
    nvs_close(my_handle);

    //restart esp32
    esp_restart();
}

static esp_err_t nvs_set_get_vl53l1(VL53L1_CalibrationData_t *calib_data, bool is_set)   //set: 1  get:0
{

    nvs_handle my_handle;
    esp_err_t err;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    ESP_ERROR_CHECK(err);
    size_t calib_data_size = (size_t)sizeof(VL53L1_CalibrationData_t);
    if (is_set)
    {

        //store calibrationData
        char *pData = malloc(calib_data_size + sizeof(uint8_t));
        memcpy(pData, calib_data, calib_data_size);
        err = nvs_set_blob(my_handle, "calibrationData", pData, calib_data_size);
        ESP_ERROR_CHECK(err);
        err = nvs_commit(my_handle);
        ESP_ERROR_CHECK(err);
        ESP_LOGI(TAG, "calibrationData saved ! ");
        free(pData);
    }
    else
    {
        err = nvs_get_blob(my_handle, "calibrationData", calib_data, &calib_data_size);
        // ESP_ERROR_CHECK(err);
    }

    // Close
    nvs_close(my_handle);
    return err;
}

/*Calibration  vl53l1 module*/
static VL53L1_CalibrationData_t vl53l1_calibration(VL53L1_Dev_t *dev)
{
    int status;
    int32_t targetDistanceMilliMeter = 703;
    VL53L1_CalibrationData_t calibrationData;
    //status = VL53L1_WaitDeviceBooted(dev);
    //status = VL53L1_DataInit(dev);                                       //performs the device initialization
    //status = VL53L1_StaticInit(dev);                                     // load device settings specific for a given use case.
    status = VL53L1_SetPresetMode(dev,VL53L1_PRESETMODE_AUTONOMOUS);
    status = VL53L1_PerformRefSpadManagement(dev);
    status = VL53L1_PerformOffsetCalibration(dev,targetDistanceMilliMeter);
    status = VL53L1_PerformSingleTargetXTalkCalibration(dev,targetDistanceMilliMeter);
    status = VL53L1_GetCalibrationData(dev,&calibrationData);

    if (status)
    {
        ESP_LOGE(TAG, "vl53l1_calibration failed \n");
        calibrationData.struct_version = 0;
        return calibrationData;

    }else
    {
        ESP_LOGI(TAG, "vl53l1_calibration done ! version = %lu \n",calibrationData.struct_version);
        return calibrationData;
    }

}

/*init  vl53l1 module*/
static void vl53l1_init(VL53L1_Dev_t *dev) {
    VL53L1_UserRoi_t Roi0;
    Roi0.TopLeftX = 0; //set ROI according to requirement
    Roi0.TopLeftY = 15;
    Roi0.BotRightX = 15;
    Roi0.BotRightY = 0;
    int status = VL53L1_WaitDeviceBooted(dev);
    status = VL53L1_DataInit(dev);                                       //performs the device initialization
    status = VL53L1_StaticInit(dev);                                     // load device settings specific for a given use case.

#ifdef CALIBRATION_VL53L1
    VL53L1_CalibrationData_t calibData;
    esp_err_t err = nvs_set_get_vl53l1(&calibData, 0);
    if (err == ESP_OK)
    {
        status = VL53L1_SetCalibrationData(dev, &calibData);
        if (status != VL53L1_ERROR_NONE)
        {
            ESP_LOGE(TAG, "VL53L1_SetCalibrationData failed \n");
        }
        else
        {
            ESP_LOGI(TAG, "VL53L1_SetCalibrationData done  \n");
        }
    }
    else
    {
        //wait for calibration
        int8_t delay_prepare_for_calibration = 5;
        for (int8_t i = delay_prepare_for_calibration; i > 0; i--)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "prepare_for_calibration %d \n", i);
        }

        calibData = vl53l1_calibration(dev);
        nvs_set_get_vl53l1(&calibData, 1);

        //restart esp32
        esp_restart();

    }
#endif

    status = VL53L1_SetDistanceMode(dev,VL53L1_DISTANCEMODE_LONG);      //Max distance in dark:Short:136cm Medium:290cm long:360cm
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev,160000);
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(dev,200);     //period of time between two consecutive measurements
    status = VL53L1_SetUserROI(dev, &Roi0); //SET region of interest
    status = VL53L1_StartMeasurement(dev);

    if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53L1_StartMeasurement failed \n");

    } else {
        ESP_LOGI(TAG, "VL53L1 StartMeasurement  \n");
    }
}

void app_main(void) {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);


    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "i2c init success %d", I2C_MASTER_NUM);

    // xshut high
    gpio_set_direction(VL53L1X_XSHUT, GPIO_MODE_OUTPUT); //
    gpio_set_level(VL53L1X_XSHUT, 1);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    VL53L1_Dev_t vl53l1_dev;
    vl53l1_dev.i2c_num = I2C_MASTER_NUM;
    vl53l1_dev.I2cDevAddr = (0x52 >> 1);
    uint8_t byteData;
    uint16_t wordData;
    VL53L1_RdByte(&vl53l1_dev, 0x010F, &byteData);
    ESP_LOGI(TAG, "VL53L1X Model_ID: %02X\n\r", byteData);
    VL53L1_RdByte(&vl53l1_dev, 0x0110, &byteData);
    ESP_LOGI(TAG, "VL53L1X Module_Type: %02X\n\r", byteData);
    VL53L1_RdWord(&vl53l1_dev, 0x010F, &wordData);
    ESP_LOGI(TAG, "VL53L1X: %02X\n\r", wordData);

    vl53l1_init(&vl53l1_dev);

    VL53L1_RangingMeasurementData_t pRangingMeasurementData;
    while (1) {
        int status = VL53L1_WaitMeasurementDataReady(&vl53l1_dev); //5 HZ
        if (status == VL53L1_ERROR_NONE) {
            status = VL53L1_GetRangingMeasurementData(&vl53l1_dev, &pRangingMeasurementData);
            if (status == 0 && pRangingMeasurementData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID) {
                ESP_LOGI(TAG, "height %3.1fcm \n", pRangingMeasurementData.RangeMilliMeter * 1.0f / 10);
            } else {

                ESP_LOGW(TAG, "height %3.1fcm %d %d level:%d\n", pRangingMeasurementData.RangeMilliMeter * 1.0f / 10,
                         status, pRangingMeasurementData.RangeStatus, pRangingMeasurementData.RangeQualityLevel);
            }

            status = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1_dev); //clear Interrupt start next measurement
            vTaskDelay(pdMS_TO_TICKS(300));
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

