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

static const char *TAG = "example";


#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      6
#define LED_NUMBERS         20
#define EXAMPLE_CHASE_SPEED_MS      10

#define I2C_MASTER_SCL_IO           7
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */

static uint8_t led_strip_pixels[LED_NUMBERS * 3];
static uint8_t s_led_state = 0;
static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        for (int i = 0; i < LED_NUMBERS; ++i) {
            led_strip_set_pixel(led_strip, i, 16, 16, 16);
        }
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .max_leds = LED_NUMBERS, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
        case 0:
            *r = rgb_max;
            *g = rgb_min + rgb_adj;
            *b = rgb_min;
            break;
        case 1:
            *r = rgb_max - rgb_adj;
            *g = rgb_max;
            *b = rgb_min;
            break;
        case 2:
            *r = rgb_min;
            *g = rgb_max;
            *b = rgb_min + rgb_adj;
            break;
        case 3:
            *r = rgb_min;
            *g = rgb_max - rgb_adj;
            *b = rgb_max;
            break;
        case 4:
            *r = rgb_min + rgb_adj;
            *g = rgb_min;
            *b = rgb_max;
            break;
        default:
            *r = rgb_max;
            *g = rgb_min;
            *b = rgb_max - rgb_adj;
            break;
    }
}

static esp_err_t i2c_master_init(void)
{
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


void led_strip_test() {
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
            .gpio_num = RMT_LED_STRIP_GPIO_NUM,
            .mem_block_symbols = 64, // increase the block size can make the LED less flickering
            .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
            .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
            .loop_count = 0, // no transfer loop
    };
    while (1) {
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < LED_NUMBERS; j += 3) {
                // Build RGB pixels
                hue = j * 360 / LED_NUMBERS + start_rgb;
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                led_strip_pixels[j * 3 + 0] = green;
                led_strip_pixels[j * 3 + 1] = blue;
                led_strip_pixels[j * 3 + 2] = red;
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
}

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    //led_strip_test();

//    uint8_t data[2];
//    ESP_ERROR_CHECK(i2c_master_init());
//    ESP_LOGI(TAG, "I2C initialized successfully");
//
//    light_sensor_reset();
//    ESP_LOGI(TAG, "light sensor reset");
//    vTaskDelay(pdMS_TO_TICKS(20));
//    light_sensor_on();
//
//    while(1) {
//        light_sensor_on();
//        light_sensor_start(CONTINUE_HRES_MODE);
//        // wait 180ms
//        vTaskDelay(pdMS_TO_TICKS(180));
//        uint16_t light = light_sensor_read_light();
//
//        ESP_LOGI(TAG, "light = %d", light);
//
//        uint8_t lis3dh_status = lis3dh_read_status();
//        ESP_LOGI(TAG, "lisd3h status = %x", lis3dh_status);
//    }

    ble_start();
}

