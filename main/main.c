
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <unistd.h>
#include <string.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"
#include "camera_pins.h"

static const char *TAG = "mist_cam";

// SD SPI pin configuration for the XIAO esp32s3 sense board
#define PIN_NUM_MISO  8
#define PIN_NUM_MOSI  9
#define PIN_NUM_CLK   7
#define PIN_NUM_CS    21

#define MOUNT_POINT "/sdcard"

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    /**
    FRAMESIZE_QQVGA - 160x120
    FRAMESIZE_QVGA - 320x240
    FRAMESIZE_CIF - 400x296
    FRAMESIZE_VGA - 640x480
    FRAMESIZE_SVGA - 800x600
    FRAMESIZE_XGA - 1024x768
    FRAMESIZE_HD - 1280x720
    FRAMESIZE_SXGA - 1280x1024
    FRAMESIZE_UXGA - 1600x1200
        */
    .frame_size = FRAMESIZE_SXGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 1, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

void app_main(void)
{
    if(ESP_OK != init_camera()) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }

    // Initialize SD card
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    // Mount SD card
    sdmmc_card_t *card;
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE("SD", "Failed to mount SD card (%s)", esp_err_to_name(ret));
        return;
    }

    // Take a picture
    camera_fb_t *pic = esp_camera_fb_get();
    if (!pic) {
        ESP_LOGE("Camera", "Camera capture failed");
        return;
    }

    // Save the image to the SD card
    const char *path = MOUNT_POINT"/photo.jpg";
    FILE *file = fopen(path, "w");
    if (!file) {
        ESP_LOGE("SD", "Failed to open file for writing");
        esp_camera_fb_return(pic);
        return;
    }

    
    // Write JPEG buffer to SD card in chunks
    size_t chunk_size = 1024; // Adjust the chunk size if necessary
    size_t offset = 0;
    while (offset < pic->len) {
        size_t to_write = MIN(chunk_size, pic->len - offset);
        size_t written = fwrite(pic->buf + offset, 1, to_write, file);
        if (written != to_write) {
            ESP_LOGE("SD", "Failed to write complete chunk to file");
            fclose(file);
            esp_camera_fb_return(pic);
            return;
        }
        offset += written;
    }

    fclose(file); //Close file after writing all chunks
    fflush(file); // Ensure all data is flushed to the file

    // vTaskDelay(4000 / portTICK_RATE_MS); // Delay to ensure the file is written
    

    ESP_LOGI("Camera", "Image size: %zu bytes", pic->len);
    ESP_LOGI("Camera", "Image saved to %s", path);


    // Clean up
    esp_camera_fb_return(pic);
}
