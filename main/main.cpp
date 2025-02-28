
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <unistd.h>
#include <string.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_dsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/dsp/image/processing.hpp"

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
#define PIN_NUM_CS    static_cast<gpio_num_t>(21)

#define MOUNT_POINT "/sdcard"

#define INPUT_WIDTH 350
#define INPUT_HEIGHT 350

#define INFERENCE_TASK_STACK_SIZE (16*1024) // 16KB stack

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

    // Grayscale for FOMO model 
    .pixel_format = PIXFORMAT_GRAYSCALE, //YUV422,GRAYSCALE,RGB565,JPEG
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
    .frame_size = FRAMESIZE_VGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    // .jpeg_quality = 4, //Note used for grayscale. 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

void print_inference_result(ei_impulse_result_t result) {
    // Print how long it took to perform inference
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
}

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

void inference_task(void* pvParameters) {
    camera_fb_t *pic = (camera_fb_t*)pvParameters;

    // // Allocate memory for the image array to stores the grayscale image
    uint8_t *image_data = (uint8_t*)calloc(INPUT_WIDTH * INPUT_HEIGHT, sizeof(uint8_t));
    if (!image_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for image_data");
        esp_camera_fb_return(pic);
        return;
    }

    // Print available memory before inference
    ESP_LOGI("MEMORY", "Free SPIRAM: %d bytes", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    ESP_LOGI("MEMORY", "Free Internal RAM: %d bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    // Resize the image to the desired dimensions
    ei::image::processing::resize_image_using_mode(
        pic->buf,
        pic->width,
        pic->height,
        image_data,
        INPUT_WIDTH,
        INPUT_HEIGHT,
        1, // Grayscale image only has 1 channel
        // Crop the image to the shortest side and resize to the desired dimensions
        EI_CLASSIFIER_RESIZE_FIT_SHORTEST);

    // free up pic
    esp_camera_fb_return(pic);

    // Allocate memory for the features array, which is the normalized pixel values
    signal_t features_signal;
    features_signal.total_length = INPUT_WIDTH * INPUT_HEIGHT;
    features_signal.get_data = [&image_data](size_t offset, size_t length, float *out_ptr) -> int {
        for (size_t i = 0; i < length; i++) {
            // Note that signal data is a RGB value, so we need to reconstruct the RGB value from the grayscale value
            // Combine R, G, B values into a single hex color (R = G = B for grayscale)
            out_ptr[i] = (image_data[offset + i] << 16) | (image_data[offset + i] << 8) | image_data[offset + i];
        }

        return ESP_OK;
    };

    ei_printf("Edge Impulse standalone inferencing (Espressif ESP32)\n");
    // You also need to define the result variable before using it
    ei_impulse_result_t result = { 0 };
    // invoke the impulse
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
    }
    print_inference_result(result);

    // Delete the task when it's done
    vTaskDelete(NULL); // THIS IS THE CRITICAL LINE
}

extern "C" void app_main(void)
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

    esp_err_t ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = (spi_host_device_t)host.slot;

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
    const char *path = MOUNT_POINT"/photo.pgm";
    FILE *file = fopen(path, "w");
    if (!file) {
        ESP_LOGE("SD", "Failed to open file for writing");
        esp_camera_fb_return(pic);
        return;
    }

    // Write PGM header
    fprintf(file, "P5\n%d %d\n255\n", pic->width, pic->height);
    
    // Write buffer to SD card in chunks
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
    
    ESP_LOGI("Camera", "Image size: %zu bytes", pic->len);
    ESP_LOGI("Camera", "Image saved to %s", path);

    // Create a task with sufficient stack for inference
    xTaskCreate(inference_task, "inference_task", INFERENCE_TASK_STACK_SIZE, (void*) pic, tskIDLE_PRIORITY + 1, NULL);

    while(true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
