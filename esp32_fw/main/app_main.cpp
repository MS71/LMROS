// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <exception>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "../components/http_server/my_http_server.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ota_server.h"

#include <esp_task_wdt.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#ifdef CONFIG_IDF_TARGET_ESP32S2
#include <driver/adc_common.h>
#endif
#include "driver/gpio.h"
#include "driver/i2c.h"
#ifndef CONFIG_IDF_TARGET_ESP32S2
#include "driver/sdmmc_host.h"
#endif
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_event_legacy.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#ifdef CONFIG_ENABLE_SPIFS
#include "esp_spiffs.h"
#endif
#include "driver/rtc_io.h"
#include "esp32/rom/uart.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "i2chandler.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#if CONFIG_IDF_TARGET_ESP32S2
#include "driver/temp_sensor.h"
#endif
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "ssd1306.h"
#include "ssd1306_default_if.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"

#include "cam.h"
#include "gps.h"
#ifdef CONFIG_ENABLE_ROS2
#include "ros2node.h"
#endif
#ifdef CONFIG_ENABLE_WEBUI
#include "webui.h"
#endif

#include "console.h"

#ifdef CONFIG_ESP32S2_ULP_COPROC_ENABLED
#include "esp32/ulp.h"
#include "ulp_main.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");
#endif

void beep(int onoff);

// static esp_err_t event_handler(void* ctx, system_event_t* event);
void check_wifi_config();
void initialise_wifi(void);

// static char my_wifi_ssid[64] = {};
// static char my_wifi_psk[64] = {};
// static bool my_wifi_save_on_connected = false;

#ifdef CONFIG_ENABLE_SDCARD
sdmmc_card_t* card = NULL;
bool sdcard_ready = true;
#endif

#ifdef CONFIG_ENABLE_CAMERA
bool camera_ready = true;
#endif

static const char* TAG = "MAIN";

extern EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static esp_pm_lock_handle_t pmlock_cpu;
static esp_pm_lock_handle_t pmlock_ahb;
static esp_pm_lock_handle_t pmlock_wifi;
static int pmlock_cnt = 0;

void pm_init()
{
#if CONFIG_PM_ENABLE
    ESP_LOGI(TAG, "starting pm ...");
    // Configure dynamic frequency scaling:
    // maximum and minimum frequencies are set in sdkconfig,
    // automatic light sleep is enabled if tickless idle support is enabled.
#ifdef CONFIG_IDF_TARGET_ESP32S2
    esp_pm_config_esp32s2_t pm_config = {};
#else
    esp_pm_config_esp32_t pm_config = {};
#endif
    pm_config.max_freq_mhz = 240;
#ifdef CONFIG_IDF_TARGET_ESP32S2
    pm_config.min_freq_mhz = 10;
#else
    pm_config.min_freq_mhz = 40;
#endif
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
    pm_config.light_sleep_enable = true;
#endif
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif // CONFIG_PM_ENAB    
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "pm_lock_wifi", &pmlock_cpu);    
    esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "pm_lock_ahb", &pmlock_ahb);    
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "pm_lock_wifi", &pmlock_wifi);    
    pmlock_cnt = 0;
}

int pm_cnt()
{
    return pmlock_cnt;
}

void pm_ref()
{
    pmlock_cnt++;
    ESP_LOGV(TAG, "pmlock_cnt=%d",pmlock_cnt);
#if CONFIG_PM_ENABLE
    if( pmlock_cnt > 0 )
    {
        ESP_LOGV(TAG, "pmlock_cnt=%d aquire",pmlock_cnt);
        esp_pm_lock_acquire(pmlock_cpu);  
        esp_pm_lock_acquire(pmlock_ahb);  
        esp_pm_lock_acquire(pmlock_wifi);  
    }
#endif
}

void pm_unref()
{
    pmlock_cnt--;
    if( pmlock_cnt < 0 )
    {
        ESP_LOGE(TAG, "pmlock_cnt=%d",pmlock_cnt);
    }
    else
    {
        ESP_LOGV(TAG, "pmlock_cnt=%d",pmlock_cnt);
    }
#if CONFIG_PM_ENABLE
    if( pmlock_cnt <= 0 )
    {
        ESP_LOGV(TAG, "pmlock_cnt=%d release",pmlock_cnt);
        esp_pm_lock_release(pmlock_cpu);  
        esp_pm_lock_release(pmlock_ahb);  
        esp_pm_lock_release(pmlock_wifi);  
    }
#endif
}

// esp_ip4_addr_t s_ip_addr = {};
// uint8_t s_ip_addr_changed = 1;

#if defined(CONFIG_PARTITION_TABLE_CUSTOM) || defined(CONFIG_PARTITION_TABLE_TWO_OTA)
/**
 * @brief
 * @param param
 */
static void ota_server_task(void* param)
{
    ESP_LOGI(TAG, "ota task ...");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

    ota_server_start();
    vTaskDelete(NULL);
}
#endif

#ifdef CONFIG_ENABLE_SDCARD

// ESP32-S2 and ESP32-C3 doesn't have an SD Host peripheral, always use SPI:
#if CONFIG_IDF_TARGET_ESP32S2 ||CONFIG_IDF_TARGET_ESP32C3
#ifndef USE_SPI_MODE
#define USE_SPI_MODE
#endif // USE_SPI_MODE
// on ESP32-S2, DMA channel must be the same as host id
#define SPI_DMA_CHAN    host.slot
#endif //CONFIG_IDF_TARGET_ESP32S2

// DMA channel to be used by the SPI peripheral
#ifndef SPI_DMA_CHAN
#define SPI_DMA_CHAN    1
#endif //SPI_DMA_CHAN

/**
 * @brief
 * @param param
 */
static void sd_test_task(void* param)
{
    ESP_LOGW(TAG, "sd_test_task() %d ",sdcard_ready);
    if(sdcard_ready == true )
    {
        uint8_t fn_idx = 0;
        char fn[64];
        // Card has been initialized, print its properties
        sdmmc_card_print_info(stdout, card);

        while(1)
        {
            size_t sd_test_buf_size = 32 * 1024;
            uint8_t* sd_test_buf = (uint8_t*)malloc(sd_test_buf_size);
            if(sd_test_buf != NULL)
            {
                memset(sd_test_buf, 0, sd_test_buf_size);
                // Use POSIX and C standard library functions to work with files.
                // First create a file.
                sprintf(fn, "/sdcard/hello-%d.bin", fn_idx++);
                FILE* f = fopen(fn, "w");
                if(f == NULL)
                {
                    ESP_LOGE(TAG, "Failed to open file for writing");
                }
                else
                {
                    int n,r;
                    uint32_t i = 0;
                    int64_t t = esp_timer_get_time();
                    for(n=0;n<(8*4*8);n++)
                    {
                        pm_ref();
                        r = fwrite(sd_test_buf, 1, sd_test_buf_size, f);
                        pm_unref();
                        if( r != sd_test_buf_size )
                        {
                            ESP_LOGE(TAG, "fwrite error %d",r);
                            break;
                        }
                        i += r;
                        esp_task_wdt_reset();
                    }
                    fflush(f);
                    if((esp_timer_get_time() - t) != 0)
                    {
                        ESP_LOGI(TAG, "File (%s) written %dMByte %dkB/s", fn, 
                            (int)((i) / (1024 * 1024)),
                            (int)((1000 * i) / (esp_timer_get_time() - t)));
                    }
                    fclose(f);
                }

                {
                    f = fopen(fn, "r");
                    if(f == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to open file for reading");
                    }
                    else
                    {
                        int64_t t = esp_timer_get_time();
                        size_t i = 0;
                        while(!feof(f))
                        {
                            pm_ref();
                            int r = fread(sd_test_buf, 1, sd_test_buf_size, f);
                            pm_unref();
                            if( r < 0 )
                            {
                                ESP_LOGE(TAG, "fread error %d",r);
                                break;
                            }
                            i += r; 
                            esp_task_wdt_reset();
                        }
                        if((esp_timer_get_time() - t) != 0)
                        {
                            ESP_LOGI(TAG, "File (%s) read %dMByte %dkB/s", fn, 
                                (int)((i) / (1024 * 1024)),
                                (int)((1000 * i) / (esp_timer_get_time() - t)));
                        }
                        fclose(f);
                    }
                }
                free(sd_test_buf);
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief
 */
void sdmmc_init()
{
    esp_err_t ret;
#ifndef USE_SPI_MODE
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode, uncomment the following line:
    slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_MOSI, GPIO_PULLUP_ONLY); // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_MISO, GPIO_PULLUP_ONLY); // D0, needed in 4- and 1-line modes
    // gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    // gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_CS, GPIO_PULLUP_ONLY);  // D3, needed in 4- and 1-line modes
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_CLK, GPIO_PULLUP_ONLY); // CLK, needed in 4- and 1-line modes

#else
    ESP_LOGI(TAG, "Using SPI peripheral");

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_MOSI, GPIO_PULLUP_ONLY); // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_MISO, GPIO_PULLUP_ONLY); // D0, needed in 4- and 1-line modes
    // gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    // gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_CS, GPIO_PULLUP_ONLY);  // D3, needed in 4- and 1-line modes
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_CLK, GPIO_PULLUP_ONLY); // CLK, needed in 4- and 1-line modes

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SDSPI_PIN_NUM_MOSI,
        .miso_io_num = SDSPI_PIN_NUM_MISO,
        .sclk_io_num = SDSPI_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ESP_LOGI(TAG, "SDSPI using dma %d",SPI_DMA_CHAN);
    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SPI_DMA_CHAN);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SDSPI_PIN_NUM_CS;
    slot_config.host_id = (spi_host_device_t)host.slot;
#endif // USE_SPI_MODE

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true, .max_files = 16, .allocation_unit_size = 16 * 1024
    };

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
    // Please check its source code and implement error recovery when developing
    // production applications.
#ifndef USE_SPI_MODE
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
#else
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
#endif
    if(ret != ESP_OK)
    {
        if(ret == ESP_FAIL)
        {
            ESP_LOGE(TAG,
                "Failed to mount filesystem. "
                "If you want the card to be formatted, set format_if_mount_failed = true.");
        }
        else
        {
            ESP_LOGE(TAG,
                "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.",
                esp_err_to_name(ret));
        }
        sdcard_ready = false;
    }
    else
    {
        ESP_LOGI(TAG, "sdcard mounted");
        sdcard_ready = true;
    }
}
#endif

#ifdef CONFIG_ENABLE_SPI
spi_device_handle_t spi;

spi_bus_config_t buscfg = {};
spi_device_interface_config_t devcfg = {};

void spihost_init()
{
    esp_err_t ret;

    gpio_set_direction(SPIHOST_PIN_NUM_CS, GPIO_MODE_OUTPUT);

    buscfg.miso_io_num = SPIHOST_PIN_NUM_MISO;
    buscfg.mosi_io_num = SPIHOST_PIN_NUM_MOSI;
    buscfg.sclk_io_num = SPIHOST_PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 1024 * 8;

    devcfg.clock_speed_hz = 4 * 1000 * 1000;  // Clock out at 26 MHz
    devcfg.mode = 0;                          // SPI mode 0
    devcfg.spics_io_num = SPIHOST_PIN_NUM_CS; // CS pin
    devcfg.queue_size = 1;                    // We want to be able to queue 7 transactions at a time
    //.pre_cb=lcd_spi_pre_transfer_callback,      //Specify pre-transfer callback to handle D/C line
    devcfg.command_bits = 8;
    devcfg.address_bits = 8;
    // devcfg.dummy_bits = 32;
    devcfg.cs_ena_pretrans = 1;
    devcfg.cs_ena_posttrans = 16;
    // devcfg.flags = SPI_DEVICE_HALFDUPLEX;

    // Initialize the SPI bus
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1 /*dma*/);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

/**
 * @brief
 * @param param
 */
static void spihost_test_task(void* param)
{
    ESP_LOGW(TAG, "spihost_test_task()");
    while(1)
    {
        pm_ref();
        size_t test_buf_size = 1024;
        uint8_t* test_buf = (uint8_t*)malloc(test_buf_size);
        if(test_buf != NULL)
        {
            // ESP_LOGW(TAG, "spihost_test_task() loop ...");
            for(int i = 0; i < test_buf_size; i++)
            {
                test_buf[i] = i & 0xff;
            }

            esp_err_t ret;
            spi_transaction_t t = {};
            t.cmd = 0xAA;
            t.addr = 0x55;
            t.length = test_buf_size * 8;   // Len is in bytes, transaction length is in bits.
            t.rxlength = test_buf_size * 8; // Len is in bytes, transaction length is in bits.
            t.tx_buffer = test_buf;         // Data
            t.rx_buffer = test_buf;         // Data
            // t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
            ret = spi_device_polling_transmit(spi, &t); // Transmit!
            assert(ret == ESP_OK);                      // Should have had no issues.

            for(int i = 0; i < test_buf_size; i++)
            {
                if(test_buf[i] != (i & 0xff))
                {
                    ESP_LOGW(TAG, "spihost_test_task() payload error test_buf[%04x] != %02x", i, i & 0xff);
                    break;
                }
            }

            free(test_buf);
        }
        pm_unref();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
#endif

/**
 * @brief beeop on/off
 * @param onoff
 */
void beep(int onoff)
{
#ifdef ENABLE_BUZZER
#ifdef BUZZER_PIN
    gpio_num_t gpio = (gpio_num_t)BUZZER_PIN;
    gpio_reset_pin(gpio);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, (onoff == 0) ? 0 : 1);
#endif
#endif
}

void powersw(bool onoff)
{
#ifdef CONFIG_ROS2NODE_HW_S2_MOWER
    ESP_LOGE(TAG, "powersw %d", onoff);
    gpio_num_t gpio;
    gpio = (gpio_num_t)GPIO_PWR_ON;
    gpio_hold_dis(gpio);
    gpio_set_pull_mode(gpio, GPIO_PULLUP_PULLDOWN);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, (onoff == true) ? 1 : 0);
    gpio_hold_en(gpio);

#if 0
    gpio = (gpio_num_t)GPIO_PWR_BUS_ON;
    gpio_hold_dis(gpio);
    gpio_set_pull_mode(gpio, GPIO_PULLUP_PULLDOWN);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
	gpio_set_level(gpio, (onoff==true)?1:0);
    gpio_hold_en(gpio);
#endif

#endif
}

extern uint8_t wifi_error;
uint8_t shtdwndly = 0;
uint8_t g_adccnt = 0;
float g_ubat = 0.0;
float g_usolar = 0.0;
float g_ucharge = 0.0;
float g_uhal = 0.0;
float g_ibat = 0.0;
float g_temperature = -999.0;

static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    // Check if TP is burned into eFuse
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }
    // Check Vref is burned into eFuse
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 1   // Multisampling

static esp_adc_cal_characteristics_t* adc_chars;

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if(val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if(val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

float adc1_get_voltage(adc1_channel_t channel, float k)
{
    // Configure ADC
    uint32_t adc_reading = 0;
    // Multisampling
    for(int i = 0; i < NO_OF_SAMPLES; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_reading /= NO_OF_SAMPLES;
    // Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return k * voltage;
}

#ifdef CONFIG_ROS2NODE_HW_S2_MOWER
static void adc1_timer_callback(void* arg)
{
    uint sleep_seconds = 30*60;
    float k1 = 110.0 / 10.0;
    float k2 = (2.505 / 2.816) * (20.0 / 10.0);
    adc_power_acquire();
    g_ubat = adc1_get_voltage(ADC1_CHANNEL_0, k1) / 1000.0;
    g_usolar = adc1_get_voltage(ADC1_CHANNEL_1, k1) / 1000.0;
    g_ucharge = adc1_get_voltage(ADC1_CHANNEL_2, k1) / 1000.0;
    g_uhal = adc1_get_voltage(ADC1_CHANNEL_3, k2) / 1000.0;
    adc_power_release();
    g_ibat = /*(46.7/2.127) **/ 0.185 * (g_uhal - (5.033 / 2.0));
    // g_ibat = g_uhal;
    if(g_adccnt < 255)
        g_adccnt++;

    ESP_LOGW(TAG,
        "cnt=%d ubat=%3.1f usolar=%3.1f ucharge=%3.1f ibat=%1.6f wifi_error=%d pmlock_cnt=%d",
        g_adccnt, g_ubat, g_usolar, g_ucharge, g_ibat, wifi_error, pmlock_cnt);

    bool enter_sleep = false;

    if(g_adccnt == 10)
    {
        powersw(true); 
    }

    if(g_adccnt > 15)
    {
        if(g_ubat < UBAT_FULL)
        {
            enter_sleep = true;
        }
        if(g_usolar > (UBAT_CHARGE + UBAT_DF))
        {
            enter_sleep = false;
        }
        if(g_ucharge > (UBAT_CHARGE + UBAT_DF))
        {
            enter_sleep = false;
        }
        if(g_ubat > UBAT_CHARGE)
        {
            enter_sleep = false;
        } 
    }
    else if(g_adccnt > 5)
    {
        if(g_ubat < UBAT_EMPTY)
        {
            enter_sleep = true;
        }
    }
    
    if((g_ubat > UBAT_CHARGE) && ((g_usolar > (UBAT_CHARGE + UBAT_DF)) || (g_ucharge > (UBAT_CHARGE + UBAT_DF))))
    {
        /* do short sleeps when bat is full and charging 
         */
        sleep_seconds = 60;
    }
    
    if( wifi_error > 2 )
    {
        /* enter sleep mode when wifi is not connected
         */
        enter_sleep = true;
    }
    
    if(console_connected()==true)
    {
        /* delay shutdown when console is conneted
         */
        shtdwndly = 255;
    }

    if( shtdwndly > 0 )
    {
        /* skip shutdoen in delay state
         */
        shtdwndly--;
        enter_sleep = false;       
    }
        
    if(enter_sleep==true) 
    {
        /* shutdown
         */
        shtdwndly = 0;
        ESP_LOGW(TAG, "Entering deep sleep (%d minutes)",sleep_seconds/60);
        esp_sleep_enable_timer_wakeup(1000000UL * sleep_seconds); // sleep 1 hour
        powersw(false);
        esp_wifi_stop();
        esp_deep_sleep_start();
        while(1)
            ;
    }
}
#endif

#ifdef CONFIG_ESP32S2_ULP_COPROC_ENABLED
static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

#if 0
    /* Configure ADC channel */
    /* Note: when changing channel here, also change 'adc_channel' constant
       in adc.S */
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
#if CONFIG_IDF_TARGET_ESP32
    adc1_config_width(ADC_WIDTH_BIT_12);
#elif CONFIG_IDF_TARGET_ESP32S2
    adc1_config_width(ADC_WIDTH_BIT_13);
#endif
#endif

    adc1_ulp_enable();

    /* Set low and high thresholds, approx. 1.35V - 1.75V*/
    ulp_low_thr = 9000;
    ulp_high_thr = 10000;

    /* Set ULP wake up period to 20ms */
    ulp_set_wakeup_period(0, 20000);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_1);
    // rtc_gpio_isolate(GPIO_NUM_15);
#if CONFIG_IDF_TARGET_ESP32
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages
#endif
}

static void start_ulp_program(void)
{
    /* Reset sample counter */
    ulp_sample_counter = 0;

    /* Start the program */
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}
#endif

/**
 * @brief main
 */
extern "C"
{
    void app_main(void);
}

void app_main(void)
{
    EventBits_t ev;
    beep(1);
    powersw(false);
    // powersw(true);

#ifdef CONFIG_ROS2NODE_HW_S2_MOWER
    // Check if Two Point or Vref are burned into eFuse
    check_efuse();

    // Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type =
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_13, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

#ifdef CONFIG_IDF_TARGET_ESP32S2
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_get_config(&temp_sensor);
    temp_sensor.dac_offset = TSENS_DAC_DEFAULT; // DEFAULT: range:-10℃ ~  80℃, error < 1℃.
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
#endif

    adc1_config_width(ADC_WIDTH_BIT_13);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_6);
#endif

#ifdef CONFIG_ESP32S2_ULP_COPROC_ENABLED

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if(cause != ESP_SLEEP_WAKEUP_ULP)
    {
        printf("Not ULP wakeup\n");
        init_ulp_program();
    }
    else
    {
        printf("Deep sleep wakeup\n");
        printf("ULP did %d measurements since last reset\n", ulp_sample_counter & UINT16_MAX);
        printf("Thresholds:  low=%d  high=%d\n", ulp_low_thr, ulp_high_thr);
        ulp_last_result &= UINT16_MAX;
        printf("Value=%d was %s threshold\n", ulp_last_result, ulp_last_result < ulp_low_thr ? "below" : "above");
    }
    start_ulp_program();
#endif

#ifdef CONFIG_ESP32S2_ULP_COPROC_ENABLED
    // printf("Entering deep sleep\n\n");
    // ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    // esp_sleep_enable_ulp_wakeup();
    // powersw(false);
    // esp_sleep_enable_timer_wakeup(5000000);
    // esp_deep_sleep_start();
#endif

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG, "%dMB %s flash", spi_flash_get_chip_size() / (1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG, "heap: %d bytes free", xPortGetFreeHeapSize());

    ESP_LOGI(TAG, "init NVS ...");
    esp_err_t err = nvs_flash_init();
    if(err != ESP_OK)
    {
        ESP_LOGW(TAG, "nvs_flash_erase/erase");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    pm_init();

#ifdef CONFIG_ROS2NODE_HW_S2_MOWER
#ifndef CONFIG_ESP32S2_ULP_COPROC_ENABLED
    const esp_timer_create_args_t adc1_timer_args = { .callback = &adc1_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "adc1_timer" };
    esp_timer_handle_t adc1_timer;
    ESP_ERROR_CHECK(esp_timer_create(&adc1_timer_args, &adc1_timer));
    /* The timer has been created but is not running yet */

    ESP_ERROR_CHECK(esp_timer_start_periodic(adc1_timer, 1000000));
#endif
#endif

    check_wifi_config();

    /* start wifi ...
     */
    ESP_LOGI(TAG, "init WIFI ...");
    initialise_wifi();

#ifdef CONFIG_ENABLE_SPIFS
    ESP_LOGI(TAG, "init SPIFS ...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs", .partition_label = NULL, .max_files = 5, .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if(ret != ESP_OK)
    {
        if(ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if(ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
#endif

#if 0
    gpio_num_t gpio = (gpio_num_t)13;
    gpio_reset_pin(gpio);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    while(1) 
	{
        ESP_LOGI(TAG, "0");
		gpio_set_level(gpio, 0);
		vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "1");
		gpio_set_level(gpio, 1);
		vTaskDelay(10 / portTICK_PERIOD_MS);
    }
#endif

#ifdef CONFIG_ENABLE_I2C
    ESP_LOGI(TAG, "init I2C ...");
    i2c_handler_init();
#endif

#ifdef CONFIG_ENABLE_SDCARD
    sdmmc_init();
    xTaskCreate(&sd_test_task, "sd_test_task", 4096, NULL, DEFAULT_PRIO, NULL);
#endif

#ifdef CONFIG_ENABLE_SPI
    ESP_LOGI(TAG, "starting spi ...");
    spihost_init();
    xTaskCreate(&spihost_test_task, "spihost_test_task", 512, NULL, DEFAULT_PRIO, NULL);
#endif

    /* ... WAIT FOR WIFI ...
     */
    ESP_LOGI(TAG, "Wait until Wifi Connection ... ");
    ev = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "... Connected");

    pm_ref();

#if defined(CONFIG_PARTITION_TABLE_CUSTOM) || defined(CONFIG_PARTITION_TABLE_TWO_OTA)
    ESP_LOGI(TAG, "starting ota ...");
    xTaskCreate(&ota_server_task, "ota_server_task", 4096, NULL, DEFAULT_PRIO, NULL);
#endif

    // my_deflog = esp_log_set_vprintf(my_log);
    // ESP_LOGW(TAG, "ready");

#ifdef CONFIG_ENABLE_ROS2
    ESP_LOGI(TAG, "starting ros2 ...");
    ros2node_init();
    ros2node_start();
#endif

#ifdef CONFIG_ENABLE_GPS
    ESP_LOGI(TAG, "starting gps ...");
    gps_init();
#endif

#ifdef CONFIG_ENABLE_CAMERA
    camera_init();
#endif

    // my_deflog = esp_log_set_vprintf(my_i2clog);

#ifdef CONFIG_ENABLE_WEBUI
    ESP_LOGI(TAG, "starting webui ...");
    webui_init();
#endif

    beep(0);

    pm_unref();

    ESP_LOGI(TAG, "starting console ...");
    console();

    ESP_LOGI(TAG, "... init done. free heap: %u", xPortGetFreeHeapSize());
}

/**
 * EOF
 */
