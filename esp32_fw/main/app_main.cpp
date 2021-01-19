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

#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <driver/adc.h>
#include <driver/adc_common.h>
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
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "i2chandler.h"
#include "nvs_flash.h"
#include "esp32/rom/uart.h"
#include "sdmmc_cmd.h"
#include "driver/rtc_io.h"
#if CONFIG_IDF_TARGET_ESP32S2
#include "driver/temp_sensor.h"
#endif

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

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
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");
#endif

void beep(int onoff);

static esp_err_t event_handler(void* ctx, system_event_t* event);
static void initialise_wifi(void);

static char my_wifi_ssid[64] = {};
static char my_wifi_psk[64] = {};
static bool my_wifi_save_on_connected = false;

#ifdef CONFIG_ENABLE_SDCARD
sdmmc_card_t* card = NULL;
bool sdcard_ready = true;
#endif

#ifdef CONFIG_ENABLE_CAMERA
bool camera_ready = true;
#endif

static const char* TAG = "MAIN";

EventGroupHandle_t s_wifi_event_group;
const int CONNECTED_BIT = BIT0;

esp_ip4_addr_t s_ip_addr = {};
uint8_t s_ip_addr_changed = 1;

#if defined(CONFIG_PARTITION_TABLE_CUSTOM) || defined(CONFIG_PARTITION_TABLE_TWO_OTA)
/**
 * @brief 
 * @param param
 */
static void ota_server_task(void* param)
{
    ESP_LOGI(TAG, "ota task #1 ...");
    xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "ota task #2 ...");
    ota_server_start();
    vTaskDelete(NULL);
}
#endif

#ifdef CONFIG_ENABLE_SDCARD
// This example can use SDMMC and SPI peripherals to communicate with SD card.
// By default, SDMMC peripheral is used.
// To enable SPI mode, uncomment the following line:

#undef USE_SPI_MODE

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.

#ifdef USE_SPI_MODE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#endif // USE_SPI_MODE


/**
 * @brief 
 * @param param
 */
static void sd_test_task(void* param)
{
    if(sdcard_ready == true && card != NULL) 
	{
		ESP_LOGW(TAG, "sd_test_task()");

		uint8_t fn_idx = 0;
		char fn[64];
		// Card has been initialized, print its properties
		sdmmc_card_print_info(stdout, card);


		while(1) {
			size_t sd_test_buf_size = 2 * 1024 * 1024;
			uint8_t* sd_test_buf = (uint8_t*)malloc(sd_test_buf_size);
			if(sd_test_buf != NULL) {
				memset(sd_test_buf,0,sd_test_buf_size);
				// Use POSIX and C standard library functions to work with files.
				// First create a file.
				sprintf(fn,"/sdcard/hello-%d.bin",fn_idx++);
				FILE* f = fopen(fn, "w");
				if(f == NULL) {
					ESP_LOGE(TAG, "Failed to open file for writing");
				} else {
					uint32_t i = 0;
					int64_t t = esp_timer_get_time();
					i += fwrite(sd_test_buf, 1, sd_test_buf_size, f);
					if((esp_timer_get_time() - t) != 0) {
					ESP_LOGI(TAG, "File (%s) written %dMByte %dkB/s", 
						fn,
						(int)(sd_test_buf_size / (1024 * 1024)),
						(int)((1000 * i) / (esp_timer_get_time() - t)));
					}
					fclose(f);
				}

				{
					f = fopen(fn, "r");
					if(f == NULL) {
					ESP_LOGE(TAG, "Failed to open file for reading");
					} else {
					int64_t t = esp_timer_get_time();
					size_t i = 0;
					while(!feof(f)) {
						i += fread(sd_test_buf, 1, sd_test_buf_size, f);
					}
					if((esp_timer_get_time() - t) != 0) {
						ESP_LOGI(TAG, "File (%s) read %dkB/s", 
							fn,
							(int)((1000 * i) / (esp_timer_get_time() - t)));
					}
					fclose(f);
					}
				}
				free(sd_test_buf);
			}

			vTaskDelay(5000 / portTICK_PERIOD_MS);
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
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_MISO, GPIO_PULLUP_ONLY);  // D0, needed in 4- and 1-line modes
    // gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    // gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_CS, GPIO_PULLUP_ONLY); // D3, needed in 4- and 1-line modes
    gpio_set_pull_mode((gpio_num_t)SDSPI_PIN_NUM_CLK, GPIO_PULLUP_ONLY); // CLK, needed in 4- and 1-line modes

#else
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = SDSPI_PIN_NUM_MISO;
    slot_config.gpio_mosi = SDSPI_PIN_NUM_MOSI;
    slot_config.gpio_sck = SDSPI_PIN_NUM_CLK;
    slot_config.gpio_cs = SDSPI_PIN_NUM_CS;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
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
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if(ret != ESP_OK) {
	if(ret == ESP_FAIL) {
	    ESP_LOGE(TAG,
	        "Failed to mount filesystem. "
	        "If you want the card to be formatted, set format_if_mount_failed = true.");
	} else {
	    ESP_LOGE(TAG,
	        "Failed to initialize the card (%s). "
	        "Make sure SD card lines have pull-up resistors in place.",
	        esp_err_to_name(ret));
	}
	sdcard_ready = false;
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


	buscfg.miso_io_num=SPIHOST_PIN_NUM_MISO;
    buscfg.mosi_io_num=SPIHOST_PIN_NUM_MOSI;
	buscfg.sclk_io_num=SPIHOST_PIN_NUM_CLK;
	buscfg.quadwp_io_num=-1;
	buscfg.quadhd_io_num=-1;
	buscfg.max_transfer_sz=1024*8;
	
	devcfg.clock_speed_hz=4*1000*1000;           //Clock out at 26 MHz
	devcfg.mode=0;                         		  //SPI mode 0
    devcfg.spics_io_num=SPIHOST_PIN_NUM_CS;       //CS pin
    devcfg.queue_size=1;                          //We want to be able to queue 7 transactions at a time
    //.pre_cb=lcd_spi_pre_transfer_callback,      //Specify pre-transfer callback to handle D/C line
	devcfg.command_bits = 8;
	devcfg.address_bits = 8;
	//devcfg.dummy_bits = 32;
	devcfg.cs_ena_pretrans = 1;
	devcfg.cs_ena_posttrans = 16;
    //devcfg.flags = SPI_DEVICE_HALFDUPLEX;
		
    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1 /*dma*/ );
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
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
		size_t test_buf_size = 1024;
		uint8_t* test_buf = (uint8_t*)malloc(test_buf_size);
		if(test_buf != NULL) 
		{		
			//ESP_LOGW(TAG, "spihost_test_task() loop ...");
			for( int i=0;i<test_buf_size;i++)
			{
				test_buf[i] = i&0xff;
			}
			
		    esp_err_t ret;
			spi_transaction_t t = {};
			t.cmd = 0xAA;
			t.addr = 0x55;
			t.length=test_buf_size*8;       //Len is in bytes, transaction length is in bits.
			t.rxlength=test_buf_size*8;     //Len is in bytes, transaction length is in bits.
			t.tx_buffer=test_buf;           //Data
			t.rx_buffer=test_buf;           //Data
			//t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
			ret=spi_device_polling_transmit(spi, &t);  //Transmit!
			assert(ret==ESP_OK);            //Should have had no issues.

			for( int i=0;i<test_buf_size;i++)
			{
				if( test_buf[i] != (i&0xff) )
				{
					ESP_LOGW(TAG, "spihost_test_task() payload error test_buf[%04x] != %02x",i,i&0xff);
					break;
				}
			}

			free(test_buf);
		}		
        vTaskDelay(10 / portTICK_PERIOD_MS);
	}
    vTaskDelete(NULL);
}
#endif

void check_wifi_config()
{
    /* Install UART driver for interrupt-driven reads and writes */
    uart_driver_install((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0);
    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Disable buffering on stdin and stdout */
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    while(1) {
	nvs_handle my_handle;
	esp_err_t err = nvs_open("wifi", NVS_READWRITE, &my_handle);
	if(err == ESP_OK) {
	    size_t my_wifi_ssid_size = sizeof(my_wifi_ssid);
	    size_t my_wifi_psk_size = sizeof(my_wifi_psk);
	    nvs_get_str(my_handle, "ssid", my_wifi_ssid, &my_wifi_ssid_size);
	    nvs_get_str(my_handle, "psk", my_wifi_psk, &my_wifi_psk_size);
	    nvs_close(my_handle);
	    if(strlen(my_wifi_ssid) != 0 && strlen(my_wifi_psk) != 0) {
		return;
	    }
	    my_wifi_save_on_connected = true;
	    printf("\nenter Wifi SSID: ");
	    gets(my_wifi_ssid);
	    printf("\nenter Wifi PSK: ");
	    gets(my_wifi_psk);
	    printf("\ntrying SSID=%s and PSK=%s\n",my_wifi_ssid,my_wifi_psk);
        char yesbuffer[16] = {};
	    printf("\nenter YES: ");
	    gets(yesbuffer);
        if( strcmp("YES",yesbuffer)==0 )
        {
            esp_err_t err = nvs_open("wifi", NVS_READWRITE, &my_handle);
            if(err == ESP_OK) {
            //size_t my_wifi_ssid_size = sizeof(my_wifi_ssid);
            //size_t my_wifi_psk_size = sizeof(my_wifi_psk);
            nvs_set_str(my_handle, "ssid", my_wifi_ssid);
            nvs_set_str(my_handle, "psk", my_wifi_psk);
            nvs_close(my_handle);
            printf("SSID and PSK stored to nvs\n");
   	        my_wifi_save_on_connected = false;
        }
      }
    }
  }
}

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
    gpio_set_level(gpio, (onoff==0)?0:1);
#endif
#endif
}

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
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   4          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

float adc1_get_voltage(adc1_channel_t channel,float k)
{
        //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width); 
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, width, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return k * voltage;
}

void powersw(bool onoff)
{
#ifdef CONFIG_ROS2NODE_HW_S2_MOWER
    gpio_num_t gpio = (gpio_num_t)GPIO_PWR_ON;
    gpio_reset_pin(gpio);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
	gpio_set_level(gpio, (onoff==true)?1:0);
#endif    
}

static void adc1_timer_callback(void* arg)
{
    float k1 = 110.0/10.0;
    float k2 = 20.0/10.0;
    g_ubat = adc1_get_voltage(ADC1_CHANNEL_0,k1)/1000.0;
    g_usolar = adc1_get_voltage(ADC1_CHANNEL_1,k1)/1000.0;
    g_ucharge = adc1_get_voltage(ADC1_CHANNEL_2,k1)/1000.0;
    g_uhal = adc1_get_voltage(ADC1_CHANNEL_3,k2)/1000.0;
    g_ibat = 0.185*(g_uhal-(5.425/2.0));
    if( g_adccnt < 255 ) g_adccnt++;
    //temp_sensor_read_celsius(&g_temperature);
    ESP_LOGI(TAG, "cnt=%d ubat=%3.1f usolar=%3.1f ucharge=%3.1f ibat=%1.3f", g_adccnt, g_ubat,g_usolar,g_ucharge,g_ibat);
    //ESP_LOGI(TAG, "RAW: ubat=%d", adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_0));
    
#if 1
    if( g_adccnt > 15 )
    {
        bool enter_sleep = false;
        if( g_ubat < 12.5 )
        {
            enter_sleep = true;
        }
        if( g_usolar > 12.0 )
        {
            enter_sleep = false;
        }
        if( g_ucharge > 12.0 )
        {
            enter_sleep = false;
        }
        if( enter_sleep == true )
        {
            ESP_LOGW(TAG, "Entering deep sleep (30 minutes)\n\n");
            esp_sleep_enable_timer_wakeup(60UL * 60UL * 1000000UL); // sleep 1 hour
            powersw(false);
            esp_deep_sleep_start();
            while(1);
        }
    }
#endif
    
}

#ifdef CONFIG_ESP32S2_ULP_COPROC_ENABLED
static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
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
    //rtc_gpio_isolate(GPIO_NUM_15);
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
extern "C" {
void app_main(void);
}

void app_main(void)
{
    beep(1);
    powersw(false);
    
#if 1
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();


    //Characterize ADC
    adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_get_config(&temp_sensor);
    temp_sensor.dac_offset = TSENS_DAC_DEFAULT; // DEFAULT: range:-10℃ ~  80℃, error < 1℃.
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
    
    adc1_config_width(width); 
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_6);
#endif

#ifdef CONFIG_ESP32S2_ULP_COPROC_ENABLED
    ESP_LOGI(TAG, "init ULP ...");
    
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup\n");
        init_ulp_program();
    } else {
        printf("Deep sleep wakeup\n");
        printf("ULP did %d measurements since last reset\n", ulp_sample_counter & UINT16_MAX);
        printf("Thresholds:  low=%d  high=%d\n", ulp_low_thr, ulp_high_thr);
        ulp_last_result &= UINT16_MAX;
        printf("Value=%d was %s threshold\n", ulp_last_result,
                ulp_last_result < ulp_low_thr ? "below" : "above");
    }
    start_ulp_program();
#endif

#ifdef CONFIG_ESP32S2_ULP_COPROC_ENABLED
    printf("Entering deep sleep\n\n");
    //ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    //esp_sleep_enable_ulp_wakeup();
    powersw(false);
    esp_sleep_enable_timer_wakeup(5000000);
    //esp_deep_sleep_start();
#endif

#if 0
    //Continuously sample ADC1
    while (1) {
        temp_sensor_read_celsius(&g_temperature);
        
        printf("UBat=%3.1f USolar=%3.1f UCharge=%3.1f IBat=%3.1f %4.3f %f\n", 
            adc1_get_voltage(ADC1_CHANNEL_0,110.0/10.0),
            adc1_get_voltage(ADC1_CHANNEL_1,110.0/10.0),
            adc1_get_voltage(ADC1_CHANNEL_2,110.0/10.0),
            adc1_get_voltage(ADC1_CHANNEL_3,20.0/10.0),
            0.185*(adc1_get_voltage(ADC1_CHANNEL_3,20.0/10.0)-2500.0)/1000.0,
            g_temperature);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

#endif
    
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG, "%dMB %s flash", spi_flash_get_chip_size() / (1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG, "heap: %d bytes free",
        xPortGetFreeHeapSize());

    ESP_LOGI(TAG, "init NVS ...");
    esp_err_t err = nvs_flash_init();
    if(err != ESP_OK) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    check_wifi_config();

#if 0
    gpio_num_t gpio = (gpio_num_t)13;
    gpio_reset_pin(gpio);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    while(1) 
	{
        //ESP_LOGI(TAG, "0");
		gpio_set_level(gpio, 0);
		//vTaskDelay(10 / portTICK_PERIOD_MS);
        //ESP_LOGI(TAG, "1");
		gpio_set_level(gpio, 1);
		//vTaskDelay(10 / portTICK_PERIOD_MS);
    }
#endif

#ifndef CONFIG_ESP32S2_ULP_COPROC_ENABLED
    const esp_timer_create_args_t adc1_timer_args = {
            .callback = &adc1_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "adc1_timer"
    };
    esp_timer_handle_t adc1_timer;
    ESP_ERROR_CHECK(esp_timer_create(&adc1_timer_args, &adc1_timer));
    /* The timer has been created but is not running yet */

    ESP_ERROR_CHECK(esp_timer_start_periodic(adc1_timer, 1000000));
#endif

#ifdef CONFIG_ENABLE_SPIFS
    ESP_LOGI(TAG, "init SPIFS ...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs", .partition_label = NULL, .max_files = 5, .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if(ret != ESP_OK) {
	if(ret == ESP_FAIL) {
	    ESP_LOGE(TAG, "Failed to mount or format filesystem");
	} else if(ret == ESP_ERR_NOT_FOUND) {
	    ESP_LOGE(TAG, "Failed to find SPIFFS partition");
	} else {
	    ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
	}
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if(ret != ESP_OK) {
	ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
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

    initialise_wifi();
#if defined(CONFIG_PARTITION_TABLE_CUSTOM) || defined(CONFIG_PARTITION_TABLE_TWO_OTA)
    ESP_LOGI(TAG, "starting ota ...");
    xTaskCreate(&ota_server_task, "ota_server_task", 4096, NULL, 5, NULL);
#endif

    // my_deflog = esp_log_set_vprintf(my_log);
    //ESP_LOGW(TAG, "ready");

#ifdef CONFIG_ENABLE_SDCARD
    sdmmc_init();
    xTaskCreate(&sd_test_task, "sd_test_task", 4096, NULL, 5, NULL);
    //while(1) vTaskDelay(1000 / portTICK_PERIOD_MS);
#endif

#ifdef CONFIG_ENABLE_SPI
    ESP_LOGI(TAG, "starting spi ...");
    spihost_init();
    xTaskCreate(&spihost_test_task, "spihost_test_task", 512, NULL, 5, NULL);
#endif

#ifdef CONFIG_ENABLE_ROS2
    ESP_LOGI(TAG, "starting ros2 ...");
    ros2node_init();
#endif

#ifdef CONFIG_ENABLE_GPS
    ESP_LOGI(TAG, "starting gps ...");
    gps_init();
#endif

#ifdef CONFIG_ENABLE_CAMERA
    camera_init();
#endif

    // my_deflog = esp_log_set_vprintf(my_i2clog);

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
#endif // CONFIG_PM_ENABLE

#ifdef CONFIG_ENABLE_WEBUI
    ESP_LOGI(TAG, "starting webui ...");
    webui_init();
#endif

    beep(0);
    
    //ESP_LOGI(TAG, "sleep 3 seconds ...");
    //vTaskDelay(3000 / portTICK_PERIOD_MS);

#if 0
    while(1)
    {
    ESP_LOGE(TAG, "ADC: ubat=%f usol=%f uch=%f Ibat=%f temp=%f",
        g_ubat,
        g_usolar,
        g_ucharge,
        g_ibat,
        g_temperature);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#endif

    ESP_LOGI(TAG, "starting console ...");
    console();

    ESP_LOGI(TAG, "... init done. free heap: %u", xPortGetFreeHeapSize());
}

/**
 * @brief wifi event handler
 * @param ctx
 * @param event
 * @return 
 */
static esp_err_t event_handler(void* ctx, system_event_t* event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGW(TAG, "SYSTEM_EVENT_STA_START");
        esp_wifi_connect();
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGW(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
        s_ip_addr = event->event_info.got_ip.ip_info.ip;
        s_ip_addr_changed = 1;

        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

        if(my_wifi_save_on_connected) {
            my_wifi_save_on_connected = false;
            nvs_handle my_handle;
            esp_err_t err = nvs_open("wifi", NVS_READWRITE, &my_handle);
            if(err == ESP_OK) {
            //size_t my_wifi_ssid_size = sizeof(my_wifi_ssid);
            //size_t my_wifi_psk_size = sizeof(my_wifi_psk);
            nvs_set_str(my_handle, "ssid", my_wifi_ssid);
            nvs_set_str(my_handle, "psk", my_wifi_psk);
            nvs_close(my_handle);
            printf("SSID and PSK stored to nvs\n");
            }
        }

        ESP_LOGI(TAG, "Wifi Connected");

#ifdef CONFIG_ENABLE_ROS2
        ros2node_start();
#endif

	break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGW(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        s_ip_addr.addr = 0;
        s_ip_addr_changed = 1;

#ifdef CONFIG_ENABLE_ROS2
        ros2node_stop();
#endif
        
	break;

    default:
	break;
    }
    return ESP_OK;
}

/**
 * @brief 
 */
static void initialise_wifi(void)
{
    tcpip_adapter_init();
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    //    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, my_wifi_ssid);
    strcpy((char*)wifi_config.sta.password, my_wifi_psk);
#ifdef CONFIG_PM_ENABLE
    wifi_config.sta.listen_interval = 50;
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "ros2mower");
#ifdef CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
#endif
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");
}


/**
 * EOF
 */