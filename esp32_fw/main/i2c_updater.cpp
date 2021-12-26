#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "i2chandler.h"
#include "nvs_flash.h"

static const char* TAG = "I2CUPD";

/* https://github.com/orempel/twiboot#virtual-bootloader-section
    TWI/I2C Protocol
    A TWI/I2C master can use the following protocol for accessing the bootloader.

    Function 	                TWI/I2C data 	                                        Comment

    Abort boot timeout 	        SLA+W, 0x00, STO
    Show bootloader version 	SLA+W, 0x01, SLA+R, {16 bytes}, STO 	                ASCII, not null terminated
    Start application 	        SLA+W, 0x01, 0x80, STO
    Read chip info 	            SLA+W, 0x02, 0x00, 0x00, 0x00, SLA+R, {8 bytes}, STO 	3byte signature, 1byte
   page size, 2byte flash size, 2byte eeprom size Read 1+ flash bytes 	    SLA+W, 0x02, 0x01, addrh, addrl, SLA+R, {*
   bytes}, STO Read 1+ eeprom bytes 	    SLA+W, 0x02, 0x02, addrh, addrl, SLA+R, {* bytes}, STO Write one flash page
   SLA+W, 0x02, 0x01, addrh, addrl, {* bytes}, STO 	    page size as indicated in chip info Write 1+ eeprom bytes
   SLA+W, 0x02, 0x02, addrh, addrl, {* bytes}, STO 	    write 0 < n < page size bytes at once

    SLA+R means Start Condition, Slave Address, Read Access

    SLA+W means Start Condition, Slave Address, Write Access

    STO means Stop Condition

    A flash page / eeprom write is only triggered after the Stop Condition. During the write process twiboot will NOT
   acknowledge its slave address.

    The multiboot_tool repository contains a simple linux application that uses this protocol to access the bootloader
   over linux i2c device.

    The ispprog programming adapter can also be used as a avr910/butterfly to twiboot protocol bridge.
 */

#include "/home/maik/workspace/ros/ROS2Mower/avr/BlinkPB2/main_bin.inc"

uint16_t avr_crc16_update(uint16_t crc, uint8_t a)
{
    int i;
    crc ^= a;
    for(i = 0; i < 8; ++i)
    {
        if(crc & 1)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc = (crc >> 1);
    }
    return crc;
}

uint8_t i2c_updater_once = 1;
void i2c_updater()
{
    if(i2c_updater_once == 1)
    {
        pm_ref();

        i2c_reset(0);
        i2c_int(0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        i2c_reset(1);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        uint8_t slave_addr = 0x10;
        if(i2cnode_check(slave_addr) == ESP_OK)
        {
            i2c_updater_once = 0;

            // read chip info: 3byte signature, 1byte page size, 2byte flash size, 2byte eeprom size
            // SLA+W, 0x02, 0x00, 0x00, 0x00, SLA+R, {8 bytes}, STO
            uint8_t buf_reqchipinfo[3] = {};
            if(i2cnode_write(slave_addr, 0x02, buf_reqchipinfo, sizeof(buf_reqchipinfo)) == ESP_OK)
            {
                uint8_t buf_chipinfo[10] = {};
                if(i2cnode_read(slave_addr, buf_chipinfo, sizeof(buf_chipinfo)) == ESP_OK)
                {
                    uint32_t avr_pagesize = buf_chipinfo[3];
                    uint32_t avr_flashlen = (buf_chipinfo[4] << 8) | (buf_chipinfo[5] << 0);
                    uint32_t avr_flashcrc = (buf_chipinfo[8] << 8) | (buf_chipinfo[9] << 0);

                    ESP_LOGE(TAG,
                        "i2c_updater() addr=0x%02x "
                        "chip info: sig=%06x pagesize=%d bootldrstart=%04x eepromsize=%04x crc16=%04x",
                        slave_addr, (buf_chipinfo[0] << 16) | (buf_chipinfo[1] << 8) | (buf_chipinfo[2] << 0),
                        avr_pagesize, avr_flashlen, (buf_chipinfo[6] << 8) | (buf_chipinfo[7] << 0), avr_flashcrc);

                    {
                        uint16_t avr_crc16 = 0;
                        for(uint32_t avr_addr = 0; avr_addr < avr_flashlen; avr_addr++)
                        {
                            if(avr_addr >= main_bin_len)
                            {
                                avr_crc16 = avr_crc16_update(avr_crc16, 0xff);
                            }
                            else
                            {
                                switch(avr_addr)
                                {
                                default:
                                    avr_crc16 = avr_crc16_update(avr_crc16, (uint8_t)main_bin[avr_addr]);
                                    break;
                                }
                            }
                        }

                        {
                            ESP_LOGE(TAG,
                                "i2c_updater() addr=0x%02x "
                                "FLASHCRC16=%04x FWCRC=%04x ...",
                                slave_addr, avr_flashcrc, avr_crc16);

                            if(avr_flashcrc != avr_crc16)
                            {
                                for(uint32_t wr_addr = 0; wr_addr < avr_flashlen; wr_addr += avr_pagesize)
                                {
                                    // write one flash page
                                    // SLA+W, 0x02, 0x01, addrh, addrl, {* bytes}, STO
                                    uint8_t buf_page[1 + 2 + 64] = {};
                                    buf_page[0] = 1;
                                    buf_page[1] = (wr_addr >> 8) & 0xff;
                                    buf_page[2] = (wr_addr >> 0) & 0xff;
                                    for(int i = 0; i < avr_pagesize; i++)
                                    {
                                        if((wr_addr + i) < main_bin_len)
                                        {
                                            buf_page[3 + i] = main_bin[wr_addr + i];
                                        }
                                        else
                                        {
                                            buf_page[3 + i] = 0xff;
                                        }
                                    }

                                    if(i2cnode_write(slave_addr, 0x02, buf_page, sizeof(buf_page), 500) == ESP_OK)
                                    {
                                        // i2cnode_write(slave_addr, 0x02, buf_reqchipinfo,
                                        // sizeof(buf_reqchipinfo)); i2cnode_read(slave_addr, buf_chipinfo,
                                        // sizeof(buf_chipinfo));
                                        avr_flashcrc = (buf_chipinfo[8] << 8) | (buf_chipinfo[9] << 0);
                                        ESP_LOGE(TAG, "i2c_updater() addr=0x%02x flash %04x OK", slave_addr, wr_addr);
                                    }
                                    else
                                    {
                                        ESP_LOGE(
                                            TAG, "i2c_updater() addr=0x%02x flash %04x ERROR", slave_addr, wr_addr);
                                        break;
                                    }
                                }
                                i2cnode_write(slave_addr, 0x02, buf_reqchipinfo, sizeof(buf_reqchipinfo));
                                i2cnode_read(slave_addr, buf_chipinfo, sizeof(buf_chipinfo));
                                ESP_LOGE(TAG, "i2c_updater() addr=0x%02x (FLASHCRC16=%04x FWCRC=%04x) DONE", slave_addr,
                                    avr_flashcrc, avr_crc16);
                            }
                        }
                    }
#if 0
                    {
                        uint8_t buf_start[1] = { 0x80 };
                        // start application
                        // SLA+W, 0x01, 0x80, STO
                        ESP_LOGE(TAG, "i2c_updater() addr=0x%02x START APP", slave_addr);
                        i2cnode_write(slave_addr, 0x01, buf_start, sizeof(buf_start));
                    }
#endif
                }
            }
        }

        i2c_int(1);

        pm_unref();
    }
    else
    {
        i2cnode_check(0x10);
    }
}