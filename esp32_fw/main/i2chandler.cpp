#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#include <exception>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "../components/http_server/my_http_server.h"
#include "../components/http_server/my_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "ota_server.h"

#include "i2chandler.h"

#include "ssd1306.h"
#include "ssd1306_default_if.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"

#include "console.h"

#ifdef CONFIG_ENABLE_ROS2
#include "ros2node.h"
#endif

#ifdef CONFIG_ENABLE_I2C_BNO055
#include "BNO055ESP32.h"
#endif

#ifdef CONFIG_ENABLE_I2C_OLED_SH1106
#include "font8x8_basic.h"
#include "sh1106.h"
#endif

#ifdef CONFIG_ENABLE_I2C_VL53L0X
#include "VL53L0X.h"
#define MAX_NUM_VL53L0X I2CROS2SENSORDATA_NUM_RANGE
#define CONST_VL53L0X_ANGLE_STEP 30
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* pdata, uint32_t count);
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* pdata, uint32_t count);
#endif

#ifdef CONFIG_ENABLE_I2C_VL53L1X
#include "VL53L1X.h"
#define MAX_NUM_VL53L1X 1
#define CONST_VL53L1X_ANGLE_STEP 30
#endif

#undef DISABLE_SHUTDOWN

uint8_t powerstate();

static const char* TAG = "I2C";

#define MOTOR_P (0.30 * 128)
#define MOTOR_I (0.01 * 128)
#define MOTOR_D (0.01 * 128)

volatile bool i2c_md_active = false;

static struct
{
    bool ready;

    struct
    {
        double linear_x;
        double linear_y;
        double angular_z;
        bool update;
    } cmd_vel;

    struct
    {
        double      speed;
        bool        update;
        uint8_t     pidtunestep;
        uint8_t     pidtunemotor;
        int64_t     pidtunetout; 
    } lawn_motor;

    int64_t last_cmd_vel_time;

    SemaphoreHandle_t sem;
    SemaphoreHandle_t sem_ist;

    uint16_t ubat_mV;
    int16_t isolar_mA;
    int16_t iout_mA;
    int16_t icharge_mA;

#ifdef CONFIG_ENABLE_ROS2
    I2CROS2SensorData ros2_data;
#endif

#ifdef CONFIG_ENABLE_I2C_VL53L0X
    VL53L0X* vl53l0x[MAX_NUM_VL53L0X];
    bool bl53l0xStarted[MAX_NUM_VL53L0X];
#endif

#ifdef CONFIG_ENABLE_I2C_VL53L1X
    VL53L1X* vl53l1x[MAX_NUM_VL53L1X];
#endif

} i2c_md = {};

#ifdef CONFIG_ENABLE_I2C_BNO055
BNO055* bno055 = NULL;
bno055_calibration_t bno055_calib;
uint8_t bno055_calib_other = 0;
#endif

extern ip4_addr_t s_ip_addr;
struct SSD1306_Device I2CDisplay;

extern float g_ubat;

/**
 * @brief
 * @return
 */
bool i2c_lock()
{
    if(i2c_md.sem == NULL)
    {
        ESP_LOGE(TAG, "i2c_lock => false");
        return false;
    }

    ESP_LOGD(TAG, "i2c_lock ...");
    if(xSemaphoreTake(i2c_md.sem, portMAX_DELAY))
    {
        ESP_LOGD(TAG, "i2c_lock => true");
        return true;
    }
    ESP_LOGE(TAG, "i2c_lock => false");
    return false;
}

/**
 * @brief
 */
void i2c_release()
{
    ESP_LOGD(TAG, "i2c_release");
    xSemaphoreGive(i2c_md.sem);
}

#ifdef CONFIG_ENABLE_ROS2
/**
 * @brief
 * @param data
 * @return
 */
I2CROS2SensorData* i2c_lock_data()
{
    if(i2c_md.ready == false)
    {
        return NULL;
    }
    if(i2c_lock() == true)
    {
        return &i2c_md.ros2_data;
    }
    return NULL;
}

/**
 * @brief
 */
void i2c_release_data()
{
    i2c_release();
}
#endif

/**
 * @brief
 * @param i2caddr
 * @return
 */
esp_err_t i2cnode_check(uint8_t i2caddr)
{
    esp_err_t err = ESP_FAIL;
    if(i2c_lock() == true)
    {
        i2c_cmd_handle_t CommandHandle = NULL;
        if((CommandHandle = i2c_cmd_link_create()) != NULL)
        {
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(CommandHandle);
            err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(1));
            i2c_cmd_link_delete(CommandHandle);
        }
        i2c_release();
        return err;
    }
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief
 * @param i2caddr
 * @param regaddr
 * @param buf
 * @param buflen
 * @return
 */
esp_err_t i2cnode_read(uint8_t i2caddr, uint8_t regaddr, uint8_t* buf, uint32_t buflen)
{
    esp_err_t err = ESP_OK;
    if(i2c_lock() == true)
    {
#if 0
        if( i2caddr == 0x10 )
        {
            i2c_set_period((i2c_port_t)I2C_BUS_PORT,
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW),
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW));
        }
        else
        {
              i2c_set_period((i2c_port_t)I2C_BUS_PORT,
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW),
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW));
        }
#endif
        i2c_cmd_handle_t CommandHandle = NULL;
        if((CommandHandle = i2c_cmd_link_create()) != NULL)
        {
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(CommandHandle, regaddr, true);
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
            i2c_master_read(CommandHandle, buf, buflen, I2C_MASTER_LAST_NACK);
            i2c_master_stop(CommandHandle);
            err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            if(err != ESP_OK)
            {
                i2c_setpin_boot(1);
                ESP_LOGE(TAG, "i2cnode_read i2caddr=0x%02x regaddr=0x%02x err=0x%02x", i2caddr, regaddr, err);
                i2c_setpin_boot(0);
            }
            i2c_cmd_link_delete(CommandHandle);
        }
        i2c_release();
        return err;
    }
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief
 * @param i2caddr
 * @param regaddr
 * @param buf
 * @param buflen
 * @return
 */
esp_err_t i2cnode_write(uint8_t i2caddr, uint8_t regaddr, uint8_t *buf, uint32_t buflen, int timeout)
{
    esp_err_t err = ESP_OK;
    if(i2c_lock() == true)
    {
#if 0
        if( i2caddr == 0x10 )
        {
            i2c_set_period((i2c_port_t)I2C_BUS_PORT,
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW),
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW));
        }
        else
        {
              i2c_set_period((i2c_port_t)I2C_BUS_PORT,
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW),
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW));
        }
#endif
        //i2c_set_timeout((i2c_port_t)I2C_BUS_PORT, I2C_APB_CLK_FREQ ); 

        i2c_cmd_handle_t CommandHandle = NULL;
        if((CommandHandle = i2c_cmd_link_create()) != NULL)
        {
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
            if( buf!=NULL && buflen==1 )
            {
                i2c_master_write_byte(CommandHandle, regaddr, true);
                i2c_master_write_byte(CommandHandle, buf[0], false);
            }
            else if( buf!=NULL && buflen!=0 )
            {
                i2c_master_write_byte(CommandHandle, regaddr, true);
                i2c_master_write(CommandHandle, buf, buflen, false);
            }
            else
            {
                i2c_master_write_byte(CommandHandle, regaddr, false);                
            }
            i2c_master_stop(CommandHandle);
            err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(timeout));
 #if 1            
            if(err != ESP_OK)
            {
                ESP_LOGE(TAG, "i2cnode_write i2caddr=0x%02x regaddr=0x%02x buf[%d]=%02x%02x%02x%02x err=0x%02x tout=%d|%d",
                    i2caddr, regaddr, buflen, buf[0], buf[1], buf[2], buf[3], err, timeout,pdMS_TO_TICKS(timeout));
            }
#endif            
            i2c_cmd_link_delete(CommandHandle);
        }
        i2c_release();
        return err;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t i2cnode_write(uint8_t i2caddr, uint8_t regaddr, uint8_t* buf, uint32_t buflen)
{
    return i2cnode_write(i2caddr,regaddr,buf,buflen,I2C_TIMEOUT_MS);
}

#ifdef CONFIG_ENABLE_I2C_VL53L0X
/**
 * @brief
 * @param esp_err
 * @return
 */
static VL53L0X_Error esp_to_vl53l0x_error(esp_err_t esp_err)
{
    switch(esp_err)
    {
    case ESP_OK:
        return VL53L0X_ERROR_NONE;
    case ESP_ERR_INVALID_ARG:
        return VL53L0X_ERROR_INVALID_PARAMS;
    case ESP_FAIL:
    case ESP_ERR_INVALID_STATE:
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_TIMEOUT:
        return VL53L0X_ERROR_TIME_OUT;
    default:
        return VL53L0X_ERROR_UNDEFINED;
    }
}

/**
 * Writes the supplied byte buffer to the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to uint8_t buffer containing the data to be written
 * @param   count     Number of bytes in the supplied byte buffer
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* pdata, uint32_t count)
{
    VL53L0X_Error err = esp_to_vl53l0x_error(i2cnode_write(Dev->i2c_address, index, pdata, count));
    if(err != VL53L0X_ERROR_NONE)
        ESP_LOGW("X", "VL53L0X_WriteMulti %d", err);
    return err;
}

/**
 * Reads the requested number of bytes from the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to the uint8_t buffer to store read data
 * @param   count     Number of uint8_t's to read
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* pdata, uint32_t count)
{
    VL53L0X_Error err = esp_to_vl53l0x_error(i2cnode_read(Dev->i2c_address, index, pdata, count));
    if(err != VL53L0X_ERROR_NONE)
        ESP_LOGW("X", "VL53L0X_ReadMulti %d", err);
    return err;
}
#endif /* CONFIG_ENABLE_I2C_VL53L0X */

/**
 * @brief
 * @param i2caddr
 * @param regaddr
 * @param buf
 * @param buflen
 * @return
 */
esp_err_t i2cnode_read(uint8_t i2caddr, uint8_t* buf, uint32_t buflen)
{
    esp_err_t err = ESP_OK;
    if(i2c_lock() == true)
    {
        i2c_cmd_handle_t CommandHandle = NULL;
        if((CommandHandle = i2c_cmd_link_create()) != NULL)
        {
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
            i2c_master_read(CommandHandle, buf, buflen, I2C_MASTER_LAST_NACK);
            i2c_master_stop(CommandHandle);
            err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            if(err != ESP_OK)
            {
                ESP_LOGE(TAG, "i2cnode_read i2caddr=0x%02x buflen=%d err=0x%02x", i2caddr, buflen, err);
            }
            i2c_cmd_link_delete(CommandHandle);
        }
        i2c_release();
    }
    return err;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
uint64_t i2cnode_get_u64(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    uint64_t v = 0;
    uint8_t tmpbuf[8] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 1 << 24;
    }
    v |= ((uint64_t)tmpbuf[7] << 56);
    v |= ((uint64_t)tmpbuf[6] << 48);
    v |= ((uint64_t)tmpbuf[5] << 40);
    v |= ((uint64_t)tmpbuf[4] << 32);
    v |= ((uint64_t)tmpbuf[3] << 24);
    v |= ((uint64_t)tmpbuf[2] << 16);
    v |= ((uint64_t)tmpbuf[1] << 8);
    v |= ((uint64_t)tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
uint32_t i2cnode_get_u32(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    uint32_t v = 0;
    uint8_t tmpbuf[8] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 2 << 24;
    }
    v |= ((uint32_t)tmpbuf[3] << 24);
    v |= ((uint32_t)tmpbuf[2] << 16);
    v |= ((uint32_t)tmpbuf[1] << 8);
    v |= ((uint32_t)tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
uint16_t i2cnode_get_u16(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    uint16_t v = 0;
    uint8_t tmpbuf[2] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 3 << 24;
    }
    v |= ((uint16_t)tmpbuf[1] << 8);
    v |= ((uint16_t)tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
uint8_t i2cnode_get_u8(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    uint16_t v = 0;
    uint8_t tmpbuf[1] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 4 << 24;
    }
    v |= (tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
int64_t i2cnode_get_i64(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    int64_t v = 0;
    uint8_t tmpbuf[8] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 5 << 24;
    }
    v |= ((int64_t)tmpbuf[7] << 56);
    v |= ((int64_t)tmpbuf[6] << 48);
    v |= ((int64_t)tmpbuf[5] << 40);
    v |= ((int64_t)tmpbuf[4] << 32);
    v |= ((int64_t)tmpbuf[3] << 24);
    v |= ((int64_t)tmpbuf[2] << 16);
    v |= ((int64_t)tmpbuf[1] << 8);
    v |= ((int64_t)tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
int16_t i2cnode_get_i16(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    int16_t v = 0;
    uint8_t tmpbuf[2] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 6 << 24;
    }
    v |= (tmpbuf[1] << 8);
    v |= (tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
int8_t i2cnode_get_i8(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    int8_t v = 0;
    uint8_t tmpbuf[2] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 7 << 24;
    }
    v |= (tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_u16(uint8_t i2caddr, uint8_t regaddr, uint16_t v) /* throw(int) */
{
    esp_err_t err;
    uint8_t tmpbuf[2] = {};
    tmpbuf[0] = (v >> 0) & 0xff;
    tmpbuf[1] = (v >> 8) & 0xff;
    if((err = i2cnode_write(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 8 << 24;
    }
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_u8(uint8_t i2caddr, uint8_t regaddr, uint8_t v) /* throw(int) */
{
    esp_err_t err;
    uint8_t tmpbuf[1] = {};
    tmpbuf[0] = (v >> 0) & 0xff;
    if((err = i2cnode_write(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 9 << 24;
    }
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_i16(uint8_t i2caddr, uint8_t regaddr, int16_t v) /* throw(int) */
{
    esp_err_t err;
    uint8_t tmpbuf[2] = {};
    tmpbuf[0] = (v >> 0) & 0xff;
    tmpbuf[1] = (v >> 8) & 0xff;
    if((err = i2cnode_write(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 10 << 24;
    }
}

#ifdef CONFIG_ENABLE_I2C_OLED
/**
 * @brief
 * @param Address
 * @param IsCommand
 * @param Data
 * @param DataLength
 * @return
 */
static bool I2CDefaultWriteBytes(int Address, bool IsCommand, const uint8_t* Data, size_t DataLength)
{
    static const int SSD1306_I2C_COMMAND_MODE = 0x80;
    static const int SSD1306_I2C_DATA_MODE = 0x40;
    uint8_t ModeByte = 0;
    ModeByte = (IsCommand == true) ? SSD1306_I2C_COMMAND_MODE : SSD1306_I2C_DATA_MODE;
    NullCheck(Data, return false);

    if((i2cnode_write(Address, ModeByte, (uint8_t*)Data, DataLength)) != ESP_OK)
    {
        return false;
    }

    return true;
}

/**
 * @brief
 * @param Display
 * @param Command
 * @return
 */
static bool I2CDefaultWriteCommand(struct SSD1306_Device* Display, SSDCmd Command)
{
    uint8_t CommandByte = (uint8_t)Command;

    NullCheck(Display, return false);
    return I2CDefaultWriteBytes(Display->Address, true, (const uint8_t*)&CommandByte, 1);
}

/**
 * @brief
 * @param Display
 * @param Data
 * @param DataLength
 * @return
 */
static bool I2CDefaultWriteData(struct SSD1306_Device* Display, const uint8_t* Data, size_t DataLength)
{
    NullCheck(Display, return false);
    NullCheck(Data, return false);

    return I2CDefaultWriteBytes(Display->Address, false, Data, DataLength);
}

/**
 * @brief
 * @param Display
 * @return
 */
static bool I2CDefaultReset(struct SSD1306_Device* Display)
{
    return true;
}
#endif // CONFIG_ENABLE_I2C_OLED

#ifdef CONFIG_ENABLE_I2C_OLED_SH1106
void sh1106_init()
{
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP_CTRL, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP_ON, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP_INVERSE, true); // reverse left-right mapping
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE_REVERSE, true); // reverse up-bottom mapping

    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);

    i2c_master_write_byte(cmd, 0x00, true); // reset column low bits
    i2c_master_write_byte(cmd, 0x10, true); // reset column high bits
    i2c_master_write_byte(cmd, 0xB0, true); // reset page
    i2c_master_write_byte(cmd, 0x40, true); // set start line
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_OFFSET, true);
    i2c_master_write_byte(cmd, 0x00, true);

    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, cmd, 10 / I2C_TIMEOUT_MS);
    if(espRc == ESP_OK)
    {
        ESP_LOGI(TAG, "OLED configured successfully");
    }
    else
    {
        ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
    }
    i2c_cmd_link_delete(cmd);
}

void sh1106_set_display_start_line(i2c_cmd_handle_t cmd, uint_fast8_t start_line)
{
    // REQUIRES:
    //   0 <= start_line <= 63
    if(start_line >= 0 && start_line <= 63)
    {
        i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_START_LINE | start_line, true);
    }
}

void task_sh1106_display_pattern(void* ignore)
{
    i2c_cmd_handle_t cmd;

    for(uint8_t i = 0; i < 8; i++)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
        i2c_master_write_byte(cmd, 0xB0 | i, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
        for(uint8_t j = 0; j < 132; j++)
        {
            i2c_master_write_byte(cmd, 0xFF >> (j % 8), true);
        }
        i2c_master_stop(cmd);
        i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, cmd, 10 / I2C_TIMEOUT_MS);
        i2c_cmd_link_delete(cmd);
    }
}

void task_sh1106_display_clear(void* ignore)
{
    i2c_cmd_handle_t cmd;

    uint8_t zero[132];
    memset(zero, 0, 132);
    for(uint8_t i = 0; i < 8; i++)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
        i2c_master_write_byte(cmd, 0xB0 | i, true);

        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
        i2c_master_write(cmd, zero, 132, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, cmd, 10 / I2C_TIMEOUT_MS);
        i2c_cmd_link_delete(cmd);
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, 0x00, true); // reset column
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_write_byte(cmd, 0xB0, true); // reset page
    i2c_master_stop(cmd);
    i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, cmd, 10 / I2C_TIMEOUT_MS);
    i2c_cmd_link_delete(cmd);
}

void task_sh1106_display_text(const void* arg_text)
{
    char* text = (char*)arg_text;
    uint8_t text_len = strlen(text);

    i2c_cmd_handle_t cmd;

    uint8_t cur_page = 0;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, 0x08, true); // reset column
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

    i2c_master_stop(cmd);
    i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, cmd, 10 / I2C_TIMEOUT_MS);
    i2c_cmd_link_delete(cmd);

    for(uint8_t i = 0; i < text_len; i++)
    {
        if(text[i] == '\n')
        {
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

            i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
            i2c_master_write_byte(cmd, 0x08, true); // reset column
            i2c_master_write_byte(cmd, 0x10, true);
            i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

            i2c_master_stop(cmd);
            i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, cmd, 10 / I2C_TIMEOUT_MS);
            i2c_cmd_link_delete(cmd);
        }
        else
        {
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

            i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
            i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

            i2c_master_stop(cmd);
            i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, cmd, 10 / I2C_TIMEOUT_MS);
            i2c_cmd_link_delete(cmd);
        }
    }
}

#endif

/**
 * @brief
 */
void i2c_handle_cmd_vel()
{
#ifdef WHEEL_DIAMETER
    if(i2c_md.cmd_vel.update == true)
    {
        i2c_md.cmd_vel.update = false;

        i2c_md.last_cmd_vel_time = esp_timer_get_time();

        double wheel0_speed = 0;
        double wheel1_speed = 0;

        double wheelcircumference = WHEEL_DIAMETER * M_PI;
        double wheeldistcircumference = WHEEL_DISTANCE * M_PI;

        // *** Compute the current wheel speeds ***
        // First compute the Robot's linear and angular speeds
        double xspeed = i2c_md.cmd_vel.linear_x;
        double yspeed = i2c_md.cmd_vel.linear_y;
        double linear_speed = sqrt(xspeed * xspeed + yspeed * yspeed);
        double angular_speed = i2c_md.cmd_vel.angular_z * wheeldistcircumference / (2 * M_PI);

        if(xspeed >= 0)
        {
            // robot is moving forward
            wheel0_speed = (linear_speed + angular_speed) / wheelcircumference;
            wheel1_speed = (linear_speed - angular_speed) / wheelcircumference;
        }
        else
        {
            // robot is backing up
            wheel0_speed = (-linear_speed + angular_speed) / wheelcircumference;
            wheel1_speed = (-linear_speed - angular_speed) / wheelcircumference;
        }

#if 0
        /* limit to max RPM
         */
        if( MOTOR_RPS(wheel0_speed) > MOTOR_MAX_RPS) {
            wheel1_speed *= MOTOR_MAX_RPS / MOTOR_RPS(wheel0_speed);
            wheel0_speed *= MOTOR_MAX_RPS / MOTOR_RPS(wheel0_speed);
        }
        if( MOTOR_RPS(wheel1_speed) > MOTOR_MAX_RPS) {
            wheel0_speed *= MOTOR_MAX_RPS / MOTOR_RPS(wheel1_speed);
            wheel1_speed *= MOTOR_MAX_RPS / MOTOR_RPS(wheel1_speed);
        }
#endif

#ifdef CONFIG_ENABLE_I2C_MOTOR
#if defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, MOTOR_RPS(wheel0_speed));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, MOTOR_RPS(wheel1_speed));

        ESP_LOGW(TAG, "i2c_handle_cmd_vel() %frps %frps %d %d", wheel0_speed, wheel1_speed, MOTOR_RPS(wheel0_speed),
            MOTOR_RPS(wheel1_speed));
#endif // defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
        uint8_t tmpbuf[4] = {};
        tmpbuf[0] = (MOTOR_RPS(wheel0_speed) >> 8) & 0xff;
        tmpbuf[1] = (MOTOR_RPS(wheel0_speed) >> 0) & 0xff;
        tmpbuf[2] = (MOTOR_RPS(wheel1_speed) >> 8) & 0xff;
        tmpbuf[3] = (MOTOR_RPS(wheel1_speed) >> 0) & 0xff;
        i2cnode_write(ZUMO_I2C_ADDR, CMD_MOTORS_SET_SPEED, tmpbuf, sizeof(tmpbuf));
        ESP_LOGD(TAG, "i2c_handle_cmd_vel() %f rps %f rps %d %d", wheel0_speed, wheel1_speed, MOTOR_RPS(wheel0_speed),
            MOTOR_RPS(wheel1_speed));
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO
#endif
    }
#endif /* WHEEL_DIAMETER */
}

/**
 * @brief
 * @param x
 * @param y
 * @param z
 */
void i2c_set_cmd_vel(double x, double y, double z)
{
    i2c_md.cmd_vel.linear_x = x;
    i2c_md.cmd_vel.linear_y = y;
    i2c_md.cmd_vel.angular_z = z;
    i2c_md.cmd_vel.update = true;
}

/**
 * @brief
 * @return
 */
bool i2c_cmd_vel_active()
{
    if((i2c_md.last_cmd_vel_time != 0) && ((i2c_md.last_cmd_vel_time + 1000000 * 60) > esp_timer_get_time()))
    {
        return true;
    }

    return (i2c_md.cmd_vel.linear_x != 0.0) || (i2c_md.cmd_vel.linear_y != 0.0) || (i2c_md.cmd_vel.angular_z != 0.0);
}

void i2c_set_lawn_motor_speed(double speed)
{
    i2c_md.lawn_motor.speed = speed;
    i2c_md.lawn_motor.update = true;
}

void i2c_start_pid_tuning(int motor)
{
    ESP_LOGE(TAG, "i2c_start_pid_tuning m=%d", motor);
    i2c_md.lawn_motor.pidtunemotor = motor;
    i2c_md.lawn_motor.pidtunestep = 1;
    i2c_md.lawn_motor.pidtunetout = 0;
}

void i2c_handle_pid_tuning()
{
#ifdef CONFIG_ENABLE_I2C_MOTOR
#if defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
    if(i2c_md.lawn_motor.pidtunemotor == 0 &&
        i2c_md.lawn_motor.pidtunestep != 0)
    {
        int64_t t_us = esp_timer_get_time(); /* time in us */
        if( i2c_md.lawn_motor.pidtunetout < t_us )
        {
            ESP_LOGE(TAG, "i2c_md.lawn_motor.pidtunestep m=%d", i2c_md.lawn_motor.pidtunemotor);
            if(i2c_md.lawn_motor.pidtunestep == 1)
            {
                // #define TWI_REG_U8_MA_DIR (((0x20 + 6) << 8) | 1)
                // #define TWI_REG_U8_MA_PWM (((0x20 + 7) << 8) | 1)
                // #define TWI_REG_U16_MODE ((8 << 8) | 2)
                i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x08, 3 /*MODE_PWM*/);
                i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x26, 1 /*DIR*/);
                i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x27, 50 /*PWM*/);
                i2c_md.lawn_motor.pidtunetout = t_us + 5000000UL;
                i2c_md.lawn_motor.pidtunestep++;
            }
            else if(i2c_md.lawn_motor.pidtunestep == 2)
            {
                // #define TWI_REG_U8_MA_DIR (((0x20 + 6) << 8) | 1)
                // #define TWI_REG_U8_MA_PWM (((0x20 + 7) << 8) | 1)
                // #define TWI_REG_U16_MODE ((8 << 8) | 2)
                i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x08, 3 /*MODE_PWM*/);
                i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x26, 1 /*DIR*/);
                i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x27, 0 /*PWM*/);
                i2c_md.lawn_motor.pidtunetout = t_us + 5000000UL;
                i2c_md.lawn_motor.pidtunestep++;
            }
            else
            {
                //#define TWI_REG_U16_MODE ((8 << 8) | 2)
                i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x08, 2 /*MODE_PID*/);
                i2c_md.lawn_motor.pidtunestep = 1;
            }
        }
    }
#endif // defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOW#endif
#endif
}

void i2c_set_motor_pid(int motor, int p, int i , int d)
{
#ifdef CONFIG_ENABLE_I2C_MOTOR
#if defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
    if( motor == 0 )
    {
        // #define TWI_REG_S16_PID_K_PID ((0x10 << 8) | 6)
        uint8_t tmp[6] = {};
        #define SCALING_FACTOR  128
        int16_t k_p = (int16_t)((p/1000.0) * SCALING_FACTOR);
        int16_t k_i = (int16_t)((i/1000.0) * SCALING_FACTOR);
        int16_t k_d = (int16_t)((d/1000.0) * SCALING_FACTOR);
        tmp[0] = (k_p>>8)&0xff;
        tmp[1] = (k_p>>0)&0xff;
        tmp[2] = (k_i>>8)&0xff;
        tmp[3] = (k_i>>0)&0xff;
        tmp[4] = (k_d>>8)&0xff;
        tmp[5] = (k_d>>0)&0xff;
        i2cnode_write(MOTORNODE_I2C_ADDR, 0x10, tmp, sizeof(tmp)); // trigger motor watchdog
    }
#endif // defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
#endif
}

#if 0
/**
 * @brief
 */
void i2cnode_init_motor()
{
#if defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
#ifdef CONFIG_ENABLE_I2C_POWER
    try
    {
        i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 60);    // update TWI_MEM_SHDWNCNT
        i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x18, 13000); // stay on with ubat>12.5V
    }
    catch(int err)
    {
        i2c_setpin_boot(1);
        ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
        i2c_setpin_boot(0);
    }
#endif
#ifdef CONFIG_ENABLE_I2C_MOTOR
    try
    {
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, (int16_t)MOTOR_P);
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x22, (int16_t)MOTOR_I);
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x24, (int16_t)MOTOR_D);
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x08, 2);
        // i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, MOTOR_RPM(MOTOR_START_RPM_L));
        // i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, MOTOR_RPM(MOTOR_START_RPM_R));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, MOTOR_RPM(MOTOR_START_RPM_L));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, MOTOR_RPM(MOTOR_START_RPM_R));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x0E, 0); /* int pulse */
    }
    catch(int err)
    {
        i2c_setpin_boot(1);
        ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
        i2c_setpin_boot(0);
    }
#endif
#endif
}
#endif

uint16_t i2c_ubat_mV()
{
    return i2c_md.ubat_mV;
}

int16_t i2c_isolar_mA()
{
    return i2c_md.isolar_mA;
}

int16_t i2c_iout_mA()
{
    return i2c_md.iout_mA;
}

int16_t i2c_icharge_mA()
{
    return i2c_md.icharge_mA;
}

/**
 * @brief
 * @param enc_l
 * @param enc_r
 */
#ifdef CONFIG_ENABLE_ROS2
void i2c_handle_encoder(int16_t enc_l, int16_t enc_r)
{
#ifdef WHEEL_DIAMETER
    static int64_t time_ = 0;
    int64_t time = esp_timer_get_time(); /* time in us */

    // if(enc_l != 0 || enc_r != 0)
    {
        double dt = 0.000001 * (time - time_); /* s */
        time_ = time;

        /*
         * https://answers.ros.org/question/207392/generating-odom-message-from-encoder-ticks-for-robot_pose_ekf/
         * http://www.seattlerobotics.org/encoder/200610/Article3/IMU%20Odometry,%20by%20David%20Anderson.htm
         */
        static double x = 0;
        static double y = 0;
        static double th = 0;

        /*
         * Pololu Wheel Encoder
         */
        double DistancePerCount = (M_PI * WHEEL_DIAMETER) / (MOTOR_GEAR_N); /* [m] */

        // extract the wheel velocities from the tick signals count
        double v_left = (enc_l * DistancePerCount) / dt;  /* m/s */
        double v_right = (enc_r * DistancePerCount) / dt; /* m/s */

        double vx = ((v_right + v_left) / 2); /* m/s */
        double vy = 0;                        /* m/s */
        double vth = ((v_right - v_left) /* m/s */ / WHEEL_DISTANCE /* m */);

        double delta_x = (vx * cos(th)) * dt;
        double delta_y = (vx * sin(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        // motor_speed_l = v_left;
        // motor_speed_r = v_right;
        // cmd_vel_motor_update();

        ESP_LOGD(TAG, "vHandleEncoderSteps(%d,%d) d=%f dT=%f v=(%f,%f) x=%f y=%f th=%f ...", enc_l, enc_r,
            DistancePerCount, dt, v_left, v_right, x, y, th);

#ifdef I2CROS2SENSORDATA_USE_NAV_MSG_ODOMETRY
        i2c_md.ros2_data.msg_odom_tf.pose.pose.position.x = x;
        i2c_md.ros2_data.msg_odom_tf.pose.pose.position.y = y;
        i2c_md.ros2_data.msg_odom_tf.pose.pose.position.z = 0;
        i2c_md.ros2_data.msg_odom_tf.pose.pose.orientation.w = th;

        i2c_md.ros2_data.msg_odom_tf.twist.twist.angular.x = vx;
        i2c_md.ros2_data.msg_odom_tf.twist.twist.angular.y = vy;
        i2c_md.ros2_data.msg_odom_tf.twist.twist.angular.z = vth;

        struct timespec tv = { 0 };
        clock_gettime(CLOCK_MONOTONIC, &tv);
        i2c_md.ros2_data.msg_odom_tf.header.stamp.nanosec = tv.tv_nsec;
        i2c_md.ros2_data.msg_odom_tf.header.stamp.sec = tv.tv_sec;

        i2c_md.ros2_data.msg_odom_tf_valid = true;
        // RCSOFTCHECK(rcl_publish(&i2c_md.ros2_data.pub_odom_tf, &i2c_md.ros2_data.msg_odom_tf, NULL));
#endif
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_POSE_2D
        i2c_md.ros2_data.msg_pose_2d.x = x;
        i2c_md.ros2_data.msg_pose_2d.y = y;
        i2c_md.ros2_data.msg_pose_2d.theta = th;
        i2c_md.ros2_data.msg_pose_2d_valid = true;
#endif
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_TF

        i2c_md.ros2_data.msg_tf.transforms.data[0].transform.translation.x = x;
        i2c_md.ros2_data.msg_tf.transforms.data[0].transform.translation.y = y;
        i2c_md.ros2_data.msg_tf.transforms.data[0].transform.translation.z = 0;
        i2c_md.ros2_data.msg_tf.transforms.data[0].transform.rotation.x = vx;
        i2c_md.ros2_data.msg_tf.transforms.data[0].transform.rotation.y = vy;
        i2c_md.ros2_data.msg_tf.transforms.data[0].transform.rotation.w = 0;
        i2c_md.ros2_data.msg_tf.transforms.data[0].transform.rotation.w = vth;
        i2c_md.ros2_data.msg_tf_valid = true;
#endif

#if 0
        i2c_md.ros2_data.msg_pose.x = 0.0;
        i2c_md.ros2_data.msg_pose.y = 0.0;
        i2c_md.ros2_data.msg_pose.z += delta_th;

		//RCSOFTCHECK(rcl_publish(&i2c_md.ros2_data.pub_pose, &i2c_md.ros2_data.msg_pose, NULL));

        i2c_md.ros2_data.msg_odom.x += delta_x;
        i2c_md.ros2_data.msg_odom.y += delta_y;
        i2c_md.ros2_data.msg_odom.z = 0.0;

		//RCSOFTCHECK(rcl_publish(&i2c_md.ros2_data.pub_odom, &i2c_md.ros2_data.msg_odom, NULL));
#endif
        ESP_LOGD(TAG, "vHandleEncoderSteps() ... done");
    }
#endif /* WHEEL_DIAMETER */
}
#endif

#if defined(CONFIG_ENABLE_I2C_VL53L0X) || defined(CONFIG_ENABLE_I2C_VL53L1X)
void i2c_lidar_init()
{
#if 0
    while(1) {
        for(int i = 0; i < MAX_NUM_VL53L0X; i++) {
            uint16_t reg = i2cnode_get_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/);
            reg &= ~(1 << i);
            i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
            vTaskDelay(10 * (1 + i) / portTICK_PERIOD_MS);
            reg |= (1 << i);
            // ESP_LOGW(TAG, "I2CThread() reset vl53l0x %d reg=%04x", i, reg);
            i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
#endif
#ifdef CONFIG_ENABLE_I2C_VL53L0X
    for(int i = 0; i < MAX_NUM_VL53L0X; i++)
    {
        uint16_t reg = i2cnode_get_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/);
        reg &= ~(1 << i);
        i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        reg |= (1 << i);
        // ESP_LOGW(TAG, "I2CThread() reset vl53l0x %d reg=%04x", i, reg);
        i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
        vTaskDelay((10) / portTICK_PERIOD_MS);
        if(i2cnode_check(VL53L0X_I2C_ADDRESS_DEFAULT) == ESP_OK)
        {
            ESP_LOGW(TAG, "i2c_lidar_init() vl53l0x %d ...", i);
            // if(i2c_lock()) {
            i2c_md.vl53l0x[i] = new VL53L0X((i2c_port_t)I2C_BUS_PORT);
            if(i2c_md.vl53l0x[i] != NULL)
            {
                if(i2c_md.vl53l0x[i]->init(I2C_VL53LXY_ADDR + i))
                {
                    ESP_LOGW(TAG, "i2c_lidar_init() vl53l0x %d initialized", i);
                    // i2c_md.vl53l0x[i]->start();
                }
                else
                {
                    delete i2c_md.vl53l0x[i];
                    i2c_md.vl53l0x[i] = NULL;
                    ESP_LOGW(TAG, "i2c_lidar_init() vl53l0x %d error", i);
                }
            }
            // i2c_release();
            //}
        }
    }
#endif /* CONFIG_ENABLE_I2C_VL53L0X */

#ifdef CONFIG_ENABLE_I2C_VL53L1X
    for(int i = 0; i < MAX_NUM_VL53L1X; i++)
    {
        uint16_t reg = i2cnode_get_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/);
        reg &= ~(1 << i);
        i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
        vTaskDelay(2 / portTICK_PERIOD_MS);
        reg |= (1 << i);
        // ESP_LOGW(TAG, "I2CThread() reset vl53l1x %d reg=%04x", i, reg);
        i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
        vTaskDelay((10) / portTICK_PERIOD_MS);
        if(i2cnode_check(VL53L0X_I2C_ADDRESS_DEFAULT) == ESP_OK)
        {
            ESP_LOGW(TAG, "i2c_lidar_init() vl53l0x %d ...", i);
            if(i2c_lock() == true)
            {
                i2c_md.vl53l1x[i] = new VL53L1X(I2C_BUS_PORT);
                if(i2c_md.vl53l1x[i] != NULL)
                {
                    if(i2c_md.vl53l1x[i]->init())
                    {
                        // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
                        // You can change these settings to adjust the performance of the sensor, but
                        // the minimum timing budget is 20 ms for short distance mode and 33 ms for
                        // medium and long distance modes. See the VL53L1X datasheet for more
                        // information on range and timing limits.
                        // Start continuous readings at a rate of one measurement every 50 ms (the
                        // inter-measurement period). This period should be at least as long as the
                        // timing budget.
                        i2c_md.vl53l1x[i]->setDistanceMode(VL53L1X::Medium);
                        i2c_md.vl53l1x[i]->setMeasurementTimingBudget(30000);
                        i2c_md.vl53l1x[i]->startContinuous(30);
                        ESP_LOGW(TAG, "i2c_lidar_init() vl53l1x %d initialized", i);
                    }
                    else
                    {
                        delete i2c_md.vl53l1x[i];
                        i2c_md.vl53l1x[i] = NULL;
                        ESP_LOGW(TAG, "i2c_lidar_init() vl53l1x %d error", i);
                    }
                }
                i2c_release();
            }
        }
    }
#endif /* CONFIG_ENABLE_I2C_VL53L1X */
}

void i2c_setpin_boot(int level);

void i2c_lidar_handle()
{
    if(i2c_md.ros2_data.msg_range_trigger == 1)
    {
        ESP_LOGD(TAG, "i2c_lidar_handle() start");
        i2c_md.ros2_data.msg_range_trigger = 0;
        try
        {
#ifdef CONFIG_ENABLE_I2C_VL53L0X
            for(int i = 0; i < MAX_NUM_VL53L0X; i++)
            {
                if(i2c_md.vl53l0x[i] != NULL)
                {
                    i2c_md.ros2_data.msg_range_valid[i] = false;
                    i2c_md.vl53l0x[i]->start();
                }
            }
#endif /* CONFIG_ENABLE_I2C_VL53L0X */

#ifdef CONFIG_ENABLE_I2C_VL53L1X
            for(int i = 0; i < MAX_NUM_VL53L1X; i++)
            {
                if(i2c_md.vl53l1x[i] != NULL)
                {
                }
            }
#endif
        }
        catch(int err)
        {
            ESP_LOGE(TAG, "i2c_lidar_handle() #1 I2C exception err=0x%02x", err);
        }
    }
    else
    {
        try
        {
#ifdef CONFIG_ENABLE_I2C_VL53L0X
            for(int i = 0; i < MAX_NUM_VL53L0X; i++)
            {
                if(i2c_md.vl53l0x[i] != NULL && i2c_md.ros2_data.msg_range_valid[i] == false)
                {
                    i2c_md.ros2_data.msg_range[i].range = 0;
                    uint16_t vl53l0x_data = 0;
                    if(i2c_md.vl53l0x[i]->readData(&vl53l0x_data) == true)
                    {
                        struct timespec tv = {};
                        clock_gettime(CLOCK_MONOTONIC, &tv);
                        i2c_md.ros2_data.msg_range[i].header.stamp.nanosec = tv.tv_nsec;
                        i2c_md.ros2_data.msg_range[i].header.stamp.sec = tv.tv_sec;
                        i2c_md.ros2_data.msg_range[i].range = 0.001 * vl53l0x_data;
                        i2c_md.ros2_data.msg_range[i].min_range = 0.01;
                        i2c_md.ros2_data.msg_range[i].max_range = 1.00;
                        i2c_md.ros2_data.msg_range[i].field_of_view = 30.0 * 2 * M_PI / 360;
                        i2c_md.ros2_data.msg_range[i].radiation_type = sensor_msgs__msg__Range__INFRARED;
                        i2c_md.ros2_data.msg_range_valid[i] = true;
                        ESP_LOGD(TAG, "i2c_lidar_handle %d range_mm=%f", i, i2c_md.ros2_data.msg_range[i].range);
                    }
                }
            }
#endif /* CONFIG_ENABLE_I2C_VL53L0X */

#ifdef CONFIG_ENABLE_I2C_VL53L1X
            for(int i = 0; i < MAX_NUM_VL53L1X; i++)
            {
                if(i2c_md.vl53l1x[i] != NULL)
                {
                    if(i2c_lock() == true)
                    {
                        if(i2c_md.vl53l1x[i]->dataReady())
                        {
                            i2c_md.vl53l1x[i]->read();
                            if(i2c_md.vl53l1x[i]->ranging_data.range_status == VL53L1X::RangeValid)
                            {
                                ESP_LOGD(TAG, "i2c_lidar_handle %d range_mm=%d (%f,%f)", i,
                                    i2c_md.vl53l1x[i]->ranging_data.range_mm,
                                    i2c_md.vl53l1x[i]->ranging_data.peak_signal_count_rate_MCPS,
                                    i2c_md.vl53l1x[i]->ranging_data.ambient_count_rate_MCPS);
                                u16RangeMilliMeter[i] = i2c_md.vl53l1x[i]->ranging_data.range_mm;
                            }
                        }
                    }
                }
            }
#endif
        }
        catch(int err)
        {
            ESP_LOGE(TAG, "i2c_lidar_handle() #2 I2C exception err=0x%02x", err);
        }
    }
}
#endif

#define FRAME_ID_CAPACITY 50

uint8_t i2c_scan_skip[128 / 8] = {};
void i2c_scan()
{
    ESP_LOGW(TAG, "i2c_task() i2cdetect ...");
    uint32_t flags_available[256 / 32] = {};
    uint32_t flags_timeout[256 / 32] = {};
    for(int i = 0; i < 128; i += 16)
    {
        for(int j = 0; j < 16; j++)
        {
            int k = i + j;
            if(k >= 8 && k < 120 && (i2c_scan_skip[k / 8] & (1 << (k & 7))) == 0)
            {
                esp_err_t ret = i2cnode_check(k);
                if(ret == ESP_OK)
                {
                    flags_available[i / 32] |= (1 << (k & 31));
                }
                else if(ret == ESP_ERR_TIMEOUT)
                {
                    flags_timeout[i / 32] |= (1 << (k & 31));
                }
            }
        }
    }
    ESP_LOGW(TAG, "i2c_task()      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
    for(int i = 0; i < 128; i += 16)
    {
        char line[120] = {};
        sprintf(&line[strlen(line)], "%02x: ", i);
        for(int j = 0; j < 16; j++)
        {
            int k = i + j;
            if(k >= 8 && k < 120)
            {
                if((flags_available[i / 32] & (1 << (k & 31))) != 0)
                {
                    sprintf(&line[strlen(line)], "%02x ", k);
                }
                else if((flags_timeout[i / 32] & (1 << (k & 31))) != 0)
                {
                    sprintf(&line[strlen(line)], "UU ");
                }
                else
                {
                    sprintf(&line[strlen(line)], "-- ");
                }
            }
            else
            {
                sprintf(&line[strlen(line)], "XX ");
            }
        }
        ESP_LOGW(TAG, "i2c_task() %s", line);
    }
    ESP_LOGW(TAG, "i2c_task() i2cdetect ... done");
}

#if 0
static void i2c_loop(void* param)
{   
    pm_ref();
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
#ifdef CONFIG_ENABLE_I2C_POWER
    i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, 0x0000);
#endif
#endif

    ESP_LOGW(TAG, "i2c_task() scan ...");
    i2c_scan();

#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
    {
        uint8_t tmpbuf[5] = {};
        tmpbuf[0] = 0;
        tmpbuf[1] = 1000;
        tmpbuf[2] = 0;
        tmpbuf[3] = 1000;
        tmpbuf[4] = 10;
        i2cnode_write(ZUMO_I2C_ADDR, CMD_BEEP, tmpbuf, sizeof(tmpbuf));
    }
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO

#ifdef CONFIG_ENABLE_I2C_OLED_SH1106
    if(i2c_lock() == true)
    {
        sh1106_init();
        task_sh1106_display_pattern(NULL);
        i2c_release();
    }
#endif

#ifdef CONFIG_ENABLE_ROS2
    std_msgs__msg__Float32__init(&i2c_md.ros2_data.msg_ubat);
    i2c_md.ros2_data.msg_ubat_valid = false;

    // init odom tf message ...
#ifdef I2CROS2SENSORDATA_USE_NAV_MSG_ODOMETRY
    nav_msgs__msg__Odometry__init(&i2c_md.ros2_data.msg_odom_tf);
    i2c_md.ros2_data.msg_odom_tf_valid = false;

    i2c_md.ros2_data.msg_odom_tf.header.frame_id.data = (char*)malloc(FRAME_ID_CAPACITY * sizeof(char));
    // sprintf(i2c_md.ros2_data.msg_odom_tf.header.frame_id.data, "/odom_link");
    sprintf(i2c_md.ros2_data.msg_odom_tf.header.frame_id.data, "odom");
    i2c_md.ros2_data.msg_odom_tf.header.frame_id.size = strlen(i2c_md.ros2_data.msg_odom_tf.header.frame_id.data);
    i2c_md.ros2_data.msg_odom_tf.header.frame_id.capacity = FRAME_ID_CAPACITY;

    i2c_md.ros2_data.msg_odom_tf.child_frame_id.data = (char*)malloc(FRAME_ID_CAPACITY * sizeof(char));
    // sprintf(i2c_md.ros2_data.msg_odom_tf.child_frame_id.data, "/base_link");
    sprintf(i2c_md.ros2_data.msg_odom_tf.child_frame_id.data, "base_footprint");
    i2c_md.ros2_data.msg_odom_tf.child_frame_id.size = strlen(i2c_md.ros2_data.msg_odom_tf.child_frame_id.data);
    i2c_md.ros2_data.msg_odom_tf.child_frame_id.capacity = FRAME_ID_CAPACITY;
#endif
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_POSE_2D
    geometry_msgs__msg__Pose2D__init(&i2c_md.ros2_data.msg_pose_2d);
    i2c_md.ros2_data.msg_pose_2d_valid = false;
    i2c_md.ros2_data.msg_pose_2d.x = 0;
    i2c_md.ros2_data.msg_pose_2d.y = 0;
    i2c_md.ros2_data.msg_pose_2d.theta = 0;
#endif
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_TF
    geometry_msgs__msg__TransformStamped__Sequence__init(&i2c_md.ros2_data.msg_tf.transforms, 1);

    sprintf(i2c_md.ros2_data.msg_tf.transforms.data[0].header.frame_id.data, "odom");
    i2c_md.ros2_data.msg_tf.transforms.data[0].header.frame_id.size =
        strlen(i2c_md.ros2_data.msg_odom_tf.header.frame_id.data);
    i2c_md.ros2_data.msg_tf.transforms.data[0].header.frame_id.capacity = FRAME_ID_CAPACITY;

    i2c_md.ros2_data.msg_tf.transforms.data[0].child_frame_id.data = (char*)malloc(FRAME_ID_CAPACITY * sizeof(char));
    // sprintf(i2c_md.ros2_data.msg_tf.child_frame_id.data, "/base_link");
    sprintf(i2c_md.ros2_data.msg_tf.transforms.data[0].child_frame_id.data, "base_footprint");
    i2c_md.ros2_data.msg_tf.transforms.data[0].child_frame_id.size =
        strlen(i2c_md.ros2_data.msg_odom_tf.child_frame_id.data);
    i2c_md.ros2_data.msg_tf.transforms.data[0].child_frame_id.capacity = FRAME_ID_CAPACITY;

    // geometry_msgs__msg__Quaternion__init(&i2c_md.ros2_data.msg_tf.transforms.data[0].transform.rotation);

    i2c_md.ros2_data.msg_tf_valid = false;
#endif

    // init imu message ...
    sensor_msgs__msg__Imu__init(&i2c_md.ros2_data.msg_imu);
    i2c_md.ros2_data.msg_imu_valid = false;

    i2c_md.ros2_data.msg_imu.header.frame_id.data = (char*)malloc(FRAME_ID_CAPACITY * sizeof(char));
    sprintf(i2c_md.ros2_data.msg_imu.header.frame_id.data, "imu_link");
    i2c_md.ros2_data.msg_imu.header.frame_id.size = strlen(i2c_md.ros2_data.msg_imu.header.frame_id.data);
    i2c_md.ros2_data.msg_imu.header.frame_id.capacity = FRAME_ID_CAPACITY;

    // init range messages ...
    for(int i = 0; i < I2CROS2SENSORDATA_NUM_RANGE; i++)
    {
        sensor_msgs__msg__Range__init(&i2c_md.ros2_data.msg_range[i]);
        i2c_md.ros2_data.msg_range_valid[i] = false;

        i2c_md.ros2_data.msg_range[i].header.frame_id.data = (char*)malloc(FRAME_ID_CAPACITY * sizeof(char));
        sprintf(i2c_md.ros2_data.msg_range[i].header.frame_id.data, "range_%d_link", i);
        i2c_md.ros2_data.msg_range[i].header.frame_id.size = strlen(i2c_md.ros2_data.msg_range[i].header.frame_id.data);
        i2c_md.ros2_data.msg_range[i].header.frame_id.capacity = FRAME_ID_CAPACITY;
    }
#endif

#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
    try
    {
#ifdef CONFIG_ENABLE_I2C_POWER
        i2cnode_set_u16(STM32_I2C_ADDR, 0x60 /*I2C_REG_TB_U16_TON_TOUT*/,
            PWR_KEEP_ALIVE_DELAY); // shutdown in 10 seconds
        i2cnode_set_u16(STM32_I2C_ADDR, 0x68 /*I2C_REG_TB_U16_TOFF_PERIOD*/, 600);

        i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, 0x0000);
        // i2cnode_set_u16(STM32_I2C_ADDR, 0x64 /*I2C_REG_TB_U16_TON_WDG*/, 10); // set STM32 watchdog timeout to 10
        // seconds ...
#endif
        vTaskDelay(50 / portTICK_PERIOD_MS);
#ifdef CONFIG_ENABLE_I2C_BNO055
        // release BNO055
        {
            uint16_t reg = i2cnode_get_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/);
            reg |= (1 << 15);
            ESP_LOGW(TAG, "i2c_task() rst-reg=0x%04x", reg);
            i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            {
                int retry = 20;
                while(i2cnode_check(I2C_BNO055_ADDR) != ESP_OK && retry--)
                {
                    ESP_LOGW(TAG, "i2c_task() waiting for bno055 ...");
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);
            i2c_init_bno055();
        }
#endif
#if defined(CONFIG_ENABLE_I2C_VL53L0X) || defined(CONFIG_ENABLE_I2C_VL53L1X)
        i2c_lidar_init();
#endif
    }
    catch(int err)
    {
        ESP_LOGE(TAG, "i2c_task() I2C exception err=0x%08x", err);
        pm_unref();
        while(1)
        {
            ESP_LOGE(TAG, "i2c_task() Can't init ROS2Zumo, waiting for restart ...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO

#ifdef CONFIG_ENABLE_I2C_OLED
    SSD1306_Init_I2C(
        &I2CDisplay, 128, 64, OLED_I2C_ADDR >> 1, -1, I2CDefaultWriteCommand, I2CDefaultWriteData, I2CDefaultReset);
    SSD1306_DisplayOn(&I2CDisplay);
    SSD1306_SetContrast(&I2CDisplay, 255);
    SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);
    SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_16x31);
    SSD1306_FontDrawString(&I2CDisplay, (128 - 8 * 16) / 2, (64 - 31) / 2, "Init ...", SSD_COLOR_WHITE);
    SSD1306_Update(&I2CDisplay);
    int64_t next_oled_update = esp_timer_get_time() + 1000000UL;
#endif // CONFIG_ENABLE_I2C_OLED

    // vTaskDelay(3000 / portTICK_PERIOD_MS);

    i2cnode_init_motor();

#ifdef CONFIG_ENABLE_I2C_LDS
    ESP_LOGI(TAG, "i2c_task() I2C enable LDS ...");
    i2cnode_set_u8(STM32_I2C_ADDR, 0x48 /*I2C_REG_TB_U8_LDS_SPEED*/, 1);
#endif

    pm_unref();

#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
#ifdef CONFIG_ENABLE_I2C_OLED_SH1106
    if(i2c_lock() == true)
    {
        task_sh1106_display_clear(NULL);
        task_sh1106_display_text("Hello!\nMultiline OK!\nAnother line.");
        i2c_release();
    }
#endif
#endif

    // i2c_scan(); // a i2c scan makes the bno055 non working. dont now why

    i2c_md.ready = true;
    while(1)
    {
        /*
         * WHILE ...
         */
        bool keepon = false;
        bool shutdown = false;

#ifdef DISABLE_SHUTDOWN
        keepon = true;
#endif

        pm_ref();

#ifdef CONFIG_ENABLE_I2C_LDS
        {
            static uint16_t _lds_cnt = 0;
            uint16_t lds_cnt = 0;
            uint8_t buf[2+360*4] = {};
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
            i2cnode_set_u8(STM32_I2C_ADDR, 0x70, 0x00);
            if(i2cnode_read(STM32_I2C_ADDR, 0x70, buf, 2+360*4) == ESP_OK)
            {
                lds_cnt = buf[0]<<8 | buf[1];            
                ESP_LOGW(TAG, "lds.cnt=0x%04x", lds_cnt);
                if( lds_cnt != _lds_cnt )
                {
                    _lds_cnt = lds_cnt;
                }
            }
#endif
        }
#endif

#ifdef CONFIG_ENABLE_I2C_BNO055
        i2c_handle_bno055();
#endif // CONFIG_ENABLE_I2C_BNO055

#if defined(CONFIG_ENABLE_I2C_VL53L0X) || defined(CONFIG_ENABLE_I2C_VL53L1X)
        i2c_lidar_handle();
#endif

        try
        {

#ifdef CONFIG_ENABLE_I2C_POWER
#if defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
            i2c_md.ubat_mV = i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x28);
            i2c_md.isolar_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x30);
            i2c_md.iout_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x32);
            i2c_md.icharge_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x34);
#endif // defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
            i2c_md.ubat_mV = i2cnode_get_u16(STM32_I2C_ADDR, 0x5a);
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO
            g_ubat = (double)i2c_md.ubat_mV / 1000.0;
#endif

            // ESP_LOGW(TAG, "ubat=%f", ubat);

#ifdef CONFIG_ENABLE_ROS2
            i2c_md.ros2_data.msg_ubat.data = g_ubat;
            i2c_md.ros2_data.msg_ubat_valid = true;
#endif

#ifdef CONFIG_ENABLE_I2C_MOTOR
#if defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
            uint64_t t_mot = i2cnode_get_u64(MOTORNODE_I2C_ADDR, 0x00);
            i2cnode_set_u16(MOTORNODE_I2C_ADDR, 0x0A, 0xffff); // TWI_REG_U16_AUTOBREAK

            int64_t motor_l_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x30);
            int64_t motor_r_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x50);

            int16_t motor_l_rel_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x38);
            int16_t motor_r_rel_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x58);
#endif

#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
            int16_t motor_l_rel_enc = 0;
            int16_t motor_r_rel_enc = 0;
            {
                uint8_t tmpbuf[24] = {};
                if(ESP_OK == i2cnode_read(ZUMO_I2C_ADDR, tmpbuf, sizeof(tmpbuf)))
                {
                    int i = 6;
                    motor_l_rel_enc |= (tmpbuf[i++] << 8);
                    motor_l_rel_enc |= (tmpbuf[i++] << 0);
                    motor_r_rel_enc |= (tmpbuf[i++] << 8);
                    motor_r_rel_enc |= (tmpbuf[i++] << 0);
                }
            }
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO
#endif

#ifdef CONFIG_ENABLE_I2C_OLED
            if(next_oled_update < esp_timer_get_time())
            {
                char tmpstr[64];
                int y = 0;
                int s = 5;
                SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);

#ifdef CONFIG_ENABLE_I2C_POWER
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
                SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_7x13);
                uint32_t pm_rtc = i2cnode_get_u32(PWRNODE_I2C_ADDR, 0x04);
                sprintf(tmpstr, "P:%d T:%03d:%02d:%02d:%02d", ((esp_ota_get_running_partition()->address) >> 20) & 0x7,
                    (pm_rtc / (60 * 60 * 24)) % 1000, (pm_rtc / (60 * 60)) % 24, (pm_rtc / 60) % 60, (pm_rtc) % 60);
                SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
                y += (s + 7);
#endif // CONFIG_ROS2NODE_HW_ROS2MOWER
#endif

                SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_7x13);
                sprintf(tmpstr, "IP:.%03d IMU:%d%d%d%d   ", (s_ip_addr.addr >> 24) & 0xff, bno055_calib.sys,
                    bno055_calib.gyro, bno055_calib.mag, bno055_calib.accel);
                SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
                y += (s + 7);

                SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_13x24);
                sprintf(tmpstr, "Ub:%2.2fV", i2c_md.ubat_mV / 1000.0);
                SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
                y += (s + 13);

                SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_fallback_15x17);
                sprintf(tmpstr, "I: %3d %3d %4d", i2c_md.isolar_mA, i2c_md.iout_mA, i2c_md.icharge_mA);
                SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
                y += (s + 13);

                SSD1306_Update(&I2CDisplay);
                next_oled_update = esp_timer_get_time() + 1000000UL;
            }
#endif /* CONFIG_ENABLE_I2C_OLED */

#ifdef CONFIG_ENABLE_ROS2
#ifdef CONFIG_ENABLE_I2C_MOTOR
            i2c_handle_encoder(motor_l_rel_enc, motor_r_rel_enc);
#endif
#endif

            i2c_set_cmd_vel(0.0, 0.0, 0.0 /* rad/sec*/);
            i2c_set_cmd_vel( 0.0, 0.0, -2*M_PI / 20.0 /* rad/sec*/ );
            i2c_handle_cmd_vel();

#ifdef CONFIG_ENABLE_I2C_POWER
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
            if(keepon == true || console_connected() || i2c_cmd_vel_active())
            {
                i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 600); // update TWI_MEM_SHDWNCNT while ROS is connected
            }
            else if(i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x10) > 30)
            {
                i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 30); // update TWI_MEM_SHDWNCNT while ROS is connected
            }
            else
            {
                ESP_LOGW(TAG, "I2C shutdown in  %d seconds: keepon=%d con=%d cmd_vel=%d",
                    i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x10), (keepon) ? 1 : 0, console_connected(),
                    i2c_cmd_vel_active());
            }
#endif
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
            if(keepon == true || console_connected() ||
#ifdef CONFIG_ENABLE_ROS2
            // ros2node_connected() != 0 ||*/
#endif
                i2c_cmd_vel_active())
            {
#ifdef CONFIG_ENABLE_I2C_POWER
                i2cnode_set_u16(STM32_I2C_ADDR, 0x60 /*I2C_REG_TB_U16_TON_TOUT*/,
                    PWR_KEEP_ALIVE_DELAY); // shutdown in 10 seconds
#endif
            }
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO
#ifdef CONFIG_ENABLE_I2C_POWER
            if(i2cnode_get_u16(STM32_I2C_ADDR, 0x60 /*I2C_REG_TB_U16_TON_TOUT*/) < 2)
            {
                shutdown = true;
            }
#endif
#endif // CONFIG_ENABLE_I2C_POWER
            (void)keepon;

#ifdef CONFIG_ENABLE_I2C_MOTOR
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
            i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x0f, 2); // Motor Driver Watchdog Reset
#endif
#endif
        }
        catch(int err)
        {
            ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
        }

        pm_unref();

        while(shutdown == true)
        {
            ESP_LOGE(TAG, "shutdown, wait for power off ... ");
            // wait for shutdown ...
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        if(i2c_cmd_vel_active())
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        /*
         * ... WHILE
         */
    }
}
#endif

/**
 * @brief 
 */
void i2c_handler_bno055_init()
{
#ifdef CONFIG_ENABLE_I2C_BNO055
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
    try
    {
#ifdef CONFIG_ENABLE_I2C_POWER
        i2cnode_set_u16(STM32_I2C_ADDR, 0x60 /*I2C_REG_TB_U16_TON_TOUT*/,
            PWR_KEEP_ALIVE_DELAY); // shutdown in 10 seconds
        i2cnode_set_u16(STM32_I2C_ADDR, 0x68 /*I2C_REG_TB_U16_TOFF_PERIOD*/, 600);

        i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, 0x0000);
        // i2cnode_set_u16(STM32_I2C_ADDR, 0x64 /*I2C_REG_TB_U16_TON_WDG*/, 10); // set STM32 watchdog timeout to 10
        // seconds ...
#endif
        vTaskDelay(50 / portTICK_PERIOD_MS);

        // release BNO055
        uint16_t reg = i2cnode_get_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/);
        reg |= (1 << 15);
        ESP_LOGW(TAG, "i2c_task() rst-reg=0x%04x", reg);
        i2cnode_set_u16(STM32_I2C_ADDR, 0x4C /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        {
            int retry = 20;
            while(i2cnode_check(I2C_BNO055_ADDR) != ESP_OK && retry--)
            {
                ESP_LOGW(TAG, "i2c_task() waiting for bno055 ...");
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    catch(int err)
    {
        ESP_LOGE(TAG, "i2c_task() I2C exception err=0x%08x", err);
    }
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO

    try
    {
        bno055 = new BNO055((i2c_port_t)I2C_BUS_PORT, I2C_BNO055_ADDR);
        if(bno055 != NULL)
        {
            try
            {
                ESP_LOGI(TAG, "i2c_init_bno055() init ...");
                bno055->reset();
                bno055->begin(); // BNO055 is in CONFIG_MODE until it is changed
                bno055->enableExternalCrystal();
                bno055->setPwrModeNormal();
                bno055->setOprModeConfig();
                // bno.setSensorOffsets(storedOffsets);
                // bno055->setAxisRemap(BNO055_REMAP_CONFIG_P0, BNO055_REMAP_SIGN_P0); // see datasheet, section 3.4
                // bno055->setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
                bno055->setAxisRemap(BNO055_REMAP_CONFIG_P2, BNO055_REMAP_SIGN_P2); // see datasheet, section 3.4
                // xxbno055->setAxisRemap(BNO055_REMAP_CONFIG_P3, BNO055_REMAP_SIGN_P3); // see datasheet, section 3.4

                bno055->setUnits(BNO055_UNIT_ACCEL_MS2, BNO055_UNIT_ANGULAR_RATE_RPS, BNO055_UNIT_EULER_DEGREES,
                    BNO055_UNIT_TEMP_C, BNO055_DATA_FORMAT_ANDROID);

                bno055->setAccelConfig(
                    BNO055_CONF_ACCEL_RANGE_4G, BNO055_CONF_ACCEL_BANDWIDTH_7_81HZ, BNO055_CONF_ACCEL_MODE_NORMAL);
                /* you can specify a PoWeRMode using:
                                        - setPwrModeNormal(); (Default on startup)
                                        - setPwrModeLowPower();
                                        - setPwrModeSuspend(); (while suspended bno055 must remain in CONFIG_MODE)
                                        */
                bno055->enableAccelSlowMotionInterrupt();
                bno055->enableAccelNoMotionInterrupt();
                bno055->enableAccelAnyMotionInterrupt();
                bno055->enableAccelHighGInterrupt();
                bno055->enableGyroAnyMotionInterrupt();
                bno055->disableGyroHRInterrupt();
                bno055->clearInterruptPin();

                bno055_offsets_t o = {};
#if 1
                {
                    nvs_handle my_handle;
                    esp_err_t err = nvs_open("bno055", NVS_READWRITE, &my_handle);
                    if(err == ESP_OK)
                    {
                        size_t l = sizeof(o);
                        err = nvs_get_blob(my_handle, "bno055_offsets", &o, &l);
                        if(err == ESP_OK && l == sizeof(o))
                        {
                            ESP_LOGW(TAG, "i2c_init_bno055() ");

                            ESP_LOGW(TAG,
                                "i2c_handle_bno055() offsets read from nvs "
                                "[ax=%d,ay=%d,az=%d,mx=%d,my=%d,mz=%d,gx=%d,gy=%d,gz=%d,ar=%d,mr=%d]",
                                o.accelOffsetX, o.accelOffsetY, o.accelOffsetZ, o.magOffsetX, o.magOffsetY,
                                o.magOffsetZ, o.gyroOffsetX, o.gyroOffsetY, o.gyroOffsetZ, o.accelRadius, o.magRadius);

                            if(o.accelRadius < -2048)
                                o.accelRadius = -2048;
                            if(o.accelRadius > 2048)
                                o.accelRadius = 2048;
                            if(o.magRadius < 144)
                                o.magRadius = 144;
                            if(o.magRadius > 1280)
                                o.magRadius = 1280;
                            bno055->setSensorOffsets(o);
                        }
                        nvs_close(my_handle);
                    }
                }
#endif

                o = bno055->getSensorOffsets();
                bno055_calib = bno055->getCalibration();
                bno055_calib_other = bno055_calib.sys + bno055_calib.gyro + bno055_calib.mag + bno055_calib.accel;
                ESP_LOGW(TAG, "i2c_init_bno055() SET calib(%d,%d,%d,%d) %d,%d,%d %d,%d,%d %d,%d,%d %d,%d",
                    bno055_calib.sys, bno055_calib.gyro, bno055_calib.mag, bno055_calib.accel, o.accelOffsetX,
                    o.accelOffsetY, o.accelOffsetZ, o.magOffsetX, o.magOffsetY, o.magOffsetZ, o.gyroOffsetX,
                    o.gyroOffsetY, o.gyroOffsetZ, o.accelRadius, o.magRadius);

                bno055->clearInterruptPin();
                bno055->setOprModeNdof();

                bno055_self_test_result_t st = bno055->getSelfTestResult();
                bno055_system_error_t bno055_error = bno055->getSystemError();
                bno055_system_status_t bno055_status = bno055->getSystemStatus();
                ESP_LOGW(TAG, "i2c_init_bno055() error=%d status=%d selftest=(%d,%d,%d,%d) BNO055 init done",
                    bno055_error, bno055_status, st.mcuState, st.gyrState, st.magState, st.accState);
            }
            catch(BNO055BaseException& ex)
            {
                ESP_LOGI(TAG, "i2c_init_bno055() BNO055 exception %s", ex.what());
                delete bno055;
                bno055 = NULL;
            }
        }
    }
    catch(int err)
    {
        ESP_LOGE(TAG, "i2c_task() I2C exception err=0x%08x", err);
        delete bno055;
        bno055 = NULL;
    }
#endif
}
/**
 * @brief 
 */
void i2c_handler_bno055_loop()
{
#ifdef CONFIG_ENABLE_I2C_BNO055
    if(bno055 != NULL)
    {
        try
        {
            try
            {
                int8_t temperature = bno055->getTemp();
                bno055_system_error_t bno055_error = bno055->getSystemError();
                bno055_system_status_t bno055_status = bno055->getSystemStatus();
                bno055_interrupts_status_t irq_status = bno055->getInterruptsStatus();

                bno055->clearInterruptPin();

                if(bno055_status == BNO055_SYSTEM_STATUS_FUSION_ALGO_RUNNING &&
                    bno055_error == BNO055_SYSTEM_ERROR_NO_ERROR)
                {
                    bno055_calib = bno055->getCalibration();

                    if(bno055_calib.sys == 3 &&
                        bno055_calib_other <
                            (bno055_calib.sys + bno055_calib.gyro + bno055_calib.mag + bno055_calib.accel))
                    {
                        // ESP_LOGW(TAG, "i2c_handle_bno055() callib sys=%d %d %d %d ",
                        // bno055_calib.sys,bno055_calib.gyro,bno055_calib.mag,bno055_calib.accel);
                        if(bno055_calib.sys == 3 && bno055_calib.gyro > 1 && bno055_calib.mag > 1 &&
                            bno055_calib.accel > 1)
                        {
                            bno055_calib_other =
                                bno055_calib.sys + bno055_calib.gyro + bno055_calib.mag + bno055_calib.accel;
                            bno055->setOprModeConfig();
                            bno055_offsets_t bno055_offsets = bno055->getSensorOffsets();
                            {
                                nvs_handle my_handle;
                                bno055_offsets_t o = {};
                                esp_err_t err = nvs_open("bno055", NVS_READWRITE, &my_handle);
                                if(err == ESP_OK)
                                {
                                    size_t l = sizeof(o);
                                    err = nvs_get_blob(my_handle, "bno055_offsets", &o, &l);
                                    if(err == ESP_OK && l == sizeof(o))
                                    {
                                        if(memcpy(&o, &bno055_offsets, sizeof(bno055_offsets_t)) != 0)
                                        {
                                            ESP_LOGI(TAG,
                                                "i2c_handle_bno055() SAVE/UPDATE calib(%d,%d,%d,%d) "
                                                "[ax=%d,ay=%d,az=%d,mx=%d,my=%d,mz=%d,gx=%d,gy=%d,gz=%d,ar=%d,mr=%d]",
                                                bno055_calib.sys, bno055_calib.gyro, bno055_calib.mag,
                                                bno055_calib.accel, o.accelOffsetX, o.accelOffsetY, o.accelOffsetZ,
                                                o.magOffsetX, o.magOffsetY, o.magOffsetZ, o.gyroOffsetX, o.gyroOffsetY,
                                                o.gyroOffsetZ, o.accelRadius, o.magRadius);
                                            if(err == ESP_OK)
                                            {
                                                nvs_set_blob(my_handle, "bno055_offsets", &bno055_offsets,
                                                    sizeof(bno055_offsets));
                                            }
                                        }
                                    }
                                    nvs_close(my_handle);
                                }
                            }
                            bno055->setOprModeNdof();
                        }
                    }

                    bno055_quaternion_t quaternion = bno055->getQuaternion();
                    bno055_vector_t vector_angvel = bno055->getVectorGyroscope();
                    bno055_vector_t vector_linaccl = bno055->getVectorLinearAccel();

                    ESP_LOGD(TAG,
                        "i2c_handle_bno055() irq-status %d %d %d %d %d error=0x%02x status=0x%02x temp=%d x=%f y=%f "
                        "z=%f w=%f",
                        irq_status.accelNoSlowMotion, irq_status.accelAnyMotion, irq_status.accelHighG,
                        irq_status.gyroHR, irq_status.gyroAnyMotion, bno055_error, bno055_status, temperature,
                        quaternion.x, quaternion.y, quaternion.z, quaternion.w);

#ifdef CONFIG_ENABLE_ROS2
                    i2c_md.ros2_data.msg_imu.orientation.x = quaternion.x;
                    i2c_md.ros2_data.msg_imu.orientation.y = quaternion.y;
                    i2c_md.ros2_data.msg_imu.orientation.z = quaternion.z;
                    i2c_md.ros2_data.msg_imu.orientation.w = quaternion.w;
                    i2c_md.ros2_data.msg_imu.orientation_covariance[0] = 0.0008;
                    i2c_md.ros2_data.msg_imu.orientation_covariance[1] = 0;
                    i2c_md.ros2_data.msg_imu.orientation_covariance[2] = 0;
                    i2c_md.ros2_data.msg_imu.orientation_covariance[3] = 0;
                    i2c_md.ros2_data.msg_imu.orientation_covariance[4] = 0.0008;
                    i2c_md.ros2_data.msg_imu.orientation_covariance[5] = 0;
                    i2c_md.ros2_data.msg_imu.orientation_covariance[6] = 0;
                    i2c_md.ros2_data.msg_imu.orientation_covariance[7] = 0;
                    i2c_md.ros2_data.msg_imu.orientation_covariance[8] = 0.0008;

                    i2c_md.ros2_data.msg_imu.angular_velocity.x = vector_angvel.x /* * M_PI / 180.0*/;
                    i2c_md.ros2_data.msg_imu.angular_velocity.y = -vector_angvel.y /* * M_PI / 180.0*/;
                    i2c_md.ros2_data.msg_imu.angular_velocity.z = vector_angvel.z /* * M_PI / 180.0*/;
                    i2c_md.ros2_data.msg_imu.angular_velocity_covariance[0] = 0.02;
                    i2c_md.ros2_data.msg_imu.angular_velocity_covariance[1] = 0;
                    i2c_md.ros2_data.msg_imu.angular_velocity_covariance[2] = 0;
                    i2c_md.ros2_data.msg_imu.angular_velocity_covariance[3] = 0;
                    i2c_md.ros2_data.msg_imu.angular_velocity_covariance[4] = 0.02;
                    i2c_md.ros2_data.msg_imu.angular_velocity_covariance[5] = 0;
                    i2c_md.ros2_data.msg_imu.angular_velocity_covariance[6] = 0;
                    i2c_md.ros2_data.msg_imu.angular_velocity_covariance[7] = 0;
                    i2c_md.ros2_data.msg_imu.angular_velocity_covariance[8] = 0.02;

                    i2c_md.ros2_data.msg_imu.linear_acceleration.x = vector_linaccl.y /* / 100.0*/;
                    i2c_md.ros2_data.msg_imu.linear_acceleration.y = -vector_linaccl.x /* / 100.0*/;
                    i2c_md.ros2_data.msg_imu.linear_acceleration.z = vector_linaccl.z /* / 100.0*/;
                    i2c_md.ros2_data.msg_imu.linear_acceleration_covariance[0] = 0.04;
                    i2c_md.ros2_data.msg_imu.linear_acceleration_covariance[1] = 0;
                    i2c_md.ros2_data.msg_imu.linear_acceleration_covariance[2] = 0;
                    i2c_md.ros2_data.msg_imu.linear_acceleration_covariance[3] = 0;
                    i2c_md.ros2_data.msg_imu.linear_acceleration_covariance[4] = 0.04;
                    i2c_md.ros2_data.msg_imu.linear_acceleration_covariance[5] = 0;
                    i2c_md.ros2_data.msg_imu.linear_acceleration_covariance[6] = 0;
                    i2c_md.ros2_data.msg_imu.linear_acceleration_covariance[7] = 0;
                    i2c_md.ros2_data.msg_imu.linear_acceleration_covariance[8] = 0.04;

                    struct timespec tv = {};
                    clock_gettime(CLOCK_MONOTONIC, &tv);
                    i2c_md.ros2_data.msg_imu.header.stamp.nanosec = tv.tv_nsec;
                    i2c_md.ros2_data.msg_imu.header.stamp.sec = tv.tv_sec;

                    i2c_md.ros2_data.msg_imu_valid = true;

#endif
                }
                else
                {
                    ESP_LOGE(TAG,
                        "i2c_handle_bno055() ERR temp=%d sw=0x%04x irq-status %d %d %d %d %d error=0x%02x status=0x%02x",
                        temperature, bno055->getSWRevision(), irq_status.accelNoSlowMotion, irq_status.accelAnyMotion,
                        irq_status.accelHighG, irq_status.gyroHR, irq_status.gyroAnyMotion, bno055_error,
                        bno055_status);
                }
            }
            catch(BNO055BaseException& ex)
            {
                ESP_LOGE(TAG, "i2c_handle_bno055() exception %s", ex.what());
                delete bno055;
                bno055 = NULL;
            }
        }
        catch(int err)
        {
            ESP_LOGE(TAG, "i2c_handle_bno055() I2C exception err=0x%02x", err);
            delete bno055;
            bno055 = NULL;
        }
    }
#endif /* CONFIG_ENABLE_I2C_BNO055 */
}

void i2c_handler_lawn_motor_init()
{
#if defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
#ifdef CONFIG_ENABLE_I2C_MOTOR
    try
    {
        //i2cnode_set_i16(LAWNMOTORNODE_I2C_ADDR, 0x20, (int16_t)MOTOR_P);
        //i2cnode_set_i16(LAWNMOTORNODE_I2C_ADDR, 0x22, (int16_t)MOTOR_I);
        //i2cnode_set_i16(LAWNMOTORNODE_I2C_ADDR, 0x24, (int16_t)MOTOR_D);
        i2cnode_set_i16(LAWNMOTORNODE_I2C_ADDR, 0x08, 2);
        i2cnode_set_i16(LAWNMOTORNODE_I2C_ADDR, 0x20, 0);
        //i2cnode_set_i16(LAWNMOTORNODE_I2C_ADDR, 0x40, 5);
    }
    catch(int err)
    {
        i2c_setpin_boot(1);
        ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
        i2c_setpin_boot(0);
    }
#endif
#endif
}

void i2c_handler_lawn_motor_loop()
{
#ifdef CONFIG_ENABLE_I2C_MOTOR
#if defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
    i2cnode_set_i16(LAWNMOTORNODE_I2C_ADDR, 15, 0); // trigger motor watchdog
#endif // defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
#endif
    if(i2c_md.lawn_motor.update == true )
    {
        i2c_md.lawn_motor.update = false;
        i2cnode_set_i16(LAWNMOTORNODE_I2C_ADDR, 0x20, (int16_t)i2c_md.lawn_motor.speed);
    }
}

void i2c_handler_lawn_motor_exit()
{
}

/**
 * @brief 
 */
void i2c_handler_motor_init()
{
#if defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
#ifdef CONFIG_ENABLE_I2C_POWER
    try
    {
        i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 60);    // update TWI_MEM_SHDWNCNT
        i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x18, 13000); // stay on with ubat>12.5V
    }
    catch(int err)
    {
        i2c_setpin_boot(1);
        ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
        i2c_setpin_boot(0);
    }
#endif
#ifdef CONFIG_ENABLE_I2C_MOTOR
    try
    {
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, (int16_t)MOTOR_P);
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x22, (int16_t)MOTOR_I);
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x24, (int16_t)MOTOR_D);
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x08, 2);
        // i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, MOTOR_RPM(MOTOR_START_RPM_L));
        // i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, MOTOR_RPM(MOTOR_START_RPM_R));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, MOTOR_RPM(MOTOR_START_RPM_L));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, MOTOR_RPM(MOTOR_START_RPM_R));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x0E, 0); /* int pulse */
    }
    catch(int err)
    {
        i2c_setpin_boot(1);
        ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
        i2c_setpin_boot(0);
    }
#endif
#endif
}

/**
 * @brief 
 */
void i2c_handler_motor_loop()
{
#ifdef CONFIG_ENABLE_I2C_MOTOR
#if defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
    i2cnode_set_i16(MOTORNODE_I2C_ADDR, 15, 0); // trigger motor watchdog
#endif // defined(CONFIG_ROS2NODE_HW_ROS2MOWER) || defined(CONFIG_ROS2NODE_HW_S2_MOWER)
#endif
    //i2c_set_cmd_vel(0.0, 0.0, 0.0 /* rad/sec*/);
    //i2c_set_cmd_vel( 0.0, 0.0, -2*M_PI / 10.0 /* rad/sec*/ );
    i2c_handle_cmd_vel();    
}

/**
 * @brief 
 */
void i2c_handler_motor_exit()
{
    //i2cnode_init_motor();
}

/**
 * @brief 
 * @param level
 */
void i2c_int(int level)
{
    i2c_md.ready = false;

#ifdef I2C_BUS_INT
    if(level == 1)
    {
        gpio_set_direction((gpio_num_t)I2C_BUS_INT, GPIO_MODE_INPUT);
        //gpio_set_direction((gpio_num_t)I2C_BUS_INT, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode((gpio_num_t)I2C_BUS_INT, GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)I2C_BUS_INT, 1);
    }
    else
    {
        gpio_set_direction((gpio_num_t)I2C_BUS_INT, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode((gpio_num_t)I2C_BUS_INT, GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)I2C_BUS_INT, 0);
    }
#endif
}

/**
 * @brief 
 * @param level
 */
void i2c_reset(int level)
{
#ifdef I2C_BUS_RESET
    if(level == 1)
    {
        //gpio_set_direction((gpio_num_t)I2C_BUS_RESET, GPIO_MODE_INPUT);
        gpio_set_direction((gpio_num_t)I2C_BUS_RESET, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode((gpio_num_t)I2C_BUS_RESET, GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)I2C_BUS_RESET, 1);
    }
    else
    {
        gpio_set_direction((gpio_num_t)I2C_BUS_RESET, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode((gpio_num_t)I2C_BUS_RESET, GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)I2C_BUS_RESET, 0);
    }
#endif
}

/**
 * @brief 
 * @param level
 */
void i2c_setpin_boot(int level)
{
    if(level == 1)
    {
        gpio_set_direction((gpio_num_t)0, GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)0, GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)0, 1);
    }
    else
    {
        gpio_set_direction((gpio_num_t)0, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode((gpio_num_t)0, GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)0, 0);
    }
}

/**
 * @brief
 * @param param
 */
static void i2c_task(void* param)
{   
    ESP_LOGW(TAG, "i2c_task() ...");

    static i2c_config_t Config;
    memset(&Config, 0, sizeof(i2c_config_t));
    Config.mode = I2C_MODE_MASTER;
    Config.sda_io_num = (gpio_num_t)I2C_BUS_SDA;
    // Config.sda_io_num = (gpio_num_t)13;
    Config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    Config.scl_io_num = (gpio_num_t)I2C_BUS_SCL;
    // Config.scl_io_num = (gpio_num_t)12;
    Config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    Config.master.clk_speed = I2C_BUS_CLOCK;
    i2c_param_config((i2c_port_t)I2C_BUS_PORT, &Config);
    i2c_driver_install((i2c_port_t)I2C_BUS_PORT, Config.mode, 0, 0, 0);
    i2c_set_timeout((i2c_port_t)I2C_BUS_PORT, I2C_APB_CLK_FREQ / 10); /* 10ms timeout */

    // i2c_set_start_timing((i2c_port_t)I2C_BUS_PORT,I2C_APB_CLK_FREQ/1000,I2C_APB_CLK_FREQ/1000);
    // i2c_set_stop_timing((i2c_port_t)I2C_BUS_PORT,I2C_APB_CLK_FREQ/1000,I2C_APB_CLK_FREQ/1000);
    
    i2c_reset(0);
    i2c_int(1);
    i2c_setpin_boot(1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    i2c_reset(1);

    i2c_updater();

    pm_ref();
    i2c_scan();
    i2c_handler_motor_init();
    i2c_handler_lawn_motor_init();
    i2c_handler_bno055_init();
    pm_unref();

    ESP_LOGE(TAG, "test");

    ESP_LOGW(TAG, "i2c_task() started %d ...",50 / portTICK_PERIOD_MS);
    while((powerstate() != 0) && (i2c_md_active == true) )
    {
        xSemaphoreTake(i2c_md.sem_ist, 50 / portTICK_PERIOD_MS); 
        i2c_setpin_boot(0);
        ESP_LOGD(TAG, "i2c_task() %d loop ...",i2c_md_active);
        pm_ref();
        i2c_handler_motor_loop();
        i2c_handler_lawn_motor_loop();
        i2c_handle_pid_tuning();
        i2c_handler_bno055_loop();  
        i2c_updater();
        pm_unref();
        ESP_LOGD(TAG, "i2c_task() loop ... done");
        i2c_setpin_boot(1);
    }
    ESP_LOGW(TAG, "i2c_task() stopped ...");
    i2c_handler_lawn_motor_exit();
    i2c_handler_motor_exit();
    
    i2c_driver_delete(I2C_BUS_PORT);

    ESP_LOGW(TAG, "i2c_task() ... done");
    vTaskDelete(NULL);
}

void i2c_handler_init()
{
    memset(&i2c_md, 0, sizeof(i2c_md));
    // esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "i2c_handler_init() ...");

    i2c_md.sem = xSemaphoreCreateBinary();
    xSemaphoreGive(i2c_md.sem);
    i2c_md.sem_ist = xSemaphoreCreateBinary();
        
    i2c_md_active = false;
}

/**
 * @brief
 */
void i2c_handler_start()
{    
    ESP_LOGI(TAG, "i2c_handler_start() ... ");
#ifdef I2C_BUS_INT
    gpio_reset_pin((gpio_num_t)I2C_BUS_INT);
#endif
    i2c_reset(0);

    i2c_md_active = true;
    xTaskCreate(&i2c_task, "i2c_task", 8192, NULL, DEFAULT_PRIO, NULL);
    ESP_LOGI(TAG, "i2c_handler_start() ... done");
}

void i2c_handler_stop()
{
    ESP_LOGI(TAG, "i2c_handler_exit() ...");
    i2c_md_active = false;
    ESP_LOGI(TAG, "i2c_handler_exit() ... done");
}

/**
 * EOF
 */
