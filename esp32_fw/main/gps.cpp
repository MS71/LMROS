#include "sdkconfig.h"
#ifdef CONFIG_ENABLE_GPS
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

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

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "nvs_flash.h"

#include "ntrip.h"
#include <esp_http_client.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "gpgga.h"
#include "gps.h"
#include "nmea.h"
#include "parser.h"

#include "hwconfig.h"

#undef ENABLE_NTRIP_TEST
#undef ENABLE_ASYNC_HTTP_CLIENT

static const char* TAG = "GPS";

extern EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

uint8_t powerstate();

#define PORT 20000

#define KEEPALIVE_IDLE 5
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT 3

uint8_t gps_uart_ready = 0;
uint8_t gps_sapos_ready = 0;

#define RXBUF_SIZE 1024
uint8_t gps_rx_buffer[RXBUF_SIZE + 2];

#define BUFFER_SIZE 512

static void gps_uart_handle();

struct {
    struct {
        nmea_gpgga_s gga;
    } uart;
    struct {
        char host[48];
        uint16_t port;
        char username[32];
        char password[32];
        char mountpoint[32];
        int64_t reconnect_timeout;
        esp_http_client_handle_t http;
    } ntrip;
    struct {
        int listen_sock;
        int client_sock;        
    } tcpserv;
} gps_md = {};

/**
 * @brief 
 * @param evt
 * @return 
 */
esp_err_t gps_ntrip_http_event_handle(esp_http_client_event_t* evt)
{
    switch(evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
        ESP_LOGI(TAG, "%.*s", evt->data_len, (char*)evt->data);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        if(!esp_http_client_is_chunked_response(evt->client))
        {
            ESP_LOGI(TAG, "%.*s", evt->data_len, (char*)evt->data);
        }
        uart_write_bytes(UART_NUM_1, (const char*)evt->data, evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

/**
 * @brief 
 */
static void gps_tcpserv_stop()
{
    if( gps_md.tcpserv.client_sock != 0 )
    {
        close(gps_md.tcpserv.client_sock);
        gps_md.tcpserv.client_sock = 0;
    }
    if( gps_md.tcpserv.listen_sock != 0 )
    {
        close(gps_md.tcpserv.listen_sock);
        gps_md.tcpserv.listen_sock = 0;
    }    
}


/**
 * @brief 
 */
static void gps_tcpserv_start()
{
    if(gps_md.tcpserv.listen_sock == 0)
    {
        ESP_LOGI(TAG, "gps_tcpserv_start ...");

        struct sockaddr_in6 destAddr;
        int ip_protocol;
        int addr_family;
        char addr_str[128];
        bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);

        gps_md.tcpserv.listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if(gps_md.tcpserv.listen_sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            gps_tcpserv_stop();
            return;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(gps_md.tcpserv.listen_sock, (struct sockaddr*)&destAddr, sizeof(destAddr));
        if(err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            gps_tcpserv_stop();
            return;
        }

        err = listen(gps_md.tcpserv.listen_sock, 1);
        if(err != 0)
        {
            ESP_LOGE(TAG, "Error during listen: errno %d", errno);
            gps_tcpserv_stop();
            return;
        }

        ESP_LOGI(TAG, "gps_tcpserv_start ... ok");
    }
}
/**
 * @brief 
 */
static void gps_tcpserv_handle()
{
    if(gps_md.tcpserv.listen_sock == 0)
    {
        gps_tcpserv_start();
    }
    
    if(gps_md.tcpserv.listen_sock != 0)
    {
        if(gps_md.tcpserv.client_sock == 0)
        {
            //ESP_LOGI(TAG, "gps_tcpserv_handle ...");

            fd_set rfds;
            struct timeval tv;
            int retval;
            FD_ZERO(&rfds);
            FD_SET(gps_md.tcpserv.listen_sock, &rfds);
            tv.tv_sec = 0;
            tv.tv_usec = 1000;
            retval = select(gps_md.tcpserv.listen_sock+1, &rfds, NULL, NULL, &tv);

            if(retval == -1)
            {
                perror("select()");
                gps_tcpserv_stop();
            }
            else if(retval)
            {
                ESP_LOGI(TAG, "gps_tcpserv_handle ... ...");

                struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
                uint addrLen = sizeof(sourceAddr);
                gps_md.tcpserv.client_sock =
                    accept(gps_md.tcpserv.listen_sock, (struct sockaddr*)&sourceAddr, &addrLen);

                if(gps_md.tcpserv.client_sock > 0)
                {
                    int keepAlive = 1;
                    int keepIdle = KEEPALIVE_IDLE;
                    int keepInterval = KEEPALIVE_INTERVAL;
                    int keepCount = KEEPALIVE_COUNT;
                    setsockopt(gps_md.tcpserv.client_sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
                    setsockopt(gps_md.tcpserv.client_sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
                    setsockopt(gps_md.tcpserv.client_sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
                    setsockopt(gps_md.tcpserv.client_sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

                    ESP_LOGI(TAG, "gps_tcpserv_handle ... connected");
                }
            }
        }
    }

    if(gps_md.tcpserv.client_sock != 0)
    {
        int len = 0;
        len = recv(gps_md.tcpserv.client_sock, gps_rx_buffer, RXBUF_SIZE, MSG_DONTWAIT);
        if(len > 0)
        {
            if(uart_is_driver_installed(UART_NUM_1))
            {
                uart_write_bytes(UART_NUM_1, (const char*)gps_rx_buffer, len);
            }
        }
    }
}

/**
 * @brief 
 */
void gps_ntrip_stop()
{
    if(gps_md.ntrip.http != NULL)
    {
        esp_http_client_close(gps_md.ntrip.http);
        esp_http_client_cleanup(gps_md.ntrip.http);
        gps_md.ntrip.http = NULL;
        gps_md.ntrip.reconnect_timeout = esp_timer_get_time() + 1000000;

        if(uart_is_driver_installed(UART_NUM_1))
        {
            sprintf((char*)gps_rx_buffer,"$PESP,NTRIP,SRV,DISCONNECTED");
            uart_write_bytes(UART_NUM_1, (const char*)gps_rx_buffer, strlen((char*)gps_rx_buffer));
        }

    }
}

#if 0

static void https_async(void)
{
    esp_http_client_config_t config = {
        .url = "https://postman-echo.com/post",
        .event_handler = _http_event_handler,
        .cert_pem = postman_root_cert_pem_start,
        .is_async = true,
        .timeout_ms = 5000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err;
    const char *post_data = "Using a Palantír requires a person with great strength of will and wisdom. The Palantíri were meant to "
                            "be used by the Dúnedain to communicate throughout the Realms in Exile. During the War of the Ring, "
                            "the Palantíri were used by many individuals. Sauron used the Ithil-stone to take advantage of the users "
                            "of the other two stones, the Orthanc-stone and Anor-stone, but was also susceptible to deception himself.";
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    while (1) {
        err = esp_http_client_perform(client);
        if (err != ESP_ERR_HTTP_EAGAIN) {
            break;
        }
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTPS Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

#endif

/**
 * @brief 
 */
static void gps_ntrip_start()
{
    gps_md.ntrip.http = NULL;
    
    if( gps_md.ntrip.reconnect_timeout > esp_timer_get_time() )
    {
        return;
    }
    gps_md.ntrip.reconnect_timeout = esp_timer_get_time() + 1000000;

    EventBits_t ev = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, 1 / portTICK_PERIOD_MS);
    if(ev != CONNECTED_BIT)
    {
        return;
    }

    // Configure host URL
    esp_http_client_config_t config = {
        .host = gps_md.ntrip.host,
        .port = gps_md.ntrip.port,
        .method = HTTP_METHOD_GET,
    };
    config.path = (const char*)gps_md.ntrip.mountpoint;
    config.auth_type = HTTP_AUTH_TYPE_BASIC;
    config.username = (const char*)gps_md.ntrip.username;
    config.password = (const char*)gps_md.ntrip.password;
    config.event_handler = gps_ntrip_http_event_handle;
#ifdef ENABLE_ASYNC_HTTP_CLIENT
    config.is_async = true;
    config.timeout_ms = 10;
#endif

    // Initialize client
    gps_md.ntrip.http = esp_http_client_init(&config);
#ifdef ENABLE_ASYNC_HTTP_CLIENT
    ... todo ...
#else
    esp_http_client_set_header(gps_md.ntrip.http, "Ntrip-Version", "Ntrip/2.0");
    esp_http_client_set_header(gps_md.ntrip.http, "User-Agent", "NTRIP " NTRIP_CLIENT_NAME "/2.0");
    esp_http_client_set_header(gps_md.ntrip.http, "Connection", "close");

    esp_err_t err = esp_http_client_open(gps_md.ntrip.http, 0);
    if( err != ESP_OK )
    {
        ESP_LOGE(TAG, "Could not open HTTP connection: %d %s", err, esp_err_to_name(err));
        gps_ntrip_stop();
        return;
    }

    int content_length = esp_http_client_fetch_headers(gps_md.ntrip.http);
    if( content_length < 0 )
    {
        ESP_LOGE(TAG, "Could not connect to caster: %d %s", errno, strerror(errno));
        gps_ntrip_stop();
        return;
    }

    int status_code = esp_http_client_get_status_code(gps_md.ntrip.http);
    if( status_code != 200 )
    {
        ESP_LOGE(TAG, "Could not access mountpoint: %d", status_code);
        gps_ntrip_stop();
        return;
    }

    if( !esp_http_client_is_chunked_response(gps_md.ntrip.http) )
    {
        ESP_LOGE(TAG, "Caster did not respond with chunked transfer encoding: content_length %d", content_length);
        gps_ntrip_stop();
        return;
    }
#endif

    if(uart_is_driver_installed(UART_NUM_1))
    {
        sprintf((char*)gps_rx_buffer,"$PESP,NTRIP,SRV,CONNECTED,%s:%d,%s", 
        gps_md.ntrip.host, gps_md.ntrip.port, gps_md.ntrip.mountpoint);
        uart_write_bytes(UART_NUM_1, (const char*)gps_rx_buffer, strlen((char*)gps_rx_buffer));
    }

    ESP_LOGI(TAG, "Successfully connected");
}
    
/**
 * @brief 
 */
static void gps_ntrip_handle()
{
#ifndef ENABLE_NTRIP_TEST
    if(gps_md.ntrip.http == NULL && gps_md.uart.gga.position_fix > 0)
#endif
    {
        gps_ntrip_start();
    }

    if(gps_md.ntrip.http != NULL)
    {
        ESP_LOGI(TAG, "gps_ntrip_handle ...");
#ifdef ENABLE_ASYNC_HTTP_CLIENT
        ... todo ...
#else
        char* buffer = (char*)malloc(BUFFER_SIZE);
        if(buffer != NULL)
        {
            int len;
            while((len = esp_http_client_read(gps_md.ntrip.http, buffer, BUFFER_SIZE)) >= 0)
            {
                if(powerstate() == 0)
                {
                    break; /* stop GPS when power down */
                }
                gps_uart_handle();
                gps_tcpserv_handle();
            }
            free(buffer);
        }
#endif
        ESP_LOGI(TAG, "gps_ntrip_handle ... done");
    }
}

/**
 * @brief 
 * @param buflen
 * @param buf
 */
static void gps_handle_nmea(int buflen, const char* buf)
{
    static char linebuf[1024];
    static int linebuf_used = 0;
    int i;
    for(i = 0; i < buflen; i++)
    {
        if(linebuf_used >= sizeof(linebuf))
        {
            linebuf_used = 0;
        }
        linebuf[linebuf_used] = buf[i];
        if((((unsigned char)buf[i]) >= 32) || (buf[i] == '\r'))
        {
            linebuf_used++;
        }
        else
        {
            if(linebuf_used > 1)
            {
                if(gps_md.ntrip.http != NULL)
                {
                    linebuf[linebuf_used + 1] = 0;
                    // ESP_LOGW(TAG, "=>sapos: %d %s", linebuf_used+1,linebuf);
                    int sent = esp_http_client_write(gps_md.ntrip.http, (char*)linebuf, linebuf_used + 1);
                    if(sent < 0)
                    {
                        gps_ntrip_stop();
                    }
                }

                if(linebuf_used >= 3)
                {
                    if(linebuf[0] == '$' && linebuf[1] == 'G' && linebuf[2] == 'N')
                    {
                        linebuf[2] = 'P'; // check_checksum=0
                    }
                }
                nmea_s* p = nmea_parse(linebuf, linebuf_used + 1, 0);
                if(p != NULL)
                {
                    // ESP_LOGW(TAG, "%d %d %s", p->errors,p->type,linebuf);
                    if(NMEA_GPGGA == p->type)
                    {
                        gps_md.uart.gga = *((nmea_gpgga_s*)p);
#if 1
                        ESP_LOGW(TAG, "fix=%d N=%d lat=%d.%f long=%d.%f", 
                            gps_md.uart.gga.position_fix, 
                            gps_md.uart.gga.n_satellites,
                            gps_md.uart.gga.latitude.degrees, 
                            gps_md.uart.gga.latitude.minutes, 
                            gps_md.uart.gga.longitude.degrees,
                            gps_md.uart.gga.longitude.minutes);
#endif
                    }
                    nmea_free(p);
                }
                else
                {
                    // ESP_LOGW(TAG, "error %s", linebuf);
                }
            }
            linebuf_used = 0;
        }
        if(linebuf_used == 1 && linebuf[0] != '$')
        {
            linebuf_used = 0;
        }
    }
}

/**
 * @brief 
 */
static void gps_uart_stop()
{    
    if( uart_is_driver_installed(UART_NUM_1) )
    {
        ESP_LOGI(TAG, "gps_uart_stop");
        uart_driver_delete(UART_NUM_1);
        memset((void*)&gps_md.uart,0,sizeof(gps_md.uart));
    }
}

/**
 * @brief 
 */
static void gps_uart_start()
{
    if(!uart_is_driver_installed(UART_NUM_1))
    {
        ESP_LOGI(TAG, "gps_uart_start");

        memset((void*)&gps_md.uart, 0, sizeof(gps_md.uart));

        /* Configure parameters of an UART driver,
         * communication pins and install the driver */
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_REF_TICK,
        };

        int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

        uart_driver_install(UART_NUM_1, RXBUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
        uart_param_config(UART_NUM_1, &uart_config);
        uart_set_pin(UART_NUM_1, GPS_UART1_TXD, GPS_UART1_RXD, -1, -1);
#ifdef GPS_UART1_INV
        uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
#endif

        uart_flush(UART_NUM_1);
    }
}
/**
 * @brief 
 */
static void gps_uart_handle()
{
    if(uart_is_driver_installed(UART_NUM_1))
    {
        // ESP_LOGI(TAG, "gps_uart_handle");
        int len =
            uart_read_bytes(UART_NUM_1, gps_rx_buffer, RXBUF_SIZE, (1 + (RXBUF_SIZE / (10 * 3))) / portTICK_RATE_MS);
        if(len > 0)
        {
            if(gps_md.tcpserv.client_sock != 0)
            {
                send(gps_md.tcpserv.client_sock, gps_rx_buffer, len, 0);
            }
            gps_rx_buffer[len] = 0;
            // ESP_LOGW(TAG, "tx %d %s", len,gps_rx_buffer);
            gps_handle_nmea(len, (const char*)gps_rx_buffer);
        }
    }
}

/**
 * @brief 
 * @param pvParameters
 */
static void gps_task(void* pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "gps_task TXD=%d RXD=%d", GPS_UART1_TXD, GPS_UART1_RXD);

    nmea_load_parsers();

    while(true)
    {
        EventBits_t ev;
        const int gps_wait = 1000;

        ESP_LOGI(TAG, "Wait until power is on ... ");
        while(powerstate() == 0)
        {
            gps_uart_stop();
            vTaskDelay(gps_wait / portTICK_PERIOD_MS);
        }

        gps_uart_start();
        gps_tcpserv_start();
#ifdef ENABLE_NTRIP_TEST
        gps_ntrip_start();
#endif

        while(true)
        {
            if(powerstate() == 0)
            {
                break; /* stop GPS when power down */
            }

            gps_uart_handle();
            gps_tcpserv_handle();
            gps_ntrip_handle();
        }

        gps_ntrip_stop();
        gps_tcpserv_stop();
        gps_uart_stop();
        vTaskDelete(NULL);
    }
}

/**
 * @brief 
 */
void gps_init()
{
    nvs_handle my_handle;
    esp_err_t err = nvs_open("ntrip", NVS_READWRITE, &my_handle);
    if(err == ESP_OK)
    {
        size_t str_size;
        str_size = sizeof(gps_md.ntrip.host);
        nvs_get_str(my_handle, "host", gps_md.ntrip.host, &str_size);
        nvs_get_u16(my_handle, "port", &gps_md.ntrip.port);
        str_size = sizeof(gps_md.ntrip.username);
        nvs_get_str(my_handle, "username", gps_md.ntrip.username, &str_size);
        str_size = sizeof(gps_md.ntrip.password);
        nvs_get_str(my_handle, "password", gps_md.ntrip.password, &str_size);
        str_size = sizeof(gps_md.ntrip.mountpoint);
        nvs_get_str(my_handle, "mountpoint", gps_md.ntrip.mountpoint, &str_size);
        nvs_close(my_handle);
    }

    ESP_LOGI(TAG, "gps_init host=%s:%d user=%s passwd=%s mointpoint=%s", 
        gps_md.ntrip.host, 
        gps_md.ntrip.port, 
        gps_md.ntrip.username,
        gps_md.ntrip.password,
        gps_md.ntrip.mountpoint);

    if(gps_md.ntrip.port != 0 && 
        strlen(gps_md.ntrip.host) != 0 && 
        strlen(gps_md.ntrip.username) != 0 &&
        strlen(gps_md.ntrip.password) != 0 &&
        strlen(gps_md.ntrip.mountpoint) != 0)
    {
        xTaskCreate(gps_task, "gps_task", 16384, NULL, DEFAULT_PRIO, NULL);
    }
}
/**
 * @brief 
 */
void gps_exit()
{
}

#endif
