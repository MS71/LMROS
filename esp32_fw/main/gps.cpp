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

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "gps.h"
#include "hwconfig.h"

static const char* TAG = "GPS";

#define PORT 20000

#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

#define RXBUF_SIZE 1024
uint8_t gps_rx_buffer[RXBUF_SIZE+2];

static void gps_server_task(void* pvParameters)
{
    ESP_LOGI(TAG, "tcp_server_task TXD=%d RXD=%d", GPS_UART1_TXD, GPS_UART1_RXD);

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
    uart_set_line_inverse(UART_NUM_1,UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
#endif

    while(1)
    {
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

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if(listen_sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr*)&destAddr, sizeof(destAddr));
        if(err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket bond");

        uart_flush(UART_NUM_1);
        
        while(1)
        {

            err = listen(listen_sock, 1);
            if(err != 0)
            {
                ESP_LOGE(TAG, "Error during listen: errno %d", errno);
                break;
            }

            struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
            uint addrLen = sizeof(sourceAddr);
            int sock = accept(listen_sock, (struct sockaddr*)&sourceAddr, &addrLen);
            if(sock < 0)
            {
                ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
                break;
            }

#if 0
/*
!UBX CFG-GNSS 0 32 32 1 0 10 32 0 1
!UBX CFG-GNSS 0 32 32 1 6 8 16 0 1
!UBX CFG-MSG 3 15 0 1 0 1 0 0
!UBX CFG-MSG 3 16 0 1 0 1 0 0
!UBX CFG-MSG 1 32 0 1 0 1 0 0
 */
            ESP_LOGI(TAG, "send UBX rtklib init ...");
            const char initstr[] = "!UBX CFG-GNSS 0 32 32 1 0 10 32 0 1\n"
                             "!UBX CFG-GNSS 0 32 32 1 6 8 16 0 1\n"
                             "!UBX CFG-MSG 3 15 0 1 0 1 0 0\n"
                             "!UBX CFG-MSG 3 16 0 1 0 1 0 0\n"
                             "!UBX CFG-MSG 1 32 0 1 0 1 0 0\n";
            uart_write_bytes(UART_NUM_1, initstr, sizeof(initstr));
#endif
            int sock_ok = 1;
            ESP_LOGI(TAG, "connected");

#if 1
            {
                // Set tcp keepalive option
                int keepAlive = 1;
                int keepIdle = KEEPALIVE_IDLE;
                int keepInterval = KEEPALIVE_INTERVAL;
                int keepCount = KEEPALIVE_COUNT;            
                setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
                setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
                setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
                setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
            }
#endif
            
            while(sock_ok == 1)
            {
                int len = 0;

                len = recv(sock, gps_rx_buffer, RXBUF_SIZE, MSG_DONTWAIT);

                if(len > 0)
                {
                    gps_rx_buffer[len] = 0;
                    ESP_LOGW(TAG, "TXD[%d]:%s", len, gps_rx_buffer);
                    uart_write_bytes(UART_NUM_1, (const char*)gps_rx_buffer, len);
                }
                else if(len < 0 && errno != EWOULDBLOCK)
                {
                    ESP_LOGE(TAG, "%d",len);
                    sock_ok = -1;
                    continue;
                }

                //ESP_LOGW(TAG, "w %d",(1+(RXBUF_SIZE/(10*5))));
                // Read data from the UART
                len = uart_read_bytes(UART_NUM_1, gps_rx_buffer, RXBUF_SIZE, (1+(RXBUF_SIZE/(10*3))) / portTICK_RATE_MS);
                if(len > 0)
                {
                    gps_rx_buffer[len] = 0;
                    //ESP_LOGW(TAG, "tx %d",len);
                    int err = send(sock, gps_rx_buffer, len, 0);
                    if(err < 0)
                    {
                        ESP_LOGE(TAG, "%d %d",len,err);
                        sock_ok = -2;
                        continue;
                    }
                }
            }
            
            if(sock != -1)
            {
                ESP_LOGE(TAG, "Shutting down socket (%d) and restarting...",sock_ok);
                shutdown(sock, 0);
                close(sock);
                sock = -1;                
            }
            
            //shutdown(listen_sock, 0);            
            //close(listen_sock);
            //listen_sock = -1;
        }
    }
    vTaskDelete(NULL);
}

void gps_init()
{
    xTaskCreate(gps_server_task, "gps_server", 4096, NULL, 5, NULL);
}

void gps_exit()
{
}

#endif
