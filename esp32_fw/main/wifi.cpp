#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wnm.h"
#include "esp_rrm.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "lwip/apps/sntp.h"
#include "hwconfig.h"

//#define CONFIG_ROOMING CONFIG_WIFI_ROOMING_RSSI_THRESHOLD
#define CONFIG_ROOMING 0

static char my_wifi_ssid[64] = {};
static char my_wifi_psk[64] = {};
static bool my_wifi_save_on_connected = false;

esp_ip4_addr_t s_ip_addr = {};
uint8_t s_ip_addr_changed = 1;

/* rrm ctx */
int rrm_ctx = 0;

/* FreeRTOS event group to signal when we are connected & ready to make a request */
EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

/* esp netif object representing the WIFI station */
static esp_netif_t *sta_netif = NULL;

static const char *TAG = "wifi_roaming";

static inline uint32_t WPA_GET_LE32(const uint8_t *a)
{
	return ((uint32_t) a[3] << 24) | (a[2] << 16) | (a[1] << 8) | a[0];
}

/**
 * @brief check_wifi_config
 */
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
 * @brief event_handler
 */
static void event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t*)event_data;
		if (disconn->reason == WIFI_REASON_ROAMING) {
			ESP_LOGI(TAG, "station roaming, do nothing");
		} else {
			esp_wifi_connect();
            
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            s_ip_addr.addr = 0;
            s_ip_addr_changed = 1;
		}
#ifdef CONFIG_PM_ENABLE
        //ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
#endif        

	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
#if CONFIG_ROOMING
		if (CONFIG_ROOMING) {
			ESP_LOGI(TAG, "setting rssi threshold as %d\n", CONFIG_ROOMING);
			esp_wifi_set_rssi_threshold(CONFIG_ROOMING);
		}
#endif
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP ) {
		ip_event_got_ip_t *data = (ip_event_got_ip_t*)event_data;
        s_ip_addr = data->ip_info.ip;
        ESP_LOGW(TAG, "SYSTEM_EVENT_STA_GOT_IP %d.%d.%d.%d",
            (s_ip_addr.addr >> 0) & 0xff, 
            (s_ip_addr.addr >> 8) & 0xff,
            (s_ip_addr.addr >> 16) & 0xff, 
            (s_ip_addr.addr >> 24) & 0xff);
        s_ip_addr_changed = 1;
     
#ifdef CONFIG_PM_ENABLE
        //ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
#endif

        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
#if 1
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, "pool.ntp.org");
        sntp_init();

        wifi_ap_record_t ap_info = {};
        esp_wifi_sta_get_ap_info(&ap_info);
        ESP_LOGW(TAG, "wifi connected rssi=%d ssid=%s bssid=%02x:%02x:%02x:%02x:%02x:%02x channel=%d|%d",ap_info.rssi,ap_info.ssid,ap_info.bssid[0],ap_info.bssid[1],ap_info.bssid[2],ap_info.bssid[3],ap_info.bssid[4],ap_info.bssid[5],ap_info.primary,ap_info.second);
#endif        
    } else {
        ESP_LOGW(TAG, "unhandled event_base=0x%08x event_id=0x%08x",(int)event_base,(int)event_id);
    } 
}

#ifndef WLAN_EID_MEASURE_REPORT
#define WLAN_EID_MEASURE_REPORT 39
#endif
#ifndef MEASURE_TYPE_LCI
#define MEASURE_TYPE_LCI 9
#endif
#ifndef MEASURE_TYPE_LOCATION_CIVIC
#define MEASURE_TYPE_LOCATION_CIVIC 11
#endif
#ifndef WLAN_EID_NEIGHBOR_REPORT
#define WLAN_EID_NEIGHBOR_REPORT 52
#endif
#ifndef ETH_ALEN
#define ETH_ALEN 6
#endif

#define MAX_NEIGHBOR_LEN 512
#if 1
static char * get_btm_neighbor_list(uint8_t *report, size_t report_len)
{
	size_t len = 0;
	const uint8_t *data;
	int ret = 0;

	/*
	 * Neighbor Report element (IEEE P802.11-REVmc/D5.0)
	 * BSSID[6]
	 * BSSID Information[4]
	 * Operating Class[1]
	 * Channel Number[1]
	 * PHY Type[1]
	 * Optional Subelements[variable]
	 */
#define NR_IE_MIN_LEN (ETH_ALEN + 4 + 1 + 1 + 1)

	if (!report || report_len == 0) {
		ESP_LOGI(TAG, "RRM neighbor report is not valid");
		return NULL;
	}

	char *buf = (char*)calloc(1, MAX_NEIGHBOR_LEN);
	data = report;

	while (report_len >= 2 + NR_IE_MIN_LEN) {
		const uint8_t *nr;
		char lci[256 * 2 + 1];
		char civic[256 * 2 + 1];
		uint8_t nr_len = data[1];
		const uint8_t *pos = data, *end;

		if (pos[0] != WLAN_EID_NEIGHBOR_REPORT ||
		    nr_len < NR_IE_MIN_LEN) {
			ESP_LOGI(TAG, "CTRL: Invalid Neighbor Report element: id=%u len=%u",
					data[0], nr_len);
			ret = -1;
			goto cleanup;
		}

		if (2U + nr_len > report_len) {
			ESP_LOGI(TAG, "CTRL: Invalid Neighbor Report element: id=%u len=%zu nr_len=%u",
					data[0], report_len, nr_len);
			ret = -1;
			goto cleanup;
		}
		pos += 2;
		end = pos + nr_len;

		nr = pos;
		pos += NR_IE_MIN_LEN;

		lci[0] = '\0';
		civic[0] = '\0';
		while (end - pos > 2) {
			uint8_t s_id, s_len;

			s_id = *pos++;
			s_len = *pos++;
			if (s_len > end - pos) {
				ret = -1;
				goto cleanup;
			}
			if (s_id == WLAN_EID_MEASURE_REPORT && s_len > 3) {
				/* Measurement Token[1] */
				/* Measurement Report Mode[1] */
				/* Measurement Type[1] */
				/* Measurement Report[variable] */
				switch (pos[2]) {
					case MEASURE_TYPE_LCI:
						if (lci[0])
							break;
						memcpy(lci, pos, s_len);
						break;
					case MEASURE_TYPE_LOCATION_CIVIC:
						if (civic[0])
							break;
						memcpy(civic, pos, s_len);
						break;
				}
			}

			pos += s_len;
		}

		ESP_LOGI(TAG, "RMM neigbor report bssid=" MACSTR
				" info=0x%x op_class=%u chan=%u phy_type=%u%s%s%s%s",
				MAC2STR(nr), WPA_GET_LE32(nr + ETH_ALEN),
				nr[ETH_ALEN + 4], nr[ETH_ALEN + 5],
				nr[ETH_ALEN + 6],
				lci[0] ? " lci=" : "", lci,
				civic[0] ? " civic=" : "", civic);

		/* neighbor start */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, " neighbor=");
		/* bssid */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, MACSTR, MAC2STR(nr));
		/* , */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, ",");
		/* bssid info */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, "0x%04x", WPA_GET_LE32(nr + ETH_ALEN));
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, ",");
		/* operating class */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, "%u", nr[ETH_ALEN + 4]);
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, ",");
		/* channel number */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, "%u", nr[ETH_ALEN + 5]);
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, ",");
		/* phy type */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, "%u", nr[ETH_ALEN + 6]);
		/* optional elements, skip */

		data = end;
		report_len -= 2 + nr_len;
	}

cleanup:
	if (ret < 0) {
		free(buf);
		buf = NULL;
	}
	return buf;
}

#else

/* Sample API to create neighbor list */
char * get_tmp_neighbor_list(uint8_t *report, size_t report_len)
{
#define MAC1 "00:01:02:03:04:05"
#define MAC2 "00:02:03:04:05:06"

	char * buf = calloc(1, MAX_NEIGHBOR_LEN);
	size_t len = 0;
	char *pos;
	if (!buf)
		return NULL;

	pos = buf;
	/* create two temp neighbors */
	/* format for neighbor list : neighbor=11:22:33:44:55:66,0x0000,81,3,7,0301ff */

	/* neighbor1 start */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, " neighbor=");
	/* bssid */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, MAC1);
	/* , */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, ",");
	/* bssid info */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, "0x0000");
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, ",");
	/* operating class */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, "81");
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, ",");
	/* channel number */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, "6");
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, ",");
	/* phy type */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, "7");
	/* optional elements, skip */

	/* neighbor2 start */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, " neighbor=");
	/* bssid */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, MAC2);
	/* , */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, ",");
	/* bssid info */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, "0x0000");
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, ",");
	/* operating class */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, "81");
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, ",");
	/* channel number */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, "6");
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, ",");
	/* phy type */
	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, "7");
	/* optional elements, skip */

	len += snprintf(pos + len, MAX_NEIGHBOR_LEN - len, " ");

#undef MAC1
#undef MAC2
	return buf;
}
#endif

/**
 * @brief neighbor_report_recv_cb
 */
void neighbor_report_recv_cb(void *ctx, const uint8_t *report, size_t report_len)
{
	int *val = (int*)ctx;
	uint8_t *pos = (uint8_t *)report;
	int cand_list = 0;

	if (!report) {
		ESP_LOGE(TAG, "report is null");
		return;
	}
	if (*val != rrm_ctx) {
		ESP_LOGE(TAG, "rrm_ctx value didn't match, not initiated by us");
		return;
	}
	/* dump report info */
	ESP_LOGI(TAG, "rrm: neighbor report len=%d", report_len);
	ESP_LOG_BUFFER_HEXDUMP(TAG, pos, report_len, ESP_LOG_INFO);

	/* create neighbor list */
	char *neighbor_list = get_btm_neighbor_list(pos + 1, report_len - 1);

	/* In case neighbor list is not present issue a scan and get the list from that */
	if (!neighbor_list) {
		/* issue scan */
		wifi_scan_config_t params;
		memset(&params, 0, sizeof(wifi_scan_config_t));
		if (esp_wifi_scan_start(&params, true) < 0) {
			goto cleanup;
		}
		/* cleanup from net802.11 */
		uint16_t number = 1;
		wifi_ap_record_t ap_records;
		esp_wifi_scan_get_ap_records(&number, &ap_records);
		cand_list = 1;
	}
	/* send AP btm query, this will cause STA to roam as well */
	esp_wnm_send_bss_transition_mgmt_query(REASON_FRAME_LOSS, neighbor_list, cand_list);

cleanup:
	if (neighbor_list)
		free(neighbor_list);
}

/**
 * @brief esp_bss_rssi_low_handler
 */
#if CONFIG_ROOMING
static void esp_bss_rssi_low_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	wifi_event_bss_rssi_low_t *event = (wifi_event_bss_rssi_low_t *)event_data;

	ESP_LOGI(TAG, "%s:bss rssi is=%d", __func__, event->rssi);
	/* Lets check channel conditions */
	rrm_ctx++;
	if (esp_rrm_send_neighbor_rep_request(neighbor_report_recv_cb, &rrm_ctx) < 0) {
		/* failed to send neighbor report request */
		ESP_LOGI(TAG, "failed to send neighbor report request");
		if (esp_wnm_send_bss_transition_mgmt_query(REASON_FRAME_LOSS, NULL, 0) < 0) {
			ESP_LOGI(TAG, "failed to send btm query");
		}
	}
}
#endif

/**
 * @brief initialise_wifi
 */
void initialise_wifi(void)
{
	ESP_ERROR_CHECK(esp_netif_init());
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	sta_netif = esp_netif_create_default_wifi_sta();
	assert(sta_netif);

#if CONFIG_ROOMING
#if !defined(CONFIG_WPA_11KV_SUPPORT) || !defined(CONFIG_WPA_SCAN_CACHE)
#error enable CONFIG_WPA_11KV_SUPPORT
#error enable CONFIG_WPA_SCAN_CACHE
#endif
#endif
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
	ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
#if CONFIG_ROOMING
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_BSS_RSSI_LOW,
				&esp_bss_rssi_low_handler, NULL));
#endif

	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, my_wifi_ssid);
    strcpy((char*)wifi_config.sta.password, my_wifi_psk);
#ifdef CONFIG_PM_ENABLE
    wifi_config.sta.listen_interval = 10;
#endif
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.rm_enabled = 1;
    wifi_config.sta.btm_enabled = 1;

	ESP_LOGI(TAG, "Setting WiFi configuration SSID %s ...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
	ESP_ERROR_CHECK( esp_wifi_start() );
    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, HOSTNAME);
#ifdef CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
#endif
}

/**
 * EOF
 */
