#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "controller.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_ibeacon_api.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "driver/gpio.h"

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include <time.h>
#include <sys/time.h>
#include "esp_attr.h"
#include "esp_sleep.h"

#include "lwip/err.h"
#include "apps/sntp/sntp.h"


#define WIFI_SSID "Nieto"
#define WIFI_PASS "fun02480"

#define PIN_BLUE GPIO_NUM_23
#define PIN_RED GPIO_NUM_22
#define GPIO_OUTPUT_PINS  ((1ULL<<PIN_RED) | (1ULL<<PIN_BLUE))

/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int boot_count = 0;

static void obtain_time(void);
static void initialize_sntp(void);

static const char* DEMO_TAG = "CHALECO";
extern esp_ble_ibeacon_vendor_t vendor_config;

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static uint8_t esp_euui[16]=ESP_UUID;

// Event group
static EventGroupHandle_t wifi_event_group;
static EventGroupHandle_t pkt_event_group;
static EventGroupHandle_t alarm_event_group;
const int CONNECTED_BIT = BIT0;
const int PKT_BIT = BIT1;
const int ALARM_BIT = BIT2;


static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0xa0,
    .scan_window            = 0x90
};


static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:{
        break;
    }
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second, 0 means scan permanently
        uint32_t duration = 0;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(DEMO_TAG, "Scan start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //adv start complete event to indicate adv start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(DEMO_TAG, "Adv start failed");
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:{
            /* Search for BLE iBeacon Packet */
            if (esp_ble_is_ibeacon_packet(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len)){
                esp_ble_ibeacon_t *ibeacon_data = (esp_ble_ibeacon_t*)(scan_result->scan_rst.ble_adv);
                //ESP_LOGI(DEMO_TAG, "----------Packet Found----------");
		uint8_t baddr[6]={0};
		uint8_t bkey[16]={0};
		uint16_t bserv=0;
		int8_t bssi=0;
		bserv=ibeacon_data->ibeacon_head.service_id;		
		bssi=scan_result->scan_rst.rssi;
               // ESP_LOGI(DEMO_TAG, "RSSI of packet:%d dbm", scan_result->scan_rst.rssi);
                //ESP_LOGI(DEMO_TAG, "Service ID:%d ", ibeacon_data->ibeacon_head.service_id);
                //ESP_LOGI(DEMO_TAG, "Packet rssi %d serv_id %d\n",bssi,bserv);
		if (bserv==0x1703){
		  xEventGroupSetBits(pkt_event_group, PKT_BIT);
                  //ESP_LOGI(DEMO_TAG, "Packet Baliza.");
		  int igual=1;
		  for (int p=0;p<16;p++){
		    if(p<6)
		      baddr[p]=scan_result->scan_rst.bda[p];		
		    bkey[p]=ibeacon_data->ibeacon_vendor.key[p];
		    if(bkey[p] != esp_euui[p]){
                      ESP_LOGI(DEMO_TAG, "Packet key erronea.");
		      igual=0;
		    }
		  }
        	  gpio_set_level(PIN_BLUE, 0);
		  if((igual==1) & (bssi>-80)){
                     ESP_LOGI(DEMO_TAG, "Alarma Disparada.");
                     ESP_LOGI(DEMO_TAG, "Packet rssi %d igual %d\n",bssi,igual);
		     xEventGroupSetBits(alarm_event_group, ALARM_BIT);
		  }
		}
             }
            break;
	  }
        default:
            break;
        }
        break;
      }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(DEMO_TAG, "Scan stop failed");
        }
        else {
            ESP_LOGI(DEMO_TAG, "Stop scan successfully");
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(DEMO_TAG, "Adv stop failed");
        }
        else {
            ESP_LOGI(DEMO_TAG, "Stop adv successfully");
        }
        break;

    default:
        break;
    }
}


void ble_ibeacon_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(DEMO_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG, "gap register error, error code = %x", status);
        return;
    }

}

void ble_ibeacon_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_ibeacon_appRegister();
}


// Wifi event handler
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
   printf("Evento %d\n",event->event_id);
   switch(event->event_id) {
		
        case SYSTEM_EVENT_STA_START:
	     ESP_ERROR_CHECK(esp_wifi_connect());
	     printf("connectando...!\n");
        break;
    
	case SYSTEM_EVENT_STA_GOT_IP:
             xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
	     printf("tengo ip!\n");
        break;
	
	case SYSTEM_EVENT_STA_CONNECTED:
	     printf("Evento Conectado!\n");
        break;
	
	case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
	     printf("Evento authmode!\n");
        break;
    
	case SYSTEM_EVENT_STA_DISCONNECTED:
	     /* This is a workaround as ESP32 WiFi libs don't currently
             auto-reassociate. */
             esp_wifi_connect();
	     xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
	     printf("Desconectado !\n");
        break;
    
	default:
        break;
    }
   
	return ESP_OK;
}

void task_sntp(void *pvParameter){
    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    char strftime_buf[64];
    const int retry_count = 10;
    while(1){
        printf("obtain time: waiting for the wifi network...\n");
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
 	initialize_sntp();
	if (timeinfo.tm_year < (2017 - 1900)) {
           ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
           while(timeinfo.tm_year < (2017 - 1900) && ++retry < retry_count) {
                ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                time(&now);
                localtime_r(&now, &timeinfo);
            }
        }
        setenv("TZ", "ARG-3", 1);
        tzset();
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(TAG, "The current date/time in Argentina is: %s", strftime_buf);
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}


void main_task(void *pvParameter)
{
        time_t now;
        struct tm timeinfo;
	while(1) {
	   // wait for connection
	   printf("Main task: waiting for connection to the wifi network...\n");
	   xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	   printf("connected!\n");
	
	   // print the local IP address
	   tcpip_adapter_ip_info_t ip_info;
	   ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
	   printf("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
	   printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
	   printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));
	   printf("Sube Datos !\n");
	   vTaskDelay(5000 / portTICK_RATE_MS);
	
           vTaskDelay(200000 / portTICK_RATE_MS);
           ESP_ERROR_CHECK(esp_wifi_stop());
	   printf("Desconecto!\n");
	   vTaskDelay(200000 / portTICK_RATE_MS);
           ESP_ERROR_CHECK(esp_wifi_start());
	}
}

void pkt_led(void *pvParameter)
{
	while(1) {
	   // wait for connection
      	      printf("Pkt Led: Esperando...\n");
	      xEventGroupWaitBits(pkt_event_group, PKT_BIT, true, true, portMAX_DELAY);
              gpio_set_level(PIN_BLUE, 1);
	      vTaskDelay(500 / portTICK_RATE_MS);
     	      gpio_set_level(PIN_BLUE, 0);
	}
}

void alarm_led(void *pvParameter)
{
	while(1) {
	   // wait for connection
      	      printf("Alarm Led: Esperando...\n");
	      xEventGroupWaitBits(alarm_event_group, ALARM_BIT, true, true, portMAX_DELAY);
	      printf("ALARM Led: activado\n");
	      gpio_set_level(PIN_RED, 1);
	      vTaskDelay(5000 / portTICK_PERIOD_MS);
	      gpio_set_level(PIN_RED, 0);
	      printf("ALARM Led: Desactivado\n");
	}
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ++boot_count;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;  //disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PINS; //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = 0; //disable pull-down mode
    io_conf.pull_up_en = 0; //disable pull-up mode
    gpio_config(&io_conf);  //configure GPIO with the given settings
    

    ble_ibeacon_init();

    /* set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);
    // create the event group to handle wifi events
    wifi_event_group = xEventGroupCreate();
    pkt_event_group = xEventGroupCreate();
    alarm_event_group = xEventGroupCreate();

    // initialize the tcp stack
    tcpip_adapter_init();

    initialize_sntp();
    
    // initialize the wifi event handler
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
	
    // initialize the wifi stack in STAtion mode with config in RAM
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // configure the wifi connection and start the interface
    wifi_config_t wifi_config = {
       .sta = {
           .ssid = WIFI_SSID,
           .password = WIFI_PASS,
    //       .bssid_set = 0,
       },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    printf("Connecting to %s\n", WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_start());
	
    // start the main task
    xTaskCreate(&main_task, "main_task", 2048, NULL, 5, NULL);
    xTaskCreate(&pkt_led, "pkt_led", 2048, NULL, 5, NULL);
    xTaskCreate(&alarm_led, "alarm_led", 2048, NULL, 5, NULL);
    xTaskCreate(&task_sntp, "task_sntp", 2048, NULL, 5, NULL);
}

