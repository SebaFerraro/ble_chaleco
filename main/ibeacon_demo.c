// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD // // Licensed under the Apache License, Version 2.0 (the "License"); // you may not use this file except in compliance with the License.  // You may obtain a copy of the License at //     http://www.apache.org/licenses/LICENSE-2.0 // // Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



/****************************************************************************
*
* This file is for iBeacon demo. It supports both iBeacon sender and receiver
* which is distinguished by macros IBEACON_SENDER and IBEACON_RECEIVER,
*
* iBeacon is a trademark of Apple Inc. Before building devices which use iBeacon technology,
* visit https://developer.apple.com/ibeacon/ to obtain a license.
*
****************************************************************************/

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

#define WIFI_SSID "Wifi_Interno"
#define WIFI_PASS "1nst1tuc10n4l"

#define PIN_BLUE GPIO_NUM_23
#define PIN_RED GPIO_NUM_22
#define GPIO_OUTPUT_PINS  ((1ULL<<PIN_RED) | (1ULL<<PIN_BLUE))

static const char* DEMO_TAG = "IBEACON_DEMO";
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
                ESP_LOGI(DEMO_TAG, "Packet rssi %d serv_id %d\n",bssi,bserv);
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
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
		
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    
	case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    
	case SYSTEM_EVENT_STA_DISCONNECTED:
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    
	default:
        break;
    }
   
	return ESP_OK;
}

void main_task(void *pvParameter)
{
	// wait for connection
	printf("Main task: waiting for connection to the wifi network... ");
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	printf("connected!\n");
	
	// print the local IP address
	tcpip_adapter_ip_info_t ip_info;
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
	printf("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
	printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
	printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));
	
	while(1) {
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void pkt_led(void *pvParameter)
{
	while(1) {
	   // wait for connection
      	      printf("Pkt Led: Esperando... ");
	      xEventGroupWaitBits(pkt_event_group, PKT_BIT, true, true, portMAX_DELAY);
	      printf("Pkt Led: activado\n");
              gpio_set_level(PIN_BLUE, 1);
	      vTaskDelay(500 / portTICK_RATE_MS);
     	      gpio_set_level(PIN_BLUE, 0);
	}
}

void alarm_led(void *pvParameter)
{
	while(1) {
	   // wait for connection
      	      printf("Alarm Led: Esperando... ");
	      xEventGroupWaitBits(alarm_event_group, ALARM_BIT, true, true, portMAX_DELAY);
	      printf("ALARM Led: activado\n");
	      gpio_set_level(PIN_RED, 1);
	      vTaskDelay(5000 / portTICK_PERIOD_MS);
	      gpio_set_level(PIN_RED, 0);
	}
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
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

    // initialize the wifi event handler
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	
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
       },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    printf("Connecting to %s\n", WIFI_SSID);
	
    // start the main task
    xTaskCreate(&main_task, "main_task", 2048, NULL, 5, NULL);
    xTaskCreate(&pkt_led, "pkt_led", 2048, NULL, 5, NULL);
    xTaskCreate(&alarm_led, "alarm_led", 2048, NULL, 5, NULL);
}

