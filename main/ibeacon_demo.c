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

#define PIN_BLUE GPIO_NUM_23
#define PIN_RED GPIO_NUM_22
#define GPIO_OUTPUT_PINS  ((1ULL<<PIN_RED) | (1ULL<<PIN_BLUE))

static const char* DEMO_TAG = "IBEACON_DEMO";
extern esp_ble_ibeacon_vendor_t vendor_config;

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static uint8_t esp_euui[16]=ESP_UUID;

#if (IBEACON_MODE == IBEACON_RECEIVER)
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

#elif (IBEACON_MODE == IBEACON_SENDER)
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_NONCONN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
#endif


static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:{
#if (IBEACON_MODE == IBEACON_SENDER)
        esp_ble_gap_start_advertising(&ble_adv_params);
#endif
        break;
    }
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
#if (IBEACON_MODE == IBEACON_RECEIVER)
        //the unit of the duration is second, 0 means scan permanently
        uint32_t duration = 0;
        esp_ble_gap_start_scanning(duration);
#endif
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
                ESP_LOGI(DEMO_TAG, "----------Packet Found----------");
        	gpio_set_level(PIN_BLUE, 1);
		uint8_t baddr[6]={0};
		uint8_t bkey[16]={0};
		uint16_t bserv=0;
		int8_t bssi=0;
		bserv=ibeacon_data->ibeacon_head.service_id;		
		bssi=scan_result->scan_rst.rssi;
                ESP_LOGI(DEMO_TAG, "RSSI of packet:%d dbm", scan_result->scan_rst.rssi);
                ESP_LOGI(DEMO_TAG, "Service ID:%d ", ibeacon_data->ibeacon_head.service_id);
                ESP_LOGI(DEMO_TAG, "Packet rssi %d serv_id %d\n",bssi,bserv);
		if (bserv==0x1703){
                  ESP_LOGI(DEMO_TAG, "Packet Baliza.");
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
                  ESP_LOGI(DEMO_TAG, "Packet rssi %d igual %d\n",bssi,igual);
		  if((igual==1) & (bssi>-67)){
                     ESP_LOGI(DEMO_TAG, "Alarma Disparada.");
	             gpio_set_level(PIN_RED, 1);
		     vTaskDelay(2000 / portTICK_PERIOD_MS);
	             gpio_set_level(PIN_RED, 0);
		  }
		}
        	gpio_set_level(PIN_BLUE, 0);
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
#if (IBEACON_MODE == IBEACON_RECEIVER)
    esp_ble_gap_set_scan_params(&ble_scan_params);

#elif (IBEACON_MODE == IBEACON_SENDER)
    esp_ble_ibeacon_t ibeacon_adv_data;
    esp_err_t status = esp_ble_config_ibeacon_data (&vendor_config, &ibeacon_adv_data);
    if (status == ESP_OK){
        esp_ble_gap_config_adv_data_raw((uint8_t*)&ibeacon_adv_data, sizeof(ibeacon_adv_data));
    }
    else {
        ESP_LOGE(DEMO_TAG, "Config iBeacon data failed, status =0x%x\n", status);
    }
#endif
}

