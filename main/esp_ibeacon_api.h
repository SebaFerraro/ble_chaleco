// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



/****************************************************************************
*
* This file is for iBeacon definitions. It supports both iBeacon sender and receiver
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

#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"


/* Because current ESP IDF version doesn't support scan and adv simultaneously,
 * so iBeacon sender and receiver should not run simultaneously */
#define IBEACON_SENDER      0
#define IBEACON_RECEIVER    1
#define IBEACON_MODE IBEACON_RECEIVER

/* Major and Minor part are stored in big endian mode in iBeacon packet,
 * need to use this macro to transfer while creating or processing
 * iBeacon data */
#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00)>>8) + (((x)&0xFF)<<8))

/* Espressif WeChat official account can be found using WeChat "Yao Yi Yao Zhou Bian", 
 * if device advertises using ESP defined UUID. 
 * Please refer to http://zb.weixin.qq.com for further information. */
#define ESP_UUID    {0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1, 0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25}
#define ESP_MAJOR   10167
#define ESP_MINOR   61958


typedef struct {
    uint8_t flags[3];
    uint8_t length;
    uint8_t type;
    uint16_t service_id;
    uint8_t pad[7];
}__attribute__((packed)) esp_ble_ibeacon_head_t;

typedef struct {
    uint8_t key[16];
}__attribute__((packed)) esp_ble_ibeacon_vendor_t;


typedef struct {
    esp_ble_ibeacon_head_t ibeacon_head;
    esp_ble_ibeacon_vendor_t ibeacon_vendor;
}__attribute__((packed)) esp_ble_ibeacon_t;


/* For iBeacon packet format, please refer to Apple "Proximity Beacon Specification" doc */
/* Constant part of iBeacon data */
extern esp_ble_ibeacon_head_t ibeacon_common_head;

BOOLEAN esp_ble_is_ibeacon_packet (uint8_t *adv_data, uint8_t adv_data_len);

esp_err_t esp_ble_config_ibeacon_data (esp_ble_ibeacon_vendor_t *vendor_config, esp_ble_ibeacon_t *ibeacon_adv_data);