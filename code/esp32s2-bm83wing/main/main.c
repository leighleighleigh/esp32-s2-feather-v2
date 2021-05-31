/* USB Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// DESCRIPTION:
// This example contains minimal code to make ESP32-S2 based device
// recognizable by USB-host devices as a USB Serial Device printing output from
// the application.

#include <stdio.h>
#include <stdlib.h>
#include "string.h"

#include <sys/reent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_console.h"
#include "esp_vfs_fat.h"


#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "sdkconfig.h"

#include "bm83-uart-ctrl.h"
#include "bm83-event-ids.h"

static const char *TAG = "main";

void init_usb_cdc(void)
{
    ESP_LOGI(TAG, "USB initialization");
    tinyusb_config_t tusb_cfg = { 0 }; // the configuration uses default values
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    tinyusb_config_cdcacm_t amc_cfg = { 0 }; // the configuration uses default values
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
    esp_tusb_init_console(TINYUSB_CDC_ACM_0); // log to usb
    ESP_LOGI(TAG, "USB initialization DONE");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait a bit for devices to register
}


static void util_print_unknown_bm83_event(void *event_data)
{
    uint8_t *buffer = event_data;

    // Check start byte
    if(buffer[0] != 0xAA)
    {
        return;
    }
    
    int16_t dlc = buffer[1] << 8 | buffer[2];
    
    printf("Unknown bytes [");
    for(int i = 0; i<dlc+4; i++)
    {
        printf("0x%X",buffer[i]);
        if(i != dlc+3){
            printf(",");
        }
    }
    printf("]\n");
}

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void bm83_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    bm83_state_t *bm83_state = NULL;
    cmd_ack_t *ack_msg = NULL;
    event_btm_status_t *event_btm_status = NULL;

    switch(event_id){
        case BTM_STATE_OBJECT:
            bm83_state = (bm83_state_t *)event_data;
            ESP_LOGI(TAG,"~~~ BM83 STATE OBJECT! ~~~");
            ESP_LOGI(TAG,"POWER %s",bm83_state->btm_power ? "ON" : "OFF");
            ESP_LOGI(TAG,"ACL %s",bm83_state->acl_link ? "ON" : "OFF");
            ESP_LOGI(TAG,"A2DP %s",bm83_state->a2dp_link ? "ON" : "OFF");
            ESP_LOGI(TAG,"AVRCP %s",bm83_state->avrcp_link ? "ON" : "OFF");
            ESP_LOGI(TAG,"~~~~~~~~~~~~~~~~~~~~~~~~~~");
            break;

        case CMD_ACK:
            ack_msg = (cmd_ack_t *)event_data;
            ESP_LOGI(TAG,"GOT ACK, CMD=0x%X, STAT=0x%X",ack_msg->command_id,ack_msg->status);
            break;
        
        case EVENT_BTM_STATUS:
            event_btm_status = (event_btm_status_t *)event_data;
            ESP_LOGI(TAG,"EVT BTM_STATUS, STATE=0x%X",event_btm_status->state);
            switch(event_btm_status->state)
            {
                case 0x0:
                    ESP_LOGI(TAG,"POWER OFF");
                    break;
                case 0x1:
                    ESP_LOGI(TAG,"PAIRING STATE (discoverable)");
                    break;
                case 0x2:
                    ESP_LOGI(TAG,"POWER ON");
                    break;
                case 0x3:
                    ESP_LOGI(TAG,"PAIRING SUCCESS");
                    break;
                case 0x4:
                    ESP_LOGI(TAG,"PAIRING FAIL");
                    break;
                case 0x5:
                    ESP_LOGI(TAG,"HF/HS LINK ESTABLISHED");
                    break;
                case 0x6:
                    ESP_LOGI(TAG,"A2DP LINK ESTABLISHED");
                    break;
                case 0x7:
                    ESP_LOGI(TAG,"HF LINK DISCONNECTED");
                    break;
                case 0x8:
                    ESP_LOGI(TAG,"A2DP LINK DISCONNECTED");
                    break;
                case 0x9:
                    ESP_LOGI(TAG,"SCO LINK CONNECTED");
                    break;
                case 0xA:
                    ESP_LOGI(TAG,"SCO LINK DISCONNECTED");    
                    break;
                case 0xB:
                    ESP_LOGI(TAG,"AVRCP LINK ESTABLISHED");
                    break;
                case 0xC:
                    ESP_LOGI(TAG,"AVRCP LINK DISCONNECTED");
                    break;
                case 0xD:
                    ESP_LOGI(TAG,"STANDARD SPP CONNECTED");
                    break;
                case 0xE:
                    ESP_LOGI(TAG,"STANDARD SPP/iAP DISCONNECTED");
                    break;
                case 0xF:
                    ESP_LOGI(TAG,"STANDBY STATE");
                    break;
                case 0x10:
                    ESP_LOGI(TAG,"iAP CONNECTED");
                    break;
                case 0x11:
                    ESP_LOGI(TAG,"ACL DISCONNECTED");
                    break;
                case 0x12:
                    ESP_LOGI(TAG,"MAP CONNECTED");
                    break;                
                case 0x13:
                    ESP_LOGI(TAG,"MAP OPERATION FORBIDDEN");
                    break;
                case 0x14:
                    ESP_LOGI(TAG,"MAP DISCONNECTED");
                    break;
                case 0x15:
                    ESP_LOGI(TAG,"ACL CONNECTED");
                    break;
                case 0x16:
                    ESP_LOGI(TAG,"SPP / iAP DISCONN NO PROFILE");
                    break;
            }
            break;

        case CMD_UNKNOWN:
            // ESP_LOGW(TAG, "Unknown command.");
            // util_print_unknown_bm83_event(event_data);
            break;
    }
    
    // switch (event_id) {
    //     case GPS_UPDATE:
    //         gps = (gps_t *)bm83_runtime_t;
    //         /* print information parsed from GPS statements */
    //         ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
    //                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
    //                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
    //                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
    //                 "\t\t\t\t\t\tspeed      = %fm/s",
    //                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
    //                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
    //                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
    //         break;
    //     case GPS_UNKNOWN:
    //         /* print unknown commands (this will be Hex) */
    //         ESP_LOGW(TAG, "Unknown command:%s", (char *)event_data);
    //         break;
    //     default:
    //         break;
    // }
    
}

void app_main(void)
{
    /* Setting TinyUSB up */
    init_usb_cdc();
    
    printf(".........................\n\r");
    printf("START bm83 initialisation\n\r");
    printf(".........................\n\r");

    /* Setup BM83 driver/parser/thingy */
    bm83_parser_config_t config = BM83_CONFIG_DEFAULT();
    bm83_parser_handle_t bm83_hdl = bm83_parser_init(&config);
    // Attach our event handler
    bm83_parser_add_handler(bm83_hdl, bm83_event_handler, NULL);
    printf("Done!\n");

    
    // Send a command (raw, direct, non-blocked)
    uint8_t CMD1[7] = {170,0,3,2,0,83,168}; // {170,0,3,2,0,83,168};
    bm83_send_raw_immediate(bm83_hdl, &CMD1, 7);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait a bit for devices to register

    uint8_t CMD2[7] = {170,0,3,2,0,84,167};
    bm83_send_raw_immediate(bm83_hdl, &CMD2, 7);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait a bit for devices to register

    uint8_t CMD3[7] = {170,0,3,2,0,81,170}; // {170,0,3,2,0,83,168};
    bm83_send_raw_immediate(bm83_hdl, &CMD3, 7);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait a bit for devices to register
    
    uint8_t CMD4[7] = {170,0,3,2,0,82,169}; // {170,0,3,2,0,83,168};
    bm83_send_raw_immediate(bm83_hdl, &CMD4, 7);
}