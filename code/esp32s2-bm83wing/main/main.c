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
    for(int i = 0; i<dlc+2; i++)
    {
        printf("0x%X",buffer[i]);
        if(i != dlc+1){
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

    switch(event_id){
        case CMD_ACK:
            ack_msg = (cmd_ack_t *)event_data;
            ESP_LOGI(TAG,"GOT ACK, CMD=0x%X, STAT=0x%X",ack_msg->command_id,ack_msg->status);
            break;

        case CMD_UNKNOWN:
            ESP_LOGW(TAG, "Unknown command.");
            util_print_unknown_bm83_event(event_data);
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