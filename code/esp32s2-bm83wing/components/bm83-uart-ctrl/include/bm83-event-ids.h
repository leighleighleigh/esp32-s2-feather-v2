
#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include "esp_types.h"
#include "esp_event.h"
#include "esp_err.h"

typedef struct {
    uint8_t command_id;
    uint8_t status;
} cmd_ack_t;


typedef struct {
    uint8_t sub_evt;
    uint8_t payload[6];
    size_t payload_len;
} le_signal_evt_t;

typedef enum {
    CMD_ACK, /**/
    BTM_STATUS, /**/
    CMD_UNKNOWN,
    LE_SIGNAL_EVT
} bm83_event_id_t;

// 0x24
// 0x2D
// 0x32
// TODO: 0x01 BTM status