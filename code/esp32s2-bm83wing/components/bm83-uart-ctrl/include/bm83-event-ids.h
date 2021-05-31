
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
    uint8_t state;
} event_btm_status_t;

typedef enum {
    CMD_ACK, /**/
    BTM_STATUS, /**/
    CMD_UNKNOWN,
    EVENT_BTM_STATUS,
    BTM_STATE_OBJECT
} bm83_event_id_t;

// 0x24
// 0x2D
// 0x32
// TODO: 0x01 BTM status