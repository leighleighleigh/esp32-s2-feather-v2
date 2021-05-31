
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

typedef enum {
    CMD_ACK, /**/
    BTM_STATUS, /**/
    CMD_UNKNOWN
} bm83_event_id_t;