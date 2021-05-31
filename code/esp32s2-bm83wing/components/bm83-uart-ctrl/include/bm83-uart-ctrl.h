#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_types.h"
#include "esp_event.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/**
 * @brief Declare of BM83 Parser Event base
 *
 */
ESP_EVENT_DECLARE_BASE(ESP_BM83_EVENT);

/**
 * @brief BM83 Parser main Handle
 *
 */
typedef void *bm83_parser_handle_t;

/**
 * @brief Static struct that holds global state of the BM83. This is updated by parsed messages.
 *
 */
typedef struct {
    bool alive;
    bool a2dp;
} bm83_state_t;

/**
 * @brief Configuration of UART and IO to talk to the BM83 module.
 *
 */
typedef struct {
    struct {
        uart_port_t uart_port;        /*!< UART port number */
        uint32_t rx_pin;              /*!< UART Rx Pin number */
        uint32_t tx_pin;              /*!< UART Tx Pin number */
        uint32_t baud_rate;           /*!< UART baud rate */
        uart_word_length_t data_bits; /*!< UART data bits length */
        uart_parity_t parity;         /*!< UART parity */
        uart_stop_bits_t stop_bits;   /*!< UART stop bits length */
        uint32_t event_queue_size;    /*!< UART event queue size */
    } uart;                           /*!< UART specific configuration */
    struct {
        uint8_t mfb_pin;
        gpio_config_t io_conf;
    } io;
} bm83_parser_config_t;

// Default config (suitable for BM83 featherwing, and ESP32S2 feather)
#define BM83_CONFIG_DEFAULT()                           \
    {                                                   \
        .uart = {                                       \
            .uart_port = UART_NUM_1,                    \
            .baud_rate = 115200,                        \
            .rx_pin = GPIO_NUM_5,                       \
            .tx_pin = GPIO_NUM_4,                       \
            .data_bits = UART_DATA_8_BITS,              \
            .parity = UART_PARITY_DISABLE,              \
            .stop_bits = UART_STOP_BITS_1,              \
            .event_queue_size = 16                      \
        },                                              \
        .io = {                                         \
            .mfb_pin = GPIO_NUM_6,                      \
            .io_conf.intr_type = GPIO_INTR_DISABLE,     \
            .io_conf.mode = GPIO_MODE_OUTPUT,           \
            .io_conf.pin_bit_mask = (1ULL<<GPIO_NUM_6),   \
            .io_conf.pull_down_en = 0,                  \
            .io_conf.pull_up_en = 0                     \
        }                                               \
    }

/**
 * @brief Init BM83 Parser
 *
 * @param config Configuration of BM83 Parser
 * @return bm83_parser_handle_t handle of BM83 parser
 */
bm83_parser_handle_t bm83_parser_init(const bm83_parser_config_t *config);

/**
 * @brief Deinit BM83 Parser
 *
 * @param bm83_hdl handle of BM83 parser
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t bm83_parser_deinit(bm83_parser_handle_t bm83_hdl);

/**
 * @brief BM83 Parser Task Entry
 * @brief Handles UART events
 * @param arg runtime object of bm83 parser
 */
void bm83_parser_task_entry(void *arg);

/**
 * @brief Add user defined handler for BM83 parser
 *
 * @param bm83_hdl handle of BM83 parser
 * @param event_handler user defined event handler
 * @param handler_args handler specific arguments
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_NO_MEM: Cannot allocate memory for the handler
 *  - ESP_ERR_INVALIG_ARG: Invalid combination of event base and event id
 *  - Others: Fail
 */
esp_err_t bm83_parser_add_handler(bm83_parser_handle_t bm83_hdl, esp_event_handler_t event_handler, void *handler_args);

/**
 * @brief Remove user defined handler for BM83 parser
 *
 * @param bm83_hdl handle of BM83 parser
 * @param event_handler user defined event handler
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: Invalid combination of event base and event id
 *  - Others: Fail
 */
esp_err_t bm83_parser_remove_handler(bm83_parser_handle_t bm83_hdl, esp_event_handler_t event_handler);


/**
 * @brief Send UART data immediately
 *
 * @param bm83_hdl handle of BM83 parser
 * 
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG
 *  - Others: Fail
 */
esp_err_t bm83_send_raw_immediate(bm83_parser_handle_t bm83_hdl, const void * raw_data, size_t data_length);

/* CONFIG parameters */
void init(void);
