#include <stdio.h>
#include "esp_types.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "bm83-uart-ctrl.h"

static const char *BM83_TAG = "bm83_parser";

ESP_EVENT_DEFINE_BASE(ESP_BM83_EVENT);

/**
 * @brief BM83 Parser runtime buffer size
 *
 */
#define BM83_PARSER_RUNTIME_BUFFER_SIZE (CONFIG_BM83_PARSER_RING_BUFFER_SIZE / 2) // Buffer for UART data, within the bm83_runtime struct object
#define BM83_EVENT_LOOP_QUEUE_SIZE (16) // Queue for the event results. Larger means less likely to miss events during task handler in main code.

/**
 * @brief BM83 parser library runtime structure
 *
 */
typedef struct {
    /* Variables used for parsing */
    uint8_t start; // Start byte. Should be 0xAA.                                                
    uint16_t dlc; // DLC. Combination of both bytes (DLC_HI and DLC_LOW)        
    uint8_t *data; // Data buffer, contains opcode and arguments. DLC represents length.
    uint8_t crc; // CRC value
    bool valid; // Set true if CRC is valid! 

    /* Variables used to hold reference to objects */
    bm83_state_t parent;                                  /*!< Parent class */
    uart_port_t uart_port;                         /*!< Uart port number */
    uint8_t mfb_pin;                               /*!< MSB 'wakeup pin' for BM83 to exit low power mode */
    uint8_t *buffer;                               /*!< Runtime UART data buffer */
    esp_event_loop_handle_t event_loop_hdl;        /*!< Event loop handle. We post events here after parsing them. */
    TaskHandle_t tsk_hdl;                          /*!< BM83 Parser task handle */
    QueueHandle_t event_queue;                     /*!< UART event queue handle */
} bm83_runtime_t;

/**
 * @brief Init BM83 Parser
 *
 * @param config Configuration of BM83 Parser
 * @return bm83_parser_handle_t handle of bm83_parser
 */
bm83_parser_handle_t bm83_parser_init(const bm83_parser_config_t *config)
{
    /*  Initialise the runtime command buffer.
        This is where we store the runtime data (parser data).
    */
    bm83_runtime_t *bm83_runtime = calloc(1, sizeof(bm83_runtime_t));
    if (!bm83_runtime) {
        ESP_LOGE(BM83_TAG, "calloc memory for bm83_runtime failed");
        goto err_gps;
    }

    /* Initialise the runtime buffer size */
    bm83_runtime->buffer = calloc(1, BM83_PARSER_RUNTIME_BUFFER_SIZE);
    if (!bm83_runtime->buffer) {
        ESP_LOGE(BM83_TAG, "calloc memory for runtime buffer failed");
        goto err_buffer;
    }

    /* Set attributes for UART port etc */
    bm83_runtime->uart_port = config->uart.uart_port;
    bm83_runtime->mfb_pin = config->io.mfb_pin;
    
    /* Install UART driver */
    uart_config_t uart_config = {
        .baud_rate = config->uart.baud_rate,
        .data_bits = config->uart.data_bits,
        .parity = config->uart.parity,
        .stop_bits = config->uart.stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Install
    if (uart_driver_install(bm83_runtime->uart_port, CONFIG_BM83_PARSER_RING_BUFFER_SIZE, 0, config->uart.event_queue_size, &bm83_runtime->event_queue, 0) != ESP_OK) {
        ESP_LOGE(BM83_TAG, "install uart driver failed");
        goto err_uart_install;
    }

    // Set config
    if (uart_param_config(bm83_runtime->uart_port, &uart_config) != ESP_OK) {
        ESP_LOGE(BM83_TAG, "config uart parameter failed");
        goto err_uart_config;
    }

    // Set pins!
    if (uart_set_pin(bm83_runtime->uart_port, config->uart.tx_pin, config->uart.rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(BM83_TAG, "config uart gpio failed");
        goto err_uart_config;
    }
    
    /* Set pattern interrupt, used to detect the START of a line. 
        For microchip BM83/BM63 devices, this is 0xAA hex char.
    */
    uart_enable_pattern_det_baud_intr(bm83_runtime->uart_port, 0xAA, 1, 9, 0, 0);
    /* Set pattern queue size. This is influenced by buffer size too. */
    uart_pattern_queue_reset(bm83_runtime->uart_port, config->uart.event_queue_size);
    /* Flush any data already there */
    uart_flush(bm83_runtime->uart_port);

    /* Setup GPIO config for MFB pin */
    if(gpio_config(&config->io.io_conf) != ESP_OK)
    {
        ESP_LOGE(BM83_TAG,"install GPIO config failed");
        goto err_gpio_config;
    }

    // Rise MFB
    gpio_set_level(bm83_runtime->mfb_pin,1);
    
    /* Create Event loop. 
        This is how we signal the external (user) handlers that things have happened!
        Information may have updated in the global state (parent->(bm83_state_t)),
        or the information may be the presence of the Event itself.
    */
    esp_event_loop_args_t loop_args = {
        .queue_size = BM83_EVENT_LOOP_QUEUE_SIZE,
        .task_name = NULL
    };

    if (esp_event_loop_create(&loop_args, &bm83_runtime->event_loop_hdl) != ESP_OK) {
        ESP_LOGE(BM83_TAG, "create event loop faild");
        goto err_eloop;
    }

    /* Create BM83 Parser task. This is where the magic happens. */
    BaseType_t err = xTaskCreate(
                         bm83_parser_task_entry,
                         "bm83_parser",
                         CONFIG_BM83_PARSER_TASK_STACK_SIZE,
                         bm83_runtime,
                         CONFIG_BM83_PARSER_TASK_PRIORITY,
                         &bm83_runtime->tsk_hdl);
    if (err != pdTRUE) {
        ESP_LOGE(BM83_TAG, "create BM83 Parser task failed");
        goto err_task_create;
    }
    ESP_LOGI(BM83_TAG, "BM83 Parser init OK");

    return bm83_runtime; // Returns the runtime object!
    
    /*Error Handling*/
err_task_create:
    esp_event_loop_delete(bm83_runtime->event_loop_hdl);
err_eloop:
err_uart_install:
    uart_driver_delete(bm83_runtime->uart_port);
err_uart_config:
err_gpio_config:
err_buffer:
    free(bm83_runtime->buffer);
err_gps:
    free(bm83_runtime);
    return NULL;
}

/**
 * @brief Deinit BM83 Parser
 *
 * @param bm83_hdl handle of BM83 parser
 * @return esp_err_t ESP_OK on success,ESP_FAIL on error
 */
esp_err_t bm83_parser_deinit(bm83_parser_handle_t bm83_hdl)
{
    bm83_runtime_t *bm83_runtime = (bm83_runtime_t *)bm83_hdl;
    vTaskDelete(bm83_runtime->tsk_hdl);
    esp_event_loop_delete(bm83_runtime->event_loop_hdl);
    esp_err_t err = uart_driver_delete(bm83_runtime->uart_port);
    free(bm83_runtime->buffer);
    free(bm83_runtime);
    return err;
}

/**
 * @brief BM83 Parser Task Entry
 * @brief Handles UART events
 * @param arg runtime object of bm83 parser
 */
void bm83_parser_task_entry(void *arg)
{
    bm83_runtime_t *bm83_runtime = (bm83_runtime_t *)arg;
    uart_event_t event;

    while (1) {
        if (xQueueReceive(bm83_runtime->event_queue, &event, pdMS_TO_TICKS(200))) {
            switch (event.type) {
            case UART_DATA:
                break;
            case UART_FIFO_OVF:
                ESP_LOGW(BM83_TAG, "HW FIFO Overflow");
                uart_flush(bm83_runtime->uart_port);
                xQueueReset(bm83_runtime->event_queue);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGW(BM83_TAG, "Ring Buffer Full");
                uart_flush(bm83_runtime->uart_port);
                xQueueReset(bm83_runtime->event_queue);
                break;
            case UART_BREAK:
                ESP_LOGW(BM83_TAG, "Rx Break");
                break;
            case UART_PARITY_ERR:
                ESP_LOGE(BM83_TAG, "Parity Error");
                break;
            case UART_FRAME_ERR:
                ESP_LOGE(BM83_TAG, "Frame Error");
                break;
            case UART_PATTERN_DET:
                ESP_LOGI(BM83_TAG, "HANDLING UART PATTERN");   
                // Pattern handling code goes here
                // esp_handle_uart_pattern(bm83_runtime);
                break;
            default:
                ESP_LOGW(BM83_TAG, "unknown uart event type: %d", event.type);
                break;
            }
        }

        /* Drive the event loop! */
        esp_event_loop_run(bm83_runtime->event_loop_hdl, pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

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
esp_err_t bm83_parser_add_handler(bm83_parser_handle_t bm83_hdl, esp_event_handler_t event_handler, void *handler_args)
{
    bm83_runtime_t *bm83_runtime = (bm83_runtime_t *)bm83_hdl;
    return esp_event_handler_register_with(bm83_runtime->event_loop_hdl, ESP_BM83_EVENT, ESP_EVENT_ANY_ID,
                                           event_handler, handler_args);
}

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
esp_err_t bm83_parser_remove_handler(bm83_parser_handle_t bm83_hdl, esp_event_handler_t event_handler)
{
    bm83_runtime_t *bm83_runtime = (bm83_runtime_t *)bm83_hdl;
    return esp_event_handler_unregister_with(bm83_runtime->event_loop_hdl, ESP_BM83_EVENT, ESP_EVENT_ANY_ID, event_handler);
}

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
esp_err_t bm83_send_raw_immediate(bm83_parser_handle_t bm83_hdl, const void * raw_data, size_t data_length)
{
    bm83_runtime_t *bm83_runtime = (bm83_runtime_t *)bm83_hdl;    
    // Write data
    int write_len = uart_write_bytes(bm83_runtime->uart_port, raw_data, data_length);
    // Wait for completion    
    esp_err_t result = uart_wait_tx_done(bm83_runtime->uart_port, 100);
    return result;
}

