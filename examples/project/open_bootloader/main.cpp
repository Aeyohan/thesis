/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup bootloader_open_usb_main main.c
 * @{
 * @ingroup bootloader_open_usb
 * @brief Bootloader project main file for Open DFU over USB.
 *
 */
#include "sdk_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "boards.h"
// #include "nrf_mbr.h"
// #include "nrf_bootloader.h"
// #include "nrf_bootloader_app_start.h"
// #include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "app_error_weak.h"
// #include "nrf_bootloader_info.h"
// #include "nrf_dfu_utils.h"
#include "led_softblink.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "nrf_clock.h"

#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"

#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "bsp.h"

#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
// #include "bsp_cli.h"
// #include "nrf_cli.h"
// #include "nrf_cli_uart.h"

#include "pdm_port.h"

// #include "app_simple_timer.h"
#include "nrf_drv_systick.h"

#if NRF_CLI_ENABLED
/**
 * @brief CLI interface over UART
 */
NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            4);
#endif

/* Timer used to blink LED on DFU progress. */
// APP_TIMER_DEF(m_dfu_progress_led_timer);

static void on_error(void)
{
    NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
    // To allow the buffer to be flushed by the host.
    nrf_delay_ms(100);
#endif
#ifdef NRF_DFU_DEBUG_VERSION
    NRF_BREAKPOINT_COND;
#endif
    NVIC_SystemReset();
}


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NRF_LOG_ERROR("app_error_handler err_code:%d %s:%d", error_code, p_file_name, line_num);
    on_error();
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{
    assert_info_t assert_info =
    {
        .line_num    = line_num,
        .p_file_name = file_name,
    };

    NRF_LOG_ERROR("Error at: %s:%d", file_name, line_num);
    app_error_fault_handler(NRF_FAULT_ID_SDK_ASSERT, 0, (uint32_t)(&assert_info));

    UNUSED_VARIABLE(assert_info);
}



void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
    on_error();
}


void app_error_handler_bare(uint32_t error_code)
{
    NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
    on_error();
}


// static void dfu_progress_led_timeout_handler(void * p_context)
// {
//     app_timer_id_t timer = (app_timer_id_t)p_context;

//     uint32_t err_code = app_timer_start(timer,
//                                         APP_TIMER_TICKS(DFU_LED_CONFIG_PROGRESS_BLINK_MS),
//                                         p_context);
//     APP_ERROR_CHECK(err_code);

//     bsp_board_led_invert(BSP_BOARD_LED_1);
// }

// /**
//  * @brief Function notifies certain events in DFU process.
//  */
// static void dfu_observer(nrf_dfu_evt_type_t evt_type)
// {
//     static bool timer_created = false;
//     uint32_t err_code;

//     if (!timer_created)
//     {
//         err_code = app_timer_create(&m_dfu_progress_led_timer,
//                                     APP_TIMER_MODE_SINGLE_SHOT,
//                                     dfu_progress_led_timeout_handler);
//         APP_ERROR_CHECK(err_code);
//         timer_created = true;
//     }

//     switch (evt_type)
//     {
//         case NRF_DFU_EVT_DFU_FAILED:
//         case NRF_DFU_EVT_DFU_ABORTED:
//             err_code = led_softblink_stop();
//             APP_ERROR_CHECK(err_code);

//             err_code = app_timer_stop(m_dfu_progress_led_timer);
//             APP_ERROR_CHECK(err_code);

//             err_code = led_softblink_start(BSP_LED_1_MASK);
//             APP_ERROR_CHECK(err_code);

//             break;
//         case NRF_DFU_EVT_DFU_INITIALIZED:
//         {
//             bsp_board_init(BSP_INIT_LEDS);

//             if (!nrf_clock_lf_is_running())
//             {
//                 nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);
//             }
//             err_code = app_timer_init();
//             APP_ERROR_CHECK(err_code);

//             led_sb_init_params_t led_sb_init_param = LED_SB_INIT_DEFAULT_PARAMS(BSP_LED_1_MASK);

//             uint32_t ticks = APP_TIMER_TICKS(DFU_LED_CONFIG_TRANSPORT_INACTIVE_BREATH_MS);
//             led_sb_init_param.p_leds_port    = BSP_LED_1_PORT;
//             led_sb_init_param.on_time_ticks  = ticks;
//             led_sb_init_param.off_time_ticks = ticks;
//             led_sb_init_param.duty_cycle_max = 255;

//             err_code = led_softblink_init(&led_sb_init_param);
//             APP_ERROR_CHECK(err_code);

//             err_code = led_softblink_start(BSP_LED_1_MASK);
//             APP_ERROR_CHECK(err_code);
//             break;
//         }
//         case NRF_DFU_EVT_TRANSPORT_ACTIVATED:
//         {
//             uint32_t ticks = APP_TIMER_TICKS(DFU_LED_CONFIG_TRANSPORT_ACTIVE_BREATH_MS);
//             led_softblink_off_time_set(ticks);
//             led_softblink_on_time_set(ticks);
//             break;
//         }
//         case NRF_DFU_EVT_TRANSPORT_DEACTIVATED:
//         {
//             uint32_t ticks =  APP_TIMER_TICKS(DFU_LED_CONFIG_PROGRESS_BLINK_MS);
//             err_code = led_softblink_stop();
//             APP_ERROR_CHECK(err_code);

//             err_code = app_timer_start(m_dfu_progress_led_timer, ticks, m_dfu_progress_led_timer);
//             APP_ERROR_CHECK(err_code);

//             break;
//         }
//         default:
//             break;
//     }
// }
/**@brief Function for application main entry.
 */

#define BTN_CDC_DATA_SEND       0
#define BTN_CDC_NOTIFY_SEND     1

#define BTN_CDC_DATA_KEY_RELEASE        (bsp_event_t)(BSP_EVENT_KEY_LAST + 1)

#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1
#ifdef __cplusplus
extern "C" {
#endif
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);
#ifdef __cplusplus
}
#endif
#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 0;

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[1024]; // was 256
#define AUDIO_SAMPLE_RATE 16000
#define SAMPLE_DURATION 4
#define MAX_SAMPLES SAMPLE_DURATION * AUDIO_SAMPLE_RATE
#define TO_US (1000UL * 1000UL)
short largeBuffer[MAX_SAMPLES];
int largeSampleCount = 0;

// number of samples read
volatile int samplesRead;
volatile bool readPending;

typedef enum microphoneState {
    MIC_OFF = 0,
    MIC_INITIALISED = 1,
    MIC_WAITING = 2,
    MIC_RECORDING = 3,
    MIC_DATA_READY = 4
} microphoneState;

microphoneState micState = MIC_OFF;
void button_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void button_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    readPending = true;
}

static void led_blinking_setup()
{
    ret_code_t err_code;
    nrf_drv_gpiote_out_config_t ledConfig = GPIOTE_CONFIG_OUT_SIMPLE(false);
    err_code = nrf_drv_gpiote_out_init(LED_1, &ledConfig);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t buttonConfig = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    buttonConfig.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrf_drv_gpiote_in_init(BUTTON_1, &buttonConfig, button_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON_1, true);
}

void onPDMdata() {
    
    // query the number of bytes available
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    PDM.read(sampleBuffer, bytesAvailable);

    // 16-bit, 2 bytes per sample
    samplesRead = bytesAvailable / 2;
}

void queueRead(void);

void queueRead(void) {
    readPending = true;
}

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            // bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   READ_SIZE);
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            // bsp_board_led_off(LED_CDC_ACM_OPEN);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            // bsp_board_led_invert(LED_CDC_ACM_TX);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
            do
            {
                /*Get amount of data transfered*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0]);

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            0,
                                            READ_SIZE);
            } while (ret == NRF_SUCCESS);

            // bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}


static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            // bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            // bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

static void bsp_event_callback(bsp_event_t ev)
{
    ret_code_t ret;
    NRF_LOG_INFO("BSP Event '%d'", (unsigned int) ev);
    NRF_LOG_FLUSH();
    switch ((unsigned int)ev)
    {
        case CONCAT_2(BSP_EVENT_KEY_, BTN_CDC_DATA_SEND):
        {
            m_send_flag = 1;
            nrf_drv_gpiote_out_toggle(LED_1);
            readPending = true;
            NRF_LOG_INFO("Button Push");
            NRF_LOG_FLUSH();
            break;
        }
        
        case BTN_CDC_DATA_KEY_RELEASE :
        {
            m_send_flag = 0;
            break;
        }

        case CONCAT_2(BSP_EVENT_KEY_, BTN_CDC_NOTIFY_SEND):
        {
            ret = app_usbd_cdc_acm_serial_state_notify(&m_app_cdc_acm,
                                                       APP_USBD_CDC_ACM_SERIAL_STATE_BREAK,
                                                       false);
            UNUSED_VARIABLE(ret);
            break;
        }

        // case BSP_EVENT_KEY_0 :
        //     nrf_drv_gpiote_out_toggle(LED_1);
        //     readPending = true;
        //     NRF_LOG_INFO("Button Push");
        //     NRF_LOG_FLUSH();
        //     break;

        default:
            return; // no implementation needed
    }
}

static void init_bsp(void)
{
    // ret_code_t ret;
    // ret = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
    // APP_ERROR_CHECK(ret);
    
    // UNUSED_RETURN_VALUE(bsp_event_to_button_action_assign(BTN_CDC_DATA_SEND,
                                                        //   BSP_BUTTON_ACTION_RELEASE,
                                                        //   BTN_CDC_DATA_KEY_RELEASE));
    
    /* Configure LEDs */
    bsp_board_init(BSP_INIT_LEDS);
}

#if NRF_CLI_ENABLED
static void init_cli(void)
{
    ret_code_t ret;
    ret = bsp_cli_init(bsp_event_callback);
    APP_ERROR_CHECK(ret);
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
}
#endif

int main(void)
{
    uint32_t ret_val;
    nrfx_systick_state_t systick;

    // Must happen before flash protection is applied, since it edits a protected page.
    // nrf_bootloader_mbr_addrs_populate();

    // Protect MBR and bootloader code from being overwritten.
    // ret_val = nrf_bootloader_flash_protect(0, MBR_SIZE);
    // APP_ERROR_CHECK(ret_val);
    // ret_val = nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE);
    // APP_ERROR_CHECK(ret_val);
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    ret_val = NRF_LOG_INIT(app_timer_cnt_get);
    APP_ERROR_CHECK(ret_val);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("Application Starting");
    NRF_LOG_FLUSH();
    
    ret_val = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret_val);

    nrf_drv_clock_lfclk_request(NULL);

    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    ret_val = app_timer_init();
    APP_ERROR_CHECK(ret_val);

    init_bsp();
    #if NRF_CLI_ENABLED
        init_cli();
    #endif

    app_usbd_serial_num_generate();

    ret_val = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret_val);
    NRF_LOG_INFO("USBD CDC ACM example started.");

    NRF_LOG_FLUSH();
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_init(BSP_INIT_BUTTONS);

    bsp_board_led_invert(BSP_BOARD_LED_0);

    // ret_val = nrf_bootloader_init(dfu_observer);
    // APP_ERROR_CHECK(ret_val);

    // NRF_LOG_FLUSH();

    // NRF_LOG_ERROR("After main, should never be reached.");
    // NRF_LOG_FLUSH();

    // APP_ERROR_CHECK_BOOL(false);
    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret_val = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret_val);

    if (USBD_POWER_DETECTION)
    {
        ret_val = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret_val);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }

    NRF_LOG_INFO("Configuring PDM");
    NRF_LOG_FLUSH();
    // configure the data receive callback
    PDM.onReceive(onPDMdata);
    micState = MIC_INITIALISED;
    // optionally set the gain, defaults to 20
    // PDM.setGain(30);
    
    
    NRF_LOG_INFO("Initialised PDM");
    NRF_LOG_FLUSH();

    NRF_LOG_INFO("preparing button register");
    NRF_LOG_FLUSH();

    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Initialised Peripheral interconnect");
    NRF_LOG_FLUSH();
    if (!nrf_drv_gpiote_is_init()) {
        NRF_LOG_INFO("gpio is not initialised, initialising");
        NRF_LOG_FLUSH();
        
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("Initialised GPIOTE driver");
    NRF_LOG_FLUSH();
    
    }
    
    NRF_LOG_INFO("Starting button event config");
    NRF_LOG_FLUSH();

    led_blinking_setup();
    NRF_LOG_INFO("initialising systick");
    NRF_LOG_FLUSH();
    /* Init systick driver */
    nrf_drv_systick_init();
    nrfx_systick_get(&systick);

    NRF_LOG_INFO("initialisation complete");
    NRF_LOG_FLUSH();
    uint8_t counter = 0;
    uint32_t delay1MS = 1000;
    uint64_t longerCounter = 0;
    uint64_t msElapsed = 0; // increments when recording ~ every ms
    uint16_t durationRecord = 1000;
    

    while (true) {
        // NRF_LOG_INFO("Logging loop %i.", counter++);
        if (counter++ == 0) {
            bsp_board_led_invert(BSP_BOARD_LED_0);
            // NRF_LOG_INFO("tick");
            NRF_LOG_FLUSH();
        }

        if(longerCounter++ == 0)
        {
            
            static int  frame_counter;

            size_t size = sprintf(m_tx_buffer, "Hello USB CDC FA demo: %u\r\n", frame_counter);

            ret_val = app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);
            if (ret_val == NRF_SUCCESS)
            {
                ++frame_counter;
            }
        }

        if (readPending) {
            NRF_LOG_INFO("record button pushed");
            NRF_LOG_FLUSH();
            // read PDM sample and write to RTT viewer
            if (micState == MIC_INITIALISED) {
                if (!PDM.begin(1, 16000)) {
                    NRF_LOG_INFO("Failed to start PDM!");
                    NRF_LOG_FLUSH();
                } else {
                    NRF_LOG_INFO("Started Recording");
                    NRF_LOG_FLUSH();
                    micState = MIC_RECORDING;
                    nrfx_systick_get(&systick);
                    durationRecord = 1000;
                    msElapsed = 0;
                }
            } else {
                NRF_LOG_INFO("Mic busy");
                NRF_LOG_FLUSH();
            }
            readPending = false;
        }    

        if (micState == MIC_RECORDING) {
            //  check if the recording period has elapsed

            if (nrfx_systick_test(&systick, delay1MS)) {
                msElapsed++;
                nrfx_systick_get(&systick);
            }
            if (msElapsed > durationRecord) {
                NRF_LOG_INFO("Recorded %ds", durationRecord/1000);
                NRF_LOG_FLUSH();
                durationRecord += 1000;
            }

            if (msElapsed > SAMPLE_DURATION * 1000) {
                micState = MIC_DATA_READY;

            }


            // copy over any samples 

            if (samplesRead) {

                // print samples to the serial monitor or plotter
                NRF_LOG_INFO("captured %d samples", samplesRead);
                NRF_LOG_FLUSH();
                for (int i = 0; i < samplesRead && largeSampleCount + i < MAX_SAMPLES; i++) {
                    largeBuffer[largeSampleCount++] = sampleBuffer[i];
                }

                // clear the read count
                samplesRead = 0;    
            }    
        }

        if (micState == MIC_DATA_READY) {
            // To-Do  write to SD
            
            // for now write it to console
            // for (int i = 0; i < largeSampleCount; i++) {
                // NRF_LOG_INFO("%d",largeBuffer[i]);
            // }
            NRF_LOG_INFO("Logged %d samples", largeSampleCount);
            NRF_LOG_FLUSH();
            // start next recording at 0
            largeSampleCount = 0;
            micState = MIC_INITIALISED;
            PDM.end();
        }

        
        
        
        
        // if(m_send_flag)
        // {
        //     bsp_board_led_invert(BSP_BOARD_LED_1);
        //     static int  frame_counter;

        //     size_t size = sprintf(m_tx_buffer, "Hello USB CDC FA demo: %u\r\n", frame_counter);

        //     ret_val = app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);
        //     if (ret_val == NRF_SUCCESS)
        //     {
        //         ++frame_counter;
        //     }
        // }
    }
}




/**
 * @}
 */


