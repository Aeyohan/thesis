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

#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

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

//=============== Function Prototypes ===============
uint8_t init_usb(void);
uint8_t init_pdm(void);
uint8_t init_button(void);
uint8_t init_sd(void);

uint8_t pdm_tick(void);

static void fatfs_example(void);
uint8_t sd_prepare_file(void);
void log_filename(TCHAR* fname);
bool bcd_inc(char* character);
uint16_t get_str_size(int16_t* values, uint8_t size);
uint8_t log_values(int16_t* buffer, uint16_t size);
void wait_for_sd_idle(void);
//=============================================================

// ============== Defines =============
#define STATUS_OK 0

// PDM
#define DELAY_1_MS 1000

// SD
#define FILE_NAME   "NORDIC.TXT"
#define TEST_STRING "SD card example."

#define SDC_SCK_PIN     28    ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    31  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    2  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      20  ///< SDC chip select (CS) pin.
#define FILENAME_LEN 13

#define CHAR_OFFSET 48
#define LOG_BUFFER_SIZE 512
#define FILE_HEADER "value"

// ====================================

// ===============Global variables  ===========
// (to be part of a driver)

// PDM
uint64_t msElapsed = 0; // increments when recording ~ every ms
uint16_t durationRecord = 1000;
uint64_t longerCounter = 0;
nrfx_systick_state_t systick;

nrfx_systick_state_t delayTick;
uint16_t msCount = 0;

// SD
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

static FATFS fs;
static DIR dir;
static FILINFO fno;
static FIL file;

uint32_t bytes_written;
FRESULT ff_result;
DSTATUS disk_state = STA_NOINIT;
bool sdReady = false;

uint8_t fileCounter = 0;
char logFileName[16] = "logs/log000.txt";

char sdLogBuffer[LOG_BUFFER_SIZE];
bool skipped;
uint16_t samplesToSkip;



// ==================

/* Timer used to blink LED on DFU progress. */
// APP_TIMER_DEF(m_dfu_progress_led_timer);

// static void on_error(void)
// {
//     NRF_LOG_FINAL_FLUSH();

// #if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
//     // To allow the buffer to be flushed by the host.
//     nrf_delay_ms(100);
// #endif
// #ifdef NRF_DFU_DEBUG_VERSION
//     NRF_BREAKPOINT_COND;
// #endif
//     NVIC_SystemReset();
// }


// void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
// {
//     NRF_LOG_ERROR("app_error_handler err_code:%d %s:%d", error_code, p_file_name, line_num);
//     on_error();
// }

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



// void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
// {
//     NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
//     on_error();
// }


// void app_error_handler_bare(uint32_t error_code)
// {
//     NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
//     on_error();
// }


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
            nrfx_systick_get(&delayTick);
            msCount = 0;
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

int main(void) {
    uint32_t ret_val;

    if (init_usb() != STATUS_OK) {
        NRF_LOG_INFO("USB failed to init");
        NRF_LOG_FLUSH();    
    }

    if (init_pdm() != STATUS_OK) {
        NRF_LOG_INFO("PDM failed to init");
        NRF_LOG_FLUSH();  
    }

    if (init_button() != STATUS_OK) {
        NRF_LOG_INFO("Button and peripheral failed to init");
        NRF_LOG_FLUSH();  
    }

    NRF_LOG_INFO("initialising systick");
    NRF_LOG_FLUSH();
    
    /* Init systick driver */
    nrf_drv_systick_init();
    nrfx_systick_get(&systick);

    NRF_LOG_INFO("initialisation complete");
    NRF_LOG_FLUSH();
    uint8_t counter = 0;
    
    // fatfs_example();
    
    if (init_sd() != STATUS_OK) {
        NRF_LOG_INFO("SD initialisation failed to init, check SD card is inserted");
        NRF_LOG_FLUSH();
    }

    // sd_prepare_file();
    

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

        pdm_tick();

        
        
        
        
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

uint8_t init_usb(void) {
    uint32_t ret_val;
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
    return 0;
}

uint8_t init_pdm(void) {
    NRF_LOG_INFO("Configuring PDM");
    NRF_LOG_FLUSH();
    // configure the data receive callback
    PDM.onReceive(onPDMdata);
    micState = MIC_INITIALISED;
    // optionally set the gain, defaults to 20
    // PDM.setGain(30);
    
    
    NRF_LOG_INFO("Initialised PDM");
    NRF_LOG_FLUSH();
    return 0;
}

uint8_t init_button(void) {
    bsp_board_init(BSP_INIT_BUTTONS);

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
    return 0;
}

uint8_t init_sd(void) {
    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed. %d. %d", disk_state, disk_status(0));
        return 1;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed. %d.", ff_result);
        return 1;
    }
    return 0;
}

uint8_t pdm_tick(void) {
    
        if (readPending) {
            if (nrfx_systick_test(&delayTick, DELAY_1_MS)) {
                msCount++;
                nrfx_systick_get(&delayTick);
                nrf_drv_gpiote_out_toggle(LED_1);
            }

            if (msCount > 1500) { // wait 500ms after the button has been pushed
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
                skipped = false;
                samplesToSkip = 24000;
                nrf_drv_gpiote_out_clear(LED_1);
            }
        }    

        if (micState == MIC_RECORDING) {
            //  check if the recording period has elapsed
            if (skipped) {
                if (nrfx_systick_test(&systick, DELAY_1_MS)) {
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
            }
            


            // copy over any samples 

            if (samplesRead) {
                if (skipped) {
                    // print samples to the serial monitor or plotter
                    // NRF_LOG_INFO("captured %d samples", samplesRead);
                    NRF_LOG_FLUSH();
                    uint16_t limit = MAX_SAMPLES - largeSampleCount;
                    if (samplesRead < limit) {
                        limit = samplesRead;
                    }
                    short* smallPtr = &sampleBuffer[0];
                    short* largePtr = &largeBuffer[largeSampleCount];
                    memcpy(largePtr, smallPtr, sizeof(short) * limit);
                    largeSampleCount += limit;
                    // for (int i = 0; i < samplesRead && largeSampleCount + 1 < MAX_SAMPLES; i++) {
                    //     largeBuffer[largeSampleCount++] = sampleBuffer[i];
                    //     NRF_LOG_INFO("value: %d", sampleBuffer[i]);
                    // }
                    // NRF_LOG_INFO("sBuf: %d, lBuf %d", sampleBuffer[0], largeBuffer[0]);
                    // NRF_LOG_INFO("sBuf: %d, lBuf %d", sampleBuffer[10], largeBuffer[10]);
                    // NRF_LOG_INFO("sBuf: %d, lBuf %d", sampleBuffer[20], largeBuffer[20]);
                    // NRF_LOG_INFO("sBuf: %d, lBuf %d", sampleBuffer[50], largeBuffer[50]);
                    // NRF_LOG_INFO("sBuf: %d, lBuf %d", sampleBuffer[100], largeBuffer[100]);
                    // clear the read count
                } else {
                    largeSampleCount += samplesRead;
                    if (largeSampleCount >= samplesToSkip) {
                        skipped = true;
                        largeSampleCount = 0;
                    }
                    
                }
                
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
            
            sd_prepare_file();
            // break into 1024 batches
            // uint16_t batchSize = 1024;
            // uint8_t batchesComplete = 0;
            NRF_LOG_INFO("Writing to SD samples");
            NRF_LOG_FLUSH();
            nrf_drv_gpiote_out_clear(LED_1);
            // for (uint8_t i = 0; i < largeSampleCount; i++) {
            //     NRF_LOG_RAW_INFO("%d,", largeBuffer[i]);
            // }
            // NRF_LOG_RAW_INFO("\n\r");

            // for (uint8_t batch = 0; batch < largeSampleCount / batchSize; batch++) {
            //     NRF_LOG_INFO("Writing batch %d of %d.", batch, largeSampleCount/batchSize);
            //     NRF_LOG_FLUSH();
            //     log_values(&largeBuffer[batch * batchSize], batchSize);
            //     batchesComplete = batch;
            // }
            // log remaining
            // uint16_t remaining = largeSampleCount - (batchesComplete * batchSize);
            NRF_LOG_INFO("queued %d values to write", largeSampleCount);
            NRF_LOG_FLUSH();
            log_values(&largeBuffer[0], largeSampleCount);
            nrf_drv_gpiote_out_set(LED_1);
            NRF_LOG_INFO("Writing to SD complete");
            NRF_LOG_FLUSH();

            // start next recording at 0
            memcpy(&largeBuffer[0], 0, sizeof(short) * largeSampleCount);
            largeSampleCount = 0;
            micState = MIC_INITIALISED;
            PDM.end();
            
        }

        return 0;
}

static void fatfs_example(void) {
    

    

    
    
    NRF_LOG_RAW_INFO("");

    NRF_LOG_INFO("Writing to file " FILE_NAME "...");
    wait_for_sd_idle();
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".");
        return;
    }

    ff_result = f_write(&file, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    else
    {
        NRF_LOG_INFO("%d bytes written.", bytes_written);
    }

    (void) f_close(&file);
    return;
}


char str[9] = "testing\0";
uint8_t sd_prepare_file(void) {
    
    NRF_LOG_INFO("String test '%s', '%s'", str, "StringTest");
    NRF_LOG_INFO("\r\n Listing directory: /");
    if (disk_state == STA_NOINIT || disk_state == STA_NODISK) {
        // retry initialisation
        if (init_sd() != STATUS_OK) {
            return 1;
        }
    }
    // initialise filesystem
    wait_for_sd_idle();
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        NRF_LOG_INFO("Directory listing failed!");
        return 1;
    }
    NRF_LOG_INFO("\r\n Listing directory: /");
    do {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return 1;
        }

        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                NRF_LOG_RAW_INFO("   <DIR>   ");
                log_filename(fno.fname);
                NRF_LOG_RAW_INFO("\n\r");
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  ", fno.fsize);
                log_filename(fno.fname);
                NRF_LOG_RAW_INFO("\n\r");
            }
        }
    }
    while (fno.fname[0]);
    // NRF_LOG_RAW_INFO("\n");
    // attempt to open log folder
    wait_for_sd_idle();
    ff_result = f_opendir(&dir, "logs");
    if (ff_result)
    {
        NRF_LOG_INFO("log listing failed! %d.", ff_result);
        // attempting to create folder
        ff_result = FR_OK;
        ff_result = f_mkdir("logs");
        if (ff_result) {
            // create failed return
            NRF_LOG_INFO("log creation failed! %d.", ff_result);
            return 1; 
        }
        // attempt to navigate to directory
        wait_for_sd_idle();
        ff_result = f_opendir(&dir, "logs");
        if (ff_result) {
            // create failed return
            NRF_LOG_INFO("log folder operation failed! %d.", ff_result);
            return 2; 
        }
    }
    NRF_LOG_INFO("\r\n Listing directory: /logs");
    // also get the largest number and + 1
    char prefix[4] = "log";
    char suffix[5] = ".txt";
    char prefixc[4] = "LOG";
    char suffixc[5] = ".TXT";
    char largestFileNumber[4] = "";
    bool numberFound = false;
    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return 1;
        }

        if (fno.fname[0])
        {
            // if (fno.fattrib & AM_DIR)
            // {
            //     NRF_LOG_RAW_INFO("   <DIR>   ");
            //     log_filename(fno.fname);
            //     NRF_LOG_RAW_INFO("\n\r");
            // }
            // else
            // {
            //     NRF_LOG_RAW_INFO("%9lu  ", fno.fsize);
            //     log_filename(fno.fname);
            //     NRF_LOG_RAW_INFO("\n\r");
            // }
        }
        if (fno.fname[0] && !(fno.fattrib & AM_DIR)) {
            // check that the name matches the prefix and suffix
            // NRF_LOG_INFO("checking for log file");
            uint8_t intOkay = 0;
            if (fno.fname[0] == prefix[0] && fno.fname[1] == prefix[1] && fno.fname[2] == prefix[2]) {
                intOkay++;
            }
            if (fno.fname[6] == suffix[0] && fno.fname[7] == suffix[1] && fno.fname[8] == suffix[2] && fno.fname[9] == suffix[3]) {
                intOkay++;
            }
            if (fno.fname[0] == prefixc[0] && fno.fname[1] == prefixc[1] && fno.fname[2] == prefixc[2]) {
                intOkay++;
            }
            if (fno.fname[6] == suffixc[0] && fno.fname[7] == suffixc[1] && fno.fname[8] == suffixc[2] && fno.fname[9] == suffixc[3]) {
                intOkay++;
            }
            if (intOkay == 2) {
                // log down number
                largestFileNumber[0] = fno.fname[3];
                largestFileNumber[1] = fno.fname[4];
                largestFileNumber[2] = fno.fname[5];
                numberFound = true;
                // NRF_LOG_INFO("found a matching log file");

            }
        }
    }
    while (fno.fname[0]);

    // get the last item in the log folder:
    if (numberFound) {
        // get the next largest number and increment it
        if (bcd_inc(&largestFileNumber[2])) {
            // overflow occured repeat on the next digit
            if (bcd_inc(&largestFileNumber[1])) {
                // overflow occured again repeat on the next digit
                if (bcd_inc(&largestFileNumber[0])) {
                    // overflow on hundreds, no files available
                    return 1;
                }
            }    
        }
    } else {
        largestFileNumber[0] = '0';
        largestFileNumber[1] = '0';
        largestFileNumber[2] = '0';
    }
    // NRF_LOG_INFO("proposed filename: log%c%c%c.txt", largestFileNumber[0], largestFileNumber[1], largestFileNumber[2]);
        
    // create a new file handle
    sprintf(&logFileName[0], "logs/%s%c%c%c%s", prefix, largestFileNumber[0], largestFileNumber[1], largestFileNumber[2], suffix);
    NRF_LOG_INFO("proposed filename: %s", logFileName);
    wait_for_sd_idle();
    ff_result = f_open(&file, logFileName, FA_READ | FA_WRITE | FA_OPEN_APPEND);

    if (ff_result != FR_OK) {
        NRF_LOG_INFO("Unable to open or create file: log%c%c%c.txt", logFileName[3], logFileName[4], logFileName[5]);
        return 1;
    }
    
    ff_result = f_write(&file, FILE_HEADER, sizeof(FILE_HEADER) - 1, (UINT *) &bytes_written);
    if (ff_result != FR_OK) {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    (void) f_close(&file);
    // file successfully opened
    sdReady = true;
    

    return 0;
}

uint8_t log_values(int16_t* buffer, uint16_t size) {
    if (!sdReady) {
        NRF_LOG_INFO("SD not ready");  
    }

    // for (uint8_t i = 0; i < size; i++) {
    //     NRF_LOG_RAW_INFO("%d,", buffer[i]);
        
    // }
    // NRF_LOG_RAW_INFO("\n\r");
    // for (uint8_t i = 0; i < size; i++) {
    //     NRF_LOG_RAW_INFO("%d,", largeBuffer[i]);
    // }
    // NRF_LOG_RAW_INFO("\n\r");

    // uint8_t blockSize = 
    // memset(&sdLogBuffer[0], '\0', LOG_BUFFER_SIZE);
    // NRF_LOG_INFO("setting memory");
    // NRF_LOG_FLUSH();
    // uint8_t blockSize = 64; // 64 * 7 chars max per value = max of 448 chars 
    // uint16_t remainingBlocks = size / 16;
    uint16_t start = 0;
    wait_for_sd_idle();
    ff_result = f_open(&file, logFileName, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK) {
        NRF_LOG_INFO("Unable to open or create file: log%c%c%c.txt", logFileName[3], logFileName[4], logFileName[5]);
        return 1;
    }
    NRF_LOG_INFO("Opened file");
    NRF_LOG_FLUSH();
    // while (remainingBlocks > 0) {
    //     // also grab the size of the items
    //     uint16_t len = get_str_size(&buffer[start], blockSize); // + 1 for null terminator 
    //     // write  values in blocks of 64
    //     NRF_LOG_INFO("remaining blocks %d", remainingBlocks);
    //     NRF_LOG_FLUSH();
    //     sprintf(&sdLogBuffer[0], "\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n"
    //             "%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d", 
    //             buffer[start], buffer[start+1], buffer[start+2], buffer[start+3], buffer[start+4], buffer[start+5], buffer[start+6], buffer[start+7], 
    //             buffer[start+8], buffer[start+9], buffer[start+10], buffer[start+11], buffer[start+12], buffer[start+13], buffer[start+14], buffer[start+15], 
    //             buffer[start+16], buffer[start+17], buffer[start+18], buffer[start+19], buffer[start+20], buffer[start+21], buffer[start+22], buffer[start+23], 
    //             buffer[start+24], buffer[start+25], buffer[start+26], buffer[start+27], buffer[start+28], buffer[start+29], buffer[start+30], buffer[start+31], 
    //             buffer[start+32], buffer[start+33], buffer[start+34], buffer[start+35], buffer[start+36], buffer[start+37], buffer[start+38], buffer[start+39], 
    //             buffer[start+40], buffer[start+41], buffer[start+42], buffer[start+43], buffer[start+44], buffer[start+45], buffer[start+46], buffer[start+47], 
    //             buffer[start+48], buffer[start+49], buffer[start+50], buffer[start+51], buffer[start+52], buffer[start+53], buffer[start+54], buffer[start+55], 
    //             buffer[start+56], buffer[start+57], buffer[start+58], buffer[start+59], buffer[start+60], buffer[start+61], buffer[start+62], buffer[start+63]);
    //             start += 64;
    //     ff_result = f_write(&file, &sdLogBuffer[0], len / 2 * sizeof(char), (UINT *) &bytes_written);
    //     if (ff_result != FR_OK) {
    //         NRF_LOG_INFO("Write failed\r\n.");
    //         NRF_LOG_FLUSH();
    //     }
    //     NRF_LOG_INFO("log data %s", &sdLogBuffer[0]);
    //     f_sync(&file);
    //     // nrfx_systick_delay_ms(50);
    //     ff_result = f_write(&file, &sdLogBuffer[len / 2], len * sizeof(char), (UINT *) &bytes_written);
    //     if (ff_result != FR_OK) {
    //         NRF_LOG_INFO("Write failed\r\n.");
    //         NRF_LOG_FLUSH();
    //     }
    //     f_sync(&file);
    //     // reset buffer 
    //     memset(&sdLogBuffer[0], '\0', len); // only need to clean up what was written
    //     remainingBlocks--;
    //     // delay
        
    // }
    uint16_t complete = 0;
    uint8_t syncCount = 0;
    uint16_t divisor = 640;
    for (uint16_t i = 0; i < size; i += 64) {
        // uint16_t len = get_str_size(&buffer[i], blockSize);
        
        // sprintf(&sdLogBuffer[0], "\n%d", buffer[i]);
        // ff_result = f_write(&file, &sdLogBuffer[0], len * sizeof(char),
        // (UINT *) &bytes_written);
        short temp[64];
        memcpy(&temp[0], &buffer[i], sizeof(short) * 64);
        wait_for_sd_idle();
        int result = f_printf(&file, 
                "\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d"
                "\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d"
                "\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d"
                "\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d", 
                temp[0],temp[1],temp[2],temp[3],temp[4],temp[5],temp[6],temp[7],
                temp[8],temp[9],temp[10],temp[11],temp[12],temp[13],temp[14],temp[15],
                temp[16],temp[17],temp[18],temp[19],temp[20],temp[21],temp[22],temp[23],
                temp[24],temp[25],temp[26],temp[27],temp[28],temp[29],temp[30],temp[31],
                temp[32],temp[33],temp[34],temp[35],temp[36],temp[37],temp[38],temp[39],
                temp[40],temp[41],temp[42],temp[43],temp[44],temp[45],temp[46],temp[47],
                temp[48],temp[49],temp[50],temp[51],temp[52],temp[53],temp[54],temp[55],
                temp[56],temp[57],temp[58],temp[59],temp[60],temp[61],temp[62],temp[63]);
        if (result < 0) {
            NRF_LOG_INFO("Write failed\r\n.");
            NRF_LOG_FLUSH();
        }
        syncCount++;
        if (syncCount > 200) {
            syncCount = 0;
            f_sync(&file);
        } 
        if (i % divisor == 0) {
            NRF_LOG_INFO("progress: %d of %d ", i, size);
            NRF_LOG_FLUSH();
        }
        // memset(&sdLogBuffer[0], '\0', len + 1);
        complete = i;

    }
    for (uint16_t i = complete; i < size; i++) {
        wait_for_sd_idle();
        int result = f_printf(&file, "\n%d", buffer[i]);
        if (result < 0) {
            NRF_LOG_INFO("Write failed\r\n.");
            NRF_LOG_FLUSH();
        }        
    }
    wait_for_sd_idle();
    f_sync(&file);
    // uint16_t len = 0;
    // uint16_t position = 0;
    // NRF_LOG_INFO("Finishing up");
    // NRF_LOG_FLUSH();
    // // write the last remaining sub-block if necessary
    // while (start < size) {
    //     uint16_t temp = get_str_size(&buffer[start], 1); // 1 value at a time
    //     sprintf(&sdLogBuffer[len], "%d", buffer[start++]);
    //     len += temp;
    // }
    // NRF_LOG_INFO("block final write");
    // NRF_LOG_FLUSH();
    // ff_result = f_write(&file, &sdLogBuffer[0], len * sizeof(char), (UINT *) &bytes_written);
    // // reset buffer 
    // memset(&sdLogBuffer[0], '\0', len); // only need to clean up what was written
    
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    NRF_LOG_INFO("closing file handle");
    NRF_LOG_FLUSH();
    (void) f_close(&file);
    return 0;
}

uint16_t get_str_size(int16_t* values, uint8_t size) {
    uint16_t value = 0;

    for (uint8_t i = 0; i < size; i++) {
        uint8_t count = 2; // 1 digit + newline
        value = values[i];
        if (value < 0) {
            count++; // negative symbol
            value = value * -1; // invert for character recognition
        }
        if (value < 10) {
            ;;
        } else if (value < 100) {
            count++;
        } else if (value < 1000) {
            count += 2;
        } else if (value < 10000) {
            count += 3;
        } else {
            count += 4;
        }
        value += count;
    }
    return value;
}

// returns true on overflow
bool bcd_inc(char* character) {
    uint8_t value = character[0] - CHAR_OFFSET;
    if (value == 9) {
        character[0] = '0';
        return true;
    } else {
        character[0]++;
        return false;
    }

}

void log_filename(TCHAR* fname) {
    NRF_LOG_RAW_INFO("'");
    for (uint8_t i = 0; i < FILENAME_LEN; i++) {
        if (fname[i] != '\0') {
            NRF_LOG_RAW_INFO("%c", fname[i]);
        }
    }
    NRF_LOG_RAW_INFO("'");
}

void wait_for_sd_idle(void) {
    while (!app_sdc_idle()) {
        nrfx_systick_delay_us(10);
    }
}


