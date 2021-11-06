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

// #include "ff.h"
// #include "diskio_blkdev.h"
// #include "nrf_block_dev_sdc.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "tensorflow/lite/micro/examples/hello_world/main_functions.h"
#include "tf_helper.h"
#include "tensorflow/lite/experimental/microfrontend/lib/frontend_handler.h"
#include "specg_small_compound.h"


#include "nrf_drv_timer.h"

#define QUANTISATION 1

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

uint8_t pdm_tick(void);

// static void fatfs_example(void);
// uint8_t sd_prepare_file(void);
bool bcd_inc(char* character);
uint16_t get_str_size(int16_t* values, uint8_t size);

uint8_t init_timer(void);
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

// // // SD
// // NRF_BLOCK_DEV_SDC_DEFINE(
// //         m_block_dev_sdc,
// //         NRF_BLOCK_DEV_SDC_CONFIG(
// //                 SDC_SECTOR_SIZE,
// //                 APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
// //          ),
// //          NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
// // );

// static FATFS fs;
// static DIR dir;
// static FILINFO fno;
// static FIL file;

// uint32_t bytes_written;
// FRESULT ff_result;
// DSTATUS disk_state = STA_NOINIT;
// bool sdReady = false;

// // uint8_t fileCounter = 0;
// char logFileName[16] = "logs/log000.txt";

// char sdLogBuffer[LOG_BUFFER_SIZE];
bool skipped;
uint16_t samplesToSkip;

uint16_t mainCounter;
nrfx_systick_state_t mainTimer;

uint16_t fileCounter = 0;


const nrf_drv_timer_t TIMER_RUN = NRF_DRV_TIMER_INSTANCE(0);
volatile uint64_t us_time_counter = 0;
bool timerInit = false;
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

void timer_run_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    us_time_counter++;
    nrf_drv_gpiote_out_toggle(LED_1);
    // switch (event_type) {
    //     case NRF_TIMER_EVENT_COMPARE0:
    //         break;
    //     default:
    //         //Do nothing.
    //         break;
    // }
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
// #define SAMPLE_DURATION 4
// #define SAMPLE_DURATION_MS SAMPLE_DURATION * 1000
// #define MAX_SAMPLES SAMPLE_DURATION * AUDIO_SAMPLE_RATE

#define SAMPLE_DURATION 0.65625
#define SAMPLE_DURATION_MS (int)(SAMPLE_DURATION * 1000)
#define MAX_SAMPLES 10500
#define TO_US (1000UL * 1000UL) 
int16_t largeBuffer[MAX_SAMPLES];
int largeSampleCount = 0;
int16_t *specGraph;

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
    nrf_drv_gpiote_out_set(LED_1);
    NRF_LOG_INFO("initialising systick");
    NRF_LOG_FLUSH();
    
    /* Init systick driver */
    nrf_drv_systick_init();
    nrfx_systick_get(&systick);

    
    uint8_t counter = 0;
    
    // fatfs_example();
    
    // if (init_sd() != STATUS_OK) {
    //     NRF_LOG_INFO("SD initialisation failed to init, check SD card is inserted");
    //     NRF_LOG_FLUSH();
    // }

    // sd_prepare_file();
    NRF_LOG_INFO("initialising Tensorflow");
    NRF_LOG_FLUSH();
    if (tf_init() != STATUS_OK) {
        NRF_LOG_INFO("Tensorsflow init failed.");
        NRF_LOG_FLUSH();
    }

    if (init_timer() != STATUS_OK) {
        NRF_LOG_INFO("Timer Failed to init");
    }

    NRF_LOG_INFO("initialisation complete");
    NRF_LOG_FLUSH();

    mainCounter = 0;
    nrfx_systick_get(&mainTimer);

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

        // Update timer and check if a reading is due
        if (nrfx_systick_test(&mainTimer, DELAY_1_MS * 100)) {
            mainCounter++;
            nrfx_systick_get(&mainTimer);
        }
        if (!readPending) {
            // currently delaying, incrememnt counter
            // nrfx_systick_get(&mainTimer);
            
            if (mainCounter >= 50) { // 30 seconds = 30s * 1000ms / 100 = 300
                // reset counter, 
                mainCounter = 0;
                // set recording flag
                readPending = true;
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

float spec_fl_buffer[32][32];
uint8_t pdm_tick(void) {
    
        if (readPending) {
            if (nrfx_systick_test(&delayTick, DELAY_1_MS)) {
                msCount++;
                nrfx_systick_get(&delayTick);
                nrf_drv_gpiote_out_set(LED_1);
            }

            if (msCount > 1500) { // wait 500ms after the button has been pushed
                NRF_LOG_INFO("record request recieved");
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

                if (msElapsed > SAMPLE_DURATION_MS) {
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
                    // largeSampleCount += limit;
                    // for (int i = 0; i < samplesRead && largeSampleCount + 1 < MAX_SAMPLES; i++) {
                    //     largeBuffer[largeSampleCount++] = (__fp16)sampleBuffer[i];
                    //     // NRF_LOG_INFO("value: %d", sampleBuffer[i]);
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
            uint64_t spec_time = 0;
            uint64_t cast_time = 0;
            uint64_t inference_time = 0; 
            
            // for now write it to console
            // for (int i = 0; i < largeSampleCount; i++) {
                // NRF_LOG_INFO("%d",largeBuffer[i]);
            // }
            NRF_LOG_INFO("Logged %d samples", largeSampleCount);
            NRF_LOG_FLUSH();
            
            // sd_prepare_file();
            // break into 1024 batches
            // uint16_t batchSize = 1024;
            // uint8_t batchesComplete = 0;
            // NRF_LOG_INFO("Writing to SD samples");
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
            // NRF_LOG_INFO("queued %d values to write", largeSampleCount);
            NRF_LOG_FLUSH();
            // log_values(&largeBuffer[0], largeSampleCount);
            
            // convert to spectogram
            uint16_t shiftsComp = 0;
            size_t sizeResult = 0;
            NRF_LOG_INFO("converting to spectograph");
            us_time_counter = 0;
            nrf_drv_timer_clear(&TIMER_RUN);
            nrf_drv_timer_resume(&TIMER_RUN);
            int16_t* spectogram = get_spectograph(&largeBuffer[0], &shiftsComp, &sizeResult); 
            nrf_drv_timer_pause(&TIMER_RUN);
            spec_time = us_time_counter;
            // spectogram is valid as long as get_spectrograph isn't called again.
            // NRF_LOG_INFO("Spectrogram completed windows completed: %d, (recent window completed %d.)", shiftsComp, sizeResult);
            NRF_LOG_FLUSH();
            // infer

            us_time_counter = 0;
            nrf_drv_timer_clear(&TIMER_RUN);
            nrf_drv_timer_resume(&TIMER_RUN);
            for (uint8_t i = 0; i < 32; i++) {
                for (uint8_t j = 0; j < 32; j++) {
                    spec_fl_buffer[i][j] = (float)(spectogram[((uint16_t)(i)) * 32 + j]);
                }
            }
            nrf_drv_timer_pause(&TIMER_RUN);
            cast_time = us_time_counter;


            set_input_tensor(&spec_fl_buffer[0][0]);
            uint8_t result = 0;
            us_time_counter = 0;
            nrf_drv_timer_clear(&TIMER_RUN);
            nrf_drv_timer_resume(&TIMER_RUN);
            uint8_t retVal = tf_tick(&result);
            nrf_drv_timer_pause(&TIMER_RUN);
            inference_time = us_time_counter;
            if (result) {
                // raining
                nrf_drv_gpiote_out_set(LED_1);
                NRF_LOG_INFO("Detected Rain");
            } else {
                nrf_drv_gpiote_out_clear(LED_1);
                NRF_LOG_INFO("Not detecting rain");
            }
            
            NRF_LOG_INFO("Completed Inference");
            // NRF_LOG_FLUSH();
            
            // NRF_LOG_INFO("Completed Inference with return %d", retVal);
            NRF_LOG_FLUSH();
            
            
            // start next recording at 0
            int16_t reset = 0;
            memcpy(&largeBuffer[0], &reset, sizeof(int16_t) * largeSampleCount);
            largeSampleCount = 0;
            micState = MIC_INITIALISED;
            PDM.end();

            NRF_LOG_INFO("preprocessing %d took %d ms, inference took %d ms, casting took %d ms\n\r", audio_tflite_model_len, spec_time, inference_time, cast_time);
            
        }

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

uint8_t init_timer(void) {

    uint32_t time_ms = 1;
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_RUN, &timer_cfg, timer_run_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_RUN, time_ms);

    nrf_drv_timer_extended_compare(
         &TIMER_RUN, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    
    nrf_drv_timer_enable(&TIMER_RUN);
    return 0;
}



