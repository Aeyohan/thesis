/*
  PDM.cpp - library to interface with nRF52840 PDM peripheral
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2019 Arduino SA

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#include "pdm_port.h"
#define ARDUINO_ARCH_NRF52840
#if defined(ARDUINO_ARCH_NRF52840)

#include <hal/nrf_pdm.h>
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf.h"
#define DEFAULT_PDM_GAIN     20
#define PDM_IRQ_PRIORITY     7

#define NRF_PDM_FREQ_1280K  (nrf_pdm_freq_t)(0x0A000000UL)               ///< PDM_CLK= 1.280 MHz (32 MHz / 25) => Fs= 20000 Hz
#define NRF_PDM_FREQ_2000K  (nrf_pdm_freq_t)(0x10000000UL)               ///< PDM_CLK= 2.000 MHz (32 MHz / 16) => Fs= 31250 Hz
#define NRF_PDM_FREQ_2667K  (nrf_pdm_freq_t)(0x15000000UL)               ///< PDM_CLK= 2.667 MHz (32 MHz / 12) => Fs= 41667 Hz
#define NRF_PDM_FREQ_3200K  (nrf_pdm_freq_t)(0x19000000UL)               ///< PDM_CLK= 3.200 MHz (32 MHz / 10) => Fs= 50000 Hz
#define NRF_PDM_FREQ_4000K  (nrf_pdm_freq_t)(0x20000000UL)               ///< PDM_CLK= 4.000 MHz (32 MHz /  8) => Fs= 62500 Hz


#define PIN_OUT NRF_GPIO_PIN_MAP(0,13)


void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_drv_gpiote_out_toggle(PIN_OUT);
    NRF_LOG_INFO("in_pin_handler_tick");
    NRF_LOG_FLUSH();
}

PDMClass::PDMClass(int dinPin, int clkPin, int pwrPin) :
  _dinPin(dinPin),
  _clkPin(clkPin),
  _pwrPin(pwrPin),
  _onReceive(NULL)
{
}

PDMClass::~PDMClass()
{
}

int PDMClass::begin(int channels, long sampleRate)
{
    
  _channels = channels;

  // Enable high frequency oscillator if not already enabled
  if (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }
  }
  

  // configure the sample rate and channels
  switch (sampleRate) {
    case 16000:
      NRF_PDM->RATIO = ((PDM_RATIO_RATIO_Ratio80 << PDM_RATIO_RATIO_Pos) & PDM_RATIO_RATIO_Msk);
      nrf_pdm_clock_set(NRF_PDM_FREQ_1280K);
      break;
    case 41667:
      nrf_pdm_clock_set(NRF_PDM_FREQ_2667K);
      break;
    default:
      return 0; // unsupported
  }
  

  switch (channels) {
    case 2:
      nrf_pdm_mode_set(NRF_PDM_MODE_STEREO, NRF_PDM_EDGE_LEFTFALLING);
      break;

    case 1:
      nrf_pdm_mode_set(NRF_PDM_MODE_MONO, NRF_PDM_EDGE_LEFTFALLING);
      break;

    default:
      return 0; // unsupported
  }
  

  setGain(DEFAULT_PDM_GAIN);  

  // configure the I/O and mux
    
    
    

//   nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false); // init low
  
    nrf_gpio_cfg_output(_clkPin);

    
  
  
    nrf_gpio_cfg_input(_dinPin, NRF_GPIO_PIN_PULLUP);
    

    // nrf_drv_gpiote_in_event_enable(_dinPin, true);
  

    nrf_pdm_psel_connect(_clkPin, _dinPin);

    // clear events and enable PDM interrupts
    nrf_pdm_event_clear(NRF_PDM_EVENT_STARTED);
    nrf_pdm_event_clear(NRF_PDM_EVENT_END);
    nrf_pdm_event_clear(NRF_PDM_EVENT_STOPPED);
    nrf_pdm_int_enable(NRF_PDM_INT_STARTED | NRF_PDM_INT_STOPPED);

//   if (_pwrPin > -1) {
//     // power the mic on
//     pinMode(_pwrPin, OUTPUT);
//     digitalWrite(_pwrPin, HIGH);
//   }

  // clear the buffer
  _doubleBuffer.reset();

  // set the PDM IRQ priority and enable
  NVIC_SetPriority(PDM_IRQn, PDM_IRQ_PRIORITY);
  NVIC_ClearPendingIRQ(PDM_IRQn);
  NVIC_EnableIRQ(PDM_IRQn);

  // set the buffer for transfer
  // nrf_pdm_buffer_set((uint32_t*)_doubleBuffer.data(), _doubleBuffer.availableForWrite() / (sizeof(int16_t) * _channels));
  // _doubleBuffer.swap();
  
  // enable and trigger start task
  nrf_pdm_enable();
  nrf_pdm_event_clear(NRF_PDM_EVENT_STARTED);
  nrf_pdm_task_trigger(NRF_PDM_TASK_START);

  return 1;
}

void PDMClass::end()
{
  // disable PDM and IRQ
  nrf_pdm_disable();

  NVIC_DisableIRQ(PDM_IRQn);

//   if (_pwrPin > -1) {
//     // power the mic off
//     digitalWrite(_pwrPin, LOW);
//     pinMode(_pwrPin, INPUT);
//   }

  // Don't disable high frequency oscillator since it could be in use by RADIO

  // unconfigure the I/O and un-mux
  nrf_pdm_psel_disconnect();
  // ret_code_t err_code;
  // nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  // in_config.pull = NRF_GPIO_PIN_PULLUP;

  // err_code = nrf_drv_gpiote_in_init(_clkPin, &in_config, in_pin_handler);
  //   APP_ERROR_CHECK(err_code);
}

int PDMClass::available()
{
  NVIC_DisableIRQ(PDM_IRQn);

  size_t avail = _doubleBuffer.available();

  NVIC_EnableIRQ(PDM_IRQn);

  return avail;
}

int PDMClass::read(void* buffer, size_t size)
{
  NVIC_DisableIRQ(PDM_IRQn);

  int read = _doubleBuffer.read(buffer, size);

  NVIC_EnableIRQ(PDM_IRQn);

  return read;
}

void PDMClass::onReceive(void(*function)(void))
{
  _onReceive = function;
}

void PDMClass::setGain(int gain)
{
  nrf_pdm_gain_set(gain, gain);
}

void PDMClass::setBufferSize(int bufferSize)
{
  _doubleBuffer.setSize(bufferSize);
}

void PDMClass::IrqHandler(bool halftranfer)
{
  if (nrf_pdm_event_check(NRF_PDM_EVENT_STARTED)) {
    nrf_pdm_event_clear(NRF_PDM_EVENT_STARTED);

    if (_doubleBuffer.available() == 0) {
      // switch to the next buffer
      nrf_pdm_buffer_set((uint32_t*)_doubleBuffer.data(), _doubleBuffer.availableForWrite() / (sizeof(int16_t) * _channels));

      // make the current one available for reading
      _doubleBuffer.swap(_doubleBuffer.availableForWrite());

      // call receive callback if provided
      if (_onReceive) {
        _onReceive();
      }
    } else {
      // buffer overflow, stop
      nrf_pdm_disable();
    }
  } else if (nrf_pdm_event_check(NRF_PDM_EVENT_STOPPED)) {
    nrf_pdm_event_clear(NRF_PDM_EVENT_STOPPED);
  } else if (nrf_pdm_event_check(NRF_PDM_EVENT_END)) {
    nrf_pdm_event_clear(NRF_PDM_EVENT_END);
  }
}

extern "C" {
  __attribute__((__used__)) void PDM_IRQHandler(void)
  {
    PDM.IrqHandler(true);
  }
}

PDMClass PDM(PIN_PDM_DIN, PIN_PDM_CLK, PIN_PDM_PWR);

#endif