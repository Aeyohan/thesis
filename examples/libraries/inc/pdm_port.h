#ifndef _PDM_PORT_H_
#define _PDM_PORT_H_

#include "PDMDoubleBuffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PIN_PDM_PWR (46)
#define PIN_PDM_CLK (25)
#define PIN_PDM_DIN (26)


#ifdef __cplusplus
}
#endif

class PDMClass
{
public:
  PDMClass(int dinPin, int clkPin, int pwrPin);
  virtual ~PDMClass();

  int begin(int channels, long sampleRate);
  void end();

  virtual int available();
  virtual int read(void* buffer, size_t size);

  void onReceive(void(*)(void));

  void setGain(int gain);
  void setBufferSize(int bufferSize);

// private:
  void IrqHandler(bool halftranfer);

private:
  int _dinPin;
  int _clkPin;
  int _pwrPin;

  int _channels;
  
  PDMDoubleBuffer _doubleBuffer;
  
  void (*_onReceive)(void);
};

extern PDMClass PDM;

#endif // _PDM_PORT_H_