#ifndef LPFFILTER_H_
#define LPFFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 15 Hz

* 0 Hz - 0.1 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 4.0999392809545485 dB

* 0.2 Hz - 7.5 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.15369300891751 dB

*/

#define LPFFILTER_TAP_NUM 191

typedef struct {
  double history[LPFFILTER_TAP_NUM];
  unsigned int last_index;
} lpfFilter;

void lpfFilter_init(lpfFilter* f);
void lpfFilter_put(lpfFilter* f, double input);
double lpfFilter_get(lpfFilter* f);

#endif