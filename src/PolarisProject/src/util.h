#include "Arduino.h"

#define BUZZER_TIMER_CHANNEL 0

void beep(uint16_t duration_ms);
void queue_beep(int64_t beep_start_micros, int64_t beep_duration_micros);
void updateAsyncBeep();
void outputSiren(int64_t duration_ms);
template <typename T> T median(T arr[], int len) {
  T temp[len];
  for (int i = 0; i < len; i++) {
    temp[i] = arr[i];
  }
  for (int i = 0; i < len; i++) {
    for (int j = i; j < len; j++) {
      if (temp[j] < temp[i]) {
        T t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }
  return temp[len/2];
}

enum FlightStage
{
  NOT_LAUNCHED,
  LAUNCHED,
  POST_APOGEE,
  LANDED,
};