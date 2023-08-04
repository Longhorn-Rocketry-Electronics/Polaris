#include "Arduino.h"

void beep(uint16_t duration_ms);
void queue_beep(int64_t beep_start_micros, int64_t beep_duration_micros);
void updateAsyncBeep();