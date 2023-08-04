#include "util.h"
#include "pinout.h"

void beep(uint16_t duration_ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration_ms);
  digitalWrite(BUZZER_PIN, LOW);
}

static uint64_t beep_queue_start[10];
static uint64_t beep_queue_end[10];
static int8_t beep_queue_length = 0;

void queue_beep(int64_t beep_start_micros, int64_t beep_duration_micros) {
  if (beep_queue_length == 10) {
    return;
  }

  beep_queue_start[beep_queue_length] = beep_start_micros;
  beep_queue_end[beep_queue_length] = beep_start_micros + beep_duration_micros;
}

void updateAsyncBeep() {
  if (beep_queue_length == 0) {
    return;
  }
  int64_t now = esp_timer_get_time();
  
  if (now > beep_queue_start[0] && now < beep_queue_end[0]) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  if (now > beep_queue_end[0]) { // beep is over, shift queue down
    for (int i = 0; i < beep_queue_length; i++) {
      beep_queue_start[i] = beep_queue_start[i+1];
      beep_queue_end[i] = beep_queue_end[i+1];
    }
    beep_queue_length--;
  }
}