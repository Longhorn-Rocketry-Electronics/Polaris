#include "util.h"
#include "pinout.h"

void beep(uint16_t duration_ms) {
  return;
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
  beep_queue_length++;
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

void outputSiren(int64_t duration_ms) {
  // Output pwm wave to buzzer pin varying volume to simulate siren (goes from 0 to 100% volume every 1s)
  int64_t start = esp_timer_get_time();
  int64_t end = start + duration_ms * 1000;
  ledcSetup(BUZZER_TIMER_CHANNEL, 10000, 8);
  ledcAttachPin(BUZZER_PIN, 0);
  while (esp_timer_get_time() < end) {
    float volume = cos((esp_timer_get_time() - start) * 2 * M_PI / 1000000) * 0.5 + 0.5;
    ledcWrite(BUZZER_TIMER_CHANNEL, volume * 255);
    Serial.println(volume * 255);
    delay(10);
  }
  ledcDetachPin(BUZZER_PIN);
}

