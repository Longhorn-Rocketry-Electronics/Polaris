#include "driver/adc.h"

#define BUZZER_PIN 3
#define MAIN_CTRL 4
#define DROGUE_CTRL 5
#define MAIN_DETECT 6
#define DROGUE_DETECT 7
#define MAIN_DETECT_ADC_CHANNEL ADC1_CHANNEL_5
#define DROGUE_DETECT_ADC_CHANNEL ADC1_CHANNEL_6

#define SENSOR_CLK 45
#define SENSOR_MISO 13
#define SENSOR_MOSI 14

#define SD_CLK 10 // "CLX"
#define SD_MISO 9 // "DAT0"
#define SD_MOSI 11 // "CMD"

#define BMP_CS 21
#define GYRO_CS 33
#define ACC_CS 34
#define SD_CS 12 // "CD/DAT3"

#define V5_ENABLE 40
#define V5_BOOST 41

#define SERVO_1 15
#define SERVO_2 16
#define SERVO_3 17
#define SERVO_4 18