#include <Arduino.h>
#include "driver/spi_master.h"
#include "SPI.h"
#include "pinout.h"
#include "bmp.h"
#include "bmi088.h"
#include "util.h"
#include "SD.h"
#include <driver/adc.h>
#include "Math3D.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

SPIClass sensor_spi = SPIClass(SPI2_HOST);
SPIClass sd_spi = SPIClass(SPI3_HOST);

BMI088 bmi088 = BMI088(&sensor_spi);

int64_t last_continuity_check_micros = 0;
int64_t last_motion_update_micros = 0;
int64_t last_pressure_update_micros = 0;
int64_t last_print_micros = 0;

File* log_file;
File log_file_file;
String log_file_name;
int log_file_count = 0;


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable   detector

  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(42,OUTPUT);
  pinMode(43,OUTPUT);
  pinMode(44,OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(BMP_CS, OUTPUT);
  digitalWrite(BMP_CS, HIGH);

  pinMode(GYRO_CS, OUTPUT);
  digitalWrite(GYRO_CS, HIGH);

  pinMode(ACC_CS, OUTPUT);
  digitalWrite(ACC_CS, HIGH);

  pinMode(SD_CS, OUTPUT);
  // digitalWrite(SD_CS, HIGH);

  digitalWrite(MAIN_CTRL, LOW); // DO NOT ACCIDENTALLY FIRE
  pinMode(MAIN_CTRL, OUTPUT);

  digitalWrite(DROGUE_CTRL, LOW); // DO NOT ACCIDENTALLY FIRE
  pinMode(DROGUE_CTRL, OUTPUT);

  pinMode(V5_ENABLE, OUTPUT);
  pinMode(V5_BOOST, OUTPUT);
  
  digitalWrite(V5_ENABLE, HIGH);
  digitalWrite(V5_BOOST, LOW);
  
  sensor_spi.begin(
    SENSOR_CLK,
    SENSOR_MISO,
    SENSOR_MOSI                                                  
  );
  
  sensor_spi.beginTransaction(
    SPISettings(
      100000,
      MSBFIRST,
      SPI_MODE3
    )
  );

  beep(250);
  delay(1500);

  bmp_init(&sensor_spi);  
  delay(100);

  while (!bmi088.isConnection()) {
    Serial.println("Waiting for BMI to be alive!");
    Serial.println(bmi088.getAccID());
    Serial.println(bmi088.getGyroID());
  }

  bmi088.initialize();
 
  Serial.println("Going to SD init");
  sd_spi.begin(
    SD_CLK,
    SD_MISO,
    SD_MOSI
  );

  int tries = 0;

  while (
    !SD.begin(
      SD_CS,
      sd_spi,
      12 * 1000 * 1000,
      "/sd",
      15
    ) && tries < 3
  ) {
    Serial.println("Waiting for SD to be alive!");
    tries++;
    beep(50);
  }

  if (SD.cardType() != CARD_NONE) {
    Serial.println("SD init done; SD Parameters:");
    Serial.println(SD.cardType());
    Serial.println(SD.cardSize());

    File f = SD.open("/config.txt", FILE_READ, true);

    if (f) {
      Serial.println("File opened!");
      while (f.available()) {
        Serial.write(f.read());
      }
      f.close();
      beep(500);
      delay(100);
    } else {
      Serial.println("File open failed!");
      beep(50);
      delay(100);
    }

    

    File w = SD.open("/log.txt", FILE_APPEND, true);

    if (w) {
      Serial.println("File opened!");
      w.println("START");
      w.close();
    } else {
      Serial.println(w.getWriteError());
      Serial.println("File open failed!");
    }

    // count the number of existing log-*.txt files to determine the next log file name
    while (SD.exists("/log-" + String(log_file_count) + ".txt")) {
      log_file_count++;
    }

    log_file_file = SD.open("/log-" + String(log_file_count) + ".txt", FILE_WRITE, true);
    log_file = &log_file_file;
    log_file_name = "/log-" + String(log_file_count) + ".txt";
  } else {
    Serial.println("SD init failed!");
    log_file = NULL;
  }

  BMPStartMeasurement();
  beep(1000);

  

}


bool output = false;
byte num = 0;
Quat AttitudeEstimateQuat;

Vec3 correction_Body, correction_World;
Vec3 Accel_Body, Accel_World;
Vec3 GyroVec;
const Vec3 VERTICAL = Vector(0.0f, 0.0f, 1.0f);  // vertical vector in the World frame
float ax, ay, az, gx, gy, gz, pressure, temperature, altitude;

void loop() {
  // put your main code here, to run repeatedly:
  updateAsyncBeep();


  // digitalWrite(42, output);
  // digitalWrite(43, !output);
  // digitalWrite(44, output);
  
  // output = !output;

  // Serial.println("Hello World");

  if (esp_timer_get_time() - last_continuity_check_micros >= 5000 * 1000) { // Check continuity every 5 seconds
    uint32_t main_detect, drogue_detect;

    main_detect = analogReadMilliVolts(A5);
    drogue_detect = analogReadMilliVolts(A6);

    if (main_detect < 500) {
      queue_beep(esp_timer_get_time(), 25 * 1000); // 25ms
    }

    if (drogue_detect < 500) {
      queue_beep(esp_timer_get_time() + 100 * 1000, 25 * 1000); // 25ms
      queue_beep(esp_timer_get_time() + 200 * 1000, 25 * 1000); // 25ms
    }
  }



  if (esp_timer_get_time() - last_pressure_update_micros >= 50 * 1000) { // 20Hz
    if (last_pressure_update_micros == 0) {
      last_pressure_update_micros = esp_timer_get_time();
    }

    getBMPData(&temperature, &pressure);
    altitude = altitude_from_pressure(pressure);

    BMPStartMeasurement();

    last_pressure_update_micros = esp_timer_get_time();

    // Serial.print(esp_timer_get_time());
    // Serial.print(",");
    // Serial.print(pressure);
    // Serial.print(",");
    // Serial.print(temperature);
    // Serial.print(",");
    // Serial.print(altitude);
    // Serial.print("\n"); // ends CSV line
  }


  if (esp_timer_get_time() - last_motion_update_micros >= 5000) { // 100Hz
    if (last_motion_update_micros == 0) {
      last_motion_update_micros = esp_timer_get_time();
    }

    bmi088.getAcceleration(&ax, &ay, &az);
    int64_t time_of_read = esp_timer_get_time();
    bmi088.getGyroscope(&gx, &gy, &gz);

    AttitudeEstimateQuat.x += gx * (time_of_read - last_motion_update_micros) / 1000000.0f;
    AttitudeEstimateQuat.y += gy * (time_of_read - last_motion_update_micros) / 1000000.0f;
    AttitudeEstimateQuat.z += gz * (time_of_read - last_motion_update_micros) / 1000000.0f;

    last_motion_update_micros = time_of_read;

    if (log_file != NULL) {
      String s = String(time_of_read) + 
        "," + String(pressure) +
        "," + String(temperature) +
        "," + String(altitude) +
        "," + String(ax) +
        "," + String(ay) +
        "," + String(az) +
        "," + String(gx) +
        "," + String(gy) +
        "," + String(gz) +
        "," + String(AttitudeEstimateQuat.x) +
        "," + String(AttitudeEstimateQuat.y) +
        "," + String(AttitudeEstimateQuat.z) +
        "\n";

      log_file->print(s);
      // Serial.print("Flushing file");
      log_file->flush();
      // Serial.print("Flush done");
      // num++;
      // if (num == 12) {
      //   num = 0;
      //   Serial.print("Closing file");
      //   log_file->close();
      //   Serial.println("Reopening file");
      //   // log_file_count++;
      //   // log_file_name = "/log-" + String(log_file_count) + ".txt";
      //   log_file_file = SD.open(log_file_name, FILE_WRITE, true);
      //   log_file = &log_file_file;
      // }
      // Serial.print(s);
    }
  }

  // int32_t temp;
  // uint32_t pres;


  // BMPStartMeasurement();
  // delay(100);

  // getBMPData(&temp, &pres);

  // Serial.println("BMP OUTPUT");
  // Serial.println(temp);
  // Serial.println(pres);




  // delay(15);


    

}
