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

SPIClass sensor_spi = SPIClass(SPI2_HOST);
BMI088 bmi088 = BMI088(&sensor_spi);

int64_t last_continuity_check_micros = 0;
int64_t last_motion_update_micros = 0;
int64_t last_print_micros = 0;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(42,OUTPUT);
  pinMode(43,OUTPUT);
  pinMode(44,OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(BMP_CS, OUTPUT);
  pinMode(GYRO_CS, OUTPUT);
  pinMode(ACC_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);

  digitalWrite(MAIN_CTRL, LOW); // DO NOT ACCIDENTALLY FIRE
  pinMode(MAIN_CTRL, OUTPUT);

  digitalWrite(DROGUE_CTRL, LOW); // DO NOT ACCIDENTALLY FIRE
  pinMode(DROGUE_CTRL, OUTPUT);

  pinMode(V5_ENABLE, OUTPUT);
  pinMode(V5_BOOST, OUTPUT);
  
  digitalWrite(V5_ENABLE, HIGH);
  
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

  bmp_init(&sensor_spi);  

  while (!bmi088.isConnection()) {
    Serial.println("Waiting for BMI to be alive!");
    Serial.println(bmi088.getAccID());
    Serial.println(bmi088.getGyroID());
  }

  bmi088.initialize();

  beep(250);
  delay(2500);
 
  Serial.println("Going to SD init");
  SPIClass sd_spi = SPIClass(SPI3_HOST);
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
      4 * 1000 * 1000,
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

    File f = SD.open("config.txt", FILE_READ, true);

    if (f) {
      Serial.println("File opened!");
      while (f.available()) {
        Serial.write(f.read());
      }
      f.close();
    } else {
      Serial.println("File open failed!");
    }

    beep(500);
    delay(100);

    File w = SD.open("log.txt", FILE_APPEND, true);

    if (w) {
      Serial.println("File opened!");
      w.println("START");
      w.close();
    } else {
      Serial.println("File open failed!");
    }
  }


  beep(1000);

  

}


bool output = false;
byte num = 0;
Quat AttitudeEstimateQuat;

Vec3 correction_Body, correction_World;
Vec3 Accel_Body, Accel_World;
Vec3 GyroVec;
const Vec3 VERTICAL = Vector(0.0f, 0.0f, 1.0f);  // vertical vector in the World frame




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

  if (esp_timer_get_time() - last_motion_update_micros >= 2500) { // 400Hz
    if (last_motion_update_micros == 0) {
      last_motion_update_micros = esp_timer_get_time();
    }

    float ax, ay, az, gx, gy, gz;
    bmi088.getAcceleration(&ax, &ay, &az);
    int64_t time_of_read = esp_timer_get_time();
    bmi088.getGyroscope(&gx, &gy, &gz);

    AttitudeEstimateQuat.x += gx * (time_of_read - last_motion_update_micros) / 1000000.0f;
    AttitudeEstimateQuat.y += gy * (time_of_read - last_motion_update_micros) / 1000000.0f;
    AttitudeEstimateQuat.z += gz * (time_of_read - last_motion_update_micros) / 1000000.0f;

    // // convert gyro to rad/s from deg/s
    // gx = gx * PI / 180.0;
    // gy = gy * PI / 180.0;
    // gz = gz * PI / 180.0;

    // // convert accel to g from mg
    // ax = ax / 1000.0;
    // ay = ay / 1000.0;
    // az = az / 1000.0;

    // GyroVec = Vector(gx, gy, gz);
    // Accel_Body = Vector(ax, ay, az);

    // Accel_World = Rotate(AttitudeEstimateQuat, Accel_Body); // rotate accel from body frame to world frame

		// correction_World = CrossProd(Accel_World, VERTICAL); // cross product to determine error

		// Vec3 correction_Body = Rotate(correction_World, AttitudeEstimateQuat); // rotate correction vector to body frame

		// GyroVec = Sum(GyroVec, correction_Body);  // add correction vector to gyro data

		// Quat incrementalRotation = Quaternion(GyroVec, time_of_read - last_motion_update_micros); // quaternion integration (rotation from gyro data)    

		// AttitudeEstimateQuat = Mul(incrementalRotation, AttitudeEstimateQuat);  // quaternion integration (rotation composting through multiplication)

    Serial.print(time_of_read);
    Serial.print(",");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print(",");
    
    Serial.print(AttitudeEstimateQuat.x);
    Serial.print(",");
    Serial.print(AttitudeEstimateQuat.y);
    Serial.print(",");
    Serial.print(AttitudeEstimateQuat.z);

    Serial.print("\n"); // ends CSV line

    if (time_of_read - last_print_micros >= 100 * 1000) {

      // Serial.print(",");
      // Serial.println(AttitudeEstimateQuat.w);


      last_print_micros = time_of_read;
    }

    last_motion_update_micros = time_of_read;    

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
