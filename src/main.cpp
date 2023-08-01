#include <Arduino.h>
#include "driver/spi_master.h"
#include "SPI.h"
#include "pinout.h"
#include "bmp.h"
#include "bmi088.h"
#include "util.h"
#include "SD.h"

SPIClass sensor_spi = SPIClass(SPI2_HOST);
BMI088 bmi088 = BMI088(&sensor_spi);


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

  pinMode(MAIN_DETECT, INPUT);
  pinMode(DROGUE_DETECT, INPUT);
  adcAttachPin(MAIN_DETECT);
  adcAttachPin(DROGUE_DETECT);

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
void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(42, output);
  digitalWrite(43, !output);
  digitalWrite(44, output);

  // digitalWrite(MAIN_CTRL, output);
  // digitalWrite(DROGUE_CTRL, !output);
  
  delay(500);
  output = !output;

  // Serial.println("Hello World");

  if (num == 8) {
    uint32_t main_detect = analogRead(MAIN_DETECT) * 3300 / 4096;
    uint32_t drogue_detect = analogRead(DROGUE_DETECT) * 3300 / 4096;

    if (main_detect < 500) {
      beep(25);
      delay(25);
    }

    if (drogue_detect < 500) {
      beep(25);
      delay(25);
      beep(25);
      delay(25);
    }

    Serial.println("DETECTS");
    Serial.println(main_detect);
    Serial.println(drogue_detect);

    num = 0;
  } else {
    num++;
  }

  // int32_t temp;
  // uint32_t pres;


  // BMPStartMeasurement();
  // delay(100);

  // getBMPData(&temp, &pres);

  // Serial.println("BMP OUTPUT");
  // Serial.println(temp);
  // Serial.println(pres);

  // float ax, ay, az;

  // bmi088.getAcceleration(&ax, &ay, &az);

  // float gx, gy, gz;

  // bmi088.getGyroscope(&gx, &gy, &gz);

  // Serial.print(ax);
  // Serial.print(",");
  // Serial.print(ay);
  // Serial.print(",");
  // Serial.print(az);
  // Serial.print(",");
  // Serial.print(gx);
  // Serial.print(",");
  // Serial.print(gy);
  // Serial.print(",");
  // Serial.println(gz);

  // delay(15);

}
