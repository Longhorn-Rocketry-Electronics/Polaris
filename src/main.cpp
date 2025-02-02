#include <Arduino.h>
#include "driver/spi_master.h"
#include "SPI.h"
#include "pinout.h"
#include "bmp.h"
#include "bmi088.h"
#include "util.h"
#include "SD.h"
#include <driver/adc.h>
// #include "MadgwickAHRS.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "circularbuffer.h"
#include "servo_controller.h"
#include "Fusion.h"

SPIClass sensor_spi = SPIClass(SPI2_HOST);
SPIClass sd_spi = SPIClass(SPI3_HOST);

BMI088 bmi088 = BMI088(&sensor_spi);

int64_t last_continuity_check_micros = 0;
int64_t last_motion_update_micros = 0;
int64_t last_pressure_update_micros = 0;
int64_t last_print_micros = 0;
int64_t launch_time_micros = 0;

File* log_file;
File log_file_file;
String log_file_name;
int log_file_count = 0;
float launch_detection_threshold = 30.48; // 100 feet by default
float launch_detection_min_g = 2; // 2g by default
int main_timer_ms = 0;
int drogue_timer_ms = 0;
int main_fire_duration_ms = 5000;
int drogue_fire_duration_ms = 5000;
float main_altitude_meters = 30.48;
float drogue_altitude_meters = 0;
bool main_on_apogee = false;
bool drogue_on_apogee = true;

FusionAhrs ori;

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
        /* Config file format:
          launch_detection_threshold=30.48
          launch_detection_min_g=2.0
          main_timer_ms=0
          drogue_timer_ms=0
          main_fire_duration_ms=1000
          drogue_fire_duration_ms=1000
          main_altitude_meters=30.48
          drogue_altitude_meters=0
          main_on_apogee=false
          drogue_on_apogee=true
        */
        String line = f.readStringUntil('\n');
        line.replace(" ", "");

        Serial.println(line);
        try {
          if (line.startsWith("launch_detection_threshold")) {
            launch_detection_threshold = line.substring(27).toFloat();

            if (launch_detection_threshold < 30.48) {
              launch_detection_threshold = 30.48;
            } else if (launch_detection_threshold > 152.4) { // 500 feet
              launch_detection_threshold = 152.4;
            }
          }
          if (line.startsWith("launch_detection_min_g")) {
            launch_detection_min_g = line.substring(23).toFloat();
            launch_detection_min_g = abs(launch_detection_min_g);
          }
          if (line.startsWith("main_timer_ms")) {
            main_timer_ms = line.substring(13).toInt();
          }
          if (line.startsWith("drogue_timer_ms")) {
            drogue_timer_ms = line.substring(15).toInt();
          }
          if (line.startsWith("main_fire_duration_ms")) {
            main_fire_duration_ms = line.substring(22).toInt();
          }
          if (line.startsWith("drogue_fire_duration_ms")) {
            drogue_fire_duration_ms = line.substring(24).toInt();
          }
          if (line.startsWith("main_altitude_meters")) {
            main_altitude_meters = line.substring(21).toFloat();
          }
          if (line.startsWith("drogue_altitude_meters")) {
            drogue_altitude_meters = line.substring(23).toFloat();
          }
          if (line.startsWith("main_on_apogee")) {
            main_on_apogee = line.substring(15)[0] == 't' || line.substring(15)[0] == 'T' || line.substring(15)[0] == '1';
          }
          if (line.startsWith("drogue_on_apogee")) {
            drogue_on_apogee = line.substring(17)[0] == 't' || line.substring(17)[0] == 'T' || line.substring(17)[0] == '1';
          }
        } catch (const std::exception& e) {
          Serial.println("Error parsing config file!");
          Serial.println(e.what());
          beep(50);
          delay(100);
          beep(50);
          delay(100);
          beep(50);
          delay(100);
        }

        // Serial.println("Drogue on apogee: " + String(drogue_on_apogee));
      }
      f.close();
      beep(500);
      delay(100);
    } else {
      Serial.println("File open failed!");
      beep(50);
      delay(100);
    }

    

    File w = SD.open("/log.csv", FILE_APPEND, true);

    if (w) {
      Serial.println("File opened!");
      w.println("START");
      w.close();
    } else {
      Serial.println(w.getWriteError());
      Serial.println("File open failed!");
    }

    // count the number of existing log-*.txt files to determine the next log file name
    while (SD.exists("/log-" + String(log_file_count) + ".csv")) {
      log_file_count++;
    }

    log_file_file = SD.open("/log-" + String(log_file_count) + ".csv", FILE_WRITE, true);
    log_file = &log_file_file;
    log_file_name = "/log-" + String(log_file_count) + ".csv";

    log_file_file.print("Time (micros), Pressure (Pa), Temperature (C), Altitude (m), Yaw (deg), Pitch (deg), Roll (deg), Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (deg/s), Gy (deg/s), Gz (deg/s), Drogue Detect (mV), Main Detect (mV), Stage, Main Fired (micros), Drogue Fired (micros)\n");
  } else {
    Serial.println("SD init failed!");
    log_file = NULL;
  }

  BMPStartMeasurement();

  Serial.print("Launch Detection Threshold: ");
  Serial.println(launch_detection_threshold);

  servoSetup();

  FusionAhrsInitialise(&ori);

  beep(1000);

}



bool output = false;
byte num = 0;
float ax, ay, az, gx, gy, gz, pressure, temperature;
// relative to sea level before launch, relative to launch ground altitude after launch (meters)
float altitude = 0;
float launch_ground_altitude = 0;
CircularBuffer<float, 100> recent_altitudes;
uint16_t main_detect, drogue_detect;

CircularBuffer<float, 1000> ax_buf, ay_buf, az_buf, gx_buf, gy_buf, gz_buf, yaw_buf, pitch_buf, roll_buf, pressure_buf, temperature_buf, altitude_buf;
CircularBuffer<uint64_t, 1000> time_buf;
CircularBuffer<uint16_t, 1000> main_detect_buf, drogue_detect_buf;
CircularBuffer<FlightStage, 1000> stage_buf;

FlightStage stage = NOT_LAUNCHED;

int64_t main_fired_micros = 0;
int64_t drogue_fired_micros = 0;

bool logLineAvailable() {
  return time_buf.length() > 0;
}

String popOldestLogLine() {
  if (!logLineAvailable()) {
    return "";
  }

  float alt_val = altitude_buf.pop();

  String toRet = String(time_buf.pop()) + 
    "," + String(pressure_buf.pop()) +
    "," + String(temperature_buf.pop()) +
    "," + String(stage == NOT_LAUNCHED ? 0 : alt_val) + // if we are not launched, altitude is 0
    "," + String(yaw_buf.pop()) +
    "," + String(pitch_buf.pop()) +
    "," + String(roll_buf.pop()) +
    "," + String(ax_buf.pop()) +
    "," + String(ay_buf.pop()) +
    "," + String(az_buf.pop()) +
    "," + String(gx_buf.pop()) +
    "," + String(gy_buf.pop()) +
    "," + String(gz_buf.pop()) +
    "," + String(drogue_detect_buf.pop()) +
    "," + String(main_detect_buf.pop()) +
    "," + String(stage_buf.pop()) +
    "," + String(main_fired_micros) +
    "," + String(drogue_fired_micros) +
    "\n";

  // Time (micros), Pressure (Pa), Temperature (C), Altitude (m), Yaw (deg), Pitch (deg), Roll (deg), Ax (m/s^2), Ay (m/s^2), Az (m/s^2), Gx (deg/s), Gy (deg/s), Gz (deg/s), Drogue Detect (mV), Main Detect (mV), Stage, Main Fired (micros), Drogue Fired (micros_

  // if (Serial.availableForWrite()) {
  //   Serial.print("POPPED, LOGGING: " + toRet);
  // }
  
  return toRet;
}




void loop() {
  // put your main code here, to run repeatedly:
  updateAsyncBeep();

  servoLoop(yaw_buf.get(0), pitch_buf.get(0), roll_buf.get(0));


  // if (stage == POST_APOGEE) {
  //   Serial.println("STARTING LOOP IN POST APOGEE STATE");
  // }
  // Serial.println("Hello World");

  if (esp_timer_get_time() - last_continuity_check_micros >= 5000 * 1000) { // Check continuity every 5 seconds

    main_detect = (uint16_t)analogReadMilliVolts(A5);
    drogue_detect = (uint16_t)analogReadMilliVolts(A6);

    if (stage == NOT_LAUNCHED) {
      if (main_detect < 500) {
        queue_beep(esp_timer_get_time(), 25 * 1000); // 25ms
      }

      if (drogue_detect < 500) {
        queue_beep(esp_timer_get_time() + 100 * 1000, 25 * 1000); // 25ms
        queue_beep(esp_timer_get_time() + 200 * 1000, 25 * 1000); // 25ms
      }
    }

    if (main_detect >= 500 && drogue_detect >= 500) {
      switch (stage) {
        case NOT_LAUNCHED:
          queue_beep(esp_timer_get_time(), 250 * 1000); // 250ms
          break;
        case LAUNCHED:
        case POST_APOGEE:
          break;
        case LANDED:
          queue_beep(esp_timer_get_time(), 1000 * 1000); // 1000ms
      }
    }

    last_continuity_check_micros = esp_timer_get_time();
  }



  if (esp_timer_get_time() - last_pressure_update_micros >= 50 * 1000) { // 20Hz
    digitalWrite(42, output);
    digitalWrite(43, !output);
    digitalWrite(44, output);

    if (last_pressure_update_micros == 0) {
      last_pressure_update_micros = esp_timer_get_time();
    }

    getBMPData(&temperature, &pressure);
    altitude = altitude_from_pressure(pressure) - launch_ground_altitude;

    if (altitude > 0.01 || altitude < -0.01) { // filter out bad readings of 0

      if (stage == NOT_LAUNCHED && recent_altitudes.length() > 10) {
        float prev_alt = recent_altitudes.get(0);
        if (altitude < prev_alt - 0.25f) { // protects against bursts of wind increasing pressure, thus decreasing altitude
          altitude = prev_alt - 0.25f;
        }
        if (altitude > prev_alt + 80.0f) { // most likely an incorrect measurement (80 meters higher in 0.1seconds)
          altitude = prev_alt + 10.0f; // unlikely you will be going over 100 m/s in < 0.4s, so this should be good
        }
      }

      recent_altitudes.push(altitude);
    }

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


  if (esp_timer_get_time() - last_motion_update_micros >= 10 * 1000 - 800) { // 100Hz
    if (last_motion_update_micros == 0) {
      last_motion_update_micros = esp_timer_get_time();
    }

    bmi088.getAcceleration(&ax, &ay, &az);
    int64_t time_of_read = esp_timer_get_time();
    bmi088.getGyroscope(&gx, &gy, &gz);

    // ori.updateIMU(gx, gy, gz, ax, ay, az, (time_of_read - last_motion_update_micros) / 1000000.0f);

    const FusionVector gyroscope = {gx, gy, gz}; 
    const FusionVector accelerometer = {ax, ay, az};

    FusionAhrsUpdateNoMagnetometer(&ori, gyroscope, accelerometer, (time_of_read - last_motion_update_micros) / 1000000.0f);

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ori));

    last_motion_update_micros = time_of_read;

    String s = String(time_of_read) + 
      "," + String(pressure) +
      "," + String(temperature) +
      "," + String(altitude) +
      "," + String(yaw_buf.get(0)) +
      "," + String(pitch_buf.get(0)) +
      "," + String(roll_buf.get(0)) +
      "," + String(ax) +
      "," + String(ay) +
      "," + String(az) +
      "," + String(gx) +
      "," + String(gy) +
      "," + String(gz) +
      "," + String(drogue_detect) +
      "," + String(main_detect) +
      "," + String(stage) +
      "," + String(main_fired_micros) +
      "," + String(drogue_fired_micros) +
      "\n";

    time_buf.push(time_of_read);
    pressure_buf.push(pressure);
    temperature_buf.push(temperature);
    altitude_buf.push(altitude);
    yaw_buf.push(euler.angle.yaw);
    pitch_buf.push(euler.angle.pitch);
    roll_buf.push(euler.angle.roll);
    ax_buf.push(ax);
    ay_buf.push(ay);
    az_buf.push(az);
    gx_buf.push(gx);
    gy_buf.push(gy);
    gz_buf.push(gz);
    drogue_detect_buf.push(drogue_detect);
    main_detect_buf.push(main_detect);
    stage_buf.push(stage);


    static int lognum = 100; // log at much reduced frequency pre-launch
    if (log_file != NULL) {
      if (stage == NOT_LAUNCHED || stage == LANDED) {
        if (lognum >= 100) { // 1Hz
          output = !output;
          log_file->print(popOldestLogLine());
          log_file->flush();          
          lognum = 0;
          // Serial.println("POPPED, LOGGING w/ " + String(esp_get_free_heap_size()) + " bytes free");
          // Serial.flush();
        }
        lognum++;
      } else if (stage == LAUNCHED || stage == POST_APOGEE)
      {        // 100Hz
        int counter = 3;
        while (counter > 0 && logLineAvailable()) {
          log_file->print(popOldestLogLine());
          counter--;
          last_motion_update_micros += 100; // pretend we wrote 100us worth of data
        }
        log_file->flush();
      }

    }

    if (Serial.availableForWrite()) {
      if (num % 10 == 0 || stage == LAUNCHED || stage == POST_APOGEE || true)
      {
        Serial.println(s);
        Serial.flush();
      }
      if (num == 100) {
        num = 0;
        if (log_file == NULL) {
          // Serial.println("LOG FILE IS NULL");
        }
      }
      num++;
    }

  }    

  if (recent_altitudes.full()) {
    float newest_alt, recent_alt, mid_alt, oldest_alt;
    newest_alt = recent_altitudes.get(0);
    recent_alt = recent_altitudes.get(10);
    mid_alt = recent_altitudes.get(50);
    oldest_alt = recent_altitudes.get(99);
    switch(stage)
    {
      case NOT_LAUNCHED:
      {
        if (newest_alt > recent_alt && // This avoids a one or two element spike
            (recent_alt > oldest_alt + launch_detection_threshold ) &&
            
            ((abs(ax) >= launch_detection_min_g*1000 || abs(ay) >= launch_detection_min_g*1000 || abs(az) >= launch_detection_min_g*1000) || (ax == 0 && ay == 0 && az == 0))
            
             )
        {
          if (Serial.availableForWrite()) {
            Serial.println("NEWEST, RECENT, MID, OLDEST");
            Serial.println(newest_alt);
            Serial.println(recent_alt);
            Serial.println(mid_alt);
            Serial.println(oldest_alt);
          }
          stage = LAUNCHED;

          launch_ground_altitude = oldest_alt;       

          // find the most recent altitude that is within 3 meters of the launch ground altitude to use as the launch time
          int launch_time_index = altitude_buf.length() - 1;

          while (launch_time_index > 0 && abs(altitude_buf.get(launch_time_index) - launch_ground_altitude) > 3) {
            launch_time_index--;
          }

          launch_time_micros = time_buf.get(launch_time_index);

          // shift all the previous altitudes down by the launch ground altitude, since all future measurements will be relative to the launch ground altitude
          for (int i = 0; i < altitude_buf.length(); i++) {
            altitude_buf.setRaw(i, altitude_buf.getRaw(i) - launch_ground_altitude);
          }
          for (int i = 0; i < recent_altitudes.length(); i++) {
            recent_altitudes.setRaw(i, recent_altitudes.getRaw(i) - launch_ground_altitude);
          }

          altitude = altitude - launch_ground_altitude;

        }        
        break;
      }
      case LAUNCHED:
      {
        if (newest_alt < mid_alt && mid_alt < oldest_alt)
        {
          stage = POST_APOGEE;
          if (Serial.availableForWrite()) {
            Serial.println("POST_APOGEE -----------------------");
          }
        }
        break;
      }
      case POST_APOGEE:
      {
        float maxDifference = 0;
        for (int i =0; i < recent_altitudes.length(); i++)
        {
          float difference = abs(recent_altitudes.get(i) - recent_altitudes.get(i+1));
          if (difference > maxDifference)
          {
            maxDifference = difference;
          }
        }
        if (maxDifference < 3)
        {
          stage = LANDED;
          if (Serial.availableForWrite()) {
            Serial.println("LANDED ----------------------- " + String(maxDifference));
          }
          while (logLineAvailable()) {
            log_file->print(popOldestLogLine());
            log_file->flush();
          }
        }
      }
      case LANDED:
        break;
    }

  }



  if (stage >= LAUNCHED && stage < LANDED) {
    int64_t current_time = esp_timer_get_time();

    float latest_twenty_altitudes[20];

    for (int i = 0; i < 20; i++) {
      latest_twenty_altitudes[i] = recent_altitudes.get(i);
    }

    float median_altitude = median(latest_twenty_altitudes, 20);

    if (main_fired_micros == 0) {
      if (current_time - launch_time_micros >= main_timer_ms * 1000 && main_timer_ms > 0) {
        digitalWrite(MAIN_CTRL, HIGH);
        main_fired_micros = current_time;
      }
      if (main_on_apogee && stage == POST_APOGEE) {
        digitalWrite(MAIN_CTRL, HIGH);
        main_fired_micros = current_time;
      }
      if (median_altitude < main_altitude_meters && main_altitude_meters > 0 && stage == POST_APOGEE) {
        digitalWrite(MAIN_CTRL, HIGH);
        main_fired_micros = current_time;
      }
    }

    if (drogue_fired_micros == 0) {
      if (current_time - launch_time_micros >= drogue_timer_ms * 1000 && drogue_timer_ms > 0) {
        digitalWrite(DROGUE_CTRL, HIGH);
        drogue_fired_micros = current_time;
      }
      if (drogue_on_apogee && stage == POST_APOGEE) {
        digitalWrite(DROGUE_CTRL, HIGH);
        drogue_fired_micros = current_time;
      } else {
        // Serial.println("Not firing drogue because: " + String(drogue_on_apogee) + ", " + String(stage));
      }
      if (median_altitude < drogue_altitude_meters && drogue_altitude_meters > 0 && stage == POST_APOGEE) {
        digitalWrite(DROGUE_CTRL, HIGH);
        drogue_fired_micros = current_time;
      }
    }
  }

  if (main_fired_micros > 0 && esp_timer_get_time() - main_fired_micros >= 1000 * main_fire_duration_ms) {
    digitalWrite(MAIN_CTRL, LOW);
  }

  if (drogue_fired_micros > 0 && esp_timer_get_time() - drogue_fired_micros >= 1000 * drogue_fire_duration_ms) {
    digitalWrite(DROGUE_CTRL, LOW);
  }

  // if (stage == POST_APOGEE) {
  //   Serial.println("ENDING LOOP IN POST APOGEE STATE");
  // }
}
