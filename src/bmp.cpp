#include "bmp.h"
#include "pinout.h"
#include "Arduino.h"
#include "util.h"



#define BMP_MODE 0b11
#define BMP_OSRS_P 0b101
#define BMP_OSRS_T 0b010

// BMP CALIBRATION PARAMETERS
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

// t_fine carries fine temperature as global value
int32_t t_fine;
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
} 
// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t bmp280_compensate_P_int32(int32_t adc_P)
{
  int32_t var1, var2;
  uint32_t p;
  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
  var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000)
  {
    p = (p << 1) / ((uint32_t)var1);
  }
  else
  {
    p = (p / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
  return p;
}

SPIClass* bmp_spi;

void selectBMP() {
    digitalWrite(BMP_CS, LOW);
}

void deselectBMP() {
    digitalWrite(BMP_CS, HIGH);
}


void bmp_init(SPIClass* spi_to_use) {
    bmp_spi = spi_to_use;    

    selectBMP();

    // Send the register address of temperature calibration parameter T1 (0x88) with read bit (0x80) set
    bmp_spi->transfer(0x88 | 0x80);

    uint8_t calib_data[24];
    for (uint8_t i = 0; i < 24; i++)
    {
      calib_data[i] = bmp_spi->transfer(0);
    }

    deselectBMP();

    // Store calibration parameters as unsigned integers in the struct
    dig_T1 = *(uint16_t *)calib_data;
    dig_T2 = *(int16_t *)(calib_data + 2);
    dig_T3 = *(int16_t *)(calib_data + 4);
    dig_P1 = *(uint16_t *)(calib_data + 6);
    dig_P2 = *(int16_t *)(calib_data + 8);
    dig_P3 = *(int16_t *)(calib_data + 10);
    dig_P4 = *(int16_t *)(calib_data + 12);
    dig_P5 = *(int16_t *)(calib_data + 14);
    dig_P6 = *(int16_t *)(calib_data + 16);
    dig_P7 = *(int16_t *)(calib_data + 18);
    dig_P8 = *(int16_t *)(calib_data + 20);
    dig_P9 = *(int16_t *)(calib_data + 22);

    // Serial.println("Temperature calibration parameters: ");
    // Serial.println(dig_T1);
    // Serial.println(dig_T2);
    // Serial.println(dig_T3);
    // Serial.println("Pressure calibration parameters: ");
    // Serial.println(dig_P1);
    // Serial.println(dig_P2);
    // Serial.println(dig_P3);
    // Serial.println(dig_P4);
    // Serial.println(dig_P5);
    // Serial.println(dig_P6);
    // Serial.println(dig_P7);
    // Serial.println(dig_P8);
    // Serial.println(dig_P9);

    // Write the configuration register (0xF5)
    selectBMP();

    // Send the register address of configuration register (0xF5)
    bmp_spi->transfer(0xF5 & ~0x80);

    // Send the data uint8_t for configuration register
    // Standby time: 62.5 ms (bits [7:5] = 001)
    // Filter: x4 (bits [4:2] = 010)
    // Do not enable 3 wire SPI: 0 (bit [1:0] = 00)
    bmp_spi->transfer(0b00101000);

    deselectBMP();    
    // Write the control register (0xF4)
    selectBMP();

    // Send the register address of control register (0xF4)
    bmp_spi->transfer(0xF4 & ~0x80);

    // Send the data uint8_t for control register
    // Oversampling pressure: osrs_p (bits [4:2])
    // Oversampling temperature: osrs_t (bits [7:5])
    // Mode: forced mode (bits [1:0])
    bmp_spi->transfer((BMP_OSRS_T << 5) | (BMP_OSRS_P << 2) | 0b01);

    deselectBMP();

    // beep(100);
    // delay(100);
    return;
}

void BMPStartMeasurement() {
  selectBMP();

  // Send the register address of control register (0xF4)
  bmp_spi->transfer(0xF4 & ~0x80);

  // Send the data uint8_t for control register
  // Oversampling pressure: osrs_p (bits [4:2])
  // Oversampling temperature: osrs_t (bits [7:5])
  // Mode: mode (bits [1:0])
  bmp_spi->transfer((BMP_OSRS_P << 5) | (BMP_OSRS_T << 2) | 0b01);

  // Pull CS high to end communication
  deselectBMP();
}

void getBMPData(float *temperature, float *pressure) {
    selectBMP();

    // Send the register address of pressure MSB (0xF7) with read bit (0x80) set
    bmp_spi->transfer(0xF7 | 0x80);

    // Receive 6 bytes of data for pressure and temperature
    uint32_t press_msb = (uint32_t)bmp_spi->transfer(0);
    uint32_t press_lsb = (uint32_t)bmp_spi->transfer(0);
    uint32_t press_xlsb = (uint32_t)bmp_spi->transfer(0);
    uint32_t temp_msb = (uint32_t)bmp_spi->transfer(0);
    uint32_t temp_lsb = (uint32_t)bmp_spi->transfer(0);
    uint32_t temp_xlsb = (uint32_t)bmp_spi->transfer(0);

    // Pull CS high to end communication
    deselectBMP();

    // Serial.println("Pressure MSB: ");
    // Serial.println(press_msb);
    // Serial.println("Pressure LSB: ");
    // Serial.println(press_lsb);
    // Serial.println("Pressure XLSB: ");
    // Serial.println(press_xlsb);
    // Serial.println("Temperature MSB: ");
    // Serial.println(temp_msb);
    // Serial.println("Temperature LSB: ");
    // Serial.println(temp_lsb);
    // Serial.println("Temperature XLSB: ");
    // Serial.println(temp_xlsb);

    *temperature = (float)(bmp280_compensate_T_int32((temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4))) / 100.0f;
    *pressure = (float)(bmp280_compensate_P_int32((press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4)));

    // Serial.println("Temperature: ");
    // Serial.println(bmp280_compensate_P_int32((press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4)));
    // Serial.println("Pressure: ");
    // Serial.println(bmp280_compensate_T_int32((temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4)));

    return;
}

int32_t sea_level_pressure = 101325; // Pascals

// Takes in pressure (raw, Pascals) and returns altitude
float altitude_from_pressure(uint32_t pressure)
{
  float p = (float)pressure;

  // Calculate altitude using the barometric formula
  float h = 44330 * (1 - powf(p / sea_level_pressure, 1 / 5.25579));

  return h;
}