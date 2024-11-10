#include "SPI.h"

void BMPStartMeasurement();
void bmp_init(SPIClass* spi_to_use);
void getBMPData(float *temperature, float *pressure);
float altitude_from_pressure(float pressure);