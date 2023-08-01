#include "SPI.h"

void BMPStartMeasurement();
void bmp_init(SPIClass* spi_to_use);
void getBMPData(int32_t *temperature, uint32_t *pressure);