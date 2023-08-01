/*
    we smurfing

    https://datasheet.lcsc.com/lcsc/1811031202_Bosch-Sensortec-BMI088_C194919.pdf
*/

#include "bmi088.h"

BMI088::BMI088(SPIClass* spi_to_use) {
    spi = spi_to_use;
}

void BMI088::initialize(void) {
    setAccScaleRange(RANGE_24G);
    setAccOutputDataRate(ODR_100);
    setAccPowerMode(ACC_ACTIVE);

    setGyroScaleRange(RANGE_2000);
    setGyroOutputDataRate(ODR_2000_BW_532);
    setGyroPowerMode(GYRO_NORMAL);
}

bool BMI088::isConnection(void) {
    getAccID(); // dummy read

    return ((getAccID() == 0x1E) && (getGyroID() == 0x0F));
}

void BMI088::resetAcc(void) {
    write8(ACC, BMI088_ACC_SOFT_RESET, 0xB6);
}

void BMI088::resetGyro(void) {
    write8(GYRO, BMI088_GYRO_SOFT_RESET, 0xB6);
}

uint8_t BMI088::getAccID(void) {
    return read8(ACC, BMI088_GYRO_CHIP_ID);
}

uint8_t BMI088::getGyroID(void) {
    return read8(GYRO, BMI088_GYRO_CHIP_ID);
}

void BMI088::setAccPowerMode(acc_power_type_t mode) {
    if (mode == ACC_ACTIVE) {
        write8(ACC, BMI088_ACC_PWR_CTRl, 0x04);
        write8(ACC, BMI088_ACC_PWR_CONF, 0x00);
    } else if (mode == ACC_SUSPEND) {
        write8(ACC, BMI088_ACC_PWR_CONF, 0x03);
        write8(ACC, BMI088_ACC_PWR_CTRl, 0x00);
    }
}

void BMI088::setGyroPowerMode(gyro_power_type_t mode) {
    if (mode == GYRO_NORMAL) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_NORMAL);
    } else if (mode == GYRO_SUSPEND) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_SUSPEND);
    } else if (mode == GYRO_DEEP_SUSPEND) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_DEEP_SUSPEND);
    }
}

void BMI088::setAccScaleRange(acc_scale_type_t range) {
    if (range == RANGE_3G) {
        accRange = 3000;
    } else if (range == RANGE_6G) {
        accRange = 6000;
    } else if (range == RANGE_12G) {
        accRange = 12000;
    } else if (range == RANGE_24G) {
        accRange = 24000;
    }

    write8(ACC, BMI088_ACC_RANGE, (uint8_t)range);
}

void BMI088::setAccOutputDataRate(acc_odr_type_t odr) {
    uint8_t data = 0;

    data = read8(ACC, BMI088_ACC_CONF);
    data = data & 0xf0;
    data = data | (uint8_t)odr;

    write8(ACC, BMI088_ACC_CONF, data);
}

void BMI088::setGyroScaleRange(gyro_scale_type_t range) {
    if (range == RANGE_2000) {
        gyroRange = 2000;
    } else if (range == RANGE_1000) {
        gyroRange = 1000;
    } else if (range == RANGE_500) {
        gyroRange = 500;
    } else if (range == RANGE_250) {
        gyroRange = 250;
    } else if (range == RANGE_125) {
        gyroRange = 125;
    }

    write8(GYRO, BMI088_GYRO_RANGE, (uint8_t)range);
}

void BMI088::setGyroOutputDataRate(gyro_odr_type_t odr) {
    write8(GYRO, BMI088_GYRO_BAND_WIDTH, (uint8_t)odr);
}

void BMI088::getAcceleration(float* x, float* y, float* z) {
    uint8_t buf[6] = {0};
    uint16_t ax = 0, ay = 0, az = 0;
    float value = 0;

    read(ACC, BMI088_ACC_X_LSB, buf, 6);

    ax = buf[0] | (buf[1] << 8);
    ay = buf[2] | (buf[3] << 8);
    az = buf[4] | (buf[5] << 8);

    value = (int16_t)ax;
    *x = accRange * value / 32768;

    value = (int16_t)ay;
    *y = accRange * value / 32768;

    value = (int16_t)az;
    *z = accRange * value / 32768;
}

float BMI088::getAccelerationX(void) {
    uint16_t ax = 0;
    float value = 0;

    ax = read16(ACC, BMI088_ACC_X_LSB);

    value = (int16_t)ax;
    value = accRange * value / 32768;

    return value;
}

float BMI088::getAccelerationY(void) {
    uint16_t ay = 0;
    float value = 0;

    ay = read16(ACC, BMI088_ACC_Y_LSB);

    value = (int16_t)ay;
    value = accRange * value / 32768;

    return value;
}

float BMI088::getAccelerationZ(void) {
    uint16_t az = 0;
    float value = 0;

    az = read16(ACC, BMI088_ACC_Z_LSB);

    value = (int16_t)az;
    value = accRange * value / 32768;

    return value;
}

void BMI088::getGyroscope(float* x, float* y, float* z) {
    uint8_t buf[6] = {0};
    uint16_t gx = 0, gy = 0, gz = 0;
    float value = 0;

    read(GYRO, BMI088_GYRO_RATE_X_LSB, buf, 6);

    gx = buf[0] | (buf[1] << 8);
    gy = buf[2] | (buf[3] << 8);
    gz = buf[4] | (buf[5] << 8);

    value = (int16_t)gx;
    *x = gyroRange * value / 32768;

    value = (int16_t)gy;
    *y = gyroRange * value / 32768;

    value = (int16_t)gz;
    *z = gyroRange * value / 32768;
}

float BMI088::getGyroscopeX(void) {
    uint16_t gx = 0;
    float value = 0;

    gx = read16(GYRO, BMI088_GYRO_RATE_X_LSB);

    value = (int16_t)gx;
    value = gyroRange * value / 32768;

    return value;
}

float BMI088::getGyroscopeY(void) {
    uint16_t gy = 0;
    float value = 0;

    gy = read16(GYRO, BMI088_GYRO_RATE_Y_LSB);

    value = (int16_t)gy;
    value = gyroRange * value / 32768;

    return value;
}

float BMI088::getGyroscopeZ(void) {
    uint16_t gz = 0;
    float value = 0;

    gz = read16(GYRO, BMI088_GYRO_RATE_Z_LSB);

    value = (int16_t)gz;
    value = gyroRange * value / 32768;

    return value;
}

int16_t BMI088::getTemperature(void) {
    uint16_t data = 0;

    data = read16Be(ACC, BMI088_ACC_TEMP_MSB);
    data = data >> 5;

    if (data > 1023) {
        data = data - 2048;
    }

    return (int16_t)(data / 8 + 23);
}

void BMI088::selectDevice(bmi_device_type_t device) {
    if (device == ACC) {
        digitalWrite(ACC_CS, LOW);
        digitalWrite(GYRO_CS, HIGH);
    } else {
        digitalWrite(ACC_CS, HIGH);
        digitalWrite(GYRO_CS, LOW);
    }
}

void BMI088::deselectDevice(bmi_device_type_t device) {
    digitalWrite(device, HIGH);
}

void BMI088::write8(bmi_device_type_t dev, uint8_t reg, uint8_t val) { 
    selectDevice(dev);
    reg = reg & 0x7F; // write, bit 7 low
    spi->write(reg);
    spi->write(val);
    deselectDevice(dev);
}

uint8_t BMI088::read8(bmi_device_type_t dev, uint8_t reg) {
    selectDevice(dev);
    spi->write(reg | 0x80); // read byte
    uint8_t dummy;

    if (dev == ACC) {
        dummy = spi->transfer(0); // dummy byte
    }

    uint8_t value = spi->transfer(0);
    deselectDevice(dev);

    Serial.println("READ8: Reg, Dummy, Value");
    Serial.println(reg);
    Serial.println(dummy);
    Serial.println(value);

    return value;
}

uint16_t BMI088::read16(bmi_device_type_t dev, uint8_t reg) {
    selectDevice(dev);
    spi->write(reg | 0x80); // read byte
    if (dev == ACC) {
        spi->transfer(0); // dummy byte
    }
    uint8_t lsb = spi->transfer(0);
    uint8_t msb = spi->transfer(0);

    deselectDevice(dev);

    return (lsb | (msb << 8));
}

uint16_t BMI088::read16Be(bmi_device_type_t dev, uint8_t reg) {
    selectDevice(dev);
    spi->write(reg | 0x80); // read byte
    if (dev == ACC) {
        spi->transfer(0); // dummy byte
    }
    uint8_t msb = spi->transfer(0);
    uint8_t lsb = spi->transfer(0);

    deselectDevice(dev);

    return (lsb | (msb << 8));    
}

uint32_t BMI088::read24(bmi_device_type_t dev, uint8_t reg) {
    selectDevice(dev);
    spi->write(reg | 0x80); // read byte
    if (dev == ACC) {
        spi->transfer(0); // dummy byte
    }
    uint8_t lsb = spi->transfer(0);
    uint8_t mid = spi->transfer(0);
    uint8_t msb = spi->transfer(0);

    deselectDevice(dev);

    return (lsb | (mid << 8) | (msb << 16));
}

void BMI088::read(bmi_device_type_t dev, uint8_t reg, uint8_t* buf, uint16_t len) {
    selectDevice(dev);
    spi->write(reg | 0x80); // read byte
    if (dev == ACC) {
        spi->transfer(0); // dummy byte
    }
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = spi->transfer(0);
    }

    deselectDevice(dev);
}