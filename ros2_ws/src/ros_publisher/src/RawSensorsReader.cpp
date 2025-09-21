#include "RawSensorsReader.h"
#include <fcntl.h>
#include <unistd.h>

RawSensorsReader::RawSensorsReader() {
    fd_tmp = open("/dev/tmp102", O_RDWR);
    fd_bmp = open("/dev/bmp280", O_RDWR);
    fd_ads = open("/dev/ads1115", O_RDWR);
}

RawSensorsReader::~RawSensorsReader() {
    if(fd_tmp >= 0) close(fd_tmp);
    if(fd_bmp >= 0) close(fd_bmp);
    if(fd_ads >= 0) close(fd_ads);
}

bool RawSensorsReader::readTMP(float &tempC) {
    if(fd_tmp < 0) return false;
    short raw;
    if(read(fd_tmp, &raw, sizeof(raw)) <= 0) return false;
    tempC = (raw & 0x0FFF) * 0.0625f;
    return true;
}

bool RawSensorsReader::readBMP(BmpData &data) {
    if(fd_bmp < 0) return false;
    struct {
        int32_t temperature_centi;
        int32_t pressure_pa;
    } bmp_raw;
    if(read(fd_bmp, &bmp_raw, sizeof(bmp_raw)) <= 0) return false;
    data.temperature = bmp_raw.temperature_centi / 100.0f;
    data.pressure = bmp_raw.pressure_pa / 100.0f;
    return true;
}

bool RawSensorsReader::readADS(float &voltage) {
    if(fd_ads < 0) return false;
    int16_t raw;
    if(read(fd_ads, &raw, sizeof(raw)) <= 0) return false;
    voltage = raw / 32768.0f * 3.3f;
    return true;
}
