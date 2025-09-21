#include "SensorsReader.h"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

struct bmp_user_out {
    int32_t temperature_centi;
    int32_t pressure_pa;
};

SensorsReader::SensorsReader(QObject* parent) : QObject(parent) {
    fd_tmp = open("/dev/tmp102", O_RDWR);
    fd_bmp = open("/dev/bmp280", O_RDWR);
    fd_ads = open("/dev/ads1115", O_RDWR);
    if(fd_tmp < 0) std::cerr << "Erreur ouverture TMP102\n";
    if(fd_bmp < 0) std::cerr << "Erreur ouverture BMP280\n";
    if(fd_ads < 0) std::cerr << "Erreur ouverture ADS1115\n";
}

SensorsReader::~SensorsReader() {
    if(fd_tmp>0) close(fd_tmp);
    if(fd_bmp>0) close(fd_bmp);
    if(fd_ads>0) close(fd_ads);
}

bool SensorsReader::readTMP(float &tempC) {
    short tmp_raw;
    if(read(fd_tmp, &tmp_raw, sizeof(tmp_raw)) <= 0) return false;
    tempC = (tmp_raw & 0x0FFF) * 0.0625;
    return true;
}

bool SensorsReader::readBMP(BmpData &data) {
    bmp_user_out bmp_data;
    if(read(fd_bmp, &bmp_data, sizeof(bmp_data)) <= 0) return false;
    data.temperature = bmp_data.temperature_centi / 100.0;
    data.pressure = bmp_data.pressure_pa / 100.0;
    return true;
}

bool SensorsReader::readADS(float &voltage) {
    int16_t ads_raw;
    if(read(fd_ads, &ads_raw, sizeof(ads_raw)) <= 0) return false;
    voltage = ads_raw / 32768.0 * 3.3;
    return true;
}
