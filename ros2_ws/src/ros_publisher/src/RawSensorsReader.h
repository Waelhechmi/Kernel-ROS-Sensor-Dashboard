#pragma once
#include <cstdint>

struct BmpData {
    float temperature; // °C
    float pressure;    // hPa
};

class RawSensorsReader {
public:
    RawSensorsReader();
    ~RawSensorsReader();

    bool readTMP(float &tempC);
    bool readBMP(BmpData &data);
    bool readADS(float &voltage);

private:
    int fd_tmp;
    int fd_bmp;
    int fd_ads;
};
