#pragma once
#include <QObject>

struct BmpData {
    float temperature;
    float pressure;
};

class SensorsReader : public QObject {
    Q_OBJECT
public:
    explicit SensorsReader(QObject* parent = nullptr);
    ~SensorsReader();

    bool readTMP(float &tempC);
    bool readBMP(BmpData &data);
    bool readADS(float &voltage);

private:
    int fd_tmp, fd_bmp, fd_ads;
};
