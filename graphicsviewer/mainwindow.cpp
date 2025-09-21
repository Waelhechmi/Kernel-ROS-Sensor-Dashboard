#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QString>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    reader = new SensorsReader(this);

    // --- Timers pour mise à jour indépendante ---
    tmpTimer = new QTimer(this);
    connect(tmpTimer, &QTimer::timeout, this, &MainWindow::updateTMP);
    tmpTimer->start(500); // TMP102 toutes les 500ms

    bmpTimer = new QTimer(this);
    connect(bmpTimer, &QTimer::timeout, this, &MainWindow::updateBMP);
    bmpTimer->start(1000); // BMP280 toutes les 1s

    adsTimer = new QTimer(this);
    connect(adsTimer, &QTimer::timeout, this, &MainWindow::updateADS);
    adsTimer->start(250); // ADS1115 toutes les 250ms
}

MainWindow::~MainWindow() {
    delete ui;
}

// --- Fonctions de mise à jour ---
void MainWindow::updateTMP() {
    float tmp;
    if(reader->readTMP(tmp)) {
        ui->progressTMP->setValue(static_cast<int>(tmp));
        ui->labelTMP->setText(QString::number(tmp,'f',2) + " °C");
    } else {
        ui->labelTMP->setText("Erreur TMP102");
    }
}

void MainWindow::updateBMP() {
    BmpData bmp;
    if(reader->readBMP(bmp)) {
        ui->progressBMP->setValue(static_cast<int>(bmp.pressure));
        ui->labelBMP->setText(
            QString::number(bmp.temperature,'f',2) + " °C, " +
            QString::number(bmp.pressure,'f',2) + " hPa"
        );
    } else {
        ui->labelBMP->setText("Erreur BMP280");
    }
}

void MainWindow::updateADS() {
    float voltage;
    if(reader->readADS(voltage)) {
        ui->progressADS->setValue(static_cast<int>(voltage*1000));
        ui->labelADS->setText(QString::number(voltage,'f',3) + " V");
    } else {
        ui->labelADS->setText("Erreur ADS1115");
    }
}
