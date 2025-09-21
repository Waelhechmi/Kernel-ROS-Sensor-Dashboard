#pragma once

#include <QMainWindow>
#include <QTimer>
#include "SensorsReader.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updateTMP();
    void updateBMP();
    void updateADS();

private:
    Ui::MainWindow *ui;
    SensorsReader *reader;
    QTimer *tmpTimer;
    QTimer *bmpTimer;
    QTimer *adsTimer;
};
