/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QLabel *titleTMP;
    QLabel *labelTMP;
    QProgressBar *progressTMP;
    QLabel *titleBMP;
    QLabel *labelBMP;
    QProgressBar *progressBMP;
    QLabel *titleADS;
    QLabel *labelADS;
    QProgressBar *progressADS;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(500, 300);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        titleTMP = new QLabel(centralwidget);
        titleTMP->setObjectName(QString::fromUtf8("titleTMP"));
        titleTMP->setGeometry(QRect(40, 0, 100, 20));
        titleTMP->setAlignment(Qt::AlignCenter);
        labelTMP = new QLabel(centralwidget);
        labelTMP->setObjectName(QString::fromUtf8("labelTMP"));
        labelTMP->setGeometry(QRect(50, 220, 100, 30));
        labelTMP->setAlignment(Qt::AlignCenter);
        progressTMP = new QProgressBar(centralwidget);
        progressTMP->setObjectName(QString::fromUtf8("progressTMP"));
        progressTMP->setGeometry(QRect(50, 40, 30, 180));
        progressTMP->setMinimum(0);
        progressTMP->setMaximum(100);
        progressTMP->setOrientation(Qt::Vertical);
        titleBMP = new QLabel(centralwidget);
        titleBMP->setObjectName(QString::fromUtf8("titleBMP"));
        titleBMP->setGeometry(QRect(190, 0, 100, 20));
        titleBMP->setAlignment(Qt::AlignCenter);
        labelBMP = new QLabel(centralwidget);
        labelBMP->setObjectName(QString::fromUtf8("labelBMP"));
        labelBMP->setGeometry(QRect(200, 220, 150, 30));
        labelBMP->setAlignment(Qt::AlignCenter);
        progressBMP = new QProgressBar(centralwidget);
        progressBMP->setObjectName(QString::fromUtf8("progressBMP"));
        progressBMP->setGeometry(QRect(200, 40, 30, 180));
        progressBMP->setMinimum(900);
        progressBMP->setMaximum(1100);
        progressBMP->setOrientation(Qt::Vertical);
        titleADS = new QLabel(centralwidget);
        titleADS->setObjectName(QString::fromUtf8("titleADS"));
        titleADS->setGeometry(QRect(340, 0, 100, 20));
        titleADS->setAlignment(Qt::AlignCenter);
        labelADS = new QLabel(centralwidget);
        labelADS->setObjectName(QString::fromUtf8("labelADS"));
        labelADS->setGeometry(QRect(350, 220, 150, 30));
        labelADS->setAlignment(Qt::AlignCenter);
        progressADS = new QProgressBar(centralwidget);
        progressADS->setObjectName(QString::fromUtf8("progressADS"));
        progressADS->setGeometry(QRect(350, 40, 30, 180));
        progressADS->setMinimum(0);
        progressADS->setMaximum(3300);
        progressADS->setOrientation(Qt::Vertical);
        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Sensor Dashboard", nullptr));
        titleTMP->setText(QCoreApplication::translate("MainWindow", "TMP102", nullptr));
        labelTMP->setText(QCoreApplication::translate("MainWindow", "-- \302\260C", nullptr));
        titleBMP->setText(QCoreApplication::translate("MainWindow", "BMP280", nullptr));
        labelBMP->setText(QCoreApplication::translate("MainWindow", "--", nullptr));
        titleADS->setText(QCoreApplication::translate("MainWindow", "ADS1115", nullptr));
        labelADS->setText(QCoreApplication::translate("MainWindow", "-- V", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
