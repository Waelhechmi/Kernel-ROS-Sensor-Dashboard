QT += core gui widgets

TARGET = SensorDashboardGUI
TEMPLATE = app

SOURCES += main.cpp \
           mainwindow.cpp \
           SensorsReader.cpp

HEADERS += mainwindow.h \
           SensorsReader.h

FORMS += mainwindow.ui
