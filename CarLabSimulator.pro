#-------------------------------------------------
#
# Project created by QtCreator 2017-11-21T16:01:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CarLabSimulator
TEMPLATE = app
CONFIG += warn_on

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

LIBS += -lboost_system\
        -lboost_thread\

SOURCES += \
        main.cpp \
        gui.cpp \
    car.cpp \
    car_model_bicycle.cpp \
    map.cpp \
    simulator.cpp \
    state_memory.cpp

HEADERS += \
        gui.h \
    car.h \
    car_mode_bicycle.h \
    car_model_bicycle.h \
    map.h \
    simulator.h \
    state_memory.h

FORMS += \
        gui.ui
