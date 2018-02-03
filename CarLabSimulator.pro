
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = CarLabSimulator
TEMPLATE = app
CONFIG += warn_on
QMAKE_CXXFLAGS += -std=c++0x

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
        -pthread\

SOURCES += \
        main.cpp \
        gui.cpp \
    car.cpp \
    car_model_bicycle.cpp \
    map.cpp \
    simulator.cpp \
    state_memory.cpp \
    qcustomplot/qcustomplot.cpp \
    plot_window.cpp \
    udp_server.cpp \
    udp_client.cpp \
    params_window.cpp \
    car_model_bicycle_dynamic.cpp

HEADERS += \
        gui.h \
    car.h \
    car_mode_bicycle.h \
    car_model_bicycle.h \
    map.h \
    simulator.h \
    state_memory.h \
    qcustomplot/qcustomplot.h \
    plot_window.h \
    customtypes.h \
    udp_server.h \
    udp_client.h \
    params_window.h \
    car_model_bicycle_dynamic.h

FORMS += \
        gui.ui
