TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
LIBS += -lboost_system\
        -lboost_thread\

SOURCES += \
    main.cpp \
    udp_server.cpp \
    udp_client.cpp

HEADERS += \
    udp_server.h \
    udp_client.h
