TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    URG04LX.cpp \
    hokuyo.c \
    serial_device.c

HEADERS += \
    hokuyo.h \
    message_utils.h \
    serial_device.h \
    URG04LX.hpp \
    dbtestclass.h

LIBS += -lm -lpthread -fPIC
