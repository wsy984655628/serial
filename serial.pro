#-------------------------------------------------
#
# Project created by QtCreator 2016-03-08T15:10:42
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets serialport

TARGET = serial
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    SerialCommunication/serialconsole.cpp \
    SerialCommunication/serialsettingdialog.cpp \
    comm/mavlinkdecoder.cpp \
    comm/linkmanager.cpp \
    comm/mavlinkprotocol.cpp \
    my_gs.cpp \
    uas/uas.cpp \
    uas/uasmanager.cpp

HEADERS  += mainwindow.h \
    SerialCommunication/serialconsole.h \
    SerialCommunication/serialsettingdialog.h \
    comm/mavlinkdecoder.h \
    comm/gsmavlink.h \
    comm/linkmanager.h \
    comm/mavlinkprotocol.h \
    my_gs.h \
    uas/uasinterface.h \
    uas/uas.h \
    uas/uasmanager.h

FORMS    += mainwindow.ui \
    SerialCommunication/serialsettingdialog.ui

