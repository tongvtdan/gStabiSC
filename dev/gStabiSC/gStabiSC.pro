#-------------------------------------------------
#
# Project created by QtCreator 2013-07-31T17:06:50
#
#-------------------------------------------------

QT       += core gui xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = gStabiSC
TEMPLATE = app

#CONFIG +=qwt
win32: include(C:\Qwt-6.1.0\features\qwt.prf)
linux*: include(/usr/local/qwt-6.1.0/features/qwt.prf)


SOURCES += main.cpp\
        mainwindow.cpp \
        thirdParty/attitude_indicator/attitude_indicator.cpp
#    src/mavlinkmanager.cpp

HEADERS  += mainwindow.h \
            thirdParty/attitude_indicator/attitude_indicator.h \
    src/configuration.h
#    src/mavlinkmanager.h

FORMS    += mainwindow.ui

include(thirdParty/qextserialport/src/qextserialport.pri)

INCLUDEPATH += thirdParty/mavlink/v1.0/gremsyBGC \
               thirdParty/mavlink/v1.0
RESOURCES += \
    gGimbalResources.qrc


RC_FILE = files/images/appicon.rc

