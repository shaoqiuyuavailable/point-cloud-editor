QT       += core gui
include(D:/PCL/PCL-1.13.0/pcl1.13.0.pri)
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
DEFINES += QT_ENABLE_HIGHDPI_SCALING
DEFINES += VTK_OPENGL2_BACKEND=1

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    addpointdialog.cpp \
    main.cpp \
    mainwindow.cpp \
    operatestep.cpp \
    querypointdialog.cpp \
    sizedstack.cpp

HEADERS += \
    PointMoveTask.h \
    addpointdialog.h \
    mainwindow.h \
    operatestep.h \
    querypointdialog.h \
    sizedstack.h

FORMS += \
    addpointdialog.ui \
    mainwindow.ui \
    querypointdialog.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
