#-------------------------------------------------
#
# Project created by QtCreator 2019-12-20T16:23:41
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = geditor
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cc \
        geditor_mainwindow.cc \
    dbimportdialog.cc \
    centerpointdialog.cc \
    import_func_point_dialog.cc \
    import_gps_dialog.cc \
    settingdialog.cc \
    pointcloudfilterdialog.cc \
    vdbuploaddialog.cc \
    downloaddialog.cc

HEADERS += \
        geditor_mainwindow.h \
    dbimportdialog.h \
    centerpointdialog.h \
    import_func_point_dialog.h \
    import_gps_dialog.h \
    settingdialog.h \
    pointcloudfilterdialog.h \
    vdbuploaddialog.h \
    downloaddialog.h

FORMS += \
        geditor_mainwindow.ui \
    dbimportdialog.ui \
    centerpointdialog.ui \
    importfuncpointdialog.ui \
    importgpsdialog.ui \
    settingdialog.ui \
    pointcloudfilterdialog.ui \
    vdbuploaddialog.ui \
    downloaddialog.ui

RESOURCES += \
    res.qrc
