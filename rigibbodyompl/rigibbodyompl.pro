TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp



LIBS += -L/usr/local/lib/ -lompl



INCLUDEPATH += \
    /usr/local/include\
    /usr/local/include/ompl-1.5\
    /usr/include/eigen3
