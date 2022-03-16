TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


DEFINES += BOOST_USE_LIB

SOURCES += \
        main.cpp

LIBS += \
    -L/usr/local/lib -lompl\
    -L/home/hit/hit-6dof-projects/third/boost/lib -lboost_system\
    -L/home/hit/hit-6dof-projects/third/boost/lib -lboost_filesystem

INCLUDEPATH += \
    /usr/include\
    /usr/local/include\
    /usr/include/boost\
    /usr/local/include/ompl-1.5\
    /usr/include/eigen3
