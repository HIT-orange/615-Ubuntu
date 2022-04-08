TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=255
DEFINES += DO_NOT_USE_SHARED_MEMORY
XENO_DIR = /usr/xenomai
QMAKE_CFLAGS_WARN_ON += -Wall -W -Wno-unused-parameter -Wno-unused-variable
XENO_CONFIG = $$XENO_DIR/bin/xeno-config
XCFLAGS = $(shell $$XENO_CONFIG --skin=native  --cflags)
XLDFLAGS = $(shell $$XENO_CONFIG --skin=native --ldflags)
QMAKE_CXXFLAGS += $$XCFLAGS

DEFINES += _GLIBCXX_INCLUDE_NEXT_C_HEADERS

SOURCES += \
    main.cpp \


HEADERS += \




INCLUDEPATH += \
    ./  \
    /usr/xenomai/include/cobalt \
    /usr/xenomai/include \
    /usr/xenomai/include/alchemy \


LIBS += -lpthread \
    -L/usr/xenomai/lib/ -lalchemy \
    -L/usr/xenomai/lib/ -lcopperplate \
    -L/usr/xenomai/lib/ -lcobalt \
    -L/usr/xenomai/lib/ -lmodechk \
    #-L/usr/xenomai/lib/ -lsmokey \
    #-L/usr/xenomai/lib/ -ltrank \
    -L/usr/xenomai/lib/ -lanalogy \
    $$XLDFLAGS
#LIBS += \
#		-L./lib -lqwt \

DISTFILES += \
    main_back1.txt
