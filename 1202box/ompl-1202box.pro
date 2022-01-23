TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CFLAGS_WARN_ON += -Wall -W -Wno-unused-parameter -Wno-unused-variable

DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=255
DEFINES += DO_NOT_USE_SHARED_MEMORY
DEFINES += WIN32
DEFINES += _CONSOLE
DEFINES += _GLIBCXX_INCLUDE_NEXT_C_HEADERS
#DEFINES += _MSC_VER
DEFINES +=BOOST_USE_LIB


LIBS += -lpthread


LIBS += \
    -lpthread \
    -L../ReflexxesTypeII/lib/ -lReflexxesTypeII  \
    -L../rbdl-orb-3.0.0/build -lrbdl \
    -L../rbdl-orb-3.0.0/build -lrbdl_urdfreader \
    -L/usr/local/lib/ -lompl\
    -L/usr/local/lib -lfcl \
    -L/usr/local/lib -lccd


SOURCES += \
    collision.cpp \
    cubicpolynomial.cpp \
    dynamics.cpp \
    main.cpp \
    motionplanning.cpp \
    myplanner.cpp \
    remoteApi/extApi.c \
    remoteApi/extApiPlatform.c \
    vrepshow.cpp


INCLUDEPATH += \
    /usr/include/pcl-1.7\
    /usr/include/pcl-1.7/pcl\
    /usr/include/pcl-1.7/pcl/io\
    /usr/include\
    /usr/include/ccd\
    /usr/local/fcl/include \
    /usr/local/fcl/include/common\
    /usr/local/fcl/include/broadphase\
    /usr/local/fcl/include/narrowphase\
    /usr/local/fcl/include/geometry\
    /usr/local/fcl/include/math\
    /usr/local/include/ompl-1.5 \
    /usr/local/include/ompl-1.5/ompl \
    /usr/local/include/ompl-1.5/ompl/geometric\
    /usr/local/include/ompl-1.5/ompl/geometric/planners/rrt\
    /usr/local/include/ompl-1.5/ompl/base\
    /usr/local/include/ompl-1.5/ompl/base/spaces\
    /usr/local/include/ompl-1.5/ompl/base/goals\
    /usr/local/include/ompl-1.5/ompl/base/samplers\
    $$PWD/fcl/test\
    $$PWD/fcl/test/gtest/include\
    $$PWD/fcl/test/gtest/include/gtest/internal\
    $$PWD/fcl/test/gtest/include/gtest\
    $$PWD/remoteApi\
    $$PWD/remoteApi/include\
    $$PWD/rbdl-orb-3.0.0/include\
    $$PWD/eigen-3.3.9\
    $$PWD/rbdl-orb-3.0.0/build/include\
    $$PWD/rbdl-orb-3.0.0\
    $$PWD/boost/include\
    $$PWD/ReflexxesTypeII/include \

#    $$PWD/rbdl-orb-3.0.0/include \
#
HEADERS += \
    ./OmplSpace.h \
    ./dynamics.h \
    ./vrepshow.h\
#    kinova-api/Kinova.API.CommLayerUbuntu.h \
#    kinova-api/Kinova.API.EthCommLayerUbuntu.h \
#    kinova-api/Kinova.API.EthCommandLayerUbuntu.h \
#    kinova-api/Kinova.API.UsbCommandLayerUbuntu.h \
#    kinova-api/KinovaTypes.h \
#    remoteApi/extApi.h \
 \#    remoteApi/extApiPlatform.h
    collision.h \
    cubicpolynomial.h \
    main.h \
    motionplanning.h \
    myplanner.h

