## remove Qt dependencies
QT       -= core gui
CONFIG -= qt

## global defintions : target lib name, version
TARGET = SolARPipelineTest_SLAM
VERSION=0.11.0
PROJECTDEPLOYDIR = $${PWD}/../../../deploy

DEFINES += MYVERSION=$${VERSION}
CONFIG += c++1z
CONFIG += console

include(findremakenrules.pri)

CONFIG(debug,debug|release) {
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}

DEPENDENCIESCONFIG = sharedlib install_recurse

PROJECTCONFIG = QTVS

#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib, QMAKE_TARGET.arch and PROJECTDEPLOYDIR MUST BE DEFINED BEFORE templatelibconfig.pri inclusion
include ($$shell_quote($$shell_path($${QMAKE_REMAKEN_RULES_ROOT}/templateappconfig.pri)))  # Shell_quote & shell_path required for visual on windows

HEADERS += \

SOURCES += \
    main.cpp

unix {
    LIBS += -ldl
    QMAKE_CXXFLAGS += -DBOOST_LOG_DYN_LINK

    # Avoids adding install steps manually. To be commented to have a better control over them.
    QMAKE_POST_LINK += "make install install_deps"
}

macx {
    DEFINES += _MACOS_TARGET_
    QMAKE_MAC_SDK= macosx
    QMAKE_CFLAGS += -mmacosx-version-min=10.7 -std=c11 #-x objective-c++
    QMAKE_CXXFLAGS += -mmacosx-version-min=10.7 -std=c11 -std=c++11 -O3 -fPIC#-x objective-c++
    QMAKE_LFLAGS += -mmacosx-version-min=10.7 -v -lstdc++
    LIBS += -lstdc++ -lc -lpthread
}

win32 {

    DEFINES += WIN64 UNICODE _UNICODE
    QMAKE_COMPILER_DEFINES += _WIN64
    QMAKE_CXXFLAGS += -wd4250 -wd4251 -wd4244 -wd4275
}

DISTFILES += \
    PipelineNaturalImageMarker.xml

config_files.path = $${TARGETDEPLOYDIR}
config_files.files= $$files($${PWD}/SolARPipelineTest_SLAM_conf.xml)\
                    $$files($${PWD}/camera_calibration.json)\
                    $$files($${PWD}/fiducialMarker.yml)\
                    $$files($${PWD}/FiducialMarker.gif)
INSTALLS += config_files

linux {
  run_install.path = $${TARGETDEPLOYDIR}
  run_install.files = $${PWD}/../../../run.sh
  CONFIG(release,debug|release) {
    run_install.extra = cp $$files($${PWD}/../../../runRelease.sh) $${PWD}/../../../run.sh
  }
  CONFIG(debug,debug|release) {
    run_install.extra = cp $$files($${PWD}/../../../runDebug.sh) $${PWD}/../../../run.sh
  }
  run_install.CONFIG += nostrip
  INSTALLS += run_install
}

OTHER_FILES += \
    packagedependencies.txt \
    SolARPipelineTest_SLAM_conf.xml

#NOTE : Must be placed at the end of the .pro
include ($$shell_quote($$shell_path($${QMAKE_REMAKEN_RULES_ROOT}/remaken_install_target.pri)))) # Shell_quote & shell_path required for visual on windows
