## remove Qt dependencies
QT       -= core gui
CONFIG -= app_bundle qt

QMAKE_PROJECT_DEPTH = 0

## global defintions : target lib name, version
INSTALLSUBDIR = SolARBuild
TARGET = SolARPipelineSLAM
FRAMEWORK = $${TARGET}
VERSION=1.0.0

DEFINES += MYVERSION=$${VERSION}
DEFINES += TEMPLATE_LIBRARY
CONFIG += c++1z

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

## Configuration for Visual Studio to install binaries and dependencies. Work also for QT Creator by replacing QMAKE_INSTALL
PROJECTCONFIG = QTVS

#NOTE : CONFIG as staticlib or sharedlib, DEPENDENCIESCONFIG as staticlib or sharedlib and PROJECTDEPLOYDIR MUST BE DEFINED BEFORE templatelibbundle.pri inclusion
include ($$shell_quote($$shell_path($${QMAKE_REMAKEN_RULES_ROOT}/templatelibconfig.pri)))  # Shell_quote & shell_path required for visual on windows

## DEFINES FOR MSVC/INTEL C++ compilers
msvc {
DEFINES += "_BCOM_SHARED=__declspec(dllexport)"
}

INCLUDEPATH += interfaces/

HEADERS += interfaces/pipelineSlam.h \

SOURCES += src/pipelineSlam.cpp \
           src/component.cpp

unix {
    # Avoids adding install steps manually. To be commented to have a better control over them.
    QMAKE_POST_LINK += "make install"
}

unix {
    QMAKE_CXXFLAGS += -Wignored-qualifiers
}

win32 {

    DEFINES += WIN64 UNICODE _UNICODE
    QMAKE_COMPILER_DEFINES += _WIN64
    QMAKE_CXXFLAGS += -wd4250 -wd4251 -wd4244 -wd4275 /Od
}

header_files.path = $${PROJECTDEPLOYDIR}/interfaces
header_files.files = $$files($${PWD}/interfaces/*.h*)

xpcf_xml_files.path = $${USERHOMEFOLDER}/.xpcf/SolAR
xpcf_xml_files.files=$$files($${PWD}/xpcf*.xml)

configuration_files.path = $${PROJECTDEPLOYDIR}/configuration
configuration_files.files = $$files($${PWD}/tests/SolARPipelineTest_SLAM/SolARPipelineTest_SLAM_conf.xml)

INSTALLS += header_files
INSTALLS += xpcf_xml_files
INSTALLS += configuration_files

OTHER_FILES += \
    packagedependencies.txt

#NOTE : Must be placed at the end of the .pro
include ($$shell_quote($$shell_path($${QMAKE_REMAKEN_RULES_ROOT}/remaken_install_target.pri)))) # Shell_quote & shell_path required for visual on windows


