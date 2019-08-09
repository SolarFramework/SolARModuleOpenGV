## remove Qt dependencies
QT     -= core gui
CONFIG -= qt

## global defintions : target lib name, version
TARGET = SolARModuleOpenGV
FRAMEWORK = $$TARGET
VERSION=0.6.0

DEFINES += MYVERSION=$${VERSION}
DEFINES += TEMPLATE_LIBRARY
CONFIG += c++1z


CONFIG(debug,debug|release) {
    DEFINES += _DEBUG=1
    DEFINES += DEBUG=1
}

CONFIG(release,debug|release) {
    DEFINES += _NDEBUG=1
    DEFINES += NDEBUG=1
}


DEPENDENCIESCONFIG = shared recurse

include (../builddefs/qmake/templatelibconfig.pri)

## DEFINES FOR MSVC/INTEL C++ compilers
msvc {
DEFINES += "_BCOM_SHARED=__declspec(dllexport)"
}

INCLUDEPATH += interfaces/

HEADERS += interfaces/SolARModuleOpengv_traits.h \
interfaces/SolAROpengvAPI.h \
interfaces/SolAROpenGVHelper.h \
interfaces/PoseEstimationEPnp.h \
interfaces/PoseEstimationP3PGao.h \
interfaces/PoseEstimationP3PKneip.h \
interfaces/PoseEstimationUPnp.h \
interfaces/PoseEstimationSACEPnp.h\
interfaces/PoseEstimationSACP3PGao.h \
interfaces/PoseEstimationSACP3PKneip.h \
interfaces/Triangulation.h 



SOURCES += src/SolARModuleOpengv.cpp \
src/PoseEstimationEPnp.cpp \
src/PoseEstimationP3PGao.cpp \
src/PoseEstimationP3PKneip.cpp \
src/PoseEstimationUPnp.cpp \
src/PoseEstimationSACEPnp.cpp\
src/PoseEstimationSACP3PGao.cpp \
src/PoseEstimationSACP3PKneip.cpp \
src/Triangulation.cpp 


unix {
    QMAKE_CXXFLAGS += -Wignored-qualifiers
#    QMAKE_CXX = clang++
#    QMAKE_LINK = clang++	
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

header_files.path = $${PROJECTDEPLOYDIR}/interfaces
header_files.files = $$files($${PWD}/interfaces/*.h*)

xpcf_xml_files.path = $$(HOME)/.xpcf/SolAR
xpcf_xml_files.files=$$files($${PWD}/xpcf*.xml)

INSTALLS += header_files
INSTALLS += xpcf_xml_files

