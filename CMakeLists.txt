cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(FORM_DIR ${PROJECT_SOURCE_DIR}/forms)

find_package(PCL 1.7 REQUIRED)
pkg_check_modules(OPENNI libopenni2)
find_package(Qt4 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV 2.4.6.1 REQUIRED)

include(${QT_USE_FILE})

add_definitions(${QT_DEFINITIONS}
                ${PCL_DEFINITIONS}
                )

# its from pcl website
add_definitions(-Wall -O2 -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)

set(MOC_HEADERS
    ${SRC_DIR}/OpenCvWidget.h
    ${SRC_DIR}/OpenNiSensorThread.h
    ${SRC_DIR}/MainWidget.h
    ${SRC_DIR}/CloudWidget.h
    ${SRC_DIR}/CloudCatcher.h
    ${SRC_DIR}/PlaneFinder.h
    ${SRC_DIR}/ConfigDialog.h
    ${SRC_DIR}/ThermoCamThread.h
    ${SRC_DIR}/PointFinder.h
    )

set(FORMS
    ${FORM_DIR}/MainWidget.ui
    ${FORM_DIR}/ConfigDialog.ui
    )

QT4_WRAP_CPP(HEADERS_MOC ${MOC_HEADERS})
QT4_WRAP_UI(FORMS_MOC ${FORMS})
#QT4_ADD_RESOURCES(OHMGUI_RESOURCES_RCC ${OHMGUI_RESOURCES})

include_directories(${PROJECT_SOURCE_DIR}/build
                    ${OPENNI_INCLUDE_DIRS}
                    ${VTK_INCLUDE_DIRS}
                    ${SRC_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    )

link_directories(${OPENNI_LIBRARY_DIRS}
                 ${OpenCV_LIBRARY_DIRS}
                 ${PCL_LIBRARY_DIRS}
#                 ${PROJECT_SOURCE_DIR}/lib/i386
#                 ${PROJECT_SOURCE_DIR}/lib/x86_64
                 ${PROJECT_SOURCE_DIR}/lib
                 )


add_executable(3d-calibration-tool
               ${SRC_DIR}/main.cpp
               ${SRC_DIR}/OpenCvWidget.cpp
               ${SRC_DIR}/OpenNiDevice.cpp
               ${SRC_DIR}/OpenNiSensor.cpp
               ${SRC_DIR}/OpenNiSensorThread.cpp
               ${SRC_DIR}/MainWidget.cpp
               ${SRC_DIR}/CloudWidget.cpp
               ${SRC_DIR}/CloudCatcher.cpp
               ${SRC_DIR}/PlaneFinder.cpp
               ${SRC_DIR}/ConfigDialog.cpp
               ${SRC_DIR}/ThermoCamThread.cpp
               ${SRC_DIR}/ThermoCam.cpp
               ${SRC_DIR}/ImageBuilder.cpp
               ${SRC_DIR}/ImagerUVC.cpp
               ${SRC_DIR}/VideoDevice.cpp
               ${SRC_DIR}/PointFinder.cpp
               ${HEADERS_MOC}
               ${FORMS_MOC}
               )

target_link_libraries(3d-calibration-tool
                      ${OPENNI_LIBRARIES}
                      ${QT_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      QVTK
                      ${VTK_LIBRARIES}
                      Imager
                      ImageProcessing
                      udev
                      )
