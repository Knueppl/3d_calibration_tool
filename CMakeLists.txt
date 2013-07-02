cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(FORM_DIR ${PROJECT_SOURCE_DIR}/forms)

#find_package(PCL 1.7 REQUIRED)
pkg_check_modules(OPENNI libopenni2)
find_package(Qt4 REQUIRED)
find_package(VTK REQUIRED)

include(${QT_USE_FILE})

add_definitions(${QT_DEFINITIONS}
#                ${PCL_DEFINITIONS}
                )

set(MOC_HEADERS
    ${SRC_DIR}/OpenCvWidget.h
    ${SRC_DIR}/OpenNiSensorThread.h
    ${SRC_DIR}/MainWidget.h
    ${SRC_DIR}/CloudWidget.h
    )

set(FORMS
    ${FORM_DIR}/MainWidget.ui
    )

QT4_WRAP_CPP(HEADERS_MOC ${MOC_HEADERS})
QT4_WRAP_UI(FORMS_MOC ${FORMS})
#QT4_ADD_RESOURCES(OHMGUI_RESOURCES_RCC ${OHMGUI_RESOURCES})

include_directories(${PROJECT_SOURCE_DIR}/build
                    ${OPENNI_INCLUDE_DIRS}
                    ${VTK_INCLUDE_DIRS}
                    ${SRC_DIR}
                    /usr/local/include/pcl-1.7/
                    /usr/include/eigen3
#                    ${PCL_INCLUDE_DIRS}
                    )

#message(STATUS "Paths:")
#message(STATUS ${PCL_INCLUDE_DIRS})

link_directories(${OPENNI_LIBRARY_DIRS}
#                 ${PCL_LIBRARY_DIRS}
                 )


add_executable(3d-calibration-tool
               ${SRC_DIR}/main.cpp
               ${SRC_DIR}/OpenCvWidget.cpp
               ${SRC_DIR}/OpenNiDevice.cpp
               ${SRC_DIR}/OpenNiSensor.cpp
               ${SRC_DIR}/OpenNiSensorThread.cpp
               ${SRC_DIR}/MainWidget.cpp
               ${SRC_DIR}/CloudWidget.cpp
               ${HEADERS_MOC}
               ${FORMS_MOC}
               )

target_link_libraries(3d-calibration-tool
                      ${QT_LIBRARIES}
#                      ${PCL_LIBRARIES}
                      opencv_core
                      ${OPENNI_LIBRARIES}
                      QVTK
                      ${VTK_LIBRARIES}
                      )
