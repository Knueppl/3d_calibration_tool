cmake_minimum_required (VERSION 2.6)
include(FindPkgConfig)

set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(FORM_DIR ${PROJECT_SOURCE_DIR}/forms)

find_package(PCL 1.7 REQUIRED)
find_package(Qt4 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV 2.3 REQUIRED)

include(${QT_USE_FILE})

add_definitions(${QT_DEFINITIONS}
                ${PCL_DEFINITIONS}
                )


# sources, headers, forms and libs ...

set(SRC ${SRC_DIR}/main.cpp
        ${SRC_DIR}/OpenCvWidget.cpp
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
        )

set(LIBS ${QT_LIBRARIES}
         ${PCL_LIBRARIES}
         ${OpenCV_LIBS}
         QVTK
         ${VTK_LIBRARIES}
         Imager
         ImageProcessing
         udev
         )

set(MOC_HEADERS
    ${SRC_DIR}/OpenCvWidget.h
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

set(RESOURCES ${PROJECT_SOURCE_DIR}/src/picture/resources.qrc)


# here is a important part! OpenNI 2 is used by default, so if you want to use the fucking Kinect you have to set the Kinect flag (just uncomment next line).
set(USE_KINECT "Hack")

if (USE_KINECT)
   include_directories(/usr/include/ni)

   set(SRC ${SRC}
           ${SRC_DIR}/Kinect.cpp
           ${SRC_DIR}/KinectSensorThread.cpp
           ${SRC_DIR}/KinectSensor.cpp
           )

   set(MOC_HEADERS ${MOC_HEADERS}
                   ${SRC_DIR}/KinectSensorThread.h
                   )

   set(LIBS ${LIBS}
            OpenNI
            )

   add_definitions(-D___USE_KINECT___)
else()
   pkg_check_modules(OPENNI libopenni2)
   include_directories(${OPENNI_INCLUDE_DIRS})
   link_directories(${OPENNI_LIBRARY_DIRS})

   set(SRC ${SRC}
           ${SRC_DIR}/OpenNiDevice.cpp
           ${SRC_DIR}/OpenNiSensor.cpp
           ${SRC_DIR}/OpenNiSensorThread.cpp
           )

   set(MOC_HEADERS ${MOC_HEADERS}
                   ${SRC_DIR}/OpenNiSensorThread.h
                   )

   set(LIBS ${LIBS}
            ${OPENNI_LIBRARIES}
            )
endif()


# its from pcl website
add_definitions(-Wall -O2 -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)


QT4_WRAP_CPP(HEADERS_MOC ${MOC_HEADERS})
QT4_WRAP_UI(FORMS_MOC ${FORMS})
QT4_ADD_RESOURCES(RESOURCES_RCC ${RESOURCES})

include_directories(${PROJECT_SOURCE_DIR}/build
                    ${OPENNI_INCLUDE_DIRS}
                    ${VTK_INCLUDE_DIRS}
                    ${SRC_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    )

link_directories(${OpenCV_LIBRARY_DIRS}
                 ${PCL_LIBRARY_DIRS}
#                 ${PROJECT_SOURCE_DIR}/lib/i386
#                 ${PROJECT_SOURCE_DIR}/lib/x86_64
                 ${PROJECT_SOURCE_DIR}/lib
                 )


add_executable(3d-calibration-tool
               ${SRC}
               ${HEADERS_MOC}
               ${FORMS_MOC}
               ${RESOURCES_RCC}
               )

target_link_libraries(3d-calibration-tool
                      ${LIBS}
                      )
