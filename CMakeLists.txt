cmake_minimum_required(VERSION 2.8)
PROJECT(Bscan)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(USB-1 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories($(USB_1_INCLUDE_DIR))
include_directories($(OpenCV_INCLUDE_DIR))
include_directories(/usr/include/spinnaker)

add_executable(Bscan bscan.cpp FDOCT.cpp)
target_link_libraries(Bscan ${LIBUSB_1_LIBRARIES} -lSpinnaker -lpthread ${OpenCV_LIBS})

add_executable(Cscan cscan.cpp FDOCT.cpp)
target_link_libraries(Cscan ${LIBUSB_1_LIBRARIES} -lSpinnaker -lpthread ${OpenCV_LIBS})

