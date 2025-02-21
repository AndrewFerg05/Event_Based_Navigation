cmake_minimum_required(VERSION 3.10)

# Project name
project(Event_Based_Navigation)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use all cpp files in src and utils directories
file(GLOB SOURCES 
    "src/*.cpp" 
    "utils/common/src/*.cpp"
)

message("Building for Raspberry Pi (Linux)")

# Find libraries using pkg-config
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBCAER REQUIRED libcaer)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
pkg_check_modules(LIBSERIAL REQUIRED libserialport)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(glog REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

# OpenVINS
set(OPENVINS_INCLUDE_DIR /usr/local/include/open_vins/)
set(OPENVINS_LIBRARIES /usr/local/lib/libov_msckf_lib.so)

# Include directories
include_directories(
    include 
    utils/common/include
    ${LIBCAER_INCLUDE_DIRS} 
    ${LIBUSB_INCLUDE_DIRS} 
    ${LIBSERIAL_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}  # Use OpenCV's detected include directories
    ${Eigen3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${OPENVINS_INCLUDE_DIR}  # OpenVINS headers
)

# Link directories
link_directories(
    ${LIBCAER_LIBRARY_DIRS} 
    ${LIBUSB_LIBRARY_DIRS} 
    ${LIBSERIAL_LIBRARY_DIRS} 
    ${OpenCV_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link required libraries
target_link_libraries(${PROJECT_NAME} PRIVATE 
    ${LIBCAER_LIBRARIES} 
    ${LIBUSB_LIBRARIES}
    ${LIBSERIAL_LIBRARIES} 
    ${OpenCV_LIBS}  # Correctly links OpenCV components
    glog::glog      # Link Google glog
    ${Boost_LIBRARIES}
    ${CERES_LIBRARIES}
    yaml-cpp        # YAML-CPP
    Boost::filesystem 
    Boost::system
    ${OPENVINS_LIBRARIES}  # Link OpenVINS
)