# Prerequisites
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
    "utils/frontend/src/*.cpp"
    "utils/backend/src/*.cpp"
)

if (WIN32)
    message("Windows Device")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_TIMESPEC_DEFINED")

    # Windows paths for dependencies
    set(LIBCAER_INCLUDE_DIRS "C:/msys64/mingw64/include")
    set(LIBCAER_LIBRARY_DIRS "C:/msys64/mingw64/lib")
    set(LIBCAER_LIBRARIES "libcaer")

    set(OpenCV_INCLUDE_DIRS "C:/msys64/mingw64/include/opencv4")
    set(OpenCV_LIBRARY_DIRS "C:/msys64/mingw64/lib")
    set(OpenCV_LIBRARIES opencv_core opencv_imgcodecs opencv_highgui)

    set(LIBSERIAL_INCLUDE_DIRS "C:/msys64/mingw64/include")
    set(LIBSERIAL_LIBRARY_DIRS "C:/msys64/mingw64/lib")
    set(LIBSERIAL_LIBRARIES "libserialport")

    set(YAML_CPP_INCLUDE_DIR "C:/msys64/mingw64/include")
    set(YAML_CPP_LIB "C:/msys64/mingw64/lib/libyaml-cpp.a")

    find_package(glog REQUIRED)

    # Find Eigen3 on Windows
    find_path(EIGEN3_INCLUDE_DIR NAMES Eigen/Core PATHS "C:/msys64/mingw64/include/eigen3")
    if(NOT EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR "Eigen3 not found! Install it via MSYS2 using 'pacman -S mingw-w64-x86_64-eigen3'")
    endif()

else()
    message("Mac or Linux Device")

    # Find libraries using pkg-config
    find_package(PkgConfig REQUIRED)

    pkg_check_modules(LIBCAER REQUIRED libcaer)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    pkg_check_modules(LIBSERIAL REQUIRED libserialport)
    find_package(OpenCV REQUIRED)
    
    # Ensure OpenCV include directories are used
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIBRARY_DIRS})

    # Find Eigen3 on macOS/Linux
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)

    # Find Google glog
    find_package(glog REQUIRED)

    # 🔥 Manually Set YAML-CPP Paths
    set(YAML_CPP_INCLUDE_DIR /opt/homebrew/include)
    set(YAML_CPP_LIB /opt/homebrew/lib/libyaml-cpp.dylib)

    include_directories(${YAML_CPP_INCLUDE_DIR})
endif()

# Add the library path for linking
link_directories(${LIBCAER_LIBRARY_DIRS} ${LIBUSB_LIBRARY_DIRS} ${LIBSERIAL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Target Include Directory
target_include_directories(${PROJECT_NAME} PRIVATE 
    include 
    utils/common/include
    utils/frontend/include
    utils/backend/include
    ${LIBCAER_INCLUDE_DIRS} 
    ${LIBUSB_INCLUDE_DIRS} 
    ${LIBSERIAL_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}  # Use OpenCV's detected include directories
    ${YAML_CPP_INCLUDE_DIR}
)

# Add Eigen include directory
if (WIN32)
    target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
else()
    target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIRS})
endif()

# Link required libraries
if (WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE 
        ${LIBCAER_LIBRARIES} 
        ${OpenCV_LIBRARIES} 
        ${LIBSERIAL_LIBRARIES} 
        ws2_32
        glog::glog
        ${YAML_CPP_LIB}
    )
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE 
        ${LIBCAER_LIBRARIES} 
        ${LIBUSB_LIBRARIES}
        ${LIBSERIAL_LIBRARIES} 
        ${OpenCV_LIBS}  # Correctly links OpenCV components
        ${YAML_CPP_LIB}  # ✅ Manually linked YAML-CPP
        glog::glog      # Link Google glog
    )
endif()
