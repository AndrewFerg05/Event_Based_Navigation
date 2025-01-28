# Prerequisites
cmake_minimum_required(VERSION 3.10)

# Project name
project(Event_Based_Navigation)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use all cpp files in src
file(GLOB SOURCES "src/*.cpp")

if (WIN32)
    message("Windows Device")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_TIMESPEC_DEFINED")

    # Paths to MSYS compiler installation
    set(LIBCAER_INCLUDE_DIRS "C:/msys64/mingw64/include")
    set(LIBCAER_LIBRARY_DIRS "C:/msys64/mingw64/lib")
    set(LIBCAER_LIBRARIES "libcaer")

    set(OPENCV_INCLUDE_DIRS "C:/msys64/mingw64/include/opencv4")
    set(OPENCV_LIBRARY_DIRS "C:/msys64/mingw64/lib")
    set(OPENCV_LIBRARIES opencv_core opencv_imgcodecs opencv_highgui)

    set(LIBSERIAL_INCLUDE_DIRS "C:/msys64/mingw64/include")
    set(LIBSERIAL_LIBRARY_DIRS "C:/msys64/mingw64/lib")
    set(LIBSERIAL_LIBRARIES "libserialport")
else()
    message("Mac or Linux Device")

    # Find libraries using pkg-config
    find_package(PkgConfig REQUIRED)

    pkg_check_modules(LIBCAER REQUIRED libcaer)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    pkg_check_modules(LIBSERIAL REQUIRED libserialport)

    # Find OpenCV the correct way
    find_package(OpenCV REQUIRED)
    
    # Ensure OpenCV include directories are used
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIBRARY_DIRS})
endif()

# Add the library path for linking
link_directories(${LIBCAER_LIBRARY_DIRS} ${LIBUSB_LIBRARY_DIRS} ${LIBSERIAL_LIBRARY_DIRS})

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Target Include Directory
target_include_directories(${PROJECT_NAME} PRIVATE 
    include 
    ${LIBCAER_INCLUDE_DIRS} 
    ${LIBUSB_INCLUDE_DIRS} 
    ${LIBSERIAL_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}  # Use OpenCV's detected include directories
)

if (WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBCAER_LIBRARIES} ${OPENCV_LIBRARIES} ${LIBSERIAL_LIBRARIES} ws2_32)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE 
        ${LIBCAER_LIBRARIES} 
        ${LIBUSB_LIBRARIES}
        ${LIBSERIAL_LIBRARIES} 
        ${OpenCV_LIBS}  # Correctly links OpenCV components
    )
endif()
