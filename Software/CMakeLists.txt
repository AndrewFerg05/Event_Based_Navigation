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
    set(OPENNCV_LIBRARY_DIRS "C:/msys64/mingw64/lib")
    set(OPENCV_LIBRARIES opencv_core opencv_imgcodecs)
else()
    message("Mac or Linux Device")
    # Use pkg-config to find libcaer
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAER REQUIRED libcaer)
    pkg_check_modules(OPENCV REQUIRED opencv4)
endif()


# Add the library path for linking
link_directories(${LIBCAER_LIBRARY_DIRS} ${OPENCV_LIBRARY_DIRS})

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Target Include Directory
target_include_directories(${PROJECT_NAME} PRIVATE include ${LIBCAER_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})

# Link Libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBCAER_LIBRARIES} ${OPENCV_LIBRARIES}) 


