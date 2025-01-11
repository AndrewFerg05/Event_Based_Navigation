#Prerequisites
cmake_minimum_required(VERSION 3.10)

# Project name
project(Event_Based_Navigation)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use all cpp files in src
file(GLOB SOURCES "src/*.cpp")

# Find libcaer library
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAER REQUIRED libcaer)

# Add the library path for linking
link_directories(${LIBCAER_LIBRARY_DIRS})

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Target Include Directory
target_include_directories(${PROJECT_NAME} PRIVATE include ${LIBCAER_INCLUDE_DIRS})

# Link Libraries
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBCAER_LIBRARIES})


