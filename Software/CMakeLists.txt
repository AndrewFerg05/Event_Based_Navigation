cmake_minimum_required(VERSION 3.10)

message("${CMAKE_SYSTEM_NAME}")

# Project name
project(Event_Based_Navigation)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(
    include
    src
    lib
)

# Source files
file(GLOB_RECURSE SOURCES "src/*.cpp" "lib/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.hpp" "lib/*.hpp")

# Detect platform and apply settings accordingly
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    message(STATUS "Configuring for macOS")
    # macOS specific configurations
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
    # Add any macOS specific libraries if needed here

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    message(STATUS "Configuring for Windows")
    # Windows specific configurations
    if(MSVC)
        # Using Microsoft Visual Studio Compiler
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    else()
        # Using GCC or Clang on Windows (e.g., MinGW)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
    endif()
    # Add any Windows specific libraries if needed here

else()
    message(STATUS "Unknown platform: ${CMAKE_SYSTEM_NAME}")
    # Add any generic or platform-independent settings here if needed
endif()

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
