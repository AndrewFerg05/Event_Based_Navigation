/*
Filename    : Software/include/TypeAliases.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 14/1/25
Description : File to define type aliasing used
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 AF created
--------------------------------------------------------------------------------
*/

#ifndef TYPE_ALIASES_HPP
#define TYPE_ALIASES_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <cstdint>
#include <chrono>



//==============================================================================
//      Classes
//------------------------------------------------------------------------------

// used for testing will be replaced with actual later
using InputDataSync = uint8_t;
using TrackedFrames = uint8_t;
using OtherData = uint8_t;



// Dummy structs for data streams can change later

using CameraInfoData = uint16_t;
using ExposureData = uint16_t;

struct IMUData {
    std::chrono::nanoseconds timestamp;  // Time in nanoseconds
    struct {
        double x, y, z;
    } linear_acceleration, angular_velocity;
};

struct Header {
    uint64_t timestamp_ns;  // Equivalent to ROS Header timestamp
};

struct Event {
    uint16_t x;
    uint16_t y;
    uint64_t timestamp_ns;
    bool polarity;

    Event(uint16_t x_, uint16_t y_, uint64_t ts_, bool p_)
        : x(x_), y(y_), timestamp_ns(ts_), polarity(p_) {}
};

struct EventArray {
    Header header; 
    int width;
    int height;
    std::vector<Event> events;

    EventArray(int w, int h) : width(w), height(h) {}
};



// Dummy Image struct (similar to sensor_msgs::Image)
struct ImageData {
    std::chrono::nanoseconds timestamp;
    uint32_t width, height;
    std::vector<uint8_t> data; // Raw image data (grayscale or RGB)
};


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------






#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp