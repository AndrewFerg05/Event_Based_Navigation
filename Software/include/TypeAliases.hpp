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
using ImuCallback = uint16_t;


struct Header {
    uint64_t stamp;  // Equivalent to ROS Header timestamp
    std::string frame_id;   // Frame of reference
};

struct IMUData {
    Header header; // Timestamp and frame information

    struct {
        double x, y, z; // Linear acceleration in m/s^2
    } linear_acceleration;

    struct {
        double x, y, z; // Angular velocity in rad/s
    } angular_velocity;

    std::array<double, 9> orientation_covariance; // Orientation covariance (no orientation estimation by default)

    IMUData() {
        orientation_covariance.fill(0.0);
        orientation_covariance[0] = -1.0; // No orientation estimation
    }
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
    uint32_t width;
    uint32_t height;
    std::vector<Event> events;

    EventArray(int w, int h) : width(w), height(h) {}
};


struct ImageData {
    Header header;   // Header containing timestamp and frame_id
    uint32_t width;       // Image width
    uint32_t height;      // Image height
    uint32_t step;        // Bytes per row (needed for decoding)
    uint8_t is_bigendian;
    std::string encoding;  // Image encoding format (e.g., "mono8", "rgb8")
    std::vector<uint8_t> data; // Flattened pixel data stored row-major

    // Constructor
    ImageData() = default;
};


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------






#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp