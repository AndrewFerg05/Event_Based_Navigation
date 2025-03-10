/*
Filename    : Software/include/Types.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 05/01/25
Description : File to define types
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 AF created
--------------------------------------------------------------------------------
*/

#ifndef TYPES_HPP
#define TYPES_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <cstdint>
#include <chrono>



//==============================================================================
//      Classes
//------------------------------------------------------------------------------

struct Header {
    uint64_t stamp;  // Equivalent to ROS Header timestamp
    std::string frame_id;   // Frame of reference
};
//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------
struct Event {
    uint16_t x;
    uint16_t y;
    uint64_t timestamp_ns;
    bool polarity;

    // Default constructor (zero-initialize fields)
    Event() : x(0), y(0), timestamp_ns(0), polarity(false) {}

    // Parameterized constructor
    Event(uint16_t x_, uint16_t y_, uint64_t ts_, bool p_)
        : x(x_), y(y_), timestamp_ns(ts_), polarity(p_) {}
};

struct EventData {
    Header header; 
    uint32_t width;
    uint32_t height;
    std::vector<Event> events;

    EventData(int w, int h) : width(w), height(h) {}
};
//------------------------------------------------------------------------------
struct ImageData {
    Header header;   // Header containing timestamp and frame_id
    uint32_t width = 0;       // Image width
    uint32_t height;      // Image height
    uint32_t step;        // Bytes per row (needed for decoding)
    uint8_t is_bigendian;
    std::string encoding;  // Image encoding format (e.g., "mono8", "rgb8")
    std::vector<uint8_t> data; // Flattened pixel data stored row-major

    // Constructor
    ImageData() = default;

    // Copy Constructor
    ImageData(const ImageData& other)
    : header(other.header), 
        width(other.width), 
        height(other.height), 
        step(other.step), 
        is_bigendian(other.is_bigendian), 
        encoding(other.encoding), 
        data(other.data)
    {}
};
//------------------------------------------------------------------------------
struct TrackedFrames {
    Header header;   // Header containing timestamp and frame_id
    uint32_t width = 0;       // Image width
    uint32_t height;      // Image height
    uint32_t step;        // Bytes per row (needed for decoding)
    uint8_t is_bigendian;
    std::string encoding;  // Image encoding format (e.g., "mono8", "rgb8")
    std::vector<uint8_t> data; // Flattened pixel data stored row-major

    // Constructor
    TrackedFrames() = default;
};

//------------------------------------------------------------------------------
enum FrameType {
    REGULAR_FRAME,
    EVENT_FRAME,
    COMBINED_FRAME
};


#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp