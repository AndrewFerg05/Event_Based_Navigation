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
#include <Eigen/Core>
#include <Eigen/StdVector>

#include "Types.hpp"

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


// Event Data
using EventArray = std::vector<Event>;
using EventQueue = EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;
using EventBuffer = std::deque<Event>;

using real_t = double; //Can change between single and double

using Vector3 = Eigen::Matrix<real_t, 3, 1>;
using Vector6 = Eigen::Matrix<real_t, 6, 1>;

using ImuCallback =
  std::function<void (int64_t /*timestamp*/,
                      const Vector3& /*acc*/,
                      const Vector3& /*gyr*/)>;


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------






#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp