/*
Filename    : Software/include/TypeAliases.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 14/1/25
Description : 
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 AF created
--------------------------------------------------------------------------------
*/

#ifndef FLAGS_HPP
#define FLAGS_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <cstdint>

//==============================================================================
//      Classes
//------------------------------------------------------------------------------

// Number of events in event frame
inline size_t FLAGS_data_size_augmented_event_packet = 15000;

// IMU and Frame timestamp offset (ns)
inline int64_t FLAGS_timeshift_cam_imu = 100;       //SET FROM CALIB

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------



#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp