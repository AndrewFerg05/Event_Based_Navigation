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
//      Global Variables
//------------------------------------------------------------------------------

// Synchronisation Flags
inline size_t FLAGS_data_size_augmented_event_packet = 15000;   // Number of events in event frame
inline int64_t FLAGS_vio_frame_size = 15000;
inline float FLAGS_timeshift_cam_imu = 0.0;                     // IMU and Frame timestamp offset (s) - SET FROM CALIB
inline int32_t FLAGS_data_sync_init_skip_n_frames = 0;          // How many frames should be skipped at the beginning
inline int32_t FLAGS_noise_event_rate = 20000;                  // Minimum Number of events per second   
//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------



#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp