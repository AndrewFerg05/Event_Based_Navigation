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

inline bool FLAGS_data_use_time_interval = 0;
inline uint64_t FLAGS_data_interval_between_event_packets = 10000; //number of frames in event packet/ ms between packets set which by above flag
inline size_t FLAGS_data_size_augmented_event_packet = 1000;


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------



#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp