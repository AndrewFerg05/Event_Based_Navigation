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
#include <string>
//==============================================================================
//      Classes
//------------------------------------------------------------------------------

// GFlag replacements
constexpr bool VIO_CERES_VERBOSE = false;
constexpr bool VIO_CERES_MARGINALIZE = true;
constexpr int VIO_CERES_ITERATIONS = 3;
constexpr int VIO_CERES_SLIDING_WINDOW_SIZE = 3;
constexpr int VIO_CERES_NUM_KEYFRAMES = 5;
constexpr int VIO_CERES_NUM_THREADS = 1;

// Depth thresholds
constexpr double VIO_MIN_DEPTH = 0.1;  // Minimum depth for VIO
constexpr double VIO_MAX_DEPTH = 10.0; // Maximum depth for VIO
constexpr double VIO_MEDIAN_DEPTH = 1.0; // Median depth for VIO

// Feature tracking limits
constexpr uint64_t VIO_MIN_TRACKED_FEATURES_TOTAL = 50;  // Min tracked features globally
constexpr uint64_t VIO_MAX_TRACKED_FEATURES_PER_FRAME = 150; // Max features per frame

// Processing mode
constexpr bool VIO_DELAYED_NFRAME_PROCESSING = false; // Enable or disable delayed processing

inline bool FLAGS_data_use_time_interval = 0;
inline uint64_t FLAGS_data_interval_between_event_packets = 15000; //number of frames in event packet/ ms between packets set which by above flag
inline size_t FLAGS_data_size_augmented_event_packet = 15000;

constexpr int FLAGS_num_imus = 1;

constexpr bool vio_use_events = true; 
// Use events instead of images.

constexpr bool vio_use_events_and_images = false; 
// Use events and images. Ignores vio_use_events flag.

constexpr bool vio_rescale_intrinsics_for_distortion = true; 
// Whether to rescale the camera intrinsics to mitigate the loss of FoV introduced by image undistortion.

extern const std::string FLAGS_calib_filename;
extern const std::string FLAGS_mask_cam0;
extern const std::string FLAGS_mask_cam1;
extern const std::string FLAGS_mask_cam2;
extern const std::string FLAGS_mask_cam3;


constexpr uint64_t FLAGS_vio_frame_pyramid_levels = 3;

constexpr bool FLAGS_vio_use_events_and_images = 0;
constexpr bool FLAGS_vio_use_events = 1;
constexpr bool FLAGS_vio_rescale_intrinsics_for_distortion = 1;
//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------



#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp