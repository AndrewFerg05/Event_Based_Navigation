/*
Filename    : Software/include/Flags.hpp
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
inline bool VIO_CERES_VERBOSE = false;
inline bool VIO_CERES_MARGINALIZE = true;
inline int VIO_CERES_ITERATIONS = 3;
inline int VIO_CERES_SLIDING_WINDOW_SIZE = 3;
inline int VIO_CERES_NUM_KEYFRAMES = 5;
inline int VIO_CERES_NUM_THREADS = 1;

// Depth thresholds
inline double VIO_MIN_DEPTH = 0.1;  // Minimum depth for VIO
inline double VIO_MAX_DEPTH = 10.0; // Maximum depth for VIO
inline double VIO_MEDIAN_DEPTH = 1.0; // Median depth for VIO

// Feature tracking limits
inline uint64_t VIO_MIN_TRACKED_FEATURES_TOTAL = 50;  // Min tracked features globally
inline uint64_t VIO_MAX_TRACKED_FEATURES_PER_FRAME = 150; // Max features per frame

// Processing mode
inline bool VIO_DELAYED_NFRAME_PROCESSING = false; // Enable or disable delayed processing

inline bool FLAGS_data_use_time_interval = 0;
inline uint64_t FLAGS_data_interval_between_event_packets = 15000;
inline size_t FLAGS_data_size_augmented_event_packet = 15000;

inline int FLAGS_num_imus = 1;

inline bool vio_use_events = true; 
// Use events instead of images.

inline bool vio_use_events_and_images = false; 
// Use events and images. Ignores vio_use_events flag.

inline bool vio_rescale_intrinsics_for_distortion = true; 
// Whether to rescale the camera intrinsics to mitigate the loss of FoV introduced by image undistortion.

extern const std::string FLAGS_calib_filename;
extern const std::string FLAGS_mask_cam0;
extern const std::string FLAGS_mask_cam1;
extern const std::string FLAGS_mask_cam2;
extern const std::string FLAGS_mask_cam3;


inline uint64_t FLAGS_vio_frame_pyramid_levels = 3;

inline bool FLAGS_vio_use_events_and_images = 0;
inline bool FLAGS_vio_use_events = 1;
inline bool FLAGS_vio_rescale_intrinsics_for_distortion = 1;
inline bool FLAGS_vio_activate_backend = 1;


inline double FLAGS_vio_landmark_triangulate_purge_floor = -1;
inline double FLAGS_noise_event_rate = 20000;

inline uint64_t FLAGS_vio_frame_size = 15000;

inline bool FLAGS_vio_do_motion_correction = 1;
inline double FLAGS_vio_frame_norm_factor = 3.0;


inline uint32_t FLAGS_vio_add_every_nth_frame_to_backend = -1; 

extern const std::string FLAGS_vio_feature_tracker_patch_size_by8;
inline double FLAGS_vio_feature_tracker_termcrit_min_update_squared = 0.03;

inline double FLAGS_vio_disparity_median_for_static_motion_classification = 1.75;

inline uint64_t FLAGS_vio_max_landmarks = 1000;

inline double FLAGS_vio_min_depth = 0.1;
inline uint32_t FLAGS_vio_max_depth = 10;
inline uint32_t FLAGS_vio_median_depth = 1;

// Keyframe Selection
inline uint32_t FLAGS_vio_first_n_frames_as_keyframes = 0;
inline uint32_t FLAGS_vio_kfselect_min_every_nth_frame = -1;
inline uint32_t FLAGS_vio_kfselect_every_nth_frame = -1;
inline uint32_t FLAGS_vio_kfselect_criterion = 1;
inline uint64_t FLAGS_vio_kfselect_numfts_upper_thresh = 180;
inline uint64_t FLAGS_vio_kfselect_numfts_lower_thresh = 90;
inline uint32_t FLAGS_vio_kfselect_min_num_frames_between_kfs = 0;
inline double FLAGS_vio_kfselect_min_disparity = 30.0;
inline double FLAGS_vio_kfselect_min_dist = 0.12;


// Landmark Reprojector
inline bool FLAGS_vio_reprojector_pyr_alignment = true;
inline bool FLAGS_vio_reprojector_limit_num_features = true;
inline uint32_t FLAGS_vio_reprojector_min_quality_to_project = 2;

// Feature detector
inline bool FLAGS_vio_descriptor_use_dummy = false;
extern const std::string FLAGS_imp_detector_name;
inline uint32_t FLAGS_imp_detector_grid_size = 32;
inline uint32_t FLAGS_imp_detector_border_margin = 8;
inline uint32_t FLAGS_imp_detector_num_octaves = 3;
inline uint32_t FLAGS_imp_detector_max_features_per_frame = 300;
inline uint32_t FLAGS_vio_max_tracked_features_per_frame = 300;
inline uint32_t FLAGS_imp_brisk_uniformity_radius = 0;
inline uint32_t FLAGS_imp_brisk_absolute_threshold = 45;
inline uint32_t FLAGS_imp_detector_threshold = 10;

inline uint32_t FLAGS_vio_min_tracked_features_total = 50; // Guess

// Track Extractor
// Max number of landmarks to add to back-end optimization per keyframe.
inline uint32_t FLAGS_vio_max_num_tracks_per_update = 50;
// Number of observations in feature-track to classify landmark as persistent
inline uint32_t FLAGS_vio_num_obs_for_persistent = 10;
// Minimum parallax threshold to use a landmark in the back-end optimization
inline double FLAGS_vio_min_parallax_deg = 0.5;
// Favor an equal number of new persistent tracks in every frame.
inline bool FLAGS_vio_favor_equal_number_of_tracks_per_frame = false;
inline bool FLAGS_vio_delayed_nframe_processing = 0; // Guess and might change

// Relative pose RANSAC reprojection error threshold [px].
inline double FLAGS_vio_ransac_relpose_thresh_px = 2;
inline bool FLAGS_vio_use_5pt_ransac = false;



#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp