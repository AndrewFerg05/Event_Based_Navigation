/*
Filename    : Software/include/DataAcquisition.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 11/1/25
Description : Header file for the DataAcquisition (DA) thread
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
*/

#ifndef DATAACQUISITION_HPP
#define DATAACQUISITION_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include "ThreadInterface.hpp"
#include "TypeAliases.hpp"
#include "Types.hpp"
#include "Flags.hpp"
#include "RingBuffer.hpp"
#include "egg_timer.hpp"


//==============================================================================
//      Classes
//------------------------------------------------------------------------------


class DataAcquisition 
{
    // Types and Type aliases
    using ImuSyncBuffer = Ringbuffer<real_t, 6, 1000>;
    using ImuStampsVector = std::vector<ImuStamps>;
    using ImuAccGyrVector = std::vector<ImuAccGyrContainer>;

    struct ImageBufferItem
    {
      int64_t stamp    		    { -1 };
      ImagePtr img     { nullptr };
      inline bool empty()
      {
        return stamp == -1;
      }
    
      inline void reset()
      {
        stamp = -1;
        img.reset();
      }
    };
    using ImageBuffer = std::vector<ImageBufferItem>;

    using SynchronizedImageEventsImuCallback =
    std::function<void (const StampedImage&    /*image*/,
                        const StampedEventArray& /*event_arrays*/,
                        const ImuStamps& /*imu_timestamps*/,
                        const ImuAccGyrContainer& /*imu_measurements*/,
                        const bool& no_motion_prior)>;

public:

    explicit DataAcquisition(
        std::shared_ptr<DataQueues> data_queues,
        std::shared_ptr<CommunicationManager> comms);

    ~DataAcquisition();
    void start();  
    void idle();   
    void stop();
    void initBuffers();
    void addImageData(const ImageData& image_data);
    void addEventsData(const EventData& event_data);
    void addImuData(const IMUData& imu_data);
;
    void registerImuCallback(const ImuCallback& imu_callback)
    {
        imu_callback_ = imu_callback;
    }

    void registerCameraImuCallback(const SynchronizedImageEventsImuCallback& callback)
    {
        image_events_imu_callback_ = callback;
    }
 
private:
    // State  
    std::atomic<ThreadState> state_{ThreadState::Idle};
    std::thread acquisition_thread_;
    std::shared_ptr<DataQueues> input_data_queues_;
    std::shared_ptr<CommunicationManager> comms_interface_;
    std::atomic<bool> running_{false};

    // Functions
    void run();
    bool processDataQueues();
    void resetQueues();
    void extractAndEraseEvents(
        const int64_t& t1,
        int max_num_events_in_packet,
        EventBuffer* event_buffer,
        StampedEventArray* event_array);
    void checkSynch();
    bool validateImuBuffer(
        const int64_t& min_stamp,
        const int64_t& max_stamp,
        const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp);

    // Buffers
    ImuSyncBuffer imu_buffer_;
    EventBuffer event_buffer_;
    ImageBuffer image_buffer_;

    // Processing variables
    StampedImage sync_image_ready_to_process_;
    int64_t sync_img_ready_to_process_stamp_ { -1 };    // Stamp of current synchronized image bundle.
    int64_t last_img_bundle_min_stamp_ { -1 };          // Stamp of previous synchronized image bundle.
    int64_t timeshift_cam_imu_;                         // Correction between image and IMU time stamp - Set in Flags.hpp
    int sync_frame_count_  { 0 };                       // Count number of synchronized frames.

    // Allowed time differences of images in bundle
    static constexpr real_t c_camera_bundle_time_accuracy_ns = millisecToNanosec(2.0);

protected:
    // Callbacks and variables used in classes
    ImuCallback imu_callback_;
    SynchronizedImageEventsImuCallback image_events_imu_callback_;
    bool no_motion_prior_for_backend_;
};


#endif  // DATAACQUISITION_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp