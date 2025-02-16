/*
Filename    : Software/include/DataAcquisition.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Header file for the DataAcquisition (DA) thread
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
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
#include <opencv2/opencv.hpp>


//==============================================================================
//      Classes
//------------------------------------------------------------------------------


class DataAcquisition {
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


    using SynchronizedEventsImuCallback =
            std::function<void (const StampedEventArray& /*event_arrays*/,
                      const ImuStampsVector& /*imu_timestamps*/,
                      const ImuAccGyrVector& /*imu_measurements*/)>;


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

    void registerCameraImuCallback(const SynchronizedEventsImuCallback& callback)
    {
        events_imu_callback_ = callback;
    }
 

private:
    std::atomic<ThreadState> state_{ThreadState::Idle};  // Atomic state flag
    std::thread acquisition_thread_;
    std::shared_ptr<DataQueues> input_data_queues_;
    std::shared_ptr<CommunicationManager> comms_interface_;
    bool running_;

    void run();
    bool processDataQueues();
    void resetQueues();
    void extractAndEraseEvents();
    void checkSynch();
    bool validateImuBuffers(
        const int64_t& min_stamp,
        const int64_t& max_stamp,
        const std::vector<std::tuple<int64_t, int64_t, bool> >&
        oldest_newest_stamp_vector);

    ImuSyncBuffer imu_buffer_;
    EventBuffer event_buffer_;
    ImageBuffer image_buffer_;

    StampedEventArrays event_packages_ready_to_process_;

    StampedImage sync_image_ready_to_process_;
    int64_t sync_img_ready_to_process_stamp_ { -1 };

    size_t cur_ev_; // index of the last event processed yet

    int64_t last_package_stamp_;
    size_t last_package_ev_;
  
    int64_t last_event_package_broadcast_stamp_ { -1 };

    int64_t timeshift_cam_imu_;

    //! Count number of synchronized frames.
    int sync_frame_count_  { 0 };



protected:
    ImuCallback imu_callback_;
    SynchronizedEventsImuCallback events_imu_callback_;

};



//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
void DA_loop(std::atomic<ThreadState>& state,
                            ThreadSafeFIFO<InputDataSync>* data_DA,
                            CommunicationManager* comms);




#endif  // DATAACQUISITION_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp