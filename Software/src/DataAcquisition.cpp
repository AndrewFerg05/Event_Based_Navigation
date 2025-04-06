/*
Filename    : Software/src/DataAcquisition.cpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 11/1/25
Description : Data Acquisition code for getting DAVIS346 camera data
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------

// Local
#include "DataAcquisition.hpp"

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------

//==============================================================================
// MACROs
//------------------------------------------------------------------------------

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------


//==============================================================================
// Functions
//------------------------------------------------------------------------------

DataAcquisition::DataAcquisition(std::shared_ptr<DataQueues> data_queues)
    : input_data_queues_(data_queues), timeshift_cam_imu_(secToNanosec(FLAGS_timeshift_cam_imu)) 
    {

        CHECK_GE(FLAGS_data_size_augmented_event_packet, 0) <<
        "DA: data_size_augmented_event_packet should be positive.";
        initBuffers();
    }

DataAcquisition::~DataAcquisition() {
    stop();
}

void DataAcquisition::start() {
    LOG(INFO) << "DA: Starting thread..";

    initBuffers();      // Reset buffer again for safety

    running_ = true;
    state_ = ThreadState::Run;

    // Start thread
    // if (acquisition_thread_.joinable())
    // {
    //     LOG(INFO) << "DA: Thread already running!";
    //     return;
    // }
    // acquisition_thread_ = std::thread(&DataAcquisition::run, this);
    // LOG(INFO) << "DA: Thread Started!";
    run();
}

void DataAcquisition::idle() {
    LOG(INFO) << "DA: Idling thread...";
    state_ = ThreadState::Idle;

    // Reset variable and queues
    sync_img_ready_to_process_stamp_ = -1;
    last_img_bundle_min_stamp_ = -1;
    sync_image_ready_to_process_.first = -1;
    sync_image_ready_to_process_.second.reset();
    sync_frame_count_ = 0;
    resetQueues();
    initBuffers();

    LOG(INFO) << "DA: Thread idled!";
}

void DataAcquisition::stop() {
    LOG(INFO) << "DA: Stopping thread...";
    running_ = false;
    state_ = ThreadState::Stop;

    // Reset variable and queues
    sync_img_ready_to_process_stamp_ = -1;
    last_img_bundle_min_stamp_ = -1;
    sync_image_ready_to_process_.first = -1;
    sync_image_ready_to_process_.second.reset();
    sync_frame_count_ = 0;
    resetQueues();
    initBuffers();

    // Exit thread
    if (acquisition_thread_.joinable()) 
    {
        acquisition_thread_.join();
    }
    LOG(INFO) << "DA: Thread stopped!";
}

void DataAcquisition::initBuffers() 
{
    imu_buffer_.clear();
    event_buffer_.clear();
    image_buffer_.clear();
    image_buffer_.resize(2); 
}

void DataAcquisition::resetQueues()
{
    if (!input_data_queues_) {
        LOG(ERROR) << "DA: Data queues not initialized!";
        return;
    }

    LOG(INFO) << "DA: Resetting all data queues...";
    
    input_data_queues_->event_queue->clear();
    input_data_queues_->camera_info_queue->clear();
    input_data_queues_->imu_queue->clear();
    input_data_queues_->image_queue->clear();
    input_data_queues_->exposure_queue->clear();

    LOG(INFO) << "DA: All queues cleared";
}

void DataAcquisition::addImageData(const ImageData& image_data)
{
    // Extract deep copy of image_data
    ImagePtr img = std::make_shared<ImageData>(image_data);
    int64_t stamp = image_data.header.stamp;

    // Correct for timestamp delay between IMU and Frames.
    // stamp += timeshift_cam_imu_;

    //Skip the first N frames
    if (sync_frame_count_ < FLAGS_data_sync_init_skip_n_frames)
    {
        ++sync_frame_count_;

      return;
    }

    // Add image to first available slot in our buffer:
    int slot = -1;
    for (size_t i = 0u; i < image_buffer_.size(); ++i)
    {
        if (image_buffer_[i].empty())
        {
        slot = i;
        break;
        }
    }

    if (slot == -1)
    {
        // No space in buffer to process frame. Delete oldest one. This happens
        // also when the processing is not fast enough such that frames are skipped.
        int64_t min_stamp = std::numeric_limits<int64_t>::max();
        for (size_t i = 0u; i < image_buffer_.size(); ++i)
        {
        if (!image_buffer_[i].empty() && image_buffer_[i].stamp < min_stamp)
        {
            slot = i;
            min_stamp = image_buffer_[i].stamp;
        }
        }
    }

    image_buffer_[slot].stamp = stamp;
    image_buffer_[slot].img = img;

    // Find the latest image in the buffer
    int latest_index = -1;
    int64_t latest_stamp = std::numeric_limits<int64_t>::min();
    for (size_t i = 0; i < image_buffer_.size(); ++i)
    {
        if (!image_buffer_[i].empty() && image_buffer_[i].stamp > latest_stamp)
        {
            latest_index = i;
            latest_stamp = image_buffer_[i].stamp;
        }
    }
  
      // Ensure a valid image was found
      if (latest_index == -1)
      {
          LOG(ERROR) << "No valid images in buffer!";
          return;
      }
  
      // Retrieve and store the latest image for processing
      const ImageBufferItem& latest_image = image_buffer_[latest_index];
      
      DEBUG_CHECK_GT(latest_image.stamp, 0);
      DEBUG_CHECK(latest_image.img);
  
      sync_image_ready_to_process_ = { latest_image.stamp, latest_image.img };
      sync_img_ready_to_process_stamp_ = latest_image.stamp;
      checkSynch();
  
}

void DataAcquisition::addEventsData(const EventData& event_data)
{
    // If no events in packet return
    if(event_data.events.empty())
        return;

    // Add events to array
    EventArrayPtr events = std::make_shared<EventArray>();
    for(auto& e : event_data.events)
        events->push_back(e);

    VLOG(1000) << event_data.events[0].timestamp_ns;

    // Store event packet info in queue
    event_buffer_.insert(event_buffer_.end(), events->begin(), events->end());

    checkSynch();

}

void DataAcquisition::addImuData(const IMUData& imu_data)
{
    // Extract IMU data from queue
    const Vector3 gyr(
    imu_data.angular_velocity.x,
    imu_data.angular_velocity.y,
    imu_data.angular_velocity.z);
    const Vector3 acc(
    imu_data.linear_acceleration.x,
    imu_data.linear_acceleration.y,
    imu_data.linear_acceleration.z);
    int64_t stamp = imu_data.header.stamp;

    Vector6 acc_gyr;
    acc_gyr.head<3>() = acc;
    acc_gyr.tail<3>() = gyr;

    // Add to circular buffer
    imu_buffer_.insert(stamp, acc_gyr);

    if (imu_callback_)
    {
        // Add IMU data to VIO frontend directly
        imu_callback_(stamp, acc, gyr);
    }

    // Check if we have enough data
    checkSynch();
}

void DataAcquisition::extractAndEraseEvents(
    const int64_t& t1,
    int max_num_events_in_packet,
    EventBuffer* event_buffer,
    StampedEventArray* event_array)
{
    CHECK_NOTNULL(event_buffer);
    CHECK_NOTNULL(event_array);

    event_array->second = std::make_shared<EventArray>();

      // Check that the event buffer is not empty.
  if (event_buffer->empty()) {
    LOG(WARNING) << "Event buffer is empty.";
    // Fill with empty events.
    event_array->first = t1;
    CHECK(event_array->second->empty()) << "Event array vector should be empty";
    return;
  }

    // Fill the event array with a fixed number of events.
    event_array->second->resize((uint32_t)max_num_events_in_packet);
    size_t num_stored_events = 0;

    for (int i = event_buffer->size() - 1; i >= 0; i--) {
        const Event& event (event_buffer->at(i));
        const int64_t& cur_stamp (event.timestamp_ns);
        // Don't use the most recent events as they will be used in the next call.
        // Can be improved by doing a binary search instead...
        if (cur_stamp > t1) 
        {
            continue;
        }

        // Store all events that come before t1 until we reach the limit of events
        // in array or there are no more events in buffer.
        if (num_stored_events < (uint32_t)max_num_events_in_packet && i != 0) 
        {
            // Store them in order, first the oldest event, last the most recent.
            // So no push back.
            event_array->second->at((uint32_t)max_num_events_in_packet - 1 - num_stored_events) =
                event;
            num_stored_events++;
        }

        else 
        {
            // If we got here because there are no more events in the buffer, but
            // we did not reach the maximum size of events in the event array,
            // add the last event and skip.
            if (num_stored_events == (uint32_t)max_num_events_in_packet) 
            {
                if (i == 0) 
                {
                    // Do nothing, buffer empty, and event_array full.
                    break;
                } 
                else 
                {
                    // Buffer still with old values, event_array full.
                    // Clean old values.
                    event_buffer->erase(event_buffer->begin(), event_buffer->begin() + i);
                }
            } 
            else 
            {
                // Here i has to be equal to 0.
                // Therefore, the buffer is empty, but we did not completely
                // fill the event_array.
                event_array->second->at((uint32_t)max_num_events_in_packet - 1 - num_stored_events) = event;
                num_stored_events++;
                // Clear non-used space.
                event_array->second->erase(event_array->second->begin(),
                                          event_array->second->end()
                                          - num_stored_events);
            }
            break;
          }
    }

    // Use the last timestamp as stamp for the event_array.
    event_array->first = t1;
}

bool DataAcquisition::validateImuBuffer(
    const int64_t& min_stamp,
    const int64_t& max_stamp,
    const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
{
    // Check if we have received IMU measurements
    if (!std::get<2>(oldest_newest_stamp))
    {
        LOG(WARNING) << "Received all images but no IMU measurements!";
        return false;
    }

    // Check if at least one IMU measurement exists before the image timestamp
    if (std::get<0>(oldest_newest_stamp) >= min_stamp)
    {
        LOG(WARNING) << "Oldest IMU measurement is newer than image timestamp.";
        return false;
    }

    // Check if at least one IMU measurement exists after the image timestamp
    if (std::get<1>(oldest_newest_stamp) <= max_stamp)
    {
        VLOG(100) << "Waiting for IMU measurements.";
        return false;
    }

    return true;
}

void DataAcquisition::checkSynch()
{
    if (sync_img_ready_to_process_stamp_ < 0)
    {
      return; // Images are not synced yet.
    }

    // always provide imu structures in the callback (empty if no imu present)
    ImuStamps imu_timestamp;
    ImuAccGyrContainer imu_measurement;

    // Store oldest and newest IMU stamps
    std::tuple<int64_t, int64_t, bool> oldest_newest_stamp = imu_buffer_.getOldestAndNewestStamp();

    // imu buffers are not consistent with the image buffers
    // or the imu buffer is not filled until the image stamp, return.
    // This ensures we collect all IMU data before timestamp of the image.
    if (!validateImuBuffer(sync_img_ready_to_process_stamp_,
        sync_img_ready_to_process_stamp_,
        oldest_newest_stamp))
    {
    return;
    }

    // If this is the very first image bundle, we send all IMU messages that we have
    // received so far. For every later image bundle, we just send the IMU messages
    // that we have received in between.
    if (last_img_bundle_min_stamp_ < 0)
    {
        int64_t oldest_stamp = std::get<0>(oldest_newest_stamp);
        std::tie(imu_timestamp, imu_measurement) =
            imu_buffer_.getBetweenValuesInterpolated(
                oldest_stamp,
                sync_img_ready_to_process_stamp_);
    }
    else
    {
        std::tie(imu_timestamp, imu_measurement) =
            imu_buffer_.getBetweenValuesInterpolated(
                last_img_bundle_min_stamp_,
                sync_img_ready_to_process_stamp_);
    }

    // We should do this only when we come from the addEventsData...
    // Check that we have all events until the timestamp given by the image.
    // If not return, and wait until the event buffer is filled until this
    // timestamp.
    StampedEventArray event_array;
    static constexpr uint64_t kCollectEventsTimeoutNs = 10000u;
    static EggTimer timer (kCollectEventsTimeoutNs);
    static bool start_timer = true;
    const int64_t& last_event_timestamp = static_cast<int64_t>(event_buffer_.back().timestamp_ns);

    if(sync_img_ready_to_process_stamp_ > last_event_timestamp) 
    {
        if (start_timer) 
        {
            start_timer = false;
            timer.reset();
        }
    
        if (!timer.finished()) 
        {
            VLOG(2) << "Trying to collect events until image timestamp.\n"
            << "Current image timestamp is: "
            << sync_img_ready_to_process_stamp_ << '\n'
            << "Last event timestamp is: "
            << static_cast<int64_t>(event_buffer_.back().timestamp_ns) << '\n'
            << "Image timestamp - Last event timestamp = "
            << sync_img_ready_to_process_stamp_ - last_event_timestamp;
            return;
        } 
        else 
        {
            VLOG(2) << "Collect Events Timeout reached: "
                    << "processing current packet.";
            start_timer = true;
        }
    } 
    else 
    {
        start_timer = true;
    }

    // Get Events data between frames, similar to IMU sync above.
    int64_t t1 = sync_img_ready_to_process_stamp_;
    extractAndEraseEvents(t1, FLAGS_data_size_augmented_event_packet,
                            &event_buffer_, &event_array);
    
    // Let's process the callback.
    if(image_events_imu_callback_)
    {
        image_events_imu_callback_(sync_image_ready_to_process_, event_array,
            imu_timestamp, imu_measurement,
            no_motion_prior_for_backend_);
    }
        
    // Reset Buffer:
    for (size_t i = 0; i <= image_buffer_.size(); ++i)
    {
        ImageBufferItem& item = image_buffer_[i];
        if (std::abs(sync_img_ready_to_process_stamp_ - item.stamp)
            < c_camera_bundle_time_accuracy_ns)
        {
            item.reset();
        }
    }

    last_img_bundle_min_stamp_ = sync_img_ready_to_process_stamp_;
    sync_img_ready_to_process_stamp_ = -1;
    sync_image_ready_to_process_.first = -1;
    sync_image_ready_to_process_.second.reset();
}

bool DataAcquisition::processDataQueues()
{
        bool processed = false;

        auto imu_data = input_data_queues_->imu_queue->pop();
        if (imu_data.has_value()) {
            addImuData(imu_data.value());
            processed = true;
        }

        auto event_data = input_data_queues_->event_queue->pop();
        if (event_data.has_value()) {
            addEventsData(event_data.value());
            processed = true;
        }

        auto image_data = input_data_queues_->image_queue->pop();
        if (image_data) {
            addImageData(image_data.value());
            processed = true;
        }

        return processed;

}

void DataAcquisition::run() 
{
    while (running_) {
        switch (state_) {
            case ThreadState::Idle:
                sleep_ms(100);
                continue;

            case ThreadState::Run:
                if (!processDataQueues()) 
                {
                    // LOG(INFO) << "DA: No data in input queues - sleeping 100ms";
                    sleep_ms(100);
                }
                break;

            case ThreadState::Stop:
                return;
        }
    }
}


//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
