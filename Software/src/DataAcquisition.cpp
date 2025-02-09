/*
Filename    : Software/src/DataAcquisition.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Data Acquisition code for getting DAVIS346 camera data
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
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


DataAcquisition::DataAcquisition(std::shared_ptr<DataQueues> data_queues, std::atomic<ThreadState>& state, std::shared_ptr<CommunicationManager> comms)
    : input_data_queues_(data_queues), state_(state), comms_interface_(comms) 
    {
        cur_ev_ = 0;
        last_package_stamp_ = -1;
        last_package_ev_ = 0;
        CHECK_GE(FLAGS_data_size_augmented_event_packet, 0) <<
        "DA: data_size_augmented_event_packet should be positive.";
        initBuffers();
    }

DataAcquisition::~DataAcquisition() {
    stop();
}

void DataAcquisition::start() {
    if (acquisition_thread_.joinable()) return;  // Prevent multiple starts
    running_ = true;
    state_ = ThreadState::Run;
    acquisition_thread_ = std::thread(&DataAcquisition::run, this);
}

void DataAcquisition::idle() {
    LOG(INFO) << "DA: Resetting queues and entering idle mode...";
    resetQueues();
    imu_buffer_.clear();
    event_buffer_.clear();
    event_packages_ready_to_process_.clear();
    cur_ev_ = 0;
    last_package_stamp_ = -1;
    last_package_ev_ = 0;
    state_ = ThreadState::Idle;
}

void DataAcquisition::stop() {
    running_ = false;
    state_ = ThreadState::Stop;
    if (acquisition_thread_.joinable()) {
        acquisition_thread_.join();
    }
    LOG(INFO) << "DA: Thread stopped";
}

void DataAcquisition::initBuffers() 
{
    imu_buffer_.clear();
    event_buffer_.clear(); 
}


void DataAcquisition::addImageData()
{

}


void DataAcquisition::onlyEventsNoImagesLogic()
{
    int64_t interval_between_packets;

    if(FLAGS_data_use_time_interval)
        interval_between_packets = FLAGS_data_interval_between_event_packets * 1e3;
    else
        interval_between_packets = FLAGS_data_interval_between_event_packets;

    const size_t package_size = FLAGS_data_size_augmented_event_packet;

    for(; cur_ev_ < event_buffer_.size(); cur_ev_++)
    {
        const Event& e = event_buffer_[cur_ev_];
        const int64_t cur_stamp = e.timestamp_ns;

        bool create_packet = (!FLAGS_data_use_time_interval &&
            (cur_ev_ - last_package_ev_ >= (size_t) interval_between_packets)) ||
            (FLAGS_data_use_time_interval &&
            (cur_stamp - last_package_stamp_ > interval_between_packets));

            if(create_packet)
            {
                const int64_t package_stamp = cur_stamp;

                VLOG(100) << "DA: Creating event package from " << last_package_stamp_
                << " to " << cur_stamp
                << " with stamp: " << package_stamp;

                size_t ev = cur_ev_ > package_size ?
                            cur_ev_ - package_size : 0;

                // Build the actual event package
                VLOG(100) << "DA: Augmented event package: " << event_buffer_[ev].timestamp_ns
                << " to " << cur_stamp
                << " with " << cur_ev_ - ev + 1 << " events";

                EventArrayPtr event_array_ptr = std::make_shared<EventArray>();
                for(; ev <= cur_ev_ ; ev++)
                    event_array_ptr->push_back(event_buffer_[ev]);

                event_packages_ready_to_process_.push_back(StampedEventArray(package_stamp, event_array_ptr));
                
                // Prepare to build a new event package
                last_package_stamp_ = cur_stamp;
                last_package_ev_ = cur_ev_;
            }
    }
    

}

void DataAcquisition::addEventsData(const EventData& event_data)
{
    if(event_data.events.empty())
        return;

    EventArrayPtr events = std::make_shared<EventArray>();

    for(auto& e : event_data.events)
        events->push_back(e);

    VLOG(1000) << event_data.events[0].timestamp_ns;

    event_buffer_.insert(event_buffer_.end(), events->begin(), events->end());

    onlyEventsNoImagesLogic();
    checkImuDataAndCallback();

    static size_t event_history_size_ = 500000;
    if (cur_ev_ > event_history_size_)
    {
      size_t remove_events = cur_ev_ - event_history_size_;

      event_buffer_.erase(event_buffer_.begin(), event_buffer_.begin()
                          + remove_events);
      cur_ev_ -= remove_events;
      last_package_ev_ -= remove_events;

      VLOG(10) << "DA: Removed " << remove_events << " events from the DVS buffer";
    }

    VLOG(1000) << "DA: num events added to the queue: " << events->size();
    VLOG(1000) << "DA: total num events: " << event_buffer_.size();
}

void DataAcquisition::addImuData(const IMUData& imu_data)
{
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
    //stamp -= timeshift_cam_imu_;
    imu_buffer_[0].insert(stamp, acc_gyr);

    if (imu_callback_)
    {
        imu_callback_(stamp, acc, gyr,0);
    }

    checkImuDataAndCallback();
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
            addImageData();
            comms_interface_->queueImage(image_data.value());
            processed = true;
        }
        return processed;

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

void DataAcquisition::extractAndEraseEvents()
{

}

void DataAcquisition::checkImuDataAndCallback()
{
    if (event_packages_ready_to_process_.empty())
    {
      return; // No event package to process yet
    }

    std::vector<bool> discard_event_packet(event_packages_ready_to_process_.size(), false);

    for(size_t i = 0; i < event_packages_ready_to_process_.size(); ++i)
    {
        const StampedEventArray& event_package = event_packages_ready_to_process_[i];
        const int64_t event_package_stamp = event_package.first;

        VLOG(1000) << "Current event package stamp:" << event_package_stamp;

        ImuStampsVector imu_timestamps(1);
        ImuAccGyrVector imu_measurements(1);

        std::vector<std::tuple<int64_t, int64_t, bool>> oldest_newest_stamp_vector(1);

        std::transform(
            imu_buffer_.begin(),
            imu_buffer_.end(),
            oldest_newest_stamp_vector.begin(),
            [](const ImuSyncBuffer& imu_buffer) {
            return imu_buffer.getOldestAndNewestStamp();
        });

        const int64_t oldest_imu_stamp = std::get<0>(oldest_newest_stamp_vector[0]);
        const int64_t newest_imu_stamp = std::get<1>(oldest_newest_stamp_vector[0]);

        VLOG(1000) << "DA: Oldest IMU stamp: " << oldest_imu_stamp;
        VLOG(1000) << "DA: Newest IMU stamp: " << newest_imu_stamp;

        if (!validateImuBuffers(
                event_package_stamp,
                event_package_stamp,
                oldest_newest_stamp_vector))
        {
            if(oldest_imu_stamp >= event_package_stamp)
            {
              // Oldest IMU measurement is newer than image timestamp
              // This will happen only at the very beginning, thus
              // it is safe to simply discard the event package
              VLOG(1000) << "DA: Discarding event packet at stamp:" << event_package_stamp;
              discard_event_packet[i] = true;
            }
        }
        else
        {
            // We have enough IMU measurements to broadcast the current packet
            VLOG(1000) << "DA: last_event_package_broadcast_stamp: " << last_event_package_broadcast_stamp_;
            
            for (size_t i = 0; i < 1; ++i)
            {
                if(last_event_package_broadcast_stamp_ < 0)
                {
                int64_t oldest_stamp = std::get<0>(oldest_newest_stamp_vector[i]);
                std::tie(imu_timestamps[i], imu_measurements[i]) =
                    imu_buffer_[i].getBetweenValuesInterpolated(
                        oldest_stamp,
                        event_package_stamp);
                }
                else
                {
                std::tie(imu_timestamps[i], imu_measurements[i]) =
                    imu_buffer_[i].getBetweenValuesInterpolated(
                        //event_package.second->at(0).ts.toNSec(), <-- Should be this...
                        last_event_package_broadcast_stamp_,
                        event_package_stamp);
                }
            }
    
          events_imu_callback_(event_package, imu_timestamps, imu_measurements);
    
          // Discard the event package, now that it's been processed
          discard_event_packet[i] = true;
    
          last_event_package_broadcast_stamp_ = event_package_stamp;
        }
        
    }

    // Discard all the event packages that have been marked
    StampedEventArrays event_packages_ready_to_process_new;
    for(size_t i=0; i<event_packages_ready_to_process_.size(); ++i)
    {
        if(!discard_event_packet[i])
        {
            event_packages_ready_to_process_new.push_back(event_packages_ready_to_process_[i]);
        }
    }
    event_packages_ready_to_process_ = event_packages_ready_to_process_new;
}


bool DataAcquisition::validateImuBuffers(
    const int64_t& min_stamp,
    const int64_t& max_stamp,
    const std::vector<std::tuple<int64_t, int64_t, bool> >&
    oldest_newest_stamp_vector)
{
    // Check if we have received some IMU measurements for at least one of the imu's.
    if (std::none_of(oldest_newest_stamp_vector.begin(),
                    oldest_newest_stamp_vector.end(),
                    [](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
                    {
                        if (std::get<2>(oldest_newest_stamp))
                        {
                            return true;
                        }
                            return false;
                    }))        
    {
    return false;
    }

    // At least one IMU measurements before image
    if (std::none_of(oldest_newest_stamp_vector.begin(),
                    oldest_newest_stamp_vector.end(),
                    [&min_stamp](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
                    {
                        if (std::get<0>(oldest_newest_stamp) < min_stamp) {
                            return true;
                        }
                    return false;
                    }))
    {
    return false;
    }

    // At least one IMU measurements after image
    if (std::none_of(oldest_newest_stamp_vector.begin(),
                    oldest_newest_stamp_vector.end(),
                    [&max_stamp](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp) 
                    {
                        if (std::get<1>(oldest_newest_stamp) > max_stamp)
                        {
                            return true;
                        }
                    return false;
                    }))
    {
        return false;
    }
    
    return true;
}

void DataAcquisition::run() 
{
    while (running_) 
    {
        if (state_ == ThreadState::Idle) 
        {
            sleep_ms(10);  // Avoid busy waiting
            continue;
        }

        if (state_ == ThreadState::Stop) 
        {
            running_ = false;
            break;
        }

        if (state_ ==ThreadState::Run)
        {
            if (!processDataQueues()) 
            {
                sleep_ms(1);  // No data, sleep briefly
            }

        }
    }
}


//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
