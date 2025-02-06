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
#include "RingBuffer.hpp"



//==============================================================================
//      Classes
//------------------------------------------------------------------------------


class DataAcquisition {
    using ImuSyncBuffer = RingBuffer<real_t, 6, 1000>;
public:

    explicit DataAcquisition(
        std::shared_ptr<DataQueues> data_queues,
        std::atomic<ThreadState>& state,
        std::shared_ptr<CommunicationManager> comms);

    ~DataAcquisition();
    void start();  
    void idle();   
    void stop();
    void initBuffers();
    void addImageData();
    void addEventsData(const EventData& event_data);
    void addImuData(const IMUData& imu_data);

    void registerImuCallback(const ImuCallback& imu_callback)
    {
        imu_callback_ = imu_callback;
    }
 

private:
    std::atomic<ThreadState>& state_;  // Atomic state flag
    std::thread acquisition_thread_;
    std::shared_ptr<DataQueues> input_data_queues_;
    std::shared_ptr<CommunicationManager> comms_interface_;
    bool running_;

    void run();
    bool processDataQueues();
    void resetQueues();
    void extractAndEraseEvents();
    void checkImuDataAndImageAndEventsCallback();

    ImuSyncBuffer imu_buffer_;
    EventBuffer event_buffer_;


protected:
    ImuCallback imu_callback_;

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