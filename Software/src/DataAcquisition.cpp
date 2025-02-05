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
        initBuffers();
    }

DataAcquisition::~DataAcquisition() {
    stop();
}

void DataAcquisition::start() {

}

void DataAcquisition::idle() {

}

void DataAcquisition::stop() {

}

void DataAcquisition::initBuffers() {

}


void DataAcquisition::addImageData()
{

}

void DataAcquisition::addEventsData()
{

}

void DataAcquisition::addImuData()
{

}

void DataAcquisition::run()
{

}

void DataAcquisition::processDataQueues()
{

}

void DataAcquisition::resetQueues()
{

}

void DataAcquisition::extractAndEraseEvents()
{

}

void DataAcquisition::checkImuDataAndImageAndEventsCallback()
{

}







void DA_loop(std::atomic<ThreadState>& state,
                            ThreadSafeFIFO<InputDataSync>* data_DA,
                            CommunicationManager* comms) {
    
    InputDataSync valueToAdd = 0;

    while (true) {
        if (state == ThreadState::Stopped) {
            break;
        }

        if (state == ThreadState::Paused) {
            //TODO - Wait while some condition
            sleep_ms(100);
            continue;
        }

        if (state == ThreadState::Reset) {
            //TODO call reset function then set running again
            state = ThreadState::Running; 
        }

        if (state == ThreadState::Running) {
            //TODO - Get data from camera

            //Get data
            valueToAdd++;                       //For Sam architecture testing (replace with actual frames)

            //Synchronise data

            //Put in buffer
            data_DA->push(valueToAdd);   //For Sam architecture testing (replace with actual frames)
            comms->queueInputData(valueToAdd); 
            sleep_ms(25);
        }

        if (state == ThreadState::Test) {
            sleep_ms(100);
            std::cout << "Data Aquisition Testing" << std::endl; 
        }
    }
}
//==============================================================================
// End of File : Software/src/DataAcquisition.cpp
