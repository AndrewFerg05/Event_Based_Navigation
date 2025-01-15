/*
Filename    : Software/src/ThreadInterface.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 10/1/25
Description : Multithreading in the project
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
10-JAN-2025 SARK created to design code structure
11-JAN-2025 SARK added thread interfaces
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------

// Local
#include "ThreadInterface.hpp"

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------


//==============================================================================
// MACROs
//------------------------------------------------------------------------------
#define INCREMENT_AMOUNT   15

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------

//==============================================================================
// Functions
//------------------------------------------------------------------------------

interface_DA_to_FE::interface_DA_to_FE() = default;

interface_DA_to_FE::~interface_DA_to_FE() = default;

void interface_DA_to_FE::push(const InputDataSync& value) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        queue.push({value, false}); 
    }

    data_ready.notify_one(); 
}

std::optional<InputDataSync> interface_DA_to_FE::pop() {
    std::unique_lock<std::mutex> lock(queue_mutex);

    // Wait until there is data in the queue or the stop signal is set
    data_ready.wait(lock, [this]() { return !queue.empty() || stop; });

    if (queue.empty()) return std::nullopt;

    auto front = queue.front(); // Copy the front element
    queue.pop();                // Remove it from the queue
    if (!front.second) {
        frames_dropped++; // Increment the counter if it wasn't `peek`'d
    }

    return front.first; // Return the data
}

std::optional<InputDataSync> interface_DA_to_FE::peek() {
    std::lock_guard<std::mutex> lock(queue_mutex);

    if (queue.empty()) return std::nullopt;

    auto& front = queue.front(); // Access the front element
    front.second = true;         // Mark as `peek`'d
    return front.first;          // Return the data
}

void interface_DA_to_FE::stop_queue() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        stop = true;
    }
    data_ready.notify_all(); // Notify all waiting threads
}

int interface_DA_to_FE::get_frame_drop_count() {
    std::lock_guard<std::mutex> lock(queue_mutex);
    return frames_dropped;
}

void interface_FE_to_BE::addToBuffer(int x)
{
    std::unique_lock<std::shared_mutex> ul(mtx);
    buffer.push_back(x);
}

int interface_FE_to_BE::checkBuffer()
{
    std::shared_lock<std::shared_mutex> sl(mtx);
    return buffer.size();
}

int interface_FE_to_BE::checkIndex(char threadID)
{
    std::shared_lock<std::shared_mutex> sl(mtx);
    if (threadID == 'C')
    {
        return indexC;
    }
    else if (threadID == 'B')
    {
        return indexBE;
    }
    return -1;
}

int interface_FE_to_BE::readBuffer(char threadID)
{
    
    std::unique_lock<std::shared_mutex> ul(mtx);

    int pos = 0;

    // Find value at the oldest unread index of the thread
    if (threadID == 'C')
    {
        pos = indexC;
    }
    else if (threadID == 'B')
    {
        pos = indexBE;
    }

    if (pos < 0 || pos >= buffer.size()) 
    {
        throw std::out_of_range("Position out of range");
    }
    int value = buffer.at(pos);

    // Add to threads read index
    if (threadID == 'C')
    {
        indexC++;

        //Remove all read parts of buffer (done only on communications thread)
        for (int i = 0; i < std::min(indexC, indexBE); i++)
        {
            removeFirstFromBuffer();
        }
    }
    else if (threadID == 'B')
    {
        indexBE++;
    }

    return value;
}

void interface_FE_to_BE::removeFirstFromBuffer()
{
    if (!buffer.empty()) 
    {
        buffer.erase(buffer.begin());
    }
    indexC--;
    indexBE--;
}

//==============================================================================
// End of File : Software/src/threads.cpp
