/*
Filename    : Software/include/RingBuffer.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 05/02/25
Description : Optimized Ring Buffer for IMU and Event Synchronization
--------------------------------------------------------------------------------
*/

#ifndef RINGBUFFER_HPP
#define RINGBUFFER_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <vector>
#include <mutex>
#include <optional>
#include <cstdint>
#include <tuple>
#include <algorithm>
#include <Eigen/Dense>

//==============================================================================
//      Classes
//------------------------------------------------------------------------------

// Template class for a Ring Buffer using Eigen storage
template <typename T, size_t Rows, size_t Capacity>
class RingBuffer {
public:
    using MatrixType = Eigen::Matrix<T, Rows, Capacity>;
    using VectorType = Eigen::Matrix<T, Rows, 1>;

    explicit RingBuffer() : head_(0), tail_(0), size_(0) {
        times_.resize(Capacity, 0);  // Initialize timestamps with zeros
        data_.setZero();             // Initialize IMU data matrix with zeros
    }

    // Add an element to the buffer
    void insert(int64_t stamp, const VectorType& data) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Store timestamp
        times_[head_] = stamp;

        // Store IMU data into the Eigen matrix
        data_.col(head_) = data;

        // Advance the head index
        head_ = (head_ + 1) % Capacity;

        if (size_ < Capacity) {
            size_++;
        } else {
            // Overwrite oldest data
            tail_ = (tail_ + 1) % Capacity;
        }
    }

    // Retrieve the oldest and newest timestamp in the buffer
    std::tuple<int64_t, int64_t, bool> getOldestAndNewestStamp() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (size_ == 0) return {0, 0, false};  // No data available

        return {times_[tail_], times_[(head_ - 1 + Capacity) % Capacity], true};
    }

    // Get all values between two timestamps, interpolating if necessary
    std::pair<std::vector<int64_t>, std::vector<VectorType>> getBetweenValuesInterpolated(int64_t start_time, int64_t end_time) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<int64_t> timestamps;
        std::vector<VectorType> measurements;

        if (size_ == 0) return {timestamps, measurements};  // Return empty vectors if no data

        size_t index = tail_;
        while (index != head_) {
            int64_t ts = times_[index];

            if (ts >= start_time && ts <= end_time) {
                timestamps.push_back(ts);
                measurements.push_back(data_.col(index));
            }

            index = (index + 1) % Capacity;
        }

        return {timestamps, measurements};
    }

    // Check if the buffer has data within the given timestamp range
    bool hasDataInRange(int64_t start_time, int64_t end_time) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (size_ == 0) return false;

        size_t index = tail_;
        while (index != head_) {
            int64_t ts = times_[index];
            if (ts >= start_time && ts <= end_time) return true;
            index = (index + 1) % Capacity;
        }

        return false;
    }

    // Get the number of elements in the buffer
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return size_;
    }

    // Check if the buffer is empty
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return size_ == 0;
    }

    // Clear the buffer
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = tail_ = size_ = 0;
        times_.assign(Capacity, 0);
        data_.setZero();
    }

private:
    std::vector<int64_t> times_;  // Stores timestamps
    MatrixType data_;             // Stores IMU data as Eigen matrix
    size_t head_;                 // Write index
    size_t tail_;                 // Read index
    size_t size_;                 // Current size
    mutable std::mutex mutex_;    // Mutex for thread safety
};

#endif  // RINGBUFFER_HPP

//==============================================================================
// End of File :  Software/include/RingBuffer.hpp
//==============================================================================
