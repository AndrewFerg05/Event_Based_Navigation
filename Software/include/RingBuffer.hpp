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
#include <map>
#include <tuple>
#include <thread>
#include <utility>
#include <mutex>
#include <Eigen/Dense>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <type_traits>
#include <utility>

//==============================================================================
//      Classes
//------------------------------------------------------------------------------

struct InterpolatorNearest
{
  template<typename Ringbuffer_T>
  static typename Ringbuffer_T::DataType interpolate(
      Ringbuffer_T* buffer,
      int64_t time,
      typename Ringbuffer_T::timering_t::iterator it_before)
  {
    // the end value
    auto it_after = it_before + 1;
    if (it_after == buffer->times_.end())
    {
      return buffer->dataAtTimeIterator(it_before);
    }

    // The times are ordered, we can guarantee those differences to be positive
    if ((time - *it_before) < (*it_after - time))
    {
      return buffer->dataAtTimeIterator(it_before);
    }
    return buffer->dataAtTimeIterator(it_after);
  }

  template<typename Ringbuffer_T>
  static typename Ringbuffer_T::DataType interpolate(
      Ringbuffer_T* buffer,
      int64_t time)
  {
    auto it_before = buffer->iterator_equal_or_before(time);

    return interpolate(buffer, time, it_before);
  }
};

//! A simple linear interpolator
struct InterpolatorLinear
{
  template<typename Ringbuffer_T>
  static typename Ringbuffer_T::DataType interpolate(
      Ringbuffer_T* buffer,
      int64_t time,
      typename Ringbuffer_T::timering_t::iterator it_before)
  {
    // the end value
    auto it_after = it_before + 1;
    if (it_after == buffer->times_.end())
    {
      return buffer->dataAtTimeIterator(it_before);
    }

    const real_t w1 =
        static_cast<real_t>(time - *it_before) /
        static_cast<real_t>(*it_after - *it_before);

    return (real_t{1.0} - w1) * buffer->dataAtTimeIterator(it_before)
        + w1 * buffer->dataAtTimeIterator(it_after);
  }

  template<typename Ringbuffer_T>
  static typename Ringbuffer_T::DataType interpolate(
      Ringbuffer_T* buffer,
      int64_t time)
  {
    auto it_before = buffer->iterator_equal_or_before(time);
    // caller should check the bounds:
    CHECK(it_before != buffer->times_.end());

    return interpolate(buffer, time, it_before);
  }
};
using DefaultInterpolator = InterpolatorLinear;





#endif  // RINGBUFFER_HPP

//==============================================================================
// End of File :  Software/include/RingBuffer.hpp
//==============================================================================
