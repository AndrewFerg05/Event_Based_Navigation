/*
Filename    : Software/include/Types.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 05/01/25
Description : File to define types
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 AF created
--------------------------------------------------------------------------------
*/

#ifndef TIMECONVERSION_HPP
#define TIMECONVERSION_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <cstdint>
#include <chrono>
#include "TypeAliases.hpp"


//==============================================================================
//      Classes
//------------------------------------------------------------------------------


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
  //! Utilities for working with timestamps.
//!
//! Important: Always store int64_t nanosecond timestamps! We use signed type
//!            to avoid errors when taking differences and we use nanoseconds
//!            when saving to file to have a unique type for lookups in
//!            dictionaries/maps.

//! Seconds to nanoseconds.
inline constexpr int64_t secToNanosec(real_t seconds)
{
  return static_cast<int64_t>(seconds * 1e9);
}

//! Milliseconds to nanoseconds.
inline constexpr int64_t millisecToNanosec(real_t milliseconds)
{
  return static_cast<int64_t>(milliseconds * 1e6);
}

//! Nanoseconds to seconds.
//! WARNING: Don't pass very large or small numbers to this function as the
//!          representability of the float value does not capture nanoseconds
//!          resolution. The resulting accuracy will be in the order of
//!          hundreds of nanoseconds.
inline constexpr real_t nanosecToSecTrunc(int64_t nanoseconds)
{
  return static_cast<real_t>(nanoseconds) / 1e9;
}

//! Nanoseconds to milliseconds.
//! WARNING: Don't pass very large or very small numbers to this function as the
//!          representability of the float value does not capture nanoseconds
//!          resolution.
inline constexpr real_t nanosecToMillisecTrunc(int64_t nanoseconds)
{
  return static_cast<real_t>(nanoseconds) / 1e6;
}

//! Return total nanoseconds from seconds and nanoseconds pair.
inline constexpr int64_t nanosecFromSecAndNanosec(int32_t sec, int32_t nsec)
{ 
  return static_cast<int64_t>(sec) * 1000000000ll + static_cast<int64_t>(nsec);
}





#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp