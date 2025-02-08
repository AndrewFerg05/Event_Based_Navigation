/*
Filename    : Software/include/TypeAliases.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 14/1/25
Description : File to define type aliasing used
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 AF created
--------------------------------------------------------------------------------
*/

#ifndef TYPE_ALIASES_HPP
#define TYPE_ALIASES_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <cstdint>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include "Types.hpp"

//==============================================================================
//      Classes
//------------------------------------------------------------------------------

using real_t = double; //Can change between single and double


#define ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(SIZE, SUFFIX)            \
  using Matrix##SUFFIX = Eigen::Matrix<real_t, SIZE, SIZE>; \
  using Matrix1##SUFFIX = Eigen::Matrix<real_t, 1, SIZE>;   \
  using Matrix2##SUFFIX = Eigen::Matrix<real_t, 2, SIZE>;   \
  using Matrix3##SUFFIX = Eigen::Matrix<real_t, 3, SIZE>;   \
  using Matrix4##SUFFIX = Eigen::Matrix<real_t, 4, SIZE>;   \
  using Matrix5##SUFFIX = Eigen::Matrix<real_t, 5, SIZE>;   \
  using Matrix6##SUFFIX = Eigen::Matrix<real_t, 6, SIZE>;   \
  using Matrix7##SUFFIX = Eigen::Matrix<real_t, 7, SIZE>;   \
  using Matrix8##SUFFIX = Eigen::Matrix<real_t, 8, SIZE>;   \
  using Matrix9##SUFFIX = Eigen::Matrix<real_t, 9, SIZE>;   \
  using Matrix##SUFFIX##X = Eigen::Matrix<real_t, SIZE, Eigen::Dynamic>; \
  using MatrixX##SUFFIX = Eigen::Matrix<real_t, Eigen::Dynamic, SIZE>;   \
  static const Eigen::MatrixBase<Matrix##SUFFIX>::IdentityReturnType I_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Identity(); \
  static const Eigen::MatrixBase<Matrix##SUFFIX>::ConstantReturnType Z_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Zero()

ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(1,1);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(2,2);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(3,3);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(4,4);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(5,5);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(6,6);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(7,7);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(8,8);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(9,9);

using MatrixX = Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic>;
// used for testing will be replaced with actual later
using InputDataSync = uint8_t;
using TrackedFrames = uint8_t;
using OtherData = uint8_t;



// Dummy structs for data streams can change later

using CameraInfoData = uint16_t;
using ExposureData = uint16_t;


// Event Data
using EventArray = std::vector<Event>;
using EventQueue = EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;
using StampedEventArray = std::pair<int64_t, EventArrayPtr>;
using StampedEventArrays = std::vector<StampedEventArray>;
using EventBuffer = std::deque<Event>;



using Vector3 = Eigen::Matrix<real_t, 3, 1>;
using Vector6 = Eigen::Matrix<real_t, 6, 1>;


using ImuStamps = Eigen::Matrix<int64_t, Eigen::Dynamic, 1>;
using ImuAccGyrContainer = Matrix6X;

using ImuCallback =
  std::function<void (int64_t /*timestamp*/,
                      const Vector3& /*acc*/,
                      const Vector3& /*gyr*/)>;


//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------






#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp