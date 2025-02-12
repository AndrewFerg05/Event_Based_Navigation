/*
Filename    : Software/include/BackEnd.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Header file for the BackEnd (BE) thread
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

#ifndef BACKEND_HPP
#define BACKEND_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include "ThreadInterface.hpp"
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "estimator.hpp"
#include "TypeAliases.hpp"

//==============================================================================
//      Classes
//------------------------------------------------------------------------------

    // fwd
    class CameraRig;
    class ImuIntegrator;
    class LandmarkTable;
    class NFrameTable;
    
    // callback declarations.
    using BiasUpdateCallback = std::function<void(const Vector3& /*acc_bias*/,
                                                  const Vector3& /*gyr_bias*/)>;
    
    struct CeresBackendOptions
    {
      // Optimization settings.
      size_t num_iterations{ 10 };
      size_t num_threads { 2 };
      bool verbose { false };
    
      // Marginalization settings.
      bool marginalize { true };
      size_t num_keyframes { 5 };
      size_t num_imu_frames { 3 };
    };
    
    class BackEnd 
    {
    public:
    BackEnd(LandmarkTable& landmarks_,
                            NFrameTable& states,
                            const std::shared_ptr<const CameraRig>& rig,
                            std::shared_ptr<ImuIntegrator> imu_integrator,
                            const CeresBackendOptions& settings_ = CeresBackendOptions());
    
      ~BackEnd();
    
      void start();
      void idle();
      void stop();
    
      void addTrackedNFrame(
          const std::shared_ptr<NFrame>& nframe_k,
          const ImuStamps& imu_stamps,
          const ImuAccGyrContainer& imu_accgyr,
          const VioMotionType motion_type,
          const std::vector<LandmarkHandle>& lm_opportunistic,
          const std::vector<LandmarkHandle>& lm_persistent_new,
          const std::vector<LandmarkHandle>& lm_persistent_continued);
    
      bool updateStateWithResultFromLastOptimization(
          const bool wait_for_backend = false);
    
    private:
      void optimization();
    
      void optimizationLoop();
    
      bool addLandmarksAndObservationsToBackend(
          const std::vector<LandmarkHandle>& new_landmarks);
    
      bool addNewestObservationsToBackend(
          const std::vector<LandmarkHandle>& continued_landmarks,
          const NFrame::ConstPtr& nframe);
    
      Estimator backend_;
      bool backend_initialized_{false}; //!< Is initialized as soon as 2 frames have been added.
      CeresBackendOptions settings_;
      const size_t num_cameras_;
      //! Imu integrator is only used to update biases for frontend.
      std::shared_ptr<ImuIntegrator> imu_integrator_;
    
      // State related.
      LandmarkTable& landmarks_;
      NFrameTable& states_;
      BackendId last_added_nframe_;
      BackendId last_optimized_nframe_;
      BackendId last_updated_nframe_;
    
      // Threading
      mutable std::condition_variable wait_condition_;
      mutable std::mutex mutex_backend_;
      std::unique_ptr<std::thread> thread_;
      std::atomic_bool stop_thread_ { false };
    
    };



#endif  // BACKEND_HPP
//==============================================================================
// End of File :  Software/include/BackEnd.hpp