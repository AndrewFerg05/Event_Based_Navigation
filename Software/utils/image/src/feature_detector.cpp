#include "feature_detector.hpp"

#include "fast_detector.hpp"
#ifdef ZE_USE_BRISK
#include <imp/features/brisk_detector.hpp>
#endif
#include "camera.hpp"
#include "camera_rig.hpp"

//------------------------------------------------------------------------------
FeatureDetectionHandler::FeatureDetectionHandler(
    const DetectionStrategy& detectors)
  : detectors_(detectors)
{}

void FeatureDetectionHandler::detect(
    const ImagePyramid8uC1& pyr,
    KeypointsWrapper& features,
    bool mask_existing_keypoints)
{
  for (const AbstractDetector::Ptr& detector : detectors_)
  {
    if (features.num_detected > 0 && mask_existing_keypoints)
    {
      //! @todo: mask based on whether landmarks_vec_ has a valid id??
      VLOG(100) << "Set " << features.num_detected << " existing keypoints.";
      detector->setExistingKeypoints(features.px.leftCols(features.num_detected));
    }
    if (mask_)
    {
      detector->setMask(mask_);
    }
    detector->detect(pyr, features);
    detector->reset();
  }
}

//------------------------------------------------------------------------------
void FeatureDetectionHandler::setGrid(const OccupancyGrid2D& grid)
{
  for (const AbstractDetector::Ptr& detector : detectors_)
  {
    detector->setGrid(grid);
  }
}

//------------------------------------------------------------------------------
AbstractDetector::AbstractDetector(
    const Size2u& image_size, DetectorType type)
  : type_(type)
  , image_size_(image_size)
{}

//------------------------------------------------------------------------------
std::vector<std::shared_ptr<FeatureDetectionHandler>>
loadVioDetectorsFromGflags(const CameraRig& rig)
{
  std::vector<std::shared_ptr<FeatureDetectionHandler>> detectors;
  if (FLAGS_imp_detector_name == "BRISK")
  {
#ifdef ZE_USE_BRISK
    if (FLAGS_imp_detector_border_margin < 30)
    {
      LOG(FATAL) << "When using BRISK, border margin should not be smaller than 30"
                 << " to avoid that features are lost when extracting descriptors.";
    }
    BriskDetectorOptions brisk_options;
    brisk_options.brisk_num_octaves = FLAGS_imp_detector_num_octaves;
    brisk_options.brisk_max_keypoints = FLAGS_imp_detector_max_features_per_frame;
    brisk_options.brisk_uniformity_radius = FLAGS_imp_brisk_uniformity_radius;
    brisk_options.brisk_absolute_threshold = FLAGS_imp_brisk_absolute_threshold;
    brisk_options.border_margin = FLAGS_imp_detector_border_margin;
    brisk_options.cell_size = FLAGS_imp_detector_grid_size;
    for (const Camera::Ptr& cam : rig)
    {
      DetectionStrategy strategy;
      strategy.emplace_back(
            std::make_shared<BriskDetector>(
              brisk_options, cam->size()));
      auto detector = std::make_shared<FeatureDetectionHandler>(strategy);
      detector->setMask(cam->mask());
      detectors.emplace_back(detector);
    }
#else
    LOG(FATAL) << "BRISK detector not available.";
#endif
  }
  else if (FLAGS_imp_detector_name == "FAST")
  {
    FastDetectorOptions fast_options;
    fast_options.max_level = FLAGS_imp_detector_num_octaves - 1;
    fast_options.cell_size = FLAGS_imp_detector_grid_size;
    fast_options.border_margin = FLAGS_imp_detector_border_margin;
    fast_options.threshold = FLAGS_imp_detector_threshold;
    for (const Camera::Ptr& cam : rig)
    {
      DetectionStrategy strategy;
      strategy.emplace_back(
            std::make_shared<FastDetector>(
              fast_options, cam->size()));
      auto detector = std::make_shared<FeatureDetectionHandler>(strategy);
      detector->setMask(cam->mask());
      detectors.emplace_back(detector);
    }
  }
  else
  {
    LOG(FATAL) << "Detector '" << FLAGS_imp_detector_name << "'not supported."
               << " Choose between {FAST, BRISK}";
  }
  return detectors;
}

