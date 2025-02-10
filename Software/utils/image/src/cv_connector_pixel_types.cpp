#include "cv_connector_pixel_types.hpp"

#include <opencv2/core/core.hpp>


//------------------------------------------------------------------------------
PixelType pixelTypeFromCv(int type)
{
  switch (type)
  {
  case CV_8UC1: return PixelType::i8uC1;
  case CV_8UC2: return PixelType::i8uC2;
  case CV_8UC3: return PixelType::i8uC3;
  case CV_8UC4: return PixelType::i8uC4;
  //
  case CV_16UC1: return PixelType::i16uC1;
  case CV_16UC2: return PixelType::i16uC2;
  case CV_16UC3: return PixelType::i16uC3;
  case CV_16UC4: return PixelType::i16uC4;
  //
  case CV_32SC1: return PixelType::i32sC1;
  case CV_32SC2: return PixelType::i32sC2;
  case CV_32SC3: return PixelType::i32sC3;
  case CV_32SC4: return PixelType::i32sC4; //
  case CV_32FC1: return PixelType::i32fC1;
  case CV_32FC2: return PixelType::i32fC2;
  case CV_32FC3: return PixelType::i32fC3;
  case CV_32FC4: return PixelType::i32fC4;
  //
  default: return PixelType::undefined;
  }
}

//------------------------------------------------------------------------------
int pixelTypeToCv(PixelType type)
{
  switch (type)
  {
  case PixelType::i8uC1: return CV_8UC1;
  case PixelType::i8uC2: return CV_8UC2;
  case PixelType::i8uC3: return CV_8UC3;
  case PixelType::i8uC4: return CV_8UC4;
  //
  case PixelType::i16uC1: return CV_16UC1;
  case PixelType::i16uC2: return CV_16UC2;
  case PixelType::i16uC3: return CV_16UC3;
  case PixelType::i16uC4: return CV_16UC4;
  //
  case PixelType::i32sC1: return CV_32SC1;
  case PixelType::i32sC2: return CV_32SC2;
  case PixelType::i32sC3: return CV_32SC3;
  case PixelType::i32sC4: return CV_32SC4;
  //
  case PixelType::i32fC1: return CV_32FC1;
  case PixelType::i32fC2: return CV_32FC2;
  case PixelType::i32fC3: return CV_32FC3;
  case PixelType::i32fC4: return CV_32FC4;
  //
  default: return 0;
  }
}

