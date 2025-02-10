#include "cv_bridge.hpp"


ImageCv8uC1::Ptr cvBridgeLoad8uC1(const std::string& filename)
{
  ImageCv8uC1::Ptr img;
  cvBridgeLoad<Pixel8uC1>(img, filename, PixelOrder::gray);
  return img;
}

