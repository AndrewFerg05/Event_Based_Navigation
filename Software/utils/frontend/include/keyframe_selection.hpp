// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <tuple>

#include "transformation.hpp"
#include "TypeAliases.hpp"

#include "Flags.hpp"




// fwd
class LandmarkTable;
class NFrame;
class NFrameTable;

//! @return Returns true when new keyframe is necessary.
bool needNewKeyframe(
    const NFrame& nframe_k,
    const NFrame& nframe_lkf,
    const NFrameTable& states,
    const Transformation& T_Bk_W,
    const TransformationVector& T_C_B,
    const uint32_t num_tracked);

