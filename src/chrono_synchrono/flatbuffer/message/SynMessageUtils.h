// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Wrapper for several constructs that are common to many flatbuffer messages
// (Vectors, Quaternions, frames)
// See also flatbuffer/fbs/Utils.fbs
//
// =============================================================================

#ifndef SYN_MESSAGE_UTILS_H
#define SYN_MESSAGE_UTILS_H

#include "chrono_synchrono/flatbuffer/message/SynFlatBuffers_generated.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "chrono_synchrono/SynApi.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// Wrapper for several constructs that are common to many flatbuffer messages (Vectors, Quaternions, frames)
class SYN_API SynPose {
  public:
    ///@brief Construct a new Pose object
    SynPose(const ChVector<>& mv = ChVector<>(0, 0, 0), const ChQuaternion<>& mq = ChQuaternion<>(1, 0, 0, 0));

    ///@brief Construct a new Pose object
    SynPose(const ChFrameMoving<>& frame);

    ///@brief Construct a new Pose object from a FlatBuffers pose object
    ///
    ///@param pose the FlatBuffers pose object
    SynPose(const SynFlatBuffers::Pose* pose);

    ///@brief Convert this pose object to a flatbuffers pose type
    ///
    ///@param builder the FlatBuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Pose> the flatbuffer pose
    flatbuffers::Offset<SynFlatBuffers::Pose> ToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const;

    ChFrameMoving<>& GetFrame() { return m_frame; }

  private:
    ChFrameMoving<> m_frame;
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif