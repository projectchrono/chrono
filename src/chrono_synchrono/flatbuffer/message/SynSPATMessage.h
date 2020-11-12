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
// Authors: 肖言 (Yan Xiao), Shuo He
// =============================================================================
//
// Wraps data received from a flatbuffer SPAT message into a corresponding C++
// class
// See also flatbuffer/fbs/SPAT.fbs
//
// =============================================================================

#ifndef SYN_SPAT_MESSAGE_H
#define SYN_SPAT_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

enum class LaneColor { GREEN, YELLOW, RED };

/// Lane for the purpose of SPAT messages (i.e. something that can have its light color change)
struct IntersectionLane {
    int intersection;
    int approach;
    int lane;
    LaneColor color;

    IntersectionLane(int intersection, int approach, int lane, LaneColor color)
        : intersection(intersection), approach(approach), lane(lane), color(color) {}
};

struct SynSPATMessageState : public SynMessageState {
    std::vector<IntersectionLane> lanes;

    SynSPATMessageState() : SynMessageState(0.0) {}
    SynSPATMessageState(double time, std::vector<IntersectionLane> lanes) : SynMessageState(time), lanes(lanes) {}
    SynSPATMessageState(const SynFlatBuffers::SPAT::State* state);
};

/// Wraps data from a SPAT message into a corresponding C++ object.
class SYN_API SynSPATMessage : public SynMessage {
  public:
    ///@brief Construct a new SynSPATMessage object
    ///
    ///@param rank the rank of this message
    SynSPATMessage(int rank, std::shared_ptr<SynSPATMessageState> state = nullptr);

    ///@brief Generates and sets the state of this message from flatbuffer message
    ///
    ///@param message the flatbuffer message to convert to a MessageState object
    virtual void StateFromMessage(const SynFlatBuffers::Message* message) override;

    ///@brief Generates a SynFlatBuffers::Message from the message state
    ///
    ///@param builder the flatbuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Message> the generated message
    virtual FlatBufferMessage MessageFromState(flatbuffers::FlatBufferBuilder& builder) override;

    virtual void setColor(int intersection, int approach, int lane, LaneColor color);

    ///@brief Get the SynMessageState object
    ///
    ///@return std::shared_ptr<SynMessageState> the state associated with this message
    virtual std::shared_ptr<SynMessageState> GetState() override { return m_state; }

    ///@brief Get the SynSPATMessageState object
    ///
    ///@return std::shared_ptr<SynSPATMessageState> the state associated with this message
    std::shared_ptr<SynSPATMessageState> GetSPATState() { return m_state; }

  private:
    std::shared_ptr<SynSPATMessageState> m_state;  ///< handle to the message state
};

/// @} synchrono_flatbuffer
}  // namespace synchrono
}  // namespace chrono

#endif
