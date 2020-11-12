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
// Authors: 肖言 (Yan Xiao), Jay Taves
// =============================================================================
//
// Class for Approach Messages, those contains data on approaches, which are a
// set of lanes going towards/away from an intersection in the same direction
//
// =============================================================================

#ifndef SYN_APPROACH_MESSAGE_H
#define SYN_APPROACH_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// @brief Lane that is grouped into an approach with other ApproachLanes
struct ApproachLane {
    double width;                           ///< define the width of the lane. Control points to the side is width/2.
    std::vector<ChVector<>> controlPoints;  ///< The points define the center of the lane.

    ///
    /// @brief Construct a new Approach Lane object
    ///
    /// @param width Define the width of the lane, which means Control points to the side is width/2
    /// @param control_points The points define the center of the lane. Each adjacent points will form a rectangle used
    /// for lane detection. Will NOT form a Bezier curve
    ApproachLane(double width, std::vector<ChVector<>> control_points) : width(width), controlPoints(control_points) {}
    ApproachLane(const SynFlatBuffers::Approach::Lane* lane);
};

struct SynApproachMessageState : public SynMessageState {
    int rank;
    std::vector<ApproachLane> lanes;

    SynApproachMessageState() : SynMessageState(0.0) {}
    SynApproachMessageState(double time, int rank, std::vector<ApproachLane> lanes)
        : SynMessageState(time), rank(rank), lanes(lanes) {}
    SynApproachMessageState(const SynFlatBuffers::Approach::State* approach);
};

/// Handles approach messages, which are a group of lanes going to or from an intersection
class SYN_API SynApproachMessage : public SynMessage {
  public:
    SynApproachMessage(int rank, std::shared_ptr<SynApproachMessageState> state = nullptr);

    virtual void StateFromMessage(const SynFlatBuffers::Message* message) override;

    virtual FlatBufferMessage MessageFromState(flatbuffers::FlatBufferBuilder& builder) override;

    virtual std::shared_ptr<SynMessageState> GetState() override { return m_state; }

    std::shared_ptr<SynApproachMessageState> GetApproachState() { return m_state; }

  private:
    std::shared_ptr<SynApproachMessageState> m_state;
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
