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
// Store the Map information in the simulation. Currently only used for traffic
// light. A Map has Intersections which have approaches which have lanes
//
// =============================================================================

#ifndef SYN_MAP_MESSAGE_H
#define SYN_MAP_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynApproachMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// Contains some number of approaches - see ApproachMessage
struct Intersection {
    std::vector<SynApproachMessageState> approaches;

    Intersection() {}
    Intersection(std::vector<SynApproachMessageState> approaches) : approaches(approaches) {}
    Intersection(const SynFlatBuffers::MAP::intersection* intersection);
};

/// Contains some number of intersections
struct SynMAPMessageState : public SynMessageState {
    int rank;
    std::vector<Intersection> intersections;

    SynMAPMessageState() : SynMessageState(0.0) {}
    SynMAPMessageState(double time, int rank, std::vector<Intersection> intersections)
        : SynMessageState(time), rank(rank), intersections(intersections) {}
    SynMAPMessageState(const SynFlatBuffers::MAP::State* state);
};

/// Store the Map information in the simulation. Currently only used for traffic light. A Map has Intersections which
/// have approaches which have lanes
class SYN_API SynMAPMessage : public SynMessage {
  public:
  public:
    ///@brief Construct a new SynMAPMessage object
    ///
    ///@param rank the rank of this message
    SynMAPMessage(int rank, std::shared_ptr<SynMAPMessageState> state = nullptr);

    ///@brief Generates and sets the state of this message from flatbuffer message
    ///
    ///@param message the flatbuffer message to convert to a MessageState object
    virtual void StateFromMessage(const SynFlatBuffers::Message* message) override;

    ///@brief Generates a SynFlatBuffers::Message from the message state
    ///
    ///@param builder the flatbuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Message> the generated message
    virtual FlatBufferMessage MessageFromState(flatbuffers::FlatBufferBuilder& builder) override;

    /**
     * @brief Add the lane to environment.
     *
     * @return The lane's position in that approach
     */
    virtual int AddLane(int intersection, int approach, ApproachLane lane);

    ///@brief Get the SynMessageState object
    ///
    ///@return std::shared_ptr<SynMessageState> the state associated with this message
    virtual std::shared_ptr<SynMessageState> GetState() override { return m_state; }

    ///@brief Get the SynMAPMessageState object
    ///
    ///@return std::shared_ptr<SynMAPMessageState> the state associated with this message
    std::shared_ptr<SynMAPMessageState> GetMAPState() { return m_state; }

  private:
    std::shared_ptr<SynMAPMessageState> m_state;  ///< handle to the message state
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
