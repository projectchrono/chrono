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
// Authors: Jay Taves
// =============================================================================
//
// Class that wraps data contained in a message about Soil Contact Model (SCM)
// Deformable terrain. See chrono_vehicle/terrain/SCMDeformableTerrain.* for
// more details.
//
// =============================================================================

#ifndef SYN_SCM_MESSAGE_H
#define SYN_SCM_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

struct SynSCMTerrainState : public SynMessageState {
    std::vector<vehicle::SCMDeformableTerrain::NodeLevel> modified_nodes;

    /// Default constructor
    SynSCMTerrainState() : SynMessageState(0.0) {}

    /// Creates state with specified diffs
    SynSCMTerrainState(double time, std::vector<vehicle::SCMDeformableTerrain::NodeLevel> modified_nodes)
        : SynMessageState(time), modified_nodes(modified_nodes) {}
};

/// Class that wraps data contained in a message about Soil Contact Model (SCM) Deformable terrain.
class SYN_API SynSCMMessage : public SynMessage {
  public:
    ///@brief Construct a new SynSCMMessage object
    ///
    ///@param rank the rank of this message
    SynSCMMessage(int rank, std::shared_ptr<SynSCMTerrainState> state = nullptr);

    ///@brief Generates and sets the state of this message from flatbuffer message
    ///
    ///@param message the flatbuffer message to convert to a MessageState object
    virtual void StateFromMessage(const SynFlatBuffers::Message* message) override;

    ///@brief Generates a SynFlatBuffers::Message from the message state
    ///
    ///@param builder the flatbuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Message> the generated message
    virtual FlatBufferMessage MessageFromState(flatbuffers::FlatBufferBuilder& builder) override;

    ///@brief Get the SynMessageState object
    ///
    ///@return std::shared_ptr<SynMessageState> the state associated with this message
    virtual std::shared_ptr<SynMessageState> GetState() override { return m_state; }

    ///@brief Get the SynSCMTerrainState object
    ///
    ///@return std::shared_ptr<SynSCMTerrainState> the state associated with this message
    std::shared_ptr<SynSCMTerrainState> GetSCMState() { return m_state; }

  private:
    std::shared_ptr<SynSCMTerrainState> m_state;  ///< handle to the message state
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
