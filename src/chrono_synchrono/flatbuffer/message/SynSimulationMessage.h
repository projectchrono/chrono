// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// =============================================================================
//
// =============================================================================

#ifndef SYN_SIMULATION_MESSAGE_H
#define SYN_SIMULATION_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

///@brief This class contains diagnostic and simulation configuration based information that is
/// typically passed between CommunicationManagers in the initialization phase
///
class SYN_API SynSimulationMessage : public SynMessage {
  public:
    ///@brief Constructor
    ///
    ///@param source_id the id of the source to which the message is sent from
    ///@param destination_id the id of the destination to which the message is sent to
    ///@param quit_sim whether this message instructs the manager to end the simulation
    SynSimulationMessage(unsigned int source_id, unsigned int destination_id, bool quit_sim = false);

    ///@brief Destroy the SynMessage object
    virtual ~SynSimulationMessage();

    ///@brief Converts a received flatbuffer message to a SynMessage
    ///
    ///@param message the flatbuffer message to convert to a SynMessage
    virtual void ConvertFromFlatBuffers(const SynFlatBuffers::Message* message) override;

    ///@brief Converts this object to a flatbuffer message
    ///
    ///@param builder a flatbuffer builder to construct the message with
    ///@return FlatBufferMessage the constructed flatbuffer message
    virtual FlatBufferMessage ConvertToFlatBuffers(flatbuffers::FlatBufferBuilder& builder) const override;

    // ---------------------------------------------------------------

    bool m_quit_sim;  ///< Instruction to end the simulation early
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
