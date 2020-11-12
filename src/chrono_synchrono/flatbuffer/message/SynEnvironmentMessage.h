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
// Authors: 肖言 (Yan Xiao)
// =============================================================================
//
// Message class for Environment Agents. This class is only used to send the
// initial zombie description for an Environment agent. Environment agents do
// not synchronize their state in any way so don't need messages for any other
// purpose.
//
// =============================================================================

#ifndef SYN_ENVIRONMENT_MESSAGE_H
#define SYN_ENVIRONMENT_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynAgentMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

struct SynEnvironmentMessageState : public SynMessageState {
    int rank;

    SynEnvironmentMessageState() : SynMessageState(0.0) {}

    SynEnvironmentMessageState(double time, int rank) : SynMessageState(time), rank(rank) {}
};

/// Used to send the initial description for an environment agent
class SYN_API SynEnvironmentMessage : public SynAgentMessage {
  public:
    SynEnvironmentMessage(int rank, std::shared_ptr<SynEnvironmentMessageState> state = nullptr);

    virtual void StateFromMessage(const SynFlatBuffers::Message* message) override;

    virtual FlatBufferMessage MessageFromState(flatbuffers::FlatBufferBuilder& builder) override;

    virtual std::shared_ptr<SynMessageState> GetState() override { return m_state; }

    virtual void DescriptionFromMessage(const SynFlatBuffers::Message* message) override;

    virtual FlatBufferMessage MessageFromDescription(flatbuffers::FlatBufferBuilder& builder) override;

  private:
    std::shared_ptr<SynEnvironmentMessageState> m_state;
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif