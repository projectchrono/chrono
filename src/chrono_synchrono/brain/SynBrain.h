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
// Base class for all Brains that SynChrono agents can have
// - Advance (do physics)
// - Synchronize (exchange state info)
// - Process/GenerateMessages (for state info)
//
// =============================================================================

#ifndef SYN_BRAIN_H
#define SYN_BRAIN_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_brain
/// @{

/// Base class for any brain - must Advance the state of physics, Synchronize state with others and Process Messages.
class SynBrain {
  public:
    /// Constructor
    SynBrain(int rank) : m_rank(rank) {}

    /// Destructor
    virtual ~SynBrain() {}

    /// Advance the state of this brain until brain time syncs with passed time
    virtual void Advance(double step) = 0;

    /// Synchronize this brain to the specified time
    virtual void Synchronize(double time) = 0;

    /// Process an incoming message
    virtual void ProcessMessage(SynMessage* msg) = 0;

    /// Generate vector of SynMessage's to send
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) = 0;

    /// Get this brain's rank
    int GetRank() { return m_rank; }

  protected:
    int m_rank;  ///< rank of this brain
};

/// @} synchrono_brain

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_BRAIN_H
