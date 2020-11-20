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
// Called by CommunicationManager to transform an incoming
// SynFlatBuffers::Message into a SynMessage with state information inside of it
//
// =============================================================================

#ifndef SYN_MESSAGE_FACTORY_H
#define SYN_MESSAGE_FACTORY_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

/// Called by CommunicationManager to transform an incoming SynFlatBuffers::Message into a SynMessage
class SYN_API SynMessageFactory {
  public:
    /// Branches on message->message_type() to return various derived classes of SynMessage
    static SynMessage* GenerateMessage(const SynFlatBuffers::Message* message);
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
