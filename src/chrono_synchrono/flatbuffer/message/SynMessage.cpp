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
// This class is abstract and is meant to be the basis for which SynChrono
// messages are implemented. Each SynMessage will convert to and from FlatBuffer
// data buffers. These data buffers are communicated between different nodes to
// pass information between processes and entities.
//
// Each SynMessage has a source (where it was sent) and a destination (where its
// headed).
//
// =============================================================================

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

SynMessage::SynMessage(unsigned int source_id, unsigned int destination_id)
    : m_source_id(source_id), m_destination_id(destination_id), time(0.0) {}

SynMessage::~SynMessage() {}

}  // namespace synchrono
}  // namespace chrono
