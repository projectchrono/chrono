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
// Environment Agents have no special processing logic, but SynBrain is abstract
// so we need this concrete (although empty) derived class
//
// =============================================================================

#ifndef SYN_ENVIRONMENTBRAIN_H
#define SYN_ENVIRONMENTBRAIN_H

#include "chrono_synchrono/brain/SynBrain.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_brain
/// @{

/// All logic happens in the EnvironmentAgent class, but we still need an empty concrete class here
class SynEnvironmentBrain : public SynBrain {
  public:
    SynEnvironmentBrain(int rank) : SynBrain(rank){};

    virtual void Synchronize(double time) override{};
    virtual void Advance(double step) override{};
    virtual void ProcessMessage(SynMessage* msg) override{};
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) override{};
};

/// @} synchrono_brain

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_ENVIRONMENTBRAIN_H