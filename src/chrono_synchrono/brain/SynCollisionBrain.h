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
// Authors: Jason Zhou
// =============================================================================
//
// Brain that detects collisions with other vehicles, providing a call-back to
// determine what to do in such a case
//
// =============================================================================

#ifndef SYN_COLLISION_BRAIN_H
#define SYN_COLLISION_BRAIN_H

#include "chrono_synchrono/brain/SynVehicleBrain.h"

using namespace chrono;

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_brain
/// @{

class SYN_API SynCollisionBrain : public SynVehicleBrain {
  public:
    virtual void ProcessMessage(SynMessage* msg) override;

    void TrackLoc(SynMessage* msg, int sender_rank, ChVector<> location);

    // Helper function for circular collision detection
    void checkDistanceCircle(ChVector<> pos1, ChVector<> pos2, double radius, int sender_rank);

    void checkDistanceRec(ChVector<> pos1,
                          ChVector<> pos2,
                          double short_rec_side,
                          double long_rec_side,
                          int sender_rank);

    void SetDisplay(bool display) { m_display = display; }
    void SetMyLocation(ChVector<> location) { m_my_loc = location; }

    void TakeUserActionsRectangle();
    void TakeUserActionsCircle();

    void AddUserActionRectangle(void (*ptr)()) { m_rectangle_actions.push_back(ptr); }
    void AddUserActionCircle(void (*ptr)()) { m_circle_actions.push_back(ptr); }

    void AddTargetRank(int target) { m_target_ranks.push_back(target); }

    void PrintTargetRanks();
    void RemoveTargetRank(int rm_target);

  private:
    bool m_display = false;
    ChVector<> m_my_loc;

    // usr defined collision action array
    std::vector<void (*)()> m_rectangle_actions;
    std::vector<void (*)()> m_circle_actions;

    std::vector<int> m_target_ranks;
};

/// @} synchrono_brain

}  // namespace synchrono
}  // namespace chrono

#endif
