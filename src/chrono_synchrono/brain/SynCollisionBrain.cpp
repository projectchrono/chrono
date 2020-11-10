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

#include "chrono_synchrono/brain/SynCollisionBrain.h"

#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"
#include "chrono/core/ChVector.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace synchrono {

void SynCollisionBrain::ProcessMessage(SynMessage* msg) {
    // if the user has set the target rank(s) by using addTargetRank()
    // the TrackLoc() will only work on the specified rank
    // if no target ranks have been added, trackLoc will work for all ranks
    int sender_rank = msg->GetRank();
    if (std::find(m_target_ranks.begin(), m_target_ranks.end(), sender_rank) != m_target_ranks.end())
        TrackLoc(msg, sender_rank, m_vehicle.GetChassisBody()->GetPos());
}

//  Track location function extract the Message state data and also receives
//  the location of the agent on the current rank from the SynMPIManager
void SynCollisionBrain::TrackLoc(SynMessage* msg, int sender_rank, ChVector<> location) {
    if (m_display) {
        // update the location on the current rank everytime when processmessage()function is called.
        SetMyLocation(location);
        auto state = std::static_pointer_cast<SynWheeledVehicleState>(msg->GetState());

        // extract location data from the state object
        std::cout << "====sender rank====" << sender_rank << "=======" << std::endl;
        std::cout << (state->chassis.GetFrame().GetPos()) << std::endl;

        // display the location of the agent on the current rank
        std::cout << "====my rank====" << m_rank << "=======" << std::endl;
        std::cout << "my Loc: " << m_my_loc << std::endl;

        // create two ChVector for collision detection
        // pos1 stores the sender location, pos2 stores the current rank lcoation
        ChVector<> pos1 = state->chassis.GetFrame().GetPos();

        // calls collision detect helper function
        CheckDistanceCircle(pos1, m_my_loc, 5.0, sender_rank);
        CheckDistanceRec(pos1, m_my_loc, 3.0, 7.0, sender_rank);
        std::cout << std::endl;
    }
}

// This function check collision using a circular range
// The extra arguments are the radius of the circle and the sender rank
void SynCollisionBrain::CheckDistanceCircle(ChVector<> pos1, ChVector<> pos2, double radius, int sender_rank) {
    double res = (pos1 - pos2).Length2();

    std::cout << "distance data is: " << res << std::endl;

    // compare the distance with the given radius parameter of the circle
    if (res <= radius) {
        std::cout << "Circular Detection: rank " << m_rank << " collides with rank " << sender_rank << std::endl;
        TakeUserActionsCircle();
    } else {
        std::cout << "Circular Detection: no collision" << std::endl;
    }
}

// The function checks the collision by drawing a rectangular box
// The extra arguments required are the length of the rectangle's shorter side
// the length of the rectangle's longer side, and the rank of the sender
void SynCollisionBrain::CheckDistanceRec(ChVector<> pos1,
                                         ChVector<> pos2,
                                         double short_rec_side,
                                         double long_rec_side,
                                         int sender_rank) {
    ChVector<> pos_diff = pos1 - pos2;

    // Obtain absolute distance magnitude
    if (pos_diff.x() < 0)
        pos_diff.x() = -pos_diff.x();
    if (pos_diff.y() < 0)
        pos_diff.y() = -pos_diff.y();

    // Compare with the given lengths of the rectangal
    if (pos_diff.x() < short_rec_side && pos_diff.y() < long_rec_side) {
        std::cout << "Rectangular Detection: rank " << m_rank << " collides with rank " << sender_rank << std::endl;
        TakeUserActionsRectangle();
    } else {
        std::cout << "Rectangular Detection: no collision" << std::endl;
    }
}

void SynCollisionBrain::TakeUserActionsRectangle() {
    for (auto action : m_rectangle_actions)
        (*action)();
}

void SynCollisionBrain::TakeUserActionsCircle() {
    for (auto action : m_circle_actions)
        (*action)();
}

void SynCollisionBrain::PrintTargetRanks() {
    if (m_target_ranks.size() == 0)
        std::cout << "WARNING: No target rank has been set, will detect on all ranks" << std::endl;

    for (auto rank : m_target_ranks) {
        std::cout << "target_Rank: " << std::endl;
        std::cout << rank << std::endl;
    }
}

void SynCollisionBrain::RemoveTargetRank(int rm_target) {
    m_target_ranks.erase(std::remove(m_target_ranks.begin(), m_target_ranks.end(), rm_target), m_target_ranks.end());
}

}  // namespace synchrono
}  // namespace chrono
