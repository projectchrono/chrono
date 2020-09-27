#include "chrono_synchrono/brain/SynTrackingBrain.h"

namespace chrono {
namespace synchrono {

SynTrackingBrain::SynTrackingBrain(int rank, std::shared_ptr<ChDriver> driver, ChVehicle& vehicle)
    : SynVehicleBrain(rank, driver, vehicle) {
    m_nearest_vehicle = 1000;
    m_rank = rank;
}

SynTrackingBrain::~SynTrackingBrain() {}

void SynTrackingBrain::Synchronize(double time) {
    // m_driver->Synchronize(time);
}

void SynTrackingBrain::Advance(double step) {
    // m_driver->Advance(step);
}

void SynTrackingBrain::ProcessMessage(SynMessage* msg) {
    // if the user has set the target rank(s) by using addTargetRank()
    // the trackLoc() will only work on the specified rank
    // if no target ranks have been added, trackLoc will work for all ranks
    auto state = std::static_pointer_cast<SynWheeledVehicleState>(msg->GetState());
    int sender_rank = msg->GetRank();
    if ((std::find(target_rank.begin(), target_rank.end(), sender_rank) != target_rank.end()) || target_rank.empty()) {
        trackLoc(msg, sender_rank, m_vehicle.GetChassisBody()->GetPos());
    } else {
    }
}

//  Track location function extract the Message state data and also receives
//  the location of the agent on the current rank from the SynMPIManager
void SynTrackingBrain::trackLoc(SynMessage* msg, int sender_rank, chrono::Vector Sen) {
    if (enableDis == true) {
        // update the location on the current rank everytime when processmessage()function is called.
        myLoc = Sen;
        auto state = std::static_pointer_cast<SynWheeledVehicleState>(msg->GetState());

        // extract location data from the state object
        std::cout << "====sender rank====" << sender_rank << "=======" << std::endl;
        std::cout << (state->chassis.GetFrame().GetPos()) << std::endl;

        // display the location of the agent on the current rank
        std::cout << "====my rank====" << m_rank << "=======" << std::endl;
        std::cout << "my Loc: " << myLoc << std::endl;

        // create two ChVector for collision detection
        // pos1 stores the sender location, pos2 stores the current rank lcoation
        ChVector<> pos1 = state->chassis.GetFrame().GetPos();
        ChVector<> pos2 = myLoc;

        // calls collision detect helper function
        checkDistanceCircle(pos1, pos2, 5.0, sender_rank);
        checkDistanceRec(pos1, pos2, 3.0, 7.0, sender_rank);
        std::cout << std::endl;
    }
}

// This function check collision using a circular range
// The extra arguments are the radius of the circle and the sender rank
void SynTrackingBrain::checkDistanceCircle(ChVector<> pos1, ChVector<> pos2, double radius, int sender_rank) {
    double locData[3];
    for (int i = 0; i < 3; i++) {
        locData[i] = pos1[i] - pos2[i];
    }

    // calculate the distance between two agents
    double res = locData[0] * locData[0] + locData[1] * locData[1] + locData[2] * locData[2];
    std::cout << "distance data is: " << res << std::endl;

    // compare the distance with the given radius parameter of the circle
    if (res <= radius) {
        std::cout << "Circular Detection: rank " << m_rank << " collides with rank " << sender_rank << std::endl;
        actionUsrCir();
    } else {
        std::cout << "Circular Detection: no collision" << std::endl;
    }
}

// The function checks the collision by drawing a rectangular box
// The extra arguments requried are the length of the rectangular's shorter side
// the length of the rectangular's longer side, and the rank of the sender
void SynTrackingBrain::checkDistanceRec(ChVector<> pos1,
                                        ChVector<> pos2,
                                        double shortRecSide,
                                        double longRecSide,
                                        int sender_rank) {
    // Calculate X Y Z displacemet components
    double disX = pos1[0] - pos2[0];
    double disY = pos1[1] - pos2[1];
    double disZ = pos1[2] - pos2[2];

    // Obtain absolute distance magnitude
    if (disX < 0) {
        disX = -disX;
    }
    if (disY < 0) {
        disY = -disY;
    }
    if (disZ < 0) {
        disZ = -disZ;
    }

    // Compare with the given lengths of the rectangal
    if (disX < shortRecSide && disY < longRecSide) {
        std::cout << "Rectangular Detection: rank " << m_rank << " collides with rank " << sender_rank << std::endl;
        actionUsrRec();
    } else {
        std::cout << "Rectangular Detection: no collision" << std::endl;
    }
}

double SynTrackingBrain::DistanceToLine(ChVector<> p, ChVector<> l1, ChVector<> l2) {
    // https://www.intmath.com/plane-analytic-geometry/perpendicular-distance-point-line.php
    double A = l1.y() - l2.y();
    double B = -(l1.x() - l2.x());
    double C = l1.x() * l2.y() - l1.y() * l2.x();
    double d = abs(A * p.x() + B * p.y() + C) / sqrt(A * A + B * B);
    return d;
}

bool SynTrackingBrain::IsInsideBox(ChVector<> pos, ChVector<> sp, ChVector<> op, double w) {
    double len = (sp - op).Length();
    // TODO :: Should be w / 2, but paths are too far away from the actual lanes

    return (pos - sp).Length() < len && (pos - op).Length() < len && DistanceToLine(pos, sp, op) < w;
}

void SynTrackingBrain::updateMyLoc(chrono::Vector Sen) {
    std::cout << "testing Tbrain myLoc" << std::endl;
    myLoc = Sen;
}

void SynTrackingBrain::enableLocDisplay() {
    enableDis = true;
}

void SynTrackingBrain::disableLocDisplay() {
    enableDis = false;
};

}  // namespace synchrono
}  // namespace chrono
