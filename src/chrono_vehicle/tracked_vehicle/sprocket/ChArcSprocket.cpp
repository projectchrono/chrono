// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a sprocket template with gear profile composed of circular
// arcs, suitable for interaction with single-pin track shoes.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/sprocket/ChArcSprocket.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChSinglePinShoe.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class ArcSprocketContactCB : public ChSystem::ChCustomComputeCollisionCallback {
  public:
    ArcSprocketContactCB(ChTrackAssembly* track,  ///< associate track assembly
                         int gear_nteeth,
                         double gear_RO,
                         double gear_RC,
                         double gear_R,
                         double shoe_locF,  ///< location of front cylinder on shoe (in local frame)
                         double shoe_locR,  ///< location of rear cylinder on shoe (in local frame)
                         double shoe_R      ///< radius of shoe cylinders
                         )
        : m_track(track),
          m_sprocket(m_track->GetSprocket()),
          m_gear_nteeth(gear_nteeth),
          m_gear_RO(gear_RO),
          m_gear_RC(gear_RC),
          m_gear_R(gear_R),
          m_shoe_locF(shoe_locF),
          m_shoe_locR(shoe_locR),
          m_shoe_R(shoe_R) {}

    virtual void PerformCustomCollision(ChSystem* system) override;

  private:
    ChTrackAssembly* m_track;
    ChSharedPtr<ChSprocket> m_sprocket;
    int m_gear_nteeth;
    double m_gear_RO;
    double m_gear_RC;
    double m_gear_R;
    double m_shoe_locF;
    double m_shoe_locR;
    double m_shoe_R;
};

void ArcSprocketContactCB::PerformCustomCollision(ChSystem* system) {
    // Sprocket state
    ChVector<> locG = m_sprocket->GetGearBody()->GetPos();

    double R2 = (m_gear_RO + m_shoe_R) * (m_gear_RO + m_shoe_R);

    // Loop over all shoes in the associated track.
    for (size_t is = 0; is < m_track->GetNumTrackShoes(); ++is) {
        ChSharedPtr<ChTrackShoe> shoe = m_track->GetTrackShoe(is);
        // Broadphase collision detection: reject any shoe for which both the front
        // and rear cylinder centers are outside the gear's bounding sphere.
        ChVector<> locF = shoe->GetShoeBody()->TransformPointLocalToParent(ChVector<>(m_shoe_locF, 0, 0));
        ChVector<> locR = shoe->GetShoeBody()->TransformPointLocalToParent(ChVector<>(m_shoe_locR, 0, 0));

        if ((locF - locG).Length2() > R2 && (locR - locG).Length2() > R2)
            continue;

        // Narrowphase collision detection.

        //// TODO
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChArcSprocket::ChArcSprocket(const std::string& name) : ChSprocket(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSystem::ChCustomComputeCollisionCallback* ChArcSprocket::GetCollisionCallback(ChTrackAssembly* track) {
    // Check compatibility between this type of sprocket and the track shoes.
    // We expect track shoes of type ChSinglePinShoe.
    ChSharedPtr<ChSinglePinShoe> shoe = track->GetTrackShoe(0).DynamicCastTo<ChSinglePinShoe>();
    assert(shoe);

    // Extract parameterization of gear profile
    int gear_nteeth = GetNumTeeth();
    double gear_RO = GetOuterRadius();
    double gear_RC = GetArcCentersRadius();
    double gear_R = GetArcRadius();

    // Extract parameterization of the shoe contact geometry.
    double shoe_locF = shoe->GetFrontCylinderLoc();
    double shoe_locR = shoe->GetRearCylinderLoc();
    double shoe_R = shoe->GetCylinderRadius();

    // Create and return the callback object. Note: this pointer will be freed by the base class.
    return new ArcSprocketContactCB(track, gear_nteeth, gear_RO, gear_RC, gear_R, shoe_locF, shoe_locR, shoe_R);
}

// -----------------------------------------------------------------------------
// Create and return the sprocket gear profile.
// -----------------------------------------------------------------------------
ChSharedPtr<geometry::ChLinePath> ChArcSprocket::GetProfile() {
    ChSharedPtr<geometry::ChLinePath> profile(new geometry::ChLinePath);

    int num_teeth = GetNumTeeth();
    double R_T = GetOuterRadius();
    double R_C = GetArcCentersRadius();
    double R = GetArcRadius();

    double beta = CH_C_2PI / num_teeth;
    double sbeta = std::sin(beta / 2);
    double cbeta = std::cos(beta / 2);
    double y = (R_T * R_T + R_C * R_C - R * R) / (2 * R_C);
    double x = std::sqrt(R_T * R_T - y * y);
    double gamma = std::asin(x / R);

    for (int i = 0; i < num_teeth; ++i) {
        double alpha = -i * beta;
        ChVector<> p0(0, R_C, 0);
        ChVector<> p1(-R_T * sbeta, R_T * cbeta, 0);
        ChVector<> p2(-x, y, 0);
        ChVector<> p3(x, y, 0);
        ChVector<> p4(R_T * sbeta, R_T * cbeta, 0);
        ChQuaternion<> quat;
        quat.Q_from_AngZ(alpha);
        ChMatrix33<> rot(quat);
        p0 = rot * p0;
        p1 = rot * p1;
        p2 = rot * p2;
        p3 = rot * p3;
        p4 = rot * p4;
        geometry::ChLineSegment seg1(p1, p2);
        double angle1 = alpha + 1.5 * CH_C_PI - gamma;
        double angle2 = alpha + 1.5 * CH_C_PI + gamma;
        geometry::ChLineArc arc(ChCoordsys<>(p0), R, angle1, angle2, true);
        geometry::ChLineSegment seg2(p3, p4);
        profile->AddSubLine(seg1);
        profile->AddSubLine(arc);
        profile->AddSubLine(seg2);
    }

    return profile;
}

}  // end namespace vehicle
}  // end namespace chrono
