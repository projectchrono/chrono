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
// arcs and a flat seat, suitable for interaction with double-pin track shoes.
//
// =============================================================================

#include <cmath>

#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class SprocketDoublePinContactCB : public ChSystem::ChCustomComputeCollisionCallback {
  public:
    SprocketDoublePinContactCB(ChTrackAssembly* track,  ///< containing track assembly
                               double envelope,         ///< collision detection envelope
                               int gear_nteeth,         ///< number of teeth of the sprocket gear
                               double gear_RO,          ///< radius if the addendum circle
                               double gear_R,           ///< radius of the tooth arc profile
                               double gear_C,           ///< height of profile arcs
                               double gear_W,           ///< offset of profile arcs
                               double separation,       ///< separation between sprocket gears
                               double shoe_len,         ///< length of track shoe connector
                               double shoe_R            ///< radius of track shoe connector
                               )
        : m_track(track),
          m_envelope(envelope),
          m_sprocket(m_track->GetSprocket()),
          m_gear_nteeth(gear_nteeth),
          m_gear_RO(gear_RO),
          m_gear_R(gear_R),
          m_gear_C(gear_C),
          m_gear_W(gear_W),
          m_gear_Rhat(gear_R - envelope),
          m_separation(separation),
          m_shoe_len(shoe_len),
          m_shoe_R(shoe_R),
          m_shoe_Rhat(shoe_R + envelope) {}

    virtual void PerformCustomCollision(ChSystem* system) override;

  private:
    ChTrackAssembly* m_track;                // pointer to containing track assembly
    std::shared_ptr<ChSprocket> m_sprocket;  // handle to the sprocket

    double m_envelope;  // collision detection envelope

    int m_gear_nteeth;    // sprocket gear, number of teeth
    double m_gear_RO;     // sprocket gear, outer radius (radius of addendum circle)
    double m_gear_R;      // sprocket gear, arc radius
    double m_gear_C;      // sprocket gear, arc center height
    double m_gear_W;      // sprocket gear, arc center offset
    double m_separation;  // separation distance between sprocket gears
    double m_shoe_len;    // length of track shoe connector
    double m_shoe_R;      // radius of track shoe connector

    double m_gear_Rhat;  // adjusted gear arc radius
    double m_shoe_Rhat;  // adjusted she cylinder radius
};

void SprocketDoublePinContactCB::PerformCustomCollision(ChSystem* system) {
    // Return now if collision disabled on sproket or track shoes.
    if (!m_sprocket->GetGearBody()->GetCollide() || !m_track->GetTrackShoe(0)->GetShoeBody()->GetCollide())
        return;

    //// TODO
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSprocketDoublePin::ChSprocketDoublePin(const std::string& name) : ChSprocket(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSystem::ChCustomComputeCollisionCallback* ChSprocketDoublePin::GetCollisionCallback(ChTrackAssembly* track) {
    // Check compatibility between this type of sprocket and the track shoes.
    // We expect track shoes of type ChSinglePinShoe.
    auto shoe = std::dynamic_pointer_cast<ChTrackShoeDoublePin>(track->GetTrackShoe(0));
    assert(shoe);

    // Extract parameterization of gear profile
    int gear_nteeth = GetNumTeeth();
    double gear_RO = GetOuterRadius();
    double gear_R = GetArcRadius();
    double gear_C = GetArcCenterHeight();
    double gear_W = GetArcCenterOffset();

    // Extract parameterization of the shoe contact geometry.
    double shoe_len = shoe->GetConnectorLength();
    double shoe_R = shoe->GetConnectorRadius();

    // Create and return the callback object. Note: this pointer will be freed by the base class.
    return new SprocketDoublePinContactCB(track, 0.005, gear_nteeth, gear_RO, gear_R, gear_C, gear_W, GetSeparation(),
                                          shoe_len, shoe_R);
}

// -----------------------------------------------------------------------------
// Create and return the sprocket gear profile.
// -----------------------------------------------------------------------------
std::shared_ptr<geometry::ChLinePath> ChSprocketDoublePin::GetProfile() {
    auto profile = std::make_shared<geometry::ChLinePath>();

    int num_teeth = GetNumTeeth();
    double R_T = GetOuterRadius();
    double C = GetArcCenterHeight();
    double W = GetArcCenterOffset();
    double R_C = std::sqrt(C * C + W * W);
    double R = GetArcRadius();
    double D = W - R;

    double beta = CH_C_2PI / num_teeth;
    double sbeta = std::sin(beta / 2);
    double cbeta = std::cos(beta / 2);

    for (int i = 0; i < num_teeth; ++i) {
        ChVector<> p0L(-(R_T - D) * sbeta, (R_T - D) * cbeta, 0);
        ChVector<> p0R(+(R_T - D) * sbeta, (R_T - D) * cbeta, 0);

        ChVector<> p1L = p0L + ChVector<>(+D * cbeta, D * sbeta, 0);
        ChVector<> p1R = p0R + ChVector<>(-D * cbeta, D * sbeta, 0);

        ChVector<> p2L = ChVector<>(-C*sbeta, C*cbeta, 0) + ChVector<>(+D * cbeta, D * sbeta, 0);
        ChVector<> p2R = ChVector<>(+C*sbeta, C*cbeta, 0) + ChVector<>(-D * cbeta, D * sbeta, 0);

        ChVector<> p3L = p2L + ChVector<>(+R * cbeta, R * sbeta, 0);
        ChVector<> p3R = p2R + ChVector<>(-R * cbeta, R * sbeta, 0);

        ChVector<> p4L = p3L - ChVector<>(0, R, 0);
        ChVector<> p4R = p3R - ChVector<>(0, R, 0);

        double alpha = -i * beta;
        ChMatrix33<> rot(alpha, ChVector<>(0, 0, 1));

        p0L = rot * p0L;
        p1L = rot * p1L;
        p2L = rot * p2L;
        p3L = rot * p3L;
        p4L = rot * p4L;

        p0R = rot * p0R;
        p1R = rot * p1R;
        p2R = rot * p2R;
        p3R = rot * p3R;
        p4R = rot * p4R;

        geometry::ChLineArc arc1(ChCoordsys<>(p0L), D, alpha + CH_C_PI_2 + beta / 2, alpha + beta / 2);
        geometry::ChLineSegment seg2(p1L, p2L);
        geometry::ChLineArc arc3(ChCoordsys<>(p3L), R, alpha + CH_C_PI + beta / 2, alpha + 1.5 * CH_C_PI, true);
        geometry::ChLineSegment seg4(p4L, p4R);
        geometry::ChLineArc arc5(ChCoordsys<>(p3R), R, alpha + 1.5 * CH_C_PI, alpha + CH_C_2PI - beta / 2, true);
        geometry::ChLineSegment seg6(p2R, p1R);
        geometry::ChLineArc arc7(ChCoordsys<>(p0R), D, alpha + CH_C_PI - beta / 2, alpha + CH_C_PI_2 - beta / 2);

        profile->AddSubLine(arc1);
        profile->AddSubLine(seg2);
        profile->AddSubLine(arc3);
        profile->AddSubLine(seg4);
        profile->AddSubLine(arc5);
        profile->AddSubLine(seg6);
        profile->AddSubLine(arc7);
    }

    return profile;
}

}  // end namespace vehicle
}  // end namespace chrono
