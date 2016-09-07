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
                               double gear_RT,          ///< radius if the addendum circle
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
          m_gear_RT(gear_RT),
          m_gear_R(gear_R),
          m_gear_C(gear_C),
          m_gear_W(gear_W),
          m_gear_Rhat(gear_R - envelope),
          m_separation(separation),
          m_shoe_len(shoe_len),
          m_shoe_R(shoe_R),
          m_shoe_Rhat(shoe_R + envelope) {
        m_R_sum = (m_shoe_len + 2 * m_shoe_R) + m_gear_RT;
        m_gear_RC = std::sqrt(m_gear_C * m_gear_C + m_gear_W * m_gear_W);
        m_gear_D = m_gear_W - m_gear_R;
        m_beta = CH_C_2PI / m_gear_nteeth;
        m_sbeta = std::sin(m_beta / 2);
        m_cbeta = std::cos(m_beta / 2);

    }

    virtual void PerformCustomCollision(ChSystem* system) override;

  private:
    // Test collision between a connector body and the sprocket's gear profiles.
    void CheckConnectorSprocket(std::shared_ptr<ChBody> connector,  // connector body
                                const ChVector<>& locS_abs          // center of sprocket (global frame)
                                );

    // Test collision between a circle and the gear profile (in the plane of the gear).
    void CheckCircleProfile(std::shared_ptr<ChBody> connector,  // connector body
                            const ChVector<>& loc,              // center of circular connector end
                            const ChVector<>& p1L,
                            const ChVector<>& p2L,
                            const ChVector<>& p3L,
                            const ChVector<>& p1R,
                            const ChVector<>& p2R,
                            const ChVector<>& p3R);

    ChTrackAssembly* m_track;                // pointer to containing track assembly
    std::shared_ptr<ChSprocket> m_sprocket;  // handle to the sprocket

    double m_envelope;  // collision detection envelope

    int m_gear_nteeth;    // sprocket gear, number of teeth
    double m_gear_RT;     // sprocket gear, outer tooth radius (radius of addendum circle)
    double m_gear_R;      // sprocket gear, arc radius
    double m_gear_C;      // sprocket gear, arc center height
    double m_gear_W;      // sprocket gear, arc center offset
    double m_separation;  // separation distance between sprocket gears

    double m_gear_D;   // tooth width (D = W - R)
    double m_gear_RC;  // radius of arc centers circle (RC^2 = C^2 + W^2)

    double m_beta;   // angle between two consecutive gear teeth
    double m_sbeta;  // sin(beta/2)
    double m_cbeta;  // cos(beta/2)

    double m_shoe_len;  // length of track shoe connector
    double m_shoe_R;    // radius of track shoe connector

    double m_gear_Rhat;  // adjusted gear arc radius
    double m_shoe_Rhat;  // adjusted shoe cylinder radius

    double m_R_sum;  // test quantity for broadphase check
};

// Add contacts between the sprocket and track shoes.
void SprocketDoublePinContactCB::PerformCustomCollision(ChSystem* system) {
    // Return now if collision disabled on sproket.
    if (!m_sprocket->GetGearBody()->GetCollide())
        return;

    // Sprocket gear center location (expressed in global frame)
    ChVector<> locS_abs = m_sprocket->GetGearBody()->GetPos();

    // Loop over all track shoes in the associated track
    for (size_t is = 0; is < m_track->GetNumTrackShoes(); ++is) {
        auto shoe = std::static_pointer_cast<ChTrackShoeDoublePin>(m_track->GetTrackShoe(is));
        
        // Perform collision test for the "left" connector body
        CheckConnectorSprocket(shoe->m_connector_L, locS_abs);

        // Perform collision test for the "right" connector body
        CheckConnectorSprocket(shoe->m_connector_R, locS_abs);
    }
}

// Perform collision test between the specified connector body and the associated sprocket.
void SprocketDoublePinContactCB::CheckConnectorSprocket(std::shared_ptr<ChBody> connector, const ChVector<>& locS_abs) {
    // (1) Express the center of the connector body in the sprocket frame
    ChVector<> loc = m_sprocket->GetGearBody()->TransformPointParentToLocal(connector->GetPos());

    // (2) Broadphase collision test: no contact if the connector's center is too far from the
    // center of the gear (test perfomed in the sprocket's x-z plane).
    if (loc.x * loc.x + loc.z * loc.z > m_R_sum * m_R_sum)
        return;

    // (3) Working in the frame of the sprocket, find the candidate tooth space.
    // This is the closest tooth space to the connector center point.

    // Angle formed by 'locC' and the line z>0
    double angle = std::atan2(loc.x, loc.z);
    // Find angle of closest tooth space
    double alpha = m_beta * std::round(angle / m_beta);
    // Convert to degrees
    double alpha_deg = alpha * 180 / CH_C_PI;

    // (4) Calculate the points that define the current tooth space.
    // Note that all these points will have Y = locC.y
    ChVector<> p0L(-(m_gear_RT - m_gear_D) * m_sbeta, loc.y, (m_gear_RT - m_gear_D) * m_cbeta);
    ChVector<> p0R(+(m_gear_RT - m_gear_D) * m_sbeta, loc.y, (m_gear_RT - m_gear_D) * m_cbeta);

    ChVector<> p1L = p0L + ChVector<>(+m_gear_D * m_cbeta, 0, m_gear_D * m_sbeta);
    ChVector<> p1R = p0R + ChVector<>(-m_gear_D * m_cbeta, 0, m_gear_D * m_sbeta);

    ChVector<> p2L = ChVector<>(-m_gear_C * m_sbeta, loc.y, m_gear_C * m_cbeta) +
                     ChVector<>(+m_gear_D * m_cbeta, 0, m_gear_D * m_sbeta);
    ChVector<> p2R = ChVector<>(+m_gear_C * m_sbeta, loc.y, m_gear_C * m_cbeta) +
                     ChVector<>(-m_gear_D * m_cbeta, 0, m_gear_D * m_sbeta);

    ChVector<> p3L = p2L + ChVector<>(+m_gear_R * m_cbeta, 0, m_gear_R * m_sbeta);
    ChVector<> p3R = p2R + ChVector<>(-m_gear_R * m_cbeta, 0, m_gear_R * m_sbeta);

    ChVector<> p4L = p3L - ChVector<>(0, 0, m_gear_R);
    ChVector<> p4R = p3R - ChVector<>(0, 0, m_gear_R);

    ChMatrix33<> rot(alpha, ChVector<>(0, 1, 0));

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

    // (5) Express the centers of the two connector end-caps in the sprocket frame.
    ChVector<> P1_abs = connector->TransformPointLocalToParent(ChVector<>(m_shoe_len / 2, 0, 0));
    ChVector<> P2_abs = connector->TransformPointLocalToParent(ChVector<>(-m_shoe_len / 2, 0, 0));
    ChVector<> P1 = m_sprocket->GetGearBody()->TransformPointParentToLocal(P1_abs);
    ChVector<> P2 = m_sprocket->GetGearBody()->TransformPointParentToLocal(P2_abs);

    ////if (loc.y > 0) {
    ////    std::cout << loc.x << " " << loc.y << " " << loc.z << std::endl;
    ////    std::cout << P1.x << " " << P1.y << " " << P1.z << std::endl;
    ////    std::cout << P2.x << " " << P2.y << " " << P2.z << std::endl;
    ////    std::cout << p0L.x << " " << p0L.y << " " << p0L.z << std::endl;
    ////    std::cout << p1L.x << " " << p1L.y << " " << p1L.z << std::endl;
    ////    std::cout << p2L.x << " " << p2L.y << " " << p2L.z << std::endl;
    ////    std::cout << p3L.x << " " << p3L.y << " " << p3L.z << std::endl;
    ////    std::cout << p4L.x << " " << p4L.y << " " << p4L.z << std::endl;
    ////    std::cout << p0R.x << " " << p0R.y << " " << p0R.z << std::endl;
    ////    std::cout << p1R.x << " " << p1R.y << " " << p1R.z << std::endl;
    ////    std::cout << p2R.x << " " << p2R.y << " " << p2R.z << std::endl;
    ////    std::cout << p3R.x << " " << p3R.y << " " << p3R.z << std::endl;
    ////    std::cout << p4R.x << " " << p4R.y << " " << p4R.z << std::endl;
    ////    std::cout << std::endl;
    ////}

    // (6) Perform collision test between the front end of the connector and the gear profile.
    CheckCircleProfile(connector, P1, p1L, p2L, p3L, p1R, p2R, p3R);

    // (7) Perform collision test between the rear end of the connector and the gear profile.
    CheckCircleProfile(connector, P2, p1L, p2L, p3L, p1R, p2R, p3R);
}

// Working in the (x-z) plane of the gear, perform a 2D collision test between a circle
// of radius m_shoe_R centered at the specified location and the gear profile.
void SprocketDoublePinContactCB::CheckCircleProfile(std::shared_ptr<ChBody> connector,
                                                    const ChVector<>& loc,
                                                    const ChVector<>& p1L,
                                                    const ChVector<>& p2L,
                                                    const ChVector<>& p3L,
                                                    const ChVector<>& p1R,
                                                    const ChVector<>& p2R,
                                                    const ChVector<>& p3R) {    
    // Check circle against arc centered at p3L.

    // Check circle against arc centered at p3R.

    // Check circle against segment p1L - p2L.

    // Check circle against segment p1R - p2R.

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
    double gear_RT = GetOuterRadius();
    double gear_R = GetArcRadius();
    double gear_C = GetArcCenterHeight();
    double gear_W = GetArcCenterOffset();

    // Extract parameterization of the shoe connector contact geometry.
    double shoe_len = shoe->GetConnectorLength();
    double shoe_R = shoe->GetConnectorRadius();

    // Create and return the callback object. Note: this pointer will be freed by the base class.
    return new SprocketDoublePinContactCB(track, 0.005, gear_nteeth, gear_RT, gear_R, gear_C, gear_W, GetSeparation(),
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
