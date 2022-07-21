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
class SprocketDoublePinContactCB : public ChSystem::CustomCollisionCallback {
  public:
    SprocketDoublePinContactCB(ChTrackAssembly* track,     ///< containing track assembly
                               double envelope,            ///< collision detection envelope
                               int gear_nteeth,            ///< number of teeth of the sprocket gear
                               double gear_RT,             ///< radius if the addendum circle
                               double gear_R,              ///< radius of the tooth arc profile
                               double gear_C,              ///< height of profile arcs
                               double gear_W,              ///< offset of profile arcs
                               double separation,          ///< separation between sprocket gears
                               double shoe_len,            ///< length of track shoe connector
                               double shoe_R,              ///< radius of track shoe connector
                               bool lateral_contact,       ///< if true, enable lateral contact
                               double lateral_backlash,    ///< play relative to shoe guiding pin
                               const ChVector<>& shoe_pin  ///< location of shoe guide pin center
                               )
        : m_track(track),
          m_gear_nteeth(gear_nteeth),
          m_gear_RT(gear_RT),
          m_gear_R(gear_R),
          m_gear_C(gear_C),
          m_gear_W(gear_W),
          m_shoe_len(shoe_len),
          m_shoe_R(shoe_R),
          m_lateral_contact(lateral_contact),
          m_lateral_backlash(lateral_backlash),
          m_shoe_pin(shoe_pin) {
        m_sprocket = static_cast<ChSprocketDoublePin*>(m_track->GetSprocket().get());

        ////m_gear_Rhat = gear_R - envelope;
        ////m_shoe_Rhat = shoe_R + envelope;
        m_R_sum = (m_shoe_len + 2 * m_shoe_R) + m_gear_RT;
        m_gear_RC = std::sqrt(m_gear_C * m_gear_C + m_gear_W * m_gear_W);
        m_gear_D = m_gear_W - m_gear_R;
        m_beta = CH_C_2PI / m_gear_nteeth;
        m_sbeta = std::sin(m_beta / 2);
        m_cbeta = std::cos(m_beta / 2);

        // Create contact material for sprocket - guiding pin contacts (to prevent detracking)
        // Note: zero friction
        MaterialInfo minfo;
        minfo.mu = 0;
        minfo.cr = 0.1f;
        minfo.Y = 1e7f;
        m_material = minfo.CreateMaterial(m_sprocket->GetGearBody()->GetSystem()->GetContactMethod());
    }

    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    // Test collision between a connector body and the sprocket's gear profiles.
    void CheckConnectorSprocket(std::shared_ptr<ChBody> connector,                 // connector body
                                const ChFrame<>& shape_frame,                      // frame of connector collision shape
                                std::shared_ptr<ChMaterialSurface> mat_connector,  // connector contact material
                                const ChVector<>& locS_abs                         // center of sprocket (global frame)
    );

    // Test collision between a circle and the gear profile (in the plane of the gear).
    void CheckCircleProfile(std::shared_ptr<ChBody> connector,                 // connector body
                            std::shared_ptr<ChMaterialSurface> mat_connector,  // connector contact material
                            const ChVector<>& loc,                             // center of circular connector end
                            const ChVector<>& p1L,
                            const ChVector<>& p2L,
                            const ChVector<>& p3L,
                            const ChVector<>& p4L,
                            const ChVector<>& p1R,
                            const ChVector<>& p2R,
                            const ChVector<>& p3R,
                            const ChVector<>& p4R);

    void CheckCircleArc(std::shared_ptr<ChBody> connector,                 // connector body
                        std::shared_ptr<ChMaterialSurface> mat_connector,  // connector contact material
                        const ChVector<>& cc,                              // circle center
                        double cr,                                         // circle radius
                        const ChVector<> ac,                               // arc center
                        double ar,                                         // arc radius
                        const ChVector<>& p1,                              // arc end point 1
                        const ChVector<>& p2                               // arc end point 2
    );

    void CheckCircleSegment(std::shared_ptr<ChBody> connector,                 // connector body
                            std::shared_ptr<ChMaterialSurface> mat_connector,  // connector contact material
                            const ChVector<>& cc,                              // circle center
                            double cr,                                         // circle radius
                            const ChVector<>& p1,                              // segment end point 1
                            const ChVector<>& p2                               // segment end point 2
                            );

    // Test collision of a shoe guiding pin with the sprocket gear.
    // This may introduce one contact.
    void CheckPinSprocket(std::shared_ptr<ChTrackShoeDoublePin> shoe,  // track shoe
                          const ChVector<>& locPin_abs,                // center of guiding pin (global frame)
                          const ChVector<>& dirS_abs                   // sprocket Y direction (global frame)
    );

    ChTrackAssembly* m_track;         // pointer to containing track assembly
    ChSprocketDoublePin* m_sprocket;  // pointer to the sprocket

    int m_gear_nteeth;    // sprocket gear, number of teeth
    double m_gear_RT;     // sprocket gear, outer tooth radius (radius of addendum circle)
    double m_gear_R;      // sprocket gear, arc radius
    double m_gear_C;      // sprocket gear, arc center height
    double m_gear_W;      // sprocket gear, arc center offset

    double m_gear_D;   // tooth width (D = W - R)
    double m_gear_RC;  // radius of arc centers circle (RC^2 = C^2 + W^2)

    double m_beta;   // angle between two consecutive gear teeth
    double m_sbeta;  // sin(beta/2)
    double m_cbeta;  // cos(beta/2)

    double m_shoe_len;  // length of track shoe connector
    double m_shoe_R;    // radius of track shoe connector

    bool m_lateral_contact;     // if true, generate lateral contacts
    double m_lateral_backlash;  // backlash relative to shoe guiding pin
    ChVector<> m_shoe_pin;      // single-pin shoe, center of guiding pin

    ////double m_gear_Rhat;  // adjusted gear arc radius
    ////double m_shoe_Rhat;  // adjusted shoe cylinder radius

    double m_R_sum;  // test quantity for broadphase check

    std::shared_ptr<ChMaterialSurface> m_material;  // material for sprocket-pin contact (detracking)
};

// Add contacts between the sprocket and track shoes.
void SprocketDoublePinContactCB::OnCustomCollision(ChSystem* system) {
    // Return now if collision disabled on sprocket or track shoes.
    if (m_track->GetNumTrackShoes() == 0)
        return;
    if (!m_sprocket->GetGearBody()->GetCollide() || !m_track->GetTrackShoe(0)->GetShoeBody()->GetCollide())
        return;

    // Sprocket gear center location, expressed in global frame
    ChVector<> locS_abs = m_sprocket->GetGearBody()->GetPos();

    // Sprocket "normal" (Y axis), expressed in global frame
    ChVector<> dirS_abs = m_sprocket->GetGearBody()->GetA().Get_A_Yaxis();

    // Loop over all track shoes in the associated track
    for (size_t is = 0; is < m_track->GetNumTrackShoes(); ++is) {
        auto shoe = std::static_pointer_cast<ChTrackShoeDoublePin>(m_track->GetTrackShoe(is));

        switch (shoe->m_topology) {
            case DoublePinTrackShoeType::TWO_CONNECTORS: {
                // The collision shape frames are the same as the left and right connector body frames
                CheckConnectorSprocket(shoe->m_connector_L, *shoe->m_connector_L, shoe->GetSprocketContactMaterial(),
                                       locS_abs);
                CheckConnectorSprocket(shoe->m_connector_R, *shoe->m_connector_R, shoe->GetSprocketContactMaterial(),
                                       locS_abs);
            } break;
            case DoublePinTrackShoeType::ONE_CONNECTOR: {
                // The collision shape frames are offset in the Y direction from the connector body frame.
                ChFrame<> frame_left = *shoe->m_connector_L;
                frame_left.coord.pos += frame_left.GetA() * ChVector<>(0, shoe->GetShoeWidth() / 2, 0);
                CheckConnectorSprocket(shoe->m_connector_L, frame_left, shoe->GetSprocketContactMaterial(), locS_abs);
                ChFrame<> frame_right = *shoe->m_connector_L;
                frame_right.coord.pos -= frame_right.GetA() * ChVector<>(0, shoe->GetShoeWidth() / 2, 0);
                CheckConnectorSprocket(shoe->m_connector_L, frame_right, shoe->GetSprocketContactMaterial(), locS_abs);
            } break;
        }

        if (m_lateral_contact) {
            // Express guiding pin center in the global frame
            ChVector<> locPin_abs = shoe->GetShoeBody()->TransformPointLocalToParent(m_shoe_pin);

            // Perform collision detection with the central pin
            CheckPinSprocket(shoe, locPin_abs, dirS_abs);
        }
    }
}

// Perform collision test between the specified connector body and the associated sprocket.
void SprocketDoublePinContactCB::CheckConnectorSprocket(std::shared_ptr<ChBody> connector,
                                                        const ChFrame<>& shape_frame,
                                                        std::shared_ptr<ChMaterialSurface> mat_connector,
                                                        const ChVector<>& locS_abs) {
    // (1) Express the center of the connector shape in the sprocket frame
    ChVector<> loc = m_sprocket->GetGearBody()->TransformPointParentToLocal(shape_frame.GetPos());

    // (2) Broadphase collision test: no contact if the shape's center is too far from the
    // center of the gear (test performed in the sprocket's x-z plane).
    if (loc.x() * loc.x() + loc.z() * loc.z() > m_R_sum * m_R_sum)
        return;

    // (3) Working in the frame of the sprocket, find the candidate tooth space.
    // This is the closest tooth space to the shape center point.

    // Angle formed by 'locC' and the line z>0
    double angle = std::atan2(loc.x(), loc.z());
    // Find angle of closest tooth space
    double alpha = m_beta * std::round(angle / m_beta);

    // (4) Calculate the points that define the current tooth space.
    // Note that all these points will have Y = locC.y
    ChVector<> p0L(-(m_gear_RT - m_gear_D) * m_sbeta, loc.y(), (m_gear_RT - m_gear_D) * m_cbeta);
    ChVector<> p0R(+(m_gear_RT - m_gear_D) * m_sbeta, loc.y(), (m_gear_RT - m_gear_D) * m_cbeta);

    ChVector<> p1L = p0L + ChVector<>(+m_gear_D * m_cbeta, 0, m_gear_D * m_sbeta);
    ChVector<> p1R = p0R + ChVector<>(-m_gear_D * m_cbeta, 0, m_gear_D * m_sbeta);

    ChVector<> p2L = ChVector<>(-m_gear_C * m_sbeta, loc.y(), m_gear_C * m_cbeta) +
                     ChVector<>(+m_gear_D * m_cbeta, 0, m_gear_D * m_sbeta);
    ChVector<> p2R = ChVector<>(+m_gear_C * m_sbeta, loc.y(), m_gear_C * m_cbeta) +
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

    // (5) Express the centers of the two shape end-caps in the sprocket frame.
    ChVector<> P1_abs = shape_frame.TransformPointLocalToParent(ChVector<>(m_shoe_len / 2, 0, 0));
    ChVector<> P2_abs = shape_frame.TransformPointLocalToParent(ChVector<>(-m_shoe_len / 2, 0, 0));
    ChVector<> P1 = m_sprocket->GetGearBody()->TransformPointParentToLocal(P1_abs);
    ChVector<> P2 = m_sprocket->GetGearBody()->TransformPointParentToLocal(P2_abs);

    // (6) Perform collision test between the front end of the connector and the gear profile.
    CheckCircleProfile(connector, mat_connector, P1, p1L, p2L, p3L, p4L, p1R, p2R, p3R, p4R);

    // (7) Perform collision test between the rear end of the connector and the gear profile.
    CheckCircleProfile(connector, mat_connector, P2, p1L, p2L, p3L, p4L, p1R, p2R, p3R, p4R);
}

// Working in the (x-z) plane of the gear, perform a 2D collision test between a circle
// of radius m_shoe_R centered at the specified location and the gear profile.
void SprocketDoublePinContactCB::CheckCircleProfile(std::shared_ptr<ChBody> connector,
                                                    std::shared_ptr<ChMaterialSurface> mat_connector,
                                                    const ChVector<>& loc,
                                                    const ChVector<>& p1L,
                                                    const ChVector<>& p2L,
                                                    const ChVector<>& p3L,
                                                    const ChVector<>& p4L,
                                                    const ChVector<>& p1R,
                                                    const ChVector<>& p2R,
                                                    const ChVector<>& p3R,
                                                    const ChVector<>& p4R) {
    // Check circle against arc centered at p3L.
    CheckCircleArc(connector, mat_connector, loc, m_shoe_R, p3L, m_gear_R, p2L, p4L);

    // Check circle against arc centered at p3R.
    CheckCircleArc(connector, mat_connector, loc, m_shoe_R, p3R, m_gear_R, p3R, p4R);

    // Check circle against segment p1L - p2L.
    CheckCircleSegment(connector, mat_connector, loc, m_shoe_R, p1L, p2L);

    // Check circle against segment p1R - p2R.
    CheckCircleSegment(connector, mat_connector, loc, m_shoe_R, p1R, p2R);

    // Check circle against segment p4L - p4R.
    CheckCircleSegment(connector, mat_connector, loc, m_shoe_R, p4L, p4R);
}

// Working in the (x-z) plane, perform a 2D collision test between a circle of radius 'cr'
// centered at 'cc' (on the connector body) and an arc on the circle of radius 'ar' centered
// at 'ac', with the arc endpoints 'p1' and 'p2' (on the gear body).
void SprocketDoublePinContactCB::CheckCircleArc(std::shared_ptr<ChBody> connector,                 // connector body
                                                std::shared_ptr<ChMaterialSurface> mat_connector,  // conector material
                                                const ChVector<>& cc,                              // circle center
                                                double cr,                                         // circle radius
                                                const ChVector<> ac,                               // arc center
                                                double ar,                                         // arc radius
                                                const ChVector<>& p1,                              // arc end point 1
                                                const ChVector<>& p2                               // arc end point 2
) {
    // Find distance between centers
    ChVector<> delta = cc - ac;
    double dist2 = delta.Length2();

    // If the two centers (circle and arc) are separated by less than the difference
    // of their adjusted radii, there is no contact.
    double Rdiff = ar - cr;
    if (dist2 <= Rdiff * Rdiff)
        return;

    // If the contact point is outside the arc (p1-p2), there is no contact.
    ChVector<> a = p1 - ac;
    ChVector<> b = p2 - ac;
    ChVector<> axb = Vcross(a, b);
    ChVector<> axd = Vcross(a, delta);
    ChVector<> dxb = Vcross(delta, b);
    if (Vdot(axb, axd) <= 0 || Vdot(axb, dxb) <= 0)
        return;

    // Generate contact information (still in the sprocket frame)
    double dist = std::sqrt(dist2);
    ChVector<> normal = -delta / dist;
    ChVector<> pt_gear = ac - m_gear_R * normal;
    ChVector<> pt_shoe = cc - m_shoe_R * normal;

    // Fill in contact information and add the contact to the system.
    // Express all vectors in the global frame
    collision::ChCollisionInfo contact;
    contact.modelA = m_sprocket->GetGearBody()->GetCollisionModel().get();
    contact.modelB = connector->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = m_sprocket->GetGearBody()->TransformDirectionLocalToParent(normal);
    contact.vpA = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_gear);
    contact.vpB = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_shoe);
    contact.distance = Rdiff - dist;
    ////contact.eff_radius = cr;  //// TODO: take into account ar?

    m_sprocket->GetGearBody()->GetSystem()->GetContactContainer()->AddContact(contact, m_sprocket->GetContactMaterial(),
                                                                              mat_connector);
}

// Working in the (x-z) plane, perform a 2D collision test between the circle of radius 'cr'
// centered at 'cc' (on the connector body) and the line segment with endpoints 'p1' and 'p2'
// (on the gear body).
void SprocketDoublePinContactCB::CheckCircleSegment(
    std::shared_ptr<ChBody> connector,                 // connector body
    std::shared_ptr<ChMaterialSurface> mat_connector,  // conector material
    const ChVector<>& cc,                              // circle center
    double cr,                                         // circle radius
    const ChVector<>& p1,                              // segment end point 1
    const ChVector<>& p2                               // segment end point 2
) {
    // Find closest point on segment to circle center: X = p1 + t * (p2-p1)
    ChVector<> s = p2 - p1;
    double t = Vdot(cc - p1, s) / Vdot(s, s);
    ChClampValue(t, 0.0, 1.0);

    ChVector<> pt_gear = p1 + t * s;

    // No contact if circle center is too far from segment.
    ChVector<> delta = cc - pt_gear;
    double dist2 = delta.Length2();
    if (dist2 >= cr * cr)
        return;

    // Generate contact information (still in sprocket frame)
    double dist = std::sqrt(dist2);
    ChVector<> normal = delta / dist;
    ChVector<> pt_shoe = cc - cr * normal;

    // Fill in contact information and add the contact to the system.
    // Express all vectors in the global frame
    collision::ChCollisionInfo contact;
    contact.modelA = m_sprocket->GetGearBody()->GetCollisionModel().get();
    contact.modelB = connector->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    contact.vN = m_sprocket->GetGearBody()->TransformDirectionLocalToParent(normal);
    contact.vpA = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_gear);
    contact.vpB = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_shoe);
    contact.distance = dist - cr;
    ////contact.eff_radius = cr;

    m_sprocket->GetGearBody()->GetSystem()->GetContactContainer()->AddContact(contact, m_sprocket->GetContactMaterial(),
                                                                              mat_connector);
}

void SprocketDoublePinContactCB::CheckPinSprocket(std::shared_ptr<ChTrackShoeDoublePin> shoe,
                                                  const ChVector<>& locPin_abs,
                                                  const ChVector<>& dirS_abs) {
    // Express pin center in the sprocket frame
    ChVector<> locPin = m_sprocket->GetGearBody()->TransformPointParentToLocal(locPin_abs);

    // No contact if the pin is close enough to the sprocket's center
    if (std::abs(locPin.y()) < m_lateral_backlash)
        return;

    // No contact if pin is too far from sprocket center
    if (locPin.x() * locPin.x() + locPin.z() * locPin.z() > m_gear_RT * m_gear_RT)
        return;

    // Fill in contact information and add the contact to the system.
    // Express all vectors in the global frame
    collision::ChCollisionInfo contact;
    contact.modelA = m_sprocket->GetGearBody()->GetCollisionModel().get();
    contact.modelB = shoe->GetShoeBody()->GetCollisionModel().get();
    contact.shapeA = nullptr;
    contact.shapeB = nullptr;
    if (locPin.y() < 0) {
        contact.distance = m_lateral_backlash + locPin.y();
        contact.vN = dirS_abs;
    } else {
        contact.distance = m_lateral_backlash - locPin.y();
        contact.vN = -dirS_abs;
    }
    contact.vpA = locPin_abs - contact.distance * contact.vN;
    contact.vpB = locPin_abs;

    ////std::cout << "CONTACT";
    ////std::cout << "  pin: " << locPin.y();
    ////std::cout << "  delta: " << contact.distance;
    ////std::cout << "  normal: " << contact.vN;
    ////std::cout << std::endl;

    m_sprocket->GetGearBody()->GetSystem()->GetContactContainer()->AddContact(contact, m_material, m_material);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSprocketDoublePin::ChSprocketDoublePin(const std::string& name) : ChSprocket(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::shared_ptr<ChSystem::CustomCollisionCallback> ChSprocketDoublePin::GetCollisionCallback(ChTrackAssembly* track) {
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
    double lateral_backlash = GetLateralBacklash();

    // Extract parameterization of the shoe connector contact geometry.
    double shoe_len = shoe->GetConnectorLength();
    double shoe_R = shoe->GetConnectorRadius();
    ChVector<> shoe_locPin = shoe->GetLateralContactPoint();

    // Create and return the callback object. Note: this pointer will be freed by the base class.
    return chrono_types::make_shared<SprocketDoublePinContactCB>(track, 0.005, gear_nteeth, gear_RT, gear_R, gear_C,
                                                                 gear_W, GetSeparation(), shoe_len, shoe_R,
                                                                 m_lateral_contact, lateral_backlash, shoe_locPin);
}

// -----------------------------------------------------------------------------
// Create and return the sprocket gear profile.
// -----------------------------------------------------------------------------
std::shared_ptr<geometry::ChLinePath> ChSprocketDoublePin::GetProfile() const {
    auto profile = chrono_types::make_shared<geometry::ChLinePath>();

    int num_teeth = GetNumTeeth();
    double R_T = GetOuterRadius();
    double C = GetArcCenterHeight();
    double W = GetArcCenterOffset();
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

        ChVector<> p2L = ChVector<>(-C * sbeta, C * cbeta, 0) + ChVector<>(+D * cbeta, D * sbeta, 0);
        ChVector<> p2R = ChVector<>(+C * sbeta, C * cbeta, 0) + ChVector<>(-D * cbeta, D * sbeta, 0);

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
