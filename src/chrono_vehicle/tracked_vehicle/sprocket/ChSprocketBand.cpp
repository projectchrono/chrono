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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Base class for a sprocket template with gear profile composed of circular arcs
// and line segments, suitable for interaction with a continuous band track.
//
// =============================================================================

#include <cmath>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketBand.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBand.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// Utility function to calculate the center of a circle of given radius which
// passes through two given points.
static ChVector2<> CalcCircleCenter(const ChVector2<>& A, const ChVector2<>& B, double r, double direction) {
    // midpoint
    ChVector2<> C = (A + B) / 2;
    // distance between A and B
    double l = (B - A).Length();
    // distance between C and O
    double d = std::sqrt(r * r - l * l / 4);
    // slope of line AB
    double mAB = (B.y() - A.y()) / (B.x() - A.x());
    // slope of line CO (perpendicular to AB)
    double mCO = -1 / mAB;
    // x offset from C
    double x_offset = d / std::sqrt(1 + mCO * mCO);
    // y offset from C
    double y_offset = mCO * x_offset;
    // circle center
    ChVector2<> O(C.x() + direction * x_offset, C.y() + direction * y_offset);

    ////std::cout << std::endl;
    ////std::cout << "radius: " << r << std::endl;
    ////std::cout << A.x() << "  " << A.y() << std::endl;
    ////std::cout << B.x() << "  " << B.y() << std::endl;
    ////std::cout << O.x() << "  " << O.y() << std::endl;
    ////std::cout << "Check: " << (A - O).Length() - r << "  " << (B - O).Length() - r << std::endl;
    ////std::cout << std::endl;

    return O;
}

class SprocketBandContactCB : public ChSystem::CustomCollisionCallback {
  public:
    //// TODO Add in a collision envelope to the contact algorithm for NSC
    SprocketBandContactCB(ChTrackAssembly* track,  ///< containing track assembly
                          double envelope,         ///< collision detection envelope
                          int gear_nteeth,         ///< number of teeth of the sprocket gear
                          double separation        ///< separation between sprocket gears
                          )
        : m_track(track), m_envelope(envelope), m_gear_nteeth(gear_nteeth), m_separation(separation) {
        m_sprocket = std::dynamic_pointer_cast<ChSprocketBand>(m_track->GetSprocket());
        auto shoe = std::dynamic_pointer_cast<ChTrackShoeBand>(track->GetTrackShoe(0));

        // The angle between the centers of two sequential teeth on the sprocket
        m_beta = CH_C_2PI / m_sprocket->GetNumTeeth();

        double OutRad = m_sprocket->GetOuterRadius();

        // The angle measured from the center of the sprocket between the center of the tooth
        // and the outer line segment edge of the tooth's base width
        double HalfBaseWidthCordAng = std::asin((m_sprocket->GetBaseWidth() / 2) / OutRad);

        // The angle measured at the center of the sprocket between the end of one tooth profile
        // and the start of the next (angle where the profile runs along the outer radius
        // of the sprocket)
        double OuterRadArcAng = m_beta - 2 * HalfBaseWidthCordAng;

        m_gear_outer_radius_arc_angle_start = HalfBaseWidthCordAng;
        m_gear_outer_radius_arc_angle_end = HalfBaseWidthCordAng + OuterRadArcAng;

        // Vectors defining the current tooth's radial and perpendicular vectors
        ChVector2<> vec_Radial(1, 0);
        ChVector2<> vec_Perp(-vec_Radial.y(), vec_Radial.x());

        // Points defining the sprocket tooth's base width
        ChVector2<> ToothBaseWidthUpperPnt(OutRad * (std::cos(HalfBaseWidthCordAng)),
                                           OutRad * (std::sin(HalfBaseWidthCordAng)));
        ChVector2<> ToothBaseWidthLowerPnt(OutRad * (std::cos(-HalfBaseWidthCordAng)),
                                           OutRad * (std::sin(-HalfBaseWidthCordAng)));
        ChVector2<> ToothBaseWidthCtrPnt = 0.5 * (ToothBaseWidthUpperPnt + ToothBaseWidthLowerPnt);

        // Points defining the sprocket tooth's tip width
        ChVector2<> ToothTipWidthCtrPnt = ToothBaseWidthCtrPnt - m_sprocket->GetToothDepth() * vec_Radial;
        ChVector2<> ToothTipWidthUpperPnt = ToothTipWidthCtrPnt + 0.5 * m_sprocket->GetTipWidth() * vec_Perp;
        ChVector2<> ToothTipWidthLowerPnt = ToothTipWidthCtrPnt - 0.5 * m_sprocket->GetTipWidth() * vec_Perp;

        // Cache the points for the first sprocket tooth profile for the tooth arc centers, positive arc is in the CCW
        // direction for the first tooth profile and the negative arc is in the CW direction for the first tooth profile
        m_gear_center_p =
            CalcCircleCenter(ToothBaseWidthUpperPnt, ToothTipWidthUpperPnt, m_sprocket->GetArcRadius(), 1);
        m_gear_center_m =
            CalcCircleCenter(ToothBaseWidthLowerPnt, ToothTipWidthLowerPnt, m_sprocket->GetArcRadius(), 1);

        // Cache the starting (smallest) and ending (largest) arc angles for the positive sprocket tooth arc (ensuring
        // that both angles are positive)
        m_gear_center_p_start_angle = std::atan2(ToothBaseWidthUpperPnt.y() - m_gear_center_p.y(),
                                                 ToothBaseWidthUpperPnt.x() - m_gear_center_p.x());
        m_gear_center_p_start_angle =
            m_gear_center_p_start_angle < 0 ? m_gear_center_p_start_angle + CH_C_2PI : m_gear_center_p_start_angle;
        m_gear_center_p_end_angle = std::atan2(ToothTipWidthUpperPnt.y() - m_gear_center_p.y(),
                                               ToothTipWidthUpperPnt.x() - m_gear_center_p.x());
        m_gear_center_p_end_angle =
            m_gear_center_p_end_angle < 0 ? m_gear_center_p_end_angle + CH_C_2PI : m_gear_center_p_end_angle;
        if (m_gear_center_p_start_angle > m_gear_center_p_end_angle) {
            double temp = m_gear_center_p_start_angle;
            m_gear_center_p_start_angle = m_gear_center_p_end_angle;
            m_gear_center_p_end_angle = temp;
        }

        // Cache the starting (smallest) and ending (largest) arc angles for the negative sprocket tooth arc (ensuring
        // that both angles are positive)
        m_gear_center_m_start_angle = std::atan2(ToothBaseWidthLowerPnt.y() - m_gear_center_m.y(),
                                                 ToothBaseWidthLowerPnt.x() - m_gear_center_m.x());
        m_gear_center_m_start_angle =
            m_gear_center_m_start_angle < 0 ? m_gear_center_m_start_angle + CH_C_2PI : m_gear_center_m_start_angle;
        m_gear_center_m_end_angle = std::atan2(ToothTipWidthLowerPnt.y() - m_gear_center_m.y(),
                                               ToothTipWidthLowerPnt.x() - m_gear_center_m.x());
        m_gear_center_m_end_angle =
            m_gear_center_m_end_angle < 0 ? m_gear_center_m_end_angle + CH_C_2PI : m_gear_center_m_end_angle;
        if (m_gear_center_m_start_angle > m_gear_center_m_end_angle) {
            double temp = m_gear_center_m_start_angle;
            m_gear_center_m_start_angle = m_gear_center_m_end_angle;
            m_gear_center_m_end_angle = temp;
        }

        // Since the shoe has not been initalized yet, set a flag to cache all of the parameters that depend on the shoe
        // being initalized
        m_update_tread = true;
    }

    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    // Test collision between a tread segment body and the sprocket's gear profile
    void CheckTreadSegmentSprocket(std::shared_ptr<ChBody> treadsegment,  // tread segment body
                                   const ChVector<>& locS_abs             // center of sprocket (global frame)
    );

    // Test for collision between an arc on a tread segment body and the matching arc on the sprocket's gear profile
    void CheckTreadArcSprocketArc(
        std::shared_ptr<ChBody> treadsegment,  // tread segment body
        ChVector2<> sprocket_arc_center,       // Center of the sprocket profile arc in the sprocket's X-Z plane
        double sprocket_arc_angle_start,       // Starting (smallest & positive) angle for the sprocket arc
        double sprocket_arc_angle_end,         // Ending (largest & positive) angle for the sprocket arc
        double sprocket_arc_radius,            // Radius for the sprocket arc
        ChVector2<> tooth_arc_center,          // Center of the belt tooth's profile arc in the sprocket's X-Z plane
        double tooth_arc_angle_start,          // Starting (smallest & positive) angle for the belt tooth arc
        double tooth_arc_angle_end,            // Ending (largest & positive) angle for the belt tooth arc
        double tooth_arc_radius                // Radius for the tooth arc
    );

    void CheckTreadTipSprocketTip(std::shared_ptr<ChBody> treadsegment);

    void CheckSegmentCircle(std::shared_ptr<ChBody> BeltSegment,  // connector body
                            double cr,                            // circle radius
                            const ChVector<>& p1,                 // segment end point 1
                            const ChVector<>& p2                  // segment end point 2
    );

    ChTrackAssembly* m_track;                    // pointer to containing track assembly
    std::shared_ptr<ChSprocketBand> m_sprocket;  // handle to the sprocket

    double m_envelope;  // collision detection envelope

    int m_gear_nteeth;                            // sprocket gear, number of teeth
    double m_separation;                          // separation distance between sprocket gears
    double m_gear_tread_broadphase_dist_squared;  // Tread body to Sprocket quick Broadphase distance squared check

    ChVector2<> m_gear_center_p;                 // center of (+x) arc, in sprocket body x-z plane
    ChVector2<> m_gear_center_m;                 // center of (-x) arc, in sprocket body x-z plane
    double m_gear_center_p_start_angle;          // starting positive angle of the first sprocket tooth (+z ) arc
    double m_gear_center_p_end_angle;            // ending positive angle of the first sprocket tooth (+z ) arc
    double m_gear_center_m_start_angle;          // starting positive angle of the first sprocket tooth (-z ) arc
    double m_gear_center_m_end_angle;            // ending positive angle of the first sprocket tooth (-z ) arc
    double m_gear_outer_radius_arc_angle_start;  // starting positive angle of the first sprocket outer radius arc
    double m_gear_outer_radius_arc_angle_end;    // ending positive angle of the first sprocket outer radius arc

    ChVector2<> m_tread_center_p;         // center of (+x) arc, in tread body x-z plane
    ChVector2<> m_tread_center_m;         // center of (-x) arc, in tread body x-z plane
    double m_tread_center_p_start_angle;  // starting positive angle of the first tooth (+x ) arc
    double m_tread_center_p_end_angle;    // ending positive angle of the first tooth (+x ) arc
    double m_tread_center_m_start_angle;  // starting positive angle of the first tooth (-x ) arc
    double m_tread_center_m_end_angle;    // ending positive angle of the first tooth (-x ) arc
    double m_tread_arc_radius;            // radius of the tooth arc profile
    double m_tread_tip_halfwidth;         // half of the length (x direction) of the tread tooth tip
    double m_tread_tip_height;            // the height (z direction) of the trad tooth from its base line to its tip

    bool m_update_tread;  // flag to update the remaining cached contact properties on the first contact callback

    double m_beta;  // angle between sprocket teeth
};

// Add contacts between the sprocket and track shoes.
void SprocketBandContactCB::OnCustomCollision(ChSystem* system) {
    if (m_track->GetNumTrackShoes() == 0)
        return;

    // Temporary workaround since the shoe has not been intialized by the time the collision constructor is called.
    if (m_update_tread) {
        m_update_tread = false;

        auto shoe = std::dynamic_pointer_cast<ChTrackShoeBand>(m_track->GetTrackShoe(0));

        // Broadphase collision distance squared check for tread tooth to sprocket contact
        m_gear_tread_broadphase_dist_squared = std::pow(
            m_sprocket->GetOuterRadius() + sqrt(std::pow(shoe->GetToothBaseLength() / 2, 2) +
                                                std::pow(shoe->GetToothHeight() + shoe->GetWebThickness() / 2, 2)),
            2);

        m_tread_center_p = shoe->m_center_p;                        // center of (+x) arc, in tread body x-z plane
        m_tread_center_m = shoe->m_center_m;                        // center of (-x) arc, in tread body x-z plane
        m_tread_center_p_start_angle = shoe->m_center_p_arc_start;  // starting positive angle of the tooth (+x ) arc
        m_tread_center_p_end_angle = shoe->m_center_p_arc_end;      // ending positive angle of the tooth (+x ) arc
        m_tread_center_m_start_angle = shoe->m_center_m_arc_start;  // starting positive angle of the tooth (-x ) arc
        m_tread_center_m_end_angle = shoe->m_center_m_arc_end;      // ending positive angle of the tooth (-x ) arc

        m_tread_arc_radius = shoe->GetToothArcRadius();  // radius of the tooth arcs
        m_tread_tip_halfwidth =
            shoe->GetToothTipLength() / 2;  // half of the tip length (s direction) of the belt tooth
        m_tread_tip_height =
            shoe->GetToothHeight() +
            shoe->GetTreadThickness() / 2;  // height of the belt tooth profile from the tip to its base line
    }

    // Return now if collision disabled on sproket.
    if (!m_sprocket->GetGearBody()->GetCollide())
        return;

    // Sprocket gear center location (expressed in global frame)
    ChVector<> locS_abs = m_sprocket->GetGearBody()->GetPos();

    // Loop over all track shoes in the associated track
    for (size_t is = 0; is < m_track->GetNumTrackShoes(); ++is) {
        auto shoe = std::static_pointer_cast<ChTrackShoeBand>(m_track->GetTrackShoe(is));

        CheckTreadSegmentSprocket(shoe->GetShoeBody(), locS_abs);
    }
}

void SprocketBandContactCB::CheckTreadSegmentSprocket(
    std::shared_ptr<ChBody> treadsegment,  // tread segment body
    const ChVector<>& locS_abs             // center of sprocket (global frame)
) {
    // (1) Express the center of the web segment body in the sprocket frame
    ChVector<> loc = m_sprocket->GetGearBody()->TransformPointParentToLocal(treadsegment->GetPos());

    // (2) Broadphase collision test: no contact if the tread segments's center is too far from the
    // center of the gear (test performed in the sprocket's x-z plane).
    if (loc.x() * loc.x() + loc.z() * loc.z() > m_gear_tread_broadphase_dist_squared)
        return;

    // (3) Check the sprocket tooth tip to the belt tooth tip contact
    CheckTreadTipSprocketTip(treadsegment);

    // (4) Check for sprocket arc to tooth arc collisions
    // Working in the frame of the sprocket, find the candidate tooth space.
    // This is the closest tooth space to the tread center point.

    // Angle formed by 'loc' relative the sprocket center in the sprocket frame
    double angle = std::atan2(loc.z(), loc.x());
    angle = angle < 0 ? angle + CH_C_2PI : angle;
    // Find angle of closest tooth space
    double alpha = m_beta * std::round(angle / m_beta);

    // Determine the tooth x and z axes in the sprocket frame
    ChVector<> tooth_x_axis_point = m_sprocket->GetGearBody()->TransformPointParentToLocal(
        treadsegment->GetPos() + treadsegment->GetRot().GetXaxis());
    ChVector2<> tooth_x_axis(tooth_x_axis_point.x() - loc.x(), tooth_x_axis_point.z() - loc.z());
    tooth_x_axis.Normalize();
    ChVector2<> tooth_z_axis(-tooth_x_axis.y(), tooth_x_axis.x());

    // Check the positive arcs (positive sprocket arc to positive tooth arc contact)

    // Calculate the sprocket positive arc center point for the closest tooth spacing angle
    ChVector2<> sprocket_center_p(m_gear_center_p.x() * std::cos(alpha) - m_gear_center_p.y() * std::sin(alpha),
                                  m_gear_center_p.x() * std::sin(alpha) + m_gear_center_p.y() * std::cos(alpha));

    // Adjust start & end angle based on the selected sprocket tooth spacing angle
    double gear_center_p_start_angle = m_gear_center_p_start_angle + alpha;
    double gear_center_p_end_angle = m_gear_center_p_end_angle + alpha;

    // Ensure that the starting angle is between 0 & 2PI, adjusting ending angle to match
    while (gear_center_p_start_angle > CH_C_2PI) {
        gear_center_p_start_angle -= CH_C_2PI;
        gear_center_p_end_angle -= CH_C_2PI;
    }

    // Calculate the positive tooth arc center point in the sprocket frame
    ChVector<> tooth_center_p_3d = m_sprocket->GetGearBody()->TransformPointParentToLocal(
        treadsegment->GetPos() + m_tread_center_p.x() * treadsegment->GetRot().GetXaxis() +
        m_tread_center_p.y() * treadsegment->GetRot().GetZaxis());
    ChVector2<> tooth_center_p(tooth_center_p_3d.x(), tooth_center_p_3d.z());

    // Calculate the tooth arc starting point and end points in the sprocket frame
    // Don't add in the tooth_center_p point postion since it will get subtracted off to calculate the angle
    // This leads to the tooth arc angles in the sprocket frame
    ChVector2<> tooth_arc_start_point_p = m_tread_arc_radius * std::cos(m_tread_center_p_start_angle) * tooth_x_axis +
                                          m_tread_arc_radius * std::sin(m_tread_center_p_start_angle) * tooth_z_axis;
    ChVector2<> tooth_arc_end_point_p = m_tread_arc_radius * std::cos(m_tread_center_p_end_angle) * tooth_x_axis +
                                        m_tread_arc_radius * std::sin(m_tread_center_p_end_angle) * tooth_z_axis;

    double tooth_center_p_start_angle = std::atan2(tooth_arc_start_point_p.y(), tooth_arc_start_point_p.x());
    tooth_center_p_start_angle =
        tooth_center_p_start_angle < 0 ? tooth_center_p_start_angle + CH_C_2PI : tooth_center_p_start_angle;
    double tooth_center_p_end_angle = std::atan2(tooth_arc_end_point_p.y(), tooth_arc_end_point_p.x());
    tooth_center_p_end_angle =
        tooth_center_p_end_angle < 0 ? tooth_center_p_end_angle + CH_C_2PI : tooth_center_p_end_angle;

    if (tooth_center_p_end_angle < tooth_center_p_start_angle) {
        tooth_center_p_end_angle += CH_C_2PI;
    }

    double temp = m_sprocket->GetArcRadius();
    CheckTreadArcSprocketArc(treadsegment, sprocket_center_p, gear_center_p_start_angle, gear_center_p_end_angle,
                             m_sprocket->GetArcRadius(), tooth_center_p, tooth_center_p_start_angle,
                             tooth_center_p_end_angle, m_tread_arc_radius);

    // Check the negative arcs (negative sprocket arc to negative tooth arc contact)

    // Calculate the sprocket negative arc center point for the closest tooth spacing angle
    ChVector2<> sprocket_center_m(m_gear_center_m.x() * std::cos(alpha) - m_gear_center_m.y() * std::sin(alpha),
                                  m_gear_center_m.x() * std::sin(alpha) + m_gear_center_m.y() * std::cos(alpha));

    // Adjust start & end angle based on the selected sprocket tooth spacing angle
    double gear_center_m_start_angle = m_gear_center_m_start_angle + alpha;
    double gear_center_m_end_angle = m_gear_center_m_end_angle + alpha;

    // Ensure that the starting angle is between 0 & 2PI, adjusting ending angle to match
    while (gear_center_m_start_angle > CH_C_2PI) {
        gear_center_m_start_angle -= CH_C_2PI;
        gear_center_m_end_angle -= CH_C_2PI;
    }

    // Calculate the positive tooth arc center point in the sprocket frame
    ChVector<> tooth_center_m_3d = m_sprocket->GetGearBody()->TransformPointParentToLocal(
        treadsegment->GetPos() + m_tread_center_m.x() * treadsegment->GetRot().GetXaxis() +
        m_tread_center_m.y() * treadsegment->GetRot().GetZaxis());
    ChVector2<> tooth_center_m(tooth_center_m_3d.x(), tooth_center_m_3d.z());

    // Calculate the tooth arc starting point and end points in the sprocket frame
    // Don't add in the tooth_center_m point postion since it will get subtracted off to calculate the angle
    // This leads to the tooth arc angles in the sprocket frame
    ChVector2<> tooth_arc_start_point_m = m_tread_arc_radius * std::cos(m_tread_center_m_start_angle) * tooth_x_axis +
                                          m_tread_arc_radius * std::sin(m_tread_center_m_start_angle) * tooth_z_axis;
    ChVector2<> tooth_arc_end_point_m = m_tread_arc_radius * std::cos(m_tread_center_m_end_angle) * tooth_x_axis +
                                        m_tread_arc_radius * std::sin(m_tread_center_m_end_angle) * tooth_z_axis;

    double tooth_center_m_start_angle = std::atan2(tooth_arc_start_point_m.y(), tooth_arc_start_point_m.x());
    tooth_center_m_start_angle =
        tooth_center_m_start_angle < 0 ? tooth_center_m_start_angle + CH_C_2PI : tooth_center_m_start_angle;
    double tooth_center_m_end_angle = std::atan2(tooth_arc_end_point_m.y(), tooth_arc_end_point_m.x());
    tooth_center_m_end_angle =
        tooth_center_m_end_angle < 0 ? tooth_center_m_end_angle + CH_C_2PI : tooth_center_m_end_angle;

    if (tooth_center_m_end_angle < tooth_center_m_start_angle) {
        tooth_center_m_end_angle += CH_C_2PI;
    }

    CheckTreadArcSprocketArc(treadsegment, sprocket_center_m, gear_center_m_start_angle, gear_center_m_end_angle,
                             m_sprocket->GetArcRadius(), tooth_center_m, tooth_center_m_start_angle,
                             tooth_center_m_end_angle, m_tread_arc_radius);
}

void SprocketBandContactCB::CheckTreadTipSprocketTip(std::shared_ptr<ChBody> treadsegment) {
    // Check the tooth tip to outer sprocket arc
    // Check to see if any of the points are within the angle of an outer sprocket arc
    // If so, clip the part of the tip line segment that is out of the arc, if need and run a line segment to circle
    // check
    ChVector<> tooth_tip_p = m_sprocket->GetGearBody()->TransformPointParentToLocal(
        treadsegment->GetPos() + m_tread_tip_halfwidth * treadsegment->GetRot().GetXaxis() +
        m_tread_tip_height * treadsegment->GetRot().GetZaxis());
    tooth_tip_p.y() = 0;
    ChVector<> tooth_tip_m = m_sprocket->GetGearBody()->TransformPointParentToLocal(
        treadsegment->GetPos() - m_tread_tip_halfwidth * treadsegment->GetRot().GetXaxis() +
        m_tread_tip_height * treadsegment->GetRot().GetZaxis());
    tooth_tip_m.y() = 0;

    double tooth_tip_p_angle = std::atan2(tooth_tip_p.z(), tooth_tip_p.x());
    tooth_tip_p_angle = tooth_tip_p_angle < 0 ? tooth_tip_p_angle + CH_C_2PI : tooth_tip_p_angle;
    double tooth_tip_m_angle = std::atan2(tooth_tip_m.z(), tooth_tip_m.x());
    tooth_tip_m_angle = tooth_tip_m_angle < 0 ? tooth_tip_m_angle + CH_C_2PI : tooth_tip_m_angle;

    double tooth_tip_p_angle_adjust = m_beta * std::floor(tooth_tip_p_angle / m_beta);
    tooth_tip_p_angle -= tooth_tip_p_angle_adjust;  // Adjust the angle so that it lies within 1 tooth spacing so it is
                                                    // easier to check if it is on an outer sprocket arc
    double tooth_tip_m_angle_adjust = m_beta * std::floor(tooth_tip_m_angle / m_beta);
    tooth_tip_m_angle -= tooth_tip_m_angle_adjust;  // Adjust the angle so that it lies within 1 tooth spacing so it is
                                                    // easier to check if it is on an outer sprocket arc

    if (((tooth_tip_p_angle >= m_gear_outer_radius_arc_angle_start) &&
         (tooth_tip_p_angle <= m_gear_outer_radius_arc_angle_end)) ||
        ((tooth_tip_m_angle >= m_gear_outer_radius_arc_angle_start) &&
         (tooth_tip_m_angle <= m_gear_outer_radius_arc_angle_end))) {
        // Check for possible collisions with outer sprocket radius
        // Clamp the tooth points within the sprocket outer radius arc to prevent false contacts from being generated

        if (!((tooth_tip_p_angle >= m_gear_outer_radius_arc_angle_start) &&
              (tooth_tip_p_angle <= m_gear_outer_radius_arc_angle_end))) {
            // Clip tooth_tip_p so that it lies within the outer arc section of the sprocket profile since there is no
            // contact after this point
            double clip_angle_start = m_gear_outer_radius_arc_angle_start +
                                      tooth_tip_m_angle_adjust;  // Use m tip point, since that is in the correct arc
            double clip_angle_end = m_gear_outer_radius_arc_angle_end + tooth_tip_m_angle_adjust;

            ChVector<> vec_tooth = tooth_tip_p - tooth_tip_m;

            double a = vec_tooth.x();
            double b = -std::cos(clip_angle_end);
            double c = vec_tooth.z();
            double d = -std::sin(clip_angle_end);

            double alpha = (1 / (a * d - b * c)) * (-d * tooth_tip_m.x() + b * tooth_tip_m.z());
            ChClampValue(alpha, 0.0, 1.0);

            CheckSegmentCircle(treadsegment, m_sprocket->GetOuterRadius(), tooth_tip_m + alpha * vec_tooth,
                               tooth_tip_m);
        } else if (!((tooth_tip_m_angle >= m_gear_outer_radius_arc_angle_start) &&
                     (tooth_tip_m_angle <= m_gear_outer_radius_arc_angle_end))) {
            // Clip tooth_tip_m so that it lies within the outer arc section of the sprocket profile since there is no
            // contact after this point
            double clip_angle_start = m_gear_outer_radius_arc_angle_start +
                                      tooth_tip_p_angle_adjust;  // Use p tip point, since that is in the correct arc
            double clip_angle_end = m_gear_outer_radius_arc_angle_end + tooth_tip_p_angle_adjust;

            ChVector<> vec_tooth = tooth_tip_m - tooth_tip_p;

            double a = vec_tooth.x();
            double b = -std::cos(clip_angle_start);
            double c = vec_tooth.z();
            double d = -std::sin(clip_angle_start);

            double alpha = (1 / (a * d - b * c)) * (-d * tooth_tip_p.x() + b * tooth_tip_p.z());
            ChClampValue(alpha, 0.0, 1.0);

            CheckSegmentCircle(treadsegment, m_sprocket->GetOuterRadius(), tooth_tip_p + alpha * vec_tooth,
                               tooth_tip_p);
        } else {
            // No Tooth Clipping Needed
            CheckSegmentCircle(treadsegment, m_sprocket->GetOuterRadius(), tooth_tip_p, tooth_tip_m);
        }
    }
}

void SprocketBandContactCB::CheckTreadArcSprocketArc(std::shared_ptr<ChBody> treadsegment,
                                                     ChVector2<> sprocket_arc_center,
                                                     double sprocket_arc_angle_start,
                                                     double sprocket_arc_angle_end,
                                                     double sprocket_arc_radius,
                                                     ChVector2<> tooth_arc_center,
                                                     double tooth_arc_angle_start,
                                                     double tooth_arc_angle_end,
                                                     double tooth_arc_radius) {
    // Find the angle from the sprocket arc center through the tooth arc center.  If the angle lies within
    // the sprocket arc, use the point on the sprocket arc at that angle as the sprocket collision point.
    // If it does not lie within the arc, determine which sprocket end point is closest to that angle and use
    // that point as the sprocket collision point.

    double sprocket_contact_angle =
        std::atan2(tooth_arc_center.y() - sprocket_arc_center.y(), tooth_arc_center.x() - sprocket_arc_center.x());
    while (sprocket_contact_angle < sprocket_arc_angle_start) {
        sprocket_contact_angle += CH_C_2PI;
    }

    ChVector2<> sprocket_collision_point =
        sprocket_arc_center +
        sprocket_arc_radius * ChVector2<>(std::cos(sprocket_contact_angle), std::sin(sprocket_contact_angle));
    if (sprocket_contact_angle >= sprocket_arc_angle_end) {
        // Lies outside of the sprocket arc, so find the sprocket end point that is closest to the  the arc since the
        // sprocket_contact_angle has to be >= sprocket_arc_angle_start
        ChVector2<> arc_start_point =
            sprocket_arc_center +
            sprocket_arc_radius * ChVector2<>(std::cos(sprocket_arc_angle_start), std::sin(sprocket_arc_angle_start));
        ChVector2<> arc_start_to_collision_point = sprocket_collision_point - arc_start_point;
        ChVector2<> arc_end_point =
            sprocket_arc_center +
            sprocket_arc_radius * ChVector2<>(std::cos(sprocket_arc_angle_end), std::sin(sprocket_arc_angle_end));
        ChVector2<> arc_end_to_collision_point = sprocket_collision_point - arc_end_point;

        sprocket_collision_point = (arc_start_to_collision_point.Length2() <= arc_end_to_collision_point.Length2())
                                       ? arc_start_point
                                       : arc_end_point;
    }

    // Find the angle from the tooth arc center through the sprocket collision point. If the angle lies
    // within the tooth arc, use the point on the tooth arc at that angle as the tooth collision point. If
    // the angle does not lie within the arc, then no contact exists.
    double tooth_contact_angle = std::atan2(sprocket_collision_point.y() - tooth_arc_center.y(),
                                            sprocket_collision_point.x() - tooth_arc_center.x());
    tooth_contact_angle -=
        CH_C_2PI;  // Ensure that tooth_contact_angle is negative and this less than tooth_arc_angle_start
    while (tooth_contact_angle < tooth_arc_angle_start) {
        tooth_contact_angle += CH_C_2PI;
    }

    if (tooth_contact_angle > tooth_arc_angle_end) {
        return;  // Tooth collision point does not lie on the tooth arc
    }

    ChVector2<> tooth_collision_point =
        tooth_arc_center + tooth_arc_radius * ChVector2<>(std::cos(tooth_contact_angle), std::sin(tooth_contact_angle));

    // Subtract the tooth arc radius from distance from tooth arc center to the sprocket collision point.
    // If this distance is positive, then no contact exists.  Otherwise this is the collision distance.
    ChVector2<> sprocket_collision_point_to_tooth_arc_center = tooth_arc_center - sprocket_collision_point;
    double collision_distance = sprocket_collision_point_to_tooth_arc_center.Length() - tooth_arc_radius;

    if (collision_distance >= 0) {
        return;  // tooth arc is inside the sprocket arc, so no collision
    }

    // Find the normalized vector from the tooth collision point through the sprocket collision point
    // as well as the tooth arc center.  This is the collision normal vector.
    sprocket_collision_point_to_tooth_arc_center.Normalize();

    ChVector<> normal(sprocket_collision_point_to_tooth_arc_center.x(), 0,
                      sprocket_collision_point_to_tooth_arc_center.y());
    ChVector<> pt_gear(sprocket_collision_point.x(), 0, sprocket_collision_point.y());
    ChVector<> pt_tooth(tooth_collision_point.x(), 0, tooth_collision_point.y());

    // Fill in contact information and add the contact to the system.
    // Express all vectors in the global frame
    collision::ChCollisionInfo contact;
    contact.modelA = m_sprocket->GetGearBody()->GetCollisionModel().get();
    contact.modelB = treadsegment->GetCollisionModel().get();
    contact.vN = m_sprocket->GetGearBody()->TransformDirectionLocalToParent(normal);
    contact.vpA = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_gear);
    contact.vpB = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_tooth);
    contact.distance = collision_distance;
    ////contact.eff_radius = sprocket_arc_radius;  //// TODO: take into account tooth_arc_radius?

    m_sprocket->GetGearBody()->GetSystem()->GetContactContainer()->AddContact(contact);
}

// Working in the (x-z) plane, perform a 2D collision test between the circle of radius 'cr'
// centered at the origin (on the sprocket body) and the line segment with endpoints 'p1' and 'p2'
// (on the Belt Segement body).
void SprocketBandContactCB::CheckSegmentCircle(std::shared_ptr<ChBody> BeltSegment,  // connector body
                                               double cr,                            // circle radius
                                               const ChVector<>& p1,                 // segment end point 1
                                               const ChVector<>& p2                  // segment end point 2
) {
    // Find closest point on segment to circle center: X = p1 + t * (p2-p1)
    ChVector<> s = p2 - p1;
    double t = Vdot(-p1, s) / Vdot(s, s);
    ChClampValue(t, 0.0, 1.0);

    ChVector<> pt_segement = p1 + t * s;

    // No contact if circle center is too far from segment.
    double dist2 = pt_segement.Length2();
    if (dist2 >= cr * cr)
        return;

    // Generate contact information (still in sprocket frame)
    double dist = std::sqrt(dist2);
    ChVector<> normal = pt_segement / dist;
    ChVector<> pt_gear = cr * normal;

    // Fill in contact information and add the contact to the system.
    // Express all vectors in the global frame
    collision::ChCollisionInfo contact;
    contact.modelA = m_sprocket->GetGearBody()->GetCollisionModel().get();
    contact.modelB = BeltSegment->GetCollisionModel().get();
    contact.vN = m_sprocket->GetGearBody()->TransformDirectionLocalToParent(normal);
    contact.vpA = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_gear);
    contact.vpB = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_segement);
    contact.distance = dist - cr;
    ////contact.eff_radius = cr;

    m_sprocket->GetGearBody()->GetSystem()->GetContactContainer()->AddContact(contact);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSprocketBand::ChSprocketBand(const std::string& name) : ChSprocket(name) {}

// -----------------------------------------------------------------------------
// Initialize this sprocket subsystem.
// -----------------------------------------------------------------------------
void ChSprocketBand::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                const ChVector<>& location,
                                ChTrackAssembly* track) {
    // Invoke the base class method
    ChSprocket::Initialize(chassis, location, track);

    double radius = GetOuterRadius();
    double width = 0.5 * (GetGuideWheelWidth() - GetGuideWheelGap());
    double offset = 0.25 * (GetGuideWheelWidth() + GetGuideWheelGap());

    m_gear->GetCollisionModel()->ClearModel();

    m_gear->GetCollisionModel()->SetFamily(TrackedCollisionFamily::WHEELS);
    m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::IDLERS);

    m_gear->GetCollisionModel()->AddCylinder(radius, radius, width / 2, ChVector<>(0, offset, 0));
    m_gear->GetCollisionModel()->AddCylinder(radius, radius, width / 2, ChVector<>(0, -offset, 0));

    m_gear->GetCollisionModel()->BuildModel();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSprocketBand::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    ChSprocket::AddVisualizationAssets(vis);

    double radius = GetOuterRadius();
    double width = GetGuideWheelWidth();
    double gap = GetGuideWheelGap();

    auto cyl_1 = std::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    cyl_1->GetCylinderGeometry().p2 = ChVector<>(0, gap / 2, 0);
    cyl_1->GetCylinderGeometry().rad = radius;
    m_gear->AddAsset(cyl_1);

    auto cyl_2 = std::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = ChVector<>(0, -width / 2, 0);
    cyl_2->GetCylinderGeometry().p2 = ChVector<>(0, -gap / 2, 0);
    cyl_2->GetCylinderGeometry().rad = radius;
    m_gear->AddAsset(cyl_2);

    auto tex = std::make_shared<ChTexture>();
    tex->SetTextureFilename(chrono::GetChronoDataFile("greenwhite.png"));
    m_gear->AddAsset(tex);
}

void ChSprocketBand::RemoveVisualizationAssets() {
    m_gear->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSystem::CustomCollisionCallback* ChSprocketBand::GetCollisionCallback(ChTrackAssembly* track) {
    // Check compatibility between this type of sprocket and the track shoes.
    // We expect track shoes of type ChTrackShoeBand.
    auto shoe = std::dynamic_pointer_cast<ChTrackShoeBand>(track->GetTrackShoe(0));
    assert(shoe);

    // Extract parameterization of sprocket profile
    int sprocket_nteeth = GetNumTeeth();

    // Create and return the callback object. Note: this pointer will be freed by the base class.
    return new SprocketBandContactCB(track, 0.005, GetNumTeeth(), GetSeparation());
}

// -----------------------------------------------------------------------------
// Create and return the sprocket gear profile.
// -----------------------------------------------------------------------------
std::shared_ptr<geometry::ChLinePath> ChSprocketBand::GetProfile() {
    auto profile = std::make_shared<geometry::ChLinePath>();

    int num_teeth = GetNumTeeth();
    double OutRad = GetOuterRadius();
    double BaseLen = GetBaseWidth();
    double TipLen = GetTipWidth();
    double Depth = GetToothDepth();
    double ArcRad = GetArcRadius();

    ChVector<> SprocketCtr(0.0, 0.0, 0.0);

    // The angle between the centers of two sequential teeth on the sprocket
    double EntireToothArcAng = CH_C_2PI / num_teeth;

    // The angle measured from the center of the sprocket between the center of the tooth
    // and the outer line segment edge of the tooth's base width
    double HalfBaseWidthCordAng = std::asin((BaseLen / 2) / OutRad);

    // The angle measured at the center of the sprocket bewteen the end of one tooth profile
    // and the start of the next (angle where the profile runs along the outer radius
    // of the sprocket)
    double OuterRadArcAng = EntireToothArcAng - 2 * HalfBaseWidthCordAng;

    for (int i = 0; i < num_teeth; ++i) {
        double CtrAng = -i * EntireToothArcAng;

        // Vectors defining the current tooth's radial and perpendicular vectors
        ChVector<> vec_Radial(std::cos(CtrAng), std::sin(CtrAng), 0);
        ChVector<> vec_Perp(-vec_Radial.y(), vec_Radial.x(), 0);

        // Points defining the sprocket tooth's base width
        ChVector<> ToothBaseWidthUpperPnt(OutRad * (std::cos(CtrAng + HalfBaseWidthCordAng)),
                                          OutRad * (std::sin(CtrAng + HalfBaseWidthCordAng)), 0);
        ChVector<> ToothBaseWidthLowerPnt(OutRad * (std::cos(CtrAng - HalfBaseWidthCordAng)),
                                          OutRad * (std::sin(CtrAng - HalfBaseWidthCordAng)), 0);
        ChVector<> ToothBaseWidthCtrPnt = 0.5 * (ToothBaseWidthUpperPnt + ToothBaseWidthLowerPnt);

        // Points defining the sprocket tooth's tip width
        ChVector<> ToothTipWidthCtrPnt = ToothBaseWidthCtrPnt - Depth * vec_Radial;
        ChVector<> ToothTipWidthUpperPnt = ToothTipWidthCtrPnt + 0.5 * TipLen * vec_Perp;
        ChVector<> ToothTipWidthLowerPnt = ToothTipWidthCtrPnt - 0.5 * TipLen * vec_Perp;

        // Points defining the tip to tooth base chord midpoint for the concave tooth profile
        double ToothArcChordLen = std::sqrt(Depth * Depth + std::pow((BaseLen - TipLen) / 2, 2));
        ChVector<> ToothUpperChordMidPnt = 0.5 * (ToothBaseWidthUpperPnt + ToothTipWidthUpperPnt);
        ChVector<> ToothLowerChordMidPnt = 0.5 * (ToothBaseWidthLowerPnt + ToothTipWidthLowerPnt);

        // Define the distance to project perpendicular to the tooth arc chord  at the chord's center
        // to reach the center of the tooth arc circles
        double ToothArcChordRadiusSegmentLen = sqrt(ArcRad * ArcRad - std::pow(ToothArcChordLen / 2, 2));

        // Define the arc centers for the tooth concave profiles
        ChVector<> vec_Chord1 = ToothBaseWidthUpperPnt - ToothTipWidthUpperPnt;
        vec_Chord1.Normalize();
        ChVector<> vec_Chord1Perp(-vec_Chord1.y(), vec_Chord1.x(), 0);

        ChVector<> vec_Chord2 = ToothBaseWidthLowerPnt - ToothTipWidthLowerPnt;
        vec_Chord2.Normalize();
        ChVector<> vec_Chord2Perp(-vec_Chord2.y(), vec_Chord2.x(), 0);

        ChVector<> ToothUpperArcCtrPnt = ToothUpperChordMidPnt - vec_Chord1Perp * ToothArcChordRadiusSegmentLen;
        ChVector<> ToothLowerArcCtrPnt = ToothLowerChordMidPnt + vec_Chord2Perp * ToothArcChordRadiusSegmentLen;

        // Calculate the starting and ending arc angles for each concave tooth profile
        double UpperArc_Angle1 = std::atan2(ToothBaseWidthUpperPnt.y() - ToothUpperArcCtrPnt.y(),
                                            ToothBaseWidthUpperPnt.x() - ToothUpperArcCtrPnt.x());
        double UpperArc_Angle2 = std::atan2(ToothTipWidthUpperPnt.y() - ToothUpperArcCtrPnt.y(),
                                            ToothTipWidthUpperPnt.x() - ToothUpperArcCtrPnt.x());

        // Ensure that Angle1 is positive
        UpperArc_Angle1 = UpperArc_Angle1 < 0 ? UpperArc_Angle1 + CH_C_2PI : UpperArc_Angle1;
        // Ensure that Angle2 is greater than Angle1
        while (UpperArc_Angle2 < UpperArc_Angle1) {
            UpperArc_Angle2 += CH_C_2PI;
        }

        double LowerArc_Angle1 = std::atan2(ToothTipWidthLowerPnt.y() - ToothLowerArcCtrPnt.y(),
                                            ToothTipWidthLowerPnt.x() - ToothLowerArcCtrPnt.x());
        double LowerArc_Angle2 = std::atan2(ToothBaseWidthLowerPnt.y() - ToothLowerArcCtrPnt.y(),
                                            ToothBaseWidthLowerPnt.x() - ToothLowerArcCtrPnt.x());

        // Ensure that Angle1 is positive
        LowerArc_Angle1 = LowerArc_Angle1 < 0 ? LowerArc_Angle1 + CH_C_2PI : LowerArc_Angle1;
        // Ensure that Angle2 is greater than Angle1
        while (LowerArc_Angle2 < LowerArc_Angle1) {
            LowerArc_Angle2 += CH_C_2PI;
        }

        // Calculate the starting and ending arc angles for profile along the sprocket's outer radius
        double OuterArc_Angle1 = CtrAng - HalfBaseWidthCordAng;
        double OuterArc_Angle2 = OuterArc_Angle1 - OuterRadArcAng;

        // Start the profile geometry with the upper concave tooth arc
        geometry::ChLineArc arc1(ChCoordsys<>(ToothUpperArcCtrPnt), ArcRad, UpperArc_Angle1, UpperArc_Angle2, true);
        // Next is the straight segment for the tooth tip
        geometry::ChLineSegment seg2(ToothTipWidthUpperPnt, ToothTipWidthLowerPnt);
        // Next is the lower concave tooth arc
        geometry::ChLineArc arc3(ChCoordsys<>(ToothLowerArcCtrPnt), ArcRad, LowerArc_Angle1, LowerArc_Angle2, true);
        // Finally is the arc segment that runs along the sprocket's outer radius to the start of the next tooth profile
        geometry::ChLineArc arc4(ChCoordsys<>(SprocketCtr), OutRad, OuterArc_Angle1, OuterArc_Angle2);

        // Add this tooth segments profile to the sprocket profile
        profile->AddSubLine(arc1);
        profile->AddSubLine(seg2);
        profile->AddSubLine(arc3);
        profile->AddSubLine(arc4);
    }

    return profile;
}

}  // end namespace vehicle
}  // end namespace chrono
