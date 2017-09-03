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
// Base class for a sprocket template with gear profile composed of circular arcs
// and line segments, suitable for interaction with a continuous band track.
//
// =============================================================================

//#include <cmath>
//
//#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketCB.h"
//#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeRigidCB.h"
//#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"
//
//namespace chrono {
//namespace vehicle {
//
//// -----------------------------------------------------------------------------
//// -----------------------------------------------------------------------------
//class SprocketCBContactCallback : public ChSystem::CustomCollisionCallback {
//  public:
//    SprocketCBContactCallback(ChTrackAssembly* track,  ///< containing track assembly
//                              double envelope,         ///< collision detection envelope
//                              int spkt_nteeth,         ///< number of teeth of the sprocket gear
//                              double spkt_OutRad,
//                              double spkt_BaseLen,
//                              double spkt_TipLen,
//                              double spkt_Depth,
//                              double spkt_ArcRad,
//                              double separation,       ///< separation between sprocket gears
//                              ChVectorDynamic<> belt_WebSegLens,
//                              double belt_BaseLen,
//                              double belt_TipLen,
//                              double belt_Depth,
//                              double belt_ArcRad,
//                              double belt_WebLen
//                              )
//                              : m_track(track), 
//                                m_envelope(envelope), 
//                                m_sprocket(m_track->GetSprocket()),
//                                m_spkt_nteeth(spkt_nteeth), 
//                                m_spkt_OutRad(spkt_OutRad),
//                                m_spkt_BaseLen(spkt_BaseLen),
//                                m_spkt_TipLen(spkt_TipLen),
//                                m_spkt_Depth(spkt_Depth),
//                                m_spkt_ArcRad(spkt_ArcRad),
//                                m_separation(separation),
//                                m_belt_WebSegLens(belt_WebSegLens),
//                                m_belt_BaseLen(belt_BaseLen),
//                                m_belt_TipLen(belt_TipLen),
//                                m_belt_Depth(belt_Depth),
//                                m_belt_ArcRad(belt_ArcRad),
//                                m_belt_WebLen(belt_WebLen)
//    {
//        //// TODO
//    }
//
//    virtual void OnCustomCollision(ChSystem* system) override;
//
//  private:
//    ChTrackAssembly* m_track;                // pointer to containing track assembly
//    std::shared_ptr<ChSprocket> m_sprocket;  // handle to the sprocket
//
//    double m_envelope;      // collision detection envelope
//    int m_spkt_nteeth;      // sprocket gear, number of teeth
//    double m_spkt_OutRad;   // sprocket gear, profile outer radius
//    double m_spkt_BaseLen;  // sprocket gear, tooth base width
//    double m_spkt_TipLen;   // sprocket gear, tooth tip width
//    double m_spkt_Depth;    // sprocket gear, tooth depth from tip to base
//    double m_spkt_ArcRad;   // sprocket gear, tooth concave arc radius
//    double m_separation;    // separation distance between sprocket gears
//
//    ChVectorDynamic<> m_belt_WebSegLens;  // belt, the fractional length of each belt segment
//    double m_belt_BaseLen;  // belt, tooth base width
//    double m_belt_TipLen;   // belt, tooth tip width
//    double m_belt_Depth;    // belt, tooth depth from tip to base
//    double m_belt_ArcRad;   // belt, tooth convex arc radius
//    double m_belt_WebLen;   // belt, total web length
//};
//
//// Add contacts between the sprocket and track shoes.
//void SprocketCBContactCallback::OnCustomCollision(ChSystem* system) {
//    // Return now if collision disabled on sproket.
//    if (!m_sprocket->GetGearBody()->GetCollide())
//        return;
//
//    // Sprocket gear center location (expressed in global frame)
//    ChVector<> locS_abs = m_sprocket->GetGearBody()->GetPos();
//
//    // Loop over all track shoes in the associated track
//    for (size_t is = 0; is < m_track->GetNumTrackShoes(); ++is) {
//        auto shoe = std::static_pointer_cast<ChTrackShoeRigidCB>(m_track->GetTrackShoe(is));
//
//        //// TODO
//    }
//}
//
//// -----------------------------------------------------------------------------
//// -----------------------------------------------------------------------------
//ChSprocketCB::ChSprocketCB(const std::string& name) : ChSprocket(name) {}
//
//// -----------------------------------------------------------------------------
//// -----------------------------------------------------------------------------
//ChSystem::CustomCollisionCallback* ChSprocketCB::GetCollisionCallback(ChTrackAssembly* track) {
//    // Check compatibility between this type of sprocket and the track shoes.
//    // We expect track shoes of type ChTrackShoeRigidCB.
//    auto shoe = std::dynamic_pointer_cast<ChTrackShoeRigidCB>(track->GetTrackShoe(0));
//    assert(shoe);
//
//    // Extract parameterization of sprocket profile
//    int spkt_nteeth = GetNumTeeth();
//    double spkt_OutRad = GetOuterRadius();
//    double spkt_BaseLen = GetBaseWidth();
//    double spkt_TipLen = GetTipWidth();
//    double spkt_Depth = GetToothDepth();
//    double spkt_ArcRad = GetArcRadius();
//
//    // Extract parameterization of the ChTrackShoeRigidCB contact geometry.
//    //ChVectorDynamic<> belt_WebSegLens = shoe->GetWebSegmentFracLength();
//    //double belt_BaseLen = shoe->GetBaseWidth();
//    //double belt_TipLen = shoe->GetTipWidth();
//    //double belt_Depth = shoe->GetToothDepth();
//    //double belt_ArcRad = shoe->GetArcRadius();
//    //double belt_WebLen = shoe->GetWebLength();
//    ChVectorDynamic<> belt_WebSegLens(0);
//    double belt_BaseLen = 0;
//    double belt_TipLen = 0;
//    double belt_Depth = 0;
//    double belt_ArcRad = 0;
//    double belt_WebLen = 0;
//
//    // Create and return the callback object. Note: this pointer will be freed by the base class.
//    return new SprocketCBContactCallback(track, 0.005, spkt_nteeth, spkt_OutRad, spkt_BaseLen, spkt_TipLen, spkt_Depth, spkt_ArcRad,
//        GetSeparation(), belt_WebSegLens, belt_BaseLen, belt_TipLen, belt_Depth, belt_ArcRad, belt_WebLen);
//}
//
//// -----------------------------------------------------------------------------
//// Create and return the sprocket gear profile.
//// -----------------------------------------------------------------------------
//std::shared_ptr<geometry::ChLinePath> ChSprocketCB::GetProfile() {
//    auto profile = std::make_shared<geometry::ChLinePath>();
//
//    int num_teeth = GetNumTeeth();
//    double OutRad = GetOuterRadius();
//    double BaseLen = GetBaseWidth();
//    double TipLen = GetTipWidth();
//    double Depth = GetToothDepth();
//    double ArcRad = GetArcRadius();
//
//    ChVector<> SprocketCtr(0.0, 0.0, 0.0);
//
//    // The angle between the centers of two sequential teeth on the sprocket
//    double EntireToothArcAng = CH_C_2PI / num_teeth;
//
//    // The angle measured from the center of the sprocket between the center of the tooth
//    // and the outer line segment edge of the tooth's base width
//    double HalfBaseWidthCordAng = std::asin((BaseLen / 2) / OutRad);
//
//    // The angle measured at the center of the sprocket bewteen the end of one tooth profile
//    // and the start of the next (angle where the profile runs along the outer radius
//    // of the sprocket)
//    double OuterRadArcAng = EntireToothArcAng - 2 * HalfBaseWidthCordAng;
//
//    for (int i = 0; i < num_teeth; ++i) {
//        double CtrAng = -i*EntireToothArcAng;
//
//        // Vectors defining the current tooth's radial and perpendicular vectors
//        ChVector<> vec_Radial(std::cos(CtrAng), std::sin(CtrAng), 0);
//        ChVector<> vec_Perp(-vec_Radial.y(), vec_Radial.x(), 0);
//
//        // Points defining the sprocket tooth's base width
//        ChVector<> ToothBaseWidthUpperPnt(OutRad*(std::cos(CtrAng + HalfBaseWidthCordAng)), 
//            OutRad*(std::sin(CtrAng + HalfBaseWidthCordAng)), 
//            0);
//        ChVector<> ToothBaseWidthLowerPnt(OutRad*(std::cos(CtrAng - HalfBaseWidthCordAng)), 
//            OutRad*(std::sin(CtrAng - HalfBaseWidthCordAng)), 
//            0);
//        ChVector<> ToothBaseWidthCtrPnt = 0.5*(ToothBaseWidthUpperPnt + ToothBaseWidthLowerPnt);
//
//        // Points defining the sprocket tooth's tip width
//        ChVector<> ToothTipWidthCtrPnt = ToothBaseWidthCtrPnt - Depth*vec_Radial;
//        ChVector<> ToothTipWidthUpperPnt = ToothTipWidthCtrPnt + 0.5*TipLen*vec_Perp;
//        ChVector<> ToothTipWidthLowerPnt = ToothTipWidthCtrPnt - 0.5*TipLen*vec_Perp;
//
//        // Points defining the tip to tooth base chord midpoint for the concave tooth profile
//        double ToothArcChordLen = std::sqrt(Depth*Depth + std::pow((BaseLen - TipLen) / 2, 2));
//        ChVector<> ToothUpperChordMidPnt = 0.5*(ToothBaseWidthUpperPnt + ToothTipWidthUpperPnt);
//        ChVector<> ToothLowerChordMidPnt = 0.5*(ToothBaseWidthLowerPnt + ToothTipWidthLowerPnt);
//
//        // Define the distance to project perpendicular to the tooth arc chord  at the chord's center
//        // to reach the center of the tooth arc circles
//        double ToothArcChordRadiusSegmentLen = sqrt(ArcRad*ArcRad - std::pow(ToothArcChordLen / 2, 2));
//
//        //Define the arc centers for the tooth concave profiles
//        ChVector<> vec_Chord1 = ToothBaseWidthUpperPnt - ToothTipWidthUpperPnt;
//        vec_Chord1.Normalize();
//        ChVector<> vec_Chord1Perp(-vec_Chord1.y(), vec_Chord1.x(), 0);
//
//        ChVector<> vec_Chord2 = ToothBaseWidthLowerPnt - ToothTipWidthLowerPnt;
//        vec_Chord2.Normalize();
//        ChVector<> vec_Chord2Perp(-vec_Chord2.y(), vec_Chord2.x(), 0);
//
//        ChVector<> ToothUpperArcCtrPnt = ToothUpperChordMidPnt - vec_Chord1Perp*ToothArcChordRadiusSegmentLen;
//        ChVector<> ToothLowerArcCtrPnt = ToothLowerChordMidPnt + vec_Chord2Perp*ToothArcChordRadiusSegmentLen;
//
//        // Calculate the starting and ending arc angles for each concave tooth profile
//        double UpperArc_Angle1 = std::atan2(ToothBaseWidthUpperPnt.y() - ToothUpperArcCtrPnt.y(), ToothBaseWidthUpperPnt.x() - ToothUpperArcCtrPnt.x());
//        double UpperArc_Angle2 = std::atan2(ToothTipWidthUpperPnt.y() - ToothUpperArcCtrPnt.y(), ToothTipWidthUpperPnt.x() - ToothUpperArcCtrPnt.x());
//
//        // Ensure that Angle1 is positive
//        UpperArc_Angle1 = UpperArc_Angle1 < 0 ? UpperArc_Angle1 + CH_C_2PI : UpperArc_Angle1; 
//        // Ensure that Angle2 is greater than Angle1
//        while (UpperArc_Angle2 < UpperArc_Angle1) {
//            UpperArc_Angle2 += CH_C_2PI;
//        }
//
//        double LowerArc_Angle1 = std::atan2(ToothTipWidthLowerPnt.y() - ToothLowerArcCtrPnt.y(), ToothTipWidthLowerPnt.x() - ToothUpperArcCtrPnt.x());
//        double LowerArc_Angle2 = std::atan2(ToothBaseWidthLowerPnt.y() - ToothLowerArcCtrPnt.y(), ToothBaseWidthLowerPnt.x() - ToothUpperArcCtrPnt.x());
//
//        // Ensure that Angle1 is positive
//        LowerArc_Angle1 = LowerArc_Angle1 < 0 ? LowerArc_Angle1 + CH_C_2PI : LowerArc_Angle1;
//        // Ensure that Angle2 is greater than Angle1
//        while (LowerArc_Angle2 < LowerArc_Angle1) {
//            LowerArc_Angle2 += CH_C_2PI;
//        }
//
//        // Calculate the starting and ending arc angles for profile along the sprocket's outer radius
//        double OuterArc_Angle1 = CtrAng - HalfBaseWidthCordAng;
//        double OuterArc_Angle2 = OuterArc_Angle1 - OuterRadArcAng;
//
//
//        // Start the profile geometry with the upper concave tooth arc
//        geometry::ChLineArc arc1(ChCoordsys<>(ToothUpperArcCtrPnt), ArcRad, UpperArc_Angle2, UpperArc_Angle1);
//        // Next is the straight segment for the tooth tip
//        geometry::ChLineSegment seg2(ToothTipWidthUpperPnt, ToothTipWidthLowerPnt);
//        // Next is the lower concave tooth arc
//        geometry::ChLineArc arc3(ChCoordsys<>(ToothLowerArcCtrPnt), ArcRad, LowerArc_Angle2, LowerArc_Angle1);
//        // Finally is the arc segment that runs along the sprocket's outer radius to the start of the next tooth profile
//        geometry::ChLineArc arc4(ChCoordsys<>(SprocketCtr), OutRad, OuterArc_Angle1, OuterArc_Angle2);
//
//        // Add this tooth segments profile to the sprocket profile
//        profile->AddSubLine(arc1);
//        profile->AddSubLine(seg2);
//        profile->AddSubLine(arc3);
//        profile->AddSubLine(arc4);
//    }
//
//    return profile;
//}
//
//}  // end namespace vehicle
//}  // end namespace chrono


#include <cmath>

#include "chrono_vehicle/tracked_vehicle/sprocket/ChSprocketCB.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeRigidCB.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
    namespace vehicle {

        // -----------------------------------------------------------------------------
        // -----------------------------------------------------------------------------
        class SprocketCBContactCB : public ChSystem::CustomCollisionCallback {
        public:
            SprocketCBContactCB(ChTrackAssembly* track,  ///< containing track assembly
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

            virtual void OnCustomCollision(ChSystem* system) override;

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
                const ChVector<>& p4L,
                const ChVector<>& p1R,
                const ChVector<>& p2R,
                const ChVector<>& p3R,
                const ChVector<>& p4R);

            void CheckArcArc(std::shared_ptr<ChBody> connector,  // connector body
                const ChVector<>& bc,               // belt arc center
                double br,                          // belt arc radius
                const ChVector<>& bp1,              // belt arc end point 1
                const ChVector<>& bp2,              // belt arc end point 2
                const ChVector<> sc,                // sprocket arc center
                double sr,                          // sprockt arc radius
                const ChVector<>& sp1,              // sprocket arc end point 1
                const ChVector<>& sp2               // sprocket arc end point 2
                );

            void CheckCircleArc(std::shared_ptr<ChBody> connector,  // connector body
                const ChVector<>& cc,               // circle center
                double cr,                          // circle radius
                const ChVector<> ac,                // arc center
                double ar,                          // arc radius
                const ChVector<>& p1,               // arc end point 1
                const ChVector<>& p2                // arc end point 2
                );

            void CheckCircleSegment(std::shared_ptr<ChBody> connector,  // connector body
                const ChVector<>& cc,               // circle center
                double cr,                          // circle radius
                const ChVector<>& p1,               // segment end point 1
                const ChVector<>& p2                // segment end point 2
                );

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
        void SprocketCBContactCB::OnCustomCollision(ChSystem* system) {
            // Return now if collision disabled on sprocket.
            if (!m_sprocket->GetGearBody()->GetCollide())
                return;

            // Sprocket gear center location (expressed in global frame)
            ChVector<> locS_abs = m_sprocket->GetGearBody()->GetPos();

            // Loop over all track shoes in the associated track
            for (size_t is = 0; is < m_track->GetNumTrackShoes(); ++is) {
                auto shoe = std::static_pointer_cast<ChTrackShoeRigidCB>(m_track->GetTrackShoe(is));

                // Perform collision test for the "left" connector body
                CheckConnectorSprocket(shoe->m_connector_L, locS_abs);

                // Perform collision test for the "right" connector body
                CheckConnectorSprocket(shoe->m_connector_R, locS_abs);
            }
        }

        // Perform collision test between the specified connector body and the associated sprocket.
        void SprocketCBContactCB::CheckConnectorSprocket(std::shared_ptr<ChBody> connector, const ChVector<>& locS_abs) {
            // (1) Express the center of the connector body in the sprocket frame
            ChVector<> loc = m_sprocket->GetGearBody()->TransformPointParentToLocal(connector->GetPos());

            // (2) Broadphase collision test: no contact if the connector's center is too far from the
            // center of the gear (test performed in the sprocket's x-z plane).
            if (loc.x() * loc.x() + loc.z() * loc.z() > m_R_sum * m_R_sum)
                return;

            // (3) Working in the frame of the sprocket, find the candidate tooth space.
            // This is the closest tooth space to the connector center point.

            // Angle formed by 'locC' and the line z>0
            double angle = std::atan2(loc.x(), loc.z());
            // Find angle of closest tooth space
            double alpha = m_beta * std::round(angle / m_beta);
            // Convert to degrees
            double alpha_deg = alpha * 180 / CH_C_PI;

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

            // (5) Express the centers of the two connector end-caps in the sprocket frame.
            ChVector<> P1_abs = connector->TransformPointLocalToParent(ChVector<>(m_shoe_len / 2, 0, 0));
            ChVector<> P2_abs = connector->TransformPointLocalToParent(ChVector<>(-m_shoe_len / 2, 0, 0));
            ChVector<> P1 = m_sprocket->GetGearBody()->TransformPointParentToLocal(P1_abs);
            ChVector<> P2 = m_sprocket->GetGearBody()->TransformPointParentToLocal(P2_abs);

            // (6) Perform collision test between the front end of the connector and the gear profile.
            CheckCircleProfile(connector, P1, p1L, p2L, p3L, p4L, p1R, p2R, p3R, p4R);

            // (7) Perform collision test between the rear end of the connector and the gear profile.
            CheckCircleProfile(connector, P2, p1L, p2L, p3L, p4L, p1R, p2R, p3R, p4R);
        }

        // Working in the (x-z) plane of the gear, perform a 2D collision test between a circle
        // of radius m_shoe_R centered at the specified location and the gear profile.
        void SprocketCBContactCB::CheckCircleProfile(std::shared_ptr<ChBody> connector,
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
            CheckCircleArc(connector, loc, m_shoe_R, p3L, m_gear_R, p2L, p4L);

            // Check circle against arc centered at p3R.
            CheckCircleArc(connector, loc, m_shoe_R, p3R, m_gear_R, p3R, p4R);

            // Check circle against segment p1L - p2L.
            CheckCircleSegment(connector, loc, m_shoe_R, p1L, p2L);

            // Check circle against segment p1R - p2R.
            CheckCircleSegment(connector, loc, m_shoe_R, p1R, p2R);

            // Check circle against segment p4L - p4R.
            CheckCircleSegment(connector, loc, m_shoe_R, p4L, p4R);
        }


        // Working in the (x-z) plane, perform a 2D collision test between a circle of radius 'cr'
        // centered at 'cc' (on the connector body) and an arc on the circle of radius 'ar' centered
        // at 'ac', with the arc endpoints 'p1' and 'p2' (on the gear body).
        void SprocketCBContactCB::CheckArcArc(std::shared_ptr<ChBody> connector,  // connector body
            const ChVector<>& bc,               // belt arc center
            double br,                          // belt arc radius
            const ChVector<>& bp1,              // belt arc end point 1
            const ChVector<>& bp2,              // belt arc end point 2
            const ChVector<> sc,                // sprocket arc center
            double sr,                          // sprockt arc radius
            const ChVector<>& sp1,              // sprocket arc end point 1
            const ChVector<>& sp2               // sprocket arc end point 2
            ) {
            // Find distance between centers
            ChVector<> delta = bc - sc;
            double dist2 = delta.Length2();

            // If the two centers (circle and arc) are separated by less than the difference
            // of their adjusted radii, there is no contact.
            double Rdiff = sr - br;
            if (dist2 <= Rdiff * Rdiff)
                return;

            // If the contact point is outside the sprocket arc (sp1-sp2), there is no contact.
            ChVector<> a = sp1 - sc;
            ChVector<> b = sp2 - sc;
            ChVector<> axb = Vcross(a, b);
            ChVector<> axd = Vcross(a, delta);
            ChVector<> dxb = Vcross(delta, b);
            if (Vdot(axb, axd) <= 0 || Vdot(axb, dxb) <= 0)
                return;

            // If the contact point is outside the belt arc (bp1-bp2), there is no contact.
            a = bp1 - bc;
            b = bp2 - bc;
            axb = Vcross(a, b);
            axd = Vcross(a, delta);
            dxb = Vcross(delta, b);
            if (Vdot(axb, axd) <= 0 || Vdot(axb, dxb) <= 0)
                return;

            // Generate contact information (still in the sprocket frame)
            double dist = std::sqrt(dist2);
            ChVector<> normal = -delta / dist;
            ChVector<> pt_gear = sc - m_gear_R * normal;
            ChVector<> pt_shoe = bc - m_shoe_R * normal;

            // Fill in contact information and add the contact to the system.
            // Express all vectors in the global frame
            collision::ChCollisionInfo contact;
            contact.modelA = m_sprocket->GetGearBody()->GetCollisionModel().get();
            contact.modelB = connector->GetCollisionModel().get();
            contact.vN = m_sprocket->GetGearBody()->TransformDirectionLocalToParent(normal);
            contact.vpA = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_gear);
            contact.vpB = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_shoe);
            contact.distance = Rdiff - dist;

            m_sprocket->GetGearBody()->GetSystem()->GetContactContainer()->AddContact(contact);
        }


        // Working in the (x-z) plane, perform a 2D collision test between a circle of radius 'cr'
        // centered at 'cc' (on the connector body) and an arc on the circle of radius 'ar' centered
        // at 'ac', with the arc endpoints 'p1' and 'p2' (on the gear body).
        void SprocketCBContactCB::CheckCircleArc(std::shared_ptr<ChBody> connector,  // connector body
            const ChVector<>& cc,               // circle center
            double cr,                          // circle radius
            const ChVector<> ac,                // arc center
            double ar,                          // arc radius
            const ChVector<>& p1,               // arc end point 1
            const ChVector<>& p2                // arc end point 2
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
            contact.vN = m_sprocket->GetGearBody()->TransformDirectionLocalToParent(normal);
            contact.vpA = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_gear);
            contact.vpB = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_shoe);
            contact.distance = Rdiff - dist;

            m_sprocket->GetGearBody()->GetSystem()->GetContactContainer()->AddContact(contact);
        }

        // Working in the (x-z) plane, perform a 2D collision test between the circle of radius 'cr'
        // centered at 'cc' (on the connector body) and the line segment with endpoints 'p1' and 'p2'
        // (on the gear body).
        void SprocketCBContactCB::CheckCircleSegment(std::shared_ptr<ChBody> connector,  // connector body
            const ChVector<>& cc,               // circle center
            double cr,                          // circle radius
            const ChVector<>& p1,               // segment end point 1
            const ChVector<>& p2                // segment end point 2
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
            contact.vN = m_sprocket->GetGearBody()->TransformDirectionLocalToParent(normal);
            contact.vpA = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_gear);
            contact.vpB = m_sprocket->GetGearBody()->TransformPointLocalToParent(pt_shoe);
            contact.distance = dist - cr;

            m_sprocket->GetGearBody()->GetSystem()->GetContactContainer()->AddContact(contact);
        }

        // -----------------------------------------------------------------------------
        // -----------------------------------------------------------------------------
        ChSprocketCB::ChSprocketCB(const std::string& name) : ChSprocket(name) {}

        // -----------------------------------------------------------------------------
        // -----------------------------------------------------------------------------
        ChSystem::CustomCollisionCallback* ChSprocketCB::GetCollisionCallback(ChTrackAssembly* track) {
            // Check compatibility between this type of sprocket and the track shoes.
            // We expect track shoes of type ChSinglePinShoe.
            auto shoe = std::dynamic_pointer_cast<ChTrackShoeRigidCB>(track->GetTrackShoe(0));
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
            return new SprocketCBContactCB(track, 0.005, gear_nteeth, gear_RT, gear_R, gear_C, gear_W, GetSeparation(),
                shoe_len, shoe_R);
        }

        // -----------------------------------------------------------------------------
        // Create and return the sprocket gear profile.
        // -----------------------------------------------------------------------------
        std::shared_ptr<geometry::ChLinePath> ChSprocketCB::GetProfile() {
            auto profile = std::make_shared<geometry::ChLinePath>();
        
            //int num_teeth = GetNumTeeth();
            //double OutRad = GetOuterRadius();
            //double BaseLen = GetBaseWidth();
            //double TipLen = GetTipWidth();
            //double Depth = GetToothDepth();
            //double ArcRad = GetArcRadius();
        
            int num_teeth = 17;
            double OutRad = 0.2307;
            double BaseLen = 0.0530;
            double TipLen = 0.0128;
            double Depth = 0.0387;
            double ArcRad = 0.0542;

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
                double CtrAng = -i*EntireToothArcAng;
        
                // Vectors defining the current tooth's radial and perpendicular vectors
                ChVector<> vec_Radial(std::cos(CtrAng), std::sin(CtrAng), 0);
                ChVector<> vec_Perp(-vec_Radial.y(), vec_Radial.x(), 0);
        
                // Points defining the sprocket tooth's base width
                ChVector<> ToothBaseWidthUpperPnt(OutRad*(std::cos(CtrAng + HalfBaseWidthCordAng)), 
                    OutRad*(std::sin(CtrAng + HalfBaseWidthCordAng)), 
                    0);
                ChVector<> ToothBaseWidthLowerPnt(OutRad*(std::cos(CtrAng - HalfBaseWidthCordAng)), 
                    OutRad*(std::sin(CtrAng - HalfBaseWidthCordAng)), 
                    0);
                ChVector<> ToothBaseWidthCtrPnt = 0.5*(ToothBaseWidthUpperPnt + ToothBaseWidthLowerPnt);
        
                // Points defining the sprocket tooth's tip width
                ChVector<> ToothTipWidthCtrPnt = ToothBaseWidthCtrPnt - Depth*vec_Radial;
                ChVector<> ToothTipWidthUpperPnt = ToothTipWidthCtrPnt + 0.5*TipLen*vec_Perp;
                ChVector<> ToothTipWidthLowerPnt = ToothTipWidthCtrPnt - 0.5*TipLen*vec_Perp;
        
                // Points defining the tip to tooth base chord midpoint for the concave tooth profile
                double ToothArcChordLen = std::sqrt(Depth*Depth + std::pow((BaseLen - TipLen) / 2, 2));
                ChVector<> ToothUpperChordMidPnt = 0.5*(ToothBaseWidthUpperPnt + ToothTipWidthUpperPnt);
                ChVector<> ToothLowerChordMidPnt = 0.5*(ToothBaseWidthLowerPnt + ToothTipWidthLowerPnt);
        
                // Define the distance to project perpendicular to the tooth arc chord  at the chord's center
                // to reach the center of the tooth arc circles
                double ToothArcChordRadiusSegmentLen = sqrt(ArcRad*ArcRad - std::pow(ToothArcChordLen / 2, 2));
        
                //Define the arc centers for the tooth concave profiles
                ChVector<> vec_Chord1 = ToothBaseWidthUpperPnt - ToothTipWidthUpperPnt;
                vec_Chord1.Normalize();
                ChVector<> vec_Chord1Perp(-vec_Chord1.y(), vec_Chord1.x(), 0);
        
                ChVector<> vec_Chord2 = ToothBaseWidthLowerPnt - ToothTipWidthLowerPnt;
                vec_Chord2.Normalize();
                ChVector<> vec_Chord2Perp(-vec_Chord2.y(), vec_Chord2.x(), 0);
        
                ChVector<> ToothUpperArcCtrPnt = ToothUpperChordMidPnt - vec_Chord1Perp*ToothArcChordRadiusSegmentLen;
                ChVector<> ToothLowerArcCtrPnt = ToothLowerChordMidPnt + vec_Chord2Perp*ToothArcChordRadiusSegmentLen;
        
                // Calculate the starting and ending arc angles for each concave tooth profile
                double UpperArc_Angle1 = std::atan2(ToothBaseWidthUpperPnt.y() - ToothUpperArcCtrPnt.y(), ToothBaseWidthUpperPnt.x() - ToothUpperArcCtrPnt.x());
                double UpperArc_Angle2 = std::atan2(ToothTipWidthUpperPnt.y() - ToothUpperArcCtrPnt.y(), ToothTipWidthUpperPnt.x() - ToothUpperArcCtrPnt.x());
        
                // Ensure that Angle1 is positive
                UpperArc_Angle1 = UpperArc_Angle1 < 0 ? UpperArc_Angle1 + CH_C_2PI : UpperArc_Angle1; 
                // Ensure that Angle2 is greater than Angle1
                while (UpperArc_Angle2 < UpperArc_Angle1) {
                    UpperArc_Angle2 += CH_C_2PI;
                }
        
                double LowerArc_Angle1 = std::atan2(ToothTipWidthLowerPnt.y() - ToothLowerArcCtrPnt.y(), ToothTipWidthLowerPnt.x() - ToothLowerArcCtrPnt.x());
                double LowerArc_Angle2 = std::atan2(ToothBaseWidthLowerPnt.y() - ToothLowerArcCtrPnt.y(), ToothBaseWidthLowerPnt.x() - ToothLowerArcCtrPnt.x());
        
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



