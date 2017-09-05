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
                double separation       ///< separation between sprocket gears
                )
                : m_track(track),
                m_envelope(envelope),
                m_sprocket(m_track->GetSprocket()),
                m_gear_nteeth(gear_nteeth),
                m_separation(separation){
            }

            virtual void OnCustomCollision(ChSystem* system) override;

        private:
            ChTrackAssembly* m_track;                // pointer to containing track assembly
            std::shared_ptr<ChSprocket> m_sprocket;  // handle to the sprocket

            double m_envelope;  // collision detection envelope

            int m_gear_nteeth;    // sprocket gear, number of teeth
            double m_separation;  // separation distance between sprocket gears
        };

        // Add contacts between the sprocket and track shoes.
        void SprocketCBContactCB::OnCustomCollision(ChSystem* system) {
            // Return now if collision disabled on sproket.
            if (!m_sprocket->GetGearBody()->GetCollide())
                return;

            // Sprocket gear center location (expressed in global frame)
            ChVector<> locS_abs = m_sprocket->GetGearBody()->GetPos();

            // Loop over all track shoes in the associated track
            for (size_t is = 0; is < m_track->GetNumTrackShoes(); ++is) {
                auto shoe = std::static_pointer_cast<ChTrackShoeRigidCB>(m_track->GetTrackShoe(is));

                //// TODO
            }
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

            // Extract parameterization of sprocket profile
            int sprocket_nteeth = GetNumTeeth();

            // Create and return the callback object. Note: this pointer will be freed by the base class.
            return new SprocketCBContactCB(track, 0.005, GetNumTeeth(), GetSeparation());
        }

        // -----------------------------------------------------------------------------
        // Create and return the sprocket gear profile.
        // -----------------------------------------------------------------------------
        std::shared_ptr<geometry::ChLinePath> ChSprocketCB::GetProfile() {
            auto profile = std::make_shared<geometry::ChLinePath>();
        
            int num_teeth = GetNumTeeth();
            double OutRad = GetOuterRadius();
            double BaseLen = GetBaseWidth();
            double TipLen = GetTipWidth();
            double Depth = GetToothDepth();
            double ArcRad = GetArcRadius();
 
            //int num_teeth = 17;
            //double OutRad = 0.2307;
            //double BaseLen = 0.0530;
            //double TipLen = 0.0128;
            //double Depth = 0.0387;
            //double ArcRad = 0.0542;

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



