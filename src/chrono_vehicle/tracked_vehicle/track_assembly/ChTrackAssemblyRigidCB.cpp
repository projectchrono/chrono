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
// Base class for a continuous band track assembly using rigid-link track shoes
// (template definition).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

//#include <cmath>
//
//#include "chrono/core/ChLog.h"
//
//#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyRigidCB.h"
//
//namespace chrono {
//namespace vehicle {
//
//// -----------------------------------------------------------------------------
//// Assemble track shoes over wheels.
////
//// Returns true if the track shoes were initialized in a counter clockwise
//// direction and false otherwise.
////
//// This procedure is performed in the chassis reference frame, taking into
//// account the convention that the chassis reference frame has the x-axis
//// pointing to the front of the vehicle and the z-axis pointing up.
//// It is also assumed that the sprocket, idler, and road wheels lie in the
//// same vertical plane (in the chassis reference frame). The assembly is done
//// in the (x-z) plane.
////
//// TODO: NEEDS fixes for clock-wise wrapping (idler in front of sprocket)
////
//// -----------------------------------------------------------------------------
//bool ChTrackAssemblyRigidCB::Assemble(std::shared_ptr<ChBodyAuxRef> chassis) {
//    m_chassis = chassis;
//
//    // Number of track shoes and road wheels.
//    size_t num_shoes = m_shoes.size();
//    size_t num_wheels = m_suspensions.size();
//    size_t index = 0;
//
//    // Positions of sprocket, idler, and (front and rear) wheels (in chassis reference frame).
//    ChVector<> sprocket_pos_3d = chassis->TransformPointParentToLocal(m_sprocket->GetGearBody()->GetPos());
//    ChVector<> idler_pos_3d = chassis->TransformPointParentToLocal(m_idler->GetWheelBody()->GetPos());
//    ChVector<> front_wheel_pos_3d = chassis->TransformPointParentToLocal(m_suspensions[0]->GetWheelBody()->GetPos());
//    ChVector<> rear_wheel_pos_3d = front_wheel_pos_3d;
//    for (size_t i = 1; i < num_wheels; i++) {
//        ChVector<> wheel_pos = chassis->TransformPointParentToLocal(m_suspensions[i]->GetWheelBody()->GetPos());
//        if (wheel_pos.x() > front_wheel_pos_3d.x())
//            front_wheel_pos_3d = wheel_pos;
//        if (wheel_pos.x() < rear_wheel_pos_3d.x())
//            rear_wheel_pos_3d = wheel_pos;
//    }
//
//    // Decide whether we wrap counter-clockwise (sprocket in front of idler) or
//    // clockwise (sprocket behind idler).
//    // Set the positions of the road wheel closest to the sprocket and of the one
//    // closest to the idler.
//    bool ccw = sprocket_pos_3d.x() > idler_pos_3d.x();
//    double sign = ccw ? -1 : +1;
//    const ChVector<>& wheel_sprocket_pos_3d = ccw ? front_wheel_pos_3d : rear_wheel_pos_3d;
//    const ChVector<>& wheel_idler_pos_3d = ccw ? rear_wheel_pos_3d : front_wheel_pos_3d;
//
//    // Restrict to (x-z) plane.
//    ChVector2<> sprocket_pos(sprocket_pos_3d.x(), sprocket_pos_3d.z());
//    ChVector2<> idler_pos(idler_pos_3d.x(), idler_pos_3d.z());
//    ChVector2<> wheel_sprocket_pos(wheel_sprocket_pos_3d.x(), wheel_sprocket_pos_3d.z());
//    ChVector2<> wheel_idler_pos(wheel_idler_pos_3d.x(), wheel_idler_pos_3d.z());
//
//    //// TODO
//
//    return ccw;
//}
//
//}  // end namespace vehicle
//}  // end namespace chrono

#include <cmath>
#include <vector>
#include <algorithm>

#include "chrono/core/ChLog.h"

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyRigidCB.h"

namespace chrono {
    namespace vehicle {

        // -----------------------------------------------------------------------------
        // Assemble track shoes over wheels.
        //
        // Returns true if the track shoes were initialized in a counter clockwise
        // direction and false otherwise.
        //
        // This procedure is performed in the chassis reference frame, taking into
        // account the convention that the chassis reference frame has the x-axis
        // pointing to the front of the vehicle and the z-axis pointing up.
        // It is also assumed that the sprocket, idler, and road wheels lie in the
        // same vertical plane (in the chassis reference frame). The assembly is done
        // in the (x-z) plane.
        //
        // TODO: NEEDS fixes for clock-wise wrapping (idler in front of sprocket)
        //
        // -----------------------------------------------------------------------------
        bool ChTrackAssemblyRigidCB::Assemble(std::shared_ptr<ChBodyAuxRef> chassis) {


            // Number of track shoes and road wheels.
            size_t num_shoes = m_shoes.size();
            size_t num_wheels = m_suspensions.size();
            size_t index = 0;

            // Positions of sprocket, idler, and (front and rear) wheels (in chassis reference frame).
            ChVector<> sprocket_pos_3d = chassis->TransformPointParentToLocal(m_sprocket->GetGearBody()->GetPos());
            ChVector<> idler_pos_3d = chassis->TransformPointParentToLocal(m_idler->GetWheelBody()->GetPos());
            ChVector<> front_wheel_pos_3d = chassis->TransformPointParentToLocal(m_suspensions[0]->GetWheelBody()->GetPos());
            ChVector<> rear_wheel_pos_3d = front_wheel_pos_3d;
            for (size_t i = 1; i < num_wheels; i++) {
                ChVector<> wheel_pos = chassis->TransformPointParentToLocal(m_suspensions[i]->GetWheelBody()->GetPos());
                if (wheel_pos.x() > front_wheel_pos_3d.x())
                    front_wheel_pos_3d = wheel_pos;
                if (wheel_pos.x() < rear_wheel_pos_3d.x())
                    rear_wheel_pos_3d = wheel_pos;
            }

            // Decide whether we wrap counter-clockwise (sprocket in front of idler) or
            // clockwise (sprocket behind idler).
            // Set the positions of the road wheel closest to the sprocket and of the one
            // closest to the idler.
            bool ccw = sprocket_pos_3d.x() > idler_pos_3d.x();
            double sign = ccw ? -1 : +1;
            const ChVector<>& wheel_sprocket_pos_3d = ccw ? front_wheel_pos_3d : rear_wheel_pos_3d;
            const ChVector<>& wheel_idler_pos_3d = ccw ? rear_wheel_pos_3d : front_wheel_pos_3d;

            // Restrict to (x-z) plane.
            ChVector2<> sprocket_pos(sprocket_pos_3d.x(), sprocket_pos_3d.z());
            ChVector2<> idler_pos(idler_pos_3d.x(), idler_pos_3d.z());
            ChVector2<> wheel_sprocket_pos(wheel_sprocket_pos_3d.x(), wheel_sprocket_pos_3d.z());
            ChVector2<> wheel_idler_pos(wheel_idler_pos_3d.x(), wheel_idler_pos_3d.z());

            // Subsystem parameters.
            // Note that the idler and wheel radii are inflated by a fraction of the shoe height.
            double shoe_length = m_shoes[0]->GetShoeLength();
            double connector_length = m_shoes[0]->GetConnectorLength();
            double shoe_pitch = m_shoes[0]->GetPitch();
            double shoe_height = m_shoes[0]->GetHeight();
            double sprocket_radius = m_sprocket->GetAssemblyRadius();
            double idler_radius = m_idler->GetWheelRadius() + 0.9 * shoe_height;
            double wheel_radius = m_suspensions[0]->GetWheelRadius() + 1.1 * shoe_height;

            m_chassis = chassis;
            m_sprocket_offset = sprocket_pos_3d.y();
            m_connector_offset = m_shoes[0]->GetShoeWidth() / 2;

            std::cout << "sprocket_pos: " << sprocket_pos.x() << " " << sprocket_pos.y() << std::endl;
            std::cout << "idler_pos: " << idler_pos.x() << " " << idler_pos.y() << std::endl;
            std::cout << "wheel_sprocket_pos: " << wheel_sprocket_pos.x() << " " << wheel_sprocket_pos.y() << std::endl;
            std::cout << "wheel_idler_pos: " << wheel_idler_pos.x() << " " << wheel_idler_pos.y() << std::endl;
            std::cout << "sprocket_radius: " << sprocket_radius << std::endl;
            std::cout << "idler_radius: " << idler_radius << std::endl;
            std::cout << "wheel_radius: " << wheel_radius << std::endl;
            std::cout << "shoe_length: " << shoe_length << std::endl;
            std::cout << "connector_length: " << connector_length << std::endl;




            NewAssembleTest(chassis);
            return true;




            // Set target points around the track.
            ChVector2<> sprocket_bottom = sprocket_pos - ChVector2<>(0, sprocket_radius);
            ChVector2<> idler_top = idler_pos + ChVector2<>(0, idler_radius);
            ChVector2<> idler_bottom = idler_pos - ChVector2<>(0, idler_radius);
            ChVector2<> wheel_idler_bottom = wheel_idler_pos - ChVector2<>(0, wheel_radius);
            ChVector2<> wheel_sprocket_bottom = wheel_sprocket_pos - ChVector2<>(0, wheel_radius);

            // Keep track of the (x,z) locations of shoe body and connector bodies, as
            // well as of the angles (from the x axis) of the shoe and connector bodies.
            ChVector2<> ps;  // location of shoe center
            ChVector2<> pc;  // location of connector center
            ChVector2<> p1;  // location of rear pin (connection to previous)
            ChVector2<> p2;  // location of front pin (connection to next)
            double as;
            double ac;

            // 1. Create shoes around the sprocket, starting under the sprocket and
            //    moving away from the idler. Stop before creating a horizontal track
            //    shoe above the sprocket.

            // Create first track shoe, such that its connector body is horizontal and positioned
            // under the sprocket. Tilt the shoe body towards the roadwheel closest to the sprocket.
            pc = sprocket_bottom;
            p2 = pc + ChVector2<>(connector_length / 2, 0);
            ac = 0;
            ChVector2<> A = pc - ChVector2<>(connector_length / 2, 0);
            as = std::atan2(A.y() - wheel_sprocket_bottom.y(), A.x() - wheel_sprocket_bottom.x());
            ps = A - Vrot(ChVector2<>(shoe_length / 2, 0), as);
            p1 = ps - Vrot(ChVector2<>(shoe_length / 2, 0), as);
            CreateTrackShoe(index, ps, pc, as, ac);
            index++;

            // Cache location of rear pin (needed to close the track)
            ChVector2<> p0 = p1;

            // Wrap around sprocket.
            // ATTENTION:  USING SOME MAGIC NUMBERS HERE (angle adjustments)!!!!
            double delta = CH_C_2PI / m_sprocket->GetNumTeeth();
            for (int is = 1; is <= m_sprocket->GetNumTeeth() / 2; is++) {
                ChVector2<> A = sprocket_pos + sprocket_radius * ChVector2<>(std::sin(is*delta), -std::cos(is*delta));
                double angle = std::atan2(A.y() - p2.y(), A.x() - p2.x());
                as = angle - 0.07;
                ac = angle + 0.2;
                ps = p2 + Vrot(ChVector2<>(shoe_length / 2, 0), as);
                pc = ps + Vrot(ChVector2<>(shoe_length / 2, 0), as) + Vrot(ChVector2<>(connector_length / 2, 0), ac);
                CreateTrackShoe(index, ps, pc, as, ac);
                p2 = pc + Vrot(ChVector2<>(connector_length / 2, 0), ac);
                index++;
            }

            p1 = p2;

            // 2. Create shoes between sprocket and idler. These shoes are parallel to a
            //    line connecting the top points of the sprocket gear and idler wheel.
            //    We target a point that lies above the idler by slightly more than the
            //    track shoe's height and stop when we reach the idler location.

            // Calculate the constant pitch angle.
            double dy = (sprocket_pos.y() + sprocket_radius) - (idler_pos.y() + idler_radius);
            double dx = sprocket_pos.x() - idler_pos.x();
            double angle = ccw ? -CH_C_PI - std::atan2(dy, dx) : CH_C_PI + std::atan2(dy, -dx);

            // Create track shoes with constant orientation
            while (sign * (idler_pos.x() - p2.x() + shoe_pitch) > 0 && index < num_shoes) {
                p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
                CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
                p1 = p2;
                ++index;
            }

            // 3. Create shoes around the idler wheel. Stop when we wrap under the idler.

            // Calculate the incremental pitch angle around the idler.
            double tmp = shoe_pitch / (2 * idler_radius);
            double delta_angle = sign * std::asin(tmp);

            while (std::abs(angle) < CH_C_2PI && index < num_shoes) {
                p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
                CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
                p1 = p2;
                angle += 2 * delta_angle;
                ++index;
            }

            // 4. Create shoes between idler and closest road wheel. The shoes are parallel
            //    to a line connecting bottom points on idler and wheel. Stop when passing
            //    the wheel position.

            dy = (idler_pos.y() - idler_radius) - (wheel_idler_pos.y() - wheel_radius);
            dx = idler_pos.x() - wheel_idler_pos.x();
            angle = ccw ? -CH_C_2PI + std::atan2(dy, -dx) : -CH_C_PI - std::atan2(dy, dx);

            // Create track shoes with constant orientation
            while (sign * (p2.x() - wheel_idler_pos.x()) > 0 && index < num_shoes) {
                p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
                CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
                p1 = p2;
                ++index;
            }

            // 5. Create shoes below the road wheels. These shoes are horizontal. Stop when
            //    passing the position of the wheel closest to the sprocket.

            angle = ccw ? 0 : CH_C_2PI;

            while (sign * (p2.x() - wheel_sprocket_pos.x()) > 0 && index < num_shoes) {
                p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
                CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
                p1 = p2;
                ++index;
            }

            // 6. If we have an odd number of track shoes left, create one more shoe, tilted towards
            //    the first track shoe.

            size_t num_left = num_shoes - index;

            if (num_left % 2 == 1) {
                angle = -std::atan2(p0.y() - p1.y(), p0.x() - p1.x());
                p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
                CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
                p1 = p2;
                ++index;
                --num_left;
            }

            // 7. Check if the remaining shoes are enough to close the loop.

            double gap = (p0 - p1).Length();

            if (num_left * shoe_pitch < gap) {
                GetLog() << "\nInsufficient number of track shoes for this configuration.\n";
                GetLog() << "Missing distance: " << gap - num_left * shoe_pitch << "\n\n";
                for (size_t i = index; i < num_shoes; i++) {
                    p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
                    CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
                    p1 = p2;
                    ++index;
                }
                return ccw;
            }

            // 8. Complete the loop using the remaining shoes (always an even number)
            //    Form an isosceles triangle connecting the last initialized shoe with
            //    the very first one under the sprocket.

            double alpha = std::atan2(p0.y() - p2.y(), p0.x() - p2.x());
            double beta = std::acos(gap / (shoe_pitch * num_left));

            // Create half of the remaining shoes (with a pitch angle = alpha-beta).
            angle = sign * (alpha - beta);
            for (size_t i = 0; i < num_left / 2; i++) {
                p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
                CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
                p1 = p2;
                ++index;
            }

            // Create the second half of the remaining shoes (pitch angle = alpha+beta).
            angle = sign * (alpha + beta);
            for (size_t i = 0; i < num_left / 2; i++) {
                p2 = p1 + shoe_pitch * ChVector2<>(-sign * std::cos(angle), sign * std::sin(angle));
                CreateTrackShoe(index, 0.5 * (p1 + p2), -angle);
                p1 = p2;
                ++index;
            }

            NewAssembleTest(chassis);

            GetLog() << "Track assembly done.  Number of track shoes: " << index << "\n";
            return ccw;
        }

        void ChTrackAssemblyRigidCB::CreateTrackShoe(size_t index, ChVector2<> ps, ChVector2<> pc, double as, double ac) {
            // Set index within the track assembly
            m_shoes[index]->SetIndex(index);

            // Body locations (relative to chassis frame)
            ChVector<> loc_shoe(ps.x(), m_sprocket_offset, ps.y());
            ChVector<> loc_connector_L(pc.x(), m_sprocket_offset + m_connector_offset, pc.y());
            ChVector<> loc_connector_R(pc.x(), m_sprocket_offset - m_connector_offset, pc.y());

            // Body orientation (relative to chassis frame)
            // Note that the angle sign must be flipped (x->y positive in 2D, but x->z negative in 3D)
            ChQuaternion<> rot_shoe = Q_from_AngY(-as);
            ChQuaternion<> rot_connector = Q_from_AngY(-ac);

            // Initialize the track shoe system
            m_shoes[index]->Initialize(m_chassis, loc_shoe, rot_shoe, loc_connector_L, loc_connector_R, rot_connector);
        }

        void ChTrackAssemblyRigidCB::CreateTrackShoe(size_t index, ChVector2<> p, double angle) {
            // Set index within the track assembly
            m_shoes[index]->SetIndex(index);

            // Location of track shoe center (relative to chassis frame)
            ChVector<> loc(p.x(), m_sprocket_offset, p.y());

            m_shoes[index]->Initialize(m_chassis, loc, Q_from_AngY(-angle));
        }





















        void ChTrackAssemblyRigidCB::NewAssembleTest(std::shared_ptr<ChBodyAuxRef> chassis) {
            // ----------------------------------------------------------
            // Settings:
            // ----------------------------------------------------------
            double FirstShoeFractionalOffset = -0.5;  // Shift the start point by a fraction of a shoe length so that the assembly can start with the bekt tooth aligned with the sprocket grove
            double Tolerance = 1e-12;                 // Tolerance on how close the iterative algorithm will try to connect the beginning and end of the belt
            int Maxiter = 200;                        // Stop after this number of iterations to prevent an infinite loop in case something unexpected goes wrong

            // ----------------------------------------------------------
            // Read in the geometric parameters for the track assembly
            // ----------------------------------------------------------

            // Number of track shoes and road wheels.
            size_t num_shoes = m_shoes.size();
            size_t num_wheels = m_suspensions.size();
            size_t index = 0;

            ChVectorDynamic<> ShoeConnectionLengths(2);
            ShoeConnectionLengths(0) = m_shoes[0]->GetShoeLength();
            ShoeConnectionLengths(1) = m_shoes[0]->GetConnectorLength();

            ChMatrixDynamic<> CirclePosAll(2 + num_wheels, 2); //X & Y coordinates for the sprocket, idler, and all of the road wheels
            ChVectorDynamic<> CircleRadiusAll(2 + num_wheels); //radii for the sprocket, idler, and all of the road wheels

            ChVector<> sprocket_pos_3d = chassis->TransformPointParentToLocal(m_sprocket->GetGearBody()->GetPos());
            ChVector<> idler_pos_3d = chassis->TransformPointParentToLocal(m_idler->GetWheelBody()->GetPos());

            CirclePosAll(0, 0) = sprocket_pos_3d.x();
            CirclePosAll(0, 1) = sprocket_pos_3d.z();
            CircleRadiusAll(0) = m_sprocket->GetAssemblyRadius();

            CirclePosAll(1, 0) = idler_pos_3d.x();
            CirclePosAll(1, 1) = idler_pos_3d.z();
            CircleRadiusAll(1) = m_idler->GetWheelRadius();

            for (size_t i = 0; i < num_wheels; i++) {
                ChVector<> wheel_pos = chassis->TransformPointParentToLocal(m_suspensions[i]->GetWheelBody()->GetPos());
                CirclePosAll(i + 2, 0) = wheel_pos.x();
                CirclePosAll(i + 2, 1) = wheel_pos.z();
                CircleRadiusAll(i + 2) = m_suspensions[i]->GetWheelRadius();
            }


            // ----------------------------------------------------------
            // Find the path that encloses the sprocket, idler, and road wheels
            // Shrink wrap type algorithm
            // ----------------------------------------------------------
            // Step 1: Determine if the sprocket is ahead or behind of the other circles
            double AverageXPos = 0;
            for (size_t i = 0; i < CirclePosAll.GetRows(); i++) {
                AverageXPos += CirclePosAll(i, 0);
            }
            AverageXPos /= CirclePosAll.GetRows();

            bool SprocketInFront = (CirclePosAll(0, 0) >= AverageXPos) ? true : false;
            double StartingAngle = (CirclePosAll(0, 0) >= AverageXPos) ? 0 : CH_C_PI;


            // Start building the path around the sprocket, idler, and wheels

            size_t Current_Circle = 0;
            size_t NumCirclesOnPath = 0;
            ChMatrixDynamic<> TangentPoints(CirclePosAll.GetRows(), 4); //Will be resized later
            ChVectorDynamic<> CircleIndex(CirclePosAll.GetRows()); //Will be resized later

            // At most each circle is visted once.  Something went wrong if
            // the loop is not closed within this many passes through the algorithm
            size_t iter;
            for (iter = 0; iter < CirclePosAll.GetRows(); iter++) {
                // Step 2: Create an ordered list of the circles with repect to thier
                // distance from the starting circle;

                std::vector<double> Distances;
                std::vector<size_t> DistanceIndices;
                Distances.reserve(CirclePosAll.GetRows() - 1);
                DistanceIndices.reserve(CirclePosAll.GetRows() - 1);

                for (size_t c = 0; c < CirclePosAll.GetRows(); c++) {
                    if (c == Current_Circle){ //Don't count the distance between the circle and itself
                        continue;
                    }

                    ChVector2<> Dist(CirclePosAll(c, 0) - CirclePosAll(Current_Circle, 0), CirclePosAll(c, 1) - CirclePosAll(Current_Circle, 1));
                    Distances.push_back(Dist.Length2());
                    DistanceIndices.push_back(c);
                }

                std::vector<int> idx(Distances.size());
                std::size_t n(0);
                std::generate(std::begin(idx), std::end(idx), [&]{ return n++; });

                // Sort the DistanceIndices based on Distances
                // Based on https://stackoverflow.com/questions/25921706/creating-a-vector-of-indices-of-a-sorted-vector
                // similar to MATLAB functions [~,idx] = sort(Distances,'descend');
                // DistanceIndices = DistanceIndices(idx);
                std::sort(std::begin(idx), std::end(idx),
                    [&Distances](size_t idx1, size_t idx2) { return Distances[idx1] > Distances[idx2]; });


                // Step 3: Going down the ordered list of circles from farthest to closest,
                // find the tangent point that is the closest CCW angle to to the starting
                // angle and save those tangent points.  Attacking the problem in this order
                // skips over multiple circle tangents that are co-linear

                double minAngle = 0;
                size_t nextCircle = 0;

                ChVector2<> Tan1Pnt1;
                ChVector2<> Tan1Pnt2;
                ChVector2<> Tan2Pnt1;
                ChVector2<> Tan2Pnt2;

                for (size_t i = 0; i < DistanceIndices.size(); i++) {
                    //Find the tangent points between the current circle and the next circle to check the wrap angles on
                    ChVector2<> Circle1Pos(CirclePosAll(Current_Circle, 0), CirclePosAll(Current_Circle, 1));
                    double Circle1Rad = CircleRadiusAll(Current_Circle);
                    ChVector2<> Circle2Pos(CirclePosAll(DistanceIndices[idx[i]], 0), CirclePosAll(DistanceIndices[idx[i]], 1));
                    double Circle2Rad = CircleRadiusAll(DistanceIndices[idx[i]]);

                    FindCircleTangentPoints(Circle1Pos, Circle1Rad, Circle2Pos, Circle2Rad,
                        Tan1Pnt1, Tan1Pnt2, Tan2Pnt1, Tan2Pnt2);

                    //Calculate the wrap angles for each of the 2 external circle tangent lines
                    double Angle1 = std::atan2(Tan1Pnt1.y() - CirclePosAll(Current_Circle, 1), Tan1Pnt1.x() - CirclePosAll(Current_Circle, 0));
                    double Angle2 = std::atan2(Tan2Pnt1.y() - CirclePosAll(Current_Circle, 1), Tan2Pnt1.x() - CirclePosAll(Current_Circle, 0));

                    //Ensure that all of the angles are greater than the starting angle by at least a little bit to prevent numerical noise
                    while (Angle1 < StartingAngle + .00001){
                        Angle1 += CH_C_2PI;
                    }
                    while (Angle2 < StartingAngle + .00001){
                        Angle2 += CH_C_2PI;
                    }

                    //If this is the first point that is examined, then save that point as the inital comparison value.
                    if (i == 0) {
                        minAngle = Angle1;
                        TangentPoints(iter, 0) = Tan1Pnt1.x();
                        TangentPoints(iter, 1) = Tan1Pnt1.y();
                        TangentPoints(iter, 2) = Tan1Pnt2.x();
                        TangentPoints(iter, 3) = Tan1Pnt2.y();

                        nextCircle = DistanceIndices[idx[i]];
                    }

                    if (Angle1 < minAngle){
                        // Save Tangent
                        minAngle = Angle1;
                        TangentPoints(iter, 0) = Tan1Pnt1.x();
                        TangentPoints(iter, 1) = Tan1Pnt1.y();
                        TangentPoints(iter, 2) = Tan1Pnt2.x();
                        TangentPoints(iter, 3) = Tan1Pnt2.y();

                        nextCircle = DistanceIndices[idx[i]];
                    }

                    if (Angle2 < minAngle){
                        // Save Tangent
                        minAngle = Angle2;
                        TangentPoints(iter, 0) = Tan2Pnt1.x();
                        TangentPoints(iter, 1) = Tan2Pnt1.y();
                        TangentPoints(iter, 2) = Tan2Pnt2.x();
                        TangentPoints(iter, 3) = Tan2Pnt2.y();

                        nextCircle = DistanceIndices[idx[i]];
                    }

                }

                // Setup for the next loop through
                CircleIndex(iter) = nextCircle;
                Current_Circle = nextCircle;
                StartingAngle = std::atan2(TangentPoints(iter, 3) - CirclePosAll(Current_Circle, 1), TangentPoints(iter, 2) - CirclePosAll(Current_Circle, 0));
                if (StartingAngle < 0){
                    StartingAngle += CH_C_2PI;
                }

                //Check to see if the wrap has made it back onto the strating circle (sprocket)
                if (Current_Circle == 0){
                    break;
                }

            }


            //Save the reduced geometry corresponding to the tight wrapping of the belt path

            ChMatrixDynamic<> CirclePos(iter+1, 2); //X & Y coordinates for the sprocket and idler or any of the road wheels on the belt wrap loop
            ChVectorDynamic<> CircleRadius(iter + 1); //radii for the sprocket and idler or any of the road wheels on the belt wrap loop
            
            // Move the sprocket to the beginning position from the end
            CirclePos(0, 0) = CirclePosAll(CircleIndex(iter), 0);
            CirclePos(0, 1) = CirclePosAll(CircleIndex(iter), 1);
            CircleRadius(0) = CircleRadiusAll(CircleIndex(iter)); 
            for (size_t i = 0; i < (iter); i++) {
                CirclePos(i+1, 0) = CirclePosAll(CircleIndex(i), 0);
                CirclePos(i+1, 1) = CirclePosAll(CircleIndex(i), 1);
                CircleRadius(i+1) = CircleRadiusAll(CircleIndex(i));
            }



            //--------------------------------------------------------------------------
            // Fit the track around the outermost wrapping.
            //
            // TODO: Make sure that there is too
            // much, rather than too little track left over so that last two shoes can
            // be very slightly folded to exactly fit.
            //
            // Start the iterations with the scale such that the belt length equals the
            // outer wrapping circumferance(too small of a scale, but not by much).
            // After that, overshoot the extra length by a factor of 10 to make sure
            // that the scale is too large.
            // Then binary search tree on the scale to converge.
            //--------------------------------------------------------------------------

            double ScaleMin = 0;
            double ScaleMax = 0;
            double RemainingLenMinScale = 0;
            double RemainingLenMaxScale = 0;

            //Start by calculating the original tangent(constant) and arc lengths (variable)
            // to determine the inital scale for sprocket/idler/road wheel circles 

            double CombineShoeLength = 0;
            for (size_t i = 0; i < ShoeConnectionLengths.GetRows(); i++) {
                CombineShoeLength += ShoeConnectionLengths(i); //sum all of the seperate lengths per shoe
            }
            CombineShoeLength *= num_shoes;

            ChMatrixDynamic<> Arcs(CircleRadius.GetRows(), 3); //for each circle [Arc Length, Starting Angle of Arc, Ending Angle of Arc]

            StartingAngle = std::atan2(TangentPoints(CircleRadius.GetRows() - 1, 3) - CirclePos(0, 1), TangentPoints(CircleRadius.GetRows() - 1, 2) - CirclePos(0, 0));
            double EndingAngle = std::atan2(TangentPoints(0, 1) - CirclePos(0, 1), TangentPoints(0, 0) - CirclePos(0, 0));

            if (StartingAngle < 0){
                StartingAngle += CH_C_2PI;
            }
            while (EndingAngle < StartingAngle){
                EndingAngle += CH_C_2PI;
            }

            Arcs(0, 0) = EndingAngle - StartingAngle;
            Arcs(0, 1) = StartingAngle;
            Arcs(0, 2) = EndingAngle;

            for (size_t i = 1; i< CircleRadius.GetRows(); i++) {
                StartingAngle = std::atan2(TangentPoints(i-1, 3) - CirclePos(i, 1), TangentPoints(i-1, 2) - CirclePos(i, 0));
                EndingAngle = std::atan2(TangentPoints(i, 1) - CirclePos(i, 1), TangentPoints(i, 0) - CirclePos(i, 0));

                if (StartingAngle < 0){
                    StartingAngle += CH_C_2PI;
                }
                while (EndingAngle < StartingAngle){
                    EndingAngle += CH_C_2PI;
                }

                Arcs(i, 0) = EndingAngle - StartingAngle;
                Arcs(i, 1) = StartingAngle;
                Arcs(i, 2) = EndingAngle;
            }

            //Calculate the belt wrap length
            double LengthOfArcs = 0;
            double LengthOfTangents = 0;

            for (size_t i = 0; i < Arcs.GetRows(); i++) {
                LengthOfArcs += (Arcs(i, 0)*CircleRadius(i));

                LengthOfTangents += std::sqrt(std::pow(TangentPoints(i, 2) - TangentPoints(i, 0), 2) + std::pow(TangentPoints(i, 3) - TangentPoints(i, 1), 2));
            }

            // Calculate how the arcs need to be scaled to fit so that tangent length +
            // the arc lengths = the belt length.This should be a slight underestimate
            // of the needed scale.

            double DeltaRadius = (CombineShoeLength - (LengthOfArcs + LengthOfTangents)) / (CH_C_2PI); //Divide by 2*PI since the belt wrap forms a closed loop/circle
            ScaleMin = DeltaRadius;

            ChMatrixDynamic<> ShoePoints(1 + num_shoes*ShoeConnectionLengths.GetRows(), 2);


            for (iter = 0; iter < Maxiter; iter++){

                ChVectorDynamic<> ScaledCircleRadius = CircleRadius;
                for (size_t i = 0; i < ScaledCircleRadius.GetRows(); i++) {
                    ScaledCircleRadius(i) += DeltaRadius;
                }

                // Calculate the new tangents based off of the arc starting and ending points for each circle
                for (size_t i = 0; i < ScaledCircleRadius.GetRows(); i++) {
                    double StartingArcPoint_x = cos(Arcs(i, 1))*ScaledCircleRadius(i) + CirclePos(i, 0);
                    double StartingArcPoint_z = sin(Arcs(i, 1))*ScaledCircleRadius(i) + CirclePos(i, 1);

                    double EndingArcPoint_x = cos(Arcs(i, 2))*ScaledCircleRadius(i) + CirclePos(i, 0);
                    double EndingArcPoint_z = sin(Arcs(i, 2))*ScaledCircleRadius(i) + CirclePos(i, 1);

                    if (i == 0) { // Sprocket
                        TangentPoints(0, 0) = EndingArcPoint_x;
                        TangentPoints(0, 1) = EndingArcPoint_z;
                        TangentPoints(ScaledCircleRadius.GetRows() - 1, 2) = StartingArcPoint_x;
                        TangentPoints(ScaledCircleRadius.GetRows() - 1, 3) = StartingArcPoint_z;
                    }
                    else {
                        TangentPoints(i, 0) = EndingArcPoint_x;
                        TangentPoints(i, 1) = EndingArcPoint_z;
                        TangentPoints(i - 1, 2) = StartingArcPoint_x;
                        TangentPoints(i - 1, 3) = StartingArcPoint_z;
                    }
                }

                // Start wrapping the belt.Start with the first link all the way forward
                // (or all the way rearward) shifted by the desired offset which is used to
                // center the first track link to make orienting the sprocket profile easier

                // cordLength = 2 * r*sin(theta / 2)
                StartingAngle = 2 * std::asin(FirstShoeFractionalOffset * ShoeConnectionLengths(0) / (2 * ScaledCircleRadius(0)));

                if (!SprocketInFront) {
                    StartingAngle += CH_C_PI; //Start on the back side of the sprocket instead of the front
                }

                while (StartingAngle < Arcs(0, 1)) {
                    StartingAngle += CH_C_2PI;
                }

                ShoePoints.FillElem(0); // Zeros out the matrix since the Shoe Points are recalculated each iteration
                ShoePoints(0, 0) = std::cos(StartingAngle)*ScaledCircleRadius(0) + CirclePos(0, 0);
                ShoePoints(0, 1) = std::sin(StartingAngle)*ScaledCircleRadius(0) + CirclePos(0, 1);


                size_t shoelinktype = 0;
                size_t CurrentFeature = 0;

                // Build features array used for determining shoe points on the arcs and tangents
                ChMatrixDynamic<> Features(2 * ScaledCircleRadius.GetRows(), 6);

                for (size_t i = 0; i < ScaledCircleRadius.GetRows(); i++){
                    //Add Arc Wrap Feature (id = 0)
                    Features(2 * i, 0) = 0; // feature type id
                    Features(2 * i, 1) = CirclePos(i, 0);
                    Features(2 * i, 2) = CirclePos(i, 1);
                    Features(2 * i, 3) = Arcs(i, 1);
                    Features(2 * i, 4) = Arcs(i, 2);
                    Features(2 * i, 5) = ScaledCircleRadius(i);

                    //Add Tangent Wrap Feature (id = 1)
                    Features(1 + (2 * i), 0) = 1; //feature type id
                    Features(1 + (2 * i), 1) = TangentPoints(i, 0);
                    Features(1 + (2 * i), 2) = TangentPoints(i, 1);
                    Features(1 + (2 * i), 3) = TangentPoints(i, 2);
                    Features(1 + (2 * i), 4) = TangentPoints(i, 3);
                    Features(1 + (2 * i), 5) = 0; //Unused
                }



                // Calculate the remaining ShoePoints
                bool FoundPoint = false;
                bool InitalSprocketWrap = true;
                double ExtraLength = 0;
                double CurrentAngle = 0;
                size_t shoeidx;

                for (shoeidx = 1; shoeidx < ShoePoints.GetRows(); shoeidx++){
                    ChVector2<> Point;
                    ChVector2<> StartingPoint(ShoePoints(shoeidx - 1, 0), ShoePoints(shoeidx - 1, 1));

                    for (size_t c = 0; c < Features.GetRows(); c++) {// Make sure that all the features are only searched once to prevent and endless loop
                        if (Features(CurrentFeature, 0) == 0){
                            CheckCircleCircle(FoundPoint, Point, Features, CurrentFeature, StartingPoint, ShoeConnectionLengths(shoelinktype));
                        }
                        else{
                            CheckCircleLine(FoundPoint, Point, Features, CurrentFeature, StartingPoint, ShoeConnectionLengths(shoelinktype));
                        }

                        if (FoundPoint){
                            break; //Still on the current Feature (arc or tangent)
                        }

                        InitalSprocketWrap = false;
                        CurrentFeature += 1;
                        if (CurrentFeature >= Features.GetRows()) {
                            CurrentFeature = 0;
                        }
                    }

                    if (!FoundPoint){
                        std::cout << "Belt Assembly ERROR: Something went wrong" << std::endl;
                        return;
                    }

                    ShoePoints(shoeidx, 0) = Point.x();
                    ShoePoints(shoeidx, 1) = Point.y();

                    shoelinktype += 1;
                    if (shoelinktype >= ShoeConnectionLengths.GetRows()){
                        shoelinktype = 0;
                    }

                    ExtraLength = 0;
                    //Check to see if I have wrapped past the inital point
                    if ((!InitalSprocketWrap) && (CurrentFeature == 0)){
                        CurrentAngle = std::atan2(Point.y() - CirclePos(0, 1), Point.x() - CirclePos(0, 0));
                        while (CurrentAngle<Features(0, 3)){
                            CurrentAngle += CH_C_2PI;
                        }
                        if (CurrentAngle>StartingAngle){
                            ExtraLength = (CurrentAngle - StartingAngle)*Features(0, 5);
                            break;
                        }
                    }
                }


                for (int j = shoeidx+1; j < (ShoePoints.GetRows()); j++){
                    ExtraLength += ShoeConnectionLengths(shoelinktype);

                    shoelinktype += 1;
                    if (shoelinktype >= ShoeConnectionLengths.GetRows()){
                        shoelinktype = 0;
                    }
                }

                if (CurrentFeature > 0){
                    ExtraLength = -std::sqrt(std::pow(ShoePoints(ShoePoints.GetRows()-1, 0) - ShoePoints(0, 0), 2) + std::pow(ShoePoints(ShoePoints.GetRows()-1, 1) - ShoePoints(0, 1), 2));
                }
                else if ((ExtraLength == 0) && (CurrentAngle > StartingAngle)){
                    ExtraLength = std::sqrt(std::pow(ShoePoints(ShoePoints.GetRows()-1, 0) - ShoePoints(0, 0), 2) + std::pow(ShoePoints(ShoePoints.GetRows()-1, 1) - ShoePoints(0, 1), 2));
                }
                else if (ExtraLength == 0){
                    ExtraLength = -std::sqrt(std::pow(ShoePoints(ShoePoints.GetRows()-1, 0) - ShoePoints(0, 0), 2) + std::pow(ShoePoints(ShoePoints.GetRows()-1, 1) - ShoePoints(0, 1), 2));
                }

                if (abs(ExtraLength) < Tolerance){
                    std::cout << "Belt Wrap Algorithm Conveged after " << iter << " iterations - Extra Length: " << ExtraLength << " - Length Tolerance: " << Tolerance << std::endl;
                    break;
                }

                if (ExtraLength > 0){
                    if (DeltaRadius > ScaleMin) {
                        ScaleMin = DeltaRadius;
                        RemainingLenMinScale = ExtraLength;
                    }
                }
                else {
                    if ((DeltaRadius < ScaleMax) || (ScaleMax == 0)){
                        ScaleMax = DeltaRadius;
                        RemainingLenMaxScale = ExtraLength;
                    }
                }


                if (iter == 0) {
                    DeltaRadius = DeltaRadius + ExtraLength * 10;
                }
                else {
                    DeltaRadius = (ScaleMin + ScaleMax) / 2;
                }
            }




            //Now create all of the track shoes at the located points
            //for (size_t s = 0; s < ((ShoePoints.GetRows()/2)); s++){
            ////for (size_t s = 0; s < ((ShoePoints.GetRows() / 2)-1); s++){
            //    ChVector2<> ps((ShoePoints(s * 2, 0) + ShoePoints(1 + s * 2, 0)) / 2, (ShoePoints(s * 2, 1) + ShoePoints(1 + s * 2, 1)) / 2);
            //    ChVector2<> pc((ShoePoints(1 + s * 2, 0) + ShoePoints(2 + s * 2, 0)) / 2, (ShoePoints(1 + s * 2, 1) + ShoePoints(2 + s * 2, 1)) / 2);
            //    double as = std::atan2(ShoePoints(1 + s * 2, 1) - ShoePoints(s * 2, 1), ShoePoints(1 + s * 2, 0) - ShoePoints(s * 2, 0));
            //    double ac = std::atan2(ShoePoints(2 + s * 2, 1) - ShoePoints(1 + s * 2, 1), ShoePoints(2 + s * 2, 0) - ShoePoints(1 + s * 2, 0));

            //    CreateTrackShoe(s, ps, pc, as, ac);
            //}

            for (size_t s = 0; s < ((ShoePoints.GetRows() / 2)-1); s++){
                ChVector2<> ps((ShoePoints(s * 2, 0) + ShoePoints(1 + s * 2, 0)) / 2, (ShoePoints(s * 2, 1) + ShoePoints(1 + s * 2, 1)) / 2);
                ChVector2<> pc((ShoePoints(1 + s * 2, 0) + ShoePoints(2 + s * 2, 0)) / 2, (ShoePoints(1 + s * 2, 1) + ShoePoints(2 + s * 2, 1)) / 2);
                double as = std::atan2(ShoePoints(1 + s * 2, 1) - ShoePoints(s * 2, 1), ShoePoints(1 + s * 2, 0) - ShoePoints(s * 2, 0));
                double ac = std::atan2(ShoePoints(2 + s * 2, 1) - ShoePoints(1 + s * 2, 1), ShoePoints(2 + s * 2, 0) - ShoePoints(1 + s * 2, 0));

                CreateTrackShoe(s, ps, pc, as, ac);
            }
            ChVector2<> ps((ShoePoints(ShoePoints.GetRows() - 3, 0) + ShoePoints(ShoePoints.GetRows() - 2, 0)) / 2, (ShoePoints(ShoePoints.GetRows() - 3, 1) + ShoePoints(ShoePoints.GetRows() - 2, 1)) / 2);
            ChVector2<> pc((ShoePoints(ShoePoints.GetRows() - 2, 0) + ShoePoints(0, 0)) / 2, (ShoePoints(ShoePoints.GetRows() - 2, 1) + ShoePoints(0, 1)) / 2);
            double as = std::atan2(ShoePoints(ShoePoints.GetRows() - 2, 1) - ShoePoints(ShoePoints.GetRows() - 3, 1), ShoePoints(ShoePoints.GetRows() - 2, 0) - ShoePoints(ShoePoints.GetRows() - 3, 0));
            double ac = std::atan2(ShoePoints(0, 1) - ShoePoints(ShoePoints.GetRows() - 2, 1), ShoePoints(0, 0) - ShoePoints(ShoePoints.GetRows() - 2, 0));

            CreateTrackShoe((ShoePoints.GetRows() / 2)-1, ps, pc, as, ac);

            GetLog() << "Track assembly done.  Number of track shoes: " << ShoePoints.GetRows()/2 << "\n";

        }

        void ChTrackAssemblyRigidCB::FindCircleTangentPoints(ChVector2<> Circle1Pos,
            double Circle1Rad,
            ChVector2<> Circle2Pos,
            double Circle2Rad,
            ChVector2<>& Tan1Pnt1,
            ChVector2<>& Tan1Pnt2,
            ChVector2<>& Tan2Pnt1,
            ChVector2<>& Tan2Pnt2
            ){
            // Based on https://en.wikipedia.org/wiki/Tangent_lines_to_circles with modifications

            double x1 = Circle1Pos.x();
            double x2 = Circle2Pos.x();
            double y1 = Circle1Pos.y();
            double y2 = Circle2Pos.y();
            double r1 = Circle1Rad;
            double r2 = Circle2Rad;

            double beta = std::asin((r2 - r1) / std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2)));
            double gamma = std::atan2((y2 - y1), (x2 - x1));

            Tan1Pnt1 = ChVector2<>(x1 + r1*std::cos(gamma + (CH_C_PI_2 + beta)), y1 + r1*std::sin(gamma + (CH_C_PI_2 + beta)));
            Tan1Pnt2 = ChVector2<>(x2 + r2*std::cos(gamma + (CH_C_PI_2 + beta)), y2 + r2*std::sin(gamma + (CH_C_PI_2 + beta)));

            //Pick the external tangent on the other side of the circle
            Tan2Pnt1 = ChVector2<>(x1 + r1*std::cos(gamma - (CH_C_PI_2 + beta)), y1 + r1*std::sin(gamma - (CH_C_PI_2 + beta)));
            Tan2Pnt2 = ChVector2<>(x2 + r2*std::cos(gamma - (CH_C_PI_2 + beta)), y2 + r2*std::sin(gamma - (CH_C_PI_2 + beta)));
        }

        void ChTrackAssemblyRigidCB::CheckCircleCircle(bool& found, 
            ChVector2<>& Point, 
            ChMatrixDynamic<>& Features, 
            size_t FeatureIdx, 
            ChVector2<> StartingPoint, 
            double Radius){

            // Code was based on http://mathworld.wolfram.com/Circle-CircleIntersection.html

            found = false;
            Point.x() = 0;
            Point.y() = 0;

            //Check for any intersection
            ChVector2<> ArcCenter(Features(FeatureIdx, 1), Features(FeatureIdx, 2));
            double StartingArcAngle = Features(FeatureIdx, 3);
            double EndingArcAngle = Features(FeatureIdx, 4);

            double R = Features(FeatureIdx, 5);
            double r = Radius;
            ChVector2<>temp = ArcCenter - StartingPoint;
            double d = temp.Length();

            double y2 = (4 * d *d * R *R - std::pow(d *d - r *r + R *R, 2)) / (4 * d * d);

            if (y2 < 0){
                return; //No intersection
            }

            //Now check the two possible points(could be a degenerate case of 1 point)

            double theta = std::acos(((R *R + d *d) - r *r) / (2 * R*d));

            ChVector2<> Ctr = StartingPoint - ArcCenter;
            double Angle1 = std::atan2(Ctr.y(), Ctr.x()) - theta;
            double Angle2 = std::atan2(Ctr.y(), Ctr.x()) + theta - CH_C_2PI;

            while (Angle1 < StartingArcAngle){
                Angle1 += CH_C_2PI;
            }
            while (Angle2 < StartingArcAngle){
                Angle2 += CH_C_2PI;
            }


            double Angle0 = std::atan2(Ctr.y(), Ctr.x());
            double Angle1_check = Angle1 - 6 * CH_C_PI;
            double Angle2_check = Angle2 - 6 * CH_C_PI;

            while (Angle0 < 0){
                Angle0 += CH_C_2PI;
            }
            while (Angle1_check < Angle0){
                Angle1_check += CH_C_2PI;
            }
            while (Angle2_check < Angle0){
                Angle2_check += CH_C_2PI;
            }


            if (Angle1_check < Angle2_check){
                if (Angle1 <= EndingArcAngle){
                    found = true;
                    Point.x() = ArcCenter.x() +R*std::cos(Angle1);
                    Point.y() = ArcCenter.y() + R*std::sin(Angle1);
                }
            }
            else{
                if (Angle2 <= EndingArcAngle){
                    found = true;
                    Point.x() = ArcCenter.x() + R*std::cos(Angle2);
                    Point.y() = ArcCenter.y() + R*std::sin(Angle2);
                }
            }
        }


        void ChTrackAssemblyRigidCB::CheckCircleLine(bool& found,
            ChVector2<>& Point,
            ChMatrixDynamic<>& Features,
            size_t FeatureIdx,
            ChVector2<> StartingPoint,
            double Radius){

            // Code was based on http://mathworld.wolfram.com/Circle-LineIntersection.html

            found = false;
            Point.x() = 0;
            Point.y() = 0;

            double x1 = Features(FeatureIdx, 1) - StartingPoint.x();
            double y1 = Features(FeatureIdx, 2) - StartingPoint.y();
            double x2 = Features(FeatureIdx, 3) - StartingPoint.x();
            double y2 = Features(FeatureIdx, 4) - StartingPoint.y();
            ChVector2<> LinePnt1(x1, y1);
            ChVector2<> LinePnt2(x2, y2);
            double r = Radius;


            double dx = x2 - x1;
            double dy = y2 - y1;
            double dr = std::sqrt(dx *dx + dy *dy);
            double D = x1*y2 - x2*y1;

            double discriminant = (r*r * dr*dr) - (D*D);

            if (discriminant < 0){
                return; //No intersection
            }

            double xint_1;
            double xint_2;

            if (dy < 0){
                xint_1 = (D*dy - dx*std::sqrt(discriminant)) / (dr * dr);
                xint_2 = (D*dy + dx*std::sqrt(discriminant)) / (dr * dr);
            }
            else{
                xint_1 = (D*dy + dx*std::sqrt(discriminant)) / (dr * dr);
                xint_2 = (D*dy - dx*std::sqrt(discriminant)) / (dr * dr);
            }

            double yint_1 = (-D*dx + std::abs(dy)*std::sqrt(discriminant)) / (dr * dr);
            double yint_2 = (-D*dx - std::abs(dy)*std::sqrt(discriminant)) / (dr * dr);

            ChVector2<> intPnt1 (xint_1, yint_1);
            ChVector2<> intPnt2 (xint_2, yint_2);

            ChVector2<> Line = LinePnt2 - LinePnt1;
            if (Line.Dot(intPnt1 - LinePnt1) > Line.Dot(intPnt2 - LinePnt1)){
                auto temp = intPnt1;
                intPnt1 = intPnt2;
                intPnt2 = temp;
            }

            ChVector2<> intPnt2mLinePnt1 = intPnt2 - LinePnt1;
            ChVector2<> intPnt2mLinePnt2 = intPnt2 - LinePnt2;
            if ((intPnt2mLinePnt1.Length() <= dr) && (intPnt2mLinePnt2.Length() <= dr)){
                found = true;
                Point = intPnt2 + StartingPoint;
            }

        }


    }  // end namespace vehicle
}  // end namespace chrono
