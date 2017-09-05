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
// Base class for a continuous band track assembly using rigid-link track shoes
// (template definition).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

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
    // Position of sprocket (in chassis reference frame).
    ChVector<> sprocket_pos_3d = chassis->TransformPointParentToLocal(m_sprocket->GetGearBody()->GetPos());

    m_chassis = chassis;
    m_sprocket_offset = sprocket_pos_3d.y();

    // ----------------------------------------------------------
    // Settings:
    // ----------------------------------------------------------
    double FirstShoeFractionalOffset = -0.5;  // Shift the start point by a fraction of a shoe length so that the
                                              // assembly can start with the bekt tooth aligned with the sprocket grove
    double Tolerance =
        1e-12;  // Tolerance on how close the iterative algorithm will try to connect the beginning and end of the belt
    int Maxiter = 200;  // Stop after this number of iterations to prevent an infinite loop in case something unexpected
                        // goes wrong

    // ----------------------------------------------------------
    // Read in the geometric parameters for the track assembly
    // ----------------------------------------------------------

    // Number of track shoes and road wheels.
    int num_shoes = static_cast<int>(m_shoes.size());
    int num_wheels = static_cast<int>(m_suspensions.size());

    ChVectorDynamic<> ShoeConnectionLengths(1 + m_shoes[0]->GetNumWebSegments());
    ShoeConnectionLengths(0) = m_shoes[0]->GetToothBaseLength();

    double seg_length = m_shoes[0]->GetWebLength() / m_shoes[0]->GetNumWebSegments();
    for (int is = 0; is < m_shoes[0]->GetNumWebSegments(); is++) {
        ShoeConnectionLengths(1 + is) = seg_length;
    }

    // X & Y coordinates for the sprocket, idler, and all of the road wheels
    ChMatrixDynamic<> CirclePosAll(2 + num_wheels, 2);
    // Radii for the sprocket, idler, and all of the road wheels
    ChVectorDynamic<> CircleRadiusAll(2 + num_wheels);

    // ChVector<> sprocket_pos_3d = chassis->TransformPointParentToLocal(m_sprocket->GetGearBody()->GetPos());
    ChVector<> idler_pos_3d = chassis->TransformPointParentToLocal(m_idler->GetWheelBody()->GetPos());

    CirclePosAll(0, 0) = sprocket_pos_3d.x();
    CirclePosAll(0, 1) = sprocket_pos_3d.z();
    CircleRadiusAll(0) = m_sprocket->GetAssemblyRadius();

    CirclePosAll(1, 0) = idler_pos_3d.x();
    CirclePosAll(1, 1) = idler_pos_3d.z();
    CircleRadiusAll(1) = m_idler->GetWheelRadius();

    for (int i = 0; i < num_wheels; i++) {
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
    for (int i = 0; i < CirclePosAll.GetRows(); i++) {
        AverageXPos += CirclePosAll(i, 0);
    }
    AverageXPos /= CirclePosAll.GetRows();

    bool SprocketInFront = (CirclePosAll(0, 0) >= AverageXPos) ? true : false;
    double StartingAngle = (CirclePosAll(0, 0) >= AverageXPos) ? 0 : CH_C_PI;

    // Start building the path around the sprocket, idler, and wheels

    int Current_Circle = 0;
    int NumCirclesOnPath = 0;
    ChMatrixDynamic<> TangentPoints(CirclePosAll.GetRows(), 4);  // Will be resized later
    ChVectorDynamic<int> CircleIndex(CirclePosAll.GetRows());    // Will be resized later

    // At most each circle is visted once.  Something went wrong if
    // the loop is not closed within this many passes through the algorithm
    int iter;
    for (iter = 0; iter < CirclePosAll.GetRows(); iter++) {
        // Step 2: Create an ordered list of the circles with repect to their
        // distance from the starting circle;

        std::vector<double> Distances;
        std::vector<int> DistanceIndices;
        Distances.reserve(CirclePosAll.GetRows() - 1);
        DistanceIndices.reserve(CirclePosAll.GetRows() - 1);

        for (int c = 0; c < CirclePosAll.GetRows(); c++) {
            if (c == Current_Circle) {  // Don't count the distance between the circle and itself
                continue;
            }

            ChVector2<> Dist(CirclePosAll(c, 0) - CirclePosAll(Current_Circle, 0),
                             CirclePosAll(c, 1) - CirclePosAll(Current_Circle, 1));
            Distances.push_back(Dist.Length2());
            DistanceIndices.push_back(c);
        }

        std::vector<size_t> idx(Distances.size());
        std::size_t n(0);
        std::generate(std::begin(idx), std::end(idx), [&] { return n++; });

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
        int nextCircle = 0;

        ChVector2<> Tan1Pnt1;
        ChVector2<> Tan1Pnt2;
        ChVector2<> Tan2Pnt1;
        ChVector2<> Tan2Pnt2;

        for (size_t i = 0; i < DistanceIndices.size(); i++) {
            // Find the tangent points between the current circle and the next circle to check the wrap angles on
            ChVector2<> Circle1Pos(CirclePosAll(Current_Circle, 0), CirclePosAll(Current_Circle, 1));
            double Circle1Rad = CircleRadiusAll(Current_Circle);
            ChVector2<> Circle2Pos(CirclePosAll(DistanceIndices[idx[i]], 0), CirclePosAll(DistanceIndices[idx[i]], 1));
            double Circle2Rad = CircleRadiusAll(DistanceIndices[idx[i]]);

            FindCircleTangentPoints(Circle1Pos, Circle1Rad, Circle2Pos, Circle2Rad, Tan1Pnt1, Tan1Pnt2, Tan2Pnt1,
                                    Tan2Pnt2);

            // Calculate the wrap angles for each of the 2 external circle tangent lines
            double Angle1 = std::atan2(Tan1Pnt1.y() - CirclePosAll(Current_Circle, 1),
                                       Tan1Pnt1.x() - CirclePosAll(Current_Circle, 0));
            double Angle2 = std::atan2(Tan2Pnt1.y() - CirclePosAll(Current_Circle, 1),
                                       Tan2Pnt1.x() - CirclePosAll(Current_Circle, 0));

            // Ensure that all of the angles are greater than the starting angle by at least a little bit to prevent
            // numerical noise
            while (Angle1 < StartingAngle + .00001) {
                Angle1 += CH_C_2PI;
            }
            while (Angle2 < StartingAngle + .00001) {
                Angle2 += CH_C_2PI;
            }

            // If this is the first point that is examined, then save that point as the inital comparison value.
            if (i == 0) {
                minAngle = Angle1;
                TangentPoints(iter, 0) = Tan1Pnt1.x();
                TangentPoints(iter, 1) = Tan1Pnt1.y();
                TangentPoints(iter, 2) = Tan1Pnt2.x();
                TangentPoints(iter, 3) = Tan1Pnt2.y();

                nextCircle = DistanceIndices[idx[i]];
            }

            if (Angle1 < minAngle) {
                // Save Tangent
                minAngle = Angle1;
                TangentPoints(iter, 0) = Tan1Pnt1.x();
                TangentPoints(iter, 1) = Tan1Pnt1.y();
                TangentPoints(iter, 2) = Tan1Pnt2.x();
                TangentPoints(iter, 3) = Tan1Pnt2.y();

                nextCircle = DistanceIndices[idx[i]];
            }

            if (Angle2 < minAngle) {
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
        StartingAngle = std::atan2(TangentPoints(iter, 3) - CirclePosAll(Current_Circle, 1),
                                   TangentPoints(iter, 2) - CirclePosAll(Current_Circle, 0));
        if (StartingAngle < 0) {
            StartingAngle += CH_C_2PI;
        }

        // Check to see if the wrap has made it back onto the strating circle (sprocket)
        if (Current_Circle == 0) {
            break;
        }
    }

    // Save the reduced geometry corresponding to the tight wrapping of the belt path.

    // X & Y coordinates for the sprocket and idler or any of the road wheels on the belt wrap loop
    ChMatrixDynamic<> CirclePos(iter + 1, 2);
    // radii for the sprocket and idler or any of the road wheels on the belt wrap loop
    ChVectorDynamic<> CircleRadius(iter + 1);

    // Move the sprocket to the beginning position from the end
    CirclePos(0, 0) = CirclePosAll(CircleIndex(iter), 0);
    CirclePos(0, 1) = CirclePosAll(CircleIndex(iter), 1);
    CircleRadius(0) = CircleRadiusAll(CircleIndex(iter));
    for (int i = 0; i < (iter); i++) {
        CirclePos(i + 1, 0) = CirclePosAll(CircleIndex(i), 0);
        CirclePos(i + 1, 1) = CirclePosAll(CircleIndex(i), 1);
        CircleRadius(i + 1) = CircleRadiusAll(CircleIndex(i));
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

    // Start by calculating the original tangent(constant) and arc lengths (variable)
    // to determine the inital scale for sprocket/idler/road wheel circles

    double CombineShoeLength = 0;
    for (int i = 0; i < ShoeConnectionLengths.GetRows(); i++) {
        CombineShoeLength += ShoeConnectionLengths(i);  // sum all of the seperate lengths per shoe
    }
    CombineShoeLength *= num_shoes;

    // for each circle [Arc Length, Starting Angle of Arc, Ending Angle of Arc]
    ChMatrixDynamic<> Arcs(CircleRadius.GetRows(), 3);

    StartingAngle = std::atan2(TangentPoints(CircleRadius.GetRows() - 1, 3) - CirclePos(0, 1),
                               TangentPoints(CircleRadius.GetRows() - 1, 2) - CirclePos(0, 0));
    double EndingAngle = std::atan2(TangentPoints(0, 1) - CirclePos(0, 1), TangentPoints(0, 0) - CirclePos(0, 0));

    if (StartingAngle < 0) {
        StartingAngle += CH_C_2PI;
    }
    while (EndingAngle < StartingAngle) {
        EndingAngle += CH_C_2PI;
    }

    Arcs(0, 0) = EndingAngle - StartingAngle;
    Arcs(0, 1) = StartingAngle;
    Arcs(0, 2) = EndingAngle;

    for (int i = 1; i < CircleRadius.GetRows(); i++) {
        StartingAngle =
            std::atan2(TangentPoints(i - 1, 3) - CirclePos(i, 1), TangentPoints(i - 1, 2) - CirclePos(i, 0));
        EndingAngle = std::atan2(TangentPoints(i, 1) - CirclePos(i, 1), TangentPoints(i, 0) - CirclePos(i, 0));

        if (StartingAngle < 0) {
            StartingAngle += CH_C_2PI;
        }
        while (EndingAngle < StartingAngle) {
            EndingAngle += CH_C_2PI;
        }

        Arcs(i, 0) = EndingAngle - StartingAngle;
        Arcs(i, 1) = StartingAngle;
        Arcs(i, 2) = EndingAngle;
    }

    // Calculate the belt wrap length
    double LengthOfArcs = 0;
    double LengthOfTangents = 0;

    for (int i = 0; i < Arcs.GetRows(); i++) {
        LengthOfArcs += (Arcs(i, 0) * CircleRadius(i));

        LengthOfTangents += std::sqrt(std::pow(TangentPoints(i, 2) - TangentPoints(i, 0), 2) +
                                      std::pow(TangentPoints(i, 3) - TangentPoints(i, 1), 2));
    }

    // Calculate how the arcs need to be scaled to fit so that tangent length +
    // the arc lengths = the belt length.This should be a slight underestimate
    // of the needed scale.

    double DeltaRadius = (CombineShoeLength - (LengthOfArcs + LengthOfTangents)) /
                         (CH_C_2PI);  // Divide by 2*PI since the belt wrap forms a closed loop/circle
    ScaleMin = DeltaRadius;

    ChMatrixDynamic<> ShoePoints(1 + num_shoes * ShoeConnectionLengths.GetRows(), 2);

    for (iter = 0; iter < Maxiter; iter++) {
        ChVectorDynamic<> ScaledCircleRadius = CircleRadius;
        for (int i = 0; i < ScaledCircleRadius.GetRows(); i++) {
            ScaledCircleRadius(i) += DeltaRadius;
        }

        // Calculate the new tangents based off of the arc starting and ending points for each circle
        for (int i = 0; i < ScaledCircleRadius.GetRows(); i++) {
            double StartingArcPoint_x = cos(Arcs(i, 1)) * ScaledCircleRadius(i) + CirclePos(i, 0);
            double StartingArcPoint_z = sin(Arcs(i, 1)) * ScaledCircleRadius(i) + CirclePos(i, 1);

            double EndingArcPoint_x = cos(Arcs(i, 2)) * ScaledCircleRadius(i) + CirclePos(i, 0);
            double EndingArcPoint_z = sin(Arcs(i, 2)) * ScaledCircleRadius(i) + CirclePos(i, 1);

            if (i == 0) {  // Sprocket
                TangentPoints(0, 0) = EndingArcPoint_x;
                TangentPoints(0, 1) = EndingArcPoint_z;
                TangentPoints(ScaledCircleRadius.GetRows() - 1, 2) = StartingArcPoint_x;
                TangentPoints(ScaledCircleRadius.GetRows() - 1, 3) = StartingArcPoint_z;
            } else {
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
        StartingAngle =
            2 * std::asin(FirstShoeFractionalOffset * ShoeConnectionLengths(0) / (2 * ScaledCircleRadius(0)));

        if (!SprocketInFront) {
            StartingAngle += CH_C_PI;  // Start on the back side of the sprocket instead of the front
        }

        while (StartingAngle < Arcs(0, 1)) {
            StartingAngle += CH_C_2PI;
        }

        ShoePoints.FillElem(0);  // Zeros out the matrix since the Shoe Points are recalculated each iteration
        ShoePoints(0, 0) = std::cos(StartingAngle) * ScaledCircleRadius(0) + CirclePos(0, 0);
        ShoePoints(0, 1) = std::sin(StartingAngle) * ScaledCircleRadius(0) + CirclePos(0, 1);

        int shoelinktype = 0;
        int CurrentFeature = 0;

        // Build features array used for determining shoe points on the arcs and tangents
        ChMatrixDynamic<> Features(2 * ScaledCircleRadius.GetRows(), 6);

        for (int i = 0; i < ScaledCircleRadius.GetRows(); i++) {
            // Add Arc Wrap Feature (id = 0)
            Features(2 * i, 0) = 0;  // feature type id
            Features(2 * i, 1) = CirclePos(i, 0);
            Features(2 * i, 2) = CirclePos(i, 1);
            Features(2 * i, 3) = Arcs(i, 1);
            Features(2 * i, 4) = Arcs(i, 2);
            Features(2 * i, 5) = ScaledCircleRadius(i);

            // Add Tangent Wrap Feature (id = 1)
            Features(1 + (2 * i), 0) = 1;  // feature type id
            Features(1 + (2 * i), 1) = TangentPoints(i, 0);
            Features(1 + (2 * i), 2) = TangentPoints(i, 1);
            Features(1 + (2 * i), 3) = TangentPoints(i, 2);
            Features(1 + (2 * i), 4) = TangentPoints(i, 3);
            Features(1 + (2 * i), 5) = 0;  // Unused
        }

        // Calculate the remaining ShoePoints
        bool FoundPoint = false;
        bool InitalSprocketWrap = true;
        double ExtraLength = 0;
        double CurrentAngle = 0;
        int shoeidx;

        for (shoeidx = 1; shoeidx < ShoePoints.GetRows(); shoeidx++) {
            ChVector2<> Point;
            ChVector2<> StartingPoint(ShoePoints(shoeidx - 1, 0), ShoePoints(shoeidx - 1, 1));

            // Make sure that all the features are only searched once to prevent and endless loop
            for (int c = 0; c < Features.GetRows(); c++) {
                if (Features(CurrentFeature, 0) == 0) {
                    CheckCircleCircle(FoundPoint, Point, Features, CurrentFeature, StartingPoint,
                                      ShoeConnectionLengths(shoelinktype));
                } else {
                    CheckCircleLine(FoundPoint, Point, Features, CurrentFeature, StartingPoint,
                                    ShoeConnectionLengths(shoelinktype));
                }

                if (FoundPoint) {
                    break;  // Still on the current Feature (arc or tangent)
                }

                InitalSprocketWrap = false;
                CurrentFeature += 1;
                if (CurrentFeature >= Features.GetRows()) {
                    CurrentFeature = 0;
                }
            }

            if (!FoundPoint) {
                std::cout << "Belt Assembly ERROR: Something went wrong" << std::endl;
                assert(FoundPoint);
                return true;
            }

            ShoePoints(shoeidx, 0) = Point.x();
            ShoePoints(shoeidx, 1) = Point.y();

            shoelinktype += 1;
            if (shoelinktype >= ShoeConnectionLengths.GetRows()) {
                shoelinktype = 0;
            }

            ExtraLength = 0;
            // Check to see if I have wrapped past the inital point
            if ((!InitalSprocketWrap) && (CurrentFeature == 0)) {
                CurrentAngle = std::atan2(Point.y() - CirclePos(0, 1), Point.x() - CirclePos(0, 0));
                while (CurrentAngle < Features(0, 3)) {
                    CurrentAngle += CH_C_2PI;
                }
                if (CurrentAngle > StartingAngle) {
                    ExtraLength = (CurrentAngle - StartingAngle) * Features(0, 5);
                    break;
                }
            }
        }

        for (int j = shoeidx + 1; j < ShoePoints.GetRows(); j++) {
            ExtraLength += ShoeConnectionLengths(shoelinktype);

            shoelinktype += 1;
            if (shoelinktype >= ShoeConnectionLengths.GetRows()) {
                shoelinktype = 0;
            }
        }

        if (CurrentFeature > 0) {
            ExtraLength = -std::sqrt(std::pow(ShoePoints(ShoePoints.GetRows() - 1, 0) - ShoePoints(0, 0), 2) +
                                     std::pow(ShoePoints(ShoePoints.GetRows() - 1, 1) - ShoePoints(0, 1), 2));
        } else if ((ExtraLength == 0) && (CurrentAngle > StartingAngle)) {
            ExtraLength = std::sqrt(std::pow(ShoePoints(ShoePoints.GetRows() - 1, 0) - ShoePoints(0, 0), 2) +
                                    std::pow(ShoePoints(ShoePoints.GetRows() - 1, 1) - ShoePoints(0, 1), 2));
        } else if (ExtraLength == 0) {
            ExtraLength = -std::sqrt(std::pow(ShoePoints(ShoePoints.GetRows() - 1, 0) - ShoePoints(0, 0), 2) +
                                     std::pow(ShoePoints(ShoePoints.GetRows() - 1, 1) - ShoePoints(0, 1), 2));
        }

        if (abs(ExtraLength) < Tolerance) {
            std::cout << "Belt Wrap Algorithm Conveged after " << iter << " iterations - Extra Length: " << ExtraLength
                      << " - Length Tolerance: " << Tolerance << std::endl;
            break;
        }

        if (ExtraLength > 0) {
            if (DeltaRadius > ScaleMin) {
                ScaleMin = DeltaRadius;
                RemainingLenMinScale = ExtraLength;
            }
        } else {
            if ((DeltaRadius < ScaleMax) || (ScaleMax == 0)) {
                ScaleMax = DeltaRadius;
                RemainingLenMaxScale = ExtraLength;
            }
        }

        if (iter == 0) {
            DeltaRadius = DeltaRadius + ExtraLength * 10;
        } else {
            DeltaRadius = (ScaleMin + ScaleMax) / 2;
        }
    }

    // Now create all of the track shoes at the located points
    auto num_shoe_elements = ShoeConnectionLengths.GetRows();
    for (int s = 0; s < num_shoes; s++) {
        std::vector<ChCoordsys<>> shoe_components_coordsys;
        for (int i = 0; i < num_shoe_elements; i++) {
            ChVector<> loc(
                (ShoePoints(i + s * num_shoe_elements, 0) + ShoePoints(i + 1 + s * num_shoe_elements, 0)) / 2,
                m_sprocket_offset,
                (ShoePoints(i + s * num_shoe_elements, 1) + ShoePoints(i + 1 + s * num_shoe_elements, 1)) / 2);

            double ang =
                std::atan2(ShoePoints(i + 1 + s * num_shoe_elements, 1) - ShoePoints(i + s * num_shoe_elements, 1),
                           ShoePoints(i + 1 + s * num_shoe_elements, 0) - ShoePoints(i + s * num_shoe_elements, 0));
            ChQuaternion<> rot = Q_from_AngY(-ang);  // Negative of the angle in 3D

            shoe_components_coordsys.push_back(ChCoordsys<>(loc, rot));
        }

        // Set index within the track assembly
        m_shoes[s]->SetIndex(s);
        // Initialize the track shoe system
        m_shoes[s]->Initialize(m_chassis, shoe_components_coordsys);
    }

    GetLog() << "Track assembly done.  Number of track shoes: " << ShoePoints.GetRows() / 2 << "\n";

    return true;
}

void ChTrackAssemblyRigidCB::FindCircleTangentPoints(ChVector2<> Circle1Pos,
                                                     double Circle1Rad,
                                                     ChVector2<> Circle2Pos,
                                                     double Circle2Rad,
                                                     ChVector2<>& Tan1Pnt1,
                                                     ChVector2<>& Tan1Pnt2,
                                                     ChVector2<>& Tan2Pnt1,
                                                     ChVector2<>& Tan2Pnt2) {
    // Based on https://en.wikipedia.org/wiki/Tangent_lines_to_circles with modifications

    double x1 = Circle1Pos.x();
    double x2 = Circle2Pos.x();
    double y1 = Circle1Pos.y();
    double y2 = Circle2Pos.y();
    double r1 = Circle1Rad;
    double r2 = Circle2Rad;

    double beta = std::asin((r2 - r1) / std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2)));
    double gamma = std::atan2((y2 - y1), (x2 - x1));

    Tan1Pnt1 =
        ChVector2<>(x1 + r1 * std::cos(gamma + (CH_C_PI_2 + beta)), y1 + r1 * std::sin(gamma + (CH_C_PI_2 + beta)));
    Tan1Pnt2 =
        ChVector2<>(x2 + r2 * std::cos(gamma + (CH_C_PI_2 + beta)), y2 + r2 * std::sin(gamma + (CH_C_PI_2 + beta)));

    // Pick the external tangent on the other side of the circle
    Tan2Pnt1 =
        ChVector2<>(x1 + r1 * std::cos(gamma - (CH_C_PI_2 + beta)), y1 + r1 * std::sin(gamma - (CH_C_PI_2 + beta)));
    Tan2Pnt2 =
        ChVector2<>(x2 + r2 * std::cos(gamma - (CH_C_PI_2 + beta)), y2 + r2 * std::sin(gamma - (CH_C_PI_2 + beta)));
}

void ChTrackAssemblyRigidCB::CheckCircleCircle(bool& found,
                                               ChVector2<>& Point,
                                               ChMatrixDynamic<>& Features,
                                               int FeatureIdx,
                                               ChVector2<> StartingPoint,
                                               double Radius) {
    // Code was based on http://mathworld.wolfram.com/Circle-CircleIntersection.html

    found = false;
    Point.x() = 0;
    Point.y() = 0;

    // Check for any intersection
    ChVector2<> ArcCenter(Features(FeatureIdx, 1), Features(FeatureIdx, 2));
    double StartingArcAngle = Features(FeatureIdx, 3);
    double EndingArcAngle = Features(FeatureIdx, 4);

    double R = Features(FeatureIdx, 5);
    double r = Radius;
    ChVector2<> temp = ArcCenter - StartingPoint;
    double d = temp.Length();

    double y2 = (4 * d * d * R * R - std::pow(d * d - r * r + R * R, 2)) / (4 * d * d);

    if (y2 < 0) {
        return;  // No intersection
    }

    // Now check the two possible points(could be a degenerate case of 1 point)

    double theta = std::acos(((R * R + d * d) - r * r) / (2 * R * d));

    ChVector2<> Ctr = StartingPoint - ArcCenter;
    double Angle1 = std::atan2(Ctr.y(), Ctr.x()) - theta;
    double Angle2 = std::atan2(Ctr.y(), Ctr.x()) + theta - CH_C_2PI;

    while (Angle1 < StartingArcAngle) {
        Angle1 += CH_C_2PI;
    }
    while (Angle2 < StartingArcAngle) {
        Angle2 += CH_C_2PI;
    }

    double Angle0 = std::atan2(Ctr.y(), Ctr.x());
    double Angle1_check = Angle1 - 6 * CH_C_PI;
    double Angle2_check = Angle2 - 6 * CH_C_PI;

    while (Angle0 < 0) {
        Angle0 += CH_C_2PI;
    }
    while (Angle1_check < Angle0) {
        Angle1_check += CH_C_2PI;
    }
    while (Angle2_check < Angle0) {
        Angle2_check += CH_C_2PI;
    }

    if (Angle1_check < Angle2_check) {
        if (Angle1 <= EndingArcAngle) {
            found = true;
            Point.x() = ArcCenter.x() + R * std::cos(Angle1);
            Point.y() = ArcCenter.y() + R * std::sin(Angle1);
        }
    } else {
        if (Angle2 <= EndingArcAngle) {
            found = true;
            Point.x() = ArcCenter.x() + R * std::cos(Angle2);
            Point.y() = ArcCenter.y() + R * std::sin(Angle2);
        }
    }
}

void ChTrackAssemblyRigidCB::CheckCircleLine(bool& found,
                                             ChVector2<>& Point,
                                             ChMatrixDynamic<>& Features,
                                             int FeatureIdx,
                                             ChVector2<> StartingPoint,
                                             double Radius) {
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
    double dr = std::sqrt(dx * dx + dy * dy);
    double D = x1 * y2 - x2 * y1;

    double discriminant = (r * r * dr * dr) - (D * D);

    if (discriminant < 0) {
        return;  // No intersection
    }

    double xint_1;
    double xint_2;

    if (dy < 0) {
        xint_1 = (D * dy - dx * std::sqrt(discriminant)) / (dr * dr);
        xint_2 = (D * dy + dx * std::sqrt(discriminant)) / (dr * dr);
    } else {
        xint_1 = (D * dy + dx * std::sqrt(discriminant)) / (dr * dr);
        xint_2 = (D * dy - dx * std::sqrt(discriminant)) / (dr * dr);
    }

    double yint_1 = (-D * dx + std::abs(dy) * std::sqrt(discriminant)) / (dr * dr);
    double yint_2 = (-D * dx - std::abs(dy) * std::sqrt(discriminant)) / (dr * dr);

    ChVector2<> intPnt1(xint_1, yint_1);
    ChVector2<> intPnt2(xint_2, yint_2);

    ChVector2<> Line = LinePnt2 - LinePnt1;
    if (Line.Dot(intPnt1 - LinePnt1) > Line.Dot(intPnt2 - LinePnt1)) {
        auto temp = intPnt1;
        intPnt1 = intPnt2;
        intPnt2 = temp;
    }

    ChVector2<> intPnt2mLinePnt1 = intPnt2 - LinePnt1;
    ChVector2<> intPnt2mLinePnt2 = intPnt2 - LinePnt2;
    if ((intPnt2mLinePnt1.Length() <= dr) && (intPnt2mLinePnt2.Length() <= dr)) {
        found = true;
        Point = intPnt2 + StartingPoint;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
