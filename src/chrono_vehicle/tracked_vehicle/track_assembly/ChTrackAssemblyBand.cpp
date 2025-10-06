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
// Base class for continuous band track assembly using rigid tread.
// Derived classes specify the actual template definitions, using different track
// shoes.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <utility>

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBand.h"

#include "chrono/utils/ChUtilsInputOutput.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Calculate points for continuous band track assembly.
//
// Returns true if the track shoes are initialized in a counter clockwise
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
bool ChTrackAssemblyBand::FindAssemblyPoints(std::shared_ptr<ChBodyAuxRef> chassis,
                                             int num_shoes,
                                             const std::vector<double>& connection_lengths,
                                             std::vector<ChVector2d>& shoe_points) {
    // Shift the start point by a fraction of a shoe length so that the assembly can start with the belt
    // tooth aligned with the sprocket grove
    double FirstShoeFractionalOffset = -0.5;

    // Tolerance on how close the iterative algorithm will try to connect the beginning and end of the belt
    double Tolerance = 1e-12;

    // Stop after this number of iterations to prevent an infinite loop in case something unexpected goes wrong
    int Maxiter = 200;

    // Position of sprocket (in chassis reference frame)
    ChVector3d sprocket_pos_3d = chassis->TransformPointParentToLocal(m_sprocket->GetGearBody()->GetPos());
    m_sprocket_offset = sprocket_pos_3d.y();

    // Number of wheels
    int num_wheels = static_cast<int>(m_suspensions.size());

    // X & Z coordinates for the sprocket, idler, and all of the road wheels
    std::vector<ChVector2d> CirclePosAll(2 + num_wheels);

    // Radii for the sprocket, idler, and all of the road wheels
    std::vector<double> CircleRadiusAll(2 + num_wheels);

    ChVector3d idler_pos_3d = chassis->TransformPointParentToLocal(m_idler->GetWheelBody()->GetPos());

    CirclePosAll[0].x() = sprocket_pos_3d.x();
    CirclePosAll[0].y() = sprocket_pos_3d.z();
    CircleRadiusAll[0] = m_sprocket->GetAssemblyRadius();

    CirclePosAll[1].x() = idler_pos_3d.x();
    CirclePosAll[1].y() = idler_pos_3d.z();
    CircleRadiusAll[1] = m_idler->GetWheelRadius();

    for (int i = 0; i < num_wheels; i++) {
        ChVector3d wheel_pos = chassis->TransformPointParentToLocal(m_suspensions[i]->GetWheelBody()->GetPos());
        CirclePosAll[i + 2].x() = wheel_pos.x();
        CirclePosAll[i + 2].y() = wheel_pos.z();
        CircleRadiusAll[i + 2] = m_suspensions[i]->GetWheelRadius();
    }

    // ----------------------------------------------------------
    // Find the path that encloses the sprocket, idler, and road wheels
    // Shrink wrap type algorithm
    // ----------------------------------------------------------
    // Step 1: Determine if the sprocket is ahead or behind of the other circles
    double AverageXPos = 0;
    for (int i = 0; i < CirclePosAll.size(); i++) {
        AverageXPos += CirclePosAll[i].x();
    }
    AverageXPos /= CirclePosAll.size();

    bool SprocketInFront = (CirclePosAll[0].x() >= AverageXPos) ? true : false;
    double StartingAngle = (CirclePosAll[0].x() >= AverageXPos) ? 0 : CH_PI;

    // Start building the path around the sprocket, idler, and wheels

    int Current_Circle = 0;

    // Tangent points (start and end) between consecutive circles
    std::vector<std::pair<ChVector2d, ChVector2d>> TangentPoints(CirclePosAll.size());

    // Vector to save the indices of the circles that form the boundaries of the belt wrap
    std::vector<int> CircleIndex(CirclePosAll.size());

    // At most each circle is visited once.  Something went wrong if
    // the loop is not closed within this many passes through the algorithm
    int iter;
    for (iter = 0; iter < CirclePosAll.size(); iter++) {
        // Step 2: Create an ordered list of the circles with respect to their distance from the starting circle

        std::vector<double> Distances;
        std::vector<int> DistanceIndices;
        Distances.reserve(CirclePosAll.size() - 1);
        DistanceIndices.reserve(CirclePosAll.size() - 1);

        for (int c = 0; c < CirclePosAll.size(); c++) {
            if (c == Current_Circle) {  // Don't count the distance between the circle and itself
                continue;
            }
            ChVector2d Dist = CirclePosAll[c] - CirclePosAll[Current_Circle];
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

        ChVector2d Tan1Pnt1;
        ChVector2d Tan1Pnt2;
        ChVector2d Tan2Pnt1;
        ChVector2d Tan2Pnt2;

        for (size_t i = 0; i < DistanceIndices.size(); i++) {
            // Find the tangent points between the current circle and the next circle to check the wrap angles on
            ChVector2d Circle1Pos = CirclePosAll[Current_Circle];
            double Circle1Rad = CircleRadiusAll[Current_Circle];
            ChVector2d Circle2Pos = CirclePosAll[DistanceIndices[idx[i]]];
            double Circle2Rad = CircleRadiusAll[DistanceIndices[idx[i]]];

            FindCircleTangentPoints(Circle1Pos, Circle1Rad, Circle2Pos, Circle2Rad, Tan1Pnt1, Tan1Pnt2, Tan2Pnt1,
                                    Tan2Pnt2);

            // Calculate the wrap angles for each of the 2 external circle tangent lines
            double Angle1 = std::atan2(Tan1Pnt1.y() - CirclePosAll[Current_Circle].y(),
                                       Tan1Pnt1.x() - CirclePosAll[Current_Circle].x());
            double Angle2 = std::atan2(Tan2Pnt1.y() - CirclePosAll[Current_Circle].y(),
                                       Tan2Pnt1.x() - CirclePosAll[Current_Circle].x());

            // Ensure that all of the angles are greater than the starting angle by at least a little bit to prevent
            // numerical noise
            while (Angle1 < StartingAngle + .00001) {
                Angle1 += CH_2PI;
            }
            while (Angle2 < StartingAngle + .00001) {
                Angle2 += CH_2PI;
            }

            // If this is the first point that is examined, then save that point as the initial comparison value.
            if (i == 0) {
                minAngle = Angle1;
                TangentPoints[iter].first = Tan1Pnt1;
                TangentPoints[iter].second = Tan1Pnt2;

                nextCircle = DistanceIndices[idx[i]];
            }

            if (Angle1 < minAngle) {
                // Save Tangent as it currently has the smallest change in the belt angle to wrap around to it
                minAngle = Angle1;
                TangentPoints[iter].first = Tan1Pnt1;
                TangentPoints[iter].second = Tan1Pnt2;

                nextCircle = DistanceIndices[idx[i]];
            }

            if (Angle2 < minAngle) {
                // Save Tangent as it currently has the smallest change in the belt angle to wrap around to it
                minAngle = Angle2;
                TangentPoints[iter].first = Tan2Pnt1;
                TangentPoints[iter].second = Tan2Pnt2;

                nextCircle = DistanceIndices[idx[i]];
            }
        }

        // Setup for the next loop through
        CircleIndex[iter] = nextCircle;
        Current_Circle = nextCircle;
        StartingAngle = std::atan2(TangentPoints[iter].second.y() - CirclePosAll[Current_Circle].y(),
                                   TangentPoints[iter].second.x() - CirclePosAll[Current_Circle].x());
        if (StartingAngle < 0) {
            StartingAngle += CH_2PI;
        }

        // Check to see if the wrap has made it back onto the starting circle (sprocket) and therefore would complete
        // the belt wrap
        if (Current_Circle == 0) {
            break;
        }
    }

    // Save the reduced geometry corresponding to the tight wrapping of the belt path.

    // X & Z coordinates for the sprocket and idler or any of the road wheels on the belt wrap loop
    std::vector<ChVector2d> CirclePos(iter + 1);
    // radii for the sprocket and idler or any of the road wheels on the belt wrap loop
    std::vector<double> CircleRadius(iter + 1);

    // Move the sprocket to the beginning position from the end
    CirclePos[0] = CirclePosAll[CircleIndex[iter]];
    CircleRadius[0] = CircleRadiusAll[CircleIndex[iter]];
    for (int i = 0; i < iter; i++) {
        CirclePos[i + 1] = CirclePosAll[CircleIndex[i]];
        CircleRadius[i + 1] = CircleRadiusAll[CircleIndex[i]];
    }

    //--------------------------------------------------------------------------
    // Fit the track around the outermost wrapping.
    //
    // Start the iterations with the scale such that the belt length equals the
    // outer wrapping circumference (too small of a scale, but not by much).
    // After that, overshoot the extra length by a factor of 10 to make sure
    // that the scale is too large.
    // Then binary search tree on the scale to converge.
    //
    // TODO: Make sure that there is too much, rather than too little track left
    // over so that last two shoes can be very slightly folded to exactly fit.
    //--------------------------------------------------------------------------

    double ScaleMin = 0;
    double ScaleMax = 0;

    // Start by calculating the original tangent(constant) and arc lengths (variable)
    // to determine the initial scale for sprocket/idler/road wheel circles

    double CombineShoeLength = 0;
    for (int i = 0; i < connection_lengths.size(); i++) {
        CombineShoeLength += connection_lengths[i];  // sum all of the separate lengths per shoe
    }
    CombineShoeLength *= num_shoes;

    // for each circle [Arc Length, Starting Angle of Arc, Ending Angle of Arc]
    ChMatrixDynamic<> Arcs(CircleRadius.size(), 3);

    StartingAngle = std::atan2(TangentPoints[CircleRadius.size() - 1].second.y() - CirclePos[0].y(),
                               TangentPoints[CircleRadius.size() - 1].second.x() - CirclePos[0].x());
    double EndingAngle =
        std::atan2(TangentPoints[0].first.y() - CirclePos[0].y(), TangentPoints[0].first.x() - CirclePos[0].x());

    if (StartingAngle < 0) {
        StartingAngle += CH_2PI;
    }
    while (EndingAngle < StartingAngle) {
        EndingAngle += CH_2PI;
    }

    Arcs(0, 0) = EndingAngle - StartingAngle;
    Arcs(0, 1) = StartingAngle;
    Arcs(0, 2) = EndingAngle;

    for (int i = 1; i < CircleRadius.size(); i++) {
        StartingAngle = std::atan2(TangentPoints[i - 1].second.y() - CirclePos[i].y(),
                                   TangentPoints[i - 1].second.x() - CirclePos[i].x());
        EndingAngle =
            std::atan2(TangentPoints[i].first.y() - CirclePos[i].y(), TangentPoints[i].first.x() - CirclePos[i].x());

        if (StartingAngle < 0) {
            StartingAngle += CH_2PI;
        }
        while (EndingAngle < StartingAngle) {
            EndingAngle += CH_2PI;
        }

        Arcs(i, 0) = EndingAngle - StartingAngle;
        Arcs(i, 1) = StartingAngle;
        Arcs(i, 2) = EndingAngle;
    }

    // Calculate the belt wrap length
    double LengthOfArcs = 0;
    double LengthOfTangents = 0;

    for (int i = 0; i < Arcs.rows(); i++) {
        LengthOfArcs += (Arcs(i, 0) * CircleRadius[i]);
        LengthOfTangents += (TangentPoints[i].second - TangentPoints[i].first).Length();
    }

    // Calculate how the arcs need to be scaled to fit so that tangent length +
    // the arc lengths = the belt length.This should be a slight underestimate
    // of the needed scale to place all of the belt wrapping positions.

    // Divide by 2*PI since the belt wrap forms a closed loop/circle
    double DeltaRadius = (CombineShoeLength - (LengthOfArcs + LengthOfTangents)) / (CH_2PI);
    ScaleMin = DeltaRadius;

    // Properly size the container of output points
    shoe_points.resize(1 + num_shoes * connection_lengths.size());

    for (iter = 0; iter < Maxiter; iter++) {
        auto ScaledCircleRadius = CircleRadius;
        for (int i = 0; i < ScaledCircleRadius.size(); i++) {
            ScaledCircleRadius[i] += DeltaRadius;
        }

        // Calculate the new tangents based off of the arc starting and ending points for each circle
        for (int i = 0; i < ScaledCircleRadius.size(); i++) {
            double StartingArcPoint_x = std::cos(Arcs(i, 1)) * ScaledCircleRadius[i] + CirclePos[i].x();
            double StartingArcPoint_z = std::sin(Arcs(i, 1)) * ScaledCircleRadius[i] + CirclePos[i].y();

            double EndingArcPoint_x = std::cos(Arcs(i, 2)) * ScaledCircleRadius[i] + CirclePos[i].x();
            double EndingArcPoint_z = std::sin(Arcs(i, 2)) * ScaledCircleRadius[i] + CirclePos[i].y();

            if (i == 0) {  // Sprocket
                TangentPoints[0].first.x() = EndingArcPoint_x;
                TangentPoints[0].first.y() = EndingArcPoint_z;
                TangentPoints[ScaledCircleRadius.size() - 1].second.x() = StartingArcPoint_x;
                TangentPoints[ScaledCircleRadius.size() - 1].second.y() = StartingArcPoint_z;
            } else {
                TangentPoints[i].first.x() = EndingArcPoint_x;
                TangentPoints[i].first.y() = EndingArcPoint_z;
                TangentPoints[i - 1].second.x() = StartingArcPoint_x;
                TangentPoints[i - 1].second.y() = StartingArcPoint_z;
            }
        }

        // Start wrapping the belt.Start with the first link all the way forward
        // (or all the way rearward) shifted by the desired offset which is used to
        // center the first track link to make orienting the sprocket profile easier

        // cordLength = 2 * r*sin(theta / 2)
        StartingAngle = 2 * std::asin(FirstShoeFractionalOffset * connection_lengths[0] / (2 * ScaledCircleRadius[0]));

        if (!SprocketInFront) {
            StartingAngle += CH_PI;  // Start on the back side of the sprocket instead of the front
        }

        while (StartingAngle < Arcs(0, 1)) {
            StartingAngle += CH_2PI;
        }

        shoe_points[0] = ChVector2d(std::cos(StartingAngle) * ScaledCircleRadius[0] + CirclePos[0].x(),
                                    std::sin(StartingAngle) * ScaledCircleRadius[0] + CirclePos[0].y());

        int shoelinktype = 0;
        int CurrentFeature = 0;

        // Build features array used for determining shoe points on the arcs and tangents
        ChMatrixDynamic<> Features(2 * ScaledCircleRadius.size(), 6);

        for (int i = 0; i < ScaledCircleRadius.size(); i++) {
            // Add Arc Wrap Feature (id = 0)
            Features(2 * i, 0) = 0;  // feature type id
            Features(2 * i, 1) = CirclePos[i].x();
            Features(2 * i, 2) = CirclePos[i].y();
            Features(2 * i, 3) = Arcs(i, 1);
            Features(2 * i, 4) = Arcs(i, 2);
            Features(2 * i, 5) = ScaledCircleRadius[i];

            // Add Tangent Wrap Feature (id = 1)
            Features(1 + (2 * i), 0) = 1;  // feature type id
            Features(1 + (2 * i), 1) = TangentPoints[i].first.x();
            Features(1 + (2 * i), 2) = TangentPoints[i].first.y();
            Features(1 + (2 * i), 3) = TangentPoints[i].second.x();
            Features(1 + (2 * i), 4) = TangentPoints[i].second.y();
            Features(1 + (2 * i), 5) = 0;  // Unused
        }

        // Calculate the remaining shoe points
        bool FoundPoint = false;
        bool InitalSprocketWrap = true;
        double ExtraLength = 0;
        double CurrentAngle = 0;
        int shoeidx;

        for (shoeidx = 1; shoeidx < shoe_points.size(); shoeidx++) {
            ChVector2d Point;
            ChVector2d StartingPoint = shoe_points[shoeidx - 1];

            // Make sure that all the features are only searched once to prevent and endless loop
            for (int c = 0; c < Features.rows(); c++) {
                if (Features(CurrentFeature, 0) == 0) {
                    CheckCircleCircle(FoundPoint, Point, Features, CurrentFeature, StartingPoint,
                                      connection_lengths[shoelinktype]);
                } else {
                    CheckCircleLine(FoundPoint, Point, Features, CurrentFeature, StartingPoint,
                                    connection_lengths[shoelinktype]);
                }

                if (FoundPoint) {
                    break;  // Still on the current Feature (arc or tangent)
                }

                InitalSprocketWrap = false;
                CurrentFeature += 1;
                if (CurrentFeature >= Features.rows()) {
                    CurrentFeature = 0;
                }
            }

            if (!FoundPoint) {
                std::cerr << "Belt Assembly ERROR: Something went wrong" << std::endl;
                assert(FoundPoint);
                return true;
            }

            shoe_points[shoeidx] = Point;

            shoelinktype += 1;
            if (shoelinktype >= connection_lengths.size()) {
                shoelinktype = 0;
            }

            ExtraLength = 0;
            // Check to see if I have wrapped past the initial point
            if ((!InitalSprocketWrap) && (CurrentFeature == 0)) {
                CurrentAngle = std::atan2(Point.y() - CirclePos[0].y(), Point.x() - CirclePos[0].x());
                while (CurrentAngle < Features(0, 3)) {
                    CurrentAngle += CH_2PI;
                }
                if (CurrentAngle > StartingAngle) {
                    ExtraLength = (CurrentAngle - StartingAngle) * Features(0, 5);
                    break;
                }
            }
        }

        for (int j = shoeidx + 1; j < shoe_points.size(); j++) {
            ExtraLength += connection_lengths[shoelinktype];

            shoelinktype += 1;
            if (shoelinktype >= connection_lengths.size()) {
                shoelinktype = 0;
            }
        }

        if (CurrentFeature > 0) {
            ExtraLength = -(shoe_points.back() - shoe_points.front()).Length();
        } else if ((ExtraLength == 0) && (CurrentAngle > StartingAngle)) {
            ExtraLength = (shoe_points.back() - shoe_points.front()).Length();
        } else if (ExtraLength == 0) {
            ExtraLength = -(shoe_points.back() - shoe_points.front()).Length();
        }

        if (std::abs(ExtraLength) < Tolerance) {
            std::cout << "Belt wrap algorithm converged after " << iter << " iterations - Extra length: " << ExtraLength
                      << " - Length tolerance: " << Tolerance << "\n";
            break;
        }

        if (ExtraLength > 0) {
            if (DeltaRadius > ScaleMin) {
                ScaleMin = DeltaRadius;
            }
        } else {
            if ((DeltaRadius < ScaleMax) || (ScaleMax == 0)) {
                ScaleMax = DeltaRadius;
            }
        }

        if (iter == 0) {
            DeltaRadius = DeltaRadius + ExtraLength * 10;
        } else {
            DeltaRadius = (ScaleMin + ScaleMax) / 2;
        }
    }

    ////utils::ChWriterCSV csv;
    ////for (int i = 0; i < shoe_points.size(); i++) {
    ////    csv << shoe_points[i].x() << shoe_points[i].y() << std::endl;
    ////}
    ////csv.WriteToFile("points.txt");

    //// TODO:  right now only counter-clock-wise
    return true;
}

void ChTrackAssemblyBand::FindCircleTangentPoints(ChVector2d Circle1Pos,
                                                  double Circle1Rad,
                                                  ChVector2d Circle2Pos,
                                                  double Circle2Rad,
                                                  ChVector2d& Tan1Pnt1,
                                                  ChVector2d& Tan1Pnt2,
                                                  ChVector2d& Tan2Pnt1,
                                                  ChVector2d& Tan2Pnt2) {
    // Based on https://en.wikipedia.org/wiki/Tangent_lines_to_circles with modifications

    double x1 = Circle1Pos.x();
    double x2 = Circle2Pos.x();
    double y1 = Circle1Pos.y();
    double y2 = Circle2Pos.y();
    double r1 = Circle1Rad;
    double r2 = Circle2Rad;

    double beta = std::asin((r2 - r1) / std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2)));
    double gamma = std::atan2((y2 - y1), (x2 - x1));

    Tan1Pnt1 = ChVector2d(x1 + r1 * std::cos(gamma + (CH_PI_2 + beta)), y1 + r1 * std::sin(gamma + (CH_PI_2 + beta)));
    Tan1Pnt2 = ChVector2d(x2 + r2 * std::cos(gamma + (CH_PI_2 + beta)), y2 + r2 * std::sin(gamma + (CH_PI_2 + beta)));

    // Pick the external tangent on the other side of the circle
    Tan2Pnt1 = ChVector2d(x1 + r1 * std::cos(gamma - (CH_PI_2 + beta)), y1 + r1 * std::sin(gamma - (CH_PI_2 + beta)));
    Tan2Pnt2 = ChVector2d(x2 + r2 * std::cos(gamma - (CH_PI_2 + beta)), y2 + r2 * std::sin(gamma - (CH_PI_2 + beta)));
}

void ChTrackAssemblyBand::CheckCircleCircle(bool& found,
                                            ChVector2d& Point,
                                            ChMatrixDynamic<>& Features,
                                            int FeatureIdx,
                                            ChVector2d& StartingPoint,
                                            double Radius) {
    // Code was based on http://mathworld.wolfram.com/Circle-CircleIntersection.html

    found = false;
    Point.x() = 0;
    Point.y() = 0;

    // Check for any intersection
    ChVector2d ArcCenter(Features(FeatureIdx, 1), Features(FeatureIdx, 2));
    double StartingArcAngle = Features(FeatureIdx, 3);
    double EndingArcAngle = Features(FeatureIdx, 4);

    double R = Features(FeatureIdx, 5);
    double r = Radius;
    ChVector2d temp = ArcCenter - StartingPoint;
    double d = temp.Length();

    double y2 = (4 * d * d * R * R - std::pow(d * d - r * r + R * R, 2)) / (4 * d * d);

    if (y2 < 0) {
        return;  // No intersection
    }

    // Now check the two possible points(could be a degenerate case of 1 point)

    double theta = std::acos(((R * R + d * d) - r * r) / (2 * R * d));

    ChVector2d Ctr = StartingPoint - ArcCenter;
    double Angle1 = std::atan2(Ctr.y(), Ctr.x()) - theta;
    double Angle2 = std::atan2(Ctr.y(), Ctr.x()) + theta - CH_2PI;

    while (Angle1 < StartingArcAngle) {
        Angle1 += CH_2PI;
    }
    while (Angle2 < StartingArcAngle) {
        Angle2 += CH_2PI;
    }

    double Angle0 = std::atan2(Ctr.y(), Ctr.x());
    double Angle1_check = Angle1 - 6 * CH_PI;
    double Angle2_check = Angle2 - 6 * CH_PI;

    while (Angle0 < 0) {
        Angle0 += CH_2PI;
    }
    while (Angle1_check < Angle0) {
        Angle1_check += CH_2PI;
    }
    while (Angle2_check < Angle0) {
        Angle2_check += CH_2PI;
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

void ChTrackAssemblyBand::CheckCircleLine(bool& found,
                                          ChVector2d& Point,
                                          ChMatrixDynamic<>& Features,
                                          int FeatureIdx,
                                          ChVector2d& StartingPoint,
                                          double Radius) {
    // Code was based on http://mathworld.wolfram.com/Circle-LineIntersection.html

    found = false;
    Point.x() = 0;
    Point.y() = 0;

    double x1 = Features(FeatureIdx, 1) - StartingPoint.x();
    double y1 = Features(FeatureIdx, 2) - StartingPoint.y();
    double x2 = Features(FeatureIdx, 3) - StartingPoint.x();
    double y2 = Features(FeatureIdx, 4) - StartingPoint.y();
    ChVector2d LinePnt1(x1, y1);
    ChVector2d LinePnt2(x2, y2);
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

    ChVector2d intPnt1(xint_1, yint_1);
    ChVector2d intPnt2(xint_2, yint_2);

    ChVector2d Line = LinePnt2 - LinePnt1;
    if (Line.Dot(intPnt1 - LinePnt1) > Line.Dot(intPnt2 - LinePnt1)) {
        auto temp = intPnt1;
        intPnt1 = intPnt2;
        intPnt2 = temp;
    }

    ChVector2d intPnt2mLinePnt1 = intPnt2 - LinePnt1;
    ChVector2d intPnt2mLinePnt2 = intPnt2 - LinePnt2;
    if ((intPnt2mLinePnt1.Length() <= dr) && (intPnt2mLinePnt2.Length() <= dr)) {
        found = true;
        Point = intPnt2 + StartingPoint;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
