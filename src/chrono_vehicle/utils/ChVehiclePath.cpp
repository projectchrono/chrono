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
// Utility functions for creating paths as Bezier curves.
//
// =============================================================================

#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/ChWorldFrame.h"

namespace chrono {
namespace vehicle {

std::shared_ptr<ChBezierCurve> StraightLinePath(const ChVector3d& start,
                                                const ChVector3d end,
                                                unsigned int num_intermediate) {
    ChVector3d dir = end - start;
    double len = dir.Length();
    dir = dir / len;
    len /= (num_intermediate + 1);
    double offset = len / 10;

    std::vector<ChVector3d> points;
    std::vector<ChVector3d> inCV;
    std::vector<ChVector3d> outCV;

    points.push_back(start);
    inCV.push_back(start);
    outCV.push_back(start + dir * offset);

    for (unsigned int i = 1; i <= num_intermediate; i++) {
        points.push_back(start + dir * (i * len));
        inCV.push_back(start + dir * (i * len - offset));
        outCV.push_back(start + dir * (i * len + offset));
    }

    points.push_back(end);
    inCV.push_back(end - dir * offset);
    outCV.push_back(end);

    return chrono_types::make_shared<ChBezierCurve>(points, inCV, outCV);
}

std::shared_ptr<ChBezierCurve> CirclePath(const ChVector3d& start,
                                          double radius,
                                          double run,
                                          bool left_turn,
                                          int num_turns) {
    auto& W = ChWorldFrame::Rotation().transpose();

    double left = left_turn ? +1.0 : -1.0;
    double factor = radius * (4.0 / 3.0) * std::tan(CH_PI / 8);

    ChVector3d P0 = start;
    ChVector3d P0_in = P0;
    ChVector3d P0_out = P0 + W * ChVector3d(factor, 0, 0);

    ChVector3d P1 = start + W * ChVector3d(run, 0, 0);
    ChVector3d P1_in = P1 + W * ChVector3d(-factor, 0, 0);
    ChVector3d P1_out = P1 + W * ChVector3d(factor, 0, 0);

    ChVector3d P2 = start + W * ChVector3d(run + radius, left * radius, 0);
    ChVector3d P2_in = P2 + W * ChVector3d(0, -left * factor, 0);
    ChVector3d P2_out = P2 + W * ChVector3d(0, left * factor, 0);

    ChVector3d P3 = start + W * ChVector3d(run, 2 * left * radius, 0);
    ChVector3d P3_in = P3 + W * ChVector3d(factor, 0, 0);
    ChVector3d P3_out = P3 + W * ChVector3d(-factor, 0, 0);

    ChVector3d P4 = start + W * ChVector3d(run - radius, left * radius, 0);
    ChVector3d P4_in = P4 + W * ChVector3d(0, left * factor, 0);
    ChVector3d P4_out = P4 + W * ChVector3d(0, -left * factor, 0);

    // Load Bezier curve points, offsetting by 'center'
    std::vector<ChVector3d> points;
    std::vector<ChVector3d> inCV;
    std::vector<ChVector3d> outCV;

    points.push_back(P0);
    inCV.push_back(P0_in);
    outCV.push_back(P0_out);

    for (int i = 0; i < num_turns; i++) {
        points.push_back(P1);
        inCV.push_back(P1_in);
        outCV.push_back(P1_out);

        points.push_back(P2);
        inCV.push_back(P2_in);
        outCV.push_back(P2_out);

        points.push_back(P3);
        inCV.push_back(P3_in);
        outCV.push_back(P3_out);

        points.push_back(P4);
        inCV.push_back(P4_in);
        outCV.push_back(P4_out);
    }

    points.push_back(P1);
    inCV.push_back(P1_in);
    outCV.push_back(P1_out);

    return chrono_types::make_shared<ChBezierCurve>(points, inCV, outCV);
}

std::shared_ptr<ChBezierCurve> DoubleLaneChangePath(const ChVector3d& start,
                                                    double ramp,
                                                    double width,
                                                    double length,
                                                    double run,
                                                    bool left_turn) {
    auto& W = ChWorldFrame::Rotation().transpose();

    ChVector3d offset = W * ChVector3d(length * CH_1_3, 0, 0);
    double left = left_turn ? +1.0 : -1.0;

    std::vector<ChVector3d> points;
    std::vector<ChVector3d> inCV;
    std::vector<ChVector3d> outCV;

    ChVector3d P;

    P = start;
    points.push_back(P);
    inCV.push_back(P - offset);
    outCV.push_back(P + offset);

    P = start + W * ChVector3d(run, 0, 0);
    points.push_back(P);
    inCV.push_back(P - offset);
    outCV.push_back(P + offset);

    P = start + W * ChVector3d(run + ramp, left * width, 0);
    points.push_back(P);
    inCV.push_back(P - offset);
    outCV.push_back(P + offset);

    P = start + W * ChVector3d(run + ramp + length, left * width, 0);
    points.push_back(P);
    inCV.push_back(P - offset);
    outCV.push_back(P + offset);

    P = start + W * ChVector3d(run + 2 * ramp + length, 0, 0);
    points.push_back(P);
    inCV.push_back(P - offset);
    outCV.push_back(P + offset);

    P = start + W * ChVector3d(2 * run + 2 * ramp + length, 0, 0);
    points.push_back(P);
    inCV.push_back(P - offset);
    outCV.push_back(P + offset);

    return chrono_types::make_shared<ChBezierCurve>(points, inCV, outCV);
}

}  // end namespace vehicle
}  // end namespace chrono
