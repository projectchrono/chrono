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
// Demo on using Bezier-curve paths for Chrono::Vehicle
//
// =============================================================================

#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_postprocess/ChGnuPlot.h"
#include "chrono/core/ChDataPath.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace vehicle;
using namespace postprocess;

void plot(const std::string& out_dir, std::shared_ptr<ChBezierCurve> path, int n, const char* title, bool equal_axes = true) {
    ChVectorDynamic<> x(n);
    ChVectorDynamic<> y(n);
    double delta = 1.0 / n;
    for (int i = 0; i < n; i++) {
        ChVector3d pos = path->Eval(delta * i);
        x(i) = pos.x();
        y(i) = pos.y();
    }
    std::string filename = out_dir + "/" + title + ".gpl";
    ChGnuPlot mplot(filename);
    mplot.SetGrid();
    mplot.SetLabelX("x");
    mplot.SetLabelY("y");
    if (equal_axes)
        mplot.SetCommand("set size ratio -1");
    mplot.SetCommand("set offsets graph 0.01, 0.01, 0.01, 0.01");
    std::string title_cmd("set title '");
    title_cmd += title;
    title_cmd += "'";
    mplot.SetCommand(title_cmd);
    mplot.Plot(x, y, "", " every 1 pt 1 ps 0.5 ");
    mplot.Plot(x, y, "", " with lines lt -1 lc rgb'#00AAEE' ");
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "DEMO_PATHS";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Straight-line path
    auto path1 = StraightLinePath(ChVector3d(-10, -10, 1), ChVector3d(10, 10, 1), 1);
    plot(out_dir, path1, 100, "straight line path");

    // Circle path (left)
    auto path2 = CirclePath(ChVector3d(1, 2, 0), 3.0, 5.0, true, 1);
    plot(out_dir, path2, 100, "left circle path");

    // Circle path (right)
    auto path3 = CirclePath(ChVector3d(1, 2, 0), 3.0, 5.0, false, 1);
    plot(out_dir, path3, 100, "right circle path");

    // NATO double lane change path (left)
    auto path4 = DoubleLaneChangePath(ChVector3d(-100, 0, 0.1), 28.93, 3.6105, 25.0, 100.0, true);
    plot(out_dir, path4, 100, "left NATO double lane change", false);

    // NATO double lane change path (right)
    auto path5 = DoubleLaneChangePath(ChVector3d(-100, 0, 0.1), 28.93, 3.6105, 25.0, 100.0, false);
    plot(out_dir, path5, 100, "right NATO double lane change", false);

    // ISO double lane change path (left)
    auto path6 = DoubleLaneChangePath(ChVector3d(-100, 0, 0.1), 13.5, 4.0, 11.0, 100.0, true);
    plot(out_dir, path6, 100, "right ISO double lane change", false);

    return 0;
}
