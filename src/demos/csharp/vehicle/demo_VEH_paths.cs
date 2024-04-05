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
// Authors: Radu Serban, Josh Diyn
// =============================================================================
//
// Demo on using Bezier-curve paths for Chrono::Vehicle
//
// =============================================================================

using System;
using static ChronoGlobals;
using System.IO;
using static chrono_vehicle;

namespace ChronoDemo
{
    internal class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Copyright (c) 2017 projectchrono.org");
            Console.WriteLine("Chrono version: " + CHRONO_VERSION);

            // TODO: correct CHRONO_VERSION call
            // Console.WriteLine(chrono.GetLog() + "Copyright (c) 2017 projectchrono.org\nChrono version: " + CHRONO_VERSION + "\n\n");

            // Set the path to the Chrono data files and Chrono::Vehicle data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);
            chrono_vehicle.SetDataPath(CHRONO_VEHICLE_DATA_DIR);

            // Create (if needed) output directory
            string out_dir = chrono.GetChronoOutputPath() + "DEMO_PATHS";
            if (!Directory.Exists(out_dir))
            {
                try
                {
                    Directory.CreateDirectory(out_dir);
                }
                catch // error exception catch and quit
                {
                    Console.WriteLine("Error creating directory " + out_dir);
                    Environment.Exit(1); // Exit code
                }
            }

            // Plotting function
            void Plot(ChBezierCurve path, int n, string title, bool equal_axes = true)
            {
                ChVectorDynamicd x = new ChVectorDynamicd(n);
                ChVectorDynamicd y = new ChVectorDynamicd(n);
                double delta = 1.0 / n;
                for (int i = 0; i < n; i++)
                {
                    ChVector3d pos = path.Eval(delta * i);
                    x.SetItem(i, pos.x);
                    y.SetItem(i, pos.y);
                }
                String filename = out_dir + "/" + title + ".gpl";
                ChGnuPlot mplot = new ChGnuPlot(filename);
                mplot.SetGrid();
                mplot.SetLabelX("x");
                mplot.SetLabelY("y");
                if (equal_axes)
                    mplot.SetCommand("set size ratio -1");
                mplot.SetCommand("set offsets graph 0.01, 0.01, 0.01, 0.01");
                String title_cmd = "set title '";
                title_cmd += title;
                title_cmd += "'";
                mplot.SetCommand(title_cmd);
                mplot.Plot(x, y, "", " every 1 pt 1 ps 0.5 ");
                mplot.Plot(x, y, "", " with lines lt -1 lc rgb'#00AAEE' ");
            }

            // Create the paths and call Plot for each
            // Straight-line path
            var path1 = StraightLinePath(new ChVector3d(-10, -10, 1), new ChVector3d(10, 10, 1), 1);
            Plot(path1, 100, "Straight Line Path");

            // Circle path (left)
            var path2 = CirclePath(new ChVector3d(1, 2, 0), 3.0, 5.0, true, 1);
            Plot(path2, 100, "Left Circle Path");

            // Circle path (right)
            var path3 = CirclePath(new ChVector3d(1, 2, 0), 3.0, 5.0, false, 1);
            Plot(path3, 100, "Right Circle Path");

            // NATO double lane change path (left)
            var path4 = DoubleLaneChangePath(new ChVector3d(-100, 0, 0.1), 28.93, 3.6105, 25.0, 100.0, true);
            Plot(path4, 100, "Left NATO Double Lane Change", false);

            // NATO double lane change path (right)
            var path5 = DoubleLaneChangePath(new ChVector3d(-100, 0, 0.1), 28.93, 3.6105, 25.0, 100.0, false);
            Plot(path5, 100, "Right NATO Double Lane Change", false);

            // ISO double lane change path (left)
            var path6 = DoubleLaneChangePath(new ChVector3d(-100, 0, 0.1), 13.5, 4.0, 11.0, 100.0, true);
            Plot(path6, 100, "Right ISO Double Lane Change", false);
        }
    }
}
