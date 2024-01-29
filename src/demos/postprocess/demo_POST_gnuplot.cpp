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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo on using GNUplot for plotting graphs
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChLog.h"
#include "chrono/core/ChGlobal.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/motion_functions/ChFunction_Sine.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace postprocess;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    GetLog() << "CHRONO demo that launches GNUplot for plotting graphs: \n\n";

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "DEMO_GNUPLOT";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    {
        //
        // EXAMPLE 1
        //

        // The most low-level way of using the ChGnuPlot class:
        // use the the SetCommand() or alternatively << , to build the script file.

        ChGnuPlot mplot(out_dir + "/tmp_gnuplot_1.gpl");
        mplot << "set contour";
        mplot << "set title 'Demo of specifying discrete contour levels'";
        mplot << "splot x*y";

        // When the mplot object goes out of scope and is deleted at the end of this example, the
        // script is saved on disk and GNUplot is launched.
    }

    {
        //
        // EXAMPLE 2
        //

        // Use the Open... functions to define the output terminal.
        // Open two plots in two windows and also save the second to EPS and PNG files.
        // Learn shortcut calls:
        //  SetGrid() to make a grid
        //  SetLabelX() SetLabelY() to setup labels

        ChGnuPlot mplot(out_dir + "/tmp_gnuplot_2.gpl");
        mplot.SetGrid();

        mplot.OutputWindow(0);
        mplot.SetLabelX("x");
        mplot.SetLabelY("y");
        mplot << "plot [-30:20] besj0(x)*0.12e1 with impulses, (x**besj0(x))-2.5 with points";

        mplot.OutputWindow(1);
        mplot.SetLabelX("v");
        mplot.SetLabelY("w");
        mplot << "plot [-10:10] real(sin(x)**besj0(x))";

        mplot.OutputEPS(out_dir + "/test_eps.eps");
        mplot.OutputPNG(out_dir + "/test_eps.png", 800, 600);
        mplot.Replot();  // repeat last plot
    }

    {
        //
        // EXAMPLE 3
        //

        // Learn the Plot() shortcut function, for easy plotting from a .dat file (ascii file
        // with column ordered data).

        // Step 1.
        // create a .dat file with three columns of demo data:
        std::string datafilename = out_dir + "/test_gnuplot_data.dat";
        ChStreamOutAsciiFile datafile(datafilename);
        for (double x = 0; x < 10; x += 0.1)
            datafile << x << ", " << sin(x) << ", " << cos(x) << "\n";

        // Step 2.
        // Create the plot.
        // NOTE. The plot shortcuts. In this case you pass the .dat filename, the columns IDs, title and custom settings
        // NOTE. You can have multiple Plot() calls for a single Output,
        // they will be overlapped as when you use commas in gnuplot:  "plot ... , ... , ..."
        ChGnuPlot mplot(out_dir + "/tmp_gnuplot_3.gpl");
        mplot.SetGrid();
        mplot.SetLabelX("x");
        mplot.SetLabelY("y");
        mplot.Plot(datafilename, 1, 2, "sine", " with lines lt -1 lw 2");
        mplot.Plot(datafilename, 1, 3, "cosine", " with lines lt 2 lw 2");
    }

    {
        //
        // EXAMPLE 4
        //

        // The Plot() shortcut function can be used directly on embedded data, without
        // needing to save a .dat file. One can use, for instance:
        //  - a pair of x,y vectors (use ChVectorDynamic column matrices)
        //  - ChFunction y(x) objects
        //  - columns from a ChMatrix
        // The values will be saved as embedded in the .gpl file.
        // Note, the Replot command does not work with embedded data.

        // create demo data in a pair of x,y vectors
        ChVectorDynamic<> mx(100);
        ChVectorDynamic<> my(100);
        for (int i = 0; i < 100; ++i) {
            double x = ((double)i / 100.0) * 12;
            double y = sin(x) * exp(-x * 0.2);
            mx(i) = x;
            my(i) = y;
        }
        // ..or create demo data in a ChFunction_Recorder
        ChFunction_Recorder mfun;
        for (int i = 0; i < 100; ++i) {
            double x = ((double)i / 100.0) * 12;
            double y = cos(x) * exp(-x * 0.4);
            mfun.AddPoint(x, y);
        }
        // ..or create demo data in two columns of a ChMatrix
        ChMatrixDynamic<> matr(100, 10);
        for (int i = 0; i < 100; ++i) {
            double x = ((double)i / 100.0) * 12;
            double y = cos(x) * exp(-x * 0.4);
            matr(i, 2) = x;
            matr(i, 6) = y * 0.4;
        }

        // Create the plot.
        // NOTE. The plot shortcuts.
        ChGnuPlot mplot(out_dir + "/tmp_gnuplot_4.gpl");
        mplot.SetGrid();
        mplot.Plot(mx, my, "from x,y ChVectorDynamic", " every 5 pt 1 ps 0.5");
        mplot.Plot(mfun, "from ChFunction_Recorder", " with lines lt -1 lc rgb'#00AAEE' ");
        mplot.Plot(matr, 2, 6, "from ChMatrix", " with lines lt 5");
    }

    GetLog() << "\n  CHRONO execution terminated.";

    return 0;
}
