// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================

#include "chrono/functions/ChFunctionInterp.h"

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"
#include "chrono_fsi/sph/physics/SphGeneral.cuh"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
using namespace chrono::postprocess;
#endif

#include "chrono_thirdparty/filesystem/path.h"

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::string out_dir = GetChronoOutputPath() + "FSI_Kernels/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Plot kernel functions and their gradients
    const int N = 501;
    Real h = 1;
    Real invh = 1 / h;
    Real delta = (3 * h) / (N - 1);

    ChFunctionInterp w_quadratic_func;
    ChFunctionInterp w_quadratic_grad;
    ChFunctionInterp w_cubicspline_func;
    ChFunctionInterp w_cubicspline_grad;
    ChFunctionInterp w_quinticspline_func;
    ChFunctionInterp w_quinticspline_grad;
    ChFunctionInterp w_wendland_func;
    ChFunctionInterp w_wendland_grad;

    for (int i = 0; i < N; i++) {
        Real x = i * delta;
        {
            // Quadratic kernel
            Real w = W3h_Quadratic(x, invh);
            Real3 gw = GradW3h_Quadratic(mR3(x, 0, 0), invh);
            w_quadratic_func.AddPoint(x, w);
            w_quadratic_grad.AddPoint(x, gw.x);
        }
        {
            // Cubic spline kernel
            Real w = W3h_CubicSpline(x, invh);
            Real3 gw = GradW3h_CubicSpline(mR3(x, 0, 0), invh);
            w_cubicspline_func.AddPoint(x, w);
            w_cubicspline_grad.AddPoint(x, gw.x);
        }
        {
            // Quintic spline kernel
            Real w = W3h_QuinticSpline(x, invh);
            Real3 gw = GradW3h_QuinticSpline(mR3(x, 0, 0), invh);
            w_quinticspline_func.AddPoint(x, w);
            w_quinticspline_grad.AddPoint(x, gw.x);
        }
        {
            // Wendland kernel
            Real w = W3h_Wendland(x, invh);
            Real3 gw = GradW3h_Wendland(mR3(x, 0, 0), invh);
            w_wendland_func.AddPoint(x, w);
            w_wendland_grad.AddPoint(x, gw.x);
        }
    }

#ifdef CHRONO_POSTPROCESS
    {
        postprocess::ChGnuPlot gplot_speed(out_dir + "/kernel_func.gpl");
        gplot_speed.SetGrid();
        gplot_speed.SetLabelX("x");
        gplot_speed.SetLabelY("w");
        gplot_speed.Plot(w_quadratic_func, "quadratic", " with lines lt -1 lc rgb'#00AAEE' ");
        gplot_speed.Plot(w_cubicspline_func, "cubic spline", " with lines lt -1 lc rgb'#AA00EE' ");
        gplot_speed.Plot(w_quinticspline_func, "quintic spline", " with lines lt -1 lc rgb'#EE0000' ");
        gplot_speed.Plot(w_wendland_func, "Wendland", " with lines lt -1 lc rgb'#00EE00' ");
    }
    {
        postprocess::ChGnuPlot gplot_speed(out_dir + "/kernel_grad.gpl");
        gplot_speed.SetGrid();
        gplot_speed.SetLabelX("x");
        gplot_speed.SetLabelY("grad(w)");
        gplot_speed.Plot(w_quadratic_grad, "quadratic", " with lines lt -1 lc rgb'#00AAEE' ");
        gplot_speed.Plot(w_cubicspline_grad, "cubic spline", " with lines lt -1 lc rgb'#AA00EE' ");
        gplot_speed.Plot(w_quinticspline_grad, "quintic spline", " with lines lt -1 lc rgb'#EE0000' ");
        gplot_speed.Plot(w_wendland_grad, "Wendland", " with lines lt -1 lc rgb'#00EE00' ");
    }
#endif

    // Integrate kernel functions in first octant
    double I_quadratic = 0;
    double I_cubicspline = 0;
    double I_quinticspline = 0;
    double I_wendland = 0;
    for (int ix = 0; ix < N; ix++) {
        Real x = ix * delta;
        for (int iy = 0; iy < N; iy++) {
            Real y = iy * delta;
            for (int iz = 0; iz < N; iz++) {
                Real z = iz * delta;
                Real3 d = mR3(x, y, z);
                Real l = length(d);
                I_quadratic += W3h_Quadratic(l, invh);
                I_cubicspline += W3h_CubicSpline(l, invh);
                I_quinticspline += W3h_QuinticSpline(l, invh);
                I_wendland += W3h_Wendland(l, invh);
            }
        }
    }

    // Check that kernel functions integrate to 1
    double delta3 = std::pow(delta, 3);
    std::cout << "Kernel function integrals: " << std::endl;
    std::cout << "  Quadratic:      " << 8 * I_quadratic * delta3 << std::endl;
    std::cout << "  Cubic spline:   " << 8 * I_cubicspline * delta3 << std::endl;
    std::cout << "  Quintic spline: " << 8 * I_quinticspline * delta3 << std::endl;
    std::cout << "  Wendland:       " << 8 * I_wendland * delta3 << std::endl;

    return 0;
}
