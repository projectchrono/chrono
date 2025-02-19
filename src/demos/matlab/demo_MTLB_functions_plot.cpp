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
// Demo code for using the ChFunction objects for specifying functions y=f(t)
// and calling Matlab from Chrono (in particular, using Matlab to plot data)
// =============================================================================

#include "chrono/functions/ChFunction.h"
#include "chrono_matlab/ChMatlabEngine.h"

// Use the namespace of Chrono

using namespace chrono;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Better put the Matlab stuff inside a try{}, since it may throw exception if
    // the engine is not started (because Matlab not properly installed)
    try {
        std::cout << "PERFORM TESTS OF MATLAB<->CHRONO INTERACTION\n\n";
        std::cout << "(please wait few seconds: Matlab engine must be loaded)\n\n";

        // This is the object that you can use to access the Matlab engine.
        // As soon as created, it loads the Matlab engine (if troubles happen, it
        // throws exception).
        ChMatlabEngine matlab_engine;

        //
        // EXAMPLE 1: create a ramp ChFunction, set properties, evaluate it.
        //

        std::cout << "==== Test 1...\n\n";

        ChFunctionRamp f_ramp;

        f_ramp.SetAngularCoeff(0.1);  // set angular coefficient;
        f_ramp.SetStartVal(0.4);      // set y value for x=0;

        // Evaluate y=f(x) function at a given x value, using GetVal() :
        double y = f_ramp.GetVal(10);
        // Evaluate derivative df(x)/dx at a given x value, using GetDer() :
        double ydx = f_ramp.GetDer(10);

        std::cout << "   ChFunctionRamp at x=0: y=" << y << "  dy/dx=" << ydx << "\n\n";

        //
        // EXAMPLE 2: plot a sine ChFunction
        //

        std::cout << "==== Test 2...\n\n";

        ChFunctionSine f_sine;

        f_sine.SetAmplitude(2);    // set amplitude;
        f_sine.SetFrequency(0.9);  // set frequency;

        // Evaluate y=f(x) function along 100 x points:
        ChMatrixDynamic<> x_array(100, 1);
        ChMatrixDynamic<> y_array(100, 1);
        ChMatrixDynamic<> ydx_array(100, 1);
        ChMatrixDynamic<> ydxdx_array(100, 1);
        for (int i = 0; i < 100; i++) {
            double x = (double)i / 50.0;
            x_array(i) = x;
            y_array(i) = f_sine.GetVal(x);
            ydx_array(i) = f_sine.GetDer(x);
            ydxdx_array(i) = f_sine.GetDer2(x);
        }

        // Send resulting vectors of values to Matlab
        matlab_engine.PutVariable(x_array, "x_array");
        matlab_engine.PutVariable(y_array, "y_array");
        matlab_engine.PutVariable(ydx_array, "ydx_array");
        matlab_engine.PutVariable(ydxdx_array, "ydxdx_array");

        // Plot with Matlab 'plot' command
        matlab_engine.Eval("figure;  plot(x_array,y_array,'k');");
        matlab_engine.Eval("hold on; plot(x_array,ydx_array,'g');");
        matlab_engine.Eval("grid on; plot(x_array,ydxdx_array,'r');");
        matlab_engine.Eval("legend ('y','dy/dx','ddy/dxdx');");

        std::cout << "Press a key to finish... \n";
        getchar();  // pause until key..
    } catch (std::exception mex) {
        std::cerr << mex.what() << std::endl;  // Print error on console, if Matlab did not start.
    }

    return 0;
}
