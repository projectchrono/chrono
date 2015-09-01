//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about
//
//     - how to use the ChFunction objects to
//       easily define y=f(t) functions.
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono/ChVersion.h"
#include "chrono/physics/ChFunction.h"

// Use the namespace of Chrono

using namespace chrono;

int main(int argc, char* argv[]) {
    //
    // EXAMPLE 1: create a ramp ChFunction, set properties, evaluate it.
    //

    GetLog() << "Chrono revision: " << CHRONOENGINE_VCS_REVISION << "\n";
    GetLog() << "Chrono version: " << CHRONOENGINE_VERSION << "\n";

    GetLog() << "==== Test 1...\n\n";

    ChFunction_Ramp f_ramp;

    f_ramp.Set_ang(0.1);  // set angular coefficient;
    f_ramp.Set_y0(0.4);   // set y value for x=0;

    // Evaluate y=f(x) function at a given x value, using Get_y() :
    double y = f_ramp.Get_y(10);
    // Evaluate derivative df(x)/dx at a given x value, using Get_y_dx() :
    double ydx = f_ramp.Get_y_dx(10);

    GetLog() << "   ChFunction_Ramp at x=0: y=" << y << "  dy/dx=" << ydx << "\n\n";

    //
    // EXAMPLE 2: save values of a sine ChFunction  into a file
    //

    GetLog() << "==== Test 2...\n\n";

    ChFunction_Sine f_sine;

    f_sine.Set_amp(2);     // set amplitude;
    f_sine.Set_freq(1.5);  // set frequency;

    ChStreamOutAsciiFile file_f_sine("f_sine_out.dat");

    // Evaluate y=f(x) function along 100 x points, and its derivatives,
    // and save to file (later it can be loaded, for example, in Matlab using the 'load()' command)
    for (int i = 0; i < 100; i++) {
        double x = (double)i / 50.0;
        double y = f_sine.Get_y(x);
        double ydx = f_sine.Get_y_dx(x);
        double ydxdx = f_sine.Get_y_dxdx(x);
        file_f_sine << x << " " << y << " " << ydx << " " << ydxdx << "\n";
    }

    //
    // EXAMPLE 3: use a custom function (see class at the beginning of this file)
    //

    GetLog() << "==== Test 3...\n\n";

    // The following class will be used as an example of how
    // how you can create custom functions based on the ChFunction interface.
    // There is at least one mandatory member function to implement:  Get_y().
    // [Note that the base class implements a default computation of derivatives
    // Get_ydx() and Get_ydxdx() by using a numerical differentiation, however
    // if you know the analytical expression of derivatives, you can override
    // the base Get_ydx() and Get_ydxdx() too, for higher precision.]

    class ChFunction_MyTest : public ChFunction {
      public:
        ChFunction* new_Duplicate() { return new ChFunction_MyTest; }

        double Get_y(double x) { return cos(x); }  // just for test: simple cosine
    };

    ChFunction_MyTest f_test;

    ChStreamOutAsciiFile file_f_test("f_test_out.dat");

    // Evaluate y=f(x) function along 100 x points, and its derivatives,
    // and save to file (later it can be loaded, for example, in Matlab using the 'load()' command)
    for (int i = 0; i < 100; i++) {
        double x = (double)i / 50.0;
        double y = f_test.Get_y(x);
        double ydx = f_test.Get_y_dx(x);
        double ydxdx = f_test.Get_y_dxdx(x);
        file_f_test << x << " " << y << " " << ydx << " " << ydxdx << "\n";
    }

    //
    // EXAMPLE 4: mount some functions in a sequence using ChFunction_Sequence
    //

    GetLog() << "==== Test 4...\n\n";

    ChFunction_Sequence f_sequence;

    ChSharedPtr<ChFunction_ConstAcc> f_constacc1(new ChFunction_ConstAcc);
    f_constacc1->Set_end(0.5);  // length of ramp
    f_constacc1->Set_h(0.3);    // height of ramp
    f_sequence.InsertFunct(f_constacc1, 0.5, 1, false, false, false, 0);

    ChSharedPtr<ChFunction_Const> f_const(new ChFunction_Const);
    f_sequence.InsertFunct(f_const, 0.4, 1, true, false, false, -1);

    ChSharedPtr<ChFunction_ConstAcc> f_constacc2(new ChFunction_ConstAcc);
    f_constacc2->Set_end(0.6);  // length of ramp
    f_constacc2->Set_av(0.3);   // acceleration ends after 30% length
    f_constacc2->Set_aw(0.7);   // deceleration starts after 70% length
    f_constacc2->Set_h(-0.2);   // height of ramp
    f_sequence.InsertFunct(f_constacc2, 0.6, 1, true, false, false, -1);

    f_sequence.Setup();

    ChStreamOutAsciiFile file_f_sequence("f_sequence_out.dat");

    // Evaluate y=f(x) function along 100 x points, and its derivatives,
    // and save to file (later it can be loaded, for example, in Matlab using the 'load()' command)
    for (int i = 0; i < 100; i++) {
        double x = (double)i / 50.0;
        double y = f_sequence.Get_y(x);
        double ydx = f_sequence.Get_y_dx(x);
        double ydxdx = f_sequence.Get_y_dxdx(x);
        file_f_sequence << x << " " << y << " " << ydx << " " << ydxdx << "\n";
    }

    return 0;
}
