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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Demo code for using the ChFunction objects for specifying functions y=f(t)
//
// =============================================================================

#include "chrono/ChVersion.h"
#include "chrono/motion_functions/ChFunction.h"

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
        virtual ChFunction_MyTest* Clone() const override { return new ChFunction_MyTest(); }

        virtual double Get_y(double x) const override { return cos(x); }  // just for test: simple cosine
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

    auto f_constacc1 = std::make_shared<ChFunction_ConstAcc>();
    f_constacc1->Set_end(0.5);  // length of ramp
    f_constacc1->Set_h(0.3);    // height of ramp
    f_sequence.InsertFunct(f_constacc1, 0.5, 1, false, false, false, 0);

    auto f_const = std::make_shared<ChFunction_Const>();
    f_sequence.InsertFunct(f_const, 0.4, 1, true, false, false, -1);

    auto f_constacc2 = std::make_shared<ChFunction_ConstAcc>();
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

    //
    // EXAMPLE 5: a repeating sequence
    //

    GetLog() << "==== Test 5...\n\n";

    auto f_part1 = std::make_shared<ChFunction_Ramp>();
    f_part1->Set_ang(.50);
    auto f_part2 = std::make_shared<ChFunction_Const>();
    f_part2->Set_yconst(1.0);
    auto f_part3 = std::make_shared<ChFunction_Ramp>();
    f_part3->Set_ang(-.50);

    auto f_seq = std::make_shared<ChFunction_Sequence>();
    f_seq->InsertFunct(f_part1, 1.0, 1, true);
    f_seq->InsertFunct(f_part2, 1.0, 1., true);
    f_seq->InsertFunct(f_part3, 1.0, 1., true);

    auto f_rep_seq = std::make_shared<ChFunction_Repeat>();
    f_rep_seq->Set_fa(f_seq);
    f_rep_seq->Set_window_length(3.0);
    f_rep_seq->Set_window_start(0.0);
    f_rep_seq->Set_window_phase(3.0);

    ChStreamOutAsciiFile file_f_repeat("f_repeat_out.dat");
    for (int i = 0; i < 1000; i++) {
        double x = (double)i / 50.0;
        double y = f_rep_seq->Get_y(x);
        double ydx = f_rep_seq->Get_y_dx(x);
        double ydxdx = f_rep_seq->Get_y_dxdx(x);
        file_f_repeat << x << " " << y << " " << ydx << " " << ydxdx << "\n";
    }

    return 0;
}
