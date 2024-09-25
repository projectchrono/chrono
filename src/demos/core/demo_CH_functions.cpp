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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Demo code for using the ChFunction objects for specifying functions y=f(t)
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChGlobal.h"
#include "chrono/functions/ChFunction.h"

#include "chrono_thirdparty/filesystem/path.h"

// Use the namespace of Chrono

using namespace chrono;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "DEMO_FUNCTIONS";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    //
    // EXAMPLE 1: create a ramp ChFunction, set properties, evaluate it.
    //

    std::cout << "==== Test 1...\n\n";

    ChFunctionRamp f_ramp;

    f_ramp.SetAngularCoeff(0.1);  // set angular coefficient;
    f_ramp.SetStartVal(0.4);      // set y value for x=0;

    // Evaluate y=f(x) function at a given x value, using GetVal() :
    double y_ramp = f_ramp.GetVal(10);
    // Evaluate derivative df(x)/dx at a given x value, using GetDer() :
    double ydx_ramp = f_ramp.GetDer(10);

    std::cout << "   ChFunctionRamp at x=0: y=" << y_ramp << "  dy/dx=" << ydx_ramp << "\n\n";

    //
    // EXAMPLE 2: save values of a sine ChFunction  into a file
    //

    std::cout << "==== Test 2...\n\n";

    ChFunctionSine f_sine;

    f_sine.SetAmplitude(2);    // set amplitude;
    f_sine.SetFrequency(1.5);  // set frequency;

    std::ofstream file_f_sine(out_dir + "/f_sine_out.dat");

    // Evaluate y=f(x) function along 100 x points, and its derivatives,
    // and save to file (later it can be loaded, for example, in Matlab using the 'load()' command)
    for (int i = 0; i < 100; i++) {
        double x = (double)i / 50.0;
        double y = f_sine.GetVal(x);
        double ydx = f_sine.GetDer(x);
        double ydxdx = f_sine.GetDer2(x);
        file_f_sine << x << " " << y << " " << ydx << " " << ydxdx << "\n";
    }

    //
    // EXAMPLE 3: use a custom function (see class at the beginning of this file)
    //

    std::cout << "==== Test 3...\n\n";

    // The following class will be used as an example of how
    // how you can create custom functions based on the ChFunction interface.

    class ChFunctionMyTest : public ChFunction {
      public:
        virtual ChFunctionMyTest* Clone() const override { return new ChFunctionMyTest(); }

        virtual double GetVal(double x) const override { return std::cos(x); }  // just for test: simple cosine
    };

    ChFunctionMyTest f_test;

    std::ofstream file_f_test(out_dir + "/f_test_out.dat");

    // Evaluate y=f(x) function along 100 x points, and its derivatives,
    // and save to file (later it can be loaded, for example, in Matlab using the 'load()' command)
    for (int i = 0; i < 100; i++) {
        double x = (double)i / 50.0;
        double y = f_test.GetVal(x);
        double ydx = f_test.GetDer(x);
        double ydxdx = f_test.GetDer2(x);
        file_f_test << x << " " << y << " " << ydx << " " << ydxdx << "\n";
    }

    //
    // EXAMPLE 4: mount some functions in a sequence using ChFunctionSequence
    //

    std::cout << "==== Test 4...\n\n";

    ChFunctionSequence f_sequence;

    auto f_constacc1 = chrono_types::make_shared<ChFunctionConstAcc>();
    f_constacc1->SetDuration(0.5);      // length of ramp
    f_constacc1->SetDisplacement(0.3);  // height of ramp
    f_sequence.InsertFunct(f_constacc1, 0.5, 1, false, false, false, 0);

    auto f_const = chrono_types::make_shared<ChFunctionConst>();
    f_sequence.InsertFunct(f_const, 0.4, 1, true, false, false, -1);

    auto f_constacc2 = chrono_types::make_shared<ChFunctionConstAcc>();
    f_constacc2->SetDuration(0.6);                 // length of ramp
    f_constacc2->SetFirstAccelerationEnd(0.3);     // acceleration ends after 30% length
    f_constacc2->SetSecondAccelerationStart(0.7);  // deceleration starts after 70% length
    f_constacc2->SetDisplacement(-0.2);            // height of ramp
    f_sequence.InsertFunct(f_constacc2, 0.6, 1, true, false, false, -1);

    f_sequence.Setup();

    std::ofstream file_f_sequence(out_dir + "/f_sequence_out.dat");

    // Evaluate y=f(x) function along 100 x points, and its derivatives,
    // and save to file (later it can be loaded, for example, in Matlab using the 'load()' command)
    for (int i = 0; i < 100; i++) {
        double x = (double)i / 50.0;
        double y = f_sequence.GetVal(x);
        double ydx = f_sequence.GetDer(x);
        double ydxdx = f_sequence.GetDer2(x);
        file_f_sequence << x << " " << y << " " << ydx << " " << ydxdx << "\n";
    }

    //
    // EXAMPLE 5: a repeating sequence
    //

    std::cout << "==== Test 5...\n\n";

    auto f_part1 = chrono_types::make_shared<ChFunctionRamp>();
    f_part1->SetAngularCoeff(.50);
    auto f_part2 = chrono_types::make_shared<ChFunctionConst>();
    f_part2->SetConstant(1.0);
    auto f_part3 = chrono_types::make_shared<ChFunctionRamp>();
    f_part3->SetAngularCoeff(-.50);

    auto f_seq = chrono_types::make_shared<ChFunctionSequence>();
    f_seq->InsertFunct(f_part1, 1.0, 1, true);
    f_seq->InsertFunct(f_part2, 1.0, 1., true);
    f_seq->InsertFunct(f_part3, 1.0, 1., true);

    auto f_rep_seq = chrono_types::make_shared<ChFunctionRepeat>(f_seq);
    f_rep_seq->SetSliceWidth(3.0);
    f_rep_seq->SetSliceStart(0.0);
    f_rep_seq->SetSliceShift(3.0);

    std::ofstream file_f_repeat(out_dir + "/f_repeat_out.dat");
    for (int i = 0; i < 1000; i++) {
        double x = (double)i / 50.0;
        double y = f_rep_seq->GetVal(x);
        double ydx = f_rep_seq->GetDer(x);
        double ydxdx = f_rep_seq->GetDer2(x);
        file_f_repeat << x << " " << y << " " << ydx << " " << ydxdx << "\n";
    }

    return 0;
}
