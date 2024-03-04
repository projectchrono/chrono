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
// Demo code illustrating the use of the Chrono Python parser to execute some
// Python program or formulas
//
// =============================================================================

#include <iostream>
#include <sstream>

#include "chrono_parsers/ChParserPython.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyAuxRef.h"

using namespace chrono;
using namespace chrono::parsers;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    GetLog() << " Test the execution of Python statements, formulas, programs.\n No graphical user interface.\n\n";

    // Use a ChPythonEngine object.
    // Note: currently no multiple ChPythonEngine objects can be used simultaneously.
    // Note: if you want to reset the Python environment and restart from scratch,
    //  simply use new .. delete... etc. to allocate ChPythonEngine objs on heap

    ChPythonEngine my_python;

    //
    // TEST 1   -   figure out what version of Python is run under the hood
    //

    GetLog() << " PyChrono Test 1.\n";
    my_python.Run("import sys");
    GetLog() << "Python version run by Chrono:\n";
    my_python.Run("print (sys.version)");

    //
    // TEST 2   -   execute simple instructions
    //

    GetLog() << "\n\n PyChrono Test 2.\n";
    my_python.Run("a =8.6");
    my_python.Run("b =4");
    my_python.Run("c ='blabla' ");
    my_python.Run("print('In:Python - A computation:', a/2)");

    //
    // TEST 3   -   fetch a value from a python variable (in __main__ namespace)
    //

    GetLog() << "\n\n PyChrono Test 3.\n";
    double mfval;
    my_python.GetFloat("a", mfval);
    GetLog() << "In:C++    - Passed float variable 'a' from Python, a=" << mfval << "\n";
    int mival;
    my_python.GetInteger("b", mival);
    GetLog() << "In:C++    - Passed integer variable 'b' from Python, b=" << mival << "\n";
    std::string msval;
    if (!my_python.GetString("c", msval))
        GetLog() << "Can't fetch string \n";
    GetLog() << "In:C++    - Passed string variable 'c' from Python, c=" << msval << "\n";

    //
    // TEST 4   -   set a value into a python variable (in __main__ namespace)
    //

    GetLog() << "\n\n PyChrono Test 4.\n";
    my_python.SetFloat("d", 123.5);
    my_python.Run("print('In:Python - Passed variable d from c++, d=', d)");

    //
    // TEST 5   -   errors and exceptions
    //

    // In the previous examples we didn't have any syntax errors.
    // In general, it is wise to enclose Python commands in a try-catch block
    // because errors are handled with exceptions:

    GetLog() << "\n\n PyChrono Test 5.\n";
    try {
        my_python.Run("a= this_itGoInG_TO_giVe_ErroRs!()");
    } catch (const ChException&) {
        GetLog() << "Ok, Python parsing error caught as expected.\n";
    }

    return 0;
}
