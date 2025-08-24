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
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    std::cout << " Test the execution of Python statements, formulas, programs.\n No graphical user interface.\n\n";

    // Use a ChPythonEngine object.
    // Note: currently no multiple ChPythonEngine objects can be used simultaneously.
    // Note: if you want to reset the Python environment and restart from scratch,
    //  simply use new .. delete... etc. to allocate ChPythonEngine objs on heap

    ChPythonEngine my_python;

    //
    // TEST 1   -   figure out what version of Python is run under the hood
    //

    std::cout << " PyChrono Test 1.\n";
    my_python.Run("import sys");
    std::cout << "Python version run by Chrono:\n";
    my_python.Run("print (sys.version)");

    //
    // TEST 2   -   execute simple instructions
    //

    std::cout << "\n\n PyChrono Test 2.\n";
    my_python.Run("a =8.6");
    my_python.Run("b =4");
    my_python.Run("c ='blabla' ");
    my_python.Run("print('In:Python - A computation:', a/2)");

    //
    // TEST 3   -   fetch a value from a python variable (in __main__ namespace)
    //

    std::cout << "\n\n PyChrono Test 3.\n";
    double mfval;
    my_python.GetFloat("a", mfval);
    std::cout << "In:C++    - Passed float variable 'a' from Python, a=" << mfval << "\n";
    int mival;
    my_python.GetInteger("b", mival);
    std::cout << "In:C++    - Passed integer variable 'b' from Python, b=" << mival << "\n";
    std::string msval;
    if (!my_python.GetString("c", msval))
        std::cerr << "Can't fetch string \n";
    std::cout << "In:C++    - Passed string variable 'c' from Python, c=" << msval << "\n";

    //
    // TEST 4   -   set a value into a python variable (in __main__ namespace)
    //

    std::cout << "\n\n PyChrono Test 4.\n";
    my_python.SetFloat("d", 123.5);
    my_python.Run("print('In:Python - Passed variable d from c++, d=', d)");

    //
    // TEST 5   -   set/retrieve numeric containers to/from a python list variable (in __main__ namespace)
    //

    std::cout << "\n\n PyChrono Test 5.\n";
    std::vector<double> stdvec = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};  // input as std::vector
    my_python.SetList("numvec", stdvec);
    my_python.Run("numvec = [ii * 2 for ii in numvec]\n");  // double each element
    ChVectorDynamic<> eigvec;
    my_python.GetList("numvec", eigvec);  // retrieve it as ChVectorDynamic<>, if desired
    std::cout << "Doubled C++ numeric vector: " << eigvec.transpose() << "\n";

    ChMatrixDynamic<> mymatr(3, 3);
    mymatr.setIdentity();
    my_python.SetMatrix("matr", mymatr);
    my_python.Run("matr = [[ii * 3 for ii in row] for row in matr]\n");  // triple each element
    my_python.GetMatrix("matr", mymatr);
    std::cout << "Tripled C++ numeric matrix:\n" << mymatr << "\n";

    //
    // TEST 6   -   errors and exceptions
    //

    // In the previous examples we didn't have any syntax errors.
    // In general, it is wise to enclose Python commands in a try-catch block
    // because errors are handled with exceptions:

    std::cout << "\n\n PyChrono Test 6.\n";
    try {
        my_python.Run("a= this_itGoInG_TO_giVe_ErroRs!()");
    } catch (std::exception&) {
        std::cout << "Ok, Python parsing error caught as expected.\n";
    }

    return 0;
}
