//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
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
//     - using the unit_PYPARSER for executing
//       some Python program or formula
//     - using the unit_PYPARSER for loading a
//       .py scene description saved from the
//       SolidWorks add-in
//
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_python/ChPython.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include <iostream>
#include <sstream>

using namespace chrono;

int main(int argc, char* argv[]) {
    GetLog() << " Test the execution of Python statements, formulas, programs.\n No graphical user interface.\n\n";

    // Use a ChPythonEngine object.
    // Note: currently no multiple ChPythonEngine at a single time can be used.
    // Note: if you want to reset the Python environment and restart from scratch,
    //  simply use new .. delete... etc. to allocate ChPythonEngine objs on heap

    ChPythonEngine my_python;

    //
    // TEST 1   -   execute simple instructions
    //

    my_python.Run("a =8.6");
    my_python.Run("b =4");
    my_python.Run("c ='blabla' ");
    my_python.Run("print('In:Python - A computation:', a/2)");

    //
    // TEST 2   -   fetch a value from a python variable (in __main__ namespace)
    //

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
    // TEST 3   -   set a value into a python variable (in __main__ namespace)
    //

    my_python.SetFloat("d", 123.5);
    my_python.Run("print('In:Python - Passed variable d from c++, d=', d)");

    //
    // TEST 4   -   errors and exceptions
    //

    // In previous examples we were sure that no syntax errors could happen,
    // but in general errors could happen in Python parsing and execution, so
    // it is wise to enclose Python stuff in a try-catch block because errors
    // are handled with exceptions:

    try {
        my_python.Run("a= this_itGoInG_TO_giVe_ErroRs!()");
    } catch (ChException myerror) {
        GetLog() << "Ok, Python parsing error catched as expected!\n";
    }

    //
    // TEST 5   -   load mechanical system, previously saved to disk from SolidWorks add-in
    //

    ChSystem my_system;

    try {
        // This is the instruction that loads the .py (as saved from SolidWorks) and
        // fills the system:

        my_python.ImportSolidWorksSystem(GetChronoDataFile("solid_works/swiss_escapement").c_str(), 
                                         my_system);  // note, don't type the .py suffic in filename..

        my_system.ShowHierarchy(GetLog());

        // In case you want to fetch an item, remember that they got the
        // names that you see in the CAD interface, for example suppose you know that
        // a ChBodyAuxRef has the name "escape_wheel^escapement-1":
        std::shared_ptr<ChBodyAuxRef> mbody;
        ChSystem::IteratorBodies myiterp = my_system.IterBeginBodies();
        while (myiterp != my_system.IterEndBodies()) {
            GetLog() << (*myiterp)->GetNameString().c_str() << "\n";
            if ((*myiterp)->GetNameString() == "escape_wheel^escapement-1") 
                mbody = std::dynamic_pointer_cast<ChBodyAuxRef>(*myiterp);

            ++myiterp;
        }

        if (!mbody) 
            throw ChException("Error. Could not find body from its name in SolidWorks exported file");
        else
            GetLog() << "Found body  its name in SolidWorks exported file, pos.x=" << mbody->GetPos().x << "\n";


    } catch (ChException myerror) {
        GetLog() << myerror.what();
    }

    return 0;
}
