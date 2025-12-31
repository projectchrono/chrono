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
//   Demonstration on how to call Matlab from Chrono
//
// =============================================================================

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
        // EXAMPLE 1: execute a Matlab command
        //

        std::cout << "- Execute plotting command from Chrono...\n\n";

        matlab_engine.Eval(
            "z=peaks(25); \
						    surf(z);  \
							colormap(jet); \
							pause(4); \
							");

        //
        // EXAMPLE 2: pass a Chrono matrix to Matlab
        //

        std::cout << "- Send some data to Matlab for operations and plotting...\n\n";

        ChMatrixDynamic<> m_time(30, 1);
        ChMatrixDynamic<> m_sine(30, 1);
        for (int i = 0; i < 30; i++) {
            m_time(i, 0) = ((double)i / 30.) * 5.;
            m_sine(i, 0) = sin(m_time(i, 0) * 2.);
        }
        matlab_engine.PutVariable(m_time, "m_time");
        matlab_engine.PutVariable(m_sine, "m_sine");
        matlab_engine.Eval("figure; plot(m_time,m_sine);");

        //
        // EXAMPLE 3: pass a Matlab matrix to Chrono
        //

        std::cout << "- Fetch some data from Matlab...\n\n";

        matlab_engine.Eval("m_matr=[0:0.1:5]';");

        ChMatrixDynamic<double> m_matr;
        matlab_engine.GetVariable(m_matr, "m_matr");
        std::cout << m_matr << std::endl;

        //
        // EXAMPLE 4: pass a sparse matrix to Matlab
        //

        std::cout << "- Send a sparse matrix to Matlab...\n\n";

        ChSparseMatrix m_sparse(6, 7);
        m_sparse.SetElement(3, 5, 102);
        m_sparse.SetElement(1, 2, 104);
        m_sparse.SetElement(4, 4, 101);

        matlab_engine.PutSparseMatrix(m_sparse, "m_sparse");
        matlab_engine.Eval("figure; spy(m_sparse);");

        // Wait some seconds before closing all

        matlab_engine.Eval("pause(60)");

    } catch (std::exception mex) {
        std::cerr << mex.what() << std::endl;  // Print error on console, if Matlab did not start.
    }

    system("pause");
    return 0;
}
