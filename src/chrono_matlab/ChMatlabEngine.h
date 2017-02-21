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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHMATLABENGINE_H
#define CHMATLABENGINE_H

#include "chrono_matlab/ChApiMatlab.h"
#include "chrono/core/ChMath.h"
#include "chrono/core/ChLinkedListMatrix.h"

// Following namespace trick is a fix for VS2010+ and Matlab: avoid error with typedef in matrix.h
namespace matlabengine {
#include "engine.h"
};

namespace chrono {

/// @addtogroup matlab_module
/// @{

/// Class for accessing the Matlab engine with a C++ wrapper.
/// When a ChMatlabEngine object is instanced, a Matlab engine
/// is started (assuming Matlab is properly installed and its
/// dll are available on the system) and following funcitons can
/// be used to copy variables from/to chrono::engine, and
/// to execute commands in Matlab. Useful also to exploit the
/// powerful plotting features of Matlab.
/// Note! to compile programs that include this header, your
/// makefile must be properly configured to set the Matlab SDK
/// directory, so that matlab headers and libs can be compiled/linked
/// Also, if you run an executable that used this header on a
/// system that has no Matlab installed, a 'missing dll' error
/// will pop up as soon as you try to start the program.

class ChApiMatlab ChMatlabEngine {
  private:
    //
    // DATA
    //
    matlabengine::Engine* ep;

  public:
    //
    // FUNCTIONS
    //

    ChMatlabEngine();

    virtual ~ChMatlabEngine();
    /// Return pointer to internal Matlab engine (avoid using it directly,
    /// if you can use other functions of this class that 'wrap' it.)
    matlabengine::Engine* GetEngine();

    /// Evaluate a Matlab instruction (as a string). If error happens while executing, returns false.
    bool Eval(std::string mstring);

    /// Set visibility of GUI matlab window.
    bool SetVisible(bool mvis);

    /// Put a matrix in Matlab environment, specifying its name as variable.
    /// If a variable with the same name already exist, it is overwritten.
    bool PutVariable(const ChMatrix<double>& mmatr, std::string varname);

    /// Put a sparse matrix in Matlab environment, specifying its name as variable.
    /// If a variable with the same name already exist, it is overwritten.
    bool PutSparseMatrix(const ChSparseMatrix& mmatr, std::string varname);

    /// Fetch a matrix from Matlab environment, specifying its name as variable.
    /// The used matrix must be of ChMatrixDynamic<double> type because
    /// it might undergo resizing.
    bool GetVariable(ChMatrixDynamic<double>& mmatr, std::string varname);

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite<ChMatlabEngine>();
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead<ChMatlabEngine>();
    }
};

/// @} matlab_module

}  // end namespace chrono

#endif
