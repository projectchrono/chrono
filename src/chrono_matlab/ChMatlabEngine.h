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

#ifndef CHMATLABENGINE_H
#define CHMATLABENGINE_H

#include "chrono_matlab/ChApiMatlab.h"
#include "chrono/core/ChFrame.h"

namespace matlabengine {
#include "engine.h"
};

namespace chrono {

/// @addtogroup matlab_module
/// @{

/// Class for accessing the Matlab engine.
/// A ChMatlabEngine allows variables to be copied from/to Chrono and Matlab command executed.
class ChApiMatlab ChMatlabEngine {
  public:
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
    bool PutVariable(ChMatrixConstRef mmatr, std::string varname);

    /// Put a sparse matrix in Matlab environment, specifying its name as variable.
    /// If a variable with the same name already exist, it is overwritten.
    bool PutSparseMatrix(const ChSparseMatrix& mmatr, std::string varname);

    /// Fetch a matrix from Matlab environment, specifying its name as variable.
    /// The used matrix must be of ChMatrixDynamic<double> type because
    /// it might undergo resizing.
    bool GetVariable(ChMatrixDynamic<double>& mmatr, std::string varname);

    /// Fetch a string from Matlab environment, specifying its name as variable.
    bool GetString(std::string& str, std::string varname);

    /// Keep matlab engine open even after termination of C++ program.
    /// Useful to skip initial time to reload engine (NB: thus, it must be closed manually).
    void KeepEngineOpen(bool open);

    /// Method to allow serialization of transient data from archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

  private:
    matlabengine::Engine* m_engine;
    bool m_persist;
};

/// @} matlab_module

}  // end namespace chrono

#endif
