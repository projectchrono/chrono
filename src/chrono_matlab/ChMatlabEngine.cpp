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

#include "chrono_matlab/ChMatlabEngine.h"

namespace chrono {

ChMatlabEngine::ChMatlabEngine() : m_persist(false) {
#ifdef __APPLE__
    m_engine = matlabengine::engOpen("matlab -automation -nosplash \0");
    if (!m_engine) {
        throw std::runtime_error("Can't start MATLAB engine");
    }
#else
    m_engine = matlabengine::engOpen("-automation \0");
    if (!m_engine) {
        throw std::runtime_error("Can't start MATLAB engine");
    }
#endif
}

ChMatlabEngine::~ChMatlabEngine() {
    if (m_engine)
        if (!m_persist)
            matlabengine::engClose(m_engine);
        else
            matlabengine::engSetVisible(m_engine, true);  // set engine as 'visible', for safety
    m_engine = nullptr;
}

// Return pointer to internal Matlab engine (avoid using it directly,
// if you can use other functions of this class that 'wrap' it.)
matlabengine::Engine* ChMatlabEngine::GetEngine() {
    return m_engine;
}

// Set visibility of GUI matlab window.
bool ChMatlabEngine::SetVisible(bool mvis) {
    if (matlabengine::engSetVisible(m_engine, mvis) == 0)
        return true;
    else
        return false;
}

// Keep matlab engine open even after termination of C++ program.
void ChMatlabEngine::KeepEngineOpen(bool persist) {
    m_persist = persist;
    matlabengine::engSetVisible(m_engine, true);  // set engine as 'visible', for safety
}

// Evaluate a Matlab instruction (as a string). If error happens while executing, returns false.
bool ChMatlabEngine::Eval(std::string mstring) {
    if (matlabengine::engEvalString(m_engine, mstring.c_str()) == 0)
        return true;
    else
        return false;
}

// Put a matrix in Matlab environment, specifying its name as variable.
// If a variable with the same name already exist, it is overwritten.
bool ChMatlabEngine::PutVariable(ChMatrixConstRef mmatr, std::string varname) {
    // elements in Matlab are column-major
    ChMatrixDynamic<> transfer = mmatr.transpose();

    matlabengine::mxArray* T = NULL;
    T = matlabengine::mxCreateDoubleMatrix(mmatr.rows(), mmatr.cols(), matlabengine::mxREAL);
    memcpy((char*)mxGetPr(T), (char*)transfer.data(), mmatr.rows() * mmatr.cols() * sizeof(double));
    matlabengine::engPutVariable(m_engine, varname.c_str(), T);
    matlabengine::mxDestroyArray(T);
    return true;
}

// Put a sparse matrix in Matlab environment, specifying its name as variable.
// If a variable with the same name already exist, it is overwritten.
bool ChMatlabEngine::PutSparseMatrix(const ChSparseMatrix& mmatr, std::string varname) {
    int nels = 0;
    for (int ii = 0; ii < mmatr.rows(); ii++)
        for (int jj = 0; jj < mmatr.cols(); jj++) {
            double elVal = mmatr.coeff(ii, jj);
            if (elVal || (ii + 1 == mmatr.rows() && jj + 1 == mmatr.cols()))
                ++nels;
        }

    ChMatrixDynamic<> transfer(nels, 3);

    int eln = 0;
    for (int ii = 0; ii < mmatr.rows(); ii++)
        for (int jj = 0; jj < mmatr.cols(); jj++) {
            double elVal = mmatr.coeff(ii, jj);
            if (elVal || (ii + 1 == mmatr.rows() && jj + 1 == mmatr.cols())) {
                transfer(eln, 0) = ii + 1;
                transfer(eln, 1) = jj + 1;
                transfer(eln, 2) = elVal;
                ++eln;
            }
        }

    this->PutVariable(transfer, varname.c_str());

    std::string buf = varname + " = spconvert(" + varname + ")";
    this->Eval(buf);

    return true;
}

// Fetch a matrix from Matlab environment, specifying its name as variable.
// The used matrix must be of ChMatrixDynamic<double> type because
// it might undergo resizing.
bool ChMatlabEngine::GetVariable(ChMatrixDynamic<double>& mmatr, std::string varname) {
    ChMatrixDynamic<> transfer;  // elements in Matlab are column-major

    matlabengine::mxArray* T = matlabengine::engGetVariable(m_engine, varname.c_str());
    if (T) {
        matlabengine::mwSize ndi = mxGetNumberOfDimensions(T);
        if (ndi != 2) {
            matlabengine::mxDestroyArray(T);
            return false;
        }
        const matlabengine::mwSize* siz = mxGetDimensions(T);
        transfer.resize((int)siz[1], (int)siz[0]);
        memcpy((char*)transfer.data(), (char*)mxGetPr(T), transfer.rows() * transfer.cols() * sizeof(double));
        matlabengine::mxDestroyArray(T);

        mmatr = transfer.transpose();

        return true;
    }
    matlabengine::mxDestroyArray(T);
    return false;
}

// Fetch a string from Matlab environment, specifying its name as variable.
bool ChMatlabEngine::GetString(std::string& str, std::string varname) {
    matlabengine::mxArray* T = matlabengine::engGetVariable(m_engine, varname.c_str());
    if (T) {
        str = matlabengine::mxArrayToUTF8String_800(T);
        return true;
    }
    return false;
}

void ChMatlabEngine::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChMatlabEngine>();
}

/// Method to allow de-serialization of transient data from archives.
void ChMatlabEngine::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChMatlabEngine>();
}

}  // end namespace chrono
