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

ChMatlabEngine::ChMatlabEngine() {
#ifdef __APPLE__
    ep = matlabengine::engOpen("matlab -automation -nosplash \0");
    if (!ep) {
        throw ChException("Can't start MATLAB engine");
    }
#else
    ep = matlabengine::engOpen("-automation \0");
    if (!ep) {
        throw ChException("Can't start MATLAB engine");
    }
#endif
}

ChMatlabEngine::~ChMatlabEngine() {
    if (ep)
        if (!m_persist)
            matlabengine::engClose(ep);
        else
            matlabengine::engSetVisible(ep, true); // set engine as 'visible', for safety
    ep = nullptr;
}

// Return pointer to internal Matlab engine (avoid using it directly,
// if you can use other functions of this class that 'wrap' it.)
matlabengine::Engine* ChMatlabEngine::GetEngine() {
    return ep;
}

// Set visibility of GUI matlab window.
bool ChMatlabEngine::SetVisible(bool mvis) {
    if (matlabengine::engSetVisible(ep, mvis) == 0)
        return true;
    else
        return false;
}

// Keep matlab engine open even after termination of C++ program.
void ChMatlabEngine::KeepEngineOpen(bool persist) {
    m_persist = persist;
    matlabengine::engSetVisible(ep, true); // set engine as 'visible', for safety
}

// Evaluate a Matlab instruction (as a string). If error happens while executing, returns false.
bool ChMatlabEngine::Eval(std::string mstring) {
    if (matlabengine::engEvalString(ep, mstring.c_str()) == 0)
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
    matlabengine::engPutVariable(ep, varname.c_str(), T);
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

    char buff[100];
    sprintf(buff, "%s=spconvert(%s)", varname.c_str(), varname.c_str());
    this->Eval(buff);

    return true;
}

// Fetch a matrix from Matlab environment, specifying its name as variable.
// The used matrix must be of ChMatrixDynamic<double> type because
// it might undergo resizing.
bool ChMatlabEngine::GetVariable(ChMatrixDynamic<double>& mmatr, std::string varname) {
    ChMatrixDynamic<> transfer;  // elements in Matlab are column-major

    matlabengine::mxArray* T = matlabengine::engGetVariable(ep, varname.c_str());
    if (T) {
        matlabengine::mwSize ndi = mxGetNumberOfDimensions(T);
        if (ndi != 2) {
            matlabengine::mxDestroyArray(T);
            return false;
        }
        const matlabengine::mwSize* siz = mxGetDimensions(T);
        transfer.resize((int)siz[1], (int)siz[0]);
        memcpy((char*)transfer.data(), (char*)mxGetPr(T),
               transfer.rows() * transfer.cols() * sizeof(double));
        matlabengine::mxDestroyArray(T);

        mmatr = transfer.transpose();

        return true;
    }
    matlabengine::mxDestroyArray(T);
    return false;    
}

// Fetch a string from Matlab environment, specifying its name as variable.
bool ChMatlabEngine::GetString(std::string& str, std::string varname) {
    matlabengine::mxArray* T = matlabengine::engGetVariable(ep, varname.c_str());
    if (T) {
        str = matlabengine::mxArrayToUTF8String_800(T);
        return true;
    }
    return false;
}

}  // end namespace chrono
