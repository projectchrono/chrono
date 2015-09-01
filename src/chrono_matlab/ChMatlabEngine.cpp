#include "chrono_matlab/ChMatlabEngine.h"

using namespace chrono;
using namespace std;

ChMatlabEngine::ChMatlabEngine() {
#ifdef __APPLE__
    if (!(ep = matlabengine::engOpen("matlab -automation -nosplash \0"))) {
        throw ChException("Can't start MATLAB engine");
    }
#else
    if (!(ep = matlabengine::engOpen("-automation \0"))) {
        throw ChException("Can't start MATLAB engine");
    }
#endif
}

ChMatlabEngine::~ChMatlabEngine() {
    if (ep)
        matlabengine::engClose(ep);
    ep = 0;
}
/// Return pointer to internal Matlab engine (avoid using it directly,
/// if you can use other functions of this class that 'wrap' it.)
matlabengine::Engine* ChMatlabEngine::GetEngine() {
    return ep;
}

/// Evaluate a Matlab instruction (as a string). If error happens while executing, returns false.
bool ChMatlabEngine::Eval(string mstring) {
    if (matlabengine::engEvalString(ep, mstring.c_str()) == 0)
        return true;
    else
        return false;
}

/// Set visibility of GUI matlab window.
bool ChMatlabEngine::SetVisible(bool mvis) {
    if (matlabengine::engSetVisible(ep, mvis) == 0)
        return true;
    else
        return false;
}

/// Put a matrix in Matlab environment, specifying its name as variable.
/// If a variable with the same name already exist, it is overwritten.
bool ChMatlabEngine::PutVariable(const ChMatrix<double>& mmatr, string varname) {
    ChMatrixDynamic<> transfer;  // elements in Matlab are column-major
    transfer.CopyFromMatrixT(mmatr);

    matlabengine::mxArray* T = NULL;
    T = matlabengine::mxCreateDoubleMatrix(mmatr.GetRows(), mmatr.GetColumns(), matlabengine::mxREAL);
    memcpy((char*)mxGetPr(T), (char*)transfer.GetAddress(), mmatr.GetRows() * mmatr.GetColumns() * sizeof(double));
    matlabengine::engPutVariable(ep, varname.c_str(), T);
    matlabengine::mxDestroyArray(T);
    return true;
}

/// Put a sparse matrix in Matlab environment, specifying its name as variable.
/// If a variable with the same name already exist, it is overwritten.
bool ChMatlabEngine::PutSparseMatrix(const ChSparseMatrix& mmatr, string varname) {
    int nels = 0;
    for (int ii = 0; ii < mmatr.GetRows(); ii++)
        for (int jj = 0; jj < mmatr.GetColumns(); jj++) {
            double elVal = ((ChSparseMatrix&)mmatr).GetElement(ii, jj);
            if (elVal ||
                (ii + 1 == ((ChSparseMatrix&)mmatr).GetRows() && jj + 1 == ((ChSparseMatrix&)mmatr).GetColumns()))
                ++nels;
        }

    ChMatrixDynamic<> transfer(nels, 3);

    int eln = 0;
    for (int ii = 0; ii < mmatr.GetRows(); ii++)
        for (int jj = 0; jj < mmatr.GetColumns(); jj++) {
            double elVal = ((ChSparseMatrix&)mmatr).GetElement(ii, jj);
            if (elVal ||
                (ii + 1 == ((ChSparseMatrix&)mmatr).GetRows() && jj + 1 == ((ChSparseMatrix&)mmatr).GetColumns())) {
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

/// Fetch a matrix from Matlab environment, specifying its name as variable.
/// The used matrix must be of ChMatrixDynamic<double> type because
/// it might undergo resizing.
bool ChMatlabEngine::GetVariable(ChMatrixDynamic<double>& mmatr, string varname) {
    ChMatrixDynamic<> transfer;  // elements in Matlab are column-major

    matlabengine::mxArray* T = matlabengine::engGetVariable(ep, varname.c_str());
    if (T) {
        matlabengine::mwSize ndi = mxGetNumberOfDimensions(T);
        if (ndi != 2) {
            matlabengine::mxDestroyArray(T);
            return false;
        }
        const matlabengine::mwSize* siz = mxGetDimensions(T);
        transfer.Resize((int)siz[1], (int)siz[0]);
        memcpy((char*)transfer.GetAddress(), (char*)mxGetPr(T),
               transfer.GetRows() * transfer.GetColumns() * sizeof(double));
        matlabengine::mxDestroyArray(T);

        mmatr.CopyFromMatrixT(transfer);

        return true;
    }
    matlabengine::mxDestroyArray(T);
    return false;
}
