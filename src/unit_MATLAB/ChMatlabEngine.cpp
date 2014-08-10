#include "ChMatlabEngine.h"
using namespace chrono;

ChMatlabEngine::ChMatlabEngine() {
   if (!(ep = engOpen("-automation \0"))) {
      throw ChException("Can't start MATLAB engine");
   }
}

ChMatlabEngine::~ChMatlabEngine() {
   if (ep)
      engClose (ep);
   ep = 0;
}
/// Return pointer to internal Matlab engine (avoid using it directly,
/// if you can use other functions of this class that 'wrap' it.)
Engine* ChMatlabEngine::GetEngine() {
   return ep;
}

/// Evaluate a Matlab instruction. If error happens while executing, returns false.
bool ChMatlabEngine::Eval(char* mstring) {
   if (engEvalString(ep, mstring) == 0)
      return true;
   else
      return false;
}
/// Set visibility of GUI matlab window.
bool ChMatlabEngine::SetVisible(bool mvis) {
   if (engSetVisible(ep, mvis) == 0)
      return true;
   else
      return false;
}

/// Put a matrix in Matlab environment, specifying its name as variable.
/// If a variable with the same name already exist, it is overwritten.
bool ChMatlabEngine::PutVariable(const ChMatrix<double>& mmatr,
                                 char* varname) {
   ChMatrixDynamic<> transfer;  // elements in Matlab are column-major
   transfer.CopyFromMatrixT(mmatr);

   mxArray *T = NULL;
   T = mxCreateDoubleMatrix(mmatr.GetRows(), mmatr.GetColumns(), mxREAL);
   memcpy((char *) mxGetPr(T), (char *) transfer.GetAddress(), mmatr.GetRows() * mmatr.GetColumns() * sizeof(double));
   engPutVariable(ep, varname, T);
   mxDestroyArray(T);
   return true;
}

/// Put a sparse matrix in Matlab environment, specifying its name as variable.
/// If a variable with the same name already exist, it is overwritten.
bool ChMatlabEngine::PutSparseMatrix(const ChSparseMatrix& mmatr,
                                     char* varname) {
   int nels = 0;
   for (int ii = 0; ii < mmatr.GetRows(); ii++)
      for (int jj = 0; jj < mmatr.GetColumns(); jj++) {
         double elVal = ((ChSparseMatrix&) mmatr).GetElement(ii, jj);
         if (elVal || (ii + 1 == ((ChSparseMatrix&) mmatr).GetRows() && jj + 1 == ((ChSparseMatrix&) mmatr).GetColumns()))
            ++nels;
      }

   ChMatrixDynamic<> transfer(nels, 3);

   int eln = 0;
   for (int ii = 0; ii < mmatr.GetRows(); ii++)
      for (int jj = 0; jj < mmatr.GetColumns(); jj++) {
         double elVal = ((ChSparseMatrix&) mmatr).GetElement(ii, jj);
         if (elVal || (ii + 1 == ((ChSparseMatrix&) mmatr).GetRows() && jj + 1 == ((ChSparseMatrix&) mmatr).GetColumns())) {
            transfer(eln, 0) = ii + 1;
            transfer(eln, 1) = jj + 1;
            transfer(eln, 2) = elVal;
            ++eln;
         }
      }

   this->PutVariable(transfer, varname);

   char buff[100];
   sprintf(buff, "%s=spconvert(%s)", varname, varname);
   this->Eval(buff);

   return true;
}

/// Fetch a matrix from Matlab environment, specifying its name as variable.
/// The used matrix must be of ChMatrixDynamic<double> type because
/// it might undergo resizing.
bool ChMatlabEngine::GetVariable(ChMatrixDynamic<double>& mmatr,
                                 char* varname) {
   ChMatrixDynamic<> transfer;  // elements in Matlab are column-major

   mxArray* T = engGetVariable(ep, varname);
   if (T) {
      mwSize ndi = mxGetNumberOfDimensions(T);
      if (ndi != 2) {
         mxDestroyArray(T);
         return false;
      }
      const mwSize* siz = mxGetDimensions(T);
      transfer.Resize(siz[1], siz[0]);
      memcpy((char *) transfer.GetAddress(), (char *) mxGetPr(T), transfer.GetRows() * transfer.GetColumns() * sizeof(double));
      mxDestroyArray(T);

      mmatr.CopyFromMatrixT(transfer);

      return true;
   }
   mxDestroyArray(T);
   return false;
}

