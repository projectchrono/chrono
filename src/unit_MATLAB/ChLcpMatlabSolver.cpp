//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHMATLABENGINE_H
#define CHMATLABENGINE_H

///////////////////////////////////////////////////
//  
//   ChMatlabEngine.h
//
//   Use this header if you want to exploit Matlab
//   from Chrono::Engine programs.
//
//   HEADER file for CHRONO,
//  Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChMath.h"
#include "core/ChSpmatrix.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSolver.h"

// include also the Matlab header..
#include "engine.h"



namespace chrono
{


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

class ChMatlabEngine
{
private:
      //
      // DATA
      //
   Engine *ep;

public:
      //
      // FUNCTIONS
      //

   ChMatlabEngine()
   {
      if (!(ep = engOpen("-automation \0")))
      {
         throw ChException("Can't start MATLAB engine");
      }
   }

   ~ChMatlabEngine()
   {
      if (ep) engClose(ep); ep = 0;
   }
      /// Return pointer to internal Matlab engine (avoid using it directly,
      /// if you can use other functions of this class that 'wrap' it.)
   Engine* GetEngine() {return ep;}

      /// Evaluate a Matlab instruction. If error happens while executing, returns false.
   bool Eval(char* mstring)
   {
      if (engEvalString(ep, mstring) == 0) return true;
      else return false;
   }
      /// Set visibility of GUI matlab window.
   bool SetVisible(bool mvis)
   {
      if (engSetVisible(ep, mvis) == 0) return true;
      else return false;
   }

      /// Put a matrix in Matlab environment, specifying its name as variable.
      /// If a variable with the same name already exist, it is overwritten.
   bool PutVariable(const ChMatrix<double>& mmatr, char* varname)
   {
      ChMatrixDynamic<> transfer; // elements in Matlab are column-major
      transfer.CopyFromMatrixT(mmatr);

      mxArray *T = NULL;
      T = mxCreateDoubleMatrix(mmatr.GetRows(), mmatr.GetColumns(), mxREAL);
      memcpy((char *) mxGetPr(T), (char *) transfer.GetAddress(), mmatr.GetRows()*mmatr.GetColumns()*sizeof(double));
      engPutVariable(ep, varname, T);
      mxDestroyArray(T);
      return true;
   }

      /// Put a sparse matrix in Matlab environment, specifying its name as variable.
      /// If a variable with the same name already exist, it is overwritten.
   bool PutSparseMatrix(const ChSparseMatrix& mmatr, char* varname)
   {
      int nels = 0;
      for(int ii=0; ii<mmatr.GetRows(); ii++)
         for(int jj=0; jj<mmatr.GetColumns(); jj++)
         {
            double elVal = ((ChSparseMatrix&)mmatr).GetElement(ii,jj);
            if (elVal || (ii+1==((ChSparseMatrix&)mmatr).GetRows() && jj+1==((ChSparseMatrix&)mmatr).GetColumns()))
               ++nels;
         }

      ChMatrixDynamic<> transfer(nels,3);

      int eln = 0;
      for(int ii=0; ii<mmatr.GetRows(); ii++)
         for(int jj=0; jj<mmatr.GetColumns(); jj++)
         {
            double elVal = ((ChSparseMatrix&)mmatr).GetElement(ii,jj);
            if (elVal || (ii+1==((ChSparseMatrix&)mmatr).GetRows() && jj+1==((ChSparseMatrix&)mmatr).GetColumns()))
            {
               transfer(eln,0) = ii+1;
               transfer(eln,1) = jj+1;
               transfer(eln,2) = elVal;
               ++eln;
            }
         }

      this->PutVariable(transfer, varname);

      char buff[100];
      sprintf(buff, "%s=spconvert(%s)", varname,varname);
      this->Eval(buff);

      return true;
   }

      /// Fetch a matrix from Matlab environment, specifying its name as variable.
      /// The used matrix must be of ChMatrixDynamic<double> type because
      /// it might undergo resizing.
   bool GetVariable(ChMatrixDynamic<double>& mmatr, char* varname)
   {
      ChMatrixDynamic<> transfer; // elements in Matlab are column-major

      mxArray* T = engGetVariable(ep, varname);
      if (T)
      {
         mwSize  ndi = mxGetNumberOfDimensions(T);
         if (ndi != 2)
         {
            mxDestroyArray(T);
            return false;
         }
         const mwSize* siz = mxGetDimensions(T);
         transfer.Resize(siz[1],siz[0]);
         memcpy((char *) transfer.GetAddress(), (char *) mxGetPr(T), transfer.GetRows()*transfer.GetColumns()*sizeof(double));
         mxDestroyArray(T);

         mmatr.CopyFromMatrixT(transfer);

         return true;
      }
      mxDestroyArray(T);
      return false;
   }


};





class ChLcpMatlabSolver : public ChLcpSolver
{
protected:
         //
         // DATA
         //
   ChMatlabEngine* mengine;
public:
         //
         // CONSTRUCTORS
         //

   ChLcpMatlabSolver(ChMatlabEngine& me) { mengine = &me;};

   virtual ~ChLcpMatlabSolver(){};


            /// Solve using the Matlab default direct solver (as in x=A\b)

   virtual double Solve(
            ChLcpSystemDescriptor& sysd      ///< system description with constraints and variables
            )
   {
      chrono::ChSparseMatrix mdM;
      chrono::ChSparseMatrix mdCq;
      chrono::ChSparseMatrix mdE;
      chrono::ChMatrixDynamic<double> mdf;
      chrono::ChMatrixDynamic<double> mdb;
      chrono::ChMatrixDynamic<double> mdfric;
      sysd.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);

      mengine->PutSparseMatrix(mdM, "mdM");
      mengine->PutSparseMatrix(mdCq,"mdCq");
      mengine->PutSparseMatrix(mdE, "mdE");
      mengine->PutVariable(mdf, "mdf");
      mengine->PutVariable(mdb, "mdb");
      mengine->PutVariable(mdfric, "mdfric");

      mengine->Eval("mdZ = [mdM, mdCq'; mdCq, -mdE]; mdd=[mdf;-mdb];");

      mengine->Eval("mdx = mldivide(mdZ , mdd);");

      chrono::ChMatrixDynamic<> mx;
      if (!mengine->GetVariable(mx, "mdx"))
         GetLog() << "ERROR!! cannot fetch mdx";

      sysd.FromVectorToUnknowns(mx);


      mengine->Eval("resid = norm(mdZ*mdx - mdd);");
      chrono::ChMatrixDynamic<> mres;
      mengine->GetVariable(mres, "resid");
      GetLog() << " Matlab computed residual:" << mres(0,0) << "\n";


      return 0;

   }
};




} // END_OF_NAMESPACE____


#endif
