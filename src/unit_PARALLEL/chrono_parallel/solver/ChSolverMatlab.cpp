#include "chrono_parallel/solver/ChSolverMatlab.h"

using namespace chrono;

uint ChSolverMatlab::SolveMatlab(const uint max_iter,
                                 const uint size,
                                 const custom_vector<real> &b,
                                 custom_vector<real> &x) {




}

/*
 * //
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
#include "lcp/ChLcpIterativeSolver.h"
// include also the Matlab header..
#include "engine.h"

namespace chrono {

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

   class ChMatlabEngine {
      private:
         //
         // DATA
         //
         Engine *ep;

      public:
         //
         // FUNCTIONS
         //

         ChMatlabEngine() {
            if (!(ep = engOpen("matlab -automation -nosplash \0"))) {
               throw ChException("Can't start MATLAB engine");
            }
         }

         ~ChMatlabEngine() {
            if (ep) engClose(ep);
            ep = 0;
         }
         /// Return pointer to internal Matlab engine (avoid using it directly,
         /// if you can use other functions of this class that 'wrap' it.)
         Engine* GetEngine() {
            return ep;
         }

//       /// Evaluate a Matlab instruction. If error happens while executing, returns false.
//       bool Eval(char* mstring) {
//          if (engEvalString(ep, mstring) == 0) return true;
//          else return false;
//       }

         /// Evaluate a Matlab instruction. If error happens while executing, returns false.
         bool Eval(std::string mstring) {
            if (engEvalString(ep, mstring.c_str()) == 0) return true;
            else return false;
         }

         /// Set visibility of GUI matlab window.
         bool SetVisible(bool mvis) {
            if (engSetVisible(ep, mvis) == 0) return true;
            else return false;
         }

         /// Put a matrix in Matlab environment, specifying its name as variable.
         /// If a variable with the same name already exist, it is overwritten.
         bool PutVariable(const ChMatrix<double>& mmatr,std::string varname) {
            ChMatrixDynamic<> transfer;     // elements in Matlab are column-major
            transfer.CopyFromMatrixT(mmatr);

            mxArray *T = NULL;
            T = mxCreateDoubleMatrix(mmatr.GetRows(), mmatr.GetColumns(), mxREAL);
            memcpy((char *) mxGetPr(T), (char *) transfer.GetAddress(), mmatr.GetRows() * mmatr.GetColumns() * sizeof(double));
            engPutVariable(ep, varname.c_str(), T);
            mxDestroyArray(T);
            return true;
         }

         /// Put a sparse matrix in Matlab environment, specifying its name as variable.
         /// If a variable with the same name already exist, it is overwritten.
         bool PutSparseMatrix(const ChSparseMatrix& mmatr, std::string varname) {
            int nels = 0;
            for (int ii = 0; ii < mmatr.GetRows(); ii++)
               for (int jj = 0; jj < mmatr.GetColumns(); jj++) {
                  double elVal = ((ChSparseMatrix&) mmatr).GetElement(ii, jj);
                  if (elVal || (ii + 1 == ((ChSparseMatrix&) mmatr).GetRows() && jj + 1 == ((ChSparseMatrix&) mmatr).GetColumns())) ++nels;
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
            sprintf(buff, "%s=spconvert(%s)", varname.c_str(), varname.c_str());
            this->Eval(buff);

            return true;
         }

         /// Fetch a matrix from Matlab environment, specifying its name as variable.
         /// The used matrix must be of ChMatrixDynamic<double> type because
         /// it might undergo resizing.
         bool GetVariable(ChMatrixDynamic<double>& mmatr, std::string varname) {
            ChMatrixDynamic<> transfer;     // elements in Matlab are column-major

            mxArray* T = engGetVariable(ep, varname.c_str());
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

   };

   class ChLcpMatlabSolver: public ChLcpIterativeSolver {
      protected:
         //
         // DATA
         //
         ChMatlabEngine* mengine;
      public:
         //
         // CONSTRUCTORS
         //

         ChLcpMatlabSolver(ChMatlabEngine& me) {
            mengine = &me;
         }
         ;

         virtual ~ChLcpMatlabSolver() {
         }
         ;

         /// Solve using the Matlab default direct solver (as in x=A\b)

         virtual double Solve(ChLcpSystemDescriptor& sysd      ///< system description with constraints and variables
               ) {
            std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
            std::vector<ChLcpVariables*>& mvariables = sysd.GetVariablesList();

            for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
               mconstraints[ic]->Update_auxiliary();
            }




            if (mconstraints.size() > 0) {




               chrono::ChSparseMatrix mdM;
               chrono::ChSparseMatrix mdCq;
               chrono::ChSparseMatrix mdE;
               chrono::ChMatrixDynamic<double> mdf;
               chrono::ChMatrixDynamic<double> mdb;
               chrono::ChMatrixDynamic<double> mdfric;
               ChMatrixDynamic<> mq;
               sysd.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);
               sysd.FromVariablesToVector(mq, true);

               mengine->PutSparseMatrix(mdM, "M");
               mengine->PutSparseMatrix(mdCq, "D");
               mengine->PutSparseMatrix(mdE, "E");
               mengine->PutVariable(mdf, "fh");
               mengine->PutVariable(mdb, "c");
               mengine->PutVariable(mdfric, "fric");
               mengine->PutVariable(mq, "v0");
               //mengine->Eval("mdZ = [mdM, mdCq'; mdCq, -mdE]; mdd=[mdf;-mdb];");

               //mengine->Eval("mdx = mldivide(mdZ , mdd);");
               mengine->Eval("residual = 1;");
               mengine->Eval("time = 0;");
               mengine->Eval("objective = 1;");
               mengine->Eval("iterations = 50");
               mengine->Eval("SIZE = size(c);");
               mengine->Eval("x0=zeros(SIZE);");
               mengine->Eval("D=-D';");
               mengine->Eval("h=.01;");
               mengine->Eval("k=fh;");
               mengine->Eval("fh=k-M*v0;");
               mengine->Eval("Minv = inv(M);");

               mengine->Eval("rungams;");
               //mengine->Eval("[~,gamma,residual,objective,~,~,~,~]   = solver_APGD   (M,-D,k,c,E,iterations,fric,x0);");

               chrono::ChMatrixDynamic<> mx;
               if (!mengine->GetVariable(mx, "gamma")) GetLog() << "ERROR!! cannot fetch gamma";


               sysd.FromVectorToConstraints(mx);

               chrono::ChMatrixDynamic<> mres;
               mengine->GetVariable(mres, "residual");
               chrono::ChMatrixDynamic<> mobj;
               mengine->GetVariable(mobj, "objective");

               chrono::ChMatrixDynamic<> miters;
               mengine->GetVariable(miters, "iterations");

               chrono::ChMatrixDynamic<> mtime;
               mengine->GetVariable(mtime, "time");

               //GetLog() << " [residual, objective]:  [" << mres(0, 0) << ",  " << mobj(0, 0) << "] " << miters(0, 0) << " " << mtime(0, 0) << " s\n";

            }
            chrono::ChMatrixDynamic<> mq;
            if (!mengine->GetVariable(mq, "deltav")) GetLog() << "ERROR!! cannot fetch gamma";
            sysd.FromVectorToVariables(mq);

//
//          for (unsigned int iv = 0; iv < mvariables.size(); iv++) {
//             if (mvariables[iv]->IsActive()) {
//                mvariables[iv]->Compute_invMb_v(mvariables[iv]->Get_qb(), mvariables[iv]->Get_fb());     // q = [M]'*fb
//             }
//          }


//          for (unsigned int ic = 0; ic < mconstraints.size(); ic++) {
//             if (mconstraints[ic]->IsActive()) {
//                mconstraints[ic]->Increment_q(mconstraints[ic]->Get_l_i());
//             }
//          }

            return 0;

         }
   };

}     // END_OF_NAMESPACE____

#endif
 *
 */
