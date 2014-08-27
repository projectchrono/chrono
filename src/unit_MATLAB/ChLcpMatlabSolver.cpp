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


#include "ChLcpMatlabSolver.h"


namespace chrono {

ChLcpMatlabSolver::ChLcpMatlabSolver(ChMatlabEngine& me)
{
  mengine = &me;
}

// Solve using the Matlab default direct solver (as in x=A\b)
double ChLcpMatlabSolver::Solve(ChLcpSystemDescriptor& sysd)
{
  chrono::ChSparseMatrix mdM;
  chrono::ChSparseMatrix mdCq;
  chrono::ChSparseMatrix mdE;
  chrono::ChMatrixDynamic<double> mdf;
  chrono::ChMatrixDynamic<double> mdb;
  chrono::ChMatrixDynamic<double> mdfric;
  sysd.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);

  mengine->PutSparseMatrix(mdM, "mdM");
  mengine->PutSparseMatrix(mdCq, "mdCq");
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
  GetLog() << " Matlab computed residual:" << mres(0, 0) << "\n";

  return 0;
}


} // END_OF_NAMESPACE____


