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

#include "ChLcpMklSolver.h"

namespace chrono {

ChLcpMklSolver::ChLcpMklSolver(ChMklEngine& me) {
    mklengine = &me;
}

// Solve using the Mkl default direct solver (as in x=A\b)
double ChLcpMklSolver::Solve(ChLcpSystemDescriptor& sysd) {
    chrono::ChSparseMatrix mdM;
    chrono::ChSparseMatrix mdCq;
    chrono::ChSparseMatrix mdE;
    chrono::ChMatrixDynamic<double> mdf;
    chrono::ChMatrixDynamic<double> mdb;
    chrono::ChMatrixDynamic<double> mdfric;
    sysd.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);

	mklengine->SetMatrix(&mdCq, &mdM, &mdE, &mdf, &mdb); //inside: conversion to CSR3 matrix
	mklengine->PardisoSolve();

	// la soluzione del sistema viene salvata in sysd
    chrono::ChMatrixDynamic<> mx;
	mklengine->GetSolution(&mx);
	sysd.FromVectorToUnknowns(mx);

	// il residuo viene salvato nel log ma a quanto pare non in sysd
	mklengine->GetResidual();
    GetLog() << " Mkl computed residual:" << mres(0, 0) << "\n";

    return 0;
}




}  // END_OF_NAMESPACE____
