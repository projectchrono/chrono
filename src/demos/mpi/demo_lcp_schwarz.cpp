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
//   Demo code about
//     - LCP solver with MPI cluster computing
// =============================================================================

// Include some headers used by this tutorial...

#include "lcp/ChVariablesGeneric.h"
#include "lcp/ChVariablesBody.h"
#include "lcp/ChConstraintTwoGeneric.h"
#include "core/ChLinearAlgebra.h"
#include "unit_MPI/ChMpi.h"
#include "unit_MPI/ChIterativeSchwarzMPI.h"
#include "unit_MPI/ChSystemDescriptorMPI.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;

// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[]) {
    // Initialize the MPI functionality. Use the CHMPI static functions.
    CHMPI::Init(argc, argv);

    GetLog() << " Example: use MPI for a multi-domain LCP solver \n\n\n";

    // Get infos about how many processes are launched,
    // and about the ID of this specific process.
    int numprocs = CHMPI::CommSize();
    int myid = CHMPI::CommRank();

    if (numprocs != 2) {
        if (myid == 0) {
            GetLog() << "ERROR: you must use 2 processes! \n";
            GetLog() << "       Note that this demo must be launched only \n"
                     << "       using the 'mpiexec', from the MPI toolset! \n";
        }
        CHMPI::Finalize();
        return 0;
    }

    // Use the following 'MPI-specialized' LCP system descriptor instead of
    // the normal ChSystemDescriptor:

    ChSystemDescriptorMPI mdescriptor;

    // We want that last variable of 1st domain is shared with
    // the first variable of the 2nd domain, before BeginInsertion().
    // First, tell that there is an overlapping Schwarz interface per
    // each domain:
    ChSharedInterfaceMPI minterface;
    if (myid == 0)
        minterface.SetMPIfriend(1);
    if (myid == 1)
        minterface.SetMPIfriend(0);
    mdescriptor.GetSharedInterfacesList().push_back(minterface);

    // Create a bunch of monodimensional vars and simple
    // constraints between them, using 'for' loops, just for test.
    // These are the same in two domains, like two chains of bodies.

    mdescriptor.BeginInsertion();  // ----- system description starts here

    int n_masses = 11;

    std::vector<ChVariablesGeneric*> vars;
    std::vector<ChConstraintTwoGeneric*> constraints;

    for (int im = 0; im < n_masses; im++) {
        vars.push_back(new ChVariablesGeneric(1));
        vars[im]->GetMass()(0) = 10;
        vars[im]->GetInvMass()(0) = 1. / vars[im]->GetMass()(0);
        vars[im]->Get_fb()(0) = -9.8 * vars[im]->GetMass()(0) * 0.01;
        mdescriptor.InsertVariables(vars[im]);
        if (im > 0) {
            constraints.push_back(new ChConstraintTwoGeneric(vars[im], vars[im - 1]));
            constraints[im - 1]->Set_b_i(0);
            constraints[im - 1]->Get_Cq_a()->ElementN(0) = 1;
            constraints[im - 1]->Get_Cq_b()->ElementN(0) = -1;
            mdescriptor.InsertConstraint(constraints[im - 1]);
        }
    }

    // First variable of 1st domain is 'fixed'
    if (myid == 0) {
        vars[0]->SetDisabled(true);
    }

    // Last variable of 1st domain is shared with 1st variable of 2nd domain:
    if (myid == 0) {
        ChSharedVarMPI msh;
        msh.var = vars.back();  // vars[10];
        msh.uniqueID = 1012;    // must be unique! (important if many shares in single interface)
        mdescriptor.GetSharedInterfacesList()[0].InsertSharedVariable(msh);
    }
    if (myid == 1) {
        ChSharedVarMPI msh;
        msh.var = vars[0];
        msh.uniqueID = 1014;  // must be unique! (important if many shares in single interface)
        mdescriptor.GetSharedInterfacesList()[0].InsertSharedVariable(msh);
    }

    mdescriptor.EndInsertion();  // ----- system description is finished

    /*
        ChSparseMatrix matrM;
        ChSparseMatrix matrCq;
        mdescriptor.ConvertToMatrixForm(&matrCq, &matrM, 0, 0, 0, 0, false, false);
        try
        {
            ChStreamOutAsciiFile fileM ("sch_dump_M.dat");
            ChStreamOutAsciiFile fileCq ("sch_dump_Cq.dat");
            matrM.StreamOUTsparseMatlabFormat(fileM);
            matrCq.StreamOUTsparseMatlabFormat(fileCq);
        }
        catch (ChException myex)
        {
            GetLog() << "FILE ERROR: " << myex.what();
        }
    */

    // Solve the problem with an iterative solver, for an
    // approximate (but very fast) solution:
    //
    // .. create the solver
    ChIterativeSchwarzMPI msolver_iter(20,     // outer iterations (updates between domains)
                                          480,    // inner iterations
                                          false,  // don't use warm start
                                          0.0,    // termination tolerance
                                          1.0     // omega
                                          );

    // .. pass the constraint and the variables to the solver
    //    to solve - that's all.
    msolver_iter.Solve(mdescriptor);

    // Ok, now present the result to the user, with some
    // statistical information:
    double max_res, max_LCPerr;
    mdescriptor.ComputeFeasabilityViolation(max_res, max_LCPerr);

    // Other checks

    GetLog() << "ID=" << CHMPI::CommRank();
    GetLog() << " METRICS: max residual: " << max_res << "  max LCP error: " << max_LCPerr << "  \n\n";
    GetLog() << "ID=" << CHMPI::CommRank();
    GetLog() << "  Primal vars: (qb) ---------------\n";
    for (int im = 0; im < n_masses; im++) {
        GetLog() << "   " << vars[im]->Get_qb()(0) << "\n";
    }
    GetLog() << "\nID=" << CHMPI::CommRank();
    GetLog() << "  Dual vars: (constr.lambdas) -----\n\n";
    for (int im = 0; im < n_masses - 1; im++) {
        GetLog() << "   " << constraints[im]->Get_l_i() << "\n";  // << "  res: " << constraints[im]->Get_c_i() << "\n";
    }

    // Terminate the MPI functionality.
    CHMPI::Finalize();

    return 0;
}
