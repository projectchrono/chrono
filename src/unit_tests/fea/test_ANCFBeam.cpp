// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Antonio Recuero
// =============================================================================
//
// Unit test for ANCF gradient-deficient beam element
//
// Successful execution of this unit test may validate: this element's internal
// force, distributed gravity, inertia, and numerical integration implementations.
//
// The reference file data was validated by digitizing Figs. 4 and 5 of the paper
// Gerstmayr and Shabana, 2006, "Analysis of Thin Beams and Cables Using the Absolute
// Nodal Co-ordinate Formulation", Nonlinear Dynamics, 45: 109�130.
//
// Special attention must be paid to the number of Gauss points for gravity. For
// successful verification, this number must be 2.
// =============================================================================
#include <cstdio>

#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/lcp/ChLcpIterativePMINRES.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"
#include "chrono_fea/ChLoadsBeam.h"
#include "physics/ChLoadContainer.h"
#include "chrono_fea/ChElementBeamANCF.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/core/ChVector.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"

using namespace chrono;
using namespace chrono::fea;

const double precision = 1e-6;

int main(int argc, char* argv[]) {
    // Utils to open/read files: Load reference solution ("golden") file
    ChMatrixDynamic<> FileInputMat(20000, 7);
    std::string beam_validation_file = GetChronoDataPath() + "testing/" + "UT_ANCFBeam.txt";
    std::ifstream fileMid(beam_validation_file);

    if (!fileMid.is_open()) {
        fileMid.open(beam_validation_file);
    }
    if (!fileMid) {
        std::cout << "Cannot open file.\n";
        exit(1);
    }
    for (int x = 0; x < 20000; x++) {
        fileMid >> FileInputMat[x][0] >> FileInputMat[x][1] >> FileInputMat[x][2] >> FileInputMat[x][3] >>
            FileInputMat[x][4] >> FileInputMat[x][5] >> FileInputMat[x][6];
    }
    fileMid.close();

    // Create a Chrono::Engine physical system
    ChSystem my_system;
    unsigned int num_steps = 200;
    utils::Data m_data;  // Matrices to store datac

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    ChSharedPtr<ChMesh> my_mesh(new ChMesh);

    const double f_const = 5.0;  // Gerstmayr's paper's parameter
    double diam = 0.0;
    const double Ang_VelY = 4.0;
    const double beam_length = 1.0;
    unsigned int NElem = 4;
    double rho = 0.0;

    ChSharedPtr<ChBeamSectionCable> msection_cable(new ChBeamSectionCable);
    diam = sqrt(1e-6 / CH_C_PI) * 2.0 * f_const;
    msection_cable->SetDiameter(diam);
    msection_cable->SetYoungModulus(1e9 / pow(f_const, 4));
    msection_cable->SetBeamRaleyghDamping(0.000);
    msection_cable->SetI(CH_C_PI / 4.0 * pow(diam / 2, 4));
    rho = 8000 / pow(f_const, 2);
    msection_cable->SetDensity(rho);

    // Create the nodes
    ChSharedPtr<ChNodeFEAxyzD> hnodeancf1(new ChNodeFEAxyzD(ChVector<>(0, 0, 0.0), ChVector<>(1, 0, 0)));
    ChSharedPtr<ChNodeFEAxyzD> hnodeancf2(new ChNodeFEAxyzD(ChVector<>(beam_length / 4, 0, 0), ChVector<>(1, 0, 0)));
    ChSharedPtr<ChNodeFEAxyzD> hnodeancf3(new ChNodeFEAxyzD(ChVector<>(beam_length / 2, 0, 0), ChVector<>(1, 0, 0)));
    ChSharedPtr<ChNodeFEAxyzD> hnodeancf4(
        new ChNodeFEAxyzD(ChVector<>(3.0 * beam_length / 4, 0, 0), ChVector<>(1, 0, 0)));
    ChSharedPtr<ChNodeFEAxyzD> hnodeancf5(new ChNodeFEAxyzD(ChVector<>(beam_length, 0, 0), ChVector<>(1, 0, 0)));

    my_mesh->AddNode(hnodeancf1);
    my_mesh->AddNode(hnodeancf2);
    my_mesh->AddNode(hnodeancf3);
    my_mesh->AddNode(hnodeancf4);
    my_mesh->AddNode(hnodeancf5);

    // Create the element 1
    ChSharedPtr<ChElementBeamANCF> belementancf1(new ChElementBeamANCF);
    belementancf1->SetNodes(hnodeancf1, hnodeancf2);
    belementancf1->SetSection(msection_cable);
    my_mesh->AddElement(belementancf1);

    // Create the element 2
    ChSharedPtr<ChElementBeamANCF> belementancf2(new ChElementBeamANCF);
    belementancf2->SetNodes(hnodeancf2, hnodeancf3);
    belementancf2->SetSection(msection_cable);
    my_mesh->AddElement(belementancf2);

    // Create the element 3
    ChSharedPtr<ChElementBeamANCF> belementancf3(new ChElementBeamANCF);
    belementancf3->SetNodes(hnodeancf3, hnodeancf4);
    belementancf3->SetSection(msection_cable);
    my_mesh->AddElement(belementancf3);

    // Create the element 4
    ChSharedPtr<ChElementBeamANCF> belementancf4(new ChElementBeamANCF);
    belementancf4->SetNodes(hnodeancf4, hnodeancf5);
    belementancf4->SetSection(msection_cable);
    my_mesh->AddElement(belementancf4);
    ChSharedPtr<ChBody> mtruss(new ChBody);
    mtruss->SetBodyFixed(true);

    ChSharedPtr<ChLinkPointFrame> constraint_hinge(new ChLinkPointFrame);
    constraint_hinge->Initialize(hnodeancf1, mtruss);
    my_system.Add(constraint_hinge);
    my_mesh->SetupInitial();

    // Cancel automatic gravity
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Add external forces and initial conditions
    // Angular velocity initial condition
    hnodeancf1->SetPos_dt(ChVector<>(0, 0, -Ang_VelY * beam_length / NElem * 0.0));
    hnodeancf2->SetPos_dt(ChVector<>(0, 0, -Ang_VelY * beam_length / NElem * 1.0));
    hnodeancf3->SetPos_dt(ChVector<>(0, 0, -Ang_VelY * beam_length / NElem * 2.0));
    hnodeancf4->SetPos_dt(ChVector<>(0, 0, -Ang_VelY * beam_length / NElem * 3.0));
    hnodeancf5->SetPos_dt(ChVector<>(0, 0, -Ang_VelY * beam_length / NElem * 4.0));

    // First: loads must be added to "load containers",
    // and load containers must be added to your ChSystem
    ChSharedPtr<ChLoadContainer> mloadcontainer(new ChLoadContainer);
    my_system.Add(mloadcontainer);

    // Add gravity (constant volumetric load): Use 2 Gauss integration points

    ChSharedPtr<ChLoad<ChLoaderGravity> > mgravity1(new ChLoad<ChLoaderGravity>(belementancf1));
    mgravity1->loader.SetNumIntPoints(2);
    mloadcontainer->Add(mgravity1);
    ChSharedPtr<ChLoad<ChLoaderGravity> > mgravity2(new ChLoad<ChLoaderGravity>(belementancf2));
    mgravity2->loader.SetNumIntPoints(2);
    mloadcontainer->Add(mgravity2);
    ChSharedPtr<ChLoad<ChLoaderGravity> > mgravity3(new ChLoad<ChLoaderGravity>(belementancf3));
    mgravity3->loader.SetNumIntPoints(2);
    mloadcontainer->Add(mgravity3);
    ChSharedPtr<ChLoad<ChLoaderGravity> > mgravity4(new ChLoad<ChLoaderGravity>(belementancf4));
    mgravity4->loader.SetNumIntPoints(2);
    mloadcontainer->Add(mgravity4);

    // Change solver settings
    my_system.SetLcpSolverType(
        ChSystem::LCP_ITERATIVE_MINRES);  // <- NEEDED THIS OR ::LCP_SIMPLEX because other LCP_ITERATIVE_MINRES
    // solvers can't handle stiffness matrices
    my_system.SetIterLCPwarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetIterLCPmaxItersSpeed(200);
    my_system.SetIterLCPmaxItersStab(200);
    my_system.SetTolForce(1e-14);
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

    // Change type of integrator:
    my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    //  my_system.SetIntegrationType(chrono::ChSystem::INT_HHT);  // precise,slower, might iterate each step
    my_system.SetEndTime(12.5);
    // if later you want to change integrator settings:
    if (ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>()) {
        mystepper->SetAlpha(0.0);
        mystepper->SetMaxiters(60);
        mystepper->SetTolerance(1e-14);
    }

    /* m_data.resize(7);
     utils::CSV_writer csv(" ");
     std::ifstream file2("UT_ANCFBeam.txt");

     for (size_t col = 0; col < 7; col++)
         m_data[col].resize(num_steps);*/

    for (unsigned int it = 0; it < num_steps; it++) {
        my_system.DoStepDynamics(0.0001);
        std::cout << "Time t = " << my_system.GetChTime() << "s \n";
        // Checking midpoint and tip Y displacement
        double AbsVal = abs(hnodeancf3->GetPos().y - FileInputMat[it][4]);
        double AbsVal2 = abs(hnodeancf5->GetPos().z - FileInputMat[it][6]);
        AbsVal = ChMax(AbsVal, AbsVal2);

        if (AbsVal > precision) {
            std::cout << "Unit test check failed \n";
            return 1;
        }
    }
    std::cout << "Unit test check succeeded \n";
    /*
    // This code snippet creates the benchmark file.
    m_data[0][it] = my_system.GetChTime();
    m_data[1][it] = hnodeancf1->GetD().x,
    m_data[2][it] = hnodeancf1->GetD().y;
    m_data[3][it] = hnodeancf1->GetD().z;
    m_data[4][it] = hnodeancf3->GetPos().y;
    m_data[5][it] = hnodeancf5->GetPos().x;
    m_data[6][it] = hnodeancf5->GetPos().z;
    csv << m_data[0][it] << m_data[1][it] << m_data[2][it] << m_data[3][it] << m_data[4][it] << m_data[5][it] <<
    m_data[6][it] << std::endl;
    // Advance system state
    std::cout << "Time t = " << my_system.GetChTime() << "s \n";
    csv.write_to_file("UT_ANCFBeam.txt"); */

    return 0;
}
