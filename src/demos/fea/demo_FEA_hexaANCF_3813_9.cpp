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
// Authors: Bryan Peterson, Antonio Recuero
// =============================================================================
//
// Demo for 9-node, large deformation brick element
// The user can run seven demos in the main function. 1) Axial dynamics excites
// a beam made up of brick elements axially; 2) BendingQuasiStatic applies a qua-
// sistatic load at a corner of a plate and is used for convergence verification;
// 3) Swinging shell is used for verification of dynamics, rigid body and large
// deformation problems, 4) ShellBrickContact is used to visualize contact between
//  ANCF shell elements and 9-node bricks, 5) SimpleBoxContact is intended to
// visualize contact between rigid bodies and bricks. 6) DPCapPress is a soil bin
// scenario in which a set of nodal forces is applied to validate the Drucker-
// Prager Cap Model.
// The user can uncomment and run any of the seven demos
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"

#include "chrono/fea/ChElementHexaANCF_3813_9.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChContactSurfaceMesh.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

// -----------------------------------------------------------------------------

void AxialDynamics(const std::string& out_dir);
void BendingQuasiStatic(const std::string& out_dir);
void SwingingShell(const std::string& out_dir);
void SoilBin(const std::string& out_dir);
void SimpleBoxContact(const std::string& out_dir);
void ShellBrickContact(const std::string& out_dir);
void DPCapPress(const std::string& out_dir);

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create (if needed) the output directory
    const std::string out_dir = GetChronoOutputPath() + "FEA_BRICK9";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // DPCapPress(out_dir);
    // ShellBrickContact(out_dir);
    // SimpleBoxContact(out_dir);
    // SoilBin(out_dir);
    // AxialDynamics(out_dir);
    // BendingQuasiStatic(out_dir);
    SwingingShell(out_dir);

    return 0;
}

// Soil Bin case testing Drucker-Prager Cap model
void DPCapPress(const std::string& out_dir) {
    FILE* outputfile;
    ChSystemSMC sys;
    sys.UseMaterialProperties(false);
    sys.SetAdhesionForceModel(ChSystemSMC::AdhesionForceModel::Constant);
    // sys.SetContactForceModel(ChSystemSMC::ContactForceModel::PlainCoulomb);
    sys.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "     9-Node, Large Deformation Brick Element with implicit integration " << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 0.48;  // 0.4;
    double plate_lenght_y = 0.48;  // 0.4;
    double plate_lenght_z = 0.6;   // 0.6;

    // Specification of the mesh
    int numDiv_x = 12;  // 10;
    int numDiv_y = 12;  // 10;
    int numDiv_z = 8;   // 8;

    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y * numDiv_z;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    bool Plasticity = true;
    double timestep = 1e-4;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;
            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (j == 0) {
                node->SetFixed(true);
            }
            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Create an orthotropic material.
    double rho = 2149.0;
    ChVector3d E(54.1e6, 54.1e6, 54.1e6);         // (1.379e7, 1.379e7, 1.379e7);
    ChVector3d nu(0.293021, 0.293021, 0.293021);  // (0.3, 0.3, 0.3);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());

    // Read hardening parameter look-up table
    FILE* inputfile;
    char str1[100];
    int MAXCOUNT = 100;
    int RowN;

    // inputfile = fopen(GetChronoDataFile("fea/Hardening_parameter_table.INP").c_str(), "r");
    inputfile = fopen(GetChronoDataFile("fea/CapHardeningInformation_TriaxialAxial.INP").c_str(), "r");
    if (inputfile == NULL) {
        printf("Input data file not found!!\n");
        exit(1);
    }
    fgets(str1, MAXCOUNT, inputfile);
    printf("%s\n", str1);

    fscanf(inputfile, "%d\n", &RowN);
    ChVectorDynamic<double> m_DPVector1(RowN);
    ChVectorDynamic<double> m_DPVector2(RowN);
    for (int i = 0; i < RowN; i++) {
        fscanf(inputfile, " %lf %lf\n", &m_DPVector1(i), &m_DPVector2(i));
    }
    // Modified hardening parameter to be consistent with ABAQUS !!!
    for (int i = 0; i < RowN; i++) {
        m_DPVector2(i) = (m_DPVector2(i) + 210926.0 / std::tan(51.7848 * 3.141592653589793 / 180.0)) /
                         (0.5 * std::tan(51.7848 * 3.141592653589793 / 180.0) + 1.0);
    }

    std::shared_ptr<ChContactMaterialSMC> my_surfacematerial(new ChContactMaterialSMC);
    my_surfacematerial->SetKn(3200);  //(10e6);
    my_surfacematerial->SetKt(3200);  //(10e6);
    my_surfacematerial->SetGn(32);    // (10e3);
    my_surfacematerial->SetGt(32);    // (10e3);

    auto my_contactsurface = chrono_types::make_shared<ChContactSurfaceNodeCloud>(my_surfacematerial);
    my_contactsurface->AddAllNodes(*my_mesh, 0.005);
    my_mesh->AddContactSurface(my_contactsurface);

    ChMatrixNM<double, 9, 8> CCPInitial;
    for (int k = 0; k < 8; k++) {
        CCPInitial(0, k) = 1;
        CCPInitial(4, k) = 1;
        CCPInitial(8, k) = 1;
    }

    auto strain_formulation = ChElementHexaANCF_3813_9::StrainFormulation::Hencky;
    auto plasticity_formulation = ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager_Cap;

    // Create the elements
    int jj = -1;
    int kk = -1;
    for (int i = 0; i < TotalNumElements; i++) {
        if (i % (numDiv_x * numDiv_y) == 0) {
            jj++;
            kk = 0;
        }
        // Adjacent nodes
        int node0 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + jj * (N_x * N_y);
        int node1 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + jj * (N_x * N_y);
        int node2 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + N_x + jj * (N_x * N_y);
        int node3 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + N_x + jj * (N_x * N_y);
        int node4 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + XYNumNodes + jj * (N_x * N_y);
        int node5 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + XYNumNodes + jj * (N_x * N_y);
        int node6 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + N_x + XYNumNodes + jj * (N_x * N_y);
        int node7 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + N_x + XYNumNodes + jj * (N_x * N_y);
        int node8 = (numDiv_z + 1) * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.0);     // Structural damping for this element
        element->SetDPIterationNo(50);  // Set maximum number of iterations for Drucker-Prager Newton-Raphson
        element->SetDPYieldTol(1e-5);   // Set stop tolerance for Drucker-Prager Newton-Raphson
        element->SetStrainFormulation(strain_formulation);
        element->SetPlasticityFormulation(plasticity_formulation);
        if (strain_formulation == ChElementHexaANCF_3813_9::StrainFormulation::Hencky) {
            element->SetPlasticity(Plasticity);
            if (Plasticity) {
                element->SetYieldStress(210926.0);
                element->SetHardeningSlope(0.0);
                element->SetCCPInitial(CCPInitial);
                if (plasticity_formulation == ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager) {
                    element->SetFriction(10.0);
                    element->SetDilatancy(0.0);
                    element->SetDPType(3);
                } else if (plasticity_formulation ==
                           ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager_Cap) {
                    element->SetFriction(51.7848);
                    element->SetDilatancy(51.7848);
                    element->SetDPType(3);
                    element->SetDPVector1(m_DPVector1);
                    element->SetDPVector2(m_DPVector2);
                    element->SetDPVectorSize(RowN);
                    element->SetDPCapBeta(0.5 * std::tan(51.7848 * 3.141592653589793 / 180.0));
                }
            }
        }

        // Add element to mesh
        my_mesh->AddElement(element);
        kk++;
    }

    // Add the mesh to the system
    sys.Add(my_mesh);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    auto mvisualizemeshref = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshref->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddVisualShapeFEA(mvisualizemeshref);

    auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddVisualShapeFEA(mvisualizemeshC);

    auto mvisualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizemeshD);

    auto mvisualizemeshcoll = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshcoll->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddVisualShapeFEA(mvisualizemeshcoll);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "9-Node, Large Deformation Brick Element",
                                         ChVector3d(-0.4, -0.3, 0.0), ChVector3d(0.0, 0.5, -0.1));

    // Use the MKL Solver
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
    mkl_solver->LockSparsityPattern(true);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxIters(25);
    mystepper->SetAbsTolerances(1e-4, 1e-2);
    mystepper->SetVerbose(true);

    sys.Update();

    std::string filename = out_dir + "/DPCapPress.txt";
    outputfile = fopen(filename.c_str(), "w");

    double start = std::clock();
    int Iter = 0;

    double force = 0.0;

    while (vis->Run() && (sys.GetChTime() <= 0.5)) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);

        int offset_top = N_x * N_y * numDiv_z;
        int offset_mid = (numDiv_y / 2 - 1) * N_x;
        int inc = 0;
        // node force
        force = -1700 * std::sin(sys.GetChTime() * CH_PI);
        for (inc = 0; inc < numDiv_x / 4; inc++) {
            for (int ii = 0; ii < numDiv_x / 2 + 1; ii++) {
                auto nodeforce = std::dynamic_pointer_cast<ChNodeFEAxyz>(
                    my_mesh->GetNode(offset_top + offset_mid + N_y * inc + numDiv_x / 4 + ii));
                nodeforce->SetForce(ChVector3d(0.0, 0.0, force));
            }
        }

        Iter += mystepper->GetNumIterations();
        std::cout << "t = " << sys.GetChTime() << std::endl;
        std::cout << "Last it: " << mystepper->GetNumIterations() << std::endl;

        fprintf(outputfile, "%15.7e  ", sys.GetChTime());
        inc = inc / 2;
        for (int ii = 0; ii < N_x; ii++) {
            auto nodeforce =
                std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(offset_top + offset_mid + N_y * inc + ii));
            fprintf(outputfile, "%15.7e  ", nodeforce->GetPos().x());
            fprintf(outputfile, "%15.7e  ", nodeforce->GetPos().y());
            fprintf(outputfile, "%15.7e  ", nodeforce->GetPos().z());
        }
        fprintf(outputfile, "\n  ");
    }

    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "Simulation Time: " << duration << std::endl;
    std::cout << "Force Time: " << my_mesh->GetTimeInternalForces() << std::endl;
    std::cout << "Jacobian Time: " << my_mesh->GetTimeJacobianLoad() << std::endl;
    std::cout << "Solver Time: " << sys.GetTimerLSsolve() << std::endl;
    std::cout << Iter << std::endl;
}

// Test1 Case
void ShellBrickContact(const std::string& out_dir) {
    FILE* outputfile;
    ChSystemSMC sys;
    sys.UseMaterialProperties(false);
    sys.SetAdhesionForceModel(ChSystemSMC::AdhesionForceModel::Constant);
    sys.SetContactForceModel(ChSystemSMC::ContactForceModel::PlainCoulomb);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "     9-Node, Large Deformation Brick Element with implicit integration " << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    // Create a separate mesh for the shell elements so that gravity can be disabled only for the shells
    auto my_shell_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the bricked plate.
    double plate_lenght_x = 0.5;
    double plate_lenght_y = 0.5;
    double plate_lenght_z = 0.125;

    // Geometry of the ANCF shell.
    double shell_lenght_x = 0.1;
    double shell_lenght_y = 0.1;

    // Specification of the mesh for bricked plate.
    int numDiv_x = 8;
    int numDiv_y = 8;
    int numDiv_z = 2;

    // Specification of the mesh for ANCF shell.
    int SnumDiv_x = 4;
    int SnumDiv_y = 4;

    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;

    int SN_x = SnumDiv_x + 1;

    // Number of elements in the z direction is considered as 1.
    int TotalNumElements = numDiv_x * numDiv_y * numDiv_z;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    ////int TotalNumNodes = (numDiv_z + 1) * XYNumNodes + TotalNumElements;

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    double Sdx = shell_lenght_x / SnumDiv_x;
    double Sdy = shell_lenght_y / SnumDiv_y;

    bool Plasticity = true;
    double timestep = 1e-4;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;
            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (j == 0) {
                node->SetFixed(true);
            }
            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    for (int k = 0; k < 25; k++) {  // Create a 5 by 5 flat ANCF shell
        // Node location
        double loc_x = (k % (SnumDiv_x + 1)) * Sdx + 0.15;
        double loc_y = (k / (SnumDiv_x + 1)) % (SnumDiv_y + 1) * Sdy + 0.15;
        double loc_z = 0.13;
        auto nodeshell =
            chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(loc_x, loc_y, loc_z), ChVector3d(0.0, 0.0, 1.0));
        nodeshell->SetMass(0);
        my_shell_mesh->AddNode(nodeshell);
    }

    // Get a handle to the tip node.
    auto nodetip1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode((numDiv_z + 1) * XYNumNodes - XYNumNodes));
    // Create an orthotropic material.
    double rho = 200.0;
    ChVector3d E(1.379e7, 1.379e7, 1.379e7);
    ChVector3d nu(0.3, 0.3, 0.3);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());
    double rhoS = 8000;
    ChVector3d ES(2.1e10, 2.1e10, 2.1e10);                 // Modulus of elasticity
    ChVector3d nuS(0.3, 0.3, 0.3);                         // Poisson ratio
    ChVector3d GS(8.0769231e9, 8.0769231e9, 8.0769231e9);  // Modulus of rigidity
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rhoS, ES, nuS, GS);
    std::shared_ptr<ChContactMaterialSMC> my_surfacematerial(new ChContactMaterialSMC);
    my_surfacematerial->SetKn(3e6f);
    my_surfacematerial->SetKt(3e6f);
    my_surfacematerial->SetGn(3e3f);
    my_surfacematerial->SetGt(3e3f);

    // Set reference initial strain tensor for plasticity (initial state, undeformed).
    ChMatrixNM<double, 9, 8> CCPInitial;
    for (int k = 0; k < 8; k++) {
        CCPInitial(0, k) = 1;
        CCPInitial(4, k) = 1;
        CCPInitial(8, k) = 1;
    }

    auto strain_formulation = ChElementHexaANCF_3813_9::StrainFormulation::Hencky;
    auto plasticity_formulation = ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager;

    // Create the elements for the bricked plate (made up of 9-node brick elements).
    int jj = -1;
    int kk = 0;
    for (int i = 0; i < TotalNumElements; i++) {
        if (i % (numDiv_x * numDiv_y) == 0) {
            jj++;
            kk = 0;
        }
        // Adjacent nodes
        int node0 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + jj * (N_x * N_y);
        int node1 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + jj * (N_x * N_y);
        int node2 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + N_x + jj * (N_x * N_y);
        int node3 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + N_x + jj * (N_x * N_y);
        int node4 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + XYNumNodes + jj * (N_x * N_y);
        int node5 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + XYNumNodes + jj * (N_x * N_y);
        int node6 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + N_x + XYNumNodes + jj * (N_x * N_y);
        int node7 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + N_x + XYNumNodes + jj * (N_x * N_y);
        int node8 = (numDiv_z + 1) * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.0);     // Structural damping for this element
        element->SetDPIterationNo(50);  // Set maximum number of iterations for Drucker-Prager Newton-Raphson
        element->SetDPYieldTol(1e-8);   // Set stop tolerance for Drucker-Prager Newton-Raphson
        element->SetStrainFormulation(strain_formulation);
        element->SetPlasticityFormulation(plasticity_formulation);
        if (strain_formulation == ChElementHexaANCF_3813_9::StrainFormulation::Hencky) {
            element->SetPlasticity(Plasticity);
            if (Plasticity) {
                element->SetYieldStress(0.0);
                element->SetHardeningSlope(5e5);
                element->SetCCPInitial(CCPInitial);
                if (plasticity_formulation == ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager) {
                    element->SetFriction(10.0);
                    element->SetDilatancy(10.0);
                    element->SetDPType(3);
                }
            }
        }

        // Add element to mesh
        my_mesh->AddElement(element);
        kk++;
    }

    // Add the mesh to the system.
    sys.Add(my_mesh);

    auto my_contactsurface = chrono_types::make_shared<ChContactSurfaceMesh>(my_surfacematerial);
    my_contactsurface->AddFacesFromBoundary(*my_mesh, 0.005);
    my_mesh->AddContactSurface(my_contactsurface);

    for (int ii = 0; ii < SnumDiv_x * SnumDiv_y; ii++) {
        int node0 = (ii / (SnumDiv_x)) * (SN_x) + ii % SnumDiv_x;
        int node1 = (ii / (SnumDiv_x)) * (SN_x) + ii % SnumDiv_x + 1;
        int node2 = (ii / (SnumDiv_x)) * (SN_x) + ii % SnumDiv_x + 1 + SN_x;
        int node3 = (ii / (SnumDiv_x)) * (SN_x) + ii % SnumDiv_x + SN_x;

        auto elementshell = chrono_types::make_shared<ChElementShellANCF_3423>();
        elementshell->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_shell_mesh->GetNode(node0)),
                               std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_shell_mesh->GetNode(node1)),
                               std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_shell_mesh->GetNode(node2)),
                               std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_shell_mesh->GetNode(node3)));

        elementshell->SetDimensions(Sdx, Sdy);
        elementshell->AddLayer(dz, 0 * CH_DEG_TO_RAD, mat);
        elementshell->SetAlphaDamp(0.0);  // Structural damping for this element
        my_shell_mesh->AddElement(elementshell);
    }

    // Add the mesh to the system.
    sys.Add(my_shell_mesh);

    auto my_contactsurface_shell = chrono_types::make_shared<ChContactSurfaceMesh>(my_surfacematerial);
    my_contactsurface_shell->AddFacesFromBoundary(*my_shell_mesh, 0.005);
    my_shell_mesh->AddContactSurface(my_contactsurface_shell);

    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    // Turn off gravity for only the shell elements
    my_shell_mesh->SetAutomaticGravity(false);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    auto mvisualizemeshref = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshref->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddVisualShapeFEA(mvisualizemeshref);

    auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddVisualShapeFEA(mvisualizemeshC);

    auto mvisualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizemeshD);

    auto mvisualizemeshcoll = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshcoll->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddVisualShapeFEA(mvisualizemeshcoll);

    // Duplicate irrlicht settings for the shell mesh

    auto mvisualizemesh_shell = chrono_types::make_shared<ChVisualShapeFEA>(my_shell_mesh);
    mvisualizemesh_shell->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh_shell->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh_shell->SetShrinkElements(true, 0.85);
    mvisualizemesh_shell->SetSmoothFaces(true);
    my_shell_mesh->AddVisualShapeFEA(mvisualizemesh_shell);

    auto mvisualizemeshref_shell = chrono_types::make_shared<ChVisualShapeFEA>(my_shell_mesh);
    mvisualizemeshref_shell->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshref_shell->SetWireframe(true);
    mvisualizemeshref_shell->SetDrawInUndeformedReference(true);
    my_shell_mesh->AddVisualShapeFEA(mvisualizemeshref_shell);

    auto mvisualizemeshC_shell = chrono_types::make_shared<ChVisualShapeFEA>(my_shell_mesh);
    mvisualizemeshC_shell->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshC_shell->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC_shell->SetSymbolsThickness(0.004);
    my_shell_mesh->AddVisualShapeFEA(mvisualizemeshC_shell);

    auto mvisualizemeshD_shell = chrono_types::make_shared<ChVisualShapeFEA>(my_shell_mesh);
    mvisualizemeshD_shell->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_VECT_SPEED);
    mvisualizemeshD_shell->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    mvisualizemeshD_shell->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshD_shell->SetSymbolsScale(1);
    mvisualizemeshD_shell->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD_shell->SetZbufferHide(false);
    my_shell_mesh->AddVisualShapeFEA(mvisualizemeshD_shell);

    auto mvisualizemeshcoll_shell = chrono_types::make_shared<ChVisualShapeFEA>(my_shell_mesh);
    mvisualizemeshcoll_shell->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
    mvisualizemeshcoll_shell->SetWireframe(true);
    mvisualizemeshcoll_shell->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_shell_mesh->AddVisualShapeFEA(mvisualizemeshcoll_shell);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "9-Node, Large Deformation Brick Element",
                                         ChVector3d(-0.4, -0.3, 0.0), ChVector3d(0.0, 0.5, -0.1));

    // Use the MKL Solver
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
    mkl_solver->LockSparsityPattern(true);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxIters(20);
    mystepper->SetAbsTolerances(1e-4, 1e-2);
    mystepper->SetVerbose(true);

    sys.Update();

    std::string filename = out_dir + "/ShellBrickContact.txt";
    outputfile = fopen(filename.c_str(), "w");
    fprintf(outputfile, "%15.7e  ", sys.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
    fprintf(outputfile, "\n  ");

    double start = std::clock();
    int Iter = 0;
    int timecount = 0;
    while (vis->Run() && (sys.GetChTime() <= 1.0)) {
        if (sys.GetChTime() < 0.5) {
            for (int ii = 0; ii < 25; ii++) {
                auto Snode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_shell_mesh->GetNode(ii));
                Snode->SetForce(ChVector3d(0.0, 0.0, -100.0 * timecount * timestep));
            }
        } else {
            for (int ii = 0; ii < 25; ii++) {
                auto Snode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_shell_mesh->GetNode(ii));
                Snode->SetForce(ChVector3d(0.0, 0.0, 10.0 * timecount * timestep));
            }
        }

        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);

        Iter += mystepper->GetNumIterations();
        std::cout << "t = " << sys.GetChTime() << std::endl;
        std::cout << "Last it: " << mystepper->GetNumIterations() << std::endl;
        // std::cout << "Body Contact F: " << Plate->GetContactForce() << std::endl;
        std::cout << nodetip1->GetPos().x() << std::endl;
        std::cout << nodetip1->GetPos().y() << std::endl;
        std::cout << nodetip1->GetPos().z() << std::endl;
        std::cout << nodetip1->GetPosDt().z() << std::endl;

        fprintf(outputfile, "%15.7e  ", sys.GetChTime());
        for (int in = 0; in < XYNumNodes; in++) {
            auto nodetest = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(in));
            fprintf(outputfile, "%15.7e  ", nodetest->GetPos().x());
            fprintf(outputfile, "%15.7e  ", nodetest->GetPos().y());
            fprintf(outputfile, "%15.7e  ", nodetest->GetPos().z());
        }
        // fprintf(outputfile, "%15.7e  ", Plate->GetContactForce().z());
        fprintf(outputfile, "\n  ");
    }
    timecount++;

    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "Simulation Time: " << duration << std::endl;
    std::cout << "Force Time: " << my_mesh->GetTimeInternalForces() << std::endl;
    std::cout << "Jacobian Time: " << my_mesh->GetTimeJacobianLoad() << std::endl;
    std::cout << "Solver Time: " << sys.GetTimerLSsolve() << std::endl;
    std::cout << Iter << std::endl;
}

// Test Case
void SimpleBoxContact(const std::string& out_dir) {
    FILE* outputfile;
    ChSystemSMC sys;
    sys.UseMaterialProperties(false);
    sys.SetAdhesionForceModel(ChSystemSMC::AdhesionForceModel::Constant);
    sys.SetContactForceModel(ChSystemSMC::ContactForceModel::PlainCoulomb);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "     9-Node, Large Deformation Brick Element with implicit integration " << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 0.05;
    double plate_lenght_y = 0.05;
    double plate_lenght_z = 0.05;

    // Specification of the 9-node brick shell
    int numDiv_x = 2;
    int numDiv_y = 2;
    int numDiv_z = 1;

    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y * numDiv_z;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    bool Plasticity = false;
    double brick_gap = 0.001;  // Initial separation between brick elements and box.
    double timestep = 2e-4;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz + brick_gap;
            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode((numDiv_z + 1) * XYNumNodes - XYNumNodes));
    // Create an orthotropic material.
    double rho = 8000.0;
    ChVector3d E(200e9, 200e9, 200e9);
    ChVector3d nu(0.3, 0.3, 0.3);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());
    std::shared_ptr<ChContactMaterialSMC> my_surfacematerial(new ChContactMaterialSMC);
    my_surfacematerial->SetKn(1e6f);
    my_surfacematerial->SetKt(1e6f);
    my_surfacematerial->SetGn(6e2f);
    my_surfacematerial->SetGt(6e2f);

    // Set initial configuration as elastic (plastic deformation gradient is identity)
    ChMatrixNM<double, 9, 8> CCPInitial;
    for (int k = 0; k < 8; k++) {
        CCPInitial(0, k) = 1;
        CCPInitial(4, k) = 1;
        CCPInitial(8, k) = 1;
    }

    auto strain_formulation = ChElementHexaANCF_3813_9::StrainFormulation::Hencky;
    auto plasticity_formulation = ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager;

    // Create the elements
    int jj = -1;
    int kk = -1;
    for (int i = 0; i < TotalNumElements; i++) {
        if (i % (numDiv_x * numDiv_y) == 0) {
            jj++;
            kk = 0;
        }
        // Adjacent nodes
        int node0 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + jj * (N_x * N_y);
        int node1 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + jj * (N_x * N_y);
        int node2 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + N_x + jj * (N_x * N_y);
        int node3 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + N_x + jj * (N_x * N_y);
        int node4 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + XYNumNodes + jj * (N_x * N_y);
        int node5 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + XYNumNodes + jj * (N_x * N_y);
        int node6 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + N_x + XYNumNodes + jj * (N_x * N_y);
        int node7 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + N_x + XYNumNodes + jj * (N_x * N_y);
        int node8 = (numDiv_z + 1) * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.0);     // Structural damping for this element
        element->SetDPIterationNo(50);  // Set maximum number of iterations for Drucker-Prager Newton-Raphson
        element->SetDPYieldTol(1e-8);   // Set stop tolerance for Drucker-Prager Newton-Raphson
        element->SetStrainFormulation(strain_formulation);
        element->SetPlasticityFormulation(plasticity_formulation);
        if (strain_formulation == ChElementHexaANCF_3813_9::StrainFormulation::Hencky) {
            element->SetPlasticity(Plasticity);
            if (Plasticity) {
                element->SetYieldStress(1e5);
                element->SetHardeningSlope(5e5);
                element->SetCCPInitial(CCPInitial);
                if (plasticity_formulation == ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager) {
                    element->SetFriction(10.0);   // Internal friction for Drucker-Prager
                    element->SetDilatancy(10.0);  // Dilatancy angle for non-associative plasticity
                    element->SetDPType(3);        // Hardening type (constant)
                }
            }
        }

        // Add element to mesh
        my_mesh->AddElement(element);
        kk++;
    }

    // Add the mesh to the system
    sys.Add(my_mesh);

    auto my_contactsurface = chrono_types::make_shared<ChContactSurfaceMesh>(my_surfacematerial);
    my_contactsurface->AddFacesFromBoundary(*my_mesh, 0.0);
    my_mesh->AddContactSurface(my_contactsurface);

    double plate_w = 0.1;
    double plate_l = 0.1;
    double plate_h = 0.1;
    auto Plate =
        chrono_types::make_shared<ChBodyEasyBox>(plate_l, plate_w, plate_h, 1000, true, true, my_surfacematerial);
    sys.Add(Plate);
    Plate->SetFixed(true);
    Plate->SetPos(ChVector3d(0.025, 0.025, -0.0015 - plate_h / 2));
    Plate->SetRot(ChQuaternion<>(1.0, 0.0, 0.0, 0.0));
    // Plate->SetPosDt(ChVector3d(0.0, 0.0, -0.1));

    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.99);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    auto mvisualizemeshref = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshref->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddVisualShapeFEA(mvisualizemeshref);

    auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddVisualShapeFEA(mvisualizemeshC);

    auto mvisualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizemeshD);

    auto mvisualizemeshcoll = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshcoll->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddVisualShapeFEA(mvisualizemeshcoll);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "9-Node, Large Deformation Brick Element",
                                         ChVector3d(-0.4, -0.3, 0.0), ChVector3d(0.0, 0.5, -0.1));

    // Use the MKL Solver
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
    mkl_solver->LockSparsityPattern(true);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxIters(20);
    mystepper->SetAbsTolerances(1e-3, 1e-2);
    mystepper->SetVerbose(true);

    sys.Update();

    std::string filename = out_dir + "/SimpleBoxContact.txt";
    outputfile = fopen(filename.c_str(), "w");
    fprintf(outputfile, "%15.7e  ", sys.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
    fprintf(outputfile, "\n  ");

    double start = std::clock();
    int Iter = 0;
    int timecount = 0;
    while (vis->Run() && (sys.GetChTime() <= 1.0)) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);

        Iter += mystepper->GetNumIterations();
        // std::cout << "t = " << sys.GetChTime() << std::endl;
        // std::cout << "Last it: " << mystepper->GetNumIterations() << std::endl;
        // std::cout << "Plate Pos: " << Plate->GetPos();
        // std::cout << "Plate Vel: " << Plate->GetPosDt();
        // std::cout << "Body Contact F: " << Plate->GetContactForce() << std::endl;
        // std::cout << nodetip1->GetPos().x() << std::endl;
        // std::cout << nodetip1->GetPos().y() << std::endl;
        // std::cout << nodetip1->GetPos().z() << std::endl;
        // std::cout << nodetip1->GetPosDt().z() << std::endl;
        fprintf(outputfile, "%15.7e  ", sys.GetChTime());
        for (int in = 0; in < XYNumNodes; in++) {
            auto nodetest = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(in));
            fprintf(outputfile, "%15.7e  ", nodetest->GetPos().x());
            fprintf(outputfile, "%15.7e  ", nodetest->GetPos().y());
            fprintf(outputfile, "%15.7e  ", nodetest->GetPos().z());
        }
        fprintf(outputfile, "%15.7e  ", Plate->GetContactForce().z());
        fprintf(outputfile, "\n  ");

        timecount++;
    }

    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "Simulation Time: " << duration << std::endl;
    std::cout << "Force Time: " << my_mesh->GetTimeInternalForces() << std::endl;
    std::cout << "Jacobian Time: " << my_mesh->GetTimeJacobianLoad() << std::endl;
    std::cout << "Solver Time: " << sys.GetTimerLSsolve() << std::endl;
    std::cout << Iter << std::endl;
}

// SoilBin Dynamic
void SoilBin(const std::string& out_dir) {
    FILE* outputfile;
    ChSystemSMC sys;
    sys.UseMaterialProperties(false);
    sys.SetAdhesionForceModel(ChSystemSMC::AdhesionForceModel::Constant);
    // sys.SetContactForceModel(ChSystemSMC::ContactForceModel::PlainCoulomb);
    sys.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "     9-Node, Large Deformation Brick Element with implicit integration " << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 0.4;
    double plate_lenght_y = 0.4;
    double plate_lenght_z = 0.6;

    // Specification of the mesh
    int numDiv_x = 10;
    int numDiv_y = 10;
    int numDiv_z = 8;

    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y * numDiv_z;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    bool Plasticity = true;
    double timestep = 5e-5;  //

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;
            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (j == 0) {
                node->SetFixed(true);
            }
            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    auto nodecenter = std::dynamic_pointer_cast<ChNodeFEAxyz>(
        my_mesh->GetNode(N_x * N_y * numDiv_z + N_x * (numDiv_y / 2) + (N_x - 1) / 2));

    // Create an orthotropic material.
    double rho = 200.0;
    ChVector3d E(1.379e7, 1.379e7, 1.379e7);
    ChVector3d nu(0.3, 0.3, 0.3);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());
    std::shared_ptr<ChContactMaterialSMC> my_surfacematerial(new ChContactMaterialSMC);
    my_surfacematerial->SetKn(0.2e4);  // 0.2e6
    my_surfacematerial->SetKt(0.2e4);  // 0.2e6
    my_surfacematerial->SetGn(0.2e2);  // 0.2e4
    my_surfacematerial->SetGt(0.2e2);  // 0.2e4
    ChMatrixNM<double, 9, 8> CCPInitial;
    for (int k = 0; k < 8; k++) {
        CCPInitial(0, k) = 1;
        CCPInitial(4, k) = 1;
        CCPInitial(8, k) = 1;
    }

    auto strain_formulation = ChElementHexaANCF_3813_9::StrainFormulation::Hencky;
    auto plasticity_formulation = ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager;

    // Create the elements
    int jj = -1;
    int kk = -1;
    for (int i = 0; i < TotalNumElements; i++) {
        if (i % (numDiv_x * numDiv_y) == 0) {
            jj++;
            kk = 0;
        }
        // Adjacent nodes
        int node0 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + jj * (N_x * N_y);
        int node1 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + jj * (N_x * N_y);
        int node2 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + N_x + jj * (N_x * N_y);
        int node3 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + N_x + jj * (N_x * N_y);
        int node4 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + XYNumNodes + jj * (N_x * N_y);
        int node5 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + XYNumNodes + jj * (N_x * N_y);
        int node6 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + 1 + N_x + XYNumNodes + jj * (N_x * N_y);
        int node7 = (kk / (numDiv_x)) * (N_x) + kk % numDiv_x + N_x + XYNumNodes + jj * (N_x * N_y);
        int node8 = (numDiv_z + 1) * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(5e-4);    // Structural damping for this element
        element->SetDPIterationNo(50);  // Set maximum number of iterations for Drucker-Prager Newton-Raphson
        element->SetDPYieldTol(1e-8);   // Set stop tolerance for Drucker-Prager Newton-Raphson
        element->SetStrainFormulation(strain_formulation);
        element->SetPlasticityFormulation(plasticity_formulation);
        if (strain_formulation == ChElementHexaANCF_3813_9::StrainFormulation::Hencky) {
            element->SetPlasticity(Plasticity);
            if (Plasticity) {
                element->SetYieldStress(10000.0);
                element->SetHardeningSlope(5000);
                element->SetCCPInitial(CCPInitial);
                if (plasticity_formulation == ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager) {
                    element->SetFriction(0.00001);
                    element->SetDilatancy(0.00001);
                    element->SetDPType(3);
                }
            }
        }

        // Add element to mesh
        my_mesh->AddElement(element);
        kk++;
    }

    // Add the mesh to the system
    sys.Add(my_mesh);

    auto my_contactsurface = chrono_types::make_shared<ChContactSurfaceNodeCloud>(my_surfacematerial);
    my_contactsurface->AddAllNodes(*my_mesh, 0.006);
    my_mesh->AddContactSurface(my_contactsurface);

    // Creat punch
    double plate_w = 0.2;  // 0.15
    double plate_l = 0.2;  // 0.15
    double plate_h = 0.1;
    auto Plate =
        chrono_types::make_shared<ChBodyEasyBox>(plate_l, plate_w, plate_h, 1000, true, true, my_surfacematerial);
    sys.Add(Plate);
    Plate->SetFixed(false);
    Plate->SetPos(ChVector3d(0.2, 0.2, 0.6001 + plate_h / 2));
    Plate->SetRot(ChQuaternion<>(1.0, 0.0, 0.0, 0.0));
    Plate->SetPosDt(ChVector3d(0.0, 0.0, 0.0));
    Plate->SetRotDt(ChQuaternion<>(0.0, 0.0, 0.0, 0.0));
    Plate->SetMass(1.2265625);

    //// Create ground body
    auto Ground = chrono_types::make_shared<ChBody>();
    Ground->SetFixed(true);
    Ground->SetPos(ChVector3d(0.0, 0.0, -0.02));
    Ground->SetRot(ChQuaternion<>(1.0, 0.0, 0.0, 0.0));
    sys.Add(Ground);

    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    my_mesh->SetAutomaticGravity(false);

    std::shared_ptr<ChLinkLockPointPlane> constraintLateral;
    std::shared_ptr<ChLinkLockPointPlane> constraintLongitudinal;

    ////// Constrain only the lateral displacement of the Rim
    constraintLateral = chrono_types::make_shared<ChLinkLockPointPlane>();
    sys.AddLink(constraintLateral);
    constraintLateral->Initialize(Plate, Ground, ChFrame<>(Plate->GetPos(), QuatFromAngleX(CH_PI_2)));

    constraintLongitudinal = chrono_types::make_shared<ChLinkLockPointPlane>();
    sys.AddLink(constraintLongitudinal);
    constraintLongitudinal->Initialize(Plate, Ground, ChFrame<>(Plate->GetPos(), QuatFromAngleY(CH_PI_2)));

    // Create a load container and a body force load on the plate
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    auto plate_load = chrono_types::make_shared<ChLoadBodyForce>(Plate, VNULL, false, VNULL, true);
    load_container->Add(plate_load);
    sys.Add(load_container);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    auto mvisualizemeshref = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshref->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddVisualShapeFEA(mvisualizemeshref);

    auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddVisualShapeFEA(mvisualizemeshC);

    auto mvisualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizemeshD);

    auto mvisualizemeshcoll = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshcoll->SetFEMdataType(ChVisualShapeFEA::DataType::CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddVisualShapeFEA(mvisualizemeshcoll);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "9-Node, Large Deformation Brick Element",
                                         ChVector3d(-0.4, -0.3, 0.0), ChVector3d(0.0, 0.5, -0.1));

    // Use the MKL Solver
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
    mkl_solver->LockSparsityPattern(true);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxIters(20);
    mystepper->SetAbsTolerances(1e-4, 1e-2);
    mystepper->SetVerbose(false);

    sys.Update();

    std::string filename = out_dir + "/SoilBin.txt";
    outputfile = fopen(filename.c_str(), "w");
    fprintf(outputfile, "%15.7e  ", sys.GetChTime());
    for (int ii = 0; ii < N_x; ii++) {
        auto nodetest =
            std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(N_x * N_y * numDiv_z + N_x * (numDiv_y / 2) + ii));
        fprintf(outputfile, "%15.7e  ", nodetest->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetest->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetest->GetPos().z());
    }
    fprintf(outputfile, "\n  ");

    double start = std::clock();
    int Iter = 0;

    while (vis->Run()) {
        double time = sys.GetChTime();
        if (time > 1)
            break;
        plate_load->SetForce(ChVector3d(0, 0, -1500 * std::sin(time * CH_PI)), false);
        Plate->SetRot(ChQuaternion<>(1.0, 0.0, 0.0, 0.0));

        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);

        Iter += mystepper->GetNumIterations();
        std::cout << "t = " << time << std::endl;
        std::cout << "   Last it: " << mystepper->GetNumIterations() << std::endl;
        std::cout << "   Plate Pos: " << Plate->GetPos() << std::endl;
        std::cout << "   Plate Vel: " << Plate->GetPosDt() << std::endl;
        std::cout << "   Plate Rot: " << Plate->GetRot() << std::endl;
        std::cout << "   Plate Rot_v: " << Plate->GetRotDt() << std::endl;
        std::cout << "   Body Contact F: " << Plate->GetContactForce() << std::endl;
        std::cout << "   Center node pos: " << nodecenter->GetPos() << std::endl;

        fprintf(outputfile, "%15.7e  ", time);
        for (int ii = 0; ii < N_x; ii++) {
            auto nodetest = std::dynamic_pointer_cast<ChNodeFEAxyz>(
                my_mesh->GetNode(N_x * N_y * numDiv_z + N_x * (numDiv_y / 2) + ii));
            fprintf(outputfile, "%15.7e  ", nodetest->GetPos().x());
            fprintf(outputfile, "%15.7e  ", nodetest->GetPos().y());
            fprintf(outputfile, "%15.7e  ", nodetest->GetPos().z());
        }
        fprintf(outputfile, "%15.7e  ", Plate->GetContactForce().x());
        fprintf(outputfile, "%15.7e  ", Plate->GetContactForce().y());
        fprintf(outputfile, "%15.7e  ", Plate->GetContactForce().z());
        fprintf(outputfile, "%15.7e  ", Plate->GetPos().z());
        fprintf(outputfile, "\n  ");
    }

    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "Simulation Time: " << duration << std::endl;
    std::cout << "Force Time: " << my_mesh->GetTimeInternalForces() << std::endl;
    std::cout << "Jacobian Time: " << my_mesh->GetTimeJacobianLoad() << std::endl;
    std::cout << "Solver Time: " << sys.GetTimerLSsolve() << std::endl;
    std::cout << Iter << std::endl;
}

// Axial Dynamic
void AxialDynamics(const std::string& out_dir) {
    FILE* outputfile;
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "     9-Node, Large Deformation Brick Element with implicit integration " << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 0.05;
    double plate_lenght_z = 0.05;

    // Specification of the mesh
    int numDiv_x = 20;
    int numDiv_y = 1;
    int numDiv_z = 1;

    int N_x = numDiv_x + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    bool Plasticity = false;
    double timestep = 1e-4;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;
            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (i % (numDiv_x + 1) == 0) {
                node->SetFixed(true);
            }
            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    double force = 0.0;
    // Get a handle to the tip node.
    auto nodetip1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));
    auto nodetip2 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1 - numDiv_x - 1));
    auto nodetip3 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(XYNumNodes - 1));
    auto nodetip4 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(XYNumNodes - 1 - numDiv_x - 1));
    nodetip1->SetForce(ChVector3d(0.0, 0.0, 0.0));
    nodetip2->SetForce(ChVector3d(0.0, 0.0, 0.0));
    nodetip3->SetForce(ChVector3d(0.0, 0.0, 0.0));
    nodetip4->SetForce(ChVector3d(0.0, 0.0, 0.0));
    // Create an orthotropic material.
    double rho = 7850.0;
    ChVector3d E(1.0e7, 1.0e7, 1.0e7);
    ChVector3d nu(0.3, 0.3, 0.3);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());

    ChMatrixNM<double, 9, 8> CCPInitial;
    for (int k = 0; k < 8; k++) {
        CCPInitial(0, k) = 1;
        CCPInitial(4, k) = 1;
        CCPInitial(8, k) = 1;
    }

    auto strain_formulation = ChElementHexaANCF_3813_9::StrainFormulation::Hencky;
    auto plasticity_formulation = ChElementHexaANCF_3813_9::PlasticityFormulation::J2;

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node4 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + XYNumNodes;
        int node5 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + XYNumNodes;
        int node6 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x + XYNumNodes;
        int node7 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x + XYNumNodes;
        int node8 = 2 * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.0);     // Structural damping for this element
        element->SetDPIterationNo(50);  // Set maximum number of iterations for Drucker-Prager Newton-Raphson
        element->SetDPYieldTol(1e-8);   // Set stop tolerance for Drucker-Prager Newton-Raphson
        element->SetStrainFormulation(strain_formulation);
        element->SetPlasticityFormulation(plasticity_formulation);
        if (strain_formulation == ChElementHexaANCF_3813_9::StrainFormulation::Hencky) {
            element->SetPlasticity(Plasticity);
            if (Plasticity) {
                element->SetYieldStress(1e5);
                element->SetHardeningSlope(5e5);
                element->SetCCPInitial(CCPInitial);
                if (plasticity_formulation == ChElementHexaANCF_3813_9::PlasticityFormulation::DruckerPrager) {
                    element->SetFriction(10.0);
                    element->SetDilatancy(10.0);
                    element->SetDPType(3);
                }
            }
        }

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    sys.Add(my_mesh);

    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, 0.0));
    my_mesh->SetAutomaticGravity(false);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    // auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    // mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    // mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    // mvisualizemesh->SetShrinkElements(true, 0.85);
    // mvisualizemesh->SetSmoothFaces(true);
    // my_mesh->AddVisualShapeFEA(mvisualizemesh);

    // auto mvisualizemeshref = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    // mvisualizemeshref->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    // mvisualizemeshref->SetWireframe(true);
    // mvisualizemeshref->SetDrawInUndeformedReference(true);
    // my_mesh->AddVisualShapeFEA(mvisualizemeshref);

    // auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    // mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    // mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    // mvisualizemeshC->SetSymbolsThickness(0.004);
    // my_mesh->AddVisualShapeFEA(mvisualizemeshC);

    // auto mvisualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    //// mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_VECT_SPEED);
    // mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    // mvisualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    // mvisualizemeshD->SetSymbolsScale(1);
    // mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    // mvisualizemeshD->SetZbufferHide(false);
    // my_mesh->AddVisualShapeFEA(mvisualizemeshD);

    // application.AssetBindAll();
    // application.AssetUpdateAll();

    // Use the MKL Solver
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
    mkl_solver->LockSparsityPattern(true);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxIters(20);
    mystepper->SetAbsTolerances(1e-4, 1e-2);
    mystepper->SetVerbose(false);

    sys.Update();

    std::string filename = out_dir + "/AxialDynamics.txt";
    outputfile = fopen(filename.c_str(), "w");
    fprintf(outputfile, "%15.7e  ", sys.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
    fprintf(outputfile, "\n  ");

    double start = std::clock();
    int Iter = 0;
    while (/*vis->Run() && */ (sys.GetChTime() <= 1.0)) {
        // application.BeginScene();
        // application.Render();
        // application.DoStep();

        force = 300 * std::sin(sys.GetChTime() * CH_PI) / 4;
        nodetip1->SetForce(ChVector3d(force, 0.0, 0.0));
        nodetip2->SetForce(ChVector3d(force, 0.0, 0.0));
        nodetip3->SetForce(ChVector3d(force, 0.0, 0.0));
        nodetip4->SetForce(ChVector3d(force, 0.0, 0.0));
        sys.DoStepDynamics(timestep);
        // application.EndScene();
        Iter += mystepper->GetNumIterations();
        // std::cout << "t = " << sys.GetChTime() << std::endl;
        // std::cout << "Last it: " << mystepper->GetNumIterations() << "\n\n";
        // if (!application.GetPaused()) {
        fprintf(outputfile, "%15.7e  ", sys.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
        fprintf(outputfile, "\n  ");
        //}
    }
    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "Simulation Time: " << duration << std::endl;
    std::cout << "Force Time: " << my_mesh->GetTimeInternalForces() << std::endl;
    std::cout << "Jacobian Time: " << my_mesh->GetTimeJacobianLoad() << std::endl;
    std::cout << "Solver Time: " << sys.GetTimerLSsolve() << std::endl;
    std::cout << Iter << std::endl;
}

// QuasiStatic
void BendingQuasiStatic(const std::string& out_dir) {
    FILE* outputfile;
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "  9-Node, Large Deformation Brick Element: Bending Problem " << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;

    // Specification of the mesh
    int numDiv_x = 8;
    int numDiv_y = 8;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    double timestep = 5e-3;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;

            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (i % (numDiv_x + 1) == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));

    // All layers for all elements share the same material.
    double rho = 500;
    ChVector3d E(2.1e8, 2.1e8, 2.1e8);
    ChVector3d nu(0.3, 0.3, 0.3);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node4 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + XYNumNodes;
        int node5 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + XYNumNodes;
        int node6 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x + XYNumNodes;
        int node7 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x + XYNumNodes;
        int node8 = 2 * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.25);  // Structural damping for this element

        element->SetStrainFormulation(ChElementHexaANCF_3813_9::StrainFormulation::Hencky);
        element->SetPlasticity(false);

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    sys.Add(my_mesh);

    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, 0.0));

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    auto mvisualizemeshref = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshref->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddVisualShapeFEA(mvisualizemeshref);

    auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddVisualShapeFEA(mvisualizemeshC);

    auto mvisualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    // mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizemeshD);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "9-Node, Large Deformation Brick Element",
                                         ChVector3d(-0.4, -0.3, 0.0), ChVector3d(0.0, 0.5, -0.1));

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Use the MKL Solver
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
    mkl_solver->LockSparsityPattern(true);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxIters(2000);
    mystepper->SetAbsTolerances(1e-3, 1e-1);
    mystepper->SetVerbose(true);

    sys.Update();

    std::string filename = out_dir + "/BendingQuasistatic.txt";
    outputfile = fopen(filename.c_str(), "w");
    fprintf(outputfile, "%15.7e  ", sys.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
    fprintf(outputfile, "\n  ");

    double force = 0.0;
    while (vis->Run() && (sys.GetChTime() <= 2.0)) {
        vis->BeginScene();
        vis->Render();

        if (sys.GetChTime() > 1.0) {
            force = -50;
        } else {
            force = -50 * sys.GetChTime();
        }

        nodetip->SetForce(ChVector3d(0.0, 0.0, force));

        std::cout << sys.GetChTime() << " " << nodetip->GetPos().x() << " " << nodetip->GetPos().y() << " "
                  << nodetip->GetPos().z() << std::endl;

        sys.DoStepDynamics(timestep);

        std::cout << "Force: " << force << std::endl;
        fprintf(outputfile, "%15.7e  ", sys.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
        fprintf(outputfile, "\n  ");

        vis->EndScene();
    }
}

// Swinging (Bricked) Shell
void SwingingShell(const std::string& out_dir) {
    FILE* outputfile;
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    std::cout << "--------------------------------------------------------------------" << std::endl;
    std::cout << "--------------------------------------------------------------------" << std::endl;
    std::cout << " 9-Node, Large Deformation Brick Element: Swinging (Bricked) Shell  " << std::endl;
    std::cout << "--------------------------------------------------------------------" << std::endl;

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;

    // Specification of the mesh
    int numDiv_x = 16;
    int numDiv_y = 16;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    double timestep = 2e-3;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;

            // Create the node
            auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(loc_x, loc_y, loc_z));
            node->SetMass(0);
            // Fix all nodes along the axis X=0
            if (i == 0 && j == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAcurv>(ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0),
                                                             ChVector3d(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));
    nodetip->SetForce(ChVector3d(0.0, 0.0, -0.0));

    // All layers for all elements share the same material.
    double rho = 1000;
    ChVector3d E(2.1e7, 2.1e7, 2.1e7);
    ChVector3d nu(0.3, 0.3, 0.3);
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetRayleighDampingBeta(0.0);
    material->SetRayleighDampingAlpha(0.0);
    material->SetDensity(rho);
    material->SetYoungModulus(E.x());
    material->SetPoissonRatio(nu.x());

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node4 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + XYNumNodes;
        int node5 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + XYNumNodes;
        int node6 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x + XYNumNodes;
        int node7 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x + XYNumNodes;
        int node8 = 2 * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementHexaANCF_3813_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector3d(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.005);  // Structural damping for this element
        element->SetStrainFormulation(ChElementHexaANCF_3813_9::StrainFormulation::Hencky);
        element->SetPlasticity(false);

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    sys.Add(my_mesh);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    sys.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));

    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    auto mvisualizemeshref = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshref->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_STRESS_VONMISES);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddVisualShapeFEA(mvisualizemeshref);

    auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddVisualShapeFEA(mvisualizemeshC);

    auto mvisualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    // mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizemeshD);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "9-Node, Large Deformation Brick Element",
                                         ChVector3d(-0.4, -0.3, -0.5), ChVector3d(0.0, 0.5, -0.1));

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Use the MKL Solver
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
    mkl_solver->LockSparsityPattern(true);

    // Set the time integrator parameters
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxIters(20);
    mystepper->SetAbsTolerances(1e-3, 1e-1);
    mystepper->SetVerbose(true);

    sys.Update();

    std::string filename = out_dir + "SwingingShell.txt";
    outputfile = fopen(filename.c_str(), "w");
    fprintf(outputfile, "%15.7e  ", sys.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
    fprintf(outputfile, "\n  ");

    while (vis->Run() && (sys.GetChTime() < 2.01)) {
        vis->BeginScene();
        vis->Render();

        sys.DoStepDynamics(timestep);

        fprintf(outputfile, "%15.7e  ", sys.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
        fprintf(outputfile, "\n  ");

        vis->EndScene();

        std::cout << sys.GetChTime() << " " << nodetip->GetPos().x() << " " << nodetip->GetPos().y() << " "
                  << nodetip->GetPos().z() << std::endl;
    }
}
