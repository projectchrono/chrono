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
// Unit test for continuum-based bilinear shear deformable shell element using ANCF
//
// Successful execution of this unit test may validate: this element's elastic, isotropic
// force formulation and numerical integration implementations.
//
// The reference file data was validated by matching the steady-state response of the
// flat shell tip (i.e. only steady-state was validated) in the paper by Yamashita, Valkeapaa,
// Jayakumar, and Sugiyama, "Continuum Mechanics Based Bilinear Shear Deformable Shell
// Element Using Absolute Nodal Coordinate Formulation", ASME Journal of Computational and
// Nonlinear Dynamics, 10(5), 051012 (Sep 01, 2015). See its Figure 4.
//
// Gravity must be disabled; only a constant load of -50 N at a corner is used. Only
// 10 time steps are checked by default. User can increase this value up to 4000.
// =============================================================================
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fea;

const double precision = 1e-5;  // Used to accept/reject implementation

int main(int argc, char* argv[]) {
    // Utils to open/read files: Load reference solution ("golden") file
    ChMatrixDynamic<> FileInputMat(4000, 2);
    std::string shell_validation_file = GetChronoDataPath() + "testing/" + "UT_ANCFShellIso.txt";
    std::ifstream fileMid(shell_validation_file);
    if (!fileMid.is_open()) {
        fileMid.open(shell_validation_file);
    }
    if (!fileMid) {
        std::cout << "Cannot open file.\n";
        exit(1);
    }
    for (int x = 0; x < 4000; x++) {
        fileMid >> FileInputMat[x][0] >> FileInputMat[x][1];
    }
    fileMid.close();

    // Definition of the model
    ChSystem my_system;

    // The physical system: it contains all physical objects.
    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    ChSharedPtr<ChMesh> my_mesh(new ChMesh);
    int numFlexBody = 1;
    const int num_steps = 10;        // Number of time steps for unit test (range 1 to 4000)
    const double time_step = 0.001;  // Time step
    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;  // small thickness
    // Specification of the mesh
    const int numDiv_x = 4;
    const int numDiv_y = 4;
    const int numDiv_z = 1;
    const int N_x = numDiv_x + 1;
    const int N_y = numDiv_y + 1;
    const int N_z = numDiv_z + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    int MaxMNUM = 0;
    int MTYPE = 0;
    int MaxLayNum = 0;
    ChMatrixDynamic<double> COORDFlex(TotalNumNodes, 6);
    ChMatrixDynamic<double> VELCYFlex(TotalNumNodes, 6);
    ChMatrixDynamic<int> NumNodes(TotalNumElements, 4);
    ChMatrixDynamic<int> LayNum(TotalNumElements, 1);  // Only one layer in this unit test
    ChMatrixDynamic<int> NDR(TotalNumNodes, 6);
    ChMatrixDynamic<double> ElemLengthXY(TotalNumElements, 2);
    ChMatrixNM<double, 10, 12> MPROP;
    ChMatrixNM<int, 10, 7> MNUM;
    ChMatrixNM<int, 10, 1> NumLayer;
    double LayPROP[10][7][2];
    //!------------------------------------------------!
    //!--------------- Element data--------------------!
    //!------------------------------------------------!
    for (int i = 0; i < TotalNumElements; i++) {
        // All the elements belong to the same layer, e.g layer number 1.
        LayNum(i, 0) = 1;
        // Node number of the 4 nodes which creates element i.
        // The nodes are distributed this way. First in the x direction for constant
        // y when max x is reached go to the next level for y by doing the same
        // distribution but for y+1 and keep doing until y limit is reached. Node
        // number start from 1.
        NumNodes(i, 0) = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        NumNodes(i, 1) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        NumNodes(i, 2) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        NumNodes(i, 3) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        // Let's keep the element length a fixed number in both direction. (uniform
        // distribution of nodes in both direction)
        // If the user interested in non-uniform mesh it can be manipulated here.
        ElemLengthXY(i, 0) = dx;
        ElemLengthXY(i, 1) = dy;
        if (MaxLayNum < LayNum(i, 0)) {
            MaxLayNum = LayNum(i, 0);
        }
    }
    //!----------------------------------------------!
    //!--------- NDR,COORDFlex,VELCYFlex-------------!
    //!----------------------------------------------!

    for (int i = 0; i < TotalNumNodes; i++) {
        // If the node is the first node from the left side fix the x,y,z degree of
        // freedom+ d/dx, d/dy,d/dz as well. 1for constrained 0 for ...
        //-The NDR array is used to define the degree of freedoms that are
        // constrained in the 6 variable explained above.
        NDR(i, 0) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 1) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 2) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 3) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 4) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 5) = (i % (numDiv_x + 1) == 0) ? 1 : 0;

        //-COORDFlex are the initial coordinates for each node,
        // the first three are the position and the last three are the slope
        // coordinates for the tangent vector.
        // Note that i starts from 0 but nodes starts from 1
        COORDFlex(i, 0) = (i % (numDiv_x + 1)) * dx;
        COORDFlex(i, 1) = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
        COORDFlex(i, 2) = (i) / ((numDiv_x + 1) * (numDiv_y + 1)) * dz;
        COORDFlex(i, 3) = 0;
        COORDFlex(i, 4) = 0;
        COORDFlex(i, 5) = 1;
        //-VELCYFlex is essentially the same as COORDFlex, but for the initial
        // velocity instead of position.
        // let's assume zero initial velocity for nodes
        VELCYFlex(i, 0) = 0;
        VELCYFlex(i, 1) = 0;
        VELCYFlex(i, 2) = 0;
        VELCYFlex(i, 3) = 0;
        VELCYFlex(i, 4) = 0;
        VELCYFlex(i, 5) = 0;
    }
    //!------------------------------------------------!
    //!------------- Read Layer Data-------------------!
    //!------------------------------------------------!

    for (int i = 0; i < MaxLayNum; i++) {
        NumLayer(i, 0) = i + 1;
        // For each layer define some properties
        // For multilayer problem the user should modify the following loop as they
        // would like
        for (int j = 0; j < NumLayer(i, 0); j++) {
            LayPROP[i][j][0] = dz;  // Layerheight
            LayPROP[i][j][1] = 0;   // PlyAngle
            MNUM[i][j] = 1;         // Material_ID
            if (MaxMNUM < MNUM(i, j))
                MaxMNUM = MNUM(i, j);
        }
    }
    //!------------------------------------------------!
    //!------------ Input Material Data----------------!
    //!------------------------------------------------!
    for (int i = 0; i < MaxMNUM; i++) {
        double nu_coef = 0.3;
        MTYPE = 2;  // The user must use orthotropic input (MTYPE=2) to introduce isotropic material
                    // properties for this unit test

        if (MTYPE == 2) {
            MPROP(i, 0) = 500;      // Density [kg/m3]
            MPROP(i, 1) = 2.1E+08;  // Ex//
            MPROP(i, 2) = 2.1E+08;  // Ey
            MPROP(i, 3) = 2.1E+08;  // Ez
            // Additional information for the Type 2 of Material.
            MPROP(i, 4) = 0.3;                                    // nuxy
            MPROP(i, 5) = 0.3;                                    // nuxz
            MPROP(i, 6) = 0.3;                                    // nuyz
            MPROP(i, 7) = MPROP(i, 1) / 2.0 / (1 + MPROP(i, 6));  // Gxy
            MPROP(i, 8) = MPROP(i, 1) / 2.0 / (1 + MPROP(i, 6));  // Gxz
            MPROP(i, 9) = MPROP(i, 1) / 2.0 / (1 + MPROP(i, 6));  // Gyz
        }
    }
    ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
    // Adding the nodes to the mesh
    int i = 0;

    while (i < TotalNumNodes) {
        ChSharedPtr<ChNodeFEAxyzD> node(
            new ChNodeFEAxyzD(ChVector<>(COORDFlex(i, 0), COORDFlex(i, 1), COORDFlex(i, 2)),
                              ChVector<>(COORDFlex(i, 3), COORDFlex(i, 4), COORDFlex(i, 5))));
        node->SetMass(0.0);
        my_mesh->AddNode(node);
        if (NDR(i, 0) == 1 && NDR(i, 1) == 1 && NDR(i, 2) == 1 && NDR(i, 3) == 1 && NDR(i, 4) == 1 && NDR(i, 5) == 1) {
            node->SetFixed(true);
        }
        i++;
    }

    ChSharedPtr<ChNodeFEAxyzD> nodetip(my_mesh->GetNode(TotalNumNodes - 1).DynamicCastTo<ChNodeFEAxyzD>());
    // A random node just to try
    ChSharedPtr<ChNodeFEAxyzD> noderand(my_mesh->GetNode(TotalNumNodes / 2).DynamicCastTo<ChNodeFEAxyzD>());
    // A constrained node
    ChSharedPtr<ChNodeFEAxyzD> noderclamped(my_mesh->GetNode(0).DynamicCastTo<ChNodeFEAxyzD>());

    int elemcount = 0;
    while (elemcount < TotalNumElements) {
        ChSharedPtr<ChElementShellANCF> element(new ChElementShellANCF);
        // Save material data into InertFlexVec(98x1) at each layer
        ChMatrixNM<double, 98, 1> InertFlexVec;
        InertFlexVec.Reset();
        double TotalThickness;  // element thickness
        TotalThickness = 0.0;
        int i = elemcount;
        for (int j = 0; j < NumLayer(LayNum(i, 0) - 1, 0); j++) {
            int ij = 14 * j;
            InertFlexVec(ij) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][0];  // Density
            InertFlexVec(ij + 1) = ElemLengthXY(i, 0);                   // EL
            InertFlexVec(ij + 2) = ElemLengthXY(i, 1);                   // EW
            InertFlexVec(ij + 3) = LayPROP[LayNum(i, 0) - 1][j][0];      // Thickness per layer
            TotalThickness += InertFlexVec(ij + 3);
            InertFlexVec(ij + 4) = LayPROP[LayNum(i, 0) - 1][j][1];           // Fiber angle
            InertFlexVec(ij + 5) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][1];   // Ex
            InertFlexVec(ij + 6) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][2];   // Ey
            InertFlexVec(ij + 7) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][3];   // Ez
            InertFlexVec(ij + 8) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][4];   // nuxy
            InertFlexVec(ij + 9) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][5];   // nuxz
            InertFlexVec(ij + 10) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][6];  // nuyz
            InertFlexVec(ij + 11) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][7];  // Gxy
            InertFlexVec(ij + 12) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][8];  // Gxz
            InertFlexVec(ij + 13) = MPROP[MNUM[LayNum(i, 0) - 1][j] - 1][9];  // Gyz
        }
        ChMatrixNM<double, 7, 2> GaussZRange;
        GaussZRange.Reset();
        double CurrentHeight = 0.0;
        for (int j = 0; j < NumLayer(LayNum(i, 0) - 1, 0); j++) {
            double AA = (CurrentHeight / TotalThickness - 0.5) * 2.0;
            CurrentHeight += LayPROP[LayNum(i, 0) - 1][j][0];
            double AAA = (CurrentHeight / TotalThickness - 0.5) * 2.0;
            GaussZRange(j, 0) = AA;
            GaussZRange(j, 1) = AAA;
        }
        element->SetInertFlexVec(InertFlexVec);
        element->SetGaussZRange(GaussZRange);
        element->SetNodes(my_mesh->GetNode(NumNodes[elemcount][0]).DynamicCastTo<ChNodeFEAxyzD>(),
                          my_mesh->GetNode(NumNodes[elemcount][1]).DynamicCastTo<ChNodeFEAxyzD>(),
                          my_mesh->GetNode(NumNodes[elemcount][2]).DynamicCastTo<ChNodeFEAxyzD>(),
                          my_mesh->GetNode(NumNodes[elemcount][3]).DynamicCastTo<ChNodeFEAxyzD>());
        element->SetMaterial(mmaterial);
        element->SetNumLayers(NumLayer(LayNum(i, 0) - 1, 0));
        element->SetThickness(TotalThickness);
        element->SetElemNum(elemcount);
        element->SetAlphaDamp(0.08);
        element->Setdt(0.001);                     // dt to calculate DampingCoefficient
        element->SetGravityOn(false);              // turn gravity on/off
        ChMatrixNM<double, 35, 1> StockAlpha_EAS;  // StockAlpha(5*7,1): Max #Layer is 7
        StockAlpha_EAS.Reset();
        element->SetStockAlpha(StockAlpha_EAS);
        my_mesh->AddElement(element);
        elemcount++;
    }
    // Switch off mesh class gravity
    my_mesh->SetAutomaticGravity(false);
    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);
    // Mark completion of system construction
    my_system.SetupInitial();
    // Perform a dynamic time integration:
    my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);  // <- NEEDED because other solvers can't
                                                                 // handle stiffness matrices
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetIterLCPwarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetIterLCPmaxItersSpeed(100);
    my_system.SetIterLCPmaxItersStab(100);
    my_system.SetTolForce(1e-6);

    /*ChLcpMklSolver * mkl_solver_stab = new ChLcpMklSolver; // MKL Solver option
    ChLcpMklSolver * mkl_solver_speed = new ChLcpMklSolver;
    my_system.ChangeLcpSolverStab(mkl_solver_stab);
    my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
    mkl_solver_stab->SetProblemSizeLock(true);
    mkl_solver_stab->SetSparsityPatternLock(false);
    my_system.Update();*/

    // INT_HHT or INT_EULER_IMPLICIT
    my_system.SetIntegrationType(ChSystem::INT_HHT);

    ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxiters(100);
    mystepper->SetTolerance(1e-06);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);

    utils::Data m_data;
    m_data.resize(2);
    for (size_t col = 0; col < 2; col++)
        m_data[col].resize(num_steps);
    utils::CSV_writer csv(" ");
    std::ifstream file2("UT_ANCFShellIso.txt");

    ChVector<> mforce(0, 0, 0);
    mforce(2) = -50;
    nodetip->SetForce(mforce);

    for (unsigned int it = 0; it < num_steps; it++) {
        nodetip->SetForce(mforce);
        my_system.DoStepDynamics(time_step);
        std::cout << "Time t = " << my_system.GetChTime() << "s \n";
        // Checking tip Z displacement
        double AbsVal = abs(nodetip->pos.z - FileInputMat[it][1]);
        if (AbsVal > precision) {
            std::cout << "Unit test check failed \n";
            return 1;
        }
    }
    std::cout << "Unit test check succeeded \n";
    // std::cout << "Clamped z = " << noderclamped->pos.z << "\n";
    // std::cout << "nodetip->pos.z = " << nodetip->pos.z << "\n";
    // std::cout << "mystepper->GetNumIterations()= " << mystepper->GetNumIterations() << "\n";

    // Code snippet to generate golden file
    /*m_data[0][it] = my_system.GetChTime();
    m_data[1][it] = nodetip->pos.z;
    csv << m_data[0][it] << m_data[1][it]  << std::endl;
    csv.write_to_file("UT_ANCFShellIso.txt");*/
    return 0;
}
