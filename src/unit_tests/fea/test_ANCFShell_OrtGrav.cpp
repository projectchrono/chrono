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
// Unit test for composite continuum-based bilinear shear deformable
// shell element using ANCF. A quarter of a cylindrical shell is suddenly loaded
// by gravity forces.
// The shell in this unit test is a balanced laminate (see Jones, R. "Mechanics of
// Composite Materials") which has a pair of laminae at an angle plus/minus theta.
// The user may also consult the following paper fror a precise description of
// the formulation: Yamashita, H., Jayakumar, J., and
// Sugiyama, H., "Development of shear deformable laminated shell element and its
// application to ANCF tire model", DETC2015-46173, IDETC/CIE 2015, August 2-5,
// Boston, Massachussetts.
//
// Only 10 time steps are checked by default. User can increase this value up to 2000.
//
// Successful execution of this unit test may validate: this element's gravity
// implementation, its laminated composite internal force formulation
// and numerical integration implementations.
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
#include "chrono/core/ChMathematics.h"
// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fea;

const double precision = 5e-7;  // Used to accept/reject implementation

int main(int argc, char* argv[]) {
    // Utils to open/read files: Load reference solution ("golden") file
    ChMatrixDynamic<> FileInputMat(2000, 4);
    std::string LamGravShell_Val_File = GetChronoDataPath() + "testing/" + "UT_ANCFShellOrtGrav.txt";
    std::ifstream fileMid(LamGravShell_Val_File);
    if (!fileMid.is_open()) {
        fileMid.open(LamGravShell_Val_File);
    }
    if (!fileMid) {
        std::cout << "Cannot open file.\n";
        exit(1);
    }
    for (int x = 0; x < 2000; x++) {
        fileMid >> FileInputMat[x][0] >> FileInputMat[x][1] >> FileInputMat[x][2] >> FileInputMat[x][3];
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
    // double plate_lenght_x = 1; // The length along X axis is defined parametrically in
    // the initial coordinate section
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.005;  // Thickness of EACH layer (number of layers defined below)

    // Specification of the mesh
    const int numDiv_x = 8;  // Number of elements along X axis
    const int numDiv_y = 8;  // Number of elements along Y axis
    const int numDiv_z = 1;  // Single element along the thickness
    const int N_x = numDiv_x + 1;
    const int N_y = numDiv_y + 1;
    const int N_z = numDiv_z + 1;
    const double cylRadius = 1.0;  // Radius of the cylinder
    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1);

    // For uniform mesh
    double dx = CH_C_PI / 2 / numDiv_x * cylRadius;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    int MaxMNUM = 0;
    int MTYPE = 0;
    int MaxLayNum = 0;
    ChMatrixDynamic<double> COORDFlex(TotalNumNodes, 6);
    ChMatrixDynamic<double> VELCYFlex(TotalNumNodes, 6);
    ChMatrixDynamic<int> NumNodes(TotalNumElements, 4);
    ChMatrixDynamic<int> LayNum(TotalNumElements, 1);
    ChMatrixDynamic<int> NDR(TotalNumNodes, 6);
    ChMatrixDynamic<double> ElemLengthXY(TotalNumElements, 2);
    ChMatrixNM<double, 10, 12> MPROP;
    ChMatrixNM<int, 10, 7> MNUM;
    ChMatrixNM<int, 10, 1> NumLayer;
    double LayPROP[10][7][2];

    //------------------------------------------------
    //--------------- Element data--------------------
    //------------------------------------------------

    for (int i = 0; i < TotalNumElements; i++) {
        // Here, we define how many layers each element has
        // In this example of balanced shell, there are two layers
        // The fiber angle will be input a few lines below
        LayNum(i, 0) = 2;  // Each element has two layers

        // Node number of the 4 nodes which creates element i.
        // The nodes are distributed this way. First in the x direction for constant
        // y when max x is reached go to the next level for y by doing the same
        // distribution but for y+1 and keep doing until y limit is reached. Node
        // number start from 1.
        NumNodes(i, 0) = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        NumNodes(i, 1) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        NumNodes(i, 2) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        NumNodes(i, 3) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        // Element length
        ElemLengthXY(i, 0) = dx;
        ElemLengthXY(i, 1) = dy;
        if (MaxLayNum < LayNum(i, 0)) {
            MaxLayNum = LayNum(i, 0);
        }
    }
    //--------------------------------------------------------
    //  Fixing constraints, initial coordinates and velocities
    //--------------------------------------------------------

    for (int i = 0; i < TotalNumNodes; i++) {
        // Vector NDR selects which coordinates at each are constrained
        // For this model, we fully constrain the nodes along axis X = 0
        // 1 for constrained; 0 for unconstrained
        NDR(i, 0) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 1) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 2) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 3) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 4) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 5) = (i % (numDiv_x + 1) == 0) ? 1 : 0;  // This works just fine

        // COORDFlex are the initial coordinates for each node
        // The coordinates are parameterized as a function of the cylinder radius and
        // the number of elements chosen by the user

        COORDFlex(i, 0) = cylRadius * sin((i % (numDiv_x + 1)) * (CH_C_PI / 2) / numDiv_x);
        COORDFlex(i, 1) = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
        COORDFlex(i, 2) = cylRadius * (1 - cos((i % (numDiv_x + 1)) * (CH_C_PI / 2) / numDiv_x));
        COORDFlex(i, 3) = -sin(CH_C_PI / 2 / numDiv_x * (i % (numDiv_x + 1)));
        COORDFlex(i, 4) = 0;
        COORDFlex(i, 5) = cos(CH_C_PI / 2 / numDiv_x * (i % (numDiv_x + 1)));

        VELCYFlex(i, 0) = 0;
        VELCYFlex(i, 1) = 0;
        VELCYFlex(i, 2) = 0;
        VELCYFlex(i, 3) = 0;
        VELCYFlex(i, 4) = 0;
        VELCYFlex(i, 5) = 0;
    }
    //------------------------------------------------
    //------------- Read Layer Data-------------------
    //------------------------------------------------

    for (int i = 0; i < MaxLayNum; i++) {
        NumLayer(i, 0) = i + 1;
        // For each layer we introduce parameters in LayPROP
        //
        for (int j = 0; j < NumLayer(i, 0); j++) {
            LayPROP[i][j][0] = dz;  // Height of each layer
            if (j == 0) {
                LayPROP[i][j][1] = 20;  // For first layer, fiber angle 20 degrees
            }                           // Fiber angle of each ply
            else {
                LayPROP[i][j][1] = -20;  // For second layer, fiber angle -20 degrees
            }
            MNUM[i][j] = 1;  // Material_ID
                             // In this example one single material ID (same material properties for the two layers)
            if (MaxMNUM < MNUM(i, j))
                MaxMNUM = MNUM(i, j);
        }
    }
    //------------------------------------------------
    //------------ Input Material Data----------------
    //------------------------------------------------
    for (int i = 0; i < MaxMNUM; i++) {
        double nu_coef = 0.3;
        MTYPE = 2;  // The user must use orthotropic input (MTYPE=2)
        if (MTYPE == 2) {
            MPROP(i, 0) = 500;      // Density [kg/m3]
            MPROP(i, 1) = 6.0E+08;  // Ex//
            MPROP(i, 2) = 3.0E+08;  // Ey
            MPROP(i, 3) = 3.0E+08;  // Ez
            // Additional information for the Type 2 of Material.
            MPROP(i, 4) = 0.3;       // nuxy
            MPROP(i, 5) = 0.3;       // nuxz
            MPROP(i, 6) = 0.3;       // nuyz
            MPROP(i, 7) = 1.1538E8;  // MPROP(i, 1) / 2.0 / (1 + MPROP(i, 6));  // Gxy
            MPROP(i, 8) = 1.1538E8;  // MPROP(i, 2) / 2.0 / (1 + MPROP(i, 6));  // Gxz
            MPROP(i, 9) = 1.1538E8;  // MPROP(i, 3) / 2.0 / (1 + MPROP(i, 6));  // Gyz
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
    // A constrained node
    ChSharedPtr<ChNodeFEAxyzD> noderclamped(my_mesh->GetNode(0).DynamicCastTo<ChNodeFEAxyzD>());

    int elemcount = 0;
    while (elemcount < TotalNumElements) {
        ChSharedPtr<ChElementShellANCF> element(new ChElementShellANCF);
        // Save material data into InertFlexVec(98x1) at each layer
        ChMatrixNM<double, 98, 1> InertFlexVec;
        InertFlexVec.Reset();
        double TotalThickness;  // Element thickness: Summation of all layers' thickness
        TotalThickness = 0.0;
        int i = elemcount;
        for (int j = 0; j < NumLayer(LayNum(i, 0) - 1, 0); j++) {  // For each element, define material properties
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
        // Automatically selects the Gauss range to integrate inertia/internal forces over the thickness
        // Each layer's thickness will be integrated using 2 Gauss integration points
        // LimLow and LimHigh represent the thickness range for a given layer
        ChMatrixNM<double, 7, 2> GaussZRange;
        GaussZRange.Reset();
        double CurrentHeight = 0.0;
        for (int j = 0; j < NumLayer(LayNum(i, 0) - 1, 0); j++) {
            double LimLow = (CurrentHeight / TotalThickness - 0.5) * 2.0;
            CurrentHeight += LayPROP[LayNum(i, 0) - 1][j][0];
            double LimHigh = (CurrentHeight / TotalThickness - 0.5) * 2.0;
            GaussZRange(j, 0) = LimLow;
            GaussZRange(j, 1) = LimHigh;
        }
        // Now we give some parameters element by element
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
        element->SetAlphaDamp(0.0);                // Structural damping for this
        element->Setdt(0.001);                     // dt to calculate DampingCoefficient
        element->SetGravityOn(true);               // turn gravity on/off
        ChMatrixNM<double, 35, 1> StockAlpha_EAS;  // StockAlpha(5*7,1): Max #Layer is 7
        StockAlpha_EAS.Reset();
        element->SetStockAlpha(StockAlpha_EAS);
        my_mesh->AddElement(element);
        elemcount++;
    }
    // Switch off mesh class gravity: my_mesh still does not implement this element's gravity forces
    my_mesh->SetAutomaticGravity(false);
    // This is mandatory
    my_mesh->SetupInitial();
    // Remember to add the mesh to the system
    my_system.Add(my_mesh);
    // Perform a dynamic time integration:
    my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetIterLCPwarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetIterLCPmaxItersSpeed(100);
    my_system.SetIterLCPmaxItersStab(100);
    my_system.SetTolForce(1e-09);

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
    mystepper->SetTolerance(1e-08);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(false);  //

    /*utils::Data m_data;
    m_data.resize(4);
    for (size_t col = 0; col < 4; col++)
        m_data[col].resize(num_steps);
    utils::CSV_writer csv(" ");
    std::ifstream file2("UT_ANCFShellOrtGrav.txt");*/

    for (unsigned int it = 0; it < num_steps; it++) {
        my_system.DoStepDynamics(time_step);
        std::cout << "Time t = " << my_system.GetChTime() << "s \n";
        // std::cout << "nodetip->pos.z = " << nodetip->pos.z << "\n";
        // std::cout << "mystepper->GetNumIterations()= " << mystepper->GetNumIterations() << "\n";
        // Check vertical displacement of the shell tip
        double AbsVal = abs(nodetip->pos.z - FileInputMat[it][1]);
        if (AbsVal > precision) {
            std::cout << "Unit test check failed \n";
            system("pause");
            return 1;
        }

        /* // Code snippet to generate golden file
        m_data[0][it] = my_system.GetChTime();
        m_data[1][it] = nodetip->pos.z; // Note that z component is the second row
        m_data[2][it] = nodetip->pos.x;
        m_data[3][it] = nodetip->pos.y;
        csv << m_data[0][it] << m_data[1][it] << m_data[2][it] << m_data[3][it] << std::endl;
        csv.write_to_file("UT_ANCFShellOrtGrav.txt");*/
    }
    std::cout << "Unit test check succeeded \n";
    return 0;
}
