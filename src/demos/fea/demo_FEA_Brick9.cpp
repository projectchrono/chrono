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
// Authors: Bryan Peterson, Antonio Recuero
// =============================================================================
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
// =============================================================================

#include "chrono/solver/ChSolverMINRES.h"
#include "chrono_fea/ChElementBrick_9.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_mkl/ChSolverMKL.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace irr;

void AxialDynamics();
void BendingQuasiStatic();
void SwingingShell();
void SoilBin();
void SimpleBoxContact();
void ShellBrickContact();
void DPCapPress();

int main(int argc, char* argv[]) {
	DPCapPress();
    // ShellBrickContact();
    // SimpleBoxContact();
    // SoilBin();
    // AxialDynamics();
    // BendingQuasiStatic();
    // SwingingShell();
    return 0;
}

// Soil Bin case testing Drucker-Prager Cap model
void DPCapPress() {
	FILE* outputfile;
	ChSystemDEM my_system;
	my_system.UseMaterialProperties(false);
	my_system.SetAdhesionForceModel(ChSystemDEM::AdhesionForceModel::Constant);
	//my_system.SetContactForceModel(ChSystemDEM::ContactForceModel::PlainCoulomb);
	my_system.SetContactForceModel(ChSystemDEM::ContactForceModel::Hooke);
	my_system.Set_G_acc(ChVector<>(0, 0, 0));

	// Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
	ChIrrApp application(&my_system, L"9-Node, Large Deformation Brick Element", core::dimension2d<u32>(800, 600),
		false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
		core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

	GetLog() << "-----------------------------------------------------------------------\n";
	GetLog() << "-----------------------------------------------------------------------\n";
	GetLog() << "     9-Node, Large Deformation Brick Element with implicit integration \n";
	GetLog() << "-----------------------------------------------------------------------\n";

	// Create a mesh, that is a container for groups of elements and their referenced nodes.
	auto my_mesh = std::make_shared<ChMesh>();

	// Geometry of the plate
	double plate_lenght_x = 0.48;// 0.4;
	double plate_lenght_y = 0.48;// 0.4;
	double plate_lenght_z = 0.6;// 0.6;

	// Specification of the mesh
	int numDiv_x = 12;// 10;
	int numDiv_y = 12;// 10;
	int numDiv_z = 8;// 8;

	int N_x = numDiv_x + 1;
	int N_y = numDiv_y + 1;
	int N_z = numDiv_z + 1;

	// Number of elements in the z direction is considered as 1
	int TotalNumElements = numDiv_x * numDiv_y * numDiv_z;
	int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
	int TotalNumNodes = (numDiv_z + 1) * XYNumNodes + TotalNumElements;

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
			auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
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
		auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
			ChVector<>(0.0, 0.0, 0.0));
		node->SetMass(0);
		my_mesh->AddNode(node);
	}

	// Create an orthotropic material.
	double rho = 2149.0;
	ChVector<> E(54.1e6, 54.1e6, 54.1e6);// (1.379e7, 1.379e7, 1.379e7);
	ChVector<> nu(0.293021, 0.293021, 0.293021);// (0.3, 0.3, 0.3);
	//ChVector<> G(3.8461538e6, 3.8461538e6, 3.8461538e6);
	auto material = std::make_shared<ChContinuumElastic>();
	material->Set_RayleighDampingK(0.0);
	material->Set_RayleighDampingM(0.0);
	material->Set_density(rho);
	material->Set_E(E.x());
	// material->Set_G(G.x());
	material->Set_v(nu.x());

	// Read hardening parameter look-up table
	FILE* inputfile;
	char str1[100];
	int MAXCOUNT = 100;
	int RowN;

	//inputfile = fopen(GetChronoDataFile("fea/Hardening_parameter_table.INP").c_str(), "r");
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
	//Modified hardening parameter to be consistent with ABAQUS !!! 
	for (int i = 0; i < RowN; i++) {
		m_DPVector2(i) = (m_DPVector2(i) + 210926.0 / tan(51.7848 * 3.141592653589793 / 180.0)) / (0.5*tan(51.7848 * 3.141592653589793 / 180.0) + 1.0);
	}

	std::shared_ptr<ChMaterialSurfaceDEM> my_surfacematerial(new ChMaterialSurfaceDEM);
	my_surfacematerial->SetKn(3200);//(10e6);
	my_surfacematerial->SetKt(3200);//(10e6);
	my_surfacematerial->SetGn(32);// (10e3);
	my_surfacematerial->SetGt(32);// (10e3);

	std::shared_ptr<ChContactSurfaceNodeCloud> my_contactsurface(new ChContactSurfaceNodeCloud);
	my_mesh->AddContactSurface(my_contactsurface);
	my_contactsurface->AddAllNodes(0.005);


	my_contactsurface->SetMaterialSurface(my_surfacematerial);
	ChMatrixNM<double, 9, 8> CCPInitial;
	for (int k = 0; k < 8; k++) {
		CCPInitial(0, k) = 1;
		CCPInitial(4, k) = 1;
		CCPInitial(8, k) = 1;
	}
	int jj = -1;
	int kk;
	// Create the elements
	for (int i = 0; i < TotalNumElements; i++) {
		if (i % (numDiv_x*numDiv_y) == 0) {
			jj++;
			kk = 0;
		}
		// Adjacent nodes
		int node0 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + jj*(N_x*N_y);
		int node1 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + 1 + jj*(N_x*N_y);
		int node2 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + 1 + N_x + jj*(N_x*N_y);
		int node3 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + N_x + jj*(N_x*N_y);
		int node4 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + XYNumNodes + jj*(N_x*N_y);
		int node5 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + 1 + XYNumNodes + jj*(N_x*N_y);
		int node6 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + 1 + N_x + XYNumNodes + jj*(N_x*N_y);
		int node7 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + N_x + XYNumNodes + jj*(N_x*N_y);
		int node8 = (numDiv_z + 1) * XYNumNodes + i;

		// Create the element and set its nodes.
		auto element = std::make_shared<ChElementBrick_9>();
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
		element->SetDimensions(ChVector<>(dx, dy, dz));

		// Add a single layers with a fiber angle of 0 degrees.
		element->SetMaterial(material);

		// Set other element properties
		element->SetAlphaDamp(0.0);    // Structural damping for this element
		element->SetGravityOn(false);  // turn internal gravitational force calculation off
		element->SetDPIterationNo(50); // Set maximum number of iterations for Drucker-Prager Newton-Raphson
		element->SetDPYieldTol(1e-5);  // Set stop tolerance for Drucker-Prager Newton-Raphson
		element->SetStrainFormulation(ChElementBrick_9::Hencky);
		element->SetPlasticityFormulation(ChElementBrick_9::DruckerPrager_Cap);
		if (element->GetStrainFormulation() == ChElementBrick_9::Hencky) {
			element->SetPlasticity(Plasticity);
			if (Plasticity) {
				element->SetYieldStress(210926.0);
				element->SetHardeningSlope(0.0);
				element->SetCCPInitial(CCPInitial);
				if (element->GetPlasticityFormulation() == ChElementBrick_9::DruckerPrager) {
					element->SetFriction(10.0);
					element->SetDilatancy(0.0);
					element->SetDPType(3);
				}
				if (element->GetPlasticityFormulation() == ChElementBrick_9::DruckerPrager_Cap) {
					element->SetFriction(51.7848);
					element->SetDilatancy(51.7848);
					element->SetDPType(3);
					element->SetDPVector1(m_DPVector1);
					element->SetDPVector2(m_DPVector2);
					element->SetDPVectorSize(RowN);
					element->SetDPCapBeta(0.5*tan(51.7848 * 3.141592653589793 / 180.0));
				}
			}
		}

		// Add element to mesh
		my_mesh->AddElement(element);
		kk++;
	}

	// Add the mesh to the system
	my_system.Add(my_mesh);

	// Mark completion of system construction
	my_system.SetupInitial();

	// -------------------------------------
	// Options for visualization in irrlicht
	// -------------------------------------

	auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
	mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
	mvisualizemesh->SetShrinkElements(true, 0.85);
	mvisualizemesh->SetSmoothFaces(true);
	my_mesh->AddAsset(mvisualizemesh);

	auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
	mvisualizemeshref->SetWireframe(true);
	mvisualizemeshref->SetDrawInUndeformedReference(true);
	my_mesh->AddAsset(mvisualizemeshref);

	auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
	mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
	mvisualizemeshC->SetSymbolsThickness(0.004);
	my_mesh->AddAsset(mvisualizemeshC);

	auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
	mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
	mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
	mvisualizemeshD->SetSymbolsScale(1);
	mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
	mvisualizemeshD->SetZbufferHide(false);
	my_mesh->AddAsset(mvisualizemeshD);

	auto mvisualizemeshcoll = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizemeshcoll->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_CONTACTSURFACES);
	mvisualizemeshcoll->SetWireframe(true);
	mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
	my_mesh->AddAsset(mvisualizemeshcoll);

	application.AssetBindAll();
	application.AssetUpdateAll();

	// Use the MKL Solver
	auto mkl_solver = std::make_shared<ChSolverMKL<>>();
	my_system.SetSolver(mkl_solver);
    mkl_solver->SetSparsityPatternLock(true);
	my_system.Update();

	// Set the time integrator parameters
	my_system.SetTimestepperType(ChTimestepper::Type::HHT);
	auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
	mystepper->SetAlpha(0.0);
	mystepper->SetMaxiters(25);//20
	mystepper->SetAbsTolerances(5e-5, 1e-2);//1e-5
	mystepper->SetMode(ChTimestepperHHT::POSITION);
	mystepper->SetVerbose(true);
	mystepper->SetScaling(true);
	application.SetTimestep(timestep);

	my_system.Setup();
	my_system.Update();

	outputfile = fopen("SolidBenchmark.txt", "w");

	double ChTime = 0.0;
	double start = std::clock();
	int Iter = 0;
	application.SetPaused(true);

	double force = 0.0;

	while (application.GetDevice()->run() && (my_system.GetChTime() <= 0.5)) {
		application.BeginScene();
		application.DrawAll();
		application.DoStep();

		int offset_top = N_x*N_y*numDiv_z;
		int offset_mid = (numDiv_y / 2 - 1)*N_x;
		int inc = 0;
		//node force
		force = -1700 * std::sin(my_system.GetChTime() * CH_C_PI);
		for (inc = 0; inc < numDiv_x / 4; inc++) {
			for (int ii = 0; ii < numDiv_x / 2 + 1; ii++) {
				auto nodeforce = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(offset_top + offset_mid + N_y*inc + numDiv_x / 4 + ii));
				nodeforce->SetForce(ChVector<>(0.0, 0.0, force));
			}
		}

		//my_system.DoStepDynamics(timestep);
		application.EndScene();
		Iter += mystepper->GetNumIterations();
		GetLog() << "t = " << my_system.GetChTime() << "\n";
		GetLog() << "Last it: " << mystepper->GetNumIterations() << "\n";
		if (!application.GetPaused()) {
			fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
			inc = inc / 2;
			for (int ii = 0; ii < N_x; ii++) {
				auto nodeforce = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(offset_top + offset_mid + N_y*inc + ii));
				fprintf(outputfile, "%15.7e  ", nodeforce->GetPos().x());
				fprintf(outputfile, "%15.7e  ", nodeforce->GetPos().y());
				fprintf(outputfile, "%15.7e  ", nodeforce->GetPos().z());
			}
			fprintf(outputfile, "\n  ");
		}
	}
	double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	GetLog() << "Simulation Time: " << duration << "\n";
	GetLog() << "Force Time: " << my_mesh->GetTimeInternalForces() << "\n";
	GetLog() << "Jacobian Time: " << my_mesh->GetTimeJacobianLoad() << "\n";
	GetLog() << "Solver Time: " << my_system.GetTimerSolver() << "\n";
	GetLog() << Iter << "\n";
}

// Test1 Case
void ShellBrickContact() {
    FILE* outputfile;
    ChSystemDEM my_system;
    my_system.UseMaterialProperties(false);
    my_system.SetAdhesionForceModel(ChSystemDEM::AdhesionForceModel::Constant);
    my_system.SetContactForceModel(ChSystemDEM::ContactForceModel::PlainCoulomb);
    my_system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
    ChIrrApp application(&my_system, L"9-Node, Large Deformation Brick Element", core::dimension2d<u32>(800, 600),
                         false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
                                 core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

    GetLog() << "-----------------------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------------------\n";
    GetLog() << "     9-Node, Large Deformation Brick Element with implicit integration \n";
    GetLog() << "-----------------------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Geometry of the bricked plate.
    double plate_lenght_x = 0.5;
    double plate_lenght_y = 0.5;
    double plate_lenght_z = 0.125;

    // Geometry of the ANCF shell.
    double shell_lenght_x = 0.1;
    double shell_lenght_y = 0.1;
    double shell_lenght_z = 0.01;

    // Specification of the mesh for bricked plate.
    int numDiv_x = 8;
    int numDiv_y = 8;
    int numDiv_z = 2;
    // Specification of the mesh for ANCF shell.
    int SnumDiv_x = 4;
    int SnumDiv_y = 4;
    int SnumDiv_z = 1;

    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;

    int SN_x = SnumDiv_x + 1;
    int SN_y = SnumDiv_y + 1;
    int SN_z = SnumDiv_z + 1;

    // Number of elements in the z direction is considered as 1.
    int TotalNumElements = numDiv_x * numDiv_y * numDiv_z;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    int TotalNumNodes = (numDiv_z + 1) * XYNumNodes + TotalNumElements;

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    double Sdx = shell_lenght_x / SnumDiv_x;
    double Sdy = shell_lenght_y / SnumDiv_y;
    double Sdz = shell_lenght_z / SnumDiv_z;

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
            auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
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
        auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
                                                    ChVector<>(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    for (int k = 0; k < 25; k++) {  // Create a 5 by 5 flat ANCF shell
        // Node location
        double loc_x = (k % (SnumDiv_x + 1)) * Sdx + 0.15;
        double loc_y = (k / (SnumDiv_x + 1)) % (SnumDiv_y + 1) * Sdy + 0.15;
        double loc_z = 0.13;
        auto nodeshell = std::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(0.0, 0.0, 1.0));
        nodeshell->SetMass(0);
        my_mesh->AddNode(nodeshell);
    }

    // Get a handle to the tip node.
    auto nodetip1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode((numDiv_z + 1) * XYNumNodes - XYNumNodes));
    // Create an orthotropic material.
    double rho = 200.0;
    ChVector<> E(1.379e7, 1.379e7, 1.379e7);
    ChVector<> nu(0.3, 0.3, 0.3);
    // ChVector<> G(3.8461538e6, 3.8461538e6, 3.8461538e6);
    auto material = std::make_shared<ChContinuumElastic>();
    material->Set_RayleighDampingK(0.0);
    material->Set_RayleighDampingM(0.0);
    material->Set_density(rho);
    material->Set_E(E.x());
    // material->Set_G(G.x());
    material->Set_v(nu.x());
    double rhoS = 8000;
    ChVector<> ES(2.1e10, 2.1e10, 2.1e10);                 // Modulus of elasticity
    ChVector<> nuS(0.3, 0.3, 0.3);                         // Poisson ratio
    ChVector<> GS(8.0769231e9, 8.0769231e9, 8.0769231e9);  // Modulus of rigidity
    auto mat = std::make_shared<ChMaterialShellANCF>(rhoS, ES, nuS, GS);
    std::shared_ptr<ChMaterialSurfaceDEM> my_surfacematerial(new ChMaterialSurfaceDEM);
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
    int jj = -1;
    int kk;

    // Create the elements for the bricked plate (made up of 9-node brick elements).
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
        auto element = std::make_shared<ChElementBrick_9>();
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
        element->SetDimensions(ChVector<>(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.0);     // Structural damping for this element
        element->SetGravityOn(true);    // turn internal gravitational force calculation off
        element->SetDPIterationNo(50);  // Set maximum number of iterations for Drucker-Prager Newton-Raphson
        element->SetDPYieldTol(1e-8);   // Set stop tolerance for Drucker-Prager Newton-Raphson
        element->SetStrainFormulation(ChElementBrick_9::Hencky);
        element->SetPlasticityFormulation(ChElementBrick_9::DruckerPrager);
        if (element->GetStrainFormulation() == ChElementBrick_9::Hencky) {
            element->SetPlasticity(Plasticity);
            if (Plasticity) {
                element->SetYieldStress(0.0);
                element->SetHardeningSlope(5e5);
                element->SetCCPInitial(CCPInitial);
                if (element->GetPlasticityFormulation() == ChElementBrick_9::DruckerPrager) {
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
    for (int ii = 0; ii < SnumDiv_x * SnumDiv_y; ii++) {
        int node0 = (ii / (SnumDiv_x)) * (SN_x) + ii % SnumDiv_x;
        int node1 = (ii / (SnumDiv_x)) * (SN_x) + ii % SnumDiv_x + 1;
        int node2 = (ii / (SnumDiv_x)) * (SN_x) + ii % SnumDiv_x + 1 + SN_x;
        int node3 = (ii / (SnumDiv_x)) * (SN_x) + ii % SnumDiv_x + SN_x;

        auto elementshell = std::make_shared<ChElementShellANCF>();
        elementshell->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes + node0)),
                               std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes + node1)),
                               std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes + node2)),
                               std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes + node3)));

        elementshell->SetDimensions(Sdx, Sdy);
        elementshell->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);
        elementshell->SetAlphaDamp(0.0);    // Structural damping for this element
        elementshell->SetGravityOn(false);  // turn internal gravitational force calculation off
        my_mesh->AddElement(elementshell);
    }

    // Add the mesh to the system.
    my_system.Add(my_mesh);

    // std::shared_ptr<ChContactSurfaceNodeCloud> my_contactsurface(new ChContactSurfaceNodeCloud);
    std::shared_ptr<ChContactSurfaceMesh> my_contactsurface(new ChContactSurfaceMesh);
    my_mesh->AddContactSurface(my_contactsurface);
    // my_contactsurface->AddAllNodes(0.001);
    my_contactsurface->AddFacesFromBoundary(0.005);
    my_contactsurface->SetMaterialSurface(my_surfacematerial);

    my_system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));
    my_mesh->SetAutomaticGravity(false);

    // Mark completion of system construction
    my_system.SetupInitial();

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddAsset(mvisualizemeshC);

    auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizemeshD);

    auto mvisualizemeshcoll = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshcoll->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddAsset(mvisualizemeshcoll);

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Use the MKL Solver
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    my_system.SetSolver(mkl_solver);
    mkl_solver->SetSparsityPatternLock(true);
    my_system.Update();

    // Set the time integrator parameters
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxiters(20);
    mystepper->SetAbsTolerances(1e-8, 1e-2);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetVerbose(true);
    mystepper->SetScaling(true);
    application.SetTimestep(timestep);

    my_system.Setup();
    my_system.Update();

    outputfile = fopen("SolidBenchmark.txt", "w");
    fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
    fprintf(outputfile, "\n  ");

    double ChTime = 0.0;
    double start = std::clock();
    int Iter = 0;
    int timecount = 0;
    application.SetPaused(true);
    while (application.GetDevice()->run() && (my_system.GetChTime() <= 1.0)) {
        if (my_system.GetChTime() < 0.5) {
            for (int ii = 0; ii < 25; ii++) {
                auto Snode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes + ii));
                Snode->SetForce(ChVector<>(0.0, 0.0, -100.0 * timecount * timestep));
            }
        } else {
            for (int ii = 0; ii < 25; ii++) {
                auto Snode = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes + ii));
                Snode->SetForce(ChVector<>(0.0, 0.0, 10.0 * timecount * timestep));
            }
        }
        application.BeginScene();
        application.DrawAll();
        application.DoStep();

        // my_system.DoStepDynamics(timestep);
        application.EndScene();
        Iter += mystepper->GetNumIterations();
        GetLog() << "t = " << my_system.GetChTime() << "\n";
        GetLog() << "Last it: " << mystepper->GetNumIterations() << "\n";
        // GetLog() << "Body Contact F: " << Plate->GetContactForce() << "\n";
        GetLog() << nodetip1->GetPos().x() << "\n";
        GetLog() << nodetip1->GetPos().y() << "\n";
        GetLog() << nodetip1->GetPos().z() << "\n";
        GetLog() << nodetip1->GetPos_dt().z() << "\n";
        if (!application.GetPaused() && timecount % 100 == 0) {
            fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
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
    }
    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    GetLog() << "Simulation Time: " << duration << "\n";
    GetLog() << "Force Time: " << my_mesh->GetTimeInternalForces() << "\n";
    GetLog() << "Jacobian Time: " << my_mesh->GetTimeJacobianLoad() << "\n";
    GetLog() << "Solver Time: " << my_system.GetTimerSolver() << "\n";
    GetLog() << Iter << "\n";
}

// Test Case
void SimpleBoxContact() {
    FILE* outputfile;
    ChSystemDEM my_system;
    my_system.UseMaterialProperties(false);
    my_system.SetAdhesionForceModel(ChSystemDEM::AdhesionForceModel::Constant);
    my_system.SetContactForceModel(ChSystemDEM::ContactForceModel::PlainCoulomb);
    my_system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
    ChIrrApp application(&my_system, L"9-Node, Large Deformation Brick Element", core::dimension2d<u32>(800, 600),
                         false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
                                 core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

    GetLog() << "-----------------------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------------------\n";
    GetLog() << "     9-Node, Large Deformation Brick Element with implicit integration \n";
    GetLog() << "-----------------------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

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
    int N_z = numDiv_z + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y * numDiv_z;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    int TotalNumNodes = (numDiv_z + 1) * XYNumNodes + TotalNumElements;

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
            auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
                                                    ChVector<>(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode((numDiv_z + 1) * XYNumNodes - XYNumNodes));
    // Create an orthotropic material.
    double rho = 8000.0;
    ChVector<> E(200e9, 200e9, 200e9);
    ChVector<> nu(0.3, 0.3, 0.3);
    // ChVector<> G(3.8461538e6, 3.8461538e6, 3.8461538e6);
    auto material = std::make_shared<ChContinuumElastic>();
    material->Set_RayleighDampingK(0.0);
    material->Set_RayleighDampingM(0.0);
    material->Set_density(rho);
    material->Set_E(E.x());
    // material->Set_G(G.x());
    material->Set_v(nu.x());
    std::shared_ptr<ChMaterialSurfaceDEM> my_surfacematerial(new ChMaterialSurfaceDEM);
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
    int jj = -1;
    int kk;
    // Create the elements
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
        auto element = std::make_shared<ChElementBrick_9>();
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
        element->SetDimensions(ChVector<>(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.0);     // Structural damping for this element
        element->SetGravityOn(true);    // turn internal gravitational force calculation off
        element->SetDPIterationNo(50);  // Set maximum number of iterations for Drucker-Prager Newton-Raphson
        element->SetDPYieldTol(1e-8);   // Set stop tolerance for Drucker-Prager Newton-Raphson
        element->SetStrainFormulation(ChElementBrick_9::Hencky);
        element->SetPlasticityFormulation(ChElementBrick_9::DruckerPrager);
        if (element->GetStrainFormulation() == ChElementBrick_9::Hencky) {
            element->SetPlasticity(Plasticity);
            if (Plasticity) {
                element->SetYieldStress(1e5);
                element->SetHardeningSlope(5e5);
                element->SetCCPInitial(CCPInitial);
                if (element->GetPlasticityFormulation() == ChElementBrick_9::DruckerPrager) {
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
    my_system.Add(my_mesh);

    // std::shared_ptr<ChContactSurfaceNodeCloud> my_contactsurface(new ChContactSurfaceNodeCloud);
    std::shared_ptr<ChContactSurfaceMesh> my_contactsurface(new ChContactSurfaceMesh);
    my_mesh->AddContactSurface(my_contactsurface);
    // my_contactsurface->AddAllNodes(0.0);
    my_contactsurface->AddFacesFromBoundary(0.0);
    my_contactsurface->SetMaterialSurface(my_surfacematerial);

    double plate_w = 0.1;
    double plate_l = 0.1;
    double plate_h = 0.1;
    auto Plate = std::make_shared<ChBodyEasyBox>(plate_l, plate_w, plate_h, 1000, true);
    my_system.Add(Plate);
    Plate->SetBodyFixed(true);
    Plate->SetPos(ChVector<>(0.025, 0.025, -0.0015 - plate_h / 2));
    Plate->SetRot(ChQuaternion<>(1.0, 0.0, 0.0, 0.0));
    Plate->SetMaterialSurface(my_surfacematerial);
    // Plate->SetPos_dt(ChVector<>(0.0, 0.0, -0.1));

    my_system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));
    my_mesh->SetAutomaticGravity(false);

    // Mark completion of system construction
    my_system.SetupInitial();

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.99);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddAsset(mvisualizemeshC);

    auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizemeshD);

    auto mvisualizemeshcoll = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshcoll->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
    my_mesh->AddAsset(mvisualizemeshcoll);

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Use the MKL Solver
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    my_system.SetSolver(mkl_solver);
    mkl_solver->SetSparsityPatternLock(true);
    my_system.Update();

    // Set the time integrator parameters
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxiters(20);
    mystepper->SetAbsTolerances(1e-8, 1e-2);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetVerbose(true);
    mystepper->SetScaling(true);
    application.SetTimestep(timestep);

    my_system.Setup();
    my_system.Update();

    outputfile = fopen("SolidBenchmark.txt", "w");
    fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
    fprintf(outputfile, "\n  ");

    double ChTime = 0.0;
    double start = std::clock();
    int Iter = 0;
    int timecount = 0;
    application.SetPaused(true);
    while (application.GetDevice()->run() && (my_system.GetChTime() <= 1.0)) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();

        // my_system.DoStepDynamics(timestep);
        application.EndScene();
        Iter += mystepper->GetNumIterations();
        // GetLog() << "t = " << my_system.GetChTime() << "\n";
        // GetLog() << "Last it: " << mystepper->GetNumIterations() << "\n";
        // GetLog() << "Plate Pos: " << Plate->GetPos();
        // GetLog() << "Plate Vel: " << Plate->GetPos_dt();
        // GetLog() << "Body Contact F: " << Plate->GetContactForce() << "\n";
        // GetLog() << nodetip1->GetPos().x() << "\n";
        // GetLog() << nodetip1->GetPos().y() << "\n";
        // GetLog() << nodetip1->GetPos().z() << "\n";
        // GetLog() << nodetip1->GetPos_dt().z() << "\n";
        if (!application.GetPaused() && timecount % 100 == 0) {
            fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
            for (int in = 0; in < XYNumNodes; in++) {
                auto nodetest = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(in));
                fprintf(outputfile, "%15.7e  ", nodetest->GetPos().x());
                fprintf(outputfile, "%15.7e  ", nodetest->GetPos().y());
                fprintf(outputfile, "%15.7e  ", nodetest->GetPos().z());
            }
            fprintf(outputfile, "%15.7e  ", Plate->GetContactForce().z());
            fprintf(outputfile, "\n  ");
        }
        timecount++;
    }
    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    GetLog() << "Simulation Time: " << duration << "\n";
    GetLog() << "Force Time: " << my_mesh->GetTimeInternalForces() << "\n";
    GetLog() << "Jacobian Time: " << my_mesh->GetTimeJacobianLoad() << "\n";
    GetLog() << "Solver Time: " << my_system.GetTimerSolver() << "\n";
    GetLog() << Iter << "\n";
}

// SoilBin Dynamic
void SoilBin() {
	FILE* outputfile;
	ChSystemDEM my_system;
	my_system.UseMaterialProperties(false);
	my_system.SetAdhesionForceModel(ChSystemDEM::AdhesionForceModel::Constant);
	//my_system.SetContactForceModel(ChSystemDEM::ContactForceModel::PlainCoulomb);
	my_system.SetContactForceModel(ChSystemDEM::ContactForceModel::Hooke);
	my_system.Set_G_acc(ChVector<>(0, 0, 0));

	// Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
	ChIrrApp application(&my_system, L"9-Node, Large Deformation Brick Element", core::dimension2d<u32>(800, 600),
		false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
		core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

	GetLog() << "-----------------------------------------------------------------------\n";
	GetLog() << "-----------------------------------------------------------------------\n";
	GetLog() << "     9-Node, Large Deformation Brick Element with implicit integration \n";
	GetLog() << "-----------------------------------------------------------------------\n";

	// Create a mesh, that is a container for groups of elements and their referenced nodes.
	auto my_mesh = std::make_shared<ChMesh>();

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
	int N_z = numDiv_z + 1;

	// Number of elements in the z direction is considered as 1
	int TotalNumElements = numDiv_x * numDiv_y * numDiv_z;
	int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
	int TotalNumNodes = (numDiv_z + 1) * XYNumNodes + TotalNumElements;

	// For uniform mesh
	double dx = plate_lenght_x / numDiv_x;
	double dy = plate_lenght_y / numDiv_y;
	double dz = plate_lenght_z / numDiv_z;
	bool Plasticity = true;
	double timestep = 5e-5;//

	// Create and add the nodes
	for (int j = 0; j <= numDiv_z; j++) {
		for (int i = 0; i < XYNumNodes; i++) {
			// Node location
			double loc_x = (i % (numDiv_x + 1)) * dx;
			double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
			double loc_z = j * dz;
			// Create the node
			auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
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
		auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
			ChVector<>(0.0, 0.0, 0.0));
		node->SetMass(0);
		my_mesh->AddNode(node);
	}

	auto nodecenter = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(N_x*N_y*numDiv_z + N_x*(numDiv_y / 2) + (N_x - 1) / 2));

	// Create an orthotropic material.
	double rho = 200.0;
	ChVector<> E(1.379e7, 1.379e7, 1.379e7);
	ChVector<> nu(0.3, 0.3, 0.3);
	//ChVector<> G(3.8461538e6, 3.8461538e6, 3.8461538e6);
	auto material = std::make_shared<ChContinuumElastic>();
	material->Set_RayleighDampingK(0.0);
	material->Set_RayleighDampingM(0.0);
	material->Set_density(rho);
	material->Set_E(E.x());
	// material->Set_G(G.x());
	material->Set_v(nu.x());
	std::shared_ptr<ChMaterialSurfaceDEM> my_surfacematerial(new ChMaterialSurfaceDEM);
	my_surfacematerial->SetKn(0.2e4);//0.2e6
	my_surfacematerial->SetKt(0.2e4);//0.2e6
	my_surfacematerial->SetGn(0.2e2);//0.2e4
	my_surfacematerial->SetGt(0.2e2);//0.2e4
	ChMatrixNM<double, 9, 8> CCPInitial;
	for (int k = 0; k < 8; k++) {
		CCPInitial(0, k) = 1;
		CCPInitial(4, k) = 1;
		CCPInitial(8, k) = 1;
	}
	int jj = -1;
	int kk;
	// Create the elements
	for (int i = 0; i < TotalNumElements; i++) {
		if (i % (numDiv_x*numDiv_y) == 0) {
			jj++;
			kk = 0;
		}
		// Adjacent nodes
		int node0 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + jj*(N_x*N_y);
		int node1 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + 1 + jj*(N_x*N_y);
		int node2 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + 1 + N_x + jj*(N_x*N_y);
		int node3 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + N_x + jj*(N_x*N_y);
		int node4 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + XYNumNodes + jj*(N_x*N_y);
		int node5 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + 1 + XYNumNodes + jj*(N_x*N_y);
		int node6 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + 1 + N_x + XYNumNodes + jj*(N_x*N_y);
		int node7 = (kk / (numDiv_x)) * (N_x)+kk % numDiv_x + N_x + XYNumNodes + jj*(N_x*N_y);
		int node8 = (numDiv_z + 1) * XYNumNodes + i;

		// Create the element and set its nodes.
		auto element = std::make_shared<ChElementBrick_9>();
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
		element->SetDimensions(ChVector<>(dx, dy, dz));

		// Add a single layers with a fiber angle of 0 degrees.
		element->SetMaterial(material);

		// Set other element properties
		element->SetAlphaDamp(5e-4);    // Structural damping for this element
		element->SetGravityOn(false);  // turn internal gravitational force calculation off
		element->SetDPIterationNo(50); // Set maximum number of iterations for Drucker-Prager Newton-Raphson
		element->SetDPYieldTol(1e-8);  // Set stop tolerance for Drucker-Prager Newton-Raphson
		element->SetStrainFormulation(ChElementBrick_9::Hencky);
		element->SetPlasticityFormulation(ChElementBrick_9::DruckerPrager);
		if (element->GetStrainFormulation() == ChElementBrick_9::Hencky) {
			element->SetPlasticity(Plasticity);
			if (Plasticity) {
				element->SetYieldStress(10000.0);
				element->SetHardeningSlope(5000);
				element->SetCCPInitial(CCPInitial);
				if (element->GetPlasticityFormulation() == ChElementBrick_9::DruckerPrager) {
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
	my_system.Add(my_mesh);

	std::shared_ptr<ChContactSurfaceNodeCloud> my_contactsurface(new ChContactSurfaceNodeCloud);///node cloud
	//std::shared_ptr<ChContactSurfaceMesh> my_contactsurface(new ChContactSurfaceMesh);///surface mesh

	my_mesh->AddContactSurface(my_contactsurface);

	//my_contactsurface->AddFacesFromBoundary(0.0002);//0.001///surface mesh
	my_contactsurface->AddAllNodes(0.006);//0.001///node cloud

	my_contactsurface->SetMaterialSurface(my_surfacematerial);

	// Creat punch
	double plate_w = 0.2;//0.15
	double plate_l = 0.2;//0.15
	double plate_h = 0.1;
	auto Plate = std::make_shared<ChBodyEasyBox>(plate_l, plate_w, plate_h, 1000, true);
	my_system.Add(Plate);
	Plate->SetBodyFixed(false);
	Plate->SetPos(ChVector<>(0.2, 0.2, 0.6001 + plate_h / 2));
	Plate->SetRot(ChQuaternion<>(1.0, 0.0, 0.0, 0.0));
	Plate->SetPos_dt(ChVector<>(0.0, 0.0, 0.0));
	Plate->SetRot_dt(ChQuaternion<>(0.0, 0.0, 0.0, 0.0));
	Plate->SetMass(1.2265625);
	Plate->SetMaterialSurface(my_surfacematerial);

	//// Create ground body
	auto Ground = std::make_shared<ChBody>();
	Ground->SetBodyFixed(true);
	Ground->SetPos(ChVector<>(0.0, 0.0, -0.02));
	Ground->SetRot(ChQuaternion<>(1.0, 0.0, 0.0, 0.0));
	my_system.Add(Ground);

	my_system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));
	my_mesh->SetAutomaticGravity(false);

	std::shared_ptr<ChLinkLockPointPlane> constraintLateral;
	std::shared_ptr<ChLinkLockPointPlane> constraintLongitudinal;

	////// Constrain only the lateral displacement of the Rim
	constraintLateral = std::make_shared<ChLinkLockPointPlane>();
	my_system.AddLink(constraintLateral);
	constraintLateral->Initialize(Plate, Ground, ChCoordsys<>(Plate->GetPos(), Q_from_AngX(CH_C_PI_2)));

	constraintLongitudinal = std::make_shared<ChLinkLockPointPlane>();
	my_system.AddLink(constraintLongitudinal);
	constraintLongitudinal->Initialize(Plate, Ground, ChCoordsys<>(Plate->GetPos(), Q_from_AngY(CH_C_PI_2)));


	// Mark completion of system construction
	my_system.SetupInitial();

	// -------------------------------------
	// Options for visualization in irrlicht
	// -------------------------------------

	auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
	mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
	mvisualizemesh->SetShrinkElements(true, 0.85);
	mvisualizemesh->SetSmoothFaces(true);
	my_mesh->AddAsset(mvisualizemesh);

	auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
	mvisualizemeshref->SetWireframe(true);
	mvisualizemeshref->SetDrawInUndeformedReference(true);
	my_mesh->AddAsset(mvisualizemeshref);

	auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
	mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
	mvisualizemeshC->SetSymbolsThickness(0.004);
	my_mesh->AddAsset(mvisualizemeshC);

	auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
	mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
	mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
	mvisualizemeshD->SetSymbolsScale(1);
	mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
	mvisualizemeshD->SetZbufferHide(false);
	my_mesh->AddAsset(mvisualizemeshD);

	auto mvisualizemeshcoll = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizemeshcoll->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_CONTACTSURFACES);
	mvisualizemeshcoll->SetWireframe(true);
	mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1, 0.5, 0));
	my_mesh->AddAsset(mvisualizemeshcoll);

	application.AssetBindAll();
	application.AssetUpdateAll();

	// Use the MKL Solver
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    my_system.SetSolver(mkl_solver);
    mkl_solver->SetSparsityPatternLock(true);
    my_system.Update();

	// Set the time integrator parameters
	my_system.SetTimestepperType(ChTimestepper::Type::HHT);
	auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
	mystepper->SetAlpha(0.0);
	mystepper->SetMaxiters(20);
	mystepper->SetAbsTolerances(1e-8, 1e-2);
	mystepper->SetMode(ChTimestepperHHT::POSITION);
	mystepper->SetVerbose(true);
	mystepper->SetScaling(true);
	application.SetTimestep(timestep);

	my_system.Setup();
	my_system.Update();

	outputfile = fopen("SolidBenchmark0.txt", "w");
	fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
	for (int ii = 0; ii < N_x; ii++) {
		auto nodetest = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(N_x*N_y*numDiv_z + N_x*(numDiv_y / 2) + ii));
		fprintf(outputfile, "%15.7e  ", nodetest->GetPos().x());
		fprintf(outputfile, "%15.7e  ", nodetest->GetPos().y());
		fprintf(outputfile, "%15.7e  ", nodetest->GetPos().z());
	}
	fprintf(outputfile, "\n  ");


	double ChTime = 0.0;
	double start = std::clock();
	int Iter = 0;
	application.SetPaused(true);
	while (application.GetDevice()->run() && (my_system.GetChTime() <= 1.0)) {
		Plate->Empty_forces_accumulators();
		Plate->Accumulate_force(ChVector<>(0.0, 0.0, -1500.0*sin(my_system.GetChTime()*CH_C_PI)), Plate->GetPos(), false);
		Plate->SetRot(ChQuaternion<>(1.0, 0.0, 0.0, 0.0));

		application.BeginScene();
		application.DrawAll();
		application.DoStep();

		//my_system.DoStepDynamics(timestep);
		application.EndScene();
		Iter += mystepper->GetNumIterations();
		GetLog() << "t = " << my_system.GetChTime() << "\n";
		GetLog() << "Last it: " << mystepper->GetNumIterations() << "\n";
		GetLog() << "Plate Pos: " << Plate->GetPos();
		GetLog() << "Plate Vel: " << Plate->GetPos_dt();
		GetLog() << "Plate Rot: " << Plate->GetRot();
		GetLog() << "Plate Rot_v: " << Plate->GetRot_dt();
		GetLog() << "Body Contact F: " << Plate->GetContactForce() << "\n";
		GetLog() << nodecenter->GetPos().x() << "\n";
		GetLog() << nodecenter->GetPos().y() << "\n";
		GetLog() << nodecenter->GetPos().z() << "\n";
		if (!application.GetPaused()) {
			fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
			for (int ii = 0; ii < N_x; ii++) {
				auto nodetest = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(N_x*N_y*numDiv_z + N_x*(numDiv_y / 2) + ii));
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
	}
	double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	GetLog() << "Simulation Time: " << duration << "\n";
	GetLog() << "Force Time: " << my_mesh->GetTimeInternalForces() << "\n";
	GetLog() << "Jacobian Time: " << my_mesh->GetTimeJacobianLoad() << "\n";
	GetLog() << "Solver Time: " << my_system.GetTimerSolver() << "\n";
	GetLog() << Iter << "\n";
}

// Axial Dynamic
void AxialDynamics() {
    FILE* outputfile;
    ChSystem my_system;
    my_system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
    // ChIrrApp application(&my_system, L"9-Node, Large Deformation Brick Element", core::dimension2d<u32>(800, 600),
    //                     false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    // application.AddTypicalLogo();
    // application.AddTypicalSky();
    // application.AddTypicalLights();
    // application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
    //                             core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

    GetLog() << "-----------------------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------------------\n";
    GetLog() << "     9-Node, Large Deformation Brick Element with implicit integration \n";
    GetLog() << "-----------------------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 0.05;
    double plate_lenght_z = 0.05;

    // Specification of the mesh
    int numDiv_x = 20;
    int numDiv_y = 1;
    int numDiv_z = 1;

    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    int TotalNumNodes = 2 * XYNumNodes + TotalNumElements;

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
            auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
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
        auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
                                                    ChVector<>(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    double force = 0.0;
    // Get a handle to the tip node.
    auto nodetip1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));
    auto nodetip2 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1 - numDiv_x - 1));
    auto nodetip3 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(XYNumNodes - 1));
    auto nodetip4 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(XYNumNodes - 1 - numDiv_x - 1));
    nodetip1->SetForce(ChVector<>(0.0, 0.0, 0.0));
    nodetip2->SetForce(ChVector<>(0.0, 0.0, 0.0));
    nodetip3->SetForce(ChVector<>(0.0, 0.0, 0.0));
    nodetip4->SetForce(ChVector<>(0.0, 0.0, 0.0));
    // Create an orthotropic material.
    double rho = 7850.0;
    ChVector<> E(1.0e7, 1.0e7, 1.0e7);
    ChVector<> nu(0.3, 0.3, 0.3);
    ChVector<> G(3.8461538e6, 3.8461538e6, 3.8461538e6);
    auto material = std::make_shared<ChContinuumElastic>();
    material->Set_RayleighDampingK(0.0);
    material->Set_RayleighDampingM(0.0);
    material->Set_density(rho);
    material->Set_E(E.x());
    // material->Set_G(G.x());
    material->Set_v(nu.x());
    ChMatrixNM<double, 9, 8> CCPInitial;
    for (int k = 0; k < 8; k++) {
        CCPInitial(0, k) = 1;
        CCPInitial(4, k) = 1;
        CCPInitial(8, k) = 1;
    }
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
        auto element = std::make_shared<ChElementBrick_9>();
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
        element->SetDimensions(ChVector<>(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.0);     // Structural damping for this element
        element->SetGravityOn(false);   // turn internal gravitational force calculation off
        element->SetDPIterationNo(50);  // Set maximum number of iterations for Drucker-Prager Newton-Raphson
        element->SetDPYieldTol(1e-8);   // Set stop tolerance for Drucker-Prager Newton-Raphson
        element->SetStrainFormulation(ChElementBrick_9::Hencky);
        element->SetPlasticityFormulation(ChElementBrick_9::J2);
        if (element->GetStrainFormulation() == ChElementBrick_9::Hencky) {
            element->SetPlasticity(Plasticity);
            if (Plasticity) {
                element->SetYieldStress(1e5);
                element->SetHardeningSlope(5e5);
                element->SetCCPInitial(CCPInitial);
                if (element->GetPlasticityFormulation() == ChElementBrick_9::DruckerPrager) {
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
    my_system.Add(my_mesh);

    my_system.Set_G_acc(ChVector<>(0.0, 0.0, 0.0));
    my_mesh->SetAutomaticGravity(false);

    // Mark completion of system construction
    my_system.SetupInitial();

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    // auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    // mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    // mvisualizemesh->SetShrinkElements(true, 0.85);
    // mvisualizemesh->SetSmoothFaces(true);
    // my_mesh->AddAsset(mvisualizemesh);

    // auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    // mvisualizemeshref->SetWireframe(true);
    // mvisualizemeshref->SetDrawInUndeformedReference(true);
    // my_mesh->AddAsset(mvisualizemeshref);

    // auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    // mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    // mvisualizemeshC->SetSymbolsThickness(0.004);
    // my_mesh->AddAsset(mvisualizemeshC);

    // auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    //// mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
    // mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
    // mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    // mvisualizemeshD->SetSymbolsScale(1);
    // mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    // mvisualizemeshD->SetZbufferHide(false);
    // my_mesh->AddAsset(mvisualizemeshD);

    // application.AssetBindAll();
    // application.AssetUpdateAll();

    // Use the MKL Solver
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    my_system.SetSolver(mkl_solver);
    mkl_solver->SetSparsityPatternLock(true);
    my_system.Update();

    // Set the time integrator parameters
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxiters(20);
    mystepper->SetAbsTolerances(1e-8, 1e-2);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetVerbose(false);
    mystepper->SetScaling(true);
    // application.SetTimestep(timestep);

    my_system.Setup();
    my_system.Update();

    outputfile = fopen("SolidBenchmark.txt", "w");
    fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
    fprintf(outputfile, "\n  ");

    double ChTime = 0.0;
    double start = std::clock();
    int Iter = 0;
    while (/*application.GetDevice()->run() && */ (my_system.GetChTime() <= 1.0)) {
        // application.BeginScene();
        // application.DrawAll();
        // application.DoStep();

        force = 300 * std::sin(my_system.GetChTime() * CH_C_PI) / 4;
        nodetip1->SetForce(ChVector<>(force, 0.0, 0.0));
        nodetip2->SetForce(ChVector<>(force, 0.0, 0.0));
        nodetip3->SetForce(ChVector<>(force, 0.0, 0.0));
        nodetip4->SetForce(ChVector<>(force, 0.0, 0.0));
        my_system.DoStepDynamics(timestep);
        // application.EndScene();
        Iter += mystepper->GetNumIterations();
        // GetLog() << "t = " << my_system.GetChTime() << "\n";
        // GetLog() << "Last it: " << mystepper->GetNumIterations() << "\n\n";
        // if (!application.GetPaused()) {
        fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z());
        fprintf(outputfile, "\n  ");
        //}
    }
    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    GetLog() << "Simulation Time: " << duration << "\n";
    GetLog() << "Force Time: " << my_mesh->GetTimeInternalForces() << "\n";
    GetLog() << "Jacobian Time: " << my_mesh->GetTimeJacobianLoad() << "\n";
    GetLog() << "Solver Time: " << my_system.GetTimerSolver() << "\n";
    GetLog() << Iter << "\n";
}

// QuasiStatic
void BendingQuasiStatic() {
    FILE* outputfile;
    ChSystem my_system;
    my_system.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
    ChIrrApp application(&my_system, L"9-Node, Large Deformation Brick Element: Bending Problem",
                         core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
                                 core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "  9-Node, Large Deformation Brick Element: Bending Problem \n";
    GetLog() << "-----------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();
    int numFlexBody = 1;
    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;
    // Specification of the mesh
    int numDiv_x = 8;
    int numDiv_y = 8;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    int TotalNumNodes = 2 * XYNumNodes + TotalNumElements;
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
            auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (i % (numDiv_x + 1) == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
                                                    ChVector<>(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));

    // All layers for all elements share the same material.
    double rho = 500;
    ChVector<> E(2.1e8, 2.1e8, 2.1e8);
    ChVector<> nu(0.3, 0.3, 0.3);
    ChVector<> G(8.0769231e7, 8.0769231e7, 8.0769231e7);
    auto material = std::make_shared<ChContinuumElastic>();
    material->Set_RayleighDampingK(0.0);
    material->Set_RayleighDampingM(0.0);
    material->Set_density(rho);
    material->Set_E(E.x());
    material->Set_G(G.x());
    material->Set_v(nu.x());

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
        auto element = std::make_shared<ChElementBrick_9>();
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
        element->SetDimensions(ChVector<>(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.25);   // Structural damping for this element
        element->SetGravityOn(false);  // Turn internal gravitational force calculation off

        element->SetStrainFormulation(ChElementBrick_9::Hencky);
        element->SetPlasticity(false);

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    my_system.Add(my_mesh);

    my_system.Set_G_acc(ChVector<>(0.0, 0.0, 0.0));

    // Mark completion of system construction
    my_system.SetupInitial();

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddAsset(mvisualizemeshC);

    auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizemeshD);

    application.AssetBindAll();
    application.AssetUpdateAll();

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Set up solver
    // my_system.SetSolverType(ChSolver::Type::MINRES);
    // auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    // msolver->SetDiagonalPreconditioning(true);
    // my_system.SetMaxItersSolverSpeed(100);
    // my_system.SetTolForce(1e-10);

    // Use the MKL Solver
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    my_system.SetSolver(mkl_solver);
    mkl_solver->SetSparsityPatternLock(true);
    my_system.Update();

    // Set the time integrator parameters
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(2000);
    mystepper->SetAbsTolerances(5e-5, 1e-1);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetVerbose(true);
    mystepper->SetScaling(true);
    application.SetTimestep(timestep);

    my_system.Setup();
    my_system.Update();

    outputfile = fopen("SolidBenchmark.txt", "w");
    fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
    fprintf(outputfile, "\n  ");

    double force = 0.0;
    application.SetPaused(true);
    while (application.GetDevice()->run() && (my_system.GetChTime() <= 2.0)) {
        application.BeginScene();
        application.DrawAll();
        if (!application.GetPaused()) {
            if (my_system.GetChTime() > 1.0) {
                force = -50;
            } else {
                force = -50 * my_system.GetChTime();
            }

            nodetip->SetForce(ChVector<>(0.0, 0.0, force));

            GetLog() << my_system.GetChTime() << " " << nodetip->GetPos().x() << " " << nodetip->GetPos().y() << " "
                     << nodetip->GetPos().z() << "\n";
        }
        application.DoStep();
        GetLog() << "Force: " << force << "\n";
        fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
        fprintf(outputfile, "\n  ");
        application.EndScene();
    }
}

// Swinging (Bricked) Shell
void SwingingShell() {
    FILE* outputfile;
    ChSystem my_system;
    my_system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
    ChIrrApp application(&my_system, L"9-Node, Large Deformation Brick Element: Swinging (Bricked) Shell",
                         core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
                                 core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

    GetLog() << "--------------------------------------------------------------------\n";
    GetLog() << "--------------------------------------------------------------------\n";
    GetLog() << " 9-Node, Large Deformation Brick Element: Swinging (Bricked) Shell  \n";
    GetLog() << "--------------------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;

    // Specification of the mesh
    int numDiv_x = 16;
    int numDiv_y = 16;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    int TotalNumNodes = 2 * XYNumNodes + TotalNumElements;

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
            auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
            node->SetMass(0);
            // Fix all nodes along the axis X=0
            if (i == 0 && j == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
                                                    ChVector<>(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    double force = 0.0;
    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));
    nodetip->SetForce(ChVector<>(0.0, 0.0, -0.0));

    // All layers for all elements share the same material.
    double rho = 1000;
    ChVector<> E(2.1e7, 2.1e7, 2.1e7);
    ChVector<> nu(0.3, 0.3, 0.3);
    ChVector<> G(8.0769231e6, 8.0769231e6, 8.0769231e6);
    auto material = std::make_shared<ChContinuumElastic>();
    material->Set_RayleighDampingK(0.0);
    material->Set_RayleighDampingM(0.0);
    material->Set_density(rho);
    material->Set_E(E.x());
    material->Set_G(G.x());
    material->Set_v(nu.x());

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
        auto element = std::make_shared<ChElementBrick_9>();
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
        element->SetDimensions(ChVector<>(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.005);  // Structural damping for this element
        element->SetGravityOn(true);   // turn internal gravitational force calculation off
        element->SetStrainFormulation(ChElementBrick_9::Hencky);
        element->SetPlasticity(false);

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    my_system.Add(my_mesh);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    my_system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));
    my_mesh->SetAutomaticGravity(false);

    // Mark completion of system construction
    my_system.SetupInitial();

    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRESS_VONMISES);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddAsset(mvisualizemeshC);

    auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizemeshD);

    application.AssetBindAll();
    application.AssetUpdateAll();

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Set up solver
    // my_system.SetSolverType(ChSolver::Type::MINRES);
    // auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    // msolver->SetDiagonalPreconditioning(true);
    // my_system.SetMaxItersSolverSpeed(100);
    // my_system.SetTolForce(1e-10);

    // Use the MKL Solver
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    my_system.SetSolver(mkl_solver);
    mkl_solver->SetSparsityPatternLock(true);
    my_system.Update();

    // Set the time integrator parameters
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(20);
    mystepper->SetAbsTolerances(1e-6, 1e-1);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetVerbose(true);
    mystepper->SetScaling(true);
    application.SetTimestep(timestep);

    my_system.Setup();
    my_system.Update();

    outputfile = fopen("SolidBenchmark.txt", "w");
    fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
    fprintf(outputfile, "\n  ");

    double ChTime = 0.0;
    while (application.GetDevice()->run() && (my_system.GetChTime() < 2.01)) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z());
        fprintf(outputfile, "\n  ");
        application.EndScene();
        if (!application.GetPaused()) {
            GetLog() << my_system.GetChTime() << " " << nodetip->GetPos().x() << " " << nodetip->GetPos().y() << " "
                     << nodetip->GetPos().z() << "\n";
        }
    }
}
