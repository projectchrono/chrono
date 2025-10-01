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
//
// FEA multiphysics
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChElementTetraCorot_10.h"
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementHexaCorot_20.h"
#include "chrono/fea/ChContinuumThermal.h"
#include "chrono/fea/ChContinuumElectrostatics.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChFeaMaterial.h"

#include "FEAvisualization.h"

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

using namespace chrono;
using namespace chrono::fea;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    if (false) 
    {
        auto mnode1 = chrono_types::make_shared<ChFeaNodeXYZ>();
        auto mnode2 = chrono_types::make_shared<ChFeaNodeXYZ>();
        auto mnode3 = chrono_types::make_shared<ChFeaNodeXYZ>();
        auto mnode4 = chrono_types::make_shared<ChFeaNodeXYZ>();

        mnode1->Set(0, 0, 0);
        mnode2->Set(0, 0, 1);
        mnode3->Set(0, 1, 0);
        mnode4->Set(1, 0, 0);

        auto displacement_field = chrono_types::make_shared <ChFeaFieldDisplacement3D>();
        displacement_field->AddNode(mnode1);
        displacement_field->AddNode(mnode2);
        displacement_field->AddNode(mnode3);
        displacement_field->AddNode(mnode4);

        displacement_field->NodeData(mnode1).SetFixed(true);
        displacement_field->NodeData(mnode1).SetPos(*mnode1);
        displacement_field->NodeData(mnode2).SetPos(*mnode2);
        displacement_field->NodeData(mnode3).SetPos(*mnode3);
        displacement_field->NodeData(mnode4).SetPos(ChVector3d(1, 0, 0));
        displacement_field->NodeData(mnode4).SetLoad(ChVector3d(900, 100, 200));

        auto temperature_field = chrono_types::make_shared <ChFeaFieldTemperature>();
        temperature_field->AddNode(mnode1);
        temperature_field->AddNode(mnode2);
        temperature_field->AddNode(mnode3);
        temperature_field->AddNode(mnode4);

        temperature_field->NodeData(mnode1).SetFixed(true);
        temperature_field->NodeData(mnode4).State()[0] = 100;



        auto tetrahedron1 = chrono_types::make_shared <ChFeaElementTetrahedron_4>();
        tetrahedron1->SetNodes(mnode1, mnode2, mnode3, mnode4);


        //--------------------------------------


        auto thermal_domain = chrono_types::make_shared <ChFeaDomainThermal>(temperature_field);
        thermal_domain->AddElement(tetrahedron1);

        // Needed to setup all data and pointers
        thermal_domain->InitialSetup();

        std::cout << "thermal_domain->GetNumPerNodeCoordsPosLevel() \n" << thermal_domain->GetNumPerNodeCoordsPosLevel() << "\n";
        ChVectorDynamic<> mstate_block;
        thermal_domain->GetStateBlock(tetrahedron1, mstate_block);
        std::cout << "thermal_domain->GetStateBlock(tetrahedron1,mstate_block) \n" << mstate_block << "\n";

        thermal_domain->material = chrono_types::make_shared<ChFea3DMaterialThermal>(); // not needed - already attached by default
        thermal_domain->material->SetDensity(1000);
        thermal_domain->material->SetSpecificHeatCapacity(20);
        thermal_domain->material->SetThermalConductivity(5);

        ChVectorDynamic<> Fi;
        thermal_domain->ElementComputeInternalLoads(tetrahedron1, thermal_domain->ElementData(tetrahedron1), Fi);
        std::cout << "thermal_domain->ElementComputeInternalLoads()  Fi = \n" << Fi << "\n";

        ChMatrixDynamic<> Ki(4, 4);
        thermal_domain->ElementComputeKRMmatrices(tetrahedron1, thermal_domain->ElementData(tetrahedron1), Ki, 1, 0, 0);
        std::cout << "thermal_domain->ElementComputeKRMmatrices()  Ki = \n" << Ki << "\n";

        //--------------------------------------


        auto elastic_domain = chrono_types::make_shared <ChFeaDomainElastic>(displacement_field);
        elastic_domain->AddElement(tetrahedron1);

        // Needed to setup all data and pointers
        elastic_domain->InitialSetup();

        std::cout << "elastic_domain->GetNumPerNodeCoordsPosLevel() \n" << elastic_domain->GetNumPerNodeCoordsPosLevel() << "\n";
        elastic_domain->GetStateBlock(tetrahedron1, mstate_block);
        std::cout << "elastic_domain->GetStateBlock(tetrahedron1,mstate_block) \n" << mstate_block << "\n";

        auto elastic_material = chrono_types::make_shared<ChFea3DMaterialStressStVenant>();
        elastic_domain->material = elastic_material;
        elastic_material->SetDensity(1000);
        elastic_material->SetYoungModulus(2e9);
        elastic_material->SetPoissonRatio(0.3);

        elastic_domain->ElementComputeInternalLoads(tetrahedron1, elastic_domain->ElementData(tetrahedron1), Fi);
        std::cout << "elastic_domain->ElementComputeInternalLoads()  Fi = \n" << Fi << "\n";

        //ChMatrixDynamic<> Ki;
        Ki.resize(tetrahedron1->GetNumNodes() * 3, tetrahedron1->GetNumNodes() * 3);
        elastic_domain->ElementComputeKRMmatrices(tetrahedron1, elastic_domain->ElementData(tetrahedron1), Ki, 1, 0, 0);
        std::cout << "elastic_domain->ElementComputeKRMmatrices()  Ki = \n" << Ki << "\n";

        // ---------------

        auto thermoelastic_domain = chrono_types::make_shared <ChFeaDomainThermalElastic>(temperature_field, displacement_field);
        thermoelastic_domain->AddElement(tetrahedron1);
        thermoelastic_domain->InitialSetup();

        std::cout << "thermoelastic_domain->GetNumPerNodeCoordsPosLevel() " << thermoelastic_domain->GetNumPerNodeCoordsPosLevel() << "\n";
        thermoelastic_domain->GetStateBlock(tetrahedron1, mstate_block);
        std::cout << "thermoelastic_domain->GetStateBlock(tetrahedron1,mstate_block) " << mstate_block << "\n";

    }

    //-----------------

    if (true)
    {
        class Ch3DArrayOfNodes {
            size_t n, m, k;
        public:
            std::vector<std::shared_ptr<ChFeaNodeXYZ>> data;
            Ch3DArrayOfNodes(size_t n_, size_t m_, size_t k_)
                : n(n_), m(m_), k(k_), data(n_* m_* k_) {}
            std::shared_ptr<ChFeaNodeXYZ>& at(size_t i, size_t j, size_t l) {
                return data[i * m * k + j * k + l];
            }
        };
        class Ch3DArrayOfHexa {
            size_t n, m, k;
        public:
            std::vector<std::shared_ptr<ChFeaElementHexahedron_8>> data;
            Ch3DArrayOfHexa(size_t n_, size_t m_, size_t k_)
                : n(n_), m(m_), k(k_), data(n_* m_* k_) {}
            std::shared_ptr<ChFeaElementHexahedron_8>& at(size_t i, size_t j, size_t l) {
                return data[i * m * k + j * k + l];
            }
        };

        int nlayers_x = 6; 
        int nlayers_y = 1; 
        int nlayers_z = 1;
        double W_x = 0.6; 
        double W_y = 0.1; 
        double W_z = 0.1;

        Ch3DArrayOfNodes mnodes(nlayers_x + 1, nlayers_y + 1, nlayers_z + 1);
        Ch3DArrayOfHexa  melements(nlayers_x, nlayers_y, nlayers_z);

        auto displacement_field = chrono_types::make_shared <ChFeaFieldDisplacement3D>();
        
        for (int i_z = 0; i_z <= nlayers_z; ++i_z) {
            for (int i_y = 0; i_y <= nlayers_y; ++i_y) {
                for (int i_x = 0; i_x <= nlayers_x; ++i_x) {
                    ChVector3d mypos((W_x / nlayers_x)* i_x, (W_y / nlayers_y)* i_y, (W_z / nlayers_z)* i_z);
                    auto mnode = chrono_types::make_shared<ChFeaNodeXYZ>();
                    mnode->Set(mypos);
                    displacement_field->AddNode(mnode);
                    displacement_field->NodeData(mnode).SetPos(mypos); // initial position = ref position
                    mnodes.at(i_x, i_y, i_z) = mnode;
                    if (i_x > 0 && i_y > 0 && i_z > 0) {
                        auto hexa = chrono_types::make_shared <ChFeaElementHexahedron_8>();
                        hexa->SetNodes({ mnodes.at(i_x-1, i_y-1, i_z-1), 
                                         mnodes.at(i_x  , i_y-1, i_z-1), 
                                         mnodes.at(i_x  , i_y  , i_z-1), 
                                         mnodes.at(i_x-1, i_y  , i_z-1),
                                         mnodes.at(i_x-1, i_y-1, i_z),
                                         mnodes.at(i_x  , i_y-1, i_z),
                                         mnodes.at(i_x  , i_y  , i_z),
                                         mnodes.at(i_x-1, i_y  , i_z) 
                            });
                        melements.at(i_x-1,i_y-1,i_z-1) = hexa;
                    }
                }
            }
        }
        auto elastic_domain = chrono_types::make_shared <ChFeaDomainElastic>(displacement_field);
        for (auto& created_element : melements.data)
            elastic_domain->AddElement(created_element);

        auto elastic_material = chrono_types::make_shared<ChFea3DMaterialStressStVenant>();
        elastic_domain->material = elastic_material;
        elastic_material->SetDensity(1000);
        elastic_material->SetYoungModulus(2e9);
        elastic_material->SetPoissonRatio(0.3);

        // Needed to setup all data and pointers
        elastic_domain->InitialSetup();

        //--------------------------------------
        auto mtest_element = melements.at(0, 0, 0);
        
        std::cout << "elastic_domain->GetNumPerNodeCoordsPosLevel() \n" << elastic_domain->GetNumPerNodeCoordsPosLevel() << "\n";
        ChVectorDynamic<> mstate_block;
        elastic_domain->GetStateBlock(mtest_element, mstate_block);
        std::cout << "elastic_domain->GetStateBlock(tetrahedron1,mstate_block) \n" << mstate_block << "\n";

        ChVectorDynamic<> Fi;
        elastic_domain->ElementComputeInternalLoads(mtest_element, elastic_domain->ElementData(mtest_element), Fi);
        std::cout << "elastic_domain->ElementComputeInternalLoads()  Fi = \n" << Fi << "\n";

        //ChMatrixDynamic<> Ki;
        ChMatrixDynamic<> Ki;
        Ki.resize(mtest_element->GetNumNodes() * 3, mtest_element->GetNumNodes() * 3);
        elastic_domain->ElementComputeKRMmatrices(mtest_element, elastic_domain->ElementData(mtest_element), Ki, 1, 0, 0);
        std::cout << "elastic_domain->ElementComputeKRMmatrices()  Ki = \n" << Ki << "\n";
        ChMatrixDynamic<> Mi;
        Mi.resize(mtest_element->GetNumNodes() * 3, mtest_element->GetNumNodes() * 3);
        elastic_domain->ElementComputeKRMmatrices(mtest_element, elastic_domain->ElementData(mtest_element), Mi, 0, 0, 1);
        std::cout << "elastic_domain->ElementComputeKRMmatrices()  Mi = \n" << Mi << "\n";

        // ---dynamics 
        
        // Create a Chrono physical system
        ChSystemSMC sys;

        // Create a mesh, that is a container for groups
        // of elements and their referenced nodes.
        auto my_mesh = chrono_types::make_shared<ChMesh>();

        // No gravity effect on FEA elements in this demo
        my_mesh->SetAutomaticGravity(false);

        // Remember to add the mesh to the system!
        sys.Add(my_mesh);

        
        sys.Add(displacement_field); // TODO as below
        sys.Add(elastic_domain); // TODO as below
        //my_mesh->AddField(displacement_field); // TODO
        //my_mesh->AddDomain(elastic_domain);    // TODO


        auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>();
        mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
        mvisualizebeamC->SetSymbolsThickness(0.006);
        mvisualizebeamC->SetSymbolsScale(0.01);
        mvisualizebeamC->SetZbufferHide(false);
        my_mesh->AddVisualShapeFEA(mvisualizebeamC);

        // Create the run-time visualization system
        auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "Multiphysics intro", ChVector3d(-0.1, 0.2, -0.2));

        // THE SIMULATION LOOP

        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        sys.SetSolver(solver);
        solver->SetMaxIterations(500);
        solver->SetTolerance(1e-14);
        solver->EnableDiagonalPreconditioner(true);
        solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED


        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(1e-3);

            // double time = sys.GetChTime();
        }

    }


    // Later one can access instanced per-element data as in these examples:
    //  thermal_domain->ElementData(tetrahedron1).element_data; // ...
    //  thermal_domain->ElementData(tetrahedron1).matpoints_data.size(); // ...
    //  thermal_domain->ElementData(tetrahedron1).nodes_data.size(); // ..  this would fail if no InitialSetup(






    return 0;

}
