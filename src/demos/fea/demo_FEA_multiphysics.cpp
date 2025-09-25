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

    displacement_field->node_data[mnode1].SetFixed(true);
    displacement_field->node_data[mnode1].SetPos(*mnode1);
    displacement_field->node_data[mnode2].SetPos(*mnode2);
    displacement_field->node_data[mnode3].SetPos(*mnode3);
    displacement_field->node_data[mnode4].SetPos(ChVector3d(1, 0, 0));
    displacement_field->node_data[mnode4].SetLoad(ChVector3d(200, 10, 20));

    auto temperature_field = chrono_types::make_shared <ChFeaFieldTemperature>();
    temperature_field->AddNode(mnode1);
    temperature_field->AddNode(mnode2);
    temperature_field->AddNode(mnode3);
    temperature_field->AddNode(mnode4);

    temperature_field->node_data[mnode1].SetFixed(true);
    temperature_field->node_data[mnode4].State()[0] = 100;



    auto tetrahedron1 = chrono_types::make_shared <ChFeaElementTetrahedron_4>();
    tetrahedron1->SetNodes(mnode1, mnode2, mnode3, mnode4);


    //--------------------------------------


    auto thermal_domain = chrono_types::make_shared <ChFeaMaterialDomainThermal>(temperature_field);
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
    thermal_domain->ElementComputeInternalLoads(tetrahedron1, thermal_domain->GetElementData(tetrahedron1), Fi);
    std::cout << "thermal_domain->ElementComputeInternalLoads()  Fi = \n" << Fi << "\n";

    //--------------------------------------


    auto elastic_domain = chrono_types::make_shared <ChFeaMaterialDomainElastic>(displacement_field);
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

    elastic_domain->ElementComputeInternalLoads(tetrahedron1, elastic_domain->GetElementData(tetrahedron1), Fi);
    std::cout << "elastic_domain->ElementComputeInternalLoads()  Fi = \n" << Fi << "\n";



    // ---------------

    auto thermoelastic_domain = chrono_types::make_shared <ChFeaMaterialDomainThermalElastic>(temperature_field, displacement_field);
    thermoelastic_domain->AddElement(tetrahedron1);
    thermoelastic_domain->InitialSetup();

    std::cout << "thermoelastic_domain->GetNumPerNodeCoordsPosLevel() " << thermoelastic_domain->GetNumPerNodeCoordsPosLevel() << "\n";
    thermoelastic_domain->GetStateBlock(tetrahedron1, mstate_block);
    std::cout << "thermoelastic_domain->GetStateBlock(tetrahedron1,mstate_block) " << mstate_block << "\n";



    //-----------------




    // Later one can access instanced per-element data as in these examples:
    //  thermal_domain->GetElementData(tetrahedron1).element_data; // ...
    //  thermal_domain->GetElementData(tetrahedron1).matpoints_data.size(); // ...
    //  thermal_domain->GetElementData(tetrahedron1).nodes_data.size(); // ..  this would fail if no InitialSetup(






    return 0;









    //***TODO****
    // remove the rest 
     
   
    // Create a Chrono physical system
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    

    return 0;
}
