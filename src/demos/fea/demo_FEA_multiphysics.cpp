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
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/physics/ChLoadContainer.h"
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
#include "chrono/fea/ChNodeFEAfieldXYZ.h"
#include "chrono/fea/ChDomainDeformation.h"
#include "chrono/fea/ChDomainThermal.h"
#include "chrono/fea/ChDomainThermoDeformation.h"
#include "chrono/fea/ChMaterial3DThermalNonlinear.h"
#include "chrono/fea/ChVisualDataExtractor.h"
#include "chrono/fea/ChFieldElementHexahedron8.h"
#include "chrono/fea/ChFieldElementHexahedron8Face.h"
#include "chrono/fea/ChFieldElementTetrahedron4.h"
#include "chrono/fea/ChFieldElementTetrahedron4Face.h"
#include "chrono/fea/ChFieldElementLoadableVolume.h"
#include "chrono/fea/ChLoaderHeatFlux.h"
#include "chrono/fea/ChLoaderHeatRadiation.h"
#include "chrono/fea/ChLoaderHeatVolumetricSource.h"
#include "chrono/fea/ChLoaderHeatConvection.h"
#include "chrono/fea/ChLoaderPressure.h"
#include "chrono/fea/ChBuilderVolume.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkLockTrajectory.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "FEAvisualization.h"

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

using namespace chrono;
using namespace chrono::fea;





int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    if (false) 
    {
        auto mnode1 = chrono_types::make_shared<ChNodeFEAfieldXYZ>();
        auto mnode2 = chrono_types::make_shared<ChNodeFEAfieldXYZ>();
        auto mnode3 = chrono_types::make_shared<ChNodeFEAfieldXYZ>();
        auto mnode4 = chrono_types::make_shared<ChNodeFEAfieldXYZ>();

        mnode1->Set(0, 0, 0);
        mnode2->Set(0, 0, 1);
        mnode3->Set(0, 1, 0);
        mnode4->Set(1, 0, 0);

        auto displacement_field = chrono_types::make_shared <ChFieldDisplacement3D>();
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

        auto temperature_field = chrono_types::make_shared <ChFieldTemperature>();
        temperature_field->AddNode(mnode1);
        temperature_field->AddNode(mnode2);
        temperature_field->AddNode(mnode3);
        temperature_field->AddNode(mnode4);

        temperature_field->NodeData(mnode1).SetFixed(true);
        temperature_field->NodeData(mnode4).State()[0] = 100;



        auto tetrahedron1 = chrono_types::make_shared <ChFieldElementTetrahedron4>();
        tetrahedron1->SetNodes(mnode1, mnode2, mnode3, mnode4);


        //--------------------------------------


        auto thermal_domain = chrono_types::make_shared <ChDomainThermal>(temperature_field);
        thermal_domain->AddElement(tetrahedron1);

        // Needed to setup all data and pointers
        thermal_domain->InitialSetup();

        std::cout << "thermal_domain->GetNumPerNodeCoordsPosLevel() \n" << thermal_domain->GetNumPerNodeCoordsPosLevel() << "\n";
        ChVectorDynamic<> mstate_block;
        thermal_domain->GetFieldStateBlock(tetrahedron1, mstate_block, 0);
        std::cout << "thermal_domain->GetFieldStateBlock(tetrahedron1,mstate_block,0) \n" << mstate_block << "\n";

        auto thermal_material = chrono_types::make_shared<ChMaterial3DThermalLinear>();
        thermal_material->SetDensity(1000);
        thermal_material->SetSpecificHeatCapacity(20);
        thermal_material->SetThermalConductivity(5);
        thermal_domain->material = thermal_material;

        ChVectorDynamic<> Fi;
        thermal_domain->ElementComputeInternalLoads(tetrahedron1, thermal_domain->ElementData(tetrahedron1), Fi);
        std::cout << "thermal_domain->ElementComputeInternalLoads()  Fi = \n" << Fi << "\n";

        ChMatrixDynamic<> Ki(4, 4);
        thermal_domain->ElementComputeKRMmatrices(tetrahedron1, thermal_domain->ElementData(tetrahedron1), Ki, 1, 0, 0);
        std::cout << "thermal_domain->ElementComputeKRMmatrices()  Ki = \n" << Ki << "\n";

        //--------------------------------------


        auto elastic_domain = chrono_types::make_shared <ChDomainDeformation>(displacement_field);
        elastic_domain->AddElement(tetrahedron1);

        // Needed to setup all data and pointers
        elastic_domain->InitialSetup();

        std::cout << "elastic_domain->GetNumPerNodeCoordsPosLevel() \n" << elastic_domain->GetNumPerNodeCoordsPosLevel() << "\n";
        elastic_domain->GetFieldStateBlock(tetrahedron1, mstate_block,0);
        std::cout << "elastic_domain->GetFieldStateBlock(tetrahedron1,mstate_block,0) \n" << mstate_block << "\n";

        auto elastic_material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
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

        auto domain = chrono_types::make_shared <ChDomainThermoDeformation>(temperature_field, displacement_field);
        domain->AddElement(tetrahedron1);
        domain->InitialSetup();

        std::cout << "domain->GetNumPerNodeCoordsPosLevel() " << domain->GetNumPerNodeCoordsPosLevel() << "\n";
        domain->GetFieldStateBlock(tetrahedron1, mstate_block, 0);
        std::cout << "domain->GetFieldStateBlock(tetrahedron1,mstate_block,0) " << mstate_block << "\n";
        domain->GetFieldStateBlock(tetrahedron1, mstate_block, 1);
        std::cout << "domain->GetFieldStateBlock(tetrahedron1,mstate_block,1) " << mstate_block << "\n";

        // Later one can access instanced per-element data as in these examples:
        //  thermal_domain->ElementData(tetrahedron1).element_data; // ...
        //  thermal_domain->ElementData(tetrahedron1).matpoints_data.size(); // ...
        //  thermal_domain->ElementData(tetrahedron1).nodes_data.size(); // ..  this would fail if no InitialSetup(
    }


    if (true) {

        // Create a Chrono physical system
        ChSystemNSC sys;

        auto floor = chrono_types::make_shared<ChBody>();
        floor->SetFixed(true);
        sys.Add(floor);

        auto displacement_field = chrono_types::make_shared <ChFieldDisplacement3D>();

        auto elastic_domain = chrono_types::make_shared <ChDomainDeformation>(displacement_field);

        // Build a test volume discretized with a regular grid of finite elements.
        ChBuilderVolumeBoxTetra  builder;
        builder.BuildVolume( ChFrame<>(),
            8, 3, 3,            // N of elements in x,y,z direction
            3, 0.5, 0.5);       // width in x,y,z direction

        // After Build(), the elements and the nodes must be added to domains and fields:
        for (auto& created_elementc : builder.elements.list())
            for (auto& created_element : created_elementc)
                elastic_domain->AddElement(created_element);
        for (auto& created_node : builder.nodes.list())
            displacement_field->AddNode(created_node);

        auto elastic_material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
        elastic_domain->material = elastic_material;
        elastic_material->SetDensity(1000);
        elastic_material->SetYoungModulus(3e6);
        elastic_material->SetPoissonRatio(0.3);

        //displacement_field->NodeData(builder.nodes.at(4, 0, 0)).SetLoad(ChVector3d(0,1000,0));
        //displacement_field->NodeData(builder.nodes.at(1, 1, 0)).SetPos(displacement_field->NodeData(builder.nodes.at(1, 1, 0)).GetPos() + ChVector3d(0, 0.1, 0));
        //displacement_field->NodeData(builder.nodes.at(1, 1, 1)).SetPos(displacement_field->NodeData(builder.nodes.at(1, 1, 1)).GetPos() + ChVector3d(0, 0, 0));
        
        for (auto mnode : builder.nodes.list()) {
            if (mnode->x() <= 0)
                displacement_field->NodeData(mnode).SetFixed(true);
        }
        //displacement_field->NodeData(builder.nodes.at(0, 0, 0)).SetFixed(true);
        //displacement_field->NodeData(builder.nodes.at(0, 0, 3)).SetFixed(true);

        sys.Add(elastic_domain);
        sys.Add(displacement_field);

        // Needed to setup all data and pointers
        elastic_domain->InitialSetup();
        elastic_domain->SetAutomaticGravity(true);



        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(elastic_domain);
        visual_nodes->SetGlyphsSize(0.1);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractPosDt(), 0.0, 2.0, "Vel");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        floor->AddVisualShape(visual_nodes);

        auto visual_stress = chrono_types::make_shared<ChVisualDomainGlyphs>(elastic_domain);
        visual_stress->SetGlyphsSize(2.2);
        visual_stress->AddPositionExtractor(ExtractPos());
        visual_stress->AddPropertyExtractor(ChDomainDeformation::ExtractEulerAlmansiStrain(), 0.0, 2.0, "e");
        visual_stress->SetColormap(ChColormap(ChColormap::Type::JET));
        floor->AddVisualShape(visual_stress);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(elastic_domain);
        visual_mesh->AddPositionExtractor(ExtractPos());
        visual_mesh->AddPropertyExtractor(ExtractPosDt(), 0.0, 2.0, "Vel");
        visual_mesh->SetColormap(ChColor(0, 1, 0));
        visual_mesh->SetWireframe(true);
        visual_mesh->SetShrinkElements(true, 0.9);
        //floor->AddVisualShape(visual_mesh);

        auto visual_mesh2 = chrono_types::make_shared<ChVisualDomainMesh>(elastic_domain);
        visual_mesh2->AddPositionExtractor(ExtractPos());
        visual_mesh2->AddPropertyExtractor(ChDomainDeformation::ExtractEulerAlmansiStrain().VonMises(), -0.1, 0.1, "Stretch");
        visual_mesh2->SetColormap(ChColormap(ChColormap::Type::JET));
       // visual_mesh2->SetWireframe(true);
        visual_mesh2->SetShrinkElements(true, 0.9);
        floor->AddVisualShape(visual_mesh2);

        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Test FEA");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 4, -6));
        vis->AddTypicalLights();
        
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);

        // Simulation loop
        double timestep = 0.01;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }

    }


    if (false) {

        // Create a Chrono physical system
        ChSystemNSC sys;

        auto floor = chrono_types::make_shared<ChBody>();
        floor->SetFixed(true);
        sys.Add(floor);

        auto temperature_field = chrono_types::make_shared <ChFieldTemperature>();

        auto thermal_domain = chrono_types::make_shared <ChDomainThermal>(temperature_field);

        // Build a test volume discretized with a regular grid of finite elements.
        ChBuilderVolumeBox builder;
        builder.BuildVolume( ChFrame<>(),
            5, 1, 5,        // N of elements in x,y,z direction
            3, 0.5, 3);     // width in x,y,z direction

        // After Build(), the elements and the nodes must be added to domains and fields:
        for (auto& created_element : builder.elements.list())
            thermal_domain->AddElement(created_element);
        for (auto& created_node : builder.nodes.list())
            temperature_field->AddNode(created_node);

        auto thermal_material = chrono_types::make_shared<ChMaterial3DThermalNonlinear>();
        thermal_material->SetDensity(1000);
        thermal_material->SetSpecificHeatCapacity(1.11);
        auto c_T = chrono_types::make_shared<ChFunctionInterp>();
        c_T->AddPoint(0,   0.16);
        c_T->AddPoint(250, 0.17);
        c_T->AddPoint(300, 0.11);
        thermal_material->SetThermalConductivity(c_T);
        thermal_domain->material = thermal_material;
 

        // EXAMPLE INITIAL CONDITIONS (initial temperature of some nodes)

        temperature_field->NodeData(builder.nodes.at(0, 0, 4)).T() = 400;
        temperature_field->NodeData(builder.nodes.at(0, 1, 4)).T() = 400;

        // EXAMPLE DIRICHLET CONDITIONS (fixed temperature of some nodes)

        temperature_field->NodeData(builder.nodes.at(0, 0, 0)).SetFixed(true);
        temperature_field->NodeData(builder.nodes.at(0, 0, 0)).T() = 100;
        temperature_field->NodeData(builder.nodes.at(0, 1, 0)).SetFixed(true);
        temperature_field->NodeData(builder.nodes.at(0, 1, 0)).T() = 100;
        temperature_field->NodeData(builder.nodes.at(0, 0, 1)).SetFixed(true);
        temperature_field->NodeData(builder.nodes.at(0, 0, 1)).T() = 100;
        temperature_field->NodeData(builder.nodes.at(0, 1, 1)).SetFixed(true);
        temperature_field->NodeData(builder.nodes.at(0, 1, 1)).T() = 100;
        
        // APPLY SOME LOADS

        // First: loads must be added to "load containers",
        // and load containers must be added to your system
        auto load_container = chrono_types::make_shared<ChLoadContainer>();
        sys.Add(load_container);

        // - IMPOSED HEAT FLUX ON SURFACE
        // Create a face wrapper, an auxiliary object that references a face of an
        // element as a ChLoadableUV so that can receive a surface load affecting a field 
        auto exa_face = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(2,0,3), temperature_field, 3); // 3rd face of hexa is y up

        auto heat_flux = chrono_types::make_shared<ChLoaderHeatFlux>(exa_face);
        heat_flux->SetSurfaceHeatFlux(10); // the surface flux: heat in W/m^2
        load_container->Add(heat_flux);

        // - IMPOSED HEAT SOURCE ON VOLUME
        // Create a volume wrapper, an auxiliary object that references a volume of an
        // element as a ChLoadableUVW so that it can receive a volume load affecting a field 
        auto exa_volume = chrono_types::make_shared<ChFieldElementLoadableVolume>(builder.elements.at(4, 0, 4), temperature_field);

        auto heat_source = chrono_types::make_shared<ChLoaderHeatVolumetricSource>(exa_volume);
        heat_source->SetVolumeHeatFlux(300); // the volumetric source flux: heat in W/m^3
        load_container->Add(heat_source);

        // - THERMAL RADIATION (Stefan-Boltzmann boundary condition)
        // Create a volume wrapper, an auxiliary object that references a volume of an
        // element as a ChLoadableUVW so that it can receive a volume load affecting a field 
        auto exa_face2 = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(4, 0, 0), temperature_field, 3); // 3rd face of hexa is y up

        auto heat_radiate = chrono_types::make_shared<ChLoaderHeatRadiation>(exa_face2, temperature_field);
        heat_radiate->SetEnvironmentTemperature(0); // the surface flux: heat in W/m^2
        load_container->Add(heat_radiate);

        // - THERMAL CONVECTION 
        auto exa_face3 = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(4, 0, 0), temperature_field, 3); // 3rd face of hexa is y down

        auto heat_convection = chrono_types::make_shared<ChLoaderHeatConvection>(exa_face3, temperature_field);
        heat_convection->SetSurfaceConvectionCoeff(10000);
        heat_convection->SetFluidTemperature(400);
        load_container->Add(heat_convection);

        // Needed to setup all data and pointers
        thermal_domain->InitialSetup();


        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(thermal_domain);
        visual_nodes->SetGlyphsSize(0.05);
        visual_nodes->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        floor->AddVisualShape(visual_nodes);

        auto visual_matpoints = chrono_types::make_shared<ChVisualDomainGlyphs>(thermal_domain);
        visual_matpoints->SetGlyphsSize(0.1);
        visual_matpoints->glyph_scalelenght = 0.01;
        visual_matpoints->AddPropertyExtractor(ChDomainThermal::ExtractHeatFlux(), 0.0, 50, "q flux");
        visual_matpoints->SetColormap(ChColormap(ChColormap::Type::JET));
        floor->AddVisualShape(visual_matpoints);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(thermal_domain);
        visual_mesh->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_mesh->SetColormap(ChColormap(ChColormap::Type::JET));
        visual_mesh->SetWireframe(true);
        floor->AddVisualShape(visual_mesh);

        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Test FEA");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 4, -6));
        vis->AddTypicalLights();


        // SOLVER SETTINGS

        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);


        sys.Add(thermal_domain);
        sys.Add(temperature_field);

        // Simulation loop
        double timestep = 50;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }

    }





    if (true) {

        // Create a Chrono physical system
        ChSystemNSC sys;

        auto floor = chrono_types::make_shared<ChBody>();
        floor->SetFixed(true);
        sys.Add(floor);

        auto temperature_field = chrono_types::make_shared <ChFieldTemperature>();
        auto displacement_field = chrono_types::make_shared <ChFieldDisplacement3D>();

        auto domain = chrono_types::make_shared <ChDomainThermoDeformation>(temperature_field, displacement_field);

        // Build a test volume discretized with a regular grid of finite elements.
        ChBuilderVolumeBox builder;
        builder.BuildVolume( ChFrame<>(),
            5, 1, 5,        // N of elements in x,y,z direction
            3, 0.5, 3);     // width in x,y,z direction

        // After Build(), the elements and the nodes must be added to domains and fields:
        for (auto& created_element : builder.elements.list())
            domain->AddElement(created_element);
        for (auto& created_node : builder.nodes.list()) {
            temperature_field->AddNode(created_node);
            displacement_field->AddNode(created_node);
        }


        auto thermal_material = chrono_types::make_shared<ChMaterial3DThermalNonlinear>();
        thermal_material->SetDensity(1000);
        thermal_material->SetSpecificHeatCapacity(1.11);
        auto c_T = chrono_types::make_shared<ChFunctionInterp>();
        c_T->AddPoint(0, 0.16);
        c_T->AddPoint(250, 0.17);
        c_T->AddPoint(300, 0.11);
        thermal_material->SetThermalConductivity(c_T);

        domain->material_thermalstress->material_thermal = thermal_material;

        auto elastic_material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
        elastic_material->SetDensity(1000);
        elastic_material->SetYoungModulus(6e6);
        elastic_material->SetPoissonRatio(0.3);

        domain->material_thermalstress->material_stress = elastic_material;

        domain->material_thermalstress->SetThermalExpansionCoefficient(230 * 10e-6);

        // EXAMPLE INITIAL CONDITIONS (initial temperature of some nodes)

        temperature_field->NodeData(builder.nodes.at(0, 0, 4)).T() = 400;
        temperature_field->NodeData(builder.nodes.at(0, 1, 4)).T() = 400;

        // EXAMPLE DIRICHLET CONDITIONS (fixed positions of some nodes)
        for (auto mnode : builder.nodes.list()) {
            if (mnode->x() <= 0)
                displacement_field->NodeData(mnode).SetFixed(true);
        }


        // APPLY SOME LOADS

        // First: loads must be added to "load containers",
        // and load containers must be added to your system
        auto load_container = chrono_types::make_shared<ChLoadContainer>();
        sys.Add(load_container);

        // - IMPOSED HEAT FLUX ON SURFACE
        // Create a face wrapper, an auxiliary object that references a face of an
        // element as a ChLoadableUV so that can receive a surface load affecting a field 
        auto exa_face = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(2, 0, 3), temperature_field, 3); // 3rd face of hexa is y up

        auto heat_flux = chrono_types::make_shared<ChLoaderHeatFlux>(exa_face);
        heat_flux->SetSurfaceHeatFlux(10); // the surface flux: heat in W/m^2
        load_container->Add(heat_flux);


        // Needed to setup all data and pointers
        domain->InitialSetup();
        //domain->SetAutomaticGravity(true);

        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(domain);
        visual_nodes->SetGlyphsSize(0.05);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        floor->AddVisualShape(visual_nodes);

        auto visual_matpoints = chrono_types::make_shared<ChVisualDomainGlyphs>(domain);
        visual_matpoints->SetGlyphsSize(0.1);
        visual_matpoints->glyph_scalelenght = 0.01;
        visual_matpoints->AddPositionExtractor(::ExtractPos());
        visual_matpoints->AddPropertyExtractor(ChDomainThermoDeformation::ExtractHeatFlux(), 0.0, 50, "q flux");
        visual_matpoints->SetColormap(ChColormap(ChColormap::Type::JET));
        floor->AddVisualShape(visual_matpoints);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(domain);
        visual_mesh->AddPositionExtractor(ExtractPos());
        visual_mesh->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_mesh->SetColormap(ChColormap(ChColormap::Type::JET));
        visual_mesh->SetWireframe(true);
        floor->AddVisualShape(visual_mesh);

        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Test FEA");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 4, -6));
        vis->AddTypicalLights();


        // SOLVER SETTINGS

        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);


        sys.Add(domain);
        sys.Add(temperature_field);
        sys.Add(displacement_field);

        // Simulation loop
        double timestep = 10;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }

    }



    return 0;

}
