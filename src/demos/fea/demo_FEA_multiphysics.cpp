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
#include "chrono/fea/ChDrawer.h"
#include "chrono/fea/ChFieldElementHexahedron8.h"
#include "chrono/fea/ChFieldElementHexahedron8Face.h"
#include "chrono/fea/ChFieldElementTetrahedron4.h"
#include "chrono/fea/ChFieldElementTetrahedron4Face.h"
#include "chrono/fea/ChFieldElementLoadableVolume.h"
#include "chrono/fea/ChFieldElementLoadableSurface.h"
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


    if (true) {
        
        // Introduction to the multiphysics FEA system of Chrono. 
        // Simulate elastic deformation of a cantilever.
        // This case is quite simple because there is only one field: displacement, 
        // later look at more advanced examples on using two fields, ex temperature and
        // displacement, coupled, in order to simulate thermoelastic problems.
        
        // Create a Chrono physical system
        ChSystemNSC sys;

        // FIELD
        // 
        // Create a "field", that is a collection of scalar or vectorial properties at
        // every discretization point. In this case, we need a field of xyz displacement 
        // per each point:

        auto displacement_field = chrono_types::make_shared <ChFieldDisplacement3D>();
        sys.Add(displacement_field);

        // DOMAIN
        // 
        // Create a "domain", which is a collection of finite elements operating over 
        // some field. In this case we create a ChDomainDeformation, referencing the displacement field: 

        auto elastic_domain = chrono_types::make_shared <ChDomainDeformation>(displacement_field);
        sys.Add(elastic_domain);

        // This means that all finite elements of the domain, if solid, will automatically receive 
        // a gravitational load:

        elastic_domain->SetAutomaticGravity(true);

        // MATERIAL
        // 
        // Depending on the type of domain, you can set some properties for the material 
        // to be used for the FEA. In this case we create an elastic meaterial, which 
        // for small deformations acts like linear elasticity.

        auto elastic_material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
        elastic_domain->material = elastic_material; // set the material in domain
        elastic_material->SetDensity(1000);
        elastic_material->SetYoungModulus(3e6);
        elastic_material->SetPoissonRatio(0.3);


        // CREATE FINITE ELEMENTS AND NODES
        //
        // You could create your finite elements one by one by yourself, however here for 
        // conciseness we use a ChBuilderVolumeBox helper:

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

        
        // Set atomic force on some node.
        // Note that the ChNodeFEAfieldXYZ created in the volume builder are generic nodes that
        // do not contain the xyz position, or speed, or applied atomic forces, because those properties
        // are attached by means of the displacement_field. 
        // So the way to access the position, force etc is:
        // 
        // displacement_field->NodeData(builder.nodes.at(4, 0, 0)).SetLoad(ChVector3d(0,1000,0));
        
        // Set initial position if different from default:
        //displacement_field->NodeData(builder.nodes.at(1, 1, 0)).SetPos(displacement_field->NodeData(builder.nodes.at(1, 1, 0)).GetPos() + ChVector3d(0, 0.1, 0));
        //displacement_field->NodeData(builder.nodes.at(1, 1, 1)).SetPos(displacement_field->NodeData(builder.nodes.at(1, 1, 1)).GetPos() + ChVector3d(0, 0, 0));
        
        // Set some node to fixed:
        for (auto mnode : builder.nodes.list()) {
            if (mnode->x() <= 0)
                displacement_field->NodeData(mnode).SetFixed(true);
        }

  
        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(elastic_domain);
        visual_nodes->SetGlyphsSize(0.1);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractPosDt(), 0.0, 2.0, "Vel");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        elastic_domain->AddVisualShape(visual_nodes);

        auto visual_stress = chrono_types::make_shared<ChVisualDomainGlyphs>(elastic_domain);
        visual_stress->SetGlyphsSize(2.2);
        visual_stress->AddPositionExtractor(ExtractPos());
        visual_stress->AddPropertyExtractor(ChDomainDeformation::ExtractEulerAlmansiStrain(), 0.0, 2.0, "e");
        visual_stress->SetColormap(ChColormap(ChColormap::Type::JET));
        elastic_domain->AddVisualShape(visual_stress);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(elastic_domain);
        visual_mesh->AddPositionExtractor(ExtractPos());
        visual_mesh->AddPropertyExtractor(ExtractPosDt(), 0.0, 2.0, "Vel");
        visual_mesh->SetColormap(ChColor(0, 1, 0));
        visual_mesh->SetWireframe(true);
        visual_mesh->SetShrinkElements(true, 0.9);
        //elastic_domain->AddVisualShape(visual_mesh);

        auto visual_mesh2 = chrono_types::make_shared<ChVisualDomainMesh>(elastic_domain);
        visual_mesh2->AddPositionExtractor(ExtractPos());
        visual_mesh2->AddPropertyExtractor(ChDomainDeformation::ExtractEulerAlmansiStrain().VonMises(), -0.1, 0.1, "Stretch");
        visual_mesh2->SetColormap(ChColormap(ChColormap::Type::JET));
       // visual_mesh2->SetWireframe(true);
        visual_mesh2->SetShrinkElements(true, 0.9);
        elastic_domain->AddVisualShape(visual_mesh2);

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


    if (true) {

        // Introduction to the multiphysics FEA system of Chrono. 
        // A transient heat problem.
        // This example shows how to setup a thermal problem including
        // some boundary conditions, like radiative heat loss, convection,
        // enforced heat flux on a surface, volumetric heat flux, etc.
        
        // Create a Chrono physical system
        ChSystemNSC sys;

        // FIELD
        // 
        // Create a "field", that is a collection of scalar or vectorial properties at
        // every discretization point. In this case, we need a scalar field, with T temperature 
        // per each point:

        auto temperature_field = chrono_types::make_shared <ChFieldTemperature>();
        sys.Add(temperature_field);

        // DOMAIN
        // 
        // Create a "domain", which is a collection of finite elements operating over 
        // some field. In this case we create a ChDomainThermal, referencing the temperature field: 

        auto thermal_domain = chrono_types::make_shared <ChDomainThermal>(temperature_field);
        sys.Add(thermal_domain);

        // MATERIAL
        // 
        // For the transient heat problem we must provide settings of: specific heat capacity, 
        // density, thermal conductivity. If we use ChMaterial3DThermalLinear, those properties
        // are constant, but here we use ChMaterial3DThermalNonlinear instead, so we can showcase
        // an example where the thermal conductivity is a ChFunction of temperature c=c(T), here defined
        // using a piecewise interpolation of three T-c pairs.

        auto thermal_material = chrono_types::make_shared<ChMaterial3DThermalNonlinear>();
        thermal_material->SetDensity(1000);
        thermal_material->SetSpecificHeatCapacity(1.11);
        auto c_T = chrono_types::make_shared<ChFunctionInterp>();
        c_T->AddPoint(0, 0.16);
        c_T->AddPoint(250, 0.17);
        c_T->AddPoint(300, 0.11);
        thermal_material->SetThermalConductivity(c_T);
        thermal_domain->material = thermal_material;

        // CREATE FINITE ELEMENTS AND NODES
        // 
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
        auto exa_face = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(0,0,4), 3); // 3rd face of hexa is y up
        auto exa_face_loadable = chrono_types::make_shared <ChFieldElementLoadableSurface>(exa_face, temperature_field);

        auto heat_flux = chrono_types::make_shared<ChLoaderHeatFlux>(exa_face_loadable);
        heat_flux->SetSurfaceHeatFlux(100); // the surface flux: heat in W/m^2
        load_container->Add(heat_flux);

        // - IMPOSED HEAT SOURCE ON VOLUME
        // Create a volume wrapper, an auxiliary object that references a volume of an
        // element as a ChLoadableUVW so that it can receive a volume load affecting a field 
        auto exa_volume_loadable = chrono_types::make_shared<ChFieldElementLoadableVolume>(builder.elements.at(4, 0, 4), temperature_field);

        auto heat_source = chrono_types::make_shared<ChLoaderHeatVolumetricSource>(exa_volume_loadable);
        heat_source->SetVolumeHeatFlux(300); // the volumetric source flux: heat in W/m^3
        load_container->Add(heat_source);

        // - THERMAL RADIATION (Stefan-Boltzmann boundary condition)
        // Create a volume wrapper, an auxiliary object that references a volume of an
        // element as a ChLoadableUVW so that it can receive a volume load affecting a field 
        auto exa_face2 = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(4, 0, 0), 3); // 3rd face of hexa is y up
        auto exa_face2_loadable = chrono_types::make_shared <ChFieldElementLoadableSurface>(exa_face2, temperature_field);

        auto heat_radiate = chrono_types::make_shared<ChLoaderHeatRadiation>(exa_face2_loadable, temperature_field);
        heat_radiate->SetEnvironmentTemperature(0); // the surface flux: heat in W/m^2
        load_container->Add(heat_radiate);

        // - THERMAL CONVECTION 
        auto exa_face3 = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(4, 0, 0), 3); // 3rd face of hexa is y down
        auto exa_face3_loadable = chrono_types::make_shared <ChFieldElementLoadableSurface>(exa_face3, temperature_field);

        auto heat_convection = chrono_types::make_shared<ChLoaderHeatConvection>(exa_face3_loadable, temperature_field);
        heat_convection->SetSurfaceConvectionCoeff(10000);
        heat_convection->SetFluidTemperature(400);
        load_container->Add(heat_convection);


        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(thermal_domain);
        visual_nodes->SetGlyphsSize(0.05);
        visual_nodes->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        thermal_domain->AddVisualShape(visual_nodes);

        auto visual_matpoints = chrono_types::make_shared<ChVisualDomainGlyphs>(thermal_domain);
        visual_matpoints->SetGlyphsSize(0.1);
        visual_matpoints->glyph_scalelenght = 0.01;
        visual_matpoints->AddPropertyExtractor(ChDomainThermal::ExtractHeatFlux(), 0.0, 50, "q flux");
        visual_matpoints->SetColormap(ChColormap(ChColormap::Type::JET));
        thermal_domain->AddVisualShape(visual_matpoints);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(thermal_domain);
        visual_mesh->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_mesh->SetColormap(ChColormap(ChColormap::Type::JET));
        visual_mesh->SetWireframe(true);
        thermal_domain->AddVisualShape(visual_mesh);

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

        // A coupled thermo-elastic problem, where chrono solves at the same time
        // both broblems: the elastic (deformation) and the thermal (heat conduction).
        // Because of the thermal expansion of the solid, the 3D slab will deform when
        // heat is applied on a surface. 
        
        // Create a Chrono physical system
        ChSystemNSC sys;

        // FIELD
        // 
        // Create two "fields", because here we will solve at the same time the 
        // temperature and the deformation of a solid:

        auto temperature_field = chrono_types::make_shared <ChFieldTemperature>();
        auto displacement_field = chrono_types::make_shared <ChFieldDisplacement3D>();
        sys.Add(temperature_field);
        sys.Add(displacement_field);

        // DOMAIN
        // 
        // Create a "domain", which is a collection of finite elements operating over 
        // some field. In this case we create a ChDomainThermal, that must operate on
        // two fields: the temperature field and the displacement field.
        // Note that you can look at the implememtation of the ChDomainThermoDeformation
        // and you can implement other types of coupled domains according to your needs (ex.
        // thermo-chemical, or thermo-electric, piezoelectric, etc.)

        auto domain = chrono_types::make_shared <ChDomainThermoDeformation>(temperature_field, displacement_field);
        sys.Add(domain);

        // MATERIAL
        // 
        // The ChDomainThermoDeformation  uses a material of ChMaterial3DThermalStress type. 
        // The default ChMaterial3DThermalStress that is already in the domain, is made with two 
        // components: the material for the stress problem (which could be whatever ChMaterial3DStress model 
        // like Ogden or StVenant) and the material for the elastic problem. Then you can also set the 
        // thermal expansion coefficient, that causes the coupling. Example:

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

        // CREATE FINITE ELEMENTS AND NODES
        // 
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


        // EXAMPLE INITIAL CONDITIONS (initial temperature of some nodes)

        temperature_field->NodeData(builder.nodes.at(0, 0, 4)).T() = 400;
        temperature_field->NodeData(builder.nodes.at(0, 1, 4)).T() = 400;

        // EXAMPLE DIRICHLET CONDITIONS (fixed positions of some nodes)
        for (auto mnode : builder.nodes.list()) {
            if (mnode->x() <= 0)
                displacement_field->NodeData(mnode).SetFixed(true);
        }


        // EXAMPLE OF SOME LOADS

        // First: loads must be added to "load containers",
        // and load containers must be added to your system
        auto load_container = chrono_types::make_shared<ChLoadContainer>();
        sys.Add(load_container);

        // - IMPOSED HEAT FLUX ON SURFACE
        // Create a face wrapper, an auxiliary object that references a face of an
        // element as a ChLoadableUV so that can receive a surface load affecting a field 
        auto exa_face = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(2, 0, 3), 3); // 3rd face of hexa is y up
        auto exa_face_loadable = chrono_types::make_shared <ChFieldElementLoadableSurface>(exa_face, temperature_field);

        auto heat_flux = chrono_types::make_shared<ChLoaderHeatFlux>(exa_face_loadable);
        heat_flux->SetSurfaceHeatFlux(100); // the surface flux: heat in W/m^2
        load_container->Add(heat_flux);


        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(domain);
        visual_nodes->SetGlyphsSize(0.05);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        domain->AddVisualShape(visual_nodes);

        auto visual_matpoints = chrono_types::make_shared<ChVisualDomainGlyphs>(domain);
        visual_matpoints->SetGlyphsSize(0.1);
        visual_matpoints->glyph_scalelenght = 0.01;
        visual_matpoints->AddPositionExtractor(::ExtractPos());
        visual_matpoints->AddPropertyExtractor(ChDomainThermoDeformation::ExtractHeatFlux(), 0.0, 50, "q flux");
        visual_matpoints->SetColormap(ChColormap(ChColormap::Type::JET));
        domain->AddVisualShape(visual_matpoints);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(domain);
        visual_mesh->AddPositionExtractor(ExtractPos());
        visual_mesh->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_mesh->SetColormap(ChColormap(ChColormap::Type::JET));
        visual_mesh->SetWireframe(true);
        domain->AddVisualShape(visual_mesh);

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
