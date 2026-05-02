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
// Unit test for linear elasticity of continua.
//
// Successful execution of this unit test may validate: full lagrangian
// finite elements of ChDomainDeformation.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"
#include "chrono/fea/ChDomainDeformation.h"
#include "chrono/fea/ChDrawer.h"
#include "chrono/fea/ChSurfaceOfDomain.h"
#include "chrono/fea/ChFieldElementHexahedron8.h"
#include "chrono/fea/ChFieldElementHexahedron8Face.h"
#include "chrono/fea/ChFieldElementTetrahedron4.h"
#include "chrono/fea/ChFieldElementTetrahedron4Face.h"
#include "chrono/fea/ChFieldElementLoadableVolume.h"
#include "chrono/fea/ChFieldElementLoadableSurface.h"
#include "chrono/fea/ChLoaderPressure.h"
#include "chrono/fea/ChBuilderVolume.h"
#include "chrono/fea/ChMaterial3DStressOgden.h"
#include "chrono/fea/ChMaterial3DStressNeoHookean.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif


using namespace chrono;
using namespace chrono::fea;



bool test_box_uniaxial_pressure(std::shared_ptr<ChMaterial3DStress> test_material, double test_pressure, double reference_x_displ, double tolerance, double end_time) {
    ChSystemNSC sys;

    auto displacement_field = chrono_types::make_shared<ChFieldDisplacement3D>();
    sys.Add(displacement_field);

    auto elastic_domain = chrono_types::make_shared<ChDomainDeformation>(displacement_field);
    sys.Add(elastic_domain);

    elastic_domain->SetAutomaticGravity(false);

    elastic_domain->material = test_material;  // set the material in domain

    ChBuilderVolumeBox builder;
    builder.BuildVolume(ChFrame<>(), 10, 4, 4,  // N of elements in x,y,z direction
                        1, 0.5, 0.5);           // width in x,y,z direction
    builder.AddToDomain(elastic_domain);

    // Set some node to fixed:
    std::shared_ptr<ChNodeFEAfieldXYZ> probed_node;
    for (auto mnode : builder.nodes.list()) {
        if (mnode->x() <= 0)
            displacement_field->NodeData(mnode).SetFixed(true);
        if (mnode->x() <= 1)
            probed_node = mnode;
    }

    // add loads on faces
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(load_container);

    for (auto mface : builder.faces_x_hi) {
        auto exa_face_loadable = chrono_types::make_shared<ChFieldElementLoadableSurface>(mface, displacement_field);
        auto pressure_load = chrono_types::make_shared<ChLoaderPressure>(exa_face_loadable);
        pressure_load->SetPressure(test_pressure);
        load_container->Add(pressure_load);
    }
    /*
    for (auto mface : builder.faces_x_hi) {
        for (int inode = 0; inode < mface->GetNumNodes(); ++inode) {
            displacement_field->NodeData(mface->GetNode(inode)).SetLoad(ChVector3d(6000, 0, 0));
        }
    }
    */
    bool use_mkl = true;

    #ifndef CHRONO_PARDISO_MKL
    use_mkl = false;
    #endif

    // Setup solver
    if (use_mkl) {
    #ifdef CHRONO_PARDISO_MKL
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        mkl_solver->SetVerbose(false);
        sys.SetSolver(mkl_solver);
    #endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        sys.SetSolver(solver);
        solver->SetMaxIterations(100);
        solver->SetTolerance(1e-10);
        solver->EnableDiagonalPreconditioner(true);
        solver->SetVerbose(false);
    }

    sys.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT);

    // Simulation loop
    double timestep = 0.01;

    
    if (false) {
        /*
        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(elastic_domain);
        visual_nodes->SetGlyphsSize(0.1);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractPosDt(), 0.0, 2.0, "Vel");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        elastic_domain->AddVisualShape(visual_nodes);

        auto visual_stress = chrono_types::make_shared<ChVisualDomainGlyphs>(elastic_domain);
        visual_stress->SetGlyphsSize(0.5);
        visual_stress->AddPositionExtractor(ExtractPos());
        visual_stress->AddPropertyExtractor(ChDomainDeformation::ExtractEulerAlmansiStrain(), 0.0, 2.0, "e");
        visual_stress->SetColormap(ChColormap(ChColormap::Type::JET));
        elastic_domain->AddVisualShape(visual_stress);

        // original undeformed
        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(elastic_domain);
        visual_mesh->SetColormap(ChColor(1, 1, 1));
        visual_mesh->SetWireframe(true);
        elastic_domain->AddVisualShape(visual_mesh);

        auto visual_mesh2 = chrono_types::make_shared<ChVisualDomainMesh>(elastic_domain);
        visual_mesh2->AddPositionExtractor(ExtractPos());
        visual_mesh2->AddPropertyExtractor(ChDomainDeformation::ExtractEulerAlmansiStrain().VonMises(), -0.1, 0.1, "Stretch");
        visual_mesh2->SetColormap(ChColormap(ChColormap::Type::JET));
        // visual_mesh2->SetWireframe(true);
        visual_mesh2->SetShrinkElements(true, 0.99);
        elastic_domain->AddVisualShape(visual_mesh2);

        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Test FEA");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 2, -2));
        vis->AddTypicalLights();

        while (vis->Run() && (sys.GetChTime() < end_time)) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }
        */

    } else {
        while (sys.GetChTime() < end_time) {
            sys.DoStepDynamics(timestep);
        }
    }

    // Fetch displaced position of probed node, from the displacement_field:
    ChVector3d pos_probed = displacement_field->NodeData(probed_node).GetPos();
    ChVector3d pos_reference = probed_node->GetReferencePos();

    ChVector3d result_node_displ = pos_probed - pos_reference;

    std::cout << "  x displacement of probed node = " << result_node_displ.x() << "\n";
    std::cout << "  x displacement reference = " << reference_x_displ << "\n";
    double error_percent = 100.0 * (reference_x_displ - result_node_displ.x()) / reference_x_displ;
    if (std::fabs(error_percent) > tolerance) {
        std::cout << "  TEST FAILED!!! difference % =  " << error_percent << "\n";
        return 1;
    } else {
        std::cout << "  TEST PASSED, difference % =  " << error_percent << "\n";
        return 0;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    /// TEST StVK material

    std::cout << "TEST 1: box compression, material: ChMaterial3DStressStVenant \n";
    auto elastic_material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
    elastic_material->SetDensity(1000);
    elastic_material->SetYoungModulus(3e6);
    elastic_material->SetPoissonRatio(0.39);

    if (test_box_uniaxial_pressure(elastic_material, -20000, 6.550e-3, 3.0, 0.5))
        return 1;

    /// TEST Ogden material

    std::cout << "TEST 2: box compression, material: ChMaterial3DStressOgden \n";
    auto elastic_material2 = chrono_types::make_shared<ChMaterial3DStressOgden>();
    elastic_material2->SetDensity(1000);
    elastic_material2->SetAsEquivalentNeoHookean(3e6, 0.39);

    if (test_box_uniaxial_pressure(elastic_material2, -20000, 6.550e-3, 3.0, 0.5))
        return 1;

    /// TEST Neo-Hookean material

    std::cout << "TEST 3: box compression, material: ChMaterial3DStressNeoHookean \n";
    auto elastic_material3 = chrono_types::make_shared<ChMaterial3DStressNeoHookean>();
    elastic_material3->SetDensity(1000);
    elastic_material3->SetYoungModulus(3e6);
    elastic_material3->SetPoissonRatio(0.39);

    if (test_box_uniaxial_pressure(elastic_material3, -20000, 6.550e-3, 3.0, 0.5))
        return 1;

    
    std::cout << "Unit test check succeeded" << std::endl;


    return 0;
}



