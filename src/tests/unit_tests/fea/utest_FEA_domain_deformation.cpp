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
// finite elements of ChFEModelDeformation.
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/fea/multiphysics/ChNodeFEAfieldXYZ.h"
#include "chrono/fea/multiphysics/ChFEModelDeformation.h"
#include "chrono/fea/multiphysics/ChDrawer.h"
#include "chrono/fea/multiphysics/ChSurfaceOfModel.h"
#include "chrono/fea/multiphysics/ChFieldElementHexahedron8.h"
#include "chrono/fea/multiphysics/ChFieldElementHexahedron8Face.h"
#include "chrono/fea/multiphysics/ChFieldElementTetrahedron4.h"
#include "chrono/fea/multiphysics/ChFieldElementTetrahedron4Face.h"
#include "chrono/fea/multiphysics/ChFieldElementLoadableVolume.h"
#include "chrono/fea/multiphysics/ChFieldElementLoadableSurface.h"
#include "chrono/fea/ChLoaderPressure.h"
#include "chrono/fea/multiphysics/ChBuilderVolume.h"
#include "chrono/fea/multiphysics/ChMaterial3DStressOgden.h"
#include "chrono/fea/multiphysics/ChMaterial3DStressNeoHookean.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif


using namespace chrono;
using namespace chrono::fea;



bool test_box_uniaxial_pressure(std::shared_ptr<ChMaterial3DStress> test_material, double test_pressure, double reference_x_displ, double tolerance, double end_time) {
    ChSystemNSC sys;

    auto displacement_field = chrono_types::make_shared<ChFieldDisplacement3D>();
    sys.Add(displacement_field);

    auto elastic_model = chrono_types::make_shared<ChFEModelDeformation>(displacement_field);
    sys.Add(elastic_model);

    elastic_model->SetAutomaticGravity(false);

    elastic_model->material = test_material;  // set the material in model

    // Mesh resolution and box dimensions. The constrained face and the probed node are addressed
    // through these same constants below, so they cannot drift out of sync with the mesh.
    const int nel_x = 10, nel_y = 4, nel_z = 4;      // number of elements along x,y,z
    const double W_x = 1.0, W_y = 0.5, W_z = 0.5;    // box dimensions along x,y,z

    ChBuilderVolumeBox builder;
    builder.BuildVolume(ChFrame<>(), nel_x, nel_y, nel_z, W_x, W_y, W_z);
    builder.AddToModel(elastic_model);

    // Fix the entire x=0 face. The nodes are addressed by grid index rather than by comparing
    // coordinates against 0, so this does not depend on exact floating-point node positions.
    for (int iy = 0; iy <= nel_y; ++iy)
        for (int iz = 0; iz <= nel_z; ++iz)
            displacement_field->NodeData(builder.nodes.at(0, iy, iz)).SetFixed(true);

    // Probe the corner node of the loaded x=W_x face.
    // This is the node that the original "last node with x <= 1" loop ended up selecting (every node
    // of the box satisfies x <= W_x, so that test was vacuous and the choice was really determined by
    // the order in which ChBuilderVolumeBox happens to store its nodes). The reference values below
    // are therefore unchanged; addressing the node explicitly just makes the choice independent of
    // the builder's internal storage order.
    auto probed_node = builder.nodes.at(nel_x, nel_y, nel_z);

    std::cout << "  mesh: " << nel_x << "x" << nel_y << "x" << nel_z << " hexa8 elements ("
              << builder.elements.list().size() << " elements, " << builder.nodes.list().size()
              << " nodes)\n";
    std::cout << "  probed node: grid index (" << nel_x << "," << nel_y << "," << nel_z
              << "), reference position = " << probed_node->GetReferencePos() << "\n";

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
        std::cout << "Using PardisoMKL" << std::endl;
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        mkl_solver->SetVerbose(false);
        sys.SetSolver(mkl_solver);
    #endif
    } else {
        std::cout << "Using MINRES" << std::endl;
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

        auto visual_nodes = chrono_types::make_shared<ChVisualModelGlyphs>(elastic_model);
        visual_nodes->SetGlyphsSize(0.1);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractPosDt(), 0.0, 2.0, "Vel");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        elastic_model->AddVisualShape(visual_nodes);

        auto visual_stress = chrono_types::make_shared<ChVisualModelGlyphs>(elastic_model);
        visual_stress->SetGlyphsSize(0.5);
        visual_stress->AddPositionExtractor(ExtractPos());
        visual_stress->AddPropertyExtractor(ChFEModelDeformation::ExtractEulerAlmansiStrain(), 0.0, 2.0, "e");
        visual_stress->SetColormap(ChColormap(ChColormap::Type::JET));
        elastic_model->AddVisualShape(visual_stress);

        // original undeformed
        auto visual_mesh = chrono_types::make_shared<ChVisualModelMesh>(elastic_model);
        visual_mesh->SetColormap(ChColor(1, 1, 1));
        visual_mesh->SetWireframe(true);
        elastic_model->AddVisualShape(visual_mesh);

        auto visual_mesh2 = chrono_types::make_shared<ChVisualModelMesh>(elastic_model);
        visual_mesh2->AddPositionExtractor(ExtractPos());
        visual_mesh2->AddPropertyExtractor(ChFEModelDeformation::ExtractEulerAlmansiStrain().VonMises(), -0.1, 0.1, "Stretch");
        visual_mesh2->SetColormap(ChColormap(ChColormap::Type::JET));
        // visual_mesh2->SetWireframe(true);
        visual_mesh2->SetShrinkElements(true, 0.99);
        elastic_model->AddVisualShape(visual_mesh2);

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
        std::cout << "  pressure = " << test_pressure << ", timestep = " << timestep
                  << ", end time = " << end_time << " (" << (int)std::ceil(end_time / timestep)
                  << " steps)\n";
        while (sys.GetChTime() < end_time) {
            sys.DoStepDynamics(timestep);
        }
    }

    // Reported after the run: the DOF counts are only populated once the system has been set up.
    std::cout << "  field: " << displacement_field->GetNumNodes() << " nodes, "
              << displacement_field->GetNumCoordsPosLevel() << " coords at position level ("
              << displacement_field->GetNumCoordsVelLevel() << " at velocity level)\n";

    // Fetch displaced position of probed node, from the displacement_field:
    ChVector3d pos_probed = displacement_field->NodeData(probed_node).GetPos();
    ChVector3d pos_reference = probed_node->GetReferencePos();

    ChVector3d result_node_displ = pos_probed - pos_reference;

    // Under uniform uniaxial pressure every node of the loaded x=W_x face should displace by very
    // nearly the same amount along x. A large spread here means the solution itself is corrupt
    // (rather than merely inaccurate), which is a far more useful signal than the single probed
    // value alone -- it is exactly what a garbage element tangent matrix produces.
    double dx_min = +1e30, dx_max = -1e30;
    bool any_nonfinite = false;
    for (int iy = 0; iy <= nel_y; ++iy) {
        for (int iz = 0; iz <= nel_z; ++iz) {
            auto face_node = builder.nodes.at(nel_x, iy, iz);
            double dx = (displacement_field->NodeData(face_node).GetPos() - face_node->GetReferencePos()).x();
            if (!std::isfinite(dx))
                any_nonfinite = true;
            dx_min = std::min(dx_min, dx);
            dx_max = std::max(dx_max, dx);
        }
    }
    std::cout << "  loaded face x displacement: min = " << dx_min << ", max = " << dx_max
              << ", spread = " << (dx_max - dx_min) << "\n";
    if (any_nonfinite)
        std::cout << "  WARNING: non-finite displacement on the loaded face (corrupt solution)\n";

    std::cout << "  displacement of probed node = " << result_node_displ << "\n";
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



