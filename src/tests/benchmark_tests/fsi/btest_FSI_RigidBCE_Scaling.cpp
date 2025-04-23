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
// Authors: Huzaifa Unjhawala
// =============================================================================
//
// Benchmark test for understanding the scaling of computational cost with
// increase in number of rigid bodies and thus the number of rigid BCEs
//
// =============================================================================

// #include <cassert>
// #include <cstdlib>
// #include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/ChFsiBenchmark.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// =============================================================================
template <unsigned int num_boxes>
class FsiRigidBceScalingTest : public chrono::fsi::ChBenchmarkTest {
  public:
    FsiRigidBceScalingTest();
    ~FsiRigidBceScalingTest() = default;

    ChFsiSystem* GetSystem() override { return m_sysFSI.get(); }
    void ExecuteStep() override;

    void SimulateVis();

  private:
    std::unique_ptr<ChFsiSystemSPH> m_sysFSI;
    std::unique_ptr<ChFsiFluidSystemSPH> m_sysSPH;
    std::unique_ptr<ChSystem> m_sysMBS;

    unsigned int m_num_boxes;
    double m_step_size;
    ChVector3d m_box_size;
    int m_num_fluid_particles;
    int m_num_rigid_body_particles;
    int m_num_boundary_particles;
    int m_boxes_per_layer;
};

std::vector<ChVector3d> GenerateBoxPositions(const unsigned int num_boxes,
                                             const unsigned int boxes_per_layer,
                                             const ChVector3d& box_size,
                                             const ChVector3d& granular_bin_size,
                                             double initial_spacing);

template <unsigned int num_boxes>
FsiRigidBceScalingTest<num_boxes>::FsiRigidBceScalingTest() {
    m_step_size = 1e-4;
    m_num_boxes = num_boxes;
    m_boxes_per_layer = 100;
    m_sysMBS = std::make_unique<ChSystemSMC>();
    m_sysSPH = std::make_unique<ChFsiFluidSystemSPH>();
    m_sysFSI = std::make_unique<ChFsiSystemSPH>(*m_sysMBS, *m_sysSPH);

    m_sysMBS->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    m_sysFSI->SetVerbose(false);

    int box_multiplier = 1.2;

    ChFsiFluidSystemSPH::ElasticMaterialProperties material;
    material.density = 1760;       // kg / m^3
    material.Young_modulus = 1e6;  // kg / m s ^2
    material.Poisson_ratio = 0.3;
    material.mu_I0 = 0.03;
    material.mu_fric_s = 0.793;
    material.mu_fric_2 = 0.793;
    material.average_diam = 0.005;  // m
    material.cohesion_coeff = 0;    // default

    m_sysSPH->SetElasticSPH(material);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = 0.02;  // 2 cm
    sph_params.d0_multiplier = 1.2;
    sph_params.artificial_viscosity = 0.5;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.kernel_threshold = 0.8;
    sph_params.max_velocity = 1.0;
    sph_params.num_proximity_search_steps = 1;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;

    m_sysSPH->SetSPHParameters(sph_params);

    m_sysSPH->SetStepSize(m_step_size);

    ChVector3d gravity(0, 0, -9.81);
    m_sysMBS->SetGravitationalAcceleration(gravity);
    m_sysSPH->SetGravitationalAcceleration(gravity);
    // Fixed granular box size that fits 50 boxes of size 10 x 10 x 6 cm with initial spacing between them
    // Set the depth to be 2 times the box clearance
    int boxes_in_x = m_boxes_per_layer / 10;
    int boxes_in_y = m_boxes_per_layer / 10;

    m_box_size = ChVector3d((0.1 + sph_params.initial_spacing) * boxes_in_x,
                            (0.1 + sph_params.initial_spacing) * boxes_in_y, 0.06 * 2);

    // Granular bed container size
    m_sysSPH->SetContainerDim(
        ChVector3d(m_box_size.x() * box_multiplier, m_box_size.y() * box_multiplier, m_box_size.z()));
    m_sysSPH->SetConsistentDerivativeDiscretization(false, false);  // No consistent discertization

    // Set domain boundaries, based on actual number of boxes (added in layers in the Z direction)
    ChVector3d cMin(-m_box_size.x() / 2, -m_box_size.y() / 2,
                    -box_multiplier * m_box_size.z() * (m_num_boxes / m_boxes_per_layer + 1));
    ChVector3d cMax(+m_box_size.x() / 2, +m_box_size.y() / 2,
                    +box_multiplier * m_box_size.z() * (m_num_boxes / m_boxes_per_layer + 1));
    m_sysSPH->SetComputationalDomain(ChAABB(cMin, cMax), BC_ALL_PERIODIC);

    chrono::utils::ChGridSampler<> sampler(sph_params.initial_spacing);
    ChVector3d boxCenter(0.0, 0.0, 0.0);
    ChVector3d boxHalfDim(m_box_size.x() / 2 - sph_params.initial_spacing,
                          m_box_size.y() / 2 - sph_params.initial_spacing,
                          m_box_size.z() / 2 - sph_params.initial_spacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = (int)points.size();
    double gz = gravity.z();
    for (int i = 0; i < numPart; i++) {
        double pre_ini = m_sysSPH->GetDensity() * std::abs(gz) * (-(points[i].z() + boxHalfDim.z()) + boxHalfDim.z());
        m_sysSPH->AddSPHParticle(points[i], m_sysSPH->GetDensity(), 0, m_sysSPH->GetViscosity(),
                                 ChVector3d(0),         // initial velocity
                                 ChVector3d(-pre_ini),  // tauxxyyzz
                                 ChVector3d(0)          // tauxyxzyz
        );
    }

    // Create Granular Box
    // Common contact material
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e9);
    cmaterial->SetFriction(1.f);
    cmaterial->SetRestitution(0.4f);
    cmaterial->SetAdhesion(0);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0., 0., 0.));
    ground->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ground->SetFixed(true);
    m_sysMBS->AddBody(ground);

    chrono::utils::AddBoxContainer(
        ground, cmaterial,                         //
        ChFrame<>(ChVector3d(0., 0., 0.), QUNIT),  //
        ChVector3d(box_multiplier * m_box_size.x(), box_multiplier * m_box_size.y(), m_box_size.z()),
        0.1,                   //
        ChVector3i(0, 0, -1),  //
        false);
    ground->EnableCollision(true);

    // Add BCE particles attached on the walls into FSI system
    m_sysSPH->AddBoxContainerBCE(ground,                                    //
                                 ChFrame<>(ChVector3d(0., 0., 0.), QUNIT),  //
                                 ChVector3d(box_multiplier * m_box_size.x(), box_multiplier * m_box_size.y(),
                                            m_box_size.z()),  //
                                 ChVector3i(0, 0, -1));

    // =========================================================================
    // Create rigid bodies
    // =========================================================================
    ChVector3d box_size(0.1, 0.1, 0.06);
    std::vector<ChVector3d> box_positions =
        GenerateBoxPositions(m_num_boxes, m_boxes_per_layer, box_size, m_box_size, sph_params.initial_spacing);

    for (const auto& pos : box_positions) {
        auto box = chrono_types::make_shared<ChBody>();
        box->SetPos(pos);
        box->SetRot(ChQuaternion<>(1, 0, 0, 0));
        box->SetFixed(false);
        box->AddVisualShape(chrono_types::make_shared<ChVisualShapeBox>(box_size), ChFrame<>());
        m_sysMBS->AddBody(box);
        m_sysFSI->AddFsiBody(box);
        m_sysSPH->AddBoxBCE(box, ChFrame<>(), box_size, true);
    }

    m_sysFSI->Initialize();
}

std::vector<ChVector3d> GenerateBoxPositions(const unsigned int num_boxes,
                                             const unsigned int boxes_per_layer,
                                             const ChVector3d& box_size,
                                             const ChVector3d& granular_bin_size,
                                             double initial_spacing) {
    std::vector<ChVector3d> positions;
    double box_spacing = box_size.x() + initial_spacing;

    unsigned int num_layers = (num_boxes + boxes_per_layer - 1) / boxes_per_layer;  // Calculate number of layers needed

    for (unsigned int layer = 0; layer < num_layers; ++layer) {
        for (unsigned int i = 0; i < boxes_per_layer && positions.size() < num_boxes; ++i) {
            int x_index = i % 10;
            int y_index = i / 10;
            double x_pos = (x_index - 4.5) * box_spacing;  // Center the grid around x = 0
            double y_pos = (y_index - 4.5) * box_spacing;  // Center the grid around y = 0
            double z_pos = layer * (box_size.z() + initial_spacing) + granular_bin_size.z() / 2 + initial_spacing;
            positions.emplace_back(x_pos, y_pos, z_pos);
        }
    }

    return positions;
}

template <unsigned int num_boxes>
void FsiRigidBceScalingTest<num_boxes>::ExecuteStep() {
    m_sysFSI->DoStepDynamics(m_step_size);
}

template <unsigned int num_boxes>
void FsiRigidBceScalingTest<num_boxes>::SimulateVis() {
#ifdef CHRONO_VSG
    // FSI plugin
    auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&m_sysFSI->GetFluidSystemSPH());
    visFSI->EnableFluidMarkers(true);
    visFSI->EnableBoundaryMarkers(true);
    visFSI->EnableRigidBodyMarkers(false);

    // VSG visual system (attach visFSI as plugin)
    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visFSI);
    visVSG->AttachSystem(&m_sysFSI->GetMultibodySystem());
    visVSG->SetWindowTitle("FSI Box Benchmark");
    visVSG->SetWindowSize(1280, 800);
    visVSG->AddCamera(ChVector3d(0, -3 * m_box_size.y(), 0.75 * m_box_size.z()),
                      ChVector3d(0, 0, 0.75 * m_box_size.z()));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

    visVSG->Initialize();

    while (visVSG->Run()) {
        visVSG->Render();
        ExecuteStep();
    }
#endif
}

// =============================================================================
#define NUM_SKIP_STEPS 2500  // number of steps for hot start (1e-4 * 2500 = 0.25s)
#define NUM_SIM_STEPS 10000  // number of simulation steps for each benchmark (1e-4 * 10000 = 1s)
#define REPEATS 5

CH_BM_SIMULATION_ONCE(FSI_RigidBceScaling_1, FsiRigidBceScalingTest<1>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(FSI_RigidBceScaling_10, FsiRigidBceScalingTest<10>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(FSI_RigidBceScaling_100, FsiRigidBceScalingTest<100>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(FSI_RigidBceScaling_500, FsiRigidBceScalingTest<500>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(FSI_RigidBceScaling_1000, FsiRigidBceScalingTest<1000>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(FSI_RigidBceScaling_2000, FsiRigidBceScalingTest<2000>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(FSI_RigidBceScaling_3000, FsiRigidBceScalingTest<3000>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(FSI_RigidBceScaling_4000, FsiRigidBceScalingTest<4000>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_BM_SIMULATION_ONCE(FSI_RigidBceScaling_5000, FsiRigidBceScalingTest<5000>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_VSG
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        FsiRigidBceScalingTest<500> scalingTest;
        scalingTest.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}