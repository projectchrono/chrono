// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Chrono::Multicore benchmark program using SMC method for frictional contact.
//
// The global reference frame has Z up.
// =============================================================================

// Run benchamrk tests for a number of threads between MIN and MAX (inclusive) 
// in increments of STEP.
#define TEST_MIN_THREADS 1
#define TEST_MAX_THREADS 16
#define TEST_STEP_THREADS 1

// =============================================================================

#include <cstdio>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChBenchmark.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using namespace chrono;

class SettlingSMC : public utils::ChBenchmarkTest {
  public:
    SettlingSMC();
    ~SettlingSMC() { delete m_system; }

    void SetNumthreads(int nthreads) { m_system->SetNumThreads(nthreads); }
    unsigned int GetNumParticles() const { return m_num_particles; }
    void SimulateVis();

    virtual ChSystem* GetSystem() override { return m_system; }
    virtual void ExecuteStep() override { m_system->DoStepDynamics(m_step); }

  private:
    ChSystemMulticoreSMC* m_system;
    double m_step;
    unsigned int m_num_particles;
};

SettlingSMC::SettlingSMC() : m_system(new ChSystemMulticoreSMC), m_step(1e-3) {
    // Simulation parameters
    double gravity = 9.81;

    uint max_iteration = 100;
    real tolerance = 1e-3;

    // Set gravitational acceleration
    m_system->Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    m_system->GetSettings()->solver.max_iteration_bilateral = max_iteration;
    m_system->GetSettings()->solver.tolerance = tolerance;

    m_system->GetSettings()->collision.narrowphase_algorithm = collision::ChNarrowphase::Algorithm::HYBRID;
    m_system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 1);

    // The following two lines are optional, since they are the default options.
    m_system->GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    m_system->GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    // Material properties (shared)
    float Y = 2e6f;
    float mu = 0.4f;
    float cr = 0.4f;
    // Create a common material
    auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(Y);
    mat->SetFriction(mu);
    mat->SetRestitution(cr);
    mat->SetAdhesion(0);

    // Container half-dimensions
    ChVector<> hdim(2, 2, 0.5);
    double hthick = 0.1;

    // Create a bin consisting of five boxes attached to the ground.
    auto bin = std::shared_ptr<ChBody>(m_system->NewBody());
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(-hdim.x() - hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(hdim.x() + hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, -hdim.y() - hthick, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, hdim.y() + hthick, hdim.z()));
    bin->GetCollisionModel()->BuildModel();

    m_system->AddBody(bin);

    // Create granular material in layers
    double rho = 2000;
    double radius = 0.02;
    int num_layers = 8;

    // Create a particle generator and a mixture entirely made out of spheres
    double r = 1.01 * radius;
    utils::PDSampler<double> sampler(2 * r);
    utils::Generator gen(m_system);
    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::MixtureType::SPHERE, 1.0);
    m1->setDefaultMaterial(mat);
    m1->setDefaultDensity(rho);
    m1->setDefaultSize(radius);

    // Create particles in layers until reaching the desired number of particles
    ChVector<> range(hdim.x() - r, hdim.y() - r, 0);
    ChVector<> center(0, 0, 2 * r);
    for (int il = 0; il < num_layers; il++) {
        gen.CreateObjectsBox(sampler, center, range);
        center.z() += 2 * r;
    }

    m_num_particles = gen.getTotalNumBodies();
}

// Run settling simulation with visualization
void SettlingSMC::SimulateVis() {
#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(m_system);
    vis.SetWindowTitle("Settling test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector<>(0, -6, 0), ChVector<>(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    while (vis.Run()) {
        ExecuteStep();
        vis.Render();
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 500  // number of steps for hot start
#define NUM_SIM_STEPS 500   // number of simulation steps for benchmarking

using TEST_NAME = chrono::utils::ChBenchmarkFixture<SettlingSMC, 0>;
BENCHMARK_DEFINE_F(TEST_NAME, Settle)(benchmark::State& st) {
    Reset(NUM_SKIP_STEPS);
    m_test->SetNumthreads((int)st.range(0));
    while (st.KeepRunning()) {
        m_test->Simulate(NUM_SIM_STEPS);
    }
    Report(st);
    std::cout << "Simulated " << m_test->GetNumParticles() << " particles ";
#pragma omp parallel
#pragma omp master
    std::cout << "using " << ChOMP::GetNumThreads() << " threads." << std::endl;
}
BENCHMARK_REGISTER_F(TEST_NAME, Settle)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(1)
    ->Repetitions(1)
    ->UseRealTime()
    ->DenseRange(TEST_MIN_THREADS, TEST_MAX_THREADS, TEST_STEP_THREADS);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        SettlingSMC test;
        test.SetNumthreads(8);
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
