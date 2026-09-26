// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Unit test for exception propagation out of ChFsiSystem::DoStepDynamics.
//
// An exception thrown while advancing either the fluid or the multibody phase
// must reach the caller, so that a host application can catch a failed step and
// report it, retry, or shut down cleanly. With the concurrent coupling scheme the
// multibody phase runs on a separate thread, and an exception on either thread
// used to call std::terminate (issue #826). A regression therefore aborts this
// test program instead of failing an assertion.
//
// The fluid solver is a stub that does no work, so the test needs neither a GPU
// nor any particular fluid solver module.
//
// =============================================================================

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_fsi/ChFsiFluidSystem.h"
#include "chrono_fsi/ChFsiInterface.h"
#include "chrono_fsi/ChFsiSystem.h"

using namespace chrono;
using namespace chrono::fsi;

// Fluid system that does nothing, or throws from its step when asked to.
class StubFluidSystem : public ChFsiFluidSystem {
  public:
    bool fail = false;

    virtual void SetGravitationalAcceleration(const ChVector3d& gravity) override {}
    virtual void OnDoStepDynamics(double time, double step) override {
        if (fail)
            throw std::runtime_error("CFD failed");
    }
    virtual void OnExchangeSolidForces() override {}
    virtual void OnExchangeSolidStates() override {}
    virtual void LoadSolidStates(const std::vector<FsiBodyState>& body_states) override {}
    virtual void StoreSolidForces(std::vector<FsiBodyForce>& body_forces) override {}
#ifdef CHRONO_FEA
    virtual void LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                 const std::vector<FsiMeshState>& mesh1D_states,
                                 const std::vector<FsiMeshState>& mesh2D_states) override {}
    virtual void StoreSolidForces(std::vector<FsiBodyForce>& body_forces, std::vector<FsiMeshForce>& mesh1D_forces, std::vector<FsiMeshForce>& mesh2D_forces) override {}
#endif

  protected:
    virtual void Initialize(const std::vector<FsiBodyState>& body_states) override { m_is_initialized = true; }
#ifdef CHRONO_FEA
    virtual void Initialize(const std::vector<FsiBodyState>& body_states, const std::vector<FsiMeshState>& mesh1D_states, const std::vector<FsiMeshState>& mesh2D_states) override {
        m_is_initialized = true;
    }
#endif
};

// Multibody advance that records completion, or throws when asked to.
// The delay makes the multibody phase outlast a failing fluid phase, so that the fluid exception unwinds while the
// multibody thread is still running.
class StubMBDCallback : public ChFsiSystem::MBDCallback {
  public:
    bool fail = false;
    std::atomic<bool> done{false};

    virtual void Advance(double step, double threshold) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        done = true;
        if (fail)
            throw std::runtime_error("MBS failed");
    }
};

// FSI system coupling the stub fluid system and an empty multibody system, with a selectable coupling scheme.
class StubFsiSystem : public ChFsiSystem {
  public:
    StubFsiSystem(CouplingScheme scheme) : ChFsiSystem(&m_mbs, &m_cfd) {
        m_fsi_interface = chrono_types::make_shared<ChFsiInterfaceGeneric>(&m_mbs, &m_cfd);
        SetCouplingScheme(scheme);
        SetVerbose(false);
        SetStepSizeCFD(1e-3);
        SetStepsizeMBD(1e-3);
        m_callback = chrono_types::make_shared<StubMBDCallback>();
        RegisterMBDCallback(m_callback);
        Initialize();
    }

    // Set which phases fail on the next step.
    void SetFailures(bool fail_cfd, bool fail_mbs) {
        m_cfd.fail = fail_cfd;
        m_callback->fail = fail_mbs;
        m_callback->done = false;
    }

    bool MbsDone() const { return m_callback->done; }

    // Take one step and return the message of the exception it threw, or an empty string if it did not throw.
    std::string Step() {
        try {
            DoStepDynamics(1e-3);
        } catch (const std::exception& e) {
            return e.what();
        }
        return "";
    }

  private:
    ChSystemNSC m_mbs;
    StubFluidSystem m_cfd;
    std::shared_ptr<StubMBDCallback> m_callback;
};

class FsiStepException : public ::testing::TestWithParam<ChFsiSystem::CouplingScheme> {};

TEST_P(FsiStepException, no_failure) {
    StubFsiSystem sys(GetParam());
    sys.SetFailures(false, false);
    EXPECT_EQ(sys.Step(), "");
    EXPECT_TRUE(sys.MbsDone());
}

TEST_P(FsiStepException, cfd_failure) {
    StubFsiSystem sys(GetParam());
    sys.SetFailures(true, false);
    EXPECT_EQ(sys.Step(), "CFD failed");

    // Under concurrent coupling, the multibody thread must have been joined before the exception left the step.
    if (GetParam() == ChFsiSystem::CouplingScheme::CONCURRENT)
        EXPECT_TRUE(sys.MbsDone());
}

TEST_P(FsiStepException, mbs_failure) {
    StubFsiSystem sys(GetParam());
    sys.SetFailures(false, true);
    EXPECT_EQ(sys.Step(), "MBS failed");
}

TEST_P(FsiStepException, cfd_and_mbs_failure) {
    StubFsiSystem sys(GetParam());
    sys.SetFailures(true, true);

    // Under concurrent coupling both phases run and the fluid exception is the one reported; under sequential
    // coupling the multibody phase is never reached.
    EXPECT_EQ(sys.Step(), "CFD failed");
}

TEST_P(FsiStepException, step_after_failure) {
    // A caller that catches a failed step must be able to keep using the system.
    StubFsiSystem sys(GetParam());
    sys.SetFailures(true, true);
    EXPECT_NE(sys.Step(), "");
    sys.SetFailures(false, false);
    EXPECT_EQ(sys.Step(), "");
    EXPECT_TRUE(sys.MbsDone());
}

INSTANTIATE_TEST_SUITE_P(CouplingSchemes,
                         FsiStepException,
                         ::testing::Values(ChFsiSystem::CouplingScheme::CONCURRENT, ChFsiSystem::CouplingScheme::SEQUENTIAL),
                         [](const ::testing::TestParamInfo<ChFsiSystem::CouplingScheme>& info) {
                             return info.param == ChFsiSystem::CouplingScheme::CONCURRENT ? "concurrent" : "sequential";
                         });
