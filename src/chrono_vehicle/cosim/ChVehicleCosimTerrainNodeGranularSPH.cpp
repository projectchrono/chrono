// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Wei Hu, Radu Serban
// =============================================================================
//
// Definition of the SPH granular TERRAIN NODE (using Chrono::FSI).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <set>

#include <mpi.h>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fsi/utils/ChUtilsJSON.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNodeGranularSPH.h"

using namespace chrono::fsi;

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono system and set solver parameters
// - create the Chrono FSI system
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeGranularSPH::ChVehicleCosimTerrainNodeGranularSPH(bool render)
    : ChVehicleCosimTerrainNode(Type::GRANULAR_SPH, ChContactMethod::SMC, render), m_depth(0) {
    cout << "[Terrain node] GRANULAR_SPH " << endl;

    // Create systems
    m_system = new ChSystemSMC;
    m_systemFSI = new ChSystemFsi(*m_system);

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set number of threads
    m_system->SetNumThreads(1);
}

ChVehicleCosimTerrainNodeGranularSPH::~ChVehicleCosimTerrainNodeGranularSPH() {
    delete m_systemFSI;
    delete m_system;
}

void ChVehicleCosimTerrainNodeGranularSPH::SetPropertiesSPH(const std::string& filename, double depth) {
    m_depth = depth;

    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    m_params = m_systemFSI->GetSimParams();
    fsi::utils::ParseJSON(filename, m_params, fsi::mR3(0, 0, 0));

    // Set the time integration type and the linear solver type (only for ISPH)
    m_systemFSI->SetFluidDynamics(m_params->fluid_dynamic_type);
    m_systemFSI->SetFluidSystemLinearSolver(m_params->LinearSolver);
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Initialize.
// - adjust system settings
// - create the container body
// - set m_init_height
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularSPH::Construct() {
    // Domain size
    fsi::Real bxDim = (fsi::Real)(2 * m_hdimX);
    fsi::Real byDim = (fsi::Real)(2 * m_hdimY);
    fsi::Real bzDim = (fsi::Real)(1.25 * m_depth);

    // Set up the periodic boundary condition (if not, set relative larger values)
    fsi::Real initSpace0 = m_params->MULT_INITSPACE * m_params->HSML;
    m_params->boxDimX = bxDim;
    m_params->boxDimY = byDim;
    m_params->boxDimZ = bzDim;
    m_params->cMin = chrono::fsi::mR3(-bxDim / 2, -byDim / 2, -bzDim - 10 * initSpace0) * 10;
    m_params->cMax = chrono::fsi::mR3(bxDim / 2, byDim / 2, bzDim + 10 * initSpace0) * 10;

    // Call FinalizeDomain to setup the binning for neighbor search
    fsi::utils::FinalizeDomain(m_params);

    // Create container body
    auto container = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(container);
    container->SetIdentifier(-1);
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(false);

    // Create the geometry of the boundaries

    // Bottom and Top wall - size and position
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 1 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);

    // Left and right Wall - size and position
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);

    // Front and back Wall - size and position
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 0 * initSpace0);

    // Add BCE particles attached on the walls into FSI system
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_zp, chrono::QUNIT, size_XY, 12);
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_zn, chrono::QUNIT, size_XY, 12);
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_xp, chrono::QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_xn, chrono::QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_yp, chrono::QUNIT, size_XZ, 13);
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_yn, chrono::QUNIT, size_XZ, 13);

    //// RADU TODO:  Create the fluid particles...
    //// Fluid domain:  bxDim x byDim x m_depth

    /*
            // Call FinalizeDomainCreating to setup the binning for neighbor search
            fsi::utils::FinalizeDomain(m_params);
            fsi::utils::PrepareOutputDir(m_params, demo_dir, out_dir, inputJson);

            // Create Fluid region and discretize with SPH particles
            ChVector<> boxCenter(0.0, 0.0, fzDim / 2);
            ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);

            // Use a chrono sampler to create a bucket of points
            utils::GridSampler<> sampler(initSpace0);
            utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);

            // Add fluid particles from the sampler points to the FSI system
            size_t numPart = points.size();
            for (int i = 0; i < numPart; i++) {
                // Calculate the pressure of a steady state (p = rho*g*h)
                Real pre_ini = m_params->rho0 * abs(m_params->gravity.z) * (-points[i].z() + fzDim);
                Real rho_ini = m_params->rho0 + pre_ini / (m_params->Cs * m_params->Cs);
                m_systemFSI->GetDataManager()->AddSphMarker(
                    fsi::mR4(points[i].x(), points[i].y(), points[i].z(), m_params->HSML),
                    fsi::mR3(1e-10),
                    fsi::mR4(rho_ini, pre_ini, m_params->mu0, -1));
            }
            size_t numPhases = m_systemFSI->GetDataManager()->fsiGeneralData->referenceArray.size();
            if (numPhases != 0) {
                std::cout << "Error! numPhases is wrong, thrown from main\n" << std::endl;
                std::cin.get();
                return -1;
            } else {
                m_systemFSI->GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4(0, (int)numPart, -1, -1));
                m_systemFSI->GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4((int)numPart, (int)numPart,
       0, 0));
            }


        */

    //// RADU TODO - this function must set m_init_height!
    ////m_init_height = ....

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.dat", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "Patch dimensions" << endl;
    outf << "   X = " << 2 * m_hdimX << "  Y = " << 2 * m_hdimY << endl;
    outf << "   depth = " << m_depth << endl;
}

void ChVehicleCosimTerrainNodeGranularSPH::CreateWheelProxy() {
    // Create wheel proxy body
    auto body = std::shared_ptr<ChBody>(m_system->NewBody());
    body->SetIdentifier(0);
    body->SetMass(m_rig_mass);
    body->SetBodyFixed(true);  // proxy body always fixed
    body->SetCollide(false);

    // Create collision mesh
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->getCoordsVertices() = m_mesh_data.vpos;
    trimesh->getCoordsNormals() = m_mesh_data.vnrm;
    trimesh->getIndicesVertexes() = m_mesh_data.tri;

    ////// Set collision shape
    ////body->GetCollisionModel()->ClearModel();
    ////body->GetCollisionModel()->AddTriangleMesh(m_material_tire, trimesh, false, false, ChVector<>(0),
    ///ChMatrix33<>(1), /                                           m_radius_p);
    ////body->GetCollisionModel()->SetFamily(1);
    ////body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    ////body->GetCollisionModel()->BuildModel();

    m_system->AddBody(body);
    m_proxies.push_back(ProxyBody(body, 0));

    // Add this body to the FSI system
    m_systemFSI->AddFsiBody(body);

    //// RADU TODO - Create BCE markers associated with trimesh!

    // Construction of the FSI system must be finalized before running
    m_systemFSI->Finalize();
}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeGranularSPH::UpdateWheelProxy() {
    m_proxies[0].m_body->SetPos(m_wheel_state.pos);
    m_proxies[0].m_body->SetPos_dt(m_wheel_state.lin_vel);
    m_proxies[0].m_body->SetRot(m_wheel_state.rot);
    m_proxies[0].m_body->SetWvel_par(m_wheel_state.ang_vel);
    m_proxies[0].m_body->SetWacc_par(ChVector<>(0, 0, 0));
}

// Collect resultant contact force and torque on wheel proxy body.
void ChVehicleCosimTerrainNodeGranularSPH::GetForceWheelProxy() {
    m_wheel_contact.point = ChVector<>(0, 0, 0);
    m_wheel_contact.force = m_proxies[0].m_body->Get_accumulated_force();
    m_wheel_contact.moment = m_proxies[0].m_body->Get_accumulated_torque();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::Advance(double step_size) {
    //// RADU TODO:  correlate m_step_size with m_params->dT

    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_params->dT, step_size - t);
        m_systemFSI->DoStepDynamics_FSI();
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();

    PrintWheelProxyContactData();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::OutputTerrainData(int frame) {
    //// TODO
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::PrintWheelProxyUpdateData() {
    //// TODO
}

void ChVehicleCosimTerrainNodeGranularSPH::PrintWheelProxyContactData() {
    //// TODO
}

}  // end namespace vehicle
}  // end namespace chrono
