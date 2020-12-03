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

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNodeGranularSPH.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono system and set solver parameters
// - create the OpenGL visualization window
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeGranularSPH::ChVehicleCosimTerrainNodeGranularSPH(ChContactMethod method, bool render)
    : ChVehicleCosimTerrainNode(Type::GRANULAR_SPH, method, render) {
    cout << "[Terrain node] GRANULAR_SPH "
         << " method = " << static_cast<std::underlying_type<ChContactMethod>::type>(method) << endl;

    // --------------------------
    // Create the parallel system
    // --------------------------

    // Create system and set default method-specific solver settings
    switch (m_method) {
        case ChContactMethod::SMC: {
            ChSystemParallelSMC* sys = new ChSystemParallelSMC;
            sys->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
            sys->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
            sys->GetSettings()->solver.use_material_properties = true;
            m_system = sys;
            break;
        }
        case ChContactMethod::NSC: {
            ChSystemParallelNSC* sys = new ChSystemParallelNSC;
            sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
            sys->GetSettings()->solver.max_iteration_normal = 0;
            sys->GetSettings()->solver.max_iteration_sliding = 200;
            sys->GetSettings()->solver.max_iteration_spinning = 0;
            sys->GetSettings()->solver.alpha = 0;
            sys->GetSettings()->solver.contact_recovery_speed = -1;
            sys->GetSettings()->collision.collision_envelope = 0.001;
            sys->ChangeSolverType(SolverType::APGD);
            m_system = sys;
            break;
        }
    }

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set number of threads
    m_system->SetNumThreads(1);

ChVehicleCosimTerrainNodeGranularSPH::~ChVehicleCosimTerrainNodeGranularSPH() {
    delete m_system;
}


void ChVehicleCosimRigNode::SetTerrainJSONFile(const std::string& filename) {
    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_CylinderDrop_Explicit.json");
    if (argc == 1) {
        fsi::utils::ParseJSON(inputJson, paramsH, fsi::mR3(bxDim, byDim, bzDim));
    } else if (argc == 2) {
        fsi::utils::ParseJSON(argv[1], paramsH, fsi::mR3(bxDim, byDim, bzDim));
        std::string input_json = std::string(argv[1]);
        inputJson = GetChronoDataFile(input_json);
    } else {
        ShowUsage();
        return 1;
    }

    // Dimension of the space domain
    bxDim = paramsH->boxDimX;
    byDim = paramsH->boxDimY;
    bzDim = paramsH->boxDimZ;
    // Dimension of the fluid domain
    fxDim = paramsH->fluidDimX;
    fyDim = paramsH->fluidDimY;
    fzDim = paramsH->fluidDimZ;

    // Size of the dropping cylinder
    cyl_radius = paramsH->bodyRad;
    cyl_length = paramsH->bodyLength;

    // Set up the periodic boundary condition (if not, set relative larger values)
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    paramsH->cMin = chrono::fsi::mR3(-bxDim / 2, -byDim / 2, -bzDim - 10 * initSpace0) * 10;
    paramsH->cMax = chrono::fsi::mR3( bxDim / 2,  byDim / 2,  bzDim + 10 * initSpace0) * 10;
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Initialize.
// - adjust system settings
// - create the container body
// - if specified, create the granular material
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularSPH::Construct() {
    // ---------------------
    // Create container body
    // ---------------------

    auto container = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(container);
    container->SetIdentifier(-1);
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(true);

    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), m_material_terrain, ChVector<>(m_hdimX, m_hdimY, 0.1),
                          ChVector<>(0, 0, -0.1), ChQuaternion<>(1, 0, 0, 0), true);
    container->GetCollisionModel()->BuildModel();


    // Create the geometry of the boundaries
    double initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

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
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, container, pos_zp, chrono::QUNIT, size_XY, 12);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, container, pos_zn, chrono::QUNIT, size_XY, 12);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, container, pos_xp, chrono::QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, container, pos_xn, chrono::QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, container, pos_yp, chrono::QUNIT, size_XZ, 13);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, container, pos_yn, chrono::QUNIT, size_XZ, 13);


    // --------------------------------------
    // Write file with terrain node settings
    // --------------------------------------

    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.dat", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "   Contact method = " << (m_method == ChContactMethod::SMC ? "SMC" : "NSC") << endl;
    outf << "   Use material properties? " << (m_system->GetSettings()->solver.use_material_properties ? "YES" : "NO")
         << endl;
    outf << "   Collision envelope = " << m_system->GetSettings()->collision.collision_envelope << endl;
    outf << "Patch dimensions" << endl;
    outf << "   X = " << 2 * m_hdimX << "  Y = " << 2 * m_hdimY << endl;
    outf << "Terrain material properties" << endl;
    switch (m_method) {
        case ChContactMethod::SMC: {
            auto mat = std::static_pointer_cast<ChMaterialSurfaceSMC>(m_material_terrain);
            outf << "   Coefficient of friction    = " << mat->GetKfriction() << endl;
            outf << "   Coefficient of restitution = " << mat->GetRestitution() << endl;
            outf << "   Young modulus              = " << mat->GetYoungModulus() << endl;
            outf << "   Poisson ratio              = " << mat->GetPoissonRatio() << endl;
            outf << "   Adhesion force             = " << mat->GetAdhesion() << endl;
            outf << "   Kn = " << mat->GetKn() << endl;
            outf << "   Gn = " << mat->GetGn() << endl;
            outf << "   Kt = " << mat->GetKt() << endl;
            outf << "   Gt = " << mat->GetGt() << endl;
            break;
        }
        case ChContactMethod::NSC: {
            auto mat = std::static_pointer_cast<ChMaterialSurfaceNSC>(m_material_terrain);
            outf << "   Coefficient of friction    = " << mat->GetKfriction() << endl;
            outf << "   Coefficient of restitution = " << mat->GetRestitution() << endl;
            outf << "   Cohesion force             = " << mat->GetCohesion() << endl;
            break;
        }
    }
    outf << "Proxy body properties" << endl;
    outf << "   proxies fixed? " << (m_fixed_proxies ? "YES" : "NO") << endl;
    outf << "   proxy contact radius = " << m_radius_p << endl;
}


void ChVehicleCosimTerrainNodeGranularSPH::CreateWheelProxy() {
    // Create wheel proxy body
    auto body = std::shared_ptr<ChBody>(m_system->NewBody());
    body->SetIdentifier(0);
    body->SetMass(m_rig_mass);
    ////body->SetInertiaXX();   //// TODO
    body->SetBodyFixed(m_fixed_proxies);
    body->SetCollide(true);

    // Create collision mesh
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->getCoordsVertices() = m_mesh_data.vpos;
    trimesh->getCoordsNormals() = m_mesh_data.vnrm;
    trimesh->getIndicesVertexes() = m_mesh_data.tri;

    // Set collision shape
    body->GetCollisionModel()->ClearModel();
    body->GetCollisionModel()->AddTriangleMesh(m_material_tire, trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                               m_radius_p);
    body->GetCollisionModel()->SetFamily(1);
    body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
    body->GetCollisionModel()->BuildModel();

    // Set visualization asset
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->Pos = ChVector<>(0, 0, 0);
    trimesh_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
    body->GetAssets().push_back(trimesh_shape);

    m_system->AddBody(body);
    m_proxies.push_back(ProxyBody(body, 0));

    // Add this body to the FSI system (only those have inetraction with fluid)
    myFsiSystem.AddFsiBody(body);

    // Add BCE particles attached on the cylinder into FSI system
    double cyl_radius = 0.1;
    double cyl_length = 0.2;
    fsi::utils::AddCylinderBce(myFsiSystem.GetDataManager(), paramsH, body, ChVector<>(0, 0, 0),
                               ChQuaternion<>(1, 0, 0, 0), cyl_radius, cyl_length + initSpace0, paramsH->HSML, false);

}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeGranularSPH::UpdateWheelProxy() {
    m_proxies[0].m_body->SetPos(m_wheel_state.pos);
    m_proxies[0].m_body->SetPos_dt(m_wheel_state.lin_vel);
    m_proxies[0].m_body->SetRot(m_wheel_state.rot);
    m_proxies[0].m_body->SetWvel_par(m_wheel_state.ang_vel);

    fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiData->fsiBodiesD2);
    bceWorker->UpdateRigidMarkersPositionVelocity(fsiData->sphMarkersD2, fsiData->fsiBodiesD2);
}

// Collect resultant contact force and torque on wheel proxy body.
void ChVehicleCosimTerrainNodeGranularSPH::GetForceWheelProxy() {
    // get force from BCE particles
    bceWorker->Rigid_Forces_Torques(fsiData->sphMarkersD2, fsiData->fsiBodiesD2);
    fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

    m_wheel_contact.point = ChVector<>(0, 0, 0);
    m_wheel_contact.force = m_proxies[0].m_body->GetContactForce();
    m_wheel_contact.moment = m_proxies[0].m_body->GetContactTorque();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::OnSynchronize(int step_number, double time) {
    // Nothing needed here
}

void ChVehicleCosimTerrainNodeGranularSPH::OnAdvance(double step_size) {
    fluidDynamics->IntegrateSPH(fsiData->sphMarkersD2, 
                                fsiData->sphMarkersD1, 
                                fsiData->fsiBodiesD2,
                                fsiData->fsiMeshD, 
                                0.5 * paramsH->dT);
    fluidDynamics->IntegrateSPH(fsiData->sphMarkersD1, 
                                fsiData->sphMarkersD2, 
                                fsiData->fsiBodiesD2,
                                fsiData->fsiMeshD, 
                                1.0 * paramsH->dT);

    // Force a calculation of cumulative contact forces for all bodies in the system
    // (needed at the next synchronization)
    m_system->CalculateContactForces();
}

// -----------------------------------------------------------------------------
void SetFluidDynamicsInFSI(paramsH->fluid_dynamic_type){
    // Set the time integration type and the linear solver type (only for ISPH)
    myFsiSystem.SetFluidDynamics(paramsH->fluid_dynamic_type);
}

void SetFluidSystemLinearSolverInFSI(paramsH->LinearSolver){
    // Set the time integration type and the linear solver type (only for ISPH)
    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);
}

void FinalizeDomainInFSI(paramsH){
    // Call FinalizeDomainCreating to setup the binning for neighbor search
    fsi::utils::FinalizeDomain(paramsH);
}

void PrepareOutputDirInFSI(paramsH, demo_dir, out_dir, inputJson){
    // Set the time integration type and the linear solver type (only for ISPH)
    fsi::utils::PrepareOutputDir(paramsH, demo_dir, out_dir, inputJson);
}

void CreateSPHparticlesInFSI(mphysicalSystem, myFsiSystem, paramsH){
    // Call FinalizeDomainCreating to setup the binning for neighbor search
    fsi::utils::FinalizeDomain(paramsH);
    fsi::utils::PrepareOutputDir(paramsH, demo_dir, out_dir, inputJson);

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
        Real pre_ini = paramsH->rho0 * abs(paramsH->gravity.z) * (-points[i].z() + fzDim);
        Real rho_ini = paramsH->rho0 + pre_ini / (paramsH->Cs * paramsH->Cs);
        myFsiSystem.GetDataManager()->AddSphMarker(
            fsi::mR4(points[i].x(), points[i].y(), points[i].z(), paramsH->HSML),
            fsi::mR3(1e-10),
            fsi::mR4(rho_ini, pre_ini, paramsH->mu0, -1));
    }
    size_t numPhases = myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.size();
    if (numPhases != 0) {
        std::cout << "Error! numPhases is wrong, thrown from main\n" << std::endl;
        std::cin.get();
        return -1;
    } else {
        myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4(0, (int)numPart, -1, -1));
        myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4((int)numPart, (int)numPart, 0, 0));
    }
}

void FinalizeInFSI(){
    // Construction of the FSI system must be finalized before running
    myFsiSystem.Finalize();
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
