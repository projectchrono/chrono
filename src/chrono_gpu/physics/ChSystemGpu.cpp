// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen, Dan Negrut, Radu Serban
// =============================================================================

#include <string>

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/physics/ChSystemGpuMesh_impl.h"

#include "chrono_gpu/utils/ChGpuUtilities.h"

namespace chrono {
namespace gpu {

// -----------------------------------------------------------------------------

ChSystemGpu::ChSystemGpu(float sphere_rad, float density, float3 boxDims) {
    m_sys = new ChSystemGpu_impl(sphere_rad, density, boxDims);
}

ChSystemGpuMesh::ChSystemGpuMesh(float sphere_rad, float density, float3 boxDims)
    : mesh_verbosity(CHGPU_MESH_VERBOSITY::QUIET) {
    m_sys = new ChSystemGpuMesh_impl(sphere_rad, density, boxDims);
}

ChSystemGpu::~ChSystemGpu() {
    delete m_sys;
}

ChSystemGpuMesh::~ChSystemGpuMesh() {}

// -----------------------------------------------------------------------------

void ChSystemGpu::SetGravitationalAcceleration(const ChVector<float> g) {
    m_sys->X_accGrav = (float)g.x();
    m_sys->Y_accGrav = (float)g.y();
    m_sys->Z_accGrav = (float)g.z();
}

void ChSystemGpu::SetBDFixed(bool fixed) {
    m_sys->BD_is_fixed = fixed;
}

void ChSystemGpu::SetParticleFixed(const std::vector<bool>& fixed) {
    m_sys->user_sphere_fixed = fixed;
}

void ChSystemGpu::SetOutputMode(CHGPU_OUTPUT_MODE mode) {
    m_sys->file_write_mode = mode;
}

void ChSystemGpu::SetOutputFlags(unsigned char flags) {
    m_sys->output_flags = flags;
}

void ChSystemGpu::SetFixedStepSize(float size_UU) {
    m_sys->stepSize_UU = size_UU;
}

void ChSystemGpu::DisableMinLength() {
    m_sys->use_min_length_unit = false;
}

void ChSystemGpu::SetTimeIntegrator(CHGPU_TIME_INTEGRATOR new_integrator) {
    m_sys->gran_params->time_integrator = new_integrator;
    m_sys->time_integrator = new_integrator;
}

void ChSystemGpu::SetFrictionMode(CHGPU_FRICTION_MODE new_mode) {
    m_sys->gran_params->friction_mode = new_mode;
}

void ChSystemGpu::SetRollingMode(CHGPU_ROLLING_MODE new_mode) {
    m_sys->gran_params->rolling_mode = new_mode;
}

void ChSystemGpu::SetRecordingContactInfo(bool record) {
    m_sys->gran_params->recording_contactInfo = record;
};

void ChSystemGpu::SetMaxSafeVelocity_SU(float max_vel) {
    m_sys->gran_params->max_safe_vel = max_vel;
}

void ChSystemGpu::SetPsiFactors(unsigned int psi_T, unsigned int psi_L, float psi_R) {
    m_sys->psi_T = psi_T;
    m_sys->psi_L = psi_L;
    m_sys->psi_R = psi_R;
}

// -----------------------------------------------------------------------------

void ChSystemGpu::SetStaticFrictionCoeff_SPH2SPH(float mu) {
    m_sys->gran_params->static_friction_coeff_s2s = mu;
}

void ChSystemGpu::SetStaticFrictionCoeff_SPH2WALL(float mu) {
    m_sys->gran_params->static_friction_coeff_s2w = mu;
}

void ChSystemGpu::SetRollingCoeff_SPH2SPH(float mu) {
    m_sys->rolling_coeff_s2s_UU = mu;
}

void ChSystemGpu::SetRollingCoeff_SPH2WALL(float mu) {
    m_sys->rolling_coeff_s2w_UU = mu;
}

void ChSystemGpu::SetSpinningCoeff_SPH2SPH(float mu) {
    m_sys->spinning_coeff_s2s_UU = mu;
}

void ChSystemGpu::SetSpinningCoeff_SPH2WALL(float mu) {
    m_sys->spinning_coeff_s2w_UU = mu;
}

void ChSystemGpu::SetKn_SPH2SPH(double someValue) {
    m_sys->K_n_s2s_UU = someValue;
}

void ChSystemGpu::SetKn_SPH2WALL(double someValue) {
    m_sys->K_n_s2w_UU = someValue;
}

void ChSystemGpu::SetGn_SPH2SPH(double someValue) {
    m_sys->Gamma_n_s2s_UU = someValue;
}

void ChSystemGpu::SetGn_SPH2WALL(double someValue) {
    m_sys->Gamma_n_s2w_UU = someValue;
}

void ChSystemGpu::SetKt_SPH2SPH(double someValue) {
    m_sys->K_t_s2s_UU = someValue;
}

void ChSystemGpu::SetGt_SPH2SPH(double someValue) {
    m_sys->Gamma_t_s2s_UU = someValue;
}

void ChSystemGpu::SetKt_SPH2WALL(double someValue) {
    m_sys->K_t_s2w_UU = someValue;
}

void ChSystemGpu::SetGt_SPH2WALL(double someValue) {
    m_sys->Gamma_t_s2w_UU = someValue;
}

void ChSystemGpu::SetCohesionRatio(float someValue) {
    m_sys->cohesion_over_gravity = someValue;
}

void ChSystemGpu::SetAdhesionRatio_SPH2WALL(float someValue) {
    m_sys->adhesion_s2w_over_gravity = someValue;
}

// -----------------------------------------------------------------------------

void ChSystemGpuMesh::SetStaticFrictionCoeff_SPH2MESH(float mu) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->tri_params->static_friction_coeff_s2m = mu;
}

void ChSystemGpuMesh::SetRollingCoeff_SPH2MESH(float mu) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->rolling_coeff_s2m_UU = mu;
}

void ChSystemGpuMesh::SetSpinningCoeff_SPH2MESH(float mu) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->spinning_coeff_s2m_UU = mu;
}

void ChSystemGpuMesh::SetKn_SPH2MESH(double someValue) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->K_n_s2m_UU = someValue;
}

void ChSystemGpuMesh::SetGn_SPH2MESH(double someValue) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->Gamma_n_s2m_UU = someValue;
}

void ChSystemGpuMesh::SetKt_SPH2MESH(double someValue) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->K_t_s2m_UU = someValue;
}

void ChSystemGpuMesh::SetGt_SPH2MESH(double someValue) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->Gamma_t_s2m_UU = someValue;
}

void ChSystemGpuMesh::SetAdhesionRatio_SPH2MESH(float someValue) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->adhesion_s2m_over_gravity = someValue;
}

// -----------------------------------------------------------------------------

void ChSystemGpu::SetVerbosity(CHGPU_VERBOSITY level) {
    m_sys->verbosity = level;
}

void ChSystemGpuMesh::SetMeshVerbosity(CHGPU_MESH_VERBOSITY level) {
    mesh_verbosity = level;
}

// -----------------------------------------------------------------------------

size_t ChSystemGpu::CreateBCSphere(const ChVector<float>& center,
                                   float radius,
                                   bool outward_normal,
                                   bool track_forces) {
    float sph_center[3] = {center.x(), center.y(), center.z()};
    return m_sys->CreateBCSphere(sph_center, radius, outward_normal, track_forces);
}

size_t ChSystemGpu::CreateBCConeZ(const ChVector<float>& tip,
                                  float slope,
                                  float hmax,
                                  float hmin,
                                  bool outward_normal,
                                  bool track_forces) {
    float cone_tip[3] = {tip.x(), tip.y(), tip.z()};
    return m_sys->CreateBCConeZ(cone_tip, slope, hmax, hmin, outward_normal, track_forces);
}

size_t ChSystemGpu::CreateBCPlane(const ChVector<float>& pos, const ChVector<float>& normal, bool track_forces) {
    float plane_pos[3] = {pos.x(), pos.y(), pos.z()};
    float plane_nrm[3] = {normal.x(), normal.y(), normal.z()};
    return m_sys->CreateBCPlane(plane_pos, plane_nrm, track_forces);
}

size_t ChSystemGpu::CreateBCCylinderZ(const ChVector<float>& center,
                                      float radius,
                                      bool outward_normal,
                                      bool track_forces) {
    float cyl_center[3] = {center.x(), center.y(), center.z()};
    return m_sys->CreateBCCylinderZ(cyl_center, radius, outward_normal, track_forces);
}

bool ChSystemGpu::DisableBCbyID(size_t BC_id) {
    return m_sys->DisableBCbyID(BC_id);
}

bool ChSystemGpu::EnableBCbyID(size_t BC_id) {
    return m_sys->EnableBCbyID(BC_id);
}

bool ChSystemGpu::SetBCOffsetFunction(size_t BC_id, const GranPositionFunction& offset_function) {
    return m_sys->SetBCOffsetFunction(BC_id, offset_function);
}

void ChSystemGpu::setBDWallsMotionFunction(const GranPositionFunction& pos_fn) {
    m_sys->setBDWallsMotionFunction(pos_fn);
}

// -----------------------------------------------------------------------------

unsigned int ChSystemGpu::GetNumSDs() const {
    return m_sys->nSDs;
}

ChVector<float> ChSystemGpu::GetBCPlanePosition(size_t plane_id) const {
    // todo: throw an error if BC not a plane type
    float3 pos = m_sys->GetBCPlanePosition(plane_id);
    return ChVector<float>(pos.x, pos.y, pos.z);
}

bool ChSystemGpu::GetBCReactionForces(size_t BC_id, ChVector<float>& force) const {
    float3 frc;
    bool ret = m_sys->GetBCReactionForces(BC_id, frc);
    force = ChVector<float>(frc.x, frc.y, frc.z);
    return ret;
}

int ChSystemGpu::GetNumContacts() const {
    return m_sys->GetNumContacts();
}

float ChSystemGpu::GetSimTime() const {
    return m_sys->elapsedSimTime;
}

size_t ChSystemGpu::GetNumParticles() const {
    return m_sys->nSpheres;
}

float ChSystemGpu::GetParticleRadius() const {
    return m_sys->sphere_radius_UU;
}

ChVector<float> ChSystemGpu::GetParticlePosition(int nSphere) const {
    float3 pos = m_sys->GetParticlePosition(nSphere);
    return ChVector<float>(pos.x, pos.y, pos.z);
}

ChVector<float> ChSystemGpu::GetParticleVelocity(int nSphere) const {
    float3 vel = m_sys->GetParticleLinVelocity(nSphere);
    return ChVector<float>(vel.x, vel.y, vel.z);
}

ChVector<float> ChSystemGpu::GetParticleAngVelocity(int nSphere) const {
    if (m_sys->gran_params->friction_mode == CHGPU_FRICTION_MODE::FRICTIONLESS)
        return ChVector<float>(0);

    float3 omega = m_sys->GetParticleAngVelocity(nSphere);
    return ChVector<float>(omega.x, omega.y, omega.z);
}

double ChSystemGpu::GetMaxParticleZ() const {
    return m_sys->GetMaxParticleZ();
}

size_t ChSystemGpu::EstimateMemUsage() const {
    return m_sys->EstimateMemUsage();
}

// -----------------------------------------------------------------------------

unsigned int ChSystemGpuMesh::AddMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh, float mass) {
    unsigned int id = static_cast<unsigned int>(m_meshes.size());
    m_meshes.push_back(mesh);
    m_mesh_masses.push_back(mass);

    return id;
}

unsigned int ChSystemGpuMesh::AddMesh(const std::string& filename,
                                      const ChVector<float>& translation,
                                      const ChMatrix33<float>& rotscale,
                                      float mass) {
    auto mesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    bool flag = mesh->LoadWavefrontMesh(filename, true, false);
    if (!flag)
        CHGPU_ERROR("ERROR! Mesh %s failed to load in! Exiting!\n", filename.c_str());
    if (mesh->getNumTriangles() == 0)
        printf("WARNING: Mesh %s has no triangles!\n", filename.c_str());
    mesh->Transform(translation, rotscale.cast<double>());

    unsigned int id = static_cast<unsigned int>(m_meshes.size());
    m_meshes.push_back(mesh);
    m_mesh_masses.push_back(mass);

    return id;
}

std::vector<unsigned int> ChSystemGpuMesh::AddMeshes(const std::vector<std::string>& objfilenames,
                                                     const std::vector<ChVector<float>>& translations,
                                                     const std::vector<ChMatrix33<float>>& rotscales,
                                                     const std::vector<float>& masses) {
    unsigned int size = (unsigned int)objfilenames.size();
    if (size != rotscales.size() || size != translations.size() || size != masses.size())
        CHGPU_ERROR("Mesh loading vectors must all have same size\n");
    if (size == 0)
        printf("WARNING: No meshes provided!\n");

    std::vector<unsigned int> ids(size);
    for (unsigned int i = 0; i < size; i++) {
        ids[i] = AddMesh(objfilenames[i], translations[i], rotscales[i], masses[i]);
    }

    return ids;
}

void ChSystemGpuMesh::SetMeshes() {
    int nTriangles = 0;
    for (const auto& mesh : m_meshes)
        nTriangles += mesh->getNumTriangles();

    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    ChSystemGpuMesh_impl::TriangleSoup* pMeshSoup = sys_trimesh->getMeshSoup();
    pMeshSoup->nTrianglesInSoup = nTriangles;

    if (nTriangles != 0) {
        // Allocate all of the requisite pointers
        gpuErrchk(
            cudaMallocManaged(&pMeshSoup->triangleFamily_ID, nTriangles * sizeof(unsigned int), cudaMemAttachGlobal));

        gpuErrchk(cudaMallocManaged(&pMeshSoup->node1, nTriangles * sizeof(float3), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&pMeshSoup->node2, nTriangles * sizeof(float3), cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&pMeshSoup->node3, nTriangles * sizeof(float3), cudaMemAttachGlobal));
    }

    MESH_INFO_PRINTF("Done allocating nodes for %d triangles\n", nTriangles);

    // Setup the clean copy of the mesh soup from the obj file data
    unsigned int family = 0;
    unsigned int tri_i = 0;
    // for each obj file data set
    for (const auto& mesh : m_meshes) {
        for (int i = 0; i < mesh->getNumTriangles(); i++) {
            geometry::ChTriangle tri = mesh->getTriangle(i);

            pMeshSoup->node1[tri_i] = make_float3((float)tri.p1.x(), (float)tri.p1.y(), (float)tri.p1.z());
            pMeshSoup->node2[tri_i] = make_float3((float)tri.p2.x(), (float)tri.p2.y(), (float)tri.p2.z());
            pMeshSoup->node3[tri_i] = make_float3((float)tri.p3.x(), (float)tri.p3.y(), (float)tri.p3.z());

            pMeshSoup->triangleFamily_ID[tri_i] = family;

            // Normal of a single vertex... Should still work
            int normal_i = mesh->m_face_n_indices.at(i).x();  // normals at each vertex of this triangle
            ChVector<double> normal = mesh->m_normals[normal_i];

            // Generate normal using RHR from nodes 1, 2, and 3
            ChVector<double> AB = tri.p2 - tri.p1;
            ChVector<double> AC = tri.p3 - tri.p1;
            ChVector<double> cross;
            cross.Cross(AB, AC);

            // If the normal created by a RHR traversal is not correct, switch two vertices
            if (cross.Dot(normal) < 0) {
                std::swap(pMeshSoup->node2[tri_i], pMeshSoup->node3[tri_i]);
            }
            tri_i++;
        }
        family++;
        MESH_INFO_PRINTF("Done writing family %d\n", family);
    }

    pMeshSoup->numTriangleFamilies = family;

    if (pMeshSoup->nTrianglesInSoup != 0) {
        gpuErrchk(cudaMallocManaged(&pMeshSoup->familyMass_SU, family * sizeof(float), cudaMemAttachGlobal));

        for (unsigned int i = 0; i < family; i++) {
            // NOTE The SU conversion is done in initialize after the scaling is determined
            pMeshSoup->familyMass_SU[i] = m_mesh_masses[i];
        }

        gpuErrchk(cudaMallocManaged(&pMeshSoup->generalizedForcesPerFamily,
                                    6 * pMeshSoup->numTriangleFamilies * sizeof(float), cudaMemAttachGlobal));
        // Allocate memory for the float and double frames
        gpuErrchk(cudaMallocManaged(&sys_trimesh->getTriParams()->fam_frame_broad,
                                    pMeshSoup->numTriangleFamilies * sizeof(ChSystemGpuMesh_impl::MeshFrame<float>),
                                    cudaMemAttachGlobal));
        gpuErrchk(cudaMallocManaged(&sys_trimesh->getTriParams()->fam_frame_narrow,
                                    pMeshSoup->numTriangleFamilies * sizeof(ChSystemGpuMesh_impl::MeshFrame<double>),
                                    cudaMemAttachGlobal));

        // Allocate memory for linear and angular velocity
        gpuErrchk(
            cudaMallocManaged(&pMeshSoup->vel, pMeshSoup->numTriangleFamilies * sizeof(float3), cudaMemAttachGlobal));
        gpuErrchk(
            cudaMallocManaged(&pMeshSoup->omega, pMeshSoup->numTriangleFamilies * sizeof(float3), cudaMemAttachGlobal));

        for (unsigned int i = 0; i < family; i++) {
            pMeshSoup->vel[i] = make_float3(0, 0, 0);
            pMeshSoup->omega[i] = make_float3(0, 0, 0);
        }
    }
}

void ChSystemGpuMesh::ApplyMeshMotion(unsigned int mesh_id,
                                      const ChVector<>& pos,
                                      const ChQuaternion<>& rot,
                                      const ChVector<>& lin_vel,
                                      const ChVector<>& ang_vel) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->ApplyMeshMotion(mesh_id, pos.data(), rot.data(), lin_vel.data(), ang_vel.data());
}

// -----------------------------------------------------------------------------

unsigned int ChSystemGpuMesh::GetNumMeshes() const {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    return sys_trimesh->meshSoup->numTriangleFamilies;
}

void ChSystemGpuMesh::EnableMeshCollision(bool val) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->mesh_collision_enabled = val;
}

// -----------------------------------------------------------------------------

static void convertChVector2Float3Vec(const std::vector<ChVector<float>>& points, std::vector<float3>& pointsFloat3) {
    size_t nPoints = points.size();
    pointsFloat3.resize(nPoints);
    for (size_t index = 0; index < nPoints; index++) {
        pointsFloat3.at(index).x = points.at(index)[0];
        pointsFloat3.at(index).y = points.at(index)[1];
        pointsFloat3.at(index).z = points.at(index)[2];
    }
}

// initialize particle positions, velocity and angular velocity in user units
void ChSystemGpu::SetParticlePositions(const std::vector<ChVector<float>>& points,
                                       const std::vector<ChVector<float>>& vels,
                                       const std::vector<ChVector<float>>& ang_vel) {
    std::vector<float3> pointsFloat3;
    std::vector<float3> velsFloat3;
    std::vector<float3> angVelsFloat3;
    convertChVector2Float3Vec(points, pointsFloat3);
    convertChVector2Float3Vec(vels, velsFloat3);
    convertChVector2Float3Vec(ang_vel, angVelsFloat3);
    m_sys->SetParticlePositions(pointsFloat3, velsFloat3, angVelsFloat3);
}

// -----------------------------------------------------------------------------

void ChSystemGpuMesh::Initialize() {
    if (m_meshes.size() > 0)
        SetMeshes();
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->initializeSpheres();
    sys_trimesh->initializeTriangles();
}

void ChSystemGpuMesh::InitializeMeshes() {
    if (m_meshes.size() > 0)
        SetMeshes();
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->initializeTriangles();
}

void ChSystemGpu::Initialize() {
    m_sys->initializeSpheres();
    if (m_sys->verbosity == CHGPU_VERBOSITY::INFO || m_sys->verbosity == CHGPU_VERBOSITY::METRICS) {
        printf("Approx mem usage is %s\n", pretty_format_bytes(EstimateMemUsage()).c_str());
    }
}

// -----------------------------------------------------------------------------

double ChSystemGpuMesh::AdvanceSimulation(float duration) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    return sys_trimesh->AdvanceSimulation(duration);
}

double ChSystemGpu::AdvanceSimulation(float duration) {
    return m_sys->AdvanceSimulation(duration);
}

// -----------------------------------------------------------------------------

void ChSystemGpu::WriteFile(std::string ofile) const {
    m_sys->WriteFile(ofile);
}

void ChSystemGpu::WriteContactInfoFile(std::string ofile) const {
    m_sys->WriteContactInfoFile(ofile);
}

void ChSystemGpuMesh::WriteMeshes(std::string outfilename) const {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->WriteMeshes(outfilename);
}

// -----------------------------------------------------------------------------

void ChSystemGpuMesh::CollectMeshContactForces(std::vector<ChVector<>>& forces, std::vector<ChVector<>>& torques) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    unsigned int nmeshes = sys_trimesh->meshSoup->numTriangleFamilies;
    double force_factor = sys_trimesh->FORCE_SU2UU;
    double torque_factor = sys_trimesh->TORQUE_SU2UU;

    forces.resize(nmeshes);
    torques.resize(nmeshes);

    // Pull directly from unified memory
    for (unsigned int i = 0; i < nmeshes; i++) {
        double fx = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * i + 0];
        double fy = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * i + 1];
        double fz = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * i + 2];
        forces[i] = ChVector<>(fx, fy, fz) * force_factor;  // Divide by C_F to go from SU to UU

        double tx = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * i + 3];
        double ty = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * i + 4];
        double tz = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * i + 5];
        torques[i] = ChVector<>(tx, ty, tz) * torque_factor;  // Divide by C_TAU to go from SU to UU
    }
}

void ChSystemGpuMesh::CollectMeshContactForces(int mesh, ChVector<>& force, ChVector<>& torque) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    double force_factor = sys_trimesh->FORCE_SU2UU;
    double torque_factor = sys_trimesh->TORQUE_SU2UU;

    double fx = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * mesh + 0];
    double fy = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * mesh + 1];
    double fz = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * mesh + 2];
    force = ChVector<>(fx, fy, fz) * force_factor;  // Divide by C_F to go from SU to UU

    double tx = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * mesh + 3];
    double ty = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * mesh + 4];
    double tz = sys_trimesh->meshSoup->generalizedForcesPerFamily[6 * mesh + 5];
    torque = ChVector<>(tx, ty, tz) * torque_factor;  // Divide by C_TAU to go from SU to UU
}

// -----------------------------------------------------------------------------

}  // namespace gpu
}  // namespace chrono
