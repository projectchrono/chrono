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
// Authors: Nic Olsen, Ruochun Zhang, Dan Negrut, Radu Serban
// =============================================================================

#include <string>
#include <cmath>

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/physics/ChSystemGpuMesh_impl.h"

#include "chrono_gpu/utils/ChGpuUtilities.h"

namespace chrono {
namespace gpu {

// -----------------------------------------------------------------------------

ChSystemGpu::ChSystemGpu(float sphere_rad, float density, const ChVector<float>& boxDims, ChVector<float> O) {
    m_sys = new ChSystemGpu_impl(sphere_rad, density, make_float3(boxDims.x(), boxDims.y(), boxDims.z()),
                                 make_float3(O.x(), O.y(), O.z()));
}

ChSystemGpu::ChSystemGpu(const std::string& checkpoint) {
    m_sys = new ChSystemGpu_impl(1.f, 1.f, make_float3(100, 100, 100), make_float3(0, 0, 0));
    ReadCheckpointFile(checkpoint, true);
}

ChSystemGpuMesh::ChSystemGpuMesh(float sphere_rad, float density, const ChVector<float>& boxDims, ChVector<float> O)
    : mesh_verbosity(CHGPU_MESH_VERBOSITY::QUIET) {
    m_sys = new ChSystemGpuMesh_impl(sphere_rad, density, make_float3(boxDims.x(), boxDims.y(), boxDims.z()),
                                     make_float3(O.x(), O.y(), O.z()));
}

ChSystemGpuMesh::ChSystemGpuMesh(const std::string& checkpoint) : mesh_verbosity(CHGPU_MESH_VERBOSITY::QUIET) {
    m_sys = new ChSystemGpuMesh_impl(1.f, 1.f, make_float3(100, 100, 100), make_float3(0, 0, 0));
    ReadCheckpointFile(checkpoint, true);
}

ChSystemGpu::~ChSystemGpu() {
    delete m_sys;
}

ChSystemGpuMesh::~ChSystemGpuMesh() {}

// -----------------------------------------------------------------------------

void ChSystemGpu::SetGravitationalAcceleration(const ChVector<float>& g) {
    m_sys->X_accGrav = (float)g.x();
    m_sys->Y_accGrav = (float)g.y();
    m_sys->Z_accGrav = (float)g.z();
}

void ChSystemGpu::SetGravitationalAcceleration(const float3 g) {
    m_sys->X_accGrav = g.x;
    m_sys->Y_accGrav = g.y;
    m_sys->Z_accGrav = g.z;
}

void ChSystemGpu::SetBDFixed(bool fixed) {
    m_sys->BD_is_fixed = fixed;
}

void ChSystemGpu::SetBDCenter(const ChVector<float>& O) {
    m_sys->user_coord_O_X = O.x();
    m_sys->user_coord_O_Y = O.y();
    m_sys->user_coord_O_Z = O.z();
}

void ChSystemGpu::SetParticleDensity(float density) {
    m_sys->sphere_density_UU = density;
}

void ChSystemGpu::SetParticleRadius(float rad) {
    m_sys->sphere_radius_UU = rad;
}

void ChSystemGpu::SetParticleFixed(const std::vector<bool>& fixed) {
    m_sys->user_sphere_fixed = fixed;
}

// Set particle output file format
void ChSystemGpu::SetParticleOutputMode(CHGPU_OUTPUT_MODE mode) {
    m_sys->file_write_mode = mode;
}

// Set particle output file content
void ChSystemGpu::SetParticleOutputFlags(unsigned int flags) {
    m_sys->output_flags = flags;
}

void ChSystemGpu::SetFixedStepSize(float size_UU) {
    m_sys->stepSize_UU = size_UU;
}

void ChSystemGpu::EnableMinLength(bool useMinLen) {
    m_sys->use_min_length_unit = useMinLen;
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

void ChSystemGpu::SetDefragmentOnInitialize(bool defragment) {
    m_sys->defragment_on_start = defragment;
}

void ChSystemGpu::SetRecordingContactInfo(bool record) {
    m_sys->gran_params->recording_contactInfo = record;
}

void ChSystemGpu::SetMaxSafeVelocity_SU(float max_vel) {
    m_sys->gran_params->max_safe_vel = max_vel;
}

void ChSystemGpu::SetPsiFactors(unsigned int psi_T, unsigned int psi_L, float psi_R) {
    m_sys->psi_T = psi_T;
    m_sys->psi_L = psi_L;
    m_sys->psi_R = psi_R;
}

void ChSystemGpu::SetPsiT(unsigned int psi_T) {
    m_sys->psi_T = psi_T;
}

void ChSystemGpu::SetPsiL(unsigned int psi_L) {
    m_sys->psi_L = psi_L;
}

void ChSystemGpu::SetPsiR(float psi_R) {
    m_sys->psi_R = psi_R;
}

void ChSystemGpu::SetSimTime(float time) {
    m_sys->elapsedSimTime = time;
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

void ChSystemGpu::UseMaterialBasedModel(bool val) {
    m_sys->use_mat_based = val;
    m_sys->gran_params->use_mat_based = val;
}

void ChSystemGpu::SetYoungModulus_SPH(double someValue) {
    m_sys->YoungsModulus_sphere_UU = someValue;
}

void ChSystemGpu::SetYoungModulus_WALL(double someValue) {
    m_sys->YoungsModulus_wall_UU = someValue;
}

void ChSystemGpu::SetPoissonRatio_SPH(double someValue) {
    m_sys->PoissonRatio_sphere_UU = someValue;
}

void ChSystemGpu::SetPoissonRatio_WALL(double someValue) {
    m_sys->PoissonRatio_wall_UU = someValue;
}

void ChSystemGpu::SetRestitution_SPH(double someValue) {
    m_sys->COR_sphere_UU = someValue;
}

void ChSystemGpu::SetRestitution_WALL(double someValue) {
    m_sys->COR_wall_UU = someValue;
}

// -----------------------------------------------------------------------------

void ChSystemGpuMesh::UseMaterialBasedModel(bool val) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->tri_params->use_mat_based = val;
    sys_trimesh->gran_params->use_mat_based = val;
    sys_trimesh->use_mat_based = val;
}

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

void ChSystemGpuMesh::SetYoungModulus_MESH(double someValue) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->YoungsModulus_mesh_UU = someValue;
}

void ChSystemGpuMesh::SetPoissonRatio_MESH(double someValue) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->PoissonRatio_mesh_UU = someValue;
}

void ChSystemGpuMesh::SetRestitution_MESH(double someValue) {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    sys_trimesh->COR_mesh_UU = someValue;
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
                                   bool track_forces,
                                   float sphere_mass) {
    float sph_center[3] = {center.x(), center.y(), center.z()};
    return m_sys->CreateBCSphere(sph_center, radius, outward_normal, track_forces, sphere_mass);
}

void ChSystemGpu::SetBCSpherePosition(size_t sphere_bc_id, const ChVector<float>& pos) {
    m_sys->SetBCSpherePosition(sphere_bc_id, make_float3(pos.x(), pos.y(), pos.z()));
}

ChVector<float> ChSystemGpu::GetBCSpherePosition(size_t sphere_bc_id) const {
    float3 pos = m_sys->GetBCSpherePosition(sphere_bc_id);
    return ChVector<float>(pos.x, pos.y, pos.z);
}

void ChSystemGpu::SetBCSphereVelocity(size_t sphere_bc_id, const ChVector<float>& velo) {
    m_sys->SetBCSphereVelocity(sphere_bc_id, make_float3(velo.x(), velo.y(), velo.z()));
}

ChVector<float> ChSystemGpu::GetBCSphereVelocity(size_t sphere_bc_id) const {
    float3 velo = m_sys->GetBCSphereVelocity(sphere_bc_id);
    return ChVector<float>(velo.x, velo.y, velo.z);
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

// customized plate for angle of repose test, plate has a limited width in y direction, and can move in y direction
size_t ChSystemGpu::CreateCustomizedPlate(const ChVector<float>& pos_center,
                                          const ChVector<float>& normal,
                                          float hdim_y) {
    float plate_pos[3] = {pos_center.x(), pos_center.y(), pos_center.z()};
    float plate_nrm[3] = {normal.x(), normal.y(), normal.z()};

    return m_sys->CreateCustomizedPlate(plate_pos, plate_nrm, hdim_y);
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

void ChSystemGpu::SetBCPlaneRotation(size_t plane_id, ChVector<double> center, ChVector<double> omega) {
    double3 rotation_center = make_double3(center.x(), center.y(), center.z());
    double3 rotation_omega = make_double3(omega.x(), omega.y(), omega.z());
    m_sys->SetBCPlaneRotation(plane_id, rotation_center, rotation_omega);
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

void ChSystemGpu::SetParticlePosition(int nSphere, const ChVector<double> pos) {
    double3 position = make_double3(pos.x(), pos.y(), pos.z());
    m_sys->SetParticlePosition(nSphere, position);
}

ChVector<float> ChSystemGpu::GetParticleVelocity(int nSphere) const {
    float3 vel = m_sys->GetParticleLinVelocity(nSphere);
    return ChVector<float>(vel.x, vel.y, vel.z);
}

void ChSystemGpu::SetParticleVelocity(int nSphere, const ChVector<double> velo) {
    double3 velocity = make_double3(velo.x(), velo.y(), velo.z());
    m_sys->SetParticleVelocity(nSphere, velocity);
}

ChVector<float> ChSystemGpu::GetParticleAngVelocity(int nSphere) const {
    if (m_sys->gran_params->friction_mode == CHGPU_FRICTION_MODE::FRICTIONLESS)
        return ChVector<float>(0);

    float3 omega = m_sys->GetParticleAngVelocity(nSphere);
    return ChVector<float>(omega.x, omega.y, omega.z);
}

ChVector<float> ChSystemGpu::GetParticleLinAcc(int nSphere) const {
    float3 acc = m_sys->GetParticleLinAcc(nSphere);
    return ChVector<float>(acc.x, acc.y, acc.z);
}

bool ChSystemGpu::IsFixed(int nSphere) const {
    return m_sys->IsFixed(nSphere);
}

float ChSystemGpu::GetParticlesKineticEnergy() const {
    float KE = (float)(m_sys->ComputeTotalKE());
    return KE;
}

double ChSystemGpu::GetMaxParticleZ() const {
    return m_sys->GetMaxParticleZ(true);
}

double ChSystemGpu::GetMinParticleZ() const {
    // Under the hood, GetMaxParticleZ(false) returns the lowest Z.
    return m_sys->GetMaxParticleZ(false);
}

unsigned int ChSystemGpu::GetNumParticleAboveZ(float ZValue) const {
    return m_sys->GetNumParticleAboveZ(ZValue);
}

unsigned int ChSystemGpu::GetNumParticleAboveX(float XValue) const {
    return m_sys->GetNumParticleAboveX(XValue);
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
        CHGPU_ERROR("ERROR! Mesh %s failed to load in!\n", filename.c_str());
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
        CHGPU_ERROR("ERROR! Mesh loading vectors must all have same size!\n");
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

            // If we wish to correct surface orientation based on given vertex normals, rather than using RHR...
            if (use_mesh_normals) {
                int normal_i = mesh->m_face_n_indices.at(i).x();  // normals at each vertex of this triangle
                ChVector<double> normal = mesh->m_normals.at(normal_i);

                // Generate normal using RHR from nodes 1, 2, and 3
                ChVector<double> AB = tri.p2 - tri.p1;
                ChVector<double> AC = tri.p3 - tri.p1;
                ChVector<double> cross;
                cross.Cross(AB, AC);

                // If the normal created by a RHR traversal is not correct, switch two vertices
                if (cross.Dot(normal) < 0) {
                    std::swap(pMeshSoup->node2[tri_i], pMeshSoup->node3[tri_i]);
                }
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

// Initialize particle positions, velocity and angular velocity in user units
void ChSystemGpu::SetParticles(const std::vector<ChVector<float>>& points,
                               const std::vector<ChVector<float>>& vels,
                               const std::vector<ChVector<float>>& ang_vel) {
    std::vector<float3> pointsFloat3;
    std::vector<float3> velsFloat3;
    std::vector<float3> angVelsFloat3;
    convertChVector2Float3Vec(points, pointsFloat3);
    convertChVector2Float3Vec(vels, velsFloat3);
    convertChVector2Float3Vec(ang_vel, angVelsFloat3);
    m_sys->SetParticles(pointsFloat3, velsFloat3, angVelsFloat3);
}

// Use the CSV header to determine its content for data parsing format
inline unsigned int quarryCsvFormat(const std::string& line) {
    unsigned int formatMode = 0;
    // Use the header to determine the information being loaded in.
    // Note that the order they appear is guaranteed to be like below
    if (line.find("x,y,z") == std::string::npos)
        CHGPU_ERROR("ERROR! Checkpoint file (kinematics) does not contain xyz information!\n");
    if (line.find("vx,vy,vz") != std::string::npos)
        formatMode += VEL_COMPONENTS;
    if (line.find("absv") != std::string::npos)
        formatMode += ABSV;
    if (line.find("fixed") != std::string::npos)
        formatMode += FIXITY;
    if (line.find("wx,wy,wz") != std::string::npos)
        formatMode += ANG_VEL_COMPONENTS;
    return formatMode;
}

// User the history header to determine the friction/contact info we'll load
inline unsigned int quarryHistoryFormat(const std::string& line, unsigned int max_partner) {
    unsigned int formatMode = 0;
    std::istringstream iss1(line);
    if (line.find("partners") != std::string::npos) {
        formatMode += 1;
        iss1 >> max_partner;  // this one is a string... not the number we're after
        iss1 >> max_partner;
    } else
        printf(
            "WARNING! The friction history either has no header, or its header indicates there is no history to "
            "load "
            "in!\n");
    if (line.find("history") != std::string::npos)
        formatMode += 2;
    return formatMode;
}

// Read in particle positions, velocity and angular velocity through a csv-formatted ifstream
void ChSystemGpu::ReadCsvParticles(std::ifstream& ifile, unsigned int totRow) {
    // Read the header to know the input format
    std::string line;
    std::getline(ifile, line);
    while (line.find_first_not_of(' ') == std::string::npos)
        std::getline(ifile, line);
    unsigned int formatMode = quarryCsvFormat(line);

    // Parse in every data line
    std::string number;
    unsigned int numRow = 0;
    std::vector<float3> pointsFloat3, velsFloat3, angVelsFloat3;
    std::vector<bool> fixity;
    // don't always assume we know the num of rows, and we expand buffer every time batchSize is hit
    const unsigned int batchSize = 1e6;
    while (std::getline(ifile, line)) {
        // if empty line, just keep going
        if (line.find_first_not_of(' ') == std::string::npos)
            continue;

        std::istringstream iss1(line);
        if (numRow % batchSize == 0) {
            pointsFloat3.resize(pointsFloat3.size() + batchSize);
            velsFloat3.resize(velsFloat3.size() + batchSize, make_float3(0, 0, 0));
            fixity.resize(fixity.size() + batchSize, false);
            angVelsFloat3.resize(angVelsFloat3.size() + batchSize, make_float3(0, 0, 0));
        }
        // Read xyz
        getline(iss1, number, ',');
        pointsFloat3.at(numRow).x = std::stof(number);
        getline(iss1, number, ',');
        pointsFloat3.at(numRow).y = std::stof(number);
        getline(iss1, number, ',');
        pointsFloat3.at(numRow).z = std::stof(number);
        // Read velocity
        if (formatMode & VEL_COMPONENTS) {
            getline(iss1, number, ',');
            velsFloat3.at(numRow).x = std::stof(number);
            getline(iss1, number, ',');
            velsFloat3.at(numRow).y = std::stof(number);
            getline(iss1, number, ',');
            velsFloat3.at(numRow).z = std::stof(number);
        }
        // Read absv, but we don't need it
        if (formatMode & ABSV)
            getline(iss1, number, ',');
        // Read fixity
        if (formatMode & FIXITY) {
            getline(iss1, number, ',');
            fixity.at(numRow) = (bool)std::stoi(number);
        }
        // Read angular velocity
        if (formatMode & ANG_VEL_COMPONENTS) {
            getline(iss1, number, ',');
            angVelsFloat3.at(numRow).x = std::stof(number);
            getline(iss1, number, ',');
            angVelsFloat3.at(numRow).y = std::stof(number);
            getline(iss1, number, ',');
            angVelsFloat3.at(numRow).z = std::stof(number);
        }
        numRow++;
        // If all spheres loaded, break. Don't keep going as it's not necessarily the EoF.
        if (numRow >= totRow)
            break;
    }

    // Final resize
    pointsFloat3.resize(numRow);
    velsFloat3.resize(numRow);
    angVelsFloat3.resize(numRow);
    fixity.resize(numRow);

    // Feed data to the system
    // We are directly using low-level m_sys functions here, since we're already working with float3
    // But we can choose to call user panel SetParticles and SetParticleFixed as well
    m_sys->SetParticles(pointsFloat3, velsFloat3, angVelsFloat3);
    m_sys->user_sphere_fixed = fixity;

    // Last thing: if this function runs successfully, then we automatically disable the defragment process on
    // simulation initialization. A re-started simulation would better not have its particle order re-arranged. But
    // if the user does not like it, they can still force the defragment by calling a method.
    if (numRow > 0)
        m_sys->defragment_on_start = false;
}

// Read in particle friction contact history through a hst-formatted ifstream
void ChSystemGpu::ReadHstHistory(std::ifstream& ifile, unsigned int totItem) {
    // Find the header line
    std::string line;
    std::getline(ifile, line);
    while (line.find_first_not_of(' ') == std::string::npos)
        std::getline(ifile, line);

    // The header will tell the max number of partners (use as offset),
    // but MAX_SPHERES_TOUCHED_BY_SPHERE is the monodispersity standard
    unsigned int max_partner = MAX_SPHERES_TOUCHED_BY_SPHERE;
    unsigned int formatMode = quarryHistoryFormat(line, max_partner);

    // Parse in every data line
    float3 history_UU;
    unsigned int numItem = 0;
    std::vector<unsigned int> partner;
    std::vector<float3> history;
    // don't always assume we know the num of items (spheres), and we expand buffer every time batchSize is hit
    const unsigned int batchSize = 1e6;
    while (std::getline(ifile, line)) {
        // if empty line, just keep going
        if (line.find_first_not_of(' ') == std::string::npos)
            continue;

        std::istringstream iss1(line);
        if (numItem % batchSize == 0) {
            partner.resize(partner.size() + max_partner * batchSize);
            history.resize(history.size() + max_partner * batchSize, make_float3(0, 0, 0));
        }

        // Read partner map
        if (formatMode & 1) {
            for (unsigned int i = 0; i < max_partner; i++)
                iss1 >> partner[max_partner * numItem + i];
        }

        // Read contact history
        if (formatMode & 2) {
            for (unsigned int i = 0; i < max_partner; i++) {
                iss1 >> history_UU.x;
                iss1 >> history_UU.y;
                iss1 >> history_UU.z;
                history[max_partner * numItem + i] = history_UU;
            }
        }

        numItem++;
        // If all spheres loaded, break. Don't keep going as it's not necessarily the EoF.
        if (numItem >= totItem)
            break;
    }

    // Final resize
    partner.resize(max_partner * numItem);
    history.resize(max_partner * numItem);

    // Feed the contact/history info to the system. We are directly referencing low-level m_sys structs here.
    // Because there are no user panel funtions (that loads contact/history) which can be used in a simulation
    // script. Those utils can be added, but I doubt their usefulness.
    m_sys->user_partner_map = partner;
    m_sys->user_friction_history = history;

    // Last thing: if this function runs successfully, then we automatically disable the defragment process on
    // simulation initialization. A re-started simulation would better not have its particle order re-arranged. But
    // if the user does not like it, they can still force the defragment by calling a method.
    if (numItem > 0)
        m_sys->defragment_on_start = false;
}

// Read in particle friction contact history through a file.
// Now it works with a custom format only. Potentially more formats can be added in the future.
void ChSystemGpu::ReadContactHistoryFile(const std::string& infilename) {
    std::ifstream ifile(infilename.c_str());
    if (!ifile.is_open()) {
        CHGPU_ERROR("ERROR! Checkpoint file (contact history) did not open successfully!\n");
    }

    ReadHstHistory(ifile, UINT_MAX);
}

// Read in particle positions, velocity and angular velocity through a file.
// Now it works with csv format only. Potentially more formats can be added in the future.
void ChSystemGpu::ReadParticleFile(const std::string& infilename) {
    std::ifstream ifile(infilename.c_str());
    if (!ifile.is_open()) {
        CHGPU_ERROR("ERROR! Checkpoint file (kinematics) did not open successfully!\n");
    }

    ReadCsvParticles(ifile, UINT_MAX);
}

// A smaller hasher that helps determine the indentifier type.
// It is powerful enough for our purpose and good-looking. We did not use built-in functions (such as string_view?)
// because that could require C++17, also I feel it would make the code look longer. In any case, if this small
// hasher is not sufficient anymore in future updates, we can spot that during compilation.
constexpr unsigned int hash_charr(const char* s, int off = 0) {
    return !s[off] ? 7001 : (hash_charr(s, off + 1) * 33) ^ s[off];
}
constexpr inline unsigned int operator"" _(const char* s, size_t) {
    return hash_charr(s);
}
bool diff(float a, float b) {
    return std::abs(a - b) > 1e-6f;
}
bool diff(float3 a, float3 b) {
    return std::abs(a.x - b.x) > 1e-6f || std::abs(a.y - b.y) > 1e-6f || std::abs(a.z - b.z) > 1e-6f;
}

// Use hash to find matching indentifier and load parameters. Return 1 if found no matching paramter to set, return
// 0 if status normal
bool ChSystemGpu::SetParamsFromIdentifier(const std::string& identifier, std::istringstream& iss1, bool overwrite) {
    unsigned int i;        // integer holder
    float f;               // float holder
    float3 f3;             // float3 holder
    double d;              // double holder
    bool b;                // bool holder
    bool anomaly = false;  // flag unknown identifier
    bool incst = false;    // flag parameter changes compared to current system

    switch (hash_charr(identifier.c_str())) {
        case ("density"_):
            iss1 >> f;
            incst = diff(m_sys->sphere_density_UU, f);
            m_sys->sphere_density_UU = f;
            break;
        case ("radius"_):
            iss1 >> f;
            incst = diff(m_sys->sphere_radius_UU, f);
            m_sys->sphere_radius_UU = f;
            break;
        case ("boxSize"_):
            iss1 >> f3.x;
            iss1 >> f3.y;
            iss1 >> f3.z;
            incst = diff(make_float3(m_sys->box_size_X, m_sys->box_size_Y, m_sys->box_size_Z), f3);
            m_sys->box_size_X = f3.x;
            m_sys->box_size_Y = f3.y;
            m_sys->box_size_Z = f3.z;
            break;
        case ("BDFixed"_):
            iss1 >> b;
            SetBDFixed(b);
            break;
        case ("BDCenter"_):
            iss1 >> f3.x;
            iss1 >> f3.y;
            iss1 >> f3.z;
            incst = diff(make_float3(m_sys->user_coord_O_X, m_sys->user_coord_O_Y, m_sys->user_coord_O_Z), f3);
            m_sys->user_coord_O_X = f3.x;
            m_sys->user_coord_O_Y = f3.y;
            m_sys->user_coord_O_Z = f3.z;
            break;
        case ("verbosity"_):
            iss1 >> i;
            SetVerbosity(static_cast<CHGPU_VERBOSITY>(i));
            break;
        case ("useMinLengthUnit"_):
            iss1 >> b;
            EnableMinLength(b);
            break;
        case ("recordContactInfo"_):
            iss1 >> b;
            SetRecordingContactInfo(b);
            break;
        case ("particleFileMode"_):
            iss1 >> i;
            SetParticleOutputMode(static_cast<CHGPU_OUTPUT_MODE>(i));
            break;
        case ("particleFileFlags"_):
            iss1 >> i;
            // ParticleOutputFlags is already int, instead of a enum class
            SetParticleOutputFlags(i);
            break;
        case ("fixedStepSize"_):
            iss1 >> f;
            SetFixedStepSize(f);
            break;
        case ("cohesionOverG"_):
            iss1 >> f;
            SetCohesionRatio(f);
            break;
        case ("adhesionOverG_s2w"_):
            iss1 >> f;
            SetAdhesionRatio_SPH2WALL(f);
            break;
        case ("G"_):
            iss1 >> f3.x;
            iss1 >> f3.y;
            iss1 >> f3.z;
            SetGravitationalAcceleration(f3);
            break;
        case ("elapsedTime"_):
            iss1 >> f;
            SetSimTime(f);
            break;
        case ("K_n_s2s"_):
            iss1 >> d;
            SetKn_SPH2SPH(d);
            break;
        case ("K_n_s2w"_):
            iss1 >> d;
            SetKn_SPH2WALL(d);
            break;
        case ("K_t_s2s"_):
            iss1 >> d;
            SetKt_SPH2SPH(d);
            break;
        case ("K_t_s2w"_):
            iss1 >> d;
            SetKt_SPH2WALL(d);
            break;
        case ("G_n_s2s"_):
            iss1 >> d;
            SetGn_SPH2SPH(d);
            break;
        case ("G_n_s2w"_):
            iss1 >> d;
            SetGn_SPH2WALL(d);
            break;
        case ("G_t_s2s"_):
            iss1 >> d;
            SetGt_SPH2SPH(d);
            break;
        case ("G_t_s2w"_):
            iss1 >> d;
            SetGt_SPH2WALL(d);
            break;
        case ("RollingCoeff_s2s"_):
            iss1 >> d;
            SetRollingCoeff_SPH2SPH(d);
            break;
        case ("RollingCoeff_s2w"_):
            iss1 >> d;
            SetRollingCoeff_SPH2WALL(d);
            break;
        case ("SpinningCoeff_s2s"_):
            iss1 >> d;
            SetSpinningCoeff_SPH2SPH(d);
            break;
        case ("SpinningCoeff_s2w"_):
            iss1 >> d;
            SetSpinningCoeff_SPH2WALL(d);
            break;
        case ("StaticFrictionCoeff_s2s"_):
            iss1 >> d;
            SetStaticFrictionCoeff_SPH2SPH(d);
            break;
        case ("StaticFrictionCoeff_s2w"_):
            iss1 >> d;
            SetStaticFrictionCoeff_SPH2WALL(d);
            break;
        case ("PsiT"_):
            iss1 >> i;
            SetPsiT(i);
            break;
        case ("PsiL"_):
            iss1 >> i;
            SetPsiL(i);
            break;
        case ("PsiR"_):
            iss1 >> f;
            SetPsiR(f);
            break;
        case ("frictionMode"_):
            iss1 >> i;
            SetFrictionMode(static_cast<CHGPU_FRICTION_MODE>(i));
            break;
        case ("rollingMode"_):
            iss1 >> i;
            SetRollingMode(static_cast<CHGPU_ROLLING_MODE>(i));
            break;
        case ("timeIntegrator"_):
            iss1 >> i;
            SetTimeIntegrator(static_cast<CHGPU_TIME_INTEGRATOR>(i));
            break;
        case ("maxSafeVelSU"_):
            iss1 >> f;
            SetMaxSafeVelocity_SU(f);
            break;
        default:
            anomaly = true;
    }

    if (incst && !overwrite)
        CHGPU_ERROR(
            "ERROR! Parameter \"%s\" is inconsistent with the current simulation system.\n"
            "If you wish to construct a simulation systen from scratch using this checkpoint file, then you "
            "should supply this file as the constructor parameter.\nExiting...\n",
            identifier.c_str());

    return anomaly;
}

// Read in simulation parameters. Returns the total number of particles. If instructed to overwrite, then overwrite
// cuurent simulation parameters with the values in the checkpoint file; else, when an inconsistency is found, throw
// an error.
unsigned int ChSystemGpu::ReadDatParams(std::ifstream& ifile, bool overwrite) {
    std::string line;
    unsigned int nSpheres = 0;
    std::getline(ifile, line);
    // Get rid of possible empty lines, before the real party starts
    while (line.find_first_not_of(' ') == std::string::npos)
        std::getline(ifile, line);

    std::string identifier;
    while (line != "ParamsEnd") {
        std::istringstream iss1(line);
        // Grab the param identifier
        getline(iss1, identifier, ':');
        // Call corresponding "Set" methods based on identifier
        if (identifier == "nSpheres") {
            iss1 >> nSpheres;
            if (!overwrite && (nSpheres - m_sys->nSpheres != 0))
                CHGPU_ERROR(
                    "ERROR! Number of particles in checkpoint file is inconsistent with the current system.\n"
                    "If you wish to construct a simulation systen from scratch using this checkpoint file, then "
                    "you "
                    "should supply this file as the constructor parameter.\nExiting...\n");
        } else {
            bool anomaly = SetParamsFromIdentifier(identifier, iss1, overwrite);
            if (anomaly)
                printf("WARNING! %s is an unknown parameter, skipped.\n", identifier.c_str());
        }

        // Load next line
        std::getline(ifile, line);
    }

    if (nSpheres == 0)
        printf(
            "WARNING! The checkpoint file indicates there are 0 particles to be loaded. If this is intended, the "
            "user "
            "must initialize the particles by themselves.\n");
    return nSpheres;
}

// Read in a (Chrono::Gpu generated) checkpoint file to restart a simulation. It calls ReadHstHistory,
// ReadCsvParticles and ReadDatParams to parse in the entire checkpoint file.
void ChSystemGpu::ReadCheckpointFile(const std::string& infilename, bool overwrite) {
    // Open the file
    std::ifstream ifile(infilename.c_str());
    if (!ifile.is_open()) {
        CHGPU_ERROR("ERROR! Checkpoint file did not open successfully!\n");
    }

    // Let the user know we are loading...
    printf("Reading checkpoint data from file \"%s\"...\n", infilename.c_str());

    // Process the header
    std::string line;
    std::getline(ifile, line);
    while (line.find_first_not_of(' ') == std::string::npos)
        std::getline(ifile, line);
    if (line != "ChSystemGpu")
        printf(
            "WARNING! Header of checkpoint file indicates this is not for ChSystemGpu (and it seems to be for %s)! "
            "There may still be parameters defaulted after loading this checkpoint file, and you may want to set "
            "them "
            "manually.\n",
            line.c_str());

    // Load and set all simulation parameters. Keep tab of nSpheres, we'll need that soon.
    unsigned int nSpheres;
    nSpheres = ReadDatParams(ifile, overwrite);

    // Now load the rest, particle kinematics info, contact pair/friction history, everything
    while (std::getline(ifile, line)) {
        if (line.find_first_not_of(' ') == std::string::npos)
            continue;

        // If not an empty line, then check the header to call a subroutine
        if (line == "CsvParticles")
            ReadCsvParticles(ifile, nSpheres);
        else if (line == "HstHistory") {
            if (m_sys->gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
                ReadHstHistory(ifile, nSpheres);
            }
        }
    }
}

// -----------------------------------------------------------------------------

// GpuMesh veriosn of reading checkpointed params using hashed identifier.
bool ChSystemGpuMesh::SetParamsFromIdentifier(const std::string& identifier, std::istringstream& iss1, bool overwrite) {
    float f;               // float holder
    double d;              // double holder
    bool b;                // bool holder
    bool anomaly = false;  // flag unknown identifier

    // Is the parameter one of the ChSystemGpu's? If not, we then search it in ChSystemGpuMesh.
    // The parent method returns true if it did not find a match.
    if (ChSystemGpu::SetParamsFromIdentifier(identifier, iss1, overwrite)) {
        switch (hash_charr(identifier.c_str())) {
            case ("K_n_s2m"_):
                iss1 >> d;
                SetKn_SPH2MESH(d);
                break;
            case ("K_t_s2m"_):
                iss1 >> d;
                SetKt_SPH2MESH(d);
                break;
            case ("G_n_s2m"_):
                iss1 >> d;
                SetGn_SPH2MESH(d);
                break;
            case ("G_t_s2m"_):
                iss1 >> d;
                SetGt_SPH2MESH(d);
                break;
            case ("RollingCoeff_s2m"_):
                iss1 >> f;
                SetRollingCoeff_SPH2MESH(f);
                break;
            case ("SpinningCoeff_s2m"_):
                iss1 >> f;
                SetSpinningCoeff_SPH2MESH(f);
                break;
            case ("StaticFrictionCoeff_s2m"_):
                iss1 >> f;
                SetStaticFrictionCoeff_SPH2MESH(f);
                break;
            case ("MeshCollisionEnabled"_):
                iss1 >> b;
                EnableMeshCollision(b);
                break;
            case ("adhesionOverG_s2m"_):
                iss1 >> f;
                SetAdhesionRatio_SPH2MESH(f);
                break;
            default:
                anomaly = true;
                // printf("WARNING! %s is an unknown parameter, skipped.\n", identifier.c_str());
        }
    }
    return anomaly;
}

// GpuMesh version of checkpointing loading subroutine.
void ChSystemGpuMesh::ReadCheckpointFile(const std::string& infilename, bool overwrite) {
    // Open the file
    std::ifstream ifile(infilename.c_str());
    if (!ifile.is_open()) {
        CHGPU_ERROR("ERROR! Checkpoint file did not open successfully!\n");
    }

    // Let the user know we are loading...
    printf("Reading checkpoint data from file \"%s\"...\n", infilename.c_str());

    // Process the header
    std::string line;
    std::getline(ifile, line);
    while (line.find_first_not_of(' ') == std::string::npos)
        std::getline(ifile, line);
    if (line != "ChSystemGpuMesh")
        printf(
            "WARNING! Header of checkpoint file indicates this is not for ChSystemGpuMesh (and it seems to be for "
            "%s)! "
            "There may still be parameters defaulted after loading this checkpoint file, and you may want to set "
            "them "
            "manually.\n",
            line.c_str());

    // Load and set all simulation parameters. Keep tab of nSpheres, we'll need that soon.
    unsigned int nSpheres;
    nSpheres = ReadDatParams(ifile, overwrite);

    // Now load the rest, particle kinematics info, contact pair/friction history, everything
    while (std::getline(ifile, line)) {
        if (line.find_first_not_of(' ') == std::string::npos)
            continue;

        // If not an empty line, then check the header to call a subroutine
        if (line == "CsvParticles")
            ReadCsvParticles(ifile, nSpheres);
        else if (line == "HstHistory") {
            if (m_sys->gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
                ReadHstHistory(ifile, nSpheres);
            }
        }
        // else, TODO it may need to read mesh-related info. This may be coming in the future.
    }
}

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

template <typename Enumeration>
auto as_uint(Enumeration const value) -> typename std::underlying_type<Enumeration>::type {
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

void ChSystemGpu::WriteCheckpointParams(std::ofstream& cpFile) const {
    std::ostringstream paramStream;

    paramStream << "nSpheres: " << GetNumParticles() << "\n";
    paramStream << "density: " << m_sys->sphere_density_UU << "\n";
    paramStream << "radius: " << m_sys->sphere_radius_UU << "\n";
    paramStream << "boxSize: " << m_sys->box_size_X << " " << m_sys->box_size_Y << " " << m_sys->box_size_Z << "\n";
    paramStream << "BDFixed: " << (int)(m_sys->BD_is_fixed) << "\n";
    paramStream << "BDCenter: " << m_sys->user_coord_O_X << " " << m_sys->user_coord_O_Y << " " << m_sys->user_coord_O_Z
                << "\n";
    paramStream << "verbosity: " << as_uint(m_sys->verbosity) << "\n";
    paramStream << "useMinLengthUnit: " << (int)(m_sys->use_min_length_unit) << "\n";
    paramStream << "recordContactInfo: " << (int)(m_sys->gran_params->recording_contactInfo) << "\n";
    paramStream << "particleFileMode: " << as_uint(m_sys->file_write_mode) << "\n";
    // returned OutputFlags is already an int
    paramStream << "particleFileFlags: " << m_sys->output_flags << "\n";
    paramStream << "fixedStepSize: " << m_sys->stepSize_UU << "\n";
    // It is cohesion-over-gravity and adhesion-over-gravity
    paramStream << "cohesionOverG: " << m_sys->cohesion_over_gravity << "\n";
    paramStream << "adhesionOverG_s2w: " << m_sys->adhesion_s2w_over_gravity << "\n";
    paramStream << "G: " << m_sys->X_accGrav << " " << m_sys->Y_accGrav << " " << m_sys->Z_accGrav << "\n";
    paramStream << "elapsedTime: " << GetSimTime() << "\n";

    paramStream << "K_n_s2s: " << m_sys->K_n_s2s_UU << "\n";
    paramStream << "K_n_s2w: " << m_sys->K_n_s2w_UU << "\n";
    paramStream << "K_t_s2s: " << m_sys->K_t_s2s_UU << "\n";
    paramStream << "K_t_s2w: " << m_sys->K_t_s2w_UU << "\n";
    paramStream << "G_n_s2s: " << m_sys->Gamma_n_s2s_UU << "\n";
    paramStream << "G_n_s2w: " << m_sys->Gamma_n_s2w_UU << "\n";
    paramStream << "G_t_s2s: " << m_sys->Gamma_t_s2s_UU << "\n";
    paramStream << "G_t_s2w: " << m_sys->Gamma_t_s2w_UU << "\n";
    paramStream << "RollingCoeff_s2s: " << m_sys->rolling_coeff_s2s_UU << "\n";
    paramStream << "RollingCoeff_s2w: " << m_sys->rolling_coeff_s2w_UU << "\n";
    paramStream << "SpinningCoeff_s2s: " << m_sys->spinning_coeff_s2s_UU << "\n";
    paramStream << "SpinningCoeff_s2w: " << m_sys->spinning_coeff_s2w_UU << "\n";
    paramStream << "StaticFrictionCoeff_s2s: " << m_sys->gran_params->static_friction_coeff_s2s << "\n";
    paramStream << "StaticFrictionCoeff_s2w: " << m_sys->gran_params->static_friction_coeff_s2w << "\n";

    paramStream << "PsiT: " << m_sys->psi_T << "\n";
    paramStream << "PsiL: " << m_sys->psi_L << "\n";
    paramStream << "PsiR: " << m_sys->psi_R << "\n";
    paramStream << "frictionMode: " << as_uint(m_sys->gran_params->friction_mode) << "\n";
    paramStream << "rollingMode: " << as_uint(m_sys->gran_params->rolling_mode) << "\n";

    // Notify the user that the support for CHUNG is limited. It still runs but loses some information in the
    // checkpointing process.
    paramStream << "timeIntegrator: " << as_uint(m_sys->time_integrator) << "\n";
    if (m_sys->time_integrator == CHGPU_TIME_INTEGRATOR::CHUNG)
        printf(
            "WARNING! CHUNG integrator is partially supported in this version. The acceleration and angular "
            "acceleration info is not stored in the checkpoint file, leading to a potential change in physics. You "
            "can "
            "consider using other integrators if checkpointing is needed, or wait for a fix in the next "
            "version.\n");

    paramStream << "maxSafeVelSU: " << m_sys->gran_params->max_safe_vel << "\n";

    // In the near future, TODO cache all extra boundaries here as well

    cpFile << paramStream.str();
}

void ChSystemGpu::WriteCheckpointFile(const std::string& outfilename) {
    printf("Writing checkpoint data to file \"%s\"\n", outfilename.c_str());
    std::ofstream cpFile(outfilename, std::ios::out);

    // Header, indicating the system (so meshed system will have something different)
    cpFile << std::string("ChSystemGpu\n");

    // Simulation params go right after the system name
    WriteCheckpointParams(cpFile);
    cpFile << std::string("ParamsEnd\n");
    cpFile << std::string("\n");

    // Then, the particle kinematics info
    cpFile << std::string("CsvParticles\n");
    // In checkpointing, we want all particle info written, instead of selected output that the user can enforce via
    // setting OutputFlags. Therefore, we temporarily set OutputFlags to maximum, then revert after this writting is
    // done.
    unsigned int outFlags = m_sys->output_flags;
    SetParticleOutputFlags(VEL_COMPONENTS | FIXITY | ANG_VEL_COMPONENTS);
    WriteCsvParticles(cpFile);
    SetParticleOutputFlags(outFlags);
    cpFile << std::string("\n");

    // Then, write contact pair/friction history
    if (m_sys->gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
        cpFile << std::string("HstHistory\n");
        WriteHstHistory(cpFile);
        cpFile << std::string("\n");
    }
}

void ChSystemGpu::WriteRawParticles(std::ofstream& ptFile) const {
    m_sys->WriteRawParticles(ptFile);
}

void ChSystemGpu::WriteCsvParticles(std::ofstream& ptFile) const {
    m_sys->WriteCsvParticles(ptFile);
}

void ChSystemGpu::WriteChPFParticles(std::ofstream& ptFile) const {
    m_sys->WriteChPFParticles(ptFile);
}

#ifdef USE_HDF5
void ChSystemGpu::WriteH5Particles(H5::H5File ptFile) const {
    m_sys->WriteH5Particles(ptFile);
}
#endif

void ChSystemGpu::WriteParticleFile(const std::string& outfilename) const {
    // The file writes are a pretty big slowdown in CSV mode
    if (m_sys->file_write_mode == CHGPU_OUTPUT_MODE::BINARY) {
        std::ofstream ptFile(outfilename, std::ios::out | std::ios::binary);
        WriteRawParticles(ptFile);
    } else if (m_sys->file_write_mode == CHGPU_OUTPUT_MODE::CSV) {
        std::ofstream ptFile(outfilename, std::ios::out);
        WriteCsvParticles(ptFile);
    } else if (m_sys->file_write_mode == CHGPU_OUTPUT_MODE::CHPF) {
        std::ofstream ptFile(outfilename, std::ios::out | std::ios::binary);
        WriteChPFParticles(ptFile);
    } else if (m_sys->file_write_mode == CHGPU_OUTPUT_MODE::HDF5) {
#ifdef USE_HDF5
        H5::H5File ptFile(outfilename.c_str(), H5F_ACC_TRUNC);
        WriteH5Particles(ptFile);
#else
        CHGPU_ERROR("ERROR! HDF5 Installation not found. Recompile with HDF5.\n");
#endif
    }
}

void ChSystemGpu::WriteContactInfoFile(const std::string& outfilename) const {
    m_sys->WriteContactInfoFile(outfilename);
}

void ChSystemGpu::WriteHstHistory(std::ofstream& histFile) const {
    // Dump to a stream, write to file only at end
    std::ostringstream outstrstream;

    // Figure out the write format first
    // 1: write contact_partners_map
    // 2: write contact_history_map
    unsigned int formatMode = 0;
    switch (m_sys->gran_params->friction_mode) {
        case (CHGPU_FRICTION_MODE::FRICTIONLESS):
            printf("WARNING! Currently using FRICTIONLESS model. There is no contact history to write!\n");
            return;
        case (CHGPU_FRICTION_MODE::SINGLE_STEP):
            formatMode += 1;
            break;
        case (CHGPU_FRICTION_MODE::MULTI_STEP):
            formatMode += (1 + 2);
            break;
        default:
            CHGPU_ERROR("ERROR! Unknown friction model, failed to write contact history file!\n");
    }

    if (formatMode & 1)
        outstrstream << "partners " << MAX_SPHERES_TOUCHED_BY_SPHERE;
    if (formatMode & 2)
        outstrstream << " history " << MAX_SPHERES_TOUCHED_BY_SPHERE;
    outstrstream << "\n";

    // We'll use space-separated formatting, as I found it more convenient to parse in and looks better.
    // Forget about CSV conventions, history info is not meant to be used by third-party tools anyway.
    float3 history_UU;
    for (unsigned int n = 0; n < m_sys->nSpheres; n++) {
        // Write contact_partners_map
        if (formatMode & 1) {
            for (unsigned int i = 0; i < MAX_SPHERES_TOUCHED_BY_SPHERE; i++)
                outstrstream << m_sys->contact_partners_map[MAX_SPHERES_TOUCHED_BY_SPHERE * n + i] << " ";
        }
        // Write write contact_history_map
        if (formatMode & 2) {
            for (unsigned int i = 0; i < MAX_SPHERES_TOUCHED_BY_SPHERE; i++) {
                history_UU.x =
                    m_sys->contact_history_map[MAX_SPHERES_TOUCHED_BY_SPHERE * n + i].x * m_sys->LENGTH_SU2UU;
                history_UU.y =
                    m_sys->contact_history_map[MAX_SPHERES_TOUCHED_BY_SPHERE * n + i].y * m_sys->LENGTH_SU2UU;
                history_UU.z =
                    m_sys->contact_history_map[MAX_SPHERES_TOUCHED_BY_SPHERE * n + i].z * m_sys->LENGTH_SU2UU;
                outstrstream << history_UU.x << " " << history_UU.y << " " << history_UU.z << " ";
            }
        }
        outstrstream << "\n";
    }

    histFile << outstrstream.str();
}

void ChSystemGpu::WriteContactHistoryFile(const std::string& outfilename) const {
    printf("Writing contact pair/history data to file \"%s\"\n", outfilename.c_str());
    std::ofstream histFile(outfilename, std::ios::out);

    // Call a subroutine to write. This subroutine can be used in checkpointing as well.
    WriteHstHistory(histFile);
}

// -----------------------------------------------------------------------------

// GpuMesh version of checkpoint params writing. Including extra params that only a meshed system has.
void ChSystemGpuMesh::WriteCheckpointMeshParams(std::ofstream& cpFile) const {
    std::ostringstream paramStream;
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);

    paramStream << "adhesionOverG_s2m: " << sys_trimesh->adhesion_s2m_over_gravity << "\n";
    paramStream << "K_n_s2m: " << sys_trimesh->K_n_s2m_UU << "\n";
    paramStream << "K_t_s2m: " << sys_trimesh->K_t_s2m_UU << "\n";
    paramStream << "G_n_s2m: " << sys_trimesh->Gamma_n_s2m_UU << "\n";
    paramStream << "G_t_s2m: " << sys_trimesh->Gamma_t_s2m_UU << "\n";
    paramStream << "RollingCoeff_s2m: " << sys_trimesh->rolling_coeff_s2m_UU << "\n";
    paramStream << "SpinningCoeff_s2m: " << sys_trimesh->spinning_coeff_s2m_UU << "\n";
    paramStream << "StaticFrictionCoeff_s2m: " << sys_trimesh->tri_params->static_friction_coeff_s2m << "\n";
    paramStream << "MeshCollisionEnabled: " << sys_trimesh->mesh_collision_enabled << "\n";

    cpFile << paramStream.str();
}
// GpuMesh version of checkpoint writing
void ChSystemGpuMesh::WriteCheckpointFile(const std::string& outfilename) {
    printf("Writing checkpoint data to file \"%s\"\n", outfilename.c_str());
    std::ofstream cpFile(outfilename, std::ios::out);

    // Header, indicating the system
    cpFile << std::string("ChSystemGpuMesh\n");

    // Simulation params go right after the system name
    WriteCheckpointParams(cpFile);
    WriteCheckpointMeshParams(cpFile);  // Meshed system has extra params
    cpFile << std::string("ParamsEnd\n");
    cpFile << std::string("\n");

    // Then, the particle kinematics info
    cpFile << std::string("CsvParticles\n");
    // In checkpointing, we want all particle info written, instead of selected output that the user can enforce via
    // setting OutputFlags. Therefore, we temporarily set OutputFlags to maximum, then revert after this writting is
    // done.
    unsigned int outFlags = m_sys->output_flags;
    SetParticleOutputFlags(VEL_COMPONENTS | FIXITY | ANG_VEL_COMPONENTS);
    WriteCsvParticles(cpFile);
    SetParticleOutputFlags(outFlags);
    cpFile << std::string("\n");

    // Then, write contact pair/friction history
    if (m_sys->gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
        cpFile << std::string("HstHistory\n");
        WriteHstHistory(cpFile);
        cpFile << std::string("\n");
    }

    // In the future, TODO checkpointing mesh and mesh translation/rotation goes here.
}

void ChSystemGpuMesh::WriteMesh(const std::string& outfilename, unsigned int i) const {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    if (sys_trimesh->file_write_mode == CHGPU_OUTPUT_MODE::NONE) {
        return;
    }
    if (i >= m_meshes.size()) {
        printf("WARNING: attempted to write mesh %u, yet only %zu meshes present. No mesh file generated.\n", i,
               m_meshes.size());
        return;
    }

    std::string ofile;
    if (outfilename.substr(outfilename.length() - std::min(outfilename.length(), (size_t)4)) != ".vtk" &&
        outfilename.substr(outfilename.length() - std::min(outfilename.length(), (size_t)4)) != ".VTK")
        ofile = outfilename + ".vtk";
    else
        ofile = outfilename;
    std::ofstream outfile(ofile, std::ios::out);
    std::ostringstream ostream;
    ostream << "# vtk DataFile Version 2.0\n";
    ostream << "VTK from simulation\n";
    ostream << "ASCII\n";
    ostream << "\n\n";

    ostream << "DATASET UNSTRUCTURED_GRID\n";

    const auto& mmesh = m_meshes.at(i);

    // Writing vertices
    ostream << "POINTS " << mmesh->getCoordsVertices().size() << " float" << std::endl;
    for (auto& v : mmesh->getCoordsVertices()) {
        float3 point = make_float3(v.x(), v.y(), v.z());
        sys_trimesh->ApplyFrameTransform(point, sys_trimesh->tri_params->fam_frame_broad[i].pos,
                                         sys_trimesh->tri_params->fam_frame_broad[i].rot_mat);
        ostream << point.x << " " << point.y << " " << point.z << std::endl;
    }

    // Writing faces
    ostream << "\n\n";
    ostream << "CELLS " << mmesh->getIndicesVertexes().size() << " " << 4 * mmesh->getIndicesVertexes().size()
            << std::endl;
    for (auto& f : mmesh->getIndicesVertexes())
        ostream << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;

    // Writing face types. Type 5 is generally triangles
    ostream << "\n\n";
    ostream << "CELL_TYPES " << mmesh->getIndicesVertexes().size() << std::endl;
    auto nfaces = mmesh->getIndicesVertexes().size();
    for (size_t j = 0; j < nfaces; j++)
        ostream << "5 " << std::endl;

    outfile << ostream.str();
}

/// get index list of neighbors
void ChSystemGpu::getNeighbors(unsigned int ID, std::vector<unsigned int>& neighborList) {
    m_sys->getNeighbors(ID, neighborList);
}

/// Get rolling friction torque between body i and j, return 0 if not in contact
ChVector<float> ChSystemGpu::getRollingFrictionTorque(unsigned int i, unsigned int j) {
    float3 m_roll = m_sys->getRollingFrictionTorque(i, j);
    return ChVector<float>(m_roll.x, m_roll.y, m_roll.z);
}

/// Get v_rot for rolling friction
ChVector<float> ChSystemGpu::getRollingVrot(unsigned int i, unsigned int j) {
    float3 vrot = m_sys->getRollingVrot(i, j);
    return ChVector<float>(vrot.x, vrot.y, vrot.z);
}

/// get contact char time
float ChSystemGpu::getRollingCharContactTime(unsigned int i, unsigned int j) {
    return m_sys->getRollingCharContactTime(i, j);
}

/// Get tangential friction force between body i and j, return 0 if not in contact
ChVector<float> ChSystemGpu::getSlidingFrictionForce(unsigned int i, unsigned int j) {
    float3 fr = m_sys->getSlidingFrictionForce(i, j);
    return ChVector<float>(fr.x, fr.y, fr.z);
}

/// Get normal friction force between body i and j, return 0 if not in contact
ChVector<float> ChSystemGpu::getNormalForce(unsigned int i, unsigned int j) {
    float3 N = m_sys->getNormalForce(i, j);
    return ChVector<float>(N.x, N.y, N.z);
}

void ChSystemGpuMesh::WriteMeshes(const std::string& outfilename) const {
    ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(m_sys);
    if (sys_trimesh->file_write_mode == CHGPU_OUTPUT_MODE::NONE) {
        return;
    }
    if (m_meshes.size() == 0) {
        printf(
            "WARNING: attempted to write meshes to file yet no mesh found in system cache. No mesh file "
            "generated.\n");
        return;
    }

    std::vector<unsigned int> vertexOffset(m_meshes.size() + 1, 0);
    size_t total_f = 0;
    size_t total_v = 0;

    // printf("Writing %zu mesh(es)...\n", m_meshes.size());
    std::string ofile;
    if (outfilename.substr(outfilename.length() - std::min(outfilename.length(), (size_t)4)) != ".vtk" &&
        outfilename.substr(outfilename.length() - std::min(outfilename.length(), (size_t)4)) != ".VTK")
        ofile = outfilename + ".vtk";
    else
        ofile = outfilename;
    std::ofstream outfile(ofile, std::ios::out);
    std::ostringstream ostream;
    ostream << "# vtk DataFile Version 2.0\n";
    ostream << "VTK from simulation\n";
    ostream << "ASCII\n";
    ostream << "\n\n";

    ostream << "DATASET UNSTRUCTURED_GRID\n";

    // Prescan the V and F: to write all meshes to one file, we need vertex number offset info
    unsigned int mesh_num = 0;
    for (const auto& mmesh : m_meshes) {
        vertexOffset[mesh_num + 1] = (unsigned int)mmesh->getCoordsVertices().size();
        total_v += mmesh->getCoordsVertices().size();
        total_f += mmesh->getIndicesVertexes().size();
        mesh_num++;
    }
    for (unsigned int i = 1; i < m_meshes.size(); i++)
        vertexOffset[i] = vertexOffset[i] + vertexOffset[i - 1];

    // Writing vertices
    ostream << "POINTS " << total_v << " float" << std::endl;
    mesh_num = 0;
    for (const auto& mmesh : m_meshes) {
        for (auto& v : mmesh->getCoordsVertices()) {
            float3 point = make_float3(v.x(), v.y(), v.z());
            sys_trimesh->ApplyFrameTransform(point, sys_trimesh->tri_params->fam_frame_broad[mesh_num].pos,
                                             sys_trimesh->tri_params->fam_frame_broad[mesh_num].rot_mat);
            ostream << point.x << " " << point.y << " " << point.z << std::endl;
        }
        mesh_num++;
    }

    // Writing faces
    ostream << "\n\n";
    ostream << "CELLS " << total_f << " " << 4 * total_f << std::endl;
    mesh_num = 0;
    for (const auto& mmesh : m_meshes) {
        for (auto& f : mmesh->getIndicesVertexes()) {
            ostream << "3 " << f.x() + vertexOffset[mesh_num] << " " << f.y() + vertexOffset[mesh_num] << " "
                    << f.z() + vertexOffset[mesh_num] << std::endl;
        }
        mesh_num++;
    }

    // Writing face types. Type 5 is generally triangles
    ostream << "\n\n";
    ostream << "CELL_TYPES " << total_f << std::endl;
    for (const auto& mmesh : m_meshes) {
        auto nfaces = mmesh->getIndicesVertexes().size();
        for (size_t j = 0; j < nfaces; j++)
            ostream << "5 " << std::endl;
    }

    outfile << ostream.str();
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
