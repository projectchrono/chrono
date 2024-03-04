// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Chrono custom multicore collision system.
// Contains both the broadphase and the narrowphase methods.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/utils/ChProfiler.h"

#include "chrono/collision/multicore/ChCollisionSystemMulticore.h"
#include "chrono/collision/multicore/ChRayTest.h"

namespace chrono {

CH_FACTORY_REGISTER(ChCollisionSystemMulticore)
CH_UPCASTING(ChCollisionSystemMulticore, ChCollisionSystem)

ChCollisionSystemMulticore::ChCollisionSystemMulticore() : use_aabb_active(false) {
    // Create the shared data structure with own state data
    cd_data = chrono_types::make_shared<ChCollisionData>(true);
    cd_data->collision_envelope = ChCollisionModel::GetDefaultSuggestedEnvelope();

    broadphase.cd_data = cd_data;
    narrowphase.cd_data = cd_data;
}

ChCollisionSystemMulticore::~ChCollisionSystemMulticore() {}

// -----------------------------------------------------------------------------

void ChCollisionSystemMulticore::SetEnvelope(double envelope) {
    cd_data->collision_envelope = real(envelope);
}

void ChCollisionSystemMulticore::SetBroadphaseGridResolution(const ChVector<int>& num_bins) {
    broadphase.grid_resolution = vec3(num_bins.x(), num_bins.y(), num_bins.z());
    broadphase.grid_type = ChBroadphase::GridType::FIXED_RESOLUTION;
}

void ChCollisionSystemMulticore::SetBroadphaseGridSize(const ChVector<>& bin_size) {
    broadphase.bin_size = real3(bin_size.x(), bin_size.y(), bin_size.z());
    broadphase.grid_type = ChBroadphase::GridType::FIXED_RESOLUTION;
}

void ChCollisionSystemMulticore::SetBroadphaseGridDensity(double density) {
    broadphase.grid_density = real(density);
    broadphase.grid_type = ChBroadphase::GridType::FIXED_DENSITY;
}

void ChCollisionSystemMulticore::SetNarrowphaseAlgorithm(ChNarrowphase::Algorithm algorithm) {
    narrowphase.algorithm = algorithm;
}

void ChCollisionSystemMulticore::EnableActiveBoundingBox(const ChVector<>& aabb_min, const ChVector<>& aabb_max) {
    active_aabb_min = FromChVector(aabb_min);
    active_aabb_max = FromChVector(aabb_max);

    use_aabb_active = true;
}

void ChCollisionSystemMulticore::SetNumThreads(int nthreads) {
#ifdef _OPENMP
    omp_set_num_threads(nthreads);
#endif
}

// -----------------------------------------------------------------------------

void ChCollisionSystemMulticore::Add(std::shared_ptr<ChCollisionModel> model) {
    assert(!model->HasImplementation());

    auto ct_model = chrono_types::make_shared<ChCollisionModelMulticore>(model.get());
    ct_model->Populate();

    int body_id = ct_model->GetBody()->GetId();
    short2 fam = S2(ct_model->model->GetFamilyGroup(), ct_model->model->GetFamilyMask());

    // The offset for this shape will the current total number of points in the convex data list
    auto& shape_data = cd_data->shape_data;
    int convex_data_offset = (int)shape_data.convex_rigid.size();

    // Insert the points into the global convex list
    shape_data.convex_rigid.insert(shape_data.convex_rigid.end(), ct_model->local_convex_data.begin(),
                                   ct_model->local_convex_data.end());

    // Shape index in the collision model
    int local_shape_index = 0;

    // Traverse all collision shapes in the model
    auto num_shapes = ct_model->m_shapes.size();
    assert(num_shapes == ct_model->m_ct_shapes.size());

    for (size_t i = 0; i < num_shapes; i++) {
        auto type = ct_model->m_shapes[i]->GetType();
        const auto& ct_shape = ct_model->m_ct_shapes[i];
        real3 obA = ct_shape->A;
        real3 obB = ct_shape->B;
        real3 obC = ct_shape->C;

        // Compute the global offset of the convex data structure based on the number of points already present
        int length = 1;
        int start;

        switch (type) {
            case ChCollisionShape::Type::SPHERE:
                start = (int)shape_data.sphere_rigid.size();
                shape_data.sphere_rigid.push_back(obB.x);
                break;
            case ChCollisionShape::Type::ELLIPSOID:
                start = (int)shape_data.box_like_rigid.size();
                shape_data.box_like_rigid.push_back(obB);
                break;
            case ChCollisionShape::Type::BOX:
                start = (int)shape_data.box_like_rigid.size();
                shape_data.box_like_rigid.push_back(obB);
                break;
            case ChCollisionShape::Type::CYLINDER:
                start = (int)shape_data.box_like_rigid.size();
                shape_data.box_like_rigid.push_back(obB);
                break;
            case ChCollisionShape::Type::CYLSHELL:
                start = (int)shape_data.box_like_rigid.size();
                shape_data.box_like_rigid.push_back(obB);
                break;
            case ChCollisionShape::Type::CONE:
                start = (int)shape_data.box_like_rigid.size();
                shape_data.box_like_rigid.push_back(obB);
                break;
            case ChCollisionShape::Type::CAPSULE:
                start = (int)shape_data.capsule_rigid.size();
                shape_data.capsule_rigid.push_back(real2(obB.x, obB.y));
                break;
            case ChCollisionShape::Type::ROUNDEDBOX:
                start = (int)shape_data.rbox_like_rigid.size();
                shape_data.rbox_like_rigid.push_back(real4(obB, obC.x));
                break;
            case ChCollisionShape::Type::ROUNDEDCYL:
                start = (int)shape_data.rbox_like_rigid.size();
                shape_data.rbox_like_rigid.push_back(real4(obB, obC.x));
                break;
            case ChCollisionShape::Type::CONVEXHULL:
                start = (int)(obB.y + convex_data_offset);
                length = (int)obB.x;
                break;
            case ChCollisionShape::Type::TRIANGLE:
                start = (int)shape_data.triangle_rigid.size();
                shape_data.triangle_rigid.push_back(obA);
                shape_data.triangle_rigid.push_back(obB);
                shape_data.triangle_rigid.push_back(obC);
                break;
            default:
                start = -1;
                break;
        }

        shape_data.ObA_rigid.push_back(obA);
        shape_data.ObR_rigid.push_back(ct_shape->R);
        shape_data.start_rigid.push_back(start);
        shape_data.length_rigid.push_back(length);

        shape_data.fam_rigid.push_back(fam);
        shape_data.typ_rigid.push_back(type);
        shape_data.id_rigid.push_back(body_id);
        shape_data.local_rigid.push_back(local_shape_index);
        cd_data->num_rigid_shapes++;
        local_shape_index++;
    }

    ct_models.push_back(ct_model);
}

void ChCollisionSystemMulticore::Clear() {
    ct_models.clear();
    //// TODO more here
}

#define ERASE_MACRO(x, y) x.erase(x.begin() + y);
#define ERASE_MACRO_LEN(x, y, z) x.erase(x.begin() + y, x.begin() + y + z);

void ChCollisionSystemMulticore::Remove(std::shared_ptr<ChCollisionModel> model) {
    //// TODO
    std::cout << "\nChCollisionSystemMulticore::Remove() not yet implemented.\n" << std::endl;
    throw ChException("ChCollisionSystemMulticore::Remove() not yet implemented.");
}

#undef ERASE_MACRO
#undef ERASE_MACRO_LEN

// -----------------------------------------------------------------------------

bool ChCollisionSystemMulticore::GetActiveBoundingBox(ChVector<>& aabb_min, ChVector<>& aabb_max) const {
    aabb_min = ToChVector(active_aabb_min);
    aabb_max = ToChVector(active_aabb_max);

    return use_aabb_active;
}

geometry::ChAABB ChCollisionSystemMulticore::GetBoundingBox() const {
    ChVector<> aabb_min((double)cd_data->min_bounding_point.x, (double)cd_data->min_bounding_point.y,
                        (double)cd_data->min_bounding_point.z);
    ChVector<> aabb_max((double)cd_data->max_bounding_point.x, (double)cd_data->max_bounding_point.y,
                        (double)cd_data->max_bounding_point.z);

    return geometry::ChAABB(aabb_min, aabb_max);
}

void ChCollisionSystemMulticore::ResetTimers() {
    m_timer_broad.reset();
    m_timer_narrow.reset();
}

double ChCollisionSystemMulticore::GetTimerCollisionBroad() const {
    return m_timer_broad();
}

double ChCollisionSystemMulticore::GetTimerCollisionNarrow() const {
    return m_timer_narrow();
}

// -----------------------------------------------------------------------------

void ChCollisionSystemMulticore::PreProcess() {
    assert(cd_data->owns_state_data);

    std::vector<real3>& position = *cd_data->state_data.pos_rigid;
    std::vector<quaternion>& rotation = *cd_data->state_data.rot_rigid;
    std::vector<char>& active = *cd_data->state_data.active_rigid;
    std::vector<char>& collide = *cd_data->state_data.collide_rigid;

    const auto& blist = m_system->Get_bodylist();
    int nbodies = static_cast<int>(blist.size());

    position.resize(nbodies);
    rotation.resize(nbodies);
    active.resize(nbodies);
    collide.resize(nbodies);

    cd_data->state_data.num_rigid_bodies = nbodies;
    cd_data->state_data.num_fluid_bodies = 0;

#pragma omp parallel for
    for (int i = 0; i < nbodies; i++) {
        const auto& body = blist[i];

        ChVector<>& body_pos = body->GetPos();
        ChQuaternion<>& body_rot = body->GetRot();

        position[i] = real3(body_pos.x(), body_pos.y(), body_pos.z());
        rotation[i] = quaternion(body_rot.e0(), body_rot.e1(), body_rot.e2(), body_rot.e3());

        active[i] = body->IsActive();
        collide[i] = body->GetCollide();
    }
}

void ChCollisionSystemMulticore::PostProcess() {
    if (use_aabb_active) {
        const auto& active = *cd_data->state_data.active_rigid;
        auto& blist = m_system->Get_bodylist();
        int nbodies = static_cast<int>(blist.size());

#pragma omp parallel for
        for (int i = 0; i < nbodies; i++) {
            blist[i]->SetSleeping(!active[i]);
        }
    }
}

void ChCollisionSystemMulticore::Run() {
    ResetTimers();

    if (use_aabb_active) {
        std::vector<char>& active = *cd_data->state_data.active_rigid;
        const std::vector<char>& collide = *cd_data->state_data.collide_rigid;

        body_active.resize(cd_data->state_data.num_rigid_bodies);
        std::fill(body_active.begin(), body_active.end(), false);

        GetOverlappingAABB(body_active, active_aabb_min, active_aabb_max);

#pragma omp parallel for
        for (int i = 0; i < active.size(); i++) {
            if (active[i] != 0 && collide[i] != 0) {
                active[i] = body_active[i];
            }
        }
    }

    // Broadphase
    {
        CH_PROFILE("Broad-phase");
        m_timer_broad.start();
        GenerateAABB();
        broadphase.Process();
        m_timer_broad.stop();
    }

    // Narrowphase
    {
        CH_PROFILE("Narrow-phase");
        m_timer_narrow.start();
        narrowphase.Process();
        m_timer_narrow.stop();
    }
}

// -----------------------------------------------------------------------------

void ChCollisionSystemMulticore::ReportContacts(ChContactContainer* container) {
    const auto& blist = m_system->Get_bodylist();

    // Resize global arrays with composite material properties.
    // NOTE: important to do this here, to set size to zero if no contacts (in case some other added by a custom user
    // callback)
    container->BeginAddContact();

    const auto& bids = cd_data->bids_rigid_rigid;          // global IDs of bodies in contact
    const auto& sids = cd_data->contact_shapeIDs;          // global IDs of shapes in contact
    const auto& sindex = cd_data->shape_data.local_rigid;  // collision model indexes of shapes in contact

    // Loop over all current contacts, create the cinfo structure and add contact to the container.
    // Note that inclusions in the contact container cannot be done in parallel.
    for (uint i = 0; i < cd_data->num_rigid_contacts; i++) {
        auto b1 = bids[i].x;                  // global IDs of bodies in contact
        auto b2 = bids[i].y;                  //
        auto s1 = int(sids[i] >> 32);         // global IDs of shapes in contact
        auto s2 = int(sids[i] & 0xffffffff);  //
        auto s1_index = sindex[s1];           // indexes of shapes in contact within their collision model
        auto s2_index = sindex[s2];           //

        auto modelA = (ChCollisionModelMulticore*)blist[b1]->GetCollisionModel()->GetImplementation();
        auto modelB = (ChCollisionModelMulticore*)blist[b2]->GetCollisionModel()->GetImplementation();

        ChCollisionInfo cinfo;
        cinfo.modelA = blist[b1]->GetCollisionModel().get();
        cinfo.modelB = blist[b2]->GetCollisionModel().get();
        cinfo.shapeA = modelA->m_shapes[s1_index].get();
        cinfo.shapeB = modelB->m_shapes[s2_index].get();
        cinfo.vN = ToChVector(cd_data->norm_rigid_rigid[i]);
        cinfo.vpA = ToChVector(cd_data->cpta_rigid_rigid[i]);
        cinfo.vpB = ToChVector(cd_data->cptb_rigid_rigid[i]);
        cinfo.distance = cd_data->dpth_rigid_rigid[i];
        cinfo.eff_radius = cd_data->erad_rigid_rigid[i];

        // Execute user custom callback, if any
        bool add_contact = true;
        if (this->narrow_callback)
            add_contact = this->narrow_callback->OnNarrowphase(cinfo);

        if (add_contact)
            container->AddContact(cinfo);
    }

    container->EndAddContact();
}

// -----------------------------------------------------------------------------

static void ComputeAABBSphere(const real& radius,
                              const real3& lpositon,
                              const real3& position,
                              const quaternion& body_rotation,
                              real3& minp,
                              real3& maxp) {
    real3 pos = Rotate(lpositon, body_rotation) + position;
    minp = pos - radius;
    maxp = pos + radius;
}

static void ComputeAABBTriangle(const real3& A, const real3& B, const real3& C, real3& minp, real3& maxp) {
    minp.x = Min(A.x, Min(B.x, C.x));
    minp.y = Min(A.y, Min(B.y, C.y));
    minp.z = Min(A.z, Min(B.z, C.z));
    maxp.x = Max(A.x, Max(B.x, C.x));
    maxp.y = Max(A.y, Max(B.y, C.y));
    maxp.z = Max(A.z, Max(B.z, C.z));
}

static void ComputeAABBBox(const real3& dim,
                           const real3& lpositon,
                           const real3& position,
                           const quaternion& rotation,
                           const quaternion& body_rotation,
                           real3& minp,
                           real3& maxp) {
    real3 temp = AbsRotate(rotation, dim);
    real3 pos = Rotate(lpositon, body_rotation) + position;
    minp = pos - temp;
    maxp = pos + temp;
}

/*
static void ComputeAABBCone(const real3& dim,
                            const real3& lpositon,
                            const real3& positon,
                            const quaternion& rotation,
                            const quaternion& body_rotation,
                            real3& minp,
                            real3& maxp) {
    real3 temp = AbsRotate(rotation, real3(dim.x, dim.y, dim.z / 2.0));
    real3 pos = Rotate(lpositon - real3(0, 0, dim.z / 2.0), body_rotation) + positon;
    minp = pos - temp;
    maxp = pos + temp;
}
*/

static void ComputeAABBConvex(const real3* convex_points,
                              const int start,
                              const int size,
                              const real3& lpos,
                              const real3& pos,
                              const quaternion& rot,
                              real3& minp,
                              real3& maxp) {
    real3 point_0 = Rotate(convex_points[start] + lpos, rot) + pos;

    minp = maxp = point_0;
    for (int i = start; i < start + size; i++) {
        real3 p = Rotate(convex_points[i] + lpos, rot) + pos;
        if (minp.x > p.x) {
            minp.x = p.x;
        }
        if (minp.y > p.y) {
            minp.y = p.y;
        }
        if (minp.z > p.z) {
            minp.z = p.z;
        }
        if (maxp.x < p.x) {
            maxp.x = p.x;
        }
        if (maxp.y < p.y) {
            maxp.y = p.y;
        }
        if (maxp.z < p.z) {
            maxp.z = p.z;
        }
    }
}

void ChCollisionSystemMulticore::GenerateAABB() {
    if (cd_data->num_rigid_shapes > 0) {
        const real envelope = cd_data->collision_envelope;
        const std::vector<shape_type>& typ_rigid = cd_data->shape_data.typ_rigid;
        const std::vector<int>& start_rigid = cd_data->shape_data.start_rigid;
        const std::vector<uint>& id_rigid = cd_data->shape_data.id_rigid;
        const std::vector<real3>& obj_data_A = cd_data->shape_data.ObA_rigid;
        const std::vector<quaternion>& obj_data_R = cd_data->shape_data.ObR_rigid;
        const std::vector<real3>& convex_rigid = cd_data->shape_data.convex_rigid;

        const std::vector<real3>& pos_rigid = *cd_data->state_data.pos_rigid;
        const std::vector<quaternion>& body_rot = *cd_data->state_data.rot_rigid;

        const uint num_rigid_shapes = cd_data->num_rigid_shapes;

        std::vector<real3>& aabb_min = cd_data->aabb_min;
        std::vector<real3>& aabb_max = cd_data->aabb_max;

        aabb_min.resize(num_rigid_shapes);
        aabb_max.resize(num_rigid_shapes);

#pragma omp parallel for
        for (int index = 0; index < (signed)num_rigid_shapes; index++) {
            // Shape data
            shape_type type = typ_rigid[index];
            real3 local_pos = obj_data_A[index];
            quaternion local_rot = obj_data_R[index];
            uint id = id_rigid[index];  // The rigid body corresponding to this shape
            int start = start_rigid[index];

            // Body data
            if (id == UINT_MAX)
                continue;

            real3 position = pos_rigid[id];
            quaternion rotation = Mult(body_rot[id], local_rot);
            real3 temp_min;
            real3 temp_max;

            if (type == ChCollisionShape::Type::SPHERE) {
                real radius = cd_data->shape_data.sphere_rigid[start];
                ComputeAABBSphere(radius + envelope, local_pos, position, body_rot[id], temp_min, temp_max);

            } else if (type == ChCollisionShape::Type::ELLIPSOID || type == ChCollisionShape::Type::BOX ||
                       type == ChCollisionShape::Type::CYLINDER || type == ChCollisionShape::Type::CYLSHELL ||
                       type == ChCollisionShape::Type::CONE) {
                real3 B = cd_data->shape_data.box_like_rigid[start];
                ComputeAABBBox(B + envelope, local_pos, position, rotation, body_rot[id], temp_min, temp_max);

            } else if (type == ChCollisionShape::Type::ROUNDEDBOX || type == ChCollisionShape::Type::ROUNDEDCYL) {
                real4 T = cd_data->shape_data.rbox_like_rigid[start];
                real3 B = real3(T.x, T.y, T.z) + T.w + envelope;
                ComputeAABBBox(B, local_pos, position, rotation, body_rot[id], temp_min, temp_max);

            } else if (type == ChCollisionShape::Type::CAPSULE) {
                real2 T = cd_data->shape_data.capsule_rigid[start];
                real3 B = real3(T.x, T.x + T.y, T.x) + envelope;
                ComputeAABBBox(B, local_pos, position, rotation, body_rot[id], temp_min, temp_max);

            } else if (type == ChCollisionShape::Type::CONVEXHULL) {
                int length = cd_data->shape_data.length_rigid[index];
                ComputeAABBConvex(convex_rigid.data(), start, length, local_pos, position, rotation, temp_min,
                                  temp_max);
                temp_min -= envelope;
                temp_max += envelope;

            } else if (type == ChCollisionShape::Type::TRIANGLE) {
                real3 A, B, C;

                A = cd_data->shape_data.triangle_rigid[start + 0];
                B = cd_data->shape_data.triangle_rigid[start + 1];
                C = cd_data->shape_data.triangle_rigid[start + 2];

                A = Rotate(A, body_rot[id]) + position;
                B = Rotate(B, body_rot[id]) + position;
                C = Rotate(C, body_rot[id]) + position;

                ComputeAABBTriangle(A, B, C, temp_min, temp_max);

            } else {
                continue;
            }

            aabb_min[index] = temp_min;
            aabb_max[index] = temp_max;
        }
    }
}

void ChCollisionSystemMulticore::GetOverlappingAABB(std::vector<char>& active_id, real3 Amin, real3 Amax) {
    GenerateAABB();

#pragma omp parallel for
    for (int i = 0; i < cd_data->shape_data.typ_rigid.size(); i++) {
        real3 Bmin = cd_data->aabb_min[i];
        real3 Bmax = cd_data->aabb_max[i];

        bool inContact = (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) &&
                         (Amin.z <= Bmax.z && Bmin.z <= Amax.z);
        if (inContact) {
            active_id[cd_data->shape_data.id_rigid[i]] = true;
        }
    }
}

std::vector<vec2> ChCollisionSystemMulticore::GetOverlappingPairs() {
    std::vector<vec2> pairs;
    pairs.resize(cd_data->pair_shapeIDs.size());
    for (int i = 0; i < cd_data->pair_shapeIDs.size(); i++) {
        vec2 pair = I2(int(cd_data->pair_shapeIDs[i] >> 32), int(cd_data->pair_shapeIDs[i] & 0xffffffff));
        pairs[i] = pair;
    }
    return pairs;
}

// -----------------------------------------------------------------------------

bool ChCollisionSystemMulticore::RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& result) const {
    if (cd_data->num_active_bins == 0) {
        result.hit = false;
        return false;
    }

    ChRayTest tester(cd_data);
    ChRayTest::RayHitInfo info;
    if (tester.Check(FromChVector(from), FromChVector(to), info)) {
        // Hit point
        result.hit = true;
        result.abs_hitNormal = ToChVector(info.normal);
        result.abs_hitPoint = ToChVector(info.point);
        result.dist_factor = info.t;

        // ID of the body carring the closest hit shape
        uint bid = cd_data->shape_data.id_rigid[info.shapeID];

        // Collision model of hit body
        result.hitModel = m_system->Get_bodylist()[bid]->GetCollisionModel().get();

        return true;
    }

    result.hit = false;
    return false;
}

bool ChCollisionSystemMulticore::RayHit(const ChVector<>& from,
                                        const ChVector<>& to,
                                        ChCollisionModel* model,
                                        ChRayhitResult& result) const {
    return false;
}

// -----------------------------------------------------------------------------

void DrawHemisphere(ChCollisionSystem::VisualizationCallback* vis,
                    const ChCoordsys<>& csys,
                    double radius,
                    const ChColor& color) {
    int vstep = 30;  // degrees
    int rstep = 30;  // degrees

    for (int j = 0; j < 90; j += vstep) {
        double y_low = radius * std::sin(j * CH_C_DEG_TO_RAD);
        double r_low = radius * std::cos(j * CH_C_DEG_TO_RAD);

        double y_top = radius * std::sin((j + vstep) * CH_C_DEG_TO_RAD);
        double r_top = radius * std::cos((j + vstep) * CH_C_DEG_TO_RAD);

        ChVector<> crt(r_low, y_low, 0);
        for (int i = 0; i < 360; i += rstep) {
            double rc = r_top * std::cos(i * CH_C_DEG_TO_RAD);
            double rs = r_top * std::sin(i * CH_C_DEG_TO_RAD);
            ChVector<> up(rc, y_top, rs);
            vis->DrawLine(csys.TransformPointLocalToParent(crt), csys.TransformPointLocalToParent(up), color);
            rc = r_low * std::cos((i + rstep) * CH_C_DEG_TO_RAD);
            rs = r_low * std::sin((i + rstep) * CH_C_DEG_TO_RAD);
            ChVector<> next(rc, y_low, rs);
            vis->DrawLine(csys.TransformPointLocalToParent(crt), csys.TransformPointLocalToParent(next), color);
            crt = next;
        }
    }
}

void DrawSphere(ChCollisionSystem::VisualizationCallback* vis,
                const ChCoordsys<>& csys,
                double radius,
                const ChColor& color) {
    DrawHemisphere(vis, csys, radius, color);
    ChCoordsys<> csys1 = csys;
    csys1.rot = csys.rot * Q_from_AngX(CH_C_PI);
    DrawHemisphere(vis, csys1, radius, color);
}

void DrawBox(ChCollisionSystem::VisualizationCallback* vis,
             const ChCoordsys<>& csys,
             const ChVector<>& hdim,
             const ChColor& color) {
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(-hdim[0], -hdim[1], -hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(+hdim[0], -hdim[1], -hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(+hdim[0], -hdim[1], -hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(+hdim[0], +hdim[1], -hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(+hdim[0], +hdim[1], -hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(-hdim[0], +hdim[1], -hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(-hdim[0], +hdim[1], -hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(-hdim[0], -hdim[1], -hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(-hdim[0], -hdim[1], -hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(-hdim[0], -hdim[1], +hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(+hdim[0], -hdim[1], -hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(+hdim[0], -hdim[1], +hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(+hdim[0], +hdim[1], -hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(+hdim[0], +hdim[1], +hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(-hdim[0], +hdim[1], -hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(-hdim[0], +hdim[1], +hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(-hdim[0], -hdim[1], +hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(+hdim[0], -hdim[1], +hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(+hdim[0], -hdim[1], +hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(+hdim[0], +hdim[1], +hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(+hdim[0], +hdim[1], +hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(-hdim[0], +hdim[1], +hdim[2])), color);
    vis->DrawLine(csys.TransformPointLocalToParent(ChVector<>(-hdim[0], +hdim[1], +hdim[2])),
                  csys.TransformPointLocalToParent(ChVector<>(-hdim[0], -hdim[1], +hdim[2])), color);
}

void DrawCylinder(ChCollisionSystem::VisualizationCallback* vis,
                  const ChCoordsys<>& csys,
                  double radius,
                  double hlen,
                  const ChColor& color) {
    int rstep = 30;
    ChVector<> p_start(radius, 0, -hlen);
    ChVector<> p_end(radius, 0, +hlen);

    for (int i = 0; i < 360; i += rstep) {
        vis->DrawLine(csys.TransformPointLocalToParent(p_start), csys.TransformPointLocalToParent(p_end), color);
        double rc = radius * std::cos((i + rstep) * CH_C_DEG_TO_RAD);
        double rs = radius * std::sin((i + rstep) * CH_C_DEG_TO_RAD);
        ChVector<> start(rc, rs, -hlen);
        ChVector<> end(rc, rs, +hlen);
        vis->DrawLine(csys.TransformPointLocalToParent(p_start), csys.TransformPointLocalToParent(start), color);
        vis->DrawLine(csys.TransformPointLocalToParent(p_end), csys.TransformPointLocalToParent(end), color);
        p_start = start;
        p_end = end;
    }
}

void DrawCone(ChCollisionSystem::VisualizationCallback* vis,
              const ChCoordsys<>& csys,
              double radius,
              double hlen,
              const ChColor& color) {
    int rstep = 30;
    ChVector<> p_start(radius, 0, 0);
    ChVector<> p_end(0, 0, 2 * hlen);

    for (int i = 0; i < 360; i += rstep) {
        vis->DrawLine(csys.TransformPointLocalToParent(p_start), csys.TransformPointLocalToParent(p_end), color);
        double rc = radius * std::cos((i + rstep) * CH_C_DEG_TO_RAD);
        double rs = radius * std::sin((i + rstep) * CH_C_DEG_TO_RAD);
        ChVector<> start(rc, rs, 0);
        ChVector<> end(rc, rs, 2 * hlen);
        vis->DrawLine(csys.TransformPointLocalToParent(p_start), csys.TransformPointLocalToParent(start), color);
        p_start = start;
    }
}

void DrawCapsule(ChCollisionSystem::VisualizationCallback* vis,
                 const ChCoordsys<>& csys,
                 double radius,
                 double hlen,
                 const ChColor& color) {
    int rstep = 30;

    for (int i = 0; i < 360; i += rstep) {
        double rc = radius * std::cos((i + rstep) * CH_C_DEG_TO_RAD);
        double rs = radius * std::sin((i + rstep) * CH_C_DEG_TO_RAD);
        ChVector<> start(rc, rs, -hlen);
        ChVector<> end(rc, rs, +hlen);
        vis->DrawLine(csys.TransformPointLocalToParent(start), csys.TransformPointLocalToParent(end), color);
    }
    ChCoordsys<> csys1;
    csys1.pos = csys.TransformPointLocalToParent(ChVector<>(0, 0, hlen));
    csys1.rot = csys.rot;
    DrawHemisphere(vis, csys1, radius, color);
    ChCoordsys<> csys2;
    csys2.pos = csys.TransformPointLocalToParent(ChVector<>(0, 0, -hlen));
    csys2.rot = csys.rot * Q_from_AngX(CH_C_PI);
    DrawHemisphere(vis, csys2, radius, color);
}

void ChCollisionSystemMulticore::Visualize(int flags) {
    if (!vis_callback || flags == VisualizationModes::VIS_None)
        return;

    if (flags & VisualizationModes::VIS_Shapes)
        VisualizeShapes();
    if (flags & VisualizationModes::VIS_Aabb)
        VisualizeAABB();
    if (flags & VisualizationModes::VIS_Contacts)
        VisualizeContacts();
}

void ChCollisionSystemMulticore::VisualizeShapes() {
    const real envelope = cd_data->collision_envelope;
    const std::vector<shape_type>& typ_rigid = cd_data->shape_data.typ_rigid;
    const std::vector<int>& start_rigid = cd_data->shape_data.start_rigid;
    const std::vector<uint>& id_rigid = cd_data->shape_data.id_rigid;
    const std::vector<real3>& obj_data_A = cd_data->shape_data.ObA_rigid;
    const std::vector<quaternion>& obj_data_R = cd_data->shape_data.ObR_rigid;

    const std::vector<real3>& pos_rigid = *cd_data->state_data.pos_rigid;
    const std::vector<quaternion>& body_rot = *cd_data->state_data.rot_rigid;

    const uint num_rigid_shapes = cd_data->num_rigid_shapes;

    for (uint index = 0; index < num_rigid_shapes; index++) {
        // Shape data
        shape_type type = typ_rigid[index];
        real3 local_pos = obj_data_A[index];
        quaternion local_rot = obj_data_R[index];
        uint id = id_rigid[index];  // The rigid body corresponding to this shape
        int start = start_rigid[index];

        // Body data
        if (id == UINT_MAX)
            continue;

        real3 position = pos_rigid[id] + Rotate(local_pos, body_rot[id]);
        quaternion rotation = Mult(body_rot[id], local_rot);

        switch (type) {
            case ChCollisionShape::Type::SPHERE: {
                real radius = cd_data->shape_data.sphere_rigid[start];
                DrawSphere(vis_callback.get(), ChCoordsys<>(ToChVector(position), ToChQuaternion(rotation)),
                           double(radius + envelope), ChColor(1, 0, 0));
                break;
            }
            case ChCollisionShape::Type::BOX: {
                const real3& B = cd_data->shape_data.box_like_rigid[start];
                DrawBox(vis_callback.get(), ChCoordsys<>(ToChVector(position), ToChQuaternion(rotation)),
                        ToChVector(B + envelope), ChColor(1, 0, 0));
                break;
            }
            case ChCollisionShape::Type::CYLINDER:
            case ChCollisionShape::Type::CYLSHELL: {
                const real3& B = cd_data->shape_data.box_like_rigid[start];
                DrawCylinder(vis_callback.get(), ChCoordsys<>(ToChVector(position), ToChQuaternion(rotation)),
                             double(B.x + envelope), double(B.z + envelope), ChColor(1, 0, 0));
                break;
            }
            case ChCollisionShape::Type::CAPSULE: {
                const real3& B = cd_data->shape_data.box_like_rigid[start];
                DrawCapsule(vis_callback.get(), ChCoordsys<>(ToChVector(position), ToChQuaternion(rotation)),
                            double(B.x + envelope), double(B.y + envelope), ChColor(1, 0, 0));
                break;
            }
            case ChCollisionShape::Type::ELLIPSOID: {
                break;
            }
            case ChCollisionShape::Type::CONE: {
                const real3& B = cd_data->shape_data.box_like_rigid[start];
                DrawCone(vis_callback.get(), ChCoordsys<>(ToChVector(position), ToChQuaternion(rotation)),
                         double(B.x + envelope), double(B.y + envelope), ChColor(1, 0, 0));
                break;
            }
            case ChCollisionShape::Type::TRIANGLE: {
                real3 A = Rotate(cd_data->shape_data.triangle_rigid[start + 0], body_rot[id]) + pos_rigid[id];
                real3 B = Rotate(cd_data->shape_data.triangle_rigid[start + 1], body_rot[id]) + pos_rigid[id];
                real3 C = Rotate(cd_data->shape_data.triangle_rigid[start + 2], body_rot[id]) + pos_rigid[id];
                vis_callback->DrawLine(ToChVector(A), ToChVector(B), ChColor(1, 0, 0));
                vis_callback->DrawLine(ToChVector(B), ToChVector(C), ChColor(1, 0, 0));
                vis_callback->DrawLine(ToChVector(C), ToChVector(A), ChColor(1, 0, 0));
                break;
            }
        }
    }
}

void ChCollisionSystemMulticore::VisualizeAABB() {
    const uint num_rigid_shapes = cd_data->num_rigid_shapes;
    std::vector<real3>& aabb_min = cd_data->aabb_min;
    std::vector<real3>& aabb_max = cd_data->aabb_max;

    for (uint index = 0; index < num_rigid_shapes; index++) {
        real3 center = cd_data->global_origin + 0.5 * (aabb_max[index] + aabb_min[index]);
        real3 hdim = 0.5 * (aabb_max[index] - aabb_min[index]);
        DrawBox(vis_callback.get(), ChCoordsys<>(ToChVector(center), QUNIT), ToChVector(hdim), ChColor(0, 0, 1));
    }
}

void ChCollisionSystemMulticore::VisualizeContacts() {
    for (uint i = 0; i < cd_data->num_rigid_contacts; i++) {
        real3 from = cd_data->cptb_rigid_rigid[i];
        real3 to = cd_data->cptb_rigid_rigid[i] + vis_callback->GetNormalScale() * cd_data->norm_rigid_rigid[i];
        vis_callback->DrawLine(ToChVector(from), ToChVector(to), ChColor(1, 1, 0));
    }
}

void ChCollisionSystemMulticore::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionSystemMulticore>();
    // serialize parent class
    ChCollisionSystem::ArchiveOut(marchive);
}

void ChCollisionSystemMulticore::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionSystemMulticore>();
    // deserialize parent class
    ChCollisionSystem::ArchiveIn(marchive);
}

}  // end namespace chrono
