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
// Authors: Radu Serban, Hammad Mazhar, Arman Pazouki
// =============================================================================
//
// =============================================================================

#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCapsuleShape.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChRoundedBoxShape.h"
#include "chrono/assets/ChRoundedCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace chrono {
using namespace geometry;
using namespace collision;
namespace utils {

// -----------------------------------------------------------------------------

void AddSphereGeometry(ChBody* body,
                       std::shared_ptr<ChMaterialSurface> material,
                       double radius,
                       const ChVector<>& pos,
                       const ChQuaternion<>& rot,
                       bool visualization,
                       std::shared_ptr<ChVisualMaterial> vis_material) {
    body->GetCollisionModel()->AddSphere(material, radius, pos);

    if (visualization) {
        if (!body->GetVisualModel()) {
            auto model = chrono_types::make_shared<ChVisualModel>();
            body->AddVisualModel(model);
        }
        auto sphere = chrono_types::make_shared<ChSphereShape>();
        sphere->GetSphereGeometry().rad = radius;
        sphere->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(sphere, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddEllipsoidGeometry(ChBody* body,
                          std::shared_ptr<ChMaterialSurface> material,
                          const ChVector<>& size,
                          const ChVector<>& pos,
                          const ChQuaternion<>& rot,
                          bool visualization,
                          std::shared_ptr<ChVisualMaterial> vis_material) {
    body->GetCollisionModel()->AddEllipsoid(material, size.x(), size.y(), size.z(), pos, rot);

    if (visualization) {
        if (!body->GetVisualModel()) {
            auto model = chrono_types::make_shared<ChVisualModel>();
            body->AddVisualModel(model);
        }
        auto ellipsoid = chrono_types::make_shared<ChEllipsoidShape>();
        ellipsoid->GetEllipsoidGeometry().rad = size;
        ellipsoid->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(ellipsoid, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddBoxGeometry(ChBody* body,
                    std::shared_ptr<ChMaterialSurface> material,
                    const ChVector<>& size,
                    const ChVector<>& pos,
                    const ChQuaternion<>& rot,
                    bool visualization,
                    std::shared_ptr<ChVisualMaterial> vis_material) {
    body->GetCollisionModel()->AddBox(material, size.x(), size.y(), size.z(), pos, rot);

    if (visualization) {
        if (!body->GetVisualModel()) {
            auto model = chrono_types::make_shared<ChVisualModel>();
            body->AddVisualModel(model);
        }
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = size;
        box->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(box, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddBiSphereGeometry(ChBody* body,
                         std::shared_ptr<ChMaterialSurface> material,
                         double radius,
                         double cDist,
                         const ChVector<>& pos,
                         const ChQuaternion<>& rot,
                         bool visualization,
                         std::shared_ptr<ChVisualMaterial> vis_material) {
    ChFrame<> frame;
    frame = ChFrame<>(pos, rot);
    if (ChBodyAuxRef* body_ar = dynamic_cast<ChBodyAuxRef*>(body)) {
        frame = frame >> body_ar->GetFrame_REF_to_COG();
    }
    const ChVector<>& position = frame.GetPos();

    AddSphereGeometry(body, material, radius, position + ChVector<>(0, 0.5 * cDist, 0), rot, visualization,
                      vis_material);
    AddSphereGeometry(body, material, radius, position - ChVector<>(0, 0.5 * cDist, 0), rot, visualization,
                      vis_material);
}

// -----------------------------------------------------------------------------

void AddCapsuleGeometry(ChBody* body,
                        std::shared_ptr<ChMaterialSurface> material,
                        double radius,
                        double hlen,
                        const ChVector<>& pos,
                        const ChQuaternion<>& rot,
                        bool visualization,
                        std::shared_ptr<ChVisualMaterial> vis_material) {
    body->GetCollisionModel()->AddCapsule(material, radius, hlen, pos, rot);

    if (visualization) {
        if (!body->GetVisualModel()) {
            auto model = chrono_types::make_shared<ChVisualModel>();
            body->AddVisualModel(model);
        }
        auto capsule = chrono_types::make_shared<ChCapsuleShape>();
        capsule->GetCapsuleGeometry().rad = radius;
        capsule->GetCapsuleGeometry().hlen = hlen;
        capsule->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(capsule, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddCylinderGeometry(ChBody* body,
                         std::shared_ptr<ChMaterialSurface> material,
                         double radius,
                         double hlen,
                         const ChVector<>& pos,
                         const ChQuaternion<>& rot,
                         bool visualization,
                         std::shared_ptr<ChVisualMaterial> vis_material) {
    body->GetCollisionModel()->AddCylinder(material, radius, radius, hlen, pos, rot);

    if (visualization) {
        if (!body->GetVisualModel()) {
            auto model = chrono_types::make_shared<ChVisualModel>();
            body->AddVisualModel(model);
        }
        auto cylinder = chrono_types::make_shared<ChCylinderShape>();
        cylinder->GetCylinderGeometry().rad = radius;
        cylinder->GetCylinderGeometry().p1 = ChVector<>(0, hlen, 0);
        cylinder->GetCylinderGeometry().p2 = ChVector<>(0, -hlen, 0);
        cylinder->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(cylinder, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddConeGeometry(ChBody* body,
                     std::shared_ptr<ChMaterialSurface> material,
                     double radius,
                     double height,
                     const ChVector<>& pos,
                     const ChQuaternion<>& rot,
                     bool visualization,
                     std::shared_ptr<ChVisualMaterial> vis_material) {
    ChFrame<> frame;
    frame = ChFrame<>(pos, rot);
    if (ChBodyAuxRef* body_ar = dynamic_cast<ChBodyAuxRef*>(body)) {
        frame = frame >> body_ar->GetFrame_REF_to_COG();
    }
    ChVector<> pos_shifted = frame.GetPos() + ChVector<>(0, 0.25 * height, 0);

    body->GetCollisionModel()->AddCone(material, radius, radius, height, pos_shifted, rot);

    if (visualization) {
        if (!body->GetVisualModel()) {
            auto model = chrono_types::make_shared<ChVisualModel>();
            body->AddVisualModel(model);
        }
        auto cone = chrono_types::make_shared<ChConeShape>();
        cone->GetConeGeometry().rad = ChVector<>(radius, height, radius);
        cone->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(cone, ChFrame<>(pos_shifted, rot));
    }
}

// -----------------------------------------------------------------------------

bool AddTriangleMeshGeometry(ChBody* body,
                             std::shared_ptr<ChMaterialSurface> material,
                             const std::string& obj_filename,
                             const std::string& name,
                             const ChVector<>& pos,
                             const ChQuaternion<>& rot,
                             bool visualization,
                             std::shared_ptr<ChVisualMaterial> vis_material) {
    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(obj_filename, false, false);
    if (!trimesh)
        return false;

    for (int i = 0; i < trimesh->m_vertices.size(); i++)
        trimesh->m_vertices[i] = pos + rot.Rotate(trimesh->m_vertices[i]);

    body->GetCollisionModel()->AddTriangleMesh(material, trimesh, false, false);

    if (visualization) {
        if (!body->GetVisualModel()) {
            auto model = chrono_types::make_shared<ChVisualModel>();
            body->AddVisualModel(model);
        }
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(name);
        trimesh_shape->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AddTriangleMeshConvexDecomposition(ChBody* body,
                                        std::shared_ptr<ChMaterialSurface> material,
                                        const std::string& obj_filename,
                                        const std::string& name,
                                        const ChVector<>& pos,
                                        const ChQuaternion<>& rot,
                                        float skin_thickness,
                                        bool use_original_asset,
                                        std::shared_ptr<ChVisualMaterial> vis_material) {
    if (!body->GetVisualModel()) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        body->AddVisualModel(model);
    }

    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(obj_filename, true, false);
    if (!trimesh)
        return false;

    for (int i = 0; i < trimesh->m_vertices.size(); i++) {
        trimesh->m_vertices[i] = pos + rot.Rotate(trimesh->m_vertices[i]);
    }
    collision::ChConvexDecompositionHACDv2 decomposition;

    int hacd_maxhullcount = 512;
    int hacd_maxhullmerge = 256;
    int hacd_maxhullvertexes = 64;
    float hacd_concavity = 0.2f;
    float hacd_smallclusterthreshold = 0.0f;
    float hacd_fusetolerance = 1e-9f;

    decomposition.Reset();
    decomposition.AddTriangleMesh(*trimesh);

    decomposition.SetParameters(hacd_maxhullcount, hacd_maxhullmerge, hacd_maxhullvertexes, hacd_concavity,
                                hacd_smallclusterthreshold, hacd_fusetolerance);
    decomposition.ComputeConvexDecomposition();

    int hull_count = decomposition.GetHullCount();
    std::vector<ChVector<double>> convexhull;
    for (int c = 0; c < hull_count; c++) {
        decomposition.GetConvexHullResult(c, convexhull);

        body->GetCollisionModel()->AddConvexHull(material, convexhull, pos, rot);
        if (!use_original_asset) {
            std::stringstream ss;
            ss << name << "_" << c;
            auto trimesh_convex = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
            if (!decomposition.GetConvexHullResult(c, *trimesh_convex))
                return false;

            auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh_convex);
            trimesh_shape->SetName(ss.str());
            trimesh_shape->AddMaterial(vis_material);
            body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>());
        }
    }

    if (use_original_asset) {
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(name);
        trimesh_shape->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AddTriangleMeshConvexDecompositionV2(ChBody* body,
                                          std::shared_ptr<ChMaterialSurface> material,
                                          const std::string& obj_filename,
                                          const std::string& name,
                                          const ChVector<>& pos,
                                          const ChQuaternion<>& rot,
                                          bool use_original_asset,
                                          std::shared_ptr<ChVisualMaterial> vis_material) {
    if (!body->GetVisualModel()) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        body->AddVisualModel(model);
    }

    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(obj_filename, true, false);
    if (!trimesh)
        return false;

    for (int i = 0; i < trimesh->m_vertices.size(); i++) {
        trimesh->m_vertices[i] = pos + rot.Rotate(trimesh->m_vertices[i]);
    }
    collision::ChConvexDecompositionHACDv2 mydecompositionHACDv2;

    int hacd_maxhullcount = 1024;
    int hacd_maxhullmerge = 256;
    int hacd_maxhullvertexes = 64;
    double hacd_concavity = 0.01;
    double hacd_smallclusterthreshold = 0.0;
    double hacd_fusetolerance = 1e-6;

    mydecompositionHACDv2.Reset();
    mydecompositionHACDv2.AddTriangleMesh(*trimesh);

    mydecompositionHACDv2.SetParameters(hacd_maxhullcount, hacd_maxhullmerge, hacd_maxhullvertexes,
                                        (float)hacd_concavity, (float)hacd_smallclusterthreshold,
                                        (float)hacd_fusetolerance);
    mydecompositionHACDv2.ComputeConvexDecomposition();
    collision::ChConvexDecomposition* used_decomposition = &mydecompositionHACDv2;

    int hull_count = used_decomposition->GetHullCount();

    for (int c = 0; c < hull_count; c++) {
        std::vector<ChVector<double>> convexhull;
        if (!used_decomposition->GetConvexHullResult(c, convexhull))
            return false;

        body->GetCollisionModel()->AddConvexHull(material, convexhull, pos, rot);
        if (!use_original_asset) {
            std::stringstream ss;
            ss << name << "_" << c;
            auto trimesh_convex = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
            if (!used_decomposition->GetConvexHullResult(c, *trimesh_convex))
                return false;

            auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh_convex);
            trimesh_shape->SetName(ss.str());
            trimesh_shape->AddMaterial(vis_material);
            body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>());
        }
    }
    if (use_original_asset) {
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(name);
        trimesh_shape->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>());
    }

    return true;
}

// -----------------------------------------------------------------------------

//// TODO: extend this to also work for smooth (penalty) contact systems.

bool AddTriangleMeshConvexDecompositionSplit(ChSystem* system,
                                             std::shared_ptr<ChMaterialSurface> material,
                                             const std::string& obj_filename,
                                             const std::string& name,
                                             const ChVector<>& pos,
                                             const ChQuaternion<>& rot,
                                             double total_mass) {
    assert(material->GetContactMethod() == system->GetContactMethod());

    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(obj_filename, true, false);
    if (!trimesh)
        return false;

    for (int i = 0; i < trimesh->m_vertices.size(); i++) {
        trimesh->m_vertices[i] = pos + rot.Rotate(trimesh->m_vertices[i]);
    }
    collision::ChConvexDecompositionHACDv2 mydecompositionHACDv2;

    int hacd_maxhullcount = 1024;
    int hacd_maxhullmerge = 256;
    int hacd_maxhullvertexes = 64;
    double hacd_concavity = 0.01;
    double hacd_smallclusterthreshold = 0.1;
    double hacd_fusetolerance = 1e-6;

    mydecompositionHACDv2.Reset();
    mydecompositionHACDv2.AddTriangleMesh(*trimesh);

    mydecompositionHACDv2.SetParameters(hacd_maxhullcount, hacd_maxhullmerge, hacd_maxhullvertexes,
                                        (float)hacd_concavity, (float)hacd_smallclusterthreshold,
                                        (float)hacd_fusetolerance);
    mydecompositionHACDv2.ComputeConvexDecomposition();
    collision::ChConvexDecomposition* used_decomposition = &mydecompositionHACDv2;

    int hull_count = used_decomposition->GetHullCount();

    std::shared_ptr<ChBody> body;
    double mass;
    ChVector<> center;
    ChMatrix33<> inertia;
    double sum = 0;
    for (int c = 0; c < hull_count; c++) {
        geometry::ChTriangleMeshConnected trimesh_convex;
        if (!used_decomposition->GetConvexHullResult(c, trimesh_convex))
            return false;
        trimesh_convex.ComputeMassProperties(true, mass, center, inertia);
        sum += mass;
    }

    double scale = 1.0 / sum;

    for (int c = 0; c < hull_count; c++) {
        auto trimesh_convex = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        if (!used_decomposition->GetConvexHullResult(c, *trimesh_convex))
            return false;
        trimesh_convex->ComputeMassProperties(true, mass, center, inertia);

        body = std::shared_ptr<ChBody>(system->NewBody());
        body->SetMass(mass);
        body->SetPos(pos);
        body->SetRot(rot);
        body->SetCollide(true);
        body->SetBodyFixed(false);

        std::vector<ChVector<double>> convexhull;
        if (!used_decomposition->GetConvexHullResult(c, convexhull))
            return false;
        for (size_t v = 0; v < convexhull.size(); v++) {
            convexhull[v] = convexhull[v] - center;
        }

        body->GetCollisionModel()->ClearModel();
        body->GetCollisionModel()->AddConvexHull(material, convexhull, pos, rot);
        body->GetCollisionModel()->BuildModel();

        std::stringstream ss;
        ss << name << "_" << c;
        //      geometry::ChTriangleMeshConnected trimesh_convex;
        //      used_decomposition->GetConvexHullResult(c, trimesh_convex);

        auto model = chrono_types::make_shared<ChVisualModel>();
        body->AddVisualModel(model);

        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh_convex);
        trimesh_shape->SetName(ss.str());
        body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>());

        // std::cout << mass << " " << scale * mass* total_mass << " " <<
        // inertia(0, 0) << " " << inertia(1, 1) << " " << inertia(2, 2) << std::endl;
        body->SetInertiaXX(ChVector<>(inertia(0, 0) * scale * total_mass, inertia(1, 1) * scale * total_mass,
                                      inertia(2, 2) * scale * total_mass));

        system->AddBody(body);
    }

    return true;
}

// -----------------------------------------------------------------------------

void AddTriangleGeometry(ChBody* body,
                         std::shared_ptr<ChMaterialSurface> material,
                         const ChVector<>& vertA,
                         const ChVector<>& vertB,
                         const ChVector<>& vertC,
                         const std::string& name,
                         const ChVector<>& pos,
                         const ChQuaternion<>& rot,
                         bool visualization,
                         std::shared_ptr<ChVisualMaterial> vis_material) {
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->m_vertices.clear();
    trimesh->m_face_v_indices.clear();
    trimesh->m_vertices.push_back(vertA);
    trimesh->m_vertices.push_back(vertB);
    trimesh->m_vertices.push_back(vertC);
    trimesh->m_face_v_indices.push_back(ChVector<int>(0, 1, 2));

    for (int i = 0; i < trimesh->m_vertices.size(); i++)
        trimesh->m_vertices[i] = pos + rot.Rotate(trimesh->m_vertices[i]);

    body->GetCollisionModel()->AddTriangleMesh(material, trimesh, false, false);

    if (visualization) {
        if (!body->GetVisualModel()) {
            auto model = chrono_types::make_shared<ChVisualModel>();
            body->AddVisualModel(model);
        }
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(name);
        trimesh_shape->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>());
    }
}

// -----------------------------------------------------------------------------

void AddRoundedBoxGeometry(ChBody* body,
                           std::shared_ptr<ChMaterialSurface> material,
                           const ChVector<>& size,
                           double srad,
                           const ChVector<>& pos,
                           const ChQuaternion<>& rot,
                           bool visualization,
                           std::shared_ptr<ChVisualMaterial> vis_material) {
    body->GetCollisionModel()->AddRoundedBox(material, size.x(), size.y(), size.z(), srad, pos, rot);

    if (visualization) {
        if (!body->GetVisualModel()) {
            auto model = chrono_types::make_shared<ChVisualModel>();
            body->AddVisualModel(model);
        }
        auto box = chrono_types::make_shared<ChRoundedBoxShape>();
        box->GetRoundedBoxGeometry().Size = size;
        box->GetRoundedBoxGeometry().radsphere = srad;
        box->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(box, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddRoundedCylinderGeometry(ChBody* body,
                                std::shared_ptr<ChMaterialSurface> material,
                                double radius,
                                double hlen,
                                double srad,
                                const ChVector<>& pos,
                                const ChQuaternion<>& rot,
                                bool visualization,
                                std::shared_ptr<ChVisualMaterial> vis_material) {
    body->GetCollisionModel()->AddRoundedCylinder(material, radius, radius, hlen, srad, pos, rot);

    if (visualization) {
        if (!body->GetVisualModel()) {
            auto model = chrono_types::make_shared<ChVisualModel>();
            body->AddVisualModel(model);
        }
        auto rcyl = chrono_types::make_shared<ChRoundedCylinderShape>();
        rcyl->GetRoundedCylinderGeometry().rad = radius;
        rcyl->GetRoundedCylinderGeometry().hlen = hlen;
        rcyl->GetRoundedCylinderGeometry().radsphere = srad;
        rcyl->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(rcyl, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddTorusGeometry(ChBody* body,
                      std::shared_ptr<ChMaterialSurface> material,
                      double radius,
                      double thickness,
                      int segments,
                      int angle,
                      const ChVector<>& pos,
                      const ChQuaternion<>& rot,
                      bool visualization,
                      std::shared_ptr<ChVisualMaterial> vis_material) {
    for (int i = 0; i < angle; i += angle / segments) {
        double alpha = i * CH_C_PI / 180.0;
        double x = cos(alpha) * radius;
        double z = sin(alpha) * radius;
        Quaternion q = chrono::Q_from_AngAxis(-alpha, VECT_Y) % chrono::Q_from_AngAxis(CH_C_PI / 2.0, VECT_X);
        double outer_circ = 2 * CH_C_PI * (radius + thickness);

        AddCapsuleGeometry(body, material, thickness, outer_circ / segments * .5, ChVector<>(x, 0, z) + pos, q,
                           visualization, vis_material);
    }
}

// -----------------------------------------------------------------------------
// CreateBoxContainer
//
// Create a fixed body with contact and asset geometry representing a box with 5
// walls (no top).
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> CreateBoxContainer(ChSystem* system,
                                           int id,
                                           std::shared_ptr<ChMaterialSurface> mat,
                                           const ChVector<>& hdim,
                                           double hthick,
                                           const ChVector<>& pos,
                                           const ChQuaternion<>& rot,
                                           bool collide,
                                           bool y_up,
                                           bool overlap,
                                           bool closed) {
    // Verify consistency of input arguments.
    assert(mat->GetContactMethod() == system->GetContactMethod());

    // Create the body and set material
    std::shared_ptr<ChBody> body(system->NewBody());

    // Set body properties and geometry.
    body->SetIdentifier(id);
    body->SetMass(1);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetCollide(collide);
    body->SetBodyFixed(true);
    double o_lap = 0;
    if (overlap) {
        o_lap = hthick * 2;
    }
    body->GetCollisionModel()->ClearModel();
    if (y_up) {
        AddBoxGeometry(body.get(), mat, ChVector<>(hdim.x() + o_lap, hthick, hdim.y() + o_lap),
                       ChVector<>(0, -hthick, 0));
        AddBoxGeometry(body.get(), mat, ChVector<>(hthick, hdim.z() + o_lap, hdim.y() + o_lap),
                       ChVector<>(-hdim.x() - hthick, hdim.z(), 0));
        AddBoxGeometry(body.get(), mat, ChVector<>(hthick, hdim.z() + o_lap, hdim.y() + o_lap),
                       ChVector<>(hdim.x() + hthick, hdim.z(), 0));
        AddBoxGeometry(body.get(), mat, ChVector<>(hdim.x() + o_lap, hdim.z() + o_lap, hthick),
                       ChVector<>(0, hdim.z(), -hdim.y() - hthick));
        AddBoxGeometry(body.get(), mat, ChVector<>(hdim.x() + o_lap, hdim.z() + o_lap, hthick),
                       ChVector<>(0, hdim.z(), hdim.y() + hthick));
        if (closed) {
            AddBoxGeometry(body.get(), mat, ChVector<>(hdim.x() + o_lap, hthick, hdim.y() + o_lap),
                           ChVector<>(0, hdim.z() * 2 + hthick, 0));
        }
    } else {
        AddBoxGeometry(body.get(), mat, ChVector<>(hdim.x() + o_lap, hdim.y() + o_lap, hthick),
                       ChVector<>(0, 0, -hthick));
        AddBoxGeometry(body.get(), mat, ChVector<>(hthick, hdim.y() + o_lap, hdim.z() + o_lap),
                       ChVector<>(-hdim.x() - hthick, 0, hdim.z()));
        AddBoxGeometry(body.get(), mat, ChVector<>(hthick, hdim.y() + o_lap, hdim.z() + o_lap),
                       ChVector<>(hdim.x() + hthick, 0, hdim.z()));
        AddBoxGeometry(body.get(), mat, ChVector<>(hdim.x() + o_lap, hthick, hdim.z() + o_lap),
                       ChVector<>(0, -hdim.y() - hthick, hdim.z()));
        AddBoxGeometry(body.get(), mat, ChVector<>(hdim.x() + o_lap, hthick, hdim.z() + o_lap),
                       ChVector<>(0, hdim.y() + hthick, hdim.z()));
        if (closed) {
            AddBoxGeometry(body.get(), mat, ChVector<>(hdim.x() + o_lap, hdim.y() + o_lap, hthick),
                           ChVector<>(0, 0, hdim.z() * 2 + hthick));
        }
    }
    body->GetCollisionModel()->BuildModel();

    // Attach the body to the system.
    system->AddBody(body);
    return body;
}

// -----------------------------------------------------------------------------
// CreateCylindricalContainerFromBoxes
//
// Create a fixed body with contact and asset geometry representing a cylindrical
// container with no top.
// hdim = (rad, rad, height) (the second and the third components are the same)
// by default, it is z_up
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> CreateCylindricalContainerFromBoxes(ChSystem* system,
                                                            int id,
                                                            std::shared_ptr<ChMaterialSurface> mat,
                                                            const ChVector<>& hdim,
                                                            double hthick,
                                                            int numBoxes,
                                                            const ChVector<>& pos,
                                                            const ChQuaternion<>& rot,
                                                            bool collide,
                                                            bool overlap,
                                                            bool closed,
                                                            bool isBoxBase,
                                                            bool partialVisualization) {
    // Verify consistency of input arguments.
    assert(mat->GetContactMethod() == system->GetContactMethod());

    // Create the body and set material
    std::shared_ptr<ChBody> body(system->NewBody());

    // Set body properties and geometry.
    body->SetIdentifier(id);
    // body->SetMass(1);
    body->SetPos(pos);
    body->SetRot(rot);
    body->SetCollide(collide);
    body->SetBodyFixed(true);

    double box_side = hdim.x() * 2.0 * tan(CH_C_PI / numBoxes);  // side length of cyl
    double o_lap = 0;
    if (overlap) {
        o_lap = hthick * 2;
    }
    double ang = 2.0 * CH_C_PI / numBoxes;
    ChVector<> p_boxSize = ChVector<>((box_side + hthick) / 2.0, hthick, hdim.z() + o_lap);  // size of plates
    ChVector<> p_pos;                                                                        // position of each plate
    ChQuaternion<> p_quat = QUNIT;                                                           // rotation of each plate
    body->GetCollisionModel()->ClearModel();

    for (int i = 0; i < numBoxes; i++) {
        p_pos = pos + ChVector<>(sin(ang * i) * (hthick + hdim.x()), cos(ang * i) * (hthick + hdim.x()), hdim.z());

        p_quat = Angle_to_Quat(AngleSet::RXYZ, ChVector<>(0, 0, ang * i));

        // this is here to make half the cylinder invisible.
        bool m_visualization = true;
        if ((ang * i > CH_C_PI && ang * i < 3.0 * CH_C_PI / 2.0) && partialVisualization) {
            m_visualization = false;
        }
        utils::AddBoxGeometry(body.get(), mat, p_boxSize, p_pos, p_quat, m_visualization);
    }

    // Add ground piece
    if (isBoxBase) {
        utils::AddBoxGeometry(body.get(), mat, Vector(hdim.x() + 2 * hthick, hdim.x() + 2 * hthick, hthick),
                              Vector(0, 0, -hthick), QUNIT, true);
    } else {
        utils::AddCylinderGeometry(body.get(), mat, hdim.x() + 2 * hthick, hthick, ChVector<>(0, 0, -hthick),
                                   Q_from_AngAxis(CH_C_PI / 2, VECT_X));
    }

    if (closed) {
        if (isBoxBase) {
            utils::AddBoxGeometry(body.get(), mat, Vector(hdim.x() + 2 * hthick, hdim.x() + 2 * hthick, hthick),
                                  Vector(0, 0, 2 * hdim.z() + hthick), QUNIT, true);
        } else {
            utils::AddCylinderGeometry(body.get(), mat, hdim.x() + 2 * hthick, hthick,
                                       ChVector<>(0, 0, 2 * hdim.z() + hthick), Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        }
    }

    body->GetCollisionModel()->SetEnvelope(0.2 * hthick);
    body->GetCollisionModel()->BuildModel();

    system->AddBody(body);
    return body;
}

// -----------------------------------------------------------------------------

bool LoadConvexMesh(const std::string& file_name,
                    ChTriangleMeshConnected& convex_mesh,
                    ChConvexDecompositionHACDv2& convex_shape,
                    const ChVector<>& pos,
                    const ChQuaternion<>& rot,
                    int hacd_maxhullcount,
                    int hacd_maxhullmerge,
                    int hacd_maxhullvertexes,
                    float hacd_concavity,
                    float hacd_smallclusterthreshold,
                    float hacd_fusetolerance) {
    if (!convex_mesh.LoadWavefrontMesh(file_name, true, false))
        return false;

    for (int i = 0; i < convex_mesh.m_vertices.size(); i++) {
        convex_mesh.m_vertices[i] = pos + rot.Rotate(convex_mesh.m_vertices[i]);
    }

    convex_shape.Reset();
    convex_shape.AddTriangleMesh(convex_mesh);
    convex_shape.SetParameters(hacd_maxhullcount, hacd_maxhullmerge, hacd_maxhullvertexes, hacd_concavity,
                               hacd_smallclusterthreshold, hacd_fusetolerance);
    convex_shape.ComputeConvexDecomposition();

    return true;
}

// -----------------------------------------------------------------------------
bool LoadConvexHulls(const std::string& file_name,
                     geometry::ChTriangleMeshConnected& convex_mesh,
                     std::vector<std::vector<ChVector<double>>>& convex_hulls) {
    convex_mesh.LoadWavefrontMesh(file_name, true, false);

    std::vector<tinyobj::shape_t> shapes;
    tinyobj::attrib_t att;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    if (!tinyobj::LoadObj(&att, &shapes, &materials, &warn, &err, file_name.c_str()))
        return false;

    convex_hulls.resize(shapes.size());

    for (size_t i = 0; i < shapes.size(); i++) {
        convex_hulls[i].clear();

        // hold onto unique ids temporarily
        std::vector<int> ids;

        for (int j = 0; j < shapes[i].mesh.indices.size(); j++) {
            int id = shapes[i].mesh.indices[j].vertex_index;

            // if vertex not already added, add new vertex
            if (std::find(ids.begin(), ids.end(), id) == ids.end()) {
                ChVector<double> pos(att.vertices[id * 3 + 0],  //
                                     att.vertices[id * 3 + 1],  //
                                     att.vertices[id * 3 + 2]);
                convex_hulls[i].push_back(pos);
                ids.push_back(id);
            }
        }
    }

    return true;
}
// -----------------------------------------------------------------------------

void AddConvexCollisionModel(std::shared_ptr<ChBody> body,
                             std::shared_ptr<ChMaterialSurface> material,
                             std::shared_ptr<ChTriangleMeshConnected> convex_mesh,
                             ChConvexDecompositionHACDv2& convex_shape,
                             const ChVector<>& pos,
                             const ChQuaternion<>& rot,
                             bool use_original_asset,
                             std::shared_ptr<ChVisualMaterial> vis_material) {
    if (!body->GetVisualModel()) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        body->AddVisualModel(model);
    }

    ChConvexDecomposition* used_decomposition = &convex_shape;

    int hull_count = used_decomposition->GetHullCount();

    for (int c = 0; c < hull_count; c++) {
        std::vector<ChVector<double>> convexhull;
        used_decomposition->GetConvexHullResult(c, convexhull);

        body->GetCollisionModel()->AddConvexHull(material, convexhull, pos, rot);
        // Add each convex chunk as a new asset
        if (!use_original_asset) {
            std::stringstream ss;
            ss << convex_mesh->GetFileName() << "_" << c;
            auto trimesh_convex = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
            used_decomposition->GetConvexHullResult(c, *trimesh_convex);

            auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh_convex);
            trimesh_shape->SetName(ss.str());
            trimesh_shape->AddMaterial(vis_material);
            body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>(pos, rot));
        }
    }
    // Add the original triangle mesh as asset
    if (use_original_asset) {
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(convex_mesh);
        trimesh_shape->SetName(convex_mesh->GetFileName());
        trimesh_shape->AddMaterial(vis_material);
        body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>());
    }
}

// -----------------------------------------------------------------------------

void AddConvexCollisionModel(std::shared_ptr<ChBody> body,
                             std::shared_ptr<ChMaterialSurface> material,
                             std::shared_ptr<ChTriangleMeshConnected> convex_mesh,
                             std::vector<std::vector<ChVector<double>>>& convex_hulls,
                             const ChVector<>& pos,
                             const ChQuaternion<>& rot,
                             std::shared_ptr<ChVisualMaterial> vis_material) {
    for (int c = 0; c < convex_hulls.size(); c++) {
        body->GetCollisionModel()->AddConvexHull(material, convex_hulls[c], pos, rot);
    }

    if (!body->GetVisualModel()) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        body->AddVisualModel(model);
    }

    // Add the original triangle mesh as asset
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(convex_mesh);
    trimesh_shape->SetName(convex_mesh->GetFileName());
    trimesh_shape->AddMaterial(vis_material);
    body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>(pos, rot));
}

}  // namespace utils
}  // namespace chrono
