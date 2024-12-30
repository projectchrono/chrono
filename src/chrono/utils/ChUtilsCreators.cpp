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

#include <cmath>

#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeRoundedBox.h"
#include "chrono/assets/ChVisualShapeRoundedCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace chrono {

namespace utils {

// -----------------------------------------------------------------------------

void AddSphereGeometry(ChBody* body,
                       ChContactMaterialSharedPtr material,
                       double radius,
                       const ChVector3d& pos,
                       const ChQuaterniond& rot,
                       bool visualization,
                       ChVisualMaterialSharedPtr vis_material) {
    auto cshape = chrono_types::make_shared<ChCollisionShapeSphere>(material, radius);
    body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

    if (visualization) {
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(radius);
        sphere->AddMaterial(vis_material);
        body->AddVisualShape(sphere, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddEllipsoidGeometry(ChBody* body,
                          ChContactMaterialSharedPtr material,
                          const ChVector3d& axes,
                          const ChVector3d& pos,
                          const ChQuaterniond& rot,
                          bool visualization,
                          ChVisualMaterialSharedPtr vis_material) {
    auto cshape = chrono_types::make_shared<ChCollisionShapeEllipsoid>(material, axes);
    body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

    if (visualization) {
        auto ellipsoid = chrono_types::make_shared<ChVisualShapeEllipsoid>(axes);
        ellipsoid->AddMaterial(vis_material);
        body->AddVisualShape(ellipsoid, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddBoxGeometry(ChBody* body,
                    ChContactMaterialSharedPtr material,
                    const ChVector3d& size,
                    const ChVector3d& pos,
                    const ChQuaterniond& rot,
                    bool visualization,
                    ChVisualMaterialSharedPtr vis_material) {
    auto cshape = chrono_types::make_shared<ChCollisionShapeBox>(material, size);
    body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

    if (visualization) {
        auto box = chrono_types::make_shared<ChVisualShapeBox>(size);
        box->AddMaterial(vis_material);
        body->AddVisualShape(box, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddBiSphereGeometry(ChBody* body,
                         ChContactMaterialSharedPtr material,
                         double radius,
                         double cDist,
                         const ChVector3d& pos,
                         const ChQuaterniond& rot,
                         bool visualization,
                         ChVisualMaterialSharedPtr vis_material) {
    ChFrame<> frame;
    frame = ChFrame<>(pos, rot);
    if (ChBodyAuxRef* body_ar = dynamic_cast<ChBodyAuxRef*>(body)) {
        frame = frame >> body_ar->GetFrameRefToCOM();
    }
    const ChVector3d& position = frame.GetPos();

    AddSphereGeometry(body, material, radius, position + ChVector3d(0, 0.5 * cDist, 0), rot, visualization,
                      vis_material);
    AddSphereGeometry(body, material, radius, position - ChVector3d(0, 0.5 * cDist, 0), rot, visualization,
                      vis_material);
}

// -----------------------------------------------------------------------------

void AddCapsuleGeometry(ChBody* body,
                        ChContactMaterialSharedPtr material,
                        double radius,
                        double height,
                        const ChVector3d& pos,
                        const ChQuaterniond& rot,
                        bool visualization,
                        ChVisualMaterialSharedPtr vis_material) {
    auto cshape = chrono_types::make_shared<ChCollisionShapeCapsule>(material, radius, height);
    body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

    if (visualization) {
        auto capsule = chrono_types::make_shared<ChVisualShapeCapsule>(radius, height);
        capsule->AddMaterial(vis_material);
        body->AddVisualShape(capsule, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddCylinderGeometry(ChBody* body,
                         ChContactMaterialSharedPtr material,
                         double radius,
                         double height,
                         const ChVector3d& pos,
                         const ChQuaterniond& rot,
                         bool visualization,
                         ChVisualMaterialSharedPtr vis_material) {
    auto cshape = chrono_types::make_shared<ChCollisionShapeCylinder>(material, radius, height);
    body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

    if (visualization) {
        auto cylinder = chrono_types::make_shared<ChVisualShapeCylinder>(radius, height);
        cylinder->AddMaterial(vis_material);
        body->AddVisualShape(cylinder, ChFrame<>(pos, rot));
    }
}

void AddCylinderGeometry(ChBody* body,
                         ChContactMaterialSharedPtr material,
                         double radius,
                         const ChVector3d& p1,
                         const ChVector3d& p2,
                         bool visualization,
                         ChVisualMaterialSharedPtr vis_material) {
    ChLineSegment seg(p1, p2);
    auto height = seg.GetLength();
    auto frame = seg.GetFrame();

    auto cshape = chrono_types::make_shared<ChCollisionShapeCylinder>(material, radius, height);
    body->AddCollisionShape(cshape, frame);

    if (visualization) {
        auto cylinder = chrono_types::make_shared<ChVisualShapeCylinder>(radius, height);
        cylinder->AddMaterial(vis_material);
        body->AddVisualShape(cylinder, frame);
    }
}

// -----------------------------------------------------------------------------

void AddConeGeometry(ChBody* body,
                     ChContactMaterialSharedPtr material,
                     double radius,
                     double height,
                     const ChVector3d& pos,
                     const ChQuaterniond& rot,
                     bool visualization,
                     ChVisualMaterialSharedPtr vis_material) {
    auto cshape = chrono_types::make_shared<ChCollisionShapeCone>(material, radius, height);
    body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

    if (visualization) {
        auto cone = chrono_types::make_shared<ChVisualShapeCone>(radius, height);
        cone->AddMaterial(vis_material);
        body->AddVisualShape(cone, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

bool AddTriangleMeshGeometry(ChBody* body,
                             ChContactMaterialSharedPtr material,
                             const std::string& obj_filename,
                             const std::string& name,
                             const ChVector3d& pos,
                             const ChQuaterniond& rot,
                             bool visualization,
                             ChVisualMaterialSharedPtr vis_material) {
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(obj_filename, false, false);
    if (!trimesh)
        return false;

    for (int i = 0; i < trimesh->m_vertices.size(); i++)
        trimesh->m_vertices[i] = pos + rot.Rotate(trimesh->m_vertices[i]);

    auto cshape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, trimesh, false, false);
    body->AddCollisionShape(cshape, ChFrame<>());

    if (visualization) {
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(name);
        trimesh_shape->AddMaterial(vis_material);
        body->AddVisualShape(trimesh_shape, ChFrame<>());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AddTriangleMeshConvexDecomposition(ChBody* body,
                                        ChContactMaterialSharedPtr material,
                                        const std::string& obj_filename,
                                        const std::string& name,
                                        const ChVector3d& pos,
                                        const ChQuaterniond& rot,
                                        float skin_thickness,
                                        bool use_original_asset,
                                        ChVisualMaterialSharedPtr vis_material) {
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(obj_filename, true, false);
    if (!trimesh)
        return false;

    for (int i = 0; i < trimesh->m_vertices.size(); i++) {
        trimesh->m_vertices[i] = pos + rot.Rotate(trimesh->m_vertices[i]);
    }
    ChConvexDecompositionHACDv2 decomposition;

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
    std::vector<ChVector3d> convexhull;
    for (int c = 0; c < hull_count; c++) {
        decomposition.GetConvexHullResult(c, convexhull);

        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, convexhull);
        body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

        if (!use_original_asset) {
            std::stringstream ss;
            ss << name << "_" << c;
            auto trimesh_convex = chrono_types::make_shared<ChTriangleMeshConnected>();
            if (!decomposition.GetConvexHullResult(c, *trimesh_convex))
                return false;

            auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(trimesh_convex);
            trimesh_shape->SetName(ss.str());
            trimesh_shape->AddMaterial(vis_material);
            body->AddVisualShape(trimesh_shape, ChFrame<>());
        }
    }

    if (use_original_asset) {
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(name);
        trimesh_shape->AddMaterial(vis_material);
        body->AddVisualShape(trimesh_shape, ChFrame<>());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AddTriangleMeshConvexDecompositionV2(ChBody* body,
                                          ChContactMaterialSharedPtr material,
                                          const std::string& obj_filename,
                                          const std::string& name,
                                          const ChVector3d& pos,
                                          const ChQuaterniond& rot,
                                          bool use_original_asset,
                                          ChVisualMaterialSharedPtr vis_material) {
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(obj_filename, true, false);
    if (!trimesh)
        return false;

    for (int i = 0; i < trimesh->m_vertices.size(); i++) {
        trimesh->m_vertices[i] = pos + rot.Rotate(trimesh->m_vertices[i]);
    }
    ChConvexDecompositionHACDv2 mydecompositionHACDv2;

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
    ChConvexDecomposition* used_decomposition = &mydecompositionHACDv2;

    int hull_count = used_decomposition->GetHullCount();

    for (int c = 0; c < hull_count; c++) {
        std::vector<ChVector3d> convexhull;
        if (!used_decomposition->GetConvexHullResult(c, convexhull))
            return false;

        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, convexhull);
        body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

        if (!use_original_asset) {
            std::stringstream ss;
            ss << name << "_" << c;
            auto trimesh_convex = chrono_types::make_shared<ChTriangleMeshConnected>();
            if (!used_decomposition->GetConvexHullResult(c, *trimesh_convex))
                return false;

            auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(trimesh_convex);
            trimesh_shape->SetName(ss.str());
            trimesh_shape->AddMaterial(vis_material);
            body->AddVisualShape(trimesh_shape, ChFrame<>());
        }
    }
    if (use_original_asset) {
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(name);
        trimesh_shape->AddMaterial(vis_material);
        body->AddVisualShape(trimesh_shape, ChFrame<>());
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AddTriangleMeshConvexDecompositionSplit(ChSystem* system,
                                             ChContactMaterialSharedPtr material,
                                             const std::string& obj_filename,
                                             const std::string& name,
                                             const ChVector3d& pos,
                                             const ChQuaterniond& rot,
                                             double total_mass) {
    assert(material->GetContactMethod() == system->GetContactMethod());

    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(obj_filename, true, false);
    if (!trimesh)
        return false;

    for (int i = 0; i < trimesh->m_vertices.size(); i++) {
        trimesh->m_vertices[i] = pos + rot.Rotate(trimesh->m_vertices[i]);
    }
    ChConvexDecompositionHACDv2 mydecompositionHACDv2;

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
    ChConvexDecomposition* used_decomposition = &mydecompositionHACDv2;

    int hull_count = used_decomposition->GetHullCount();

    std::shared_ptr<ChBody> body;
    double mass;
    ChVector3d center;
    ChMatrix33<> inertia;
    double sum = 0;
    for (int c = 0; c < hull_count; c++) {
        ChTriangleMeshConnected trimesh_convex;
        if (!used_decomposition->GetConvexHullResult(c, trimesh_convex))
            return false;
        trimesh_convex.ComputeMassProperties(true, mass, center, inertia);
        sum += mass;
    }

    double scale = 1.0 / sum;

    for (int c = 0; c < hull_count; c++) {
        auto trimesh_convex = chrono_types::make_shared<ChTriangleMeshConnected>();
        if (!used_decomposition->GetConvexHullResult(c, *trimesh_convex))
            return false;
        trimesh_convex->ComputeMassProperties(true, mass, center, inertia);

        body = chrono_types::make_shared<ChBody>();
        body->SetMass(mass);
        body->SetPos(pos);
        body->SetRot(rot);
        body->EnableCollision(true);
        body->SetFixed(false);

        std::vector<ChVector3d> convexhull;
        if (!used_decomposition->GetConvexHullResult(c, convexhull))
            return false;
        for (size_t v = 0; v < convexhull.size(); v++) {
            convexhull[v] = convexhull[v] - center;
        }

        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, convexhull);
        body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh_convex);
        trimesh_shape->SetName(name + "_" + std::to_string(c));
        body->AddVisualShape(trimesh_shape, ChFrame<>());

        body->SetInertiaXX(ChVector3d(inertia(0, 0) * scale * total_mass, inertia(1, 1) * scale * total_mass,
                                      inertia(2, 2) * scale * total_mass));

        system->AddBody(body);
    }

    return true;
}

// -----------------------------------------------------------------------------

void AddTriangleGeometry(ChBody* body,
                         ChContactMaterialSharedPtr material,
                         const ChVector3d& vertA,
                         const ChVector3d& vertB,
                         const ChVector3d& vertC,
                         const std::string& name,
                         const ChVector3d& pos,
                         const ChQuaterniond& rot,
                         bool visualization,
                         ChVisualMaterialSharedPtr vis_material) {
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    trimesh->m_vertices.clear();
    trimesh->m_face_v_indices.clear();
    trimesh->m_vertices.push_back(vertA);
    trimesh->m_vertices.push_back(vertB);
    trimesh->m_vertices.push_back(vertC);
    trimesh->m_face_v_indices.push_back(ChVector3i(0, 1, 2));

    for (int i = 0; i < trimesh->m_vertices.size(); i++)
        trimesh->m_vertices[i] = pos + rot.Rotate(trimesh->m_vertices[i]);

    auto cshape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, trimesh, false, false);
    body->AddCollisionShape(cshape, ChFrame<>());

    if (visualization) {
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(name);
        trimesh_shape->AddMaterial(vis_material);
        body->AddVisualShape(trimesh_shape, ChFrame<>());
    }
}

// -----------------------------------------------------------------------------

void AddRoundedBoxGeometry(ChBody* body,
                           ChContactMaterialSharedPtr material,
                           const ChVector3d& size,
                           double srad,
                           const ChVector3d& pos,
                           const ChQuaterniond& rot,
                           bool visualization,
                           ChVisualMaterialSharedPtr vis_material) {
    auto cshape = chrono_types::make_shared<ChCollisionShapeRoundedBox>(material, size, srad);
    body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

    if (visualization) {
        auto box = chrono_types::make_shared<ChVisualShapeRoundedBox>(size, srad);
        box->AddMaterial(vis_material);
        body->AddVisualShape(box, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddRoundedCylinderGeometry(ChBody* body,
                                ChContactMaterialSharedPtr material,
                                double radius,
                                double height,
                                double srad,
                                const ChVector3d& pos,
                                const ChQuaterniond& rot,
                                bool visualization,
                                ChVisualMaterialSharedPtr vis_material) {
    auto cshape = chrono_types::make_shared<ChCollisionShapeRoundedCylinder>(material, radius, height, srad);
    body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

    if (visualization) {
        auto rcyl = chrono_types::make_shared<ChVisualShapeRoundedCylinder>(radius, height, srad);
        rcyl->AddMaterial(vis_material);
        body->AddVisualShape(rcyl, ChFrame<>(pos, rot));
    }
}

// -----------------------------------------------------------------------------

void AddTorusGeometry(ChBody* body,
                      ChContactMaterialSharedPtr material,
                      double radius,
                      double thickness,
                      int segments,
                      int angle,
                      const ChVector3d& pos,
                      const ChQuaterniond& rot,
                      bool visualization,
                      ChVisualMaterialSharedPtr vis_material) {
    for (int i = 0; i < angle; i += angle / segments) {
        double alpha = i * CH_PI / 180.0;
        double x = std::cos(alpha) * radius;
        double z = std::sin(alpha) * radius;
        ChQuaterniond q = chrono::QuatFromAngleY(-alpha) * chrono::QuatFromAngleX(CH_PI_2);
        double outer_circ = 2 * CH_PI * (radius + thickness);

        AddCapsuleGeometry(body, material, thickness, outer_circ / segments * .5, ChVector3d(x, 0, z) + pos, q,
                           visualization, vis_material);
    }
}

// -----------------------------------------------------------------------------

void AddBoxContainer(std::shared_ptr<ChBody> body,
                     ChContactMaterialSharedPtr material,
                     const ChFrame<>& frame,
                     const ChVector3d& size,
                     double thickness,
                     const ChVector3i faces,
                     bool visualization,
                     ChVisualMaterialSharedPtr vis_material) {
    ChVector3d hsize = size / 2;
    double ht = thickness / 2;

    // Wall center positions
    ChVector3d xn(-hsize.x() - ht, 0, 0);
    ChVector3d xp(+hsize.x() + ht, 0, 0);
    ChVector3d yn(0, -hsize.y() - ht, 0);
    ChVector3d yp(0, +hsize.y() + ht, 0);
    ChVector3d zn(0, 0, -hsize.z() - ht);
    ChVector3d zp(0, 0, +hsize.z() + ht);

    // Wall dimensions
    ChVector3d sizeX(thickness, size.y(), size.z());
    ChVector3d sizeY(size.x(), thickness, size.z());
    ChVector3d sizeZ(size.x(), size.y(), thickness);

    // Z- wall
    if (faces.z() == -1 || faces.z() == 2) {
        auto f = frame * ChFrame<>(zn, QUNIT);
        AddBoxGeometry(body.get(), material, sizeZ, f.GetPos(), f.GetRot(), visualization, vis_material);
    }
    // Z+ wall
    if (faces.z() == +1 || faces.z() == 2) {
        auto f = frame * ChFrame<>(zp, QUNIT);
        AddBoxGeometry(body.get(), material, sizeZ, f.GetPos(), f.GetRot(), visualization, vis_material);
    }

    // X- wall
    if (faces.x() == -1 || faces.x() == 2) {
        auto f = frame * ChFrame<>(xn, QUNIT);
        AddBoxGeometry(body.get(), material, sizeX, f.GetPos(), f.GetRot(), visualization, vis_material);
    }
    // X+ wall
    if (faces.x() == +1 || faces.x() == 2) {
        auto f = frame * ChFrame<>(xp, QUNIT);
        AddBoxGeometry(body.get(), material, sizeX, f.GetPos(), f.GetRot(), visualization, vis_material);
    }

    // Y- wall
    if (faces.y() == -1 || faces.y() == 2) {
        auto f = frame * ChFrame<>(yn, QUNIT);
        AddBoxGeometry(body.get(), material, sizeY, f.GetPos(), f.GetRot(), visualization, vis_material);
    }
    // Y+ wall
    if (faces.y() == +1 || faces.y() == 2) {
        auto f = frame * ChFrame<>(yp, QUNIT);
        AddBoxGeometry(body.get(), material, sizeY, f.GetPos(), f.GetRot(), visualization, vis_material);
    }
}

// -----------------------------------------------------------------------------
// CreateBoxContainer
//
// Create a fixed body with contact and asset geometry representing a box with 5
// walls (no top).
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> CreateBoxContainer(ChSystem* system,
                                           ChContactMaterialSharedPtr mat,
                                           const ChVector3d& size,
                                           double thickness,
                                           const ChVector3d& pos,
                                           const ChQuaterniond& rot,
                                           bool collide,
                                           bool overlap,
                                           bool closed) {
    // Verify consistency of input arguments.
    assert(mat->GetContactMethod() == system->GetContactMethod());

    // Create the body and set material
    auto body = chrono_types::make_shared<ChBody>();

    // Set body properties and geometry.
    body->SetName("container_body");
    body->SetPos(pos);
    body->SetRot(rot);
    body->EnableCollision(collide);
    body->SetFixed(true);

    double o_lap = overlap ? thickness : 0.0;
    double hthick = thickness / 2;
    auto hdim = size / 2;

    AddBoxGeometry(body.get(), mat, ChVector3d(size.x() + o_lap, size.y() + o_lap, thickness),
                   ChVector3d(0, 0, -hthick));

    AddBoxGeometry(body.get(), mat, ChVector3d(thickness, size.y() + o_lap, size.z() + o_lap),
                   ChVector3d(-hdim.x() - hthick, 0, hdim.z()));
    AddBoxGeometry(body.get(), mat, ChVector3d(thickness, size.y() + o_lap, size.z() + o_lap),
                   ChVector3d(hdim.x() + hthick, 0, hdim.z()));

    AddBoxGeometry(body.get(), mat, ChVector3d(size.x() + o_lap, thickness, size.z() + o_lap),
                   ChVector3d(0, -hdim.y() - hthick, hdim.z()));
    AddBoxGeometry(body.get(), mat, ChVector3d(size.x() + o_lap, thickness, size.z() + o_lap),
                   ChVector3d(0, hdim.y() + hthick, hdim.z()));

    if (closed) {
        AddBoxGeometry(body.get(), mat, ChVector3d(size.x() + o_lap, size.y() + o_lap, thickness),
                       ChVector3d(0, 0, hdim.z() * 2 + hthick));
    }

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
                                                            ChContactMaterialSharedPtr mat,
                                                            double radius,
                                                            double height,
                                                            double thickness,
                                                            int numBoxes,
                                                            const ChVector3d& pos,
                                                            const ChQuaterniond& rot,
                                                            bool collide,
                                                            bool overlap,
                                                            bool closed,
                                                            bool isBoxBase,
                                                            bool partialVisualization) {
    // Verify consistency of input arguments.
    assert(mat->GetContactMethod() == system->GetContactMethod());

    // Create the body and set material
    auto body = chrono_types::make_shared<ChBody>();

    // Set body properties and geometry.
    body->SetPos(pos);
    body->SetRot(rot);
    body->EnableCollision(collide);
    body->SetFixed(true);

    double o_lap = overlap ? thickness : 0;
    double hthick = thickness / 2;

    // Add circumference pieces
    double box_side = radius * 2.0 * tan(CH_PI / numBoxes);                                     // side length of cyl
    ChVector3d plate_size = ChVector3d((box_side + thickness / 2), thickness, height + o_lap);  // size of plates
    double delta_angle = CH_2PI / numBoxes;

    for (int i = 0; i < numBoxes; i++) {
        double angle = i * delta_angle;
        auto plate_pos =
            pos + ChVector3d(std::sin(angle) * (hthick + radius), std::cos(angle) * (hthick + height / 2), height / 2);
        auto plate_rot = QuatFromAngleZ(angle);

        bool visualize = !partialVisualization || angle > CH_PI_2;
        utils::AddBoxGeometry(body.get(), mat, plate_size, plate_pos, plate_rot, visualize);
    }

    // Add bottom piece
    if (isBoxBase)
        utils::AddBoxGeometry(body.get(), mat, ChVector3d(2 * radius + thickness, height + thickness, thickness),
                              ChVector3d(0, 0, -hthick), QUNIT, true);
    else
        utils::AddCylinderGeometry(body.get(), mat, radius + thickness, thickness, ChVector3d(0, 0, -hthick),
                                   QuatFromAngleX(CH_PI_2));

    // Add top piece
    if (closed) {
        if (isBoxBase)
            utils::AddBoxGeometry(body.get(), mat, ChVector3d(2 * radius + thickness, height + thickness, thickness),
                                  ChVector3d(0, 0, height + hthick), QUNIT, true);
        else
            utils::AddCylinderGeometry(body.get(), mat, radius + thickness, thickness,
                                       ChVector3d(0, 0, height + hthick), QuatFromAngleX(CH_PI_2));
    }

    body->GetCollisionModel()->SetEnvelope(0.2 * thickness);

    system->AddBody(body);
    return body;
}

// -----------------------------------------------------------------------------

bool LoadConvexMesh(const std::string& file_name,
                    ChTriangleMeshConnected& convex_mesh,
                    ChConvexDecompositionHACDv2& convex_shape,
                    const ChVector3d& pos,
                    const ChQuaterniond& rot,
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
                     ChTriangleMeshConnected& convex_mesh,
                     std::vector<std::vector<ChVector3d>>& convex_hulls) {
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
                ChVector3d pos(att.vertices[id * 3 + 0],  //
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
                             ChContactMaterialSharedPtr material,
                             std::shared_ptr<ChTriangleMeshConnected> convex_mesh,
                             ChConvexDecompositionHACDv2& convex_shape,
                             const ChVector3d& pos,
                             const ChQuaterniond& rot,
                             bool use_original_asset,
                             ChVisualMaterialSharedPtr vis_material) {
    ChConvexDecomposition* used_decomposition = &convex_shape;

    int hull_count = used_decomposition->GetHullCount();

    for (int c = 0; c < hull_count; c++) {
        std::vector<ChVector3d> convexhull;
        used_decomposition->GetConvexHullResult(c, convexhull);

        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, convexhull);
        body->AddCollisionShape(cshape, ChFrame<>(pos, rot));

        // Add each convex chunk as a new asset
        if (!use_original_asset) {
            std::stringstream ss;
            ss << convex_mesh->GetFileName() << "_" << c;
            auto trimesh_convex = chrono_types::make_shared<ChTriangleMeshConnected>();
            used_decomposition->GetConvexHullResult(c, *trimesh_convex);

            auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(trimesh_convex);
            trimesh_shape->SetName(ss.str());
            trimesh_shape->AddMaterial(vis_material);
            body->AddVisualShape(trimesh_shape, ChFrame<>(pos, rot));
        }
    }
    // Add the original triangle mesh as asset
    if (use_original_asset) {
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(convex_mesh);
        trimesh_shape->SetName(convex_mesh->GetFileName());
        trimesh_shape->AddMaterial(vis_material);
        body->AddVisualShape(trimesh_shape, ChFrame<>());
    }
}

// -----------------------------------------------------------------------------

void AddConvexCollisionModel(std::shared_ptr<ChBody> body,
                             ChContactMaterialSharedPtr material,
                             std::shared_ptr<ChTriangleMeshConnected> convex_mesh,
                             std::vector<std::vector<ChVector3d>>& convex_hulls,
                             const ChVector3d& pos,
                             const ChQuaterniond& rot,
                             ChVisualMaterialSharedPtr vis_material) {
    for (int c = 0; c < convex_hulls.size(); c++) {
        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, convex_hulls[c]);
        body->AddCollisionShape(cshape, ChFrame<>(pos, rot));
    }

    // Add the original triangle mesh as asset
    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(convex_mesh);
    trimesh_shape->SetName(convex_mesh->GetFileName());
    trimesh_shape->AddMaterial(vis_material);
    body->AddVisualShape(trimesh_shape, ChFrame<>(pos, rot));
}

}  // namespace utils
}  // namespace chrono
