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
// Authors: Alessandro Tasora, Radu Serban, Arman Pazouki
// =============================================================================
//
// Classes for creating easy-to-use bodies that optionally include contact and
// visualization shapes.
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"

namespace chrono {
CH_FACTORY_REGISTER(ChBodyEasySphere)
CH_UPCASTING(ChBodyEasySphere, ChBody)

ChBodyEasySphere::ChBodyEasySphere(double radius,
                                   double density,
                                   bool visualize,
                                   bool collide,
                                   std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(radius, density, visualize, collide, material);
}

ChBodyEasySphere::ChBodyEasySphere(double radius, double density, std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(radius, density, true, true, material);
}

void ChBodyEasySphere::SetupBody(double radius,
                                 double density,
                                 bool visualize,
                                 bool collide,
                                 std::shared_ptr<ChMaterialSurface> material) {
    double mmass = density * ((4.0 / 3.0) * CH_C_PI * pow(radius, 3));
    double inertia = (2.0 / 5.0) * mmass * pow(radius, 2);

    SetMass(mmass);
    SetInertiaXX(ChVector<>(inertia, inertia, inertia));

    if (collide) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeSphere>(material, radius);
        AddCollisionShape(cshape);
        SetCollide(true);
    }
    if (visualize) {
        auto vshape = chrono_types::make_shared<ChVisualShapeSphere>(radius);
        AddVisualShape(vshape);
    }
}

void ChBodyEasySphere::ArchiveOutConstructor(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChBodyEasySphere>();
}

void* ChBodyEasySphere::ArchiveInConstructor(ChArchiveIn& marchive) {
    /*int version =*/marchive.VersionRead<ChBodyEasySphere>();

    ChBodyEasySphere* new_obj = new ChBodyEasySphere();

    return new_obj;
}

// -----------------------------------------------------------------------------
CH_FACTORY_REGISTER(ChBodyEasyEllipsoid)
CH_UPCASTING(ChBodyEasyEllipsoid, ChBody)

ChBodyEasyEllipsoid::ChBodyEasyEllipsoid(ChVector<> axes,
                                         double density,
                                         bool visualize,
                                         bool collide,
                                         std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(axes, density, visualize, collide, material);
}

ChBodyEasyEllipsoid::ChBodyEasyEllipsoid(ChVector<> axes, double density, std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(axes, density, true, true, material);
}

void ChBodyEasyEllipsoid::SetupBody(ChVector<> axes,
                                    double density,
                                    bool visualize,
                                    bool collide,
                                    std::shared_ptr<ChMaterialSurface> material) {
    double mmass = density * ((1 / 6.0) * CH_C_PI * axes.x() * axes.y() * axes.z());
    double inertiax = (1 / 20.0) * mmass * (pow(axes.y(), 2) + pow(axes.z(), 2));
    double inertiay = (1 / 20.0) * mmass * (pow(axes.x(), 2) + pow(axes.z(), 2));
    double inertiaz = (1 / 20.0) * mmass * (pow(axes.x(), 2) + pow(axes.y(), 2));

    SetMass(mmass);
    SetInertiaXX(ChVector<>(inertiax, inertiay, inertiaz));

    if (collide) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeEllipsoid>(material, axes);
        AddCollisionShape(cshape);
        SetCollide(true);
    }
    if (visualize) {
        auto vshape = chrono_types::make_shared<ChVisualShapeEllipsoid>(axes);
        AddVisualShape(vshape);
    }
}

void ChBodyEasyEllipsoid::ArchiveOutConstructor(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChBodyEasyEllipsoid>();
}

void* ChBodyEasyEllipsoid::ArchiveInConstructor(ChArchiveIn& marchive) {
    /*int version =*/marchive.VersionRead<ChBodyEasyEllipsoid>();

    ChBodyEasyEllipsoid* new_obj = new ChBodyEasyEllipsoid();

    return new_obj;
}

// -----------------------------------------------------------------------------
CH_FACTORY_REGISTER(ChBodyEasyCylinder)
CH_UPCASTING(ChBodyEasyCylinder, ChBody)

ChBodyEasyCylinder::ChBodyEasyCylinder(geometry::ChAxis direction,
                                       double radius,
                                       double height,
                                       double density,
                                       bool visualize,
                                       bool collide,
                                       std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(direction, radius, height, density, visualize, collide, material);
}

ChBodyEasyCylinder::ChBodyEasyCylinder(geometry::ChAxis direction,
                                       double radius,
                                       double height,
                                       double density,
                                       std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(direction, radius, height, density, true, true, material);
}

void ChBodyEasyCylinder::SetupBody(geometry::ChAxis direction,
                                   double radius,
                                   double height,
                                   double density,
                                   bool visualize,
                                   bool collide,
                                   std::shared_ptr<ChMaterialSurface> material) {
    double mass = density * (CH_C_PI * pow(radius, 2) * height);
    double I_axis = 0.5 * mass * pow(radius, 2);
    double I_orth = (1 / 12.0) * mass * (3 * pow(radius, 2) + pow(height, 2));
    ChQuaternion<> rot;

    SetMass(mass);

    switch (direction) {
        case geometry::ChAxis::X:
            rot = Q_from_AngY(CH_C_PI_2);
            SetInertiaXX(ChVector<>(I_axis, I_orth, I_orth));
            break;
        case geometry::ChAxis::Y:
            rot = Q_from_AngX(CH_C_PI_2);
            SetInertiaXX(ChVector<>(I_orth, I_axis, I_orth));
            break;
        case geometry::ChAxis::Z:
            rot = QUNIT;
            SetInertiaXX(ChVector<>(I_orth, I_orth, I_axis));
            break;
    }

    if (collide) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeCylinder>(material, radius, height);
        AddCollisionShape(cshape, ChFrame<>(VNULL, rot));
        SetCollide(true);
    }

    if (visualize) {
        auto vshape = chrono_types::make_shared<ChVisualShapeCylinder>(radius, height);
        AddVisualShape(vshape, ChFrame<>(VNULL, rot));
    }
}

void ChBodyEasyCylinder::ArchiveOutConstructor(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChBodyEasyCylinder>();
}

void* ChBodyEasyCylinder::ArchiveInConstructor(ChArchiveIn& marchive) {
    /*int version =*/marchive.VersionRead<ChBodyEasyCylinder>();

    ChBodyEasyCylinder* new_obj = new ChBodyEasyCylinder();

    return new_obj;
}

// -----------------------------------------------------------------------------

CH_FACTORY_REGISTER(ChBodyEasyBox)
CH_UPCASTING(ChBodyEasyBox, ChBody)

ChBodyEasyBox::ChBodyEasyBox(double Xsize,
                             double Ysize,
                             double Zsize,
                             double density,
                             bool visualize,
                             bool collide,
                             std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(Xsize, Ysize, Zsize, density, visualize, collide, material);
}

ChBodyEasyBox::ChBodyEasyBox(double Xsize,
                             double Ysize,
                             double Zsize,
                             double density,
                             std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(Xsize, Ysize, Zsize, density, true, true, material);
}

void ChBodyEasyBox::SetupBody(double Xsize,
                              double Ysize,
                              double Zsize,
                              double density,
                              bool visualize,
                              bool collide,
                              std::shared_ptr<ChMaterialSurface> material) {
    double mmass = density * (Xsize * Ysize * Zsize);

    SetMass(mmass);
    SetInertiaXX(ChVector<>((1.0 / 12.0) * mmass * (pow(Ysize, 2) + pow(Zsize, 2)),
                            (1.0 / 12.0) * mmass * (pow(Xsize, 2) + pow(Zsize, 2)),
                            (1.0 / 12.0) * mmass * (pow(Xsize, 2) + pow(Ysize, 2))));
    if (collide) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeBox>(material, Xsize, Ysize, Zsize);
        AddCollisionShape(cshape);
        SetCollide(true);
    }
    if (visualize) {
        auto vshape = chrono_types::make_shared<ChVisualShapeBox>(Xsize, Ysize, Zsize);
        AddVisualShape(vshape);
    }
}

void ChBodyEasyBox::ArchiveOutConstructor(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChBodyEasyBox>();

    // ChBodyEasy do not hold any variables; only parent classes have.
    // by archiving the ChVariables, ChVisualModel and ChCollisionModel
    // all the properties will be retrieved
}

void* ChBodyEasyBox::ArchiveInConstructor(ChArchiveIn& marchive) {
    /*int version =*/marchive.VersionRead<ChBodyEasyBox>();

    ChBodyEasyBox* new_obj = new ChBodyEasyBox();

    return new_obj;
}

// -----------------------------------------------------------------------------

CH_FACTORY_REGISTER(ChBodyEasyConvexHull)
CH_UPCASTING(ChBodyEasyConvexHull, ChBody)

ChBodyEasyConvexHull::ChBodyEasyConvexHull(std::vector<ChVector<>>& points,
                                           double density,
                                           bool visualize,
                                           bool collide,
                                           std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(points, density, visualize, collide, material);
}

ChBodyEasyConvexHull::ChBodyEasyConvexHull(std::vector<ChVector<>>& points,
                                           double density,
                                           std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(points, density, true, true, material);
}

void ChBodyEasyConvexHull::SetupBody(std::vector<ChVector<>>& points,
                                     double density,
                                     bool visualize,
                                     bool collide,
                                     std::shared_ptr<ChMaterialSurface> material) {
    auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    vshape->SetMutable(false);
    bt_utils::ChConvexHullLibraryWrapper lh;
    lh.ComputeHull(points, *vshape->GetMesh());
    if (visualize) {
        vshape->SetName("chull_mesh_" + std::to_string(GetIdentifier()));
        AddVisualShape(vshape);
    }

    double mass;
    ChVector<> baricenter;
    ChMatrix33<> inertia;
    vshape->GetMesh()->ComputeMassProperties(true, mass, baricenter, inertia);

    // Translate the convex hull baricenter so that body origin is also baricenter
    for (unsigned int i = 0; i < vshape->GetMesh()->getCoordsVertices().size(); ++i)
        vshape->GetMesh()->getCoordsVertices()[i] -= baricenter;

    SetMass(mass * density);
    SetInertia(inertia * density);

    if (collide) {
        assert(material);
        // avoid passing to collision the inner points discarded by convex hull
        // processor, so use mesh vertexes instead of all argument points
        std::vector<ChVector<>> points_reduced;
        points_reduced.resize(vshape->GetMesh()->getCoordsVertices().size());
        for (unsigned int i = 0; i < vshape->GetMesh()->getCoordsVertices().size(); ++i)
            points_reduced[i] = vshape->GetMesh()->getCoordsVertices()[i];

        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, points_reduced);
        AddCollisionShape(cshape);
        SetCollide(true);
    }

    m_mesh = vshape->GetMesh();
}

void ChBodyEasyConvexHull::ArchiveOutConstructor(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChBodyEasyConvexHull>();

    marchive << CHNVP(m_mesh);
}

void* ChBodyEasyConvexHull::ArchiveInConstructor(ChArchiveIn& marchive) {
    /*int version =*/marchive.VersionRead<ChBodyEasyConvexHull>();

    std::shared_ptr<geometry::ChTriangleMeshConnected> mesh;
    marchive >> CHNVP(mesh);

    ChBodyEasyConvexHull* new_obj = new ChBodyEasyConvexHull(mesh);

    return new_obj;
}

// -----------------------------------------------------------------------------

CH_FACTORY_REGISTER(ChBodyEasyConvexHullAuxRef)
CH_UPCASTING(ChBodyEasyConvexHullAuxRef, ChBodyAuxRef)

ChBodyEasyConvexHullAuxRef::ChBodyEasyConvexHullAuxRef(std::vector<ChVector<>>& points,
                                                       double density,
                                                       bool visualize,
                                                       bool collide,
                                                       std::shared_ptr<ChMaterialSurface> material)
    : ChBodyAuxRef() {
    SetupBody(points, density, visualize, collide, material);
}

ChBodyEasyConvexHullAuxRef::ChBodyEasyConvexHullAuxRef(std::vector<ChVector<>>& points,
                                                       double density,
                                                       std::shared_ptr<ChMaterialSurface> material)
    : ChBodyAuxRef() {
    SetupBody(points, density, true, true, material);
}

void ChBodyEasyConvexHullAuxRef::SetupBody(std::vector<ChVector<>>& points,
                                           double density,
                                           bool visualize,
                                           bool collide,
                                           std::shared_ptr<ChMaterialSurface> material) {
    auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    vshape->SetMutable(false);
    bt_utils::ChConvexHullLibraryWrapper lh;
    lh.ComputeHull(points, *vshape->GetMesh());
    if (visualize) {
        vshape->SetName("chull_mesh_" + std::to_string(GetIdentifier()));
        AddVisualShape(vshape);
    }

    double mass;
    ChVector<> baricenter;
    ChMatrix33<> inertia;
    vshape->GetMesh()->ComputeMassProperties(true, mass, baricenter, inertia);

    ChMatrix33<> principal_inertia_csys;
    ChVectorN<double, 3> principal_I;
    inertia.SelfAdjointEigenSolve(principal_inertia_csys, principal_I);
    if (principal_inertia_csys.determinant() < 0)
        principal_inertia_csys.col(0) *= -1;

    SetMass(mass * density);
    ////SetInertia(inertia * density);
    SetInertiaXX(ChVector<>(principal_I) * density);

    // Set the COG coordinates to barycenter, without displacing the REF reference
    SetFrame_COG_to_REF(ChFrame<>(baricenter, principal_inertia_csys));

    if (collide) {
        assert(material);
        // avoid passing to collision the inner points discarded by convex hull
        // processor, so use mesh vertexes instead of all argument points
        std::vector<ChVector<>> points_reduced;
        points_reduced.resize(vshape->GetMesh()->getCoordsVertices().size());
        for (unsigned int i = 0; i < vshape->GetMesh()->getCoordsVertices().size(); ++i)
            points_reduced[i] = vshape->GetMesh()->getCoordsVertices()[i];

        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, points_reduced);
        AddCollisionShape(cshape);
        SetCollide(true);
    }

    m_mesh = vshape->GetMesh();
}

void ChBodyEasyConvexHullAuxRef::ArchiveOutConstructor(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChBodyEasyConvexHullAuxRef>();

    marchive << CHNVP(m_mesh);
}

void* ChBodyEasyConvexHullAuxRef::ArchiveInConstructor(ChArchiveIn& marchive) {
    /*int version =*/marchive.VersionRead<ChBodyEasyConvexHullAuxRef>();

    std::shared_ptr<geometry::ChTriangleMeshConnected> mesh;
    marchive >> CHNVP(mesh);

    ChBodyEasyConvexHullAuxRef* new_obj = new ChBodyEasyConvexHullAuxRef(mesh);

    return new_obj;
}

// -----------------------------------------------------------------------------

CH_FACTORY_REGISTER(ChBodyEasyMesh)
CH_UPCASTING(ChBodyEasyMesh, ChBodyAuxRef)

ChBodyEasyMesh::ChBodyEasyMesh(const std::string& filename,
                               double density,
                               bool compute_mass,
                               bool visualize,
                               bool collide,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept)
    : ChBodyAuxRef() {
    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    SetupBody(trimesh, filename, density, compute_mass, visualize, collide, material, sphere_swept);
}

ChBodyEasyMesh::ChBodyEasyMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,
                               double density,
                               bool compute_mass,
                               bool visualize,
                               bool collide,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept)
    : ChBodyAuxRef() {
    SetupBody(mesh, "EasyMesh", density, compute_mass, visualize, collide, material, sphere_swept);
}

ChBodyEasyMesh::ChBodyEasyMesh(const std::string& filename,
                               double density,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept)
    : ChBodyAuxRef() {
    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    SetupBody(trimesh, filename, density, true, true, true, material, sphere_swept);
}

ChBodyEasyMesh::ChBodyEasyMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,
                               double density,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept)
    : ChBodyAuxRef() {
    SetupBody(mesh, "EasyMesh", density, true, true, true, material, sphere_swept);
}

void ChBodyEasyMesh::SetupBody(std::shared_ptr<geometry::ChTriangleMeshConnected> trimesh,
                               const std::string& name,
                               double density,
                               bool compute_mass,
                               bool visualize,
                               bool collide,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept) {
    if (visualize) {
        auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        vshape->SetMutable(false);
        vshape->SetMesh(trimesh);
        vshape->SetName(name);
        AddVisualShape(vshape);
    }

    if (compute_mass) {
        double mass;
        ChVector<> baricenter;
        ChMatrix33<> inertia;
        trimesh->ComputeMassProperties(true, mass, baricenter, inertia);

        ChMatrix33<> principal_inertia_csys;
        ChVectorN<double, 3> principal_I;
        inertia.SelfAdjointEigenSolve(principal_inertia_csys, principal_I);
        if (principal_inertia_csys.determinant() < 0)
            principal_inertia_csys.col(0) *= -1;

        SetMass(mass * density);
        SetInertiaXX(ChVector<>(principal_I) * density);

        // Set the COG coordinates to barycenter, without displacing the REF reference
        SetFrame_COG_to_REF(ChFrame<>(baricenter, principal_inertia_csys));
    }

    if (collide) {
        assert(material);
        // coll.model is respect to REF c.sys
        auto cshape =
            chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, trimesh, false, false, sphere_swept);
        AddCollisionShape(cshape);
        SetCollide(true);
    }
}

void ChBodyEasyMesh::ArchiveOutConstructor(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChBodyEasyMesh>();

    // ChBodyEasy do not hold any variables; only parent classes have.
    // by archiving the ChVariables, ChVisualModel and ChCollisionModel
    // all the properties will be retrieved
}

void* ChBodyEasyMesh::ArchiveInConstructor(ChArchiveIn& marchive) {
    /*int version =*/marchive.VersionRead<ChBodyEasyMesh>();

    ChBodyEasyMesh* new_obj = new ChBodyEasyMesh();

    return new_obj;
}

// -----------------------------------------------------------------------------

CH_FACTORY_REGISTER(ChBodyEasyClusterOfSpheres)
CH_UPCASTING(ChBodyEasyClusterOfSpheres, ChBody)

ChBodyEasyClusterOfSpheres::ChBodyEasyClusterOfSpheres(std::vector<ChVector<>>& positions,
                                                       std::vector<double>& radii,
                                                       double density,
                                                       bool visualize,
                                                       bool collide,
                                                       std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(positions, radii, density, visualize, collide, material);
}

ChBodyEasyClusterOfSpheres::ChBodyEasyClusterOfSpheres(std::vector<ChVector<>>& positions,
                                                       std::vector<double>& radii,
                                                       double density,
                                                       std::shared_ptr<ChMaterialSurface> material)
    : ChBody() {
    SetupBody(positions, radii, density, true, true, material);
}

void ChBodyEasyClusterOfSpheres::SetupBody(std::vector<ChVector<>>& positions,
                                           std::vector<double>& radii,
                                           double density,
                                           bool visualize,
                                           bool collide,
                                           std::shared_ptr<ChMaterialSurface> material) {
    assert(positions.size() == radii.size());

    double totmass = 0;
    ChMatrix33<> totinertia;
    ChVector<> baricenter = VNULL;
    totinertia.setZero();
    for (unsigned int i = 0; i < positions.size(); ++i) {
        double sphmass = density * ((4.0 / 3.0) * CH_C_PI * pow(radii[i], 3));
        baricenter = (baricenter * totmass + positions[i] * sphmass) / (totmass + sphmass);
        totmass += sphmass;
    }
    for (unsigned int i = 0; i < positions.size(); ++i) {
        double sphmass = density * ((4.0 / 3.0) * CH_C_PI * pow(radii[i], 3));
        double sphinertia = (2.0 / 5.0) * sphmass * pow(radii[i], 2);

        // Huygens-Steiner parallel axis theorem:
        ChVector<> dist = positions[i] - baricenter;
        totinertia(0, 0) += sphinertia + sphmass * (dist.Length2() - dist.x() * dist.x());
        totinertia(1, 1) += sphinertia + sphmass * (dist.Length2() - dist.y() * dist.y());
        totinertia(2, 2) += sphinertia + sphmass * (dist.Length2() - dist.z() * dist.z());
        totinertia(0, 1) += sphmass * (-dist.x() * dist.y());
        totinertia(0, 2) += sphmass * (-dist.x() * dist.z());
        totinertia(1, 2) += sphmass * (-dist.y() * dist.z());
        totinertia(1, 0) = totinertia(0, 1);
        totinertia(2, 0) = totinertia(0, 2);
        totinertia(2, 1) = totinertia(1, 2);
    }

    SetMass(totmass);
    SetInertia(totinertia);

    // Translate the cluster baricenter so that body origin is also baricenter
    std::vector<ChVector<>> offset_positions = positions;
    for (unsigned int i = 0; i < positions.size(); ++i)
        offset_positions[i] -= baricenter;

    if (collide) {
        assert(material);
        auto collision_model = chrono_types::make_shared<ChCollisionModel>();
        for (unsigned int i = 0; i < positions.size(); ++i) {
            auto cshape = chrono_types::make_shared<ChCollisionShapeSphere>(material, radii[i]);
            collision_model->AddShape(cshape, ChFrame<>(offset_positions[i], QUNIT));
        }
        AddCollisionModel(collision_model);
        SetCollide(true);
    }
    if (visualize) {
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        for (unsigned int i = 0; i < positions.size(); ++i) {
            auto vshape = chrono_types::make_shared<ChVisualShapeSphere>(radii[i]);
            vmodel->AddShape(vshape, ChFrame<>(offset_positions[i]));
        }
        AddVisualModel(vmodel);
    }
}

void ChBodyEasyClusterOfSpheres::ArchiveOutConstructor(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChBodyEasyClusterOfSpheres>();

    // ChBodyEasy do not hold any variables; only parent classes have.
    // by archiving the ChVariables, ChVisualModel and ChCollisionModel
    // all the properties will be retrieved
}

void* ChBodyEasyClusterOfSpheres::ArchiveInConstructor(ChArchiveIn& marchive) {
    /*int version =*/marchive.VersionRead<ChBodyEasyClusterOfSpheres>();

    ChBodyEasyClusterOfSpheres* new_obj = new ChBodyEasyClusterOfSpheres();

    return new_obj;
}

}  // end namespace chrono
