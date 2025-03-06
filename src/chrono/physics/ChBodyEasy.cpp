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
#include "chrono/physics/ChInertiaUtils.h"

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
                                   bool create_visualization,
                                   bool create_collision,
                                   std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(radius, density, create_visualization, create_collision, material);
}

ChBodyEasySphere::ChBodyEasySphere(double radius, double density, std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(radius, density, true, true, material);
}

void ChBodyEasySphere::SetupBody(double radius,
                                 double density,
                                 bool create_visualization,
                                 bool create_collision,
                                 std::shared_ptr<ChContactMaterial> material) {
    double mmass = density * (CH_4_3 * CH_PI * std::pow(radius, 3));
    double inertia = (2.0 / 5.0) * mmass * std::pow(radius, 2);

    SetMass(mmass);
    SetInertiaXX(ChVector3d(inertia, inertia, inertia));

    if (create_collision) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeSphere>(material, radius);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }
    if (create_visualization) {
        auto vshape = chrono_types::make_shared<ChVisualShapeSphere>(radius);
        AddVisualShape(vshape);
    }
}

void ChBodyEasySphere::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChBodyEasySphere>();
}

void* ChBodyEasySphere::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChBodyEasySphere>();

    ChBodyEasySphere* new_obj = new ChBodyEasySphere();

    return new_obj;
}

// -----------------------------------------------------------------------------
CH_FACTORY_REGISTER(ChBodyEasyEllipsoid)
CH_UPCASTING(ChBodyEasyEllipsoid, ChBody)

ChBodyEasyEllipsoid::ChBodyEasyEllipsoid(ChVector3d axes,
                                         double density,
                                         bool create_visualization,
                                         bool create_collision,
                                         std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(axes, density, create_visualization, create_collision, material);
}

ChBodyEasyEllipsoid::ChBodyEasyEllipsoid(ChVector3d axes, double density, std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(axes, density, true, true, material);
}

void ChBodyEasyEllipsoid::SetupBody(ChVector3d axes,
                                    double density,
                                    bool create_visualization,
                                    bool create_collision,
                                    std::shared_ptr<ChContactMaterial> material) {
    double mmass = density * ((1 / 6.0) * CH_PI * axes.x() * axes.y() * axes.z());
    double inertiax = (1 / 20.0) * mmass * (std::pow(axes.y(), 2) + std::pow(axes.z(), 2));
    double inertiay = (1 / 20.0) * mmass * (std::pow(axes.x(), 2) + std::pow(axes.z(), 2));
    double inertiaz = (1 / 20.0) * mmass * (std::pow(axes.x(), 2) + std::pow(axes.y(), 2));

    SetMass(mmass);
    SetInertiaXX(ChVector3d(inertiax, inertiay, inertiaz));

    if (create_collision) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeEllipsoid>(material, axes);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }
    if (create_visualization) {
        auto vshape = chrono_types::make_shared<ChVisualShapeEllipsoid>(axes);
        AddVisualShape(vshape);
    }
}

void ChBodyEasyEllipsoid::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChBodyEasyEllipsoid>();
}

void* ChBodyEasyEllipsoid::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChBodyEasyEllipsoid>();

    ChBodyEasyEllipsoid* new_obj = new ChBodyEasyEllipsoid();

    return new_obj;
}

// -----------------------------------------------------------------------------
CH_FACTORY_REGISTER(ChBodyEasyCylinder)
CH_UPCASTING(ChBodyEasyCylinder, ChBody)

ChBodyEasyCylinder::ChBodyEasyCylinder(ChAxis direction,
                                       double radius,
                                       double height,
                                       double density,
                                       bool create_visualization,
                                       bool create_collision,
                                       std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(direction, radius, height, density, create_visualization, create_collision, material);
}

ChBodyEasyCylinder::ChBodyEasyCylinder(ChAxis direction,
                                       double radius,
                                       double height,
                                       double density,
                                       std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(direction, radius, height, density, true, true, material);
}

void ChBodyEasyCylinder::SetupBody(ChAxis direction,
                                   double radius,
                                   double height,
                                   double density,
                                   bool create_visualization,
                                   bool create_collision,
                                   std::shared_ptr<ChContactMaterial> material) {
    double mass = density * (CH_PI * std::pow(radius, 2) * height);
    double I_axis = 0.5 * mass * std::pow(radius, 2);
    double I_orth = (1 / 12.0) * mass * (3 * std::pow(radius, 2) + std::pow(height, 2));
    ChQuaternion<> rot;

    SetMass(mass);

    switch (direction) {
        case ChAxis::X:
            rot = QuatFromAngleY(CH_PI_2);
            SetInertiaXX(ChVector3d(I_axis, I_orth, I_orth));
            break;
        case ChAxis::Y:
            rot = QuatFromAngleX(CH_PI_2);
            SetInertiaXX(ChVector3d(I_orth, I_axis, I_orth));
            break;
        case ChAxis::Z:
            rot = QUNIT;
            SetInertiaXX(ChVector3d(I_orth, I_orth, I_axis));
            break;
    }

    if (create_collision) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeCylinder>(material, radius, height);
        AddCollisionShape(cshape, ChFrame<>(VNULL, rot));
        EnableCollision(true);
    }

    if (create_visualization) {
        auto vshape = chrono_types::make_shared<ChVisualShapeCylinder>(radius, height);
        AddVisualShape(vshape, ChFrame<>(VNULL, rot));
    }
}

void ChBodyEasyCylinder::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChBodyEasyCylinder>();
}

void* ChBodyEasyCylinder::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChBodyEasyCylinder>();

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
                             bool create_visualization,
                             bool create_collision,
                             std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(Xsize, Ysize, Zsize, density, create_visualization, create_collision, material);
}

ChBodyEasyBox::ChBodyEasyBox(double Xsize,
                             double Ysize,
                             double Zsize,
                             double density,
                             std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(Xsize, Ysize, Zsize, density, true, true, material);
}

void ChBodyEasyBox::SetupBody(double Xsize,
                              double Ysize,
                              double Zsize,
                              double density,
                              bool create_visualization,
                              bool create_collision,
                              std::shared_ptr<ChContactMaterial> material) {
    double mmass = density * (Xsize * Ysize * Zsize);

    SetMass(mmass);
    SetInertiaXX(ChVector3d((1.0 / 12.0) * mmass * (std::pow(Ysize, 2) + std::pow(Zsize, 2)),
                            (1.0 / 12.0) * mmass * (std::pow(Xsize, 2) + std::pow(Zsize, 2)),
                            (1.0 / 12.0) * mmass * (std::pow(Xsize, 2) + std::pow(Ysize, 2))));
    if (create_collision) {
        assert(material);
        auto cshape = chrono_types::make_shared<ChCollisionShapeBox>(material, Xsize, Ysize, Zsize);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }
    if (create_visualization) {
        auto vshape = chrono_types::make_shared<ChVisualShapeBox>(Xsize, Ysize, Zsize);
        AddVisualShape(vshape);
    }
}

void ChBodyEasyBox::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChBodyEasyBox>();

    // ChBodyEasy do not hold any variables; only parent classes have.
    // by archiving the ChVariables, ChVisualModel and ChCollisionModel
    // all the properties will be retrieved
}

void* ChBodyEasyBox::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChBodyEasyBox>();

    ChBodyEasyBox* new_obj = new ChBodyEasyBox();

    return new_obj;
}

// -----------------------------------------------------------------------------

CH_FACTORY_REGISTER(ChBodyEasyConvexHull)
CH_UPCASTING(ChBodyEasyConvexHull, ChBody)

ChBodyEasyConvexHull::ChBodyEasyConvexHull(std::vector<ChVector3d>& points,
                                           double density,
                                           bool create_visualization,
                                           bool create_collision,
                                           std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(points, density, create_visualization, create_collision, material);
}

ChBodyEasyConvexHull::ChBodyEasyConvexHull(std::vector<ChVector3d>& points,
                                           double density,
                                           std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(points, density, true, true, material);
}

void ChBodyEasyConvexHull::SetupBody(std::vector<ChVector3d>& points,
                                     double density,
                                     bool create_visualization,
                                     bool create_collision,
                                     std::shared_ptr<ChContactMaterial> material) {
    auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    vshape->SetMutable(false);
    bt_utils::ChConvexHullLibraryWrapper lh;
    lh.ComputeHull(points, *vshape->GetMesh());
    if (create_visualization) {
        vshape->SetName("chull_mesh_" + std::to_string(GetIdentifier()));
        AddVisualShape(vshape);
    }

    double mass;
    ChVector3d baricenter;
    ChMatrix33<> inertia;
    vshape->GetMesh()->ComputeMassProperties(true, mass, baricenter, inertia);

    // Translate the convex hull baricenter so that body origin is also baricenter
    for (unsigned int i = 0; i < vshape->GetMesh()->GetCoordsVertices().size(); ++i)
        vshape->GetMesh()->GetCoordsVertices()[i] -= baricenter;

    SetMass(mass * density);
    SetInertia(inertia * density);

    if (create_collision) {
        assert(material);
        // avoid passing to collision the inner points discarded by convex hull
        // processor, so use mesh vertexes instead of all argument points
        std::vector<ChVector3d> points_reduced;
        points_reduced.resize(vshape->GetMesh()->GetCoordsVertices().size());
        for (unsigned int i = 0; i < vshape->GetMesh()->GetCoordsVertices().size(); ++i)
            points_reduced[i] = vshape->GetMesh()->GetCoordsVertices()[i];

        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, points_reduced);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }

    m_mesh = vshape->GetMesh();
}

void ChBodyEasyConvexHull::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChBodyEasyConvexHull>();

    archive_out << CHNVP(m_mesh);
}

void* ChBodyEasyConvexHull::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChBodyEasyConvexHull>();

    std::shared_ptr<ChTriangleMeshConnected> mesh;
    archive_in >> CHNVP(mesh);

    ChBodyEasyConvexHull* new_obj = new ChBodyEasyConvexHull(mesh);

    return new_obj;
}

// -----------------------------------------------------------------------------

CH_FACTORY_REGISTER(ChBodyEasyConvexHullAuxRef)
CH_UPCASTING(ChBodyEasyConvexHullAuxRef, ChBodyAuxRef)

ChBodyEasyConvexHullAuxRef::ChBodyEasyConvexHullAuxRef(std::vector<ChVector3d>& points,
                                                       double density,
                                                       bool create_visualization,
                                                       bool create_collision,
                                                       std::shared_ptr<ChContactMaterial> material)
    : ChBodyAuxRef() {
    SetupBody(points, density, create_visualization, create_collision, material);
}

ChBodyEasyConvexHullAuxRef::ChBodyEasyConvexHullAuxRef(std::vector<ChVector3d>& points,
                                                       double density,
                                                       std::shared_ptr<ChContactMaterial> material)
    : ChBodyAuxRef() {
    SetupBody(points, density, true, true, material);
}

void ChBodyEasyConvexHullAuxRef::SetupBody(std::vector<ChVector3d>& points,
                                           double density,
                                           bool create_visualization,
                                           bool create_collision,
                                           std::shared_ptr<ChContactMaterial> material) {
    auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    vshape->SetMutable(false);
    bt_utils::ChConvexHullLibraryWrapper lh;
    lh.ComputeHull(points, *vshape->GetMesh());
    if (create_visualization) {
        vshape->SetName("chull_mesh_" + std::to_string(GetIdentifier()));
        AddVisualShape(vshape);
    }

    double mass;
    ChVector3d baricenter;
    ChMatrix33<> inertia;
    vshape->GetMesh()->ComputeMassProperties(true, mass, baricenter, inertia);

    ChMatrix33<> principal_inertia_csys;
    ChVectorN<double, 3> principal_I;
    inertia.SelfAdjointEigenSolve(principal_inertia_csys, principal_I);
    if (principal_inertia_csys.determinant() < 0)
        principal_inertia_csys.col(0) *= -1;

    SetMass(mass * density);
    ////SetInertia(inertia * density);
    SetInertiaXX(ChVector3d(principal_I) * density);

    // Set the COG coordinates to barycenter, without displacing the REF reference
    SetFrameCOMToRef(ChFrame<>(baricenter, principal_inertia_csys));

    if (create_collision) {
        assert(material);
        // avoid passing to collision the inner points discarded by convex hull
        // processor, so use mesh vertexes instead of all argument points
        std::vector<ChVector3d> points_reduced;
        points_reduced.resize(vshape->GetMesh()->GetCoordsVertices().size());
        for (unsigned int i = 0; i < vshape->GetMesh()->GetCoordsVertices().size(); ++i)
            points_reduced[i] = vshape->GetMesh()->GetCoordsVertices()[i];

        auto cshape = chrono_types::make_shared<ChCollisionShapeConvexHull>(material, points_reduced);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }

    m_mesh = vshape->GetMesh();
}

void ChBodyEasyConvexHullAuxRef::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChBodyEasyConvexHullAuxRef>();

    archive_out << CHNVP(m_mesh);
}

void* ChBodyEasyConvexHullAuxRef::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChBodyEasyConvexHullAuxRef>();

    std::shared_ptr<ChTriangleMeshConnected> mesh;
    archive_in >> CHNVP(mesh);

    ChBodyEasyConvexHullAuxRef* new_obj = new ChBodyEasyConvexHullAuxRef(mesh);

    return new_obj;
}

// -----------------------------------------------------------------------------

CH_FACTORY_REGISTER(ChBodyEasyMesh)
CH_UPCASTING(ChBodyEasyMesh, ChBodyAuxRef)

ChBodyEasyMesh::ChBodyEasyMesh(const std::string& filename,
                               double density,
                               bool compute_mass,
                               bool create_visualization,
                               bool create_collision,
                               std::shared_ptr<ChContactMaterial> material,
                               double sphere_swept)
    : ChBodyAuxRef() {
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    SetupBody(trimesh, filename, density, compute_mass, create_visualization, create_collision, material, sphere_swept);
}

ChBodyEasyMesh::ChBodyEasyMesh(std::shared_ptr<ChTriangleMeshConnected> mesh,
                               double density,
                               bool compute_mass,
                               bool create_visualization,
                               bool create_collision,
                               std::shared_ptr<ChContactMaterial> material,
                               double sphere_swept)
    : ChBodyAuxRef() {
    SetupBody(mesh, "EasyMesh", density, compute_mass, create_visualization, create_collision, material, sphere_swept);
}

ChBodyEasyMesh::ChBodyEasyMesh(const std::string& filename,
                               double density,
                               std::shared_ptr<ChContactMaterial> material,
                               double sphere_swept)
    : ChBodyAuxRef() {
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    SetupBody(trimesh, filename, density, true, true, true, material, sphere_swept);
}

ChBodyEasyMesh::ChBodyEasyMesh(std::shared_ptr<ChTriangleMeshConnected> mesh,
                               double density,
                               std::shared_ptr<ChContactMaterial> material,
                               double sphere_swept)
    : ChBodyAuxRef() {
    SetupBody(mesh, "EasyMesh", density, true, true, true, material, sphere_swept);
}

void ChBodyEasyMesh::SetupBody(std::shared_ptr<ChTriangleMeshConnected> trimesh,
                               const std::string& name,
                               double density,
                               bool compute_mass,
                               bool create_visualization,
                               bool create_collision,
                               std::shared_ptr<ChContactMaterial> material,
                               double sphere_swept) {
    if (create_visualization) {
        auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        vshape->SetMutable(false);
        vshape->SetMesh(trimesh);
        vshape->SetName(name);
        AddVisualShape(vshape);
    }

    if (compute_mass) {
        double mass;
        ChVector3d cog;
        ChMatrix33<> inertia;
        trimesh->ComputeMassProperties(true, mass, cog, inertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_I;
        ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

        SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
        SetMass(mass * density);
        SetInertiaXX(density * principal_I);
    }

    if (create_collision) {
        assert(material);
        // coll.model is respect to REF c.sys
        auto cshape =
            chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, trimesh, false, false, sphere_swept);
        AddCollisionShape(cshape);
        EnableCollision(true);
    }
}

void ChBodyEasyMesh::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChBodyEasyMesh>();

    // ChBodyEasy do not hold any variables; only parent classes have.
    // by archiving the ChVariables, ChVisualModel and ChCollisionModel
    // all the properties will be retrieved
}

void* ChBodyEasyMesh::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChBodyEasyMesh>();

    ChBodyEasyMesh* new_obj = new ChBodyEasyMesh();

    return new_obj;
}

// -----------------------------------------------------------------------------

CH_FACTORY_REGISTER(ChBodyEasyClusterOfSpheres)
CH_UPCASTING(ChBodyEasyClusterOfSpheres, ChBody)

ChBodyEasyClusterOfSpheres::ChBodyEasyClusterOfSpheres(std::vector<ChVector3d>& positions,
                                                       std::vector<double>& radii,
                                                       double density,
                                                       bool create_visualization,
                                                       bool create_collision,
                                                       std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(positions, radii, density, create_visualization, create_collision, material);
}

ChBodyEasyClusterOfSpheres::ChBodyEasyClusterOfSpheres(std::vector<ChVector3d>& positions,
                                                       std::vector<double>& radii,
                                                       double density,
                                                       std::shared_ptr<ChContactMaterial> material)
    : ChBody() {
    SetupBody(positions, radii, density, true, true, material);
}

void ChBodyEasyClusterOfSpheres::SetupBody(std::vector<ChVector3d>& positions,
                                           std::vector<double>& radii,
                                           double density,
                                           bool create_visualization,
                                           bool create_collision,
                                           std::shared_ptr<ChContactMaterial> material) {
    assert(positions.size() == radii.size());

    double totmass = 0;
    ChMatrix33<> totinertia;
    ChVector3d baricenter = VNULL;
    totinertia.setZero();
    for (unsigned int i = 0; i < positions.size(); ++i) {
        double sphmass = density * (CH_4_3 * CH_PI * std::pow(radii[i], 3));
        baricenter = (baricenter * totmass + positions[i] * sphmass) / (totmass + sphmass);
        totmass += sphmass;
    }
    for (unsigned int i = 0; i < positions.size(); ++i) {
        double sphmass = density * (CH_4_3 * CH_PI * std::pow(radii[i], 3));
        double sphinertia = (2.0 / 5.0) * sphmass * std::pow(radii[i], 2);

        // Huygens-Steiner parallel axis theorem:
        ChVector3d dist = positions[i] - baricenter;
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
    std::vector<ChVector3d> offset_positions = positions;
    for (unsigned int i = 0; i < positions.size(); ++i)
        offset_positions[i] -= baricenter;

    if (create_collision) {
        assert(material);
        auto model = chrono_types::make_shared<ChCollisionModel>();
        for (unsigned int i = 0; i < positions.size(); ++i) {
            auto cshape = chrono_types::make_shared<ChCollisionShapeSphere>(material, radii[i]);
            model->AddShape(cshape, ChFrame<>(offset_positions[i], QUNIT));
        }
        AddCollisionModel(model);
        EnableCollision(true);
    }
    if (create_visualization) {
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        for (unsigned int i = 0; i < positions.size(); ++i) {
            auto vshape = chrono_types::make_shared<ChVisualShapeSphere>(radii[i]);
            vmodel->AddShape(vshape, ChFrame<>(offset_positions[i]));
        }
        AddVisualModel(vmodel);
    }
}

void ChBodyEasyClusterOfSpheres::ArchiveOutConstructor(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChBodyEasyClusterOfSpheres>();

    // ChBodyEasy do not hold any variables; only parent classes have.
    // by archiving the ChVariables, ChVisualModel and ChCollisionModel
    // all the properties will be retrieved
}

void* ChBodyEasyClusterOfSpheres::ArchiveInConstructor(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChBodyEasyClusterOfSpheres>();

    ChBodyEasyClusterOfSpheres* new_obj = new ChBodyEasyClusterOfSpheres();

    return new_obj;
}

}  // end namespace chrono
