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

#include "chrono/physics/ChBodyEasy.h"

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChObjShapeFile.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/collision/ChCollisionUtils.h"

namespace chrono {

ChBodyEasySphere::ChBodyEasySphere(double radius,
                                   double mdensity,
                                   bool visualize,
                                   bool collide,
                                   std::shared_ptr<ChMaterialSurface> material,
                                   std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    double mmass = mdensity * ((4.0 / 3.0) * CH_C_PI * pow(radius, 3));
    double inertia = (2.0 / 5.0) * mmass * pow(radius, 2);

    this->SetDensity((float)mdensity);
    this->SetMass(mmass);
    this->SetInertiaXX(ChVector<>(inertia, inertia, inertia));

    if (collide) {
        assert(material);
        GetCollisionModel()->ClearModel();
        GetCollisionModel()->AddSphere(material, radius);
        GetCollisionModel()->BuildModel();
        SetCollide(true);
    }
    if (visualize) {
        std::shared_ptr<ChSphereShape> vshape(new ChSphereShape());
        vshape->GetSphereGeometry().rad = radius;
        this->AddAsset(vshape);
    }
}

ChBodyEasyEllipsoid::ChBodyEasyEllipsoid(ChVector<> radius,
                                         double mdensity,
                                         bool visualize,
                                         bool collide,
                                         std::shared_ptr<ChMaterialSurface> material,
                                         std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    double mmass = mdensity * ((4.0 / 3.0) * CH_C_PI * radius.x() * radius.y() * radius.z());
    double inertiax = (1.0 / 5.0) * mmass * (pow(radius.y(), 2) + pow(radius.z(), 2));
    double inertiay = (1.0 / 5.0) * mmass * (pow(radius.x(), 2) + pow(radius.z(), 2));
    double inertiaz = (1.0 / 5.0) * mmass * (pow(radius.x(), 2) + pow(radius.y(), 2));

    this->SetDensity((float)mdensity);
    this->SetMass(mmass);
    this->SetInertiaXX(ChVector<>(inertiax, inertiay, inertiaz));

    if (collide) {
        assert(material);
        GetCollisionModel()->ClearModel();
        GetCollisionModel()->AddEllipsoid(material, radius.x(), radius.y(), radius.z());
        GetCollisionModel()->BuildModel();
        SetCollide(true);
    }
    if (visualize) {
        std::shared_ptr<ChEllipsoidShape> vshape(new ChEllipsoidShape());
        vshape->GetEllipsoidGeometry().rad = radius;
        this->AddAsset(vshape);
    }
}

ChBodyEasyCylinder::ChBodyEasyCylinder(double radius,
                                       double height,
                                       double mdensity,
                                       bool visualize,
                                       bool collide,
                                       std::shared_ptr<ChMaterialSurface> material,
                                       std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    double mmass = mdensity * (CH_C_PI * pow(radius, 2) * height);

    this->SetDensity((float)mdensity);
    this->SetMass(mmass);
    this->SetInertiaXX(ChVector<>((1.0 / 12.0) * mmass * (3 * pow(radius, 2) + pow(height, 2)),
                                  0.5 * mmass * pow(radius, 2),
                                  (1.0 / 12.0) * mmass * (3 * pow(radius, 2) + pow(height, 2))));
    if (collide) {
        assert(material);
        GetCollisionModel()->ClearModel();
        GetCollisionModel()->AddCylinder(material, radius, radius, height * 0.5);
        GetCollisionModel()->BuildModel();
        SetCollide(true);
    }
    if (visualize) {
        auto vshape = chrono_types::make_shared<ChCylinderShape>();
        vshape->GetCylinderGeometry().p1 = ChVector<>(0, -height * 0.5, 0);
        vshape->GetCylinderGeometry().p2 = ChVector<>(0, height * 0.5, 0);
        vshape->GetCylinderGeometry().rad = radius;
        this->AddAsset(vshape);
    }
}

ChBodyEasyBox::ChBodyEasyBox(double Xsize,
                             double Ysize,
                             double Zsize,
                             double mdensity,
                             bool visualize,
                             bool collide,
                             std::shared_ptr<ChMaterialSurface> material,
                             std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    double mmass = mdensity * (Xsize * Ysize * Zsize);

    this->SetDensity((float)mdensity);
    this->SetMass(mmass);
    this->SetInertiaXX(ChVector<>((1.0 / 12.0) * mmass * (pow(Ysize, 2) + pow(Zsize, 2)),
                                  (1.0 / 12.0) * mmass * (pow(Xsize, 2) + pow(Zsize, 2)),
                                  (1.0 / 12.0) * mmass * (pow(Xsize, 2) + pow(Ysize, 2))));
    if (collide) {
        assert(material);
        GetCollisionModel()->ClearModel();
        GetCollisionModel()->AddBox(material, Xsize * 0.5, Ysize * 0.5, Zsize * 0.5);
        GetCollisionModel()->BuildModel();
        SetCollide(true);
    }
    if (visualize) {
        auto vshape = chrono_types::make_shared<ChBoxShape>();
        vshape->GetBoxGeometry().SetLengths(ChVector<>(Xsize, Ysize, Zsize));
        this->AddAsset(vshape);
    }
}

ChBodyEasyConvexHull::ChBodyEasyConvexHull(std::vector<ChVector<>>& points,
                                           double mdensity,
                                           bool visualize,
                                           bool collide,
                                           std::shared_ptr<ChMaterialSurface> material,
                                           std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    auto vshape = chrono_types::make_shared<ChTriangleMeshShape>();
    collision::ChConvexHullLibraryWrapper lh;
    lh.ComputeHull(points, *vshape->GetMesh());
    if (visualize) {
        vshape->SetName("chull_mesh_" + std::to_string(GetIdentifier()));
        this->AddAsset(vshape);
    }

    double mass;
    ChVector<> baricenter;
    ChMatrix33<> inertia;
    vshape->GetMesh()->ComputeMassProperties(true, mass, baricenter, inertia);

    // Translate the convex hull baricenter so that body origin is also baricenter
    for (unsigned int i = 0; i < vshape->GetMesh()->getCoordsVertices().size(); ++i)
        vshape->GetMesh()->getCoordsVertices()[i] -= baricenter;

    this->SetDensity((float)mdensity);
    this->SetMass(mass * mdensity);
    this->SetInertia(inertia * mdensity);

    if (collide) {
        assert(material);
        // avoid passing to collision the inner points discarded by convex hull
        // processor, so use mesh vertexes instead of all argument points
        std::vector<ChVector<>> points_reduced;
        points_reduced.resize(vshape->GetMesh()->getCoordsVertices().size());
        for (unsigned int i = 0; i < vshape->GetMesh()->getCoordsVertices().size(); ++i)
            points_reduced[i] = vshape->GetMesh()->getCoordsVertices()[i];

        GetCollisionModel()->ClearModel();
        GetCollisionModel()->AddConvexHull(material, points_reduced);
        GetCollisionModel()->BuildModel();
        SetCollide(true);
    }

    m_mesh = vshape->GetMesh();
}

ChBodyEasyConvexHullAuxRef::ChBodyEasyConvexHullAuxRef(std::vector<ChVector<>>& points,
                                                       double mdensity,
                                                       bool visualize,
                                                       bool collide,
                                                       std::shared_ptr<ChMaterialSurface> material,
                                                       std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBodyAuxRef(collision_model) {
    auto vshape = chrono_types::make_shared<ChTriangleMeshShape>();
    collision::ChConvexHullLibraryWrapper lh;
    lh.ComputeHull(points, *vshape->GetMesh());
    if (visualize) {
        vshape->SetName("chull_mesh_" + std::to_string(GetIdentifier()));
        this->AddAsset(vshape);  // assets are respect to REF c.sys
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

    SetDensity((float)mdensity);
    SetMass(mass * mdensity);
    // this->SetInertia(inertia * mdensity);
    SetInertiaXX(ChVector<>(principal_I) * mdensity);

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

        GetCollisionModel()->ClearModel();
        GetCollisionModel()->AddConvexHull(material, points_reduced);
        GetCollisionModel()->BuildModel();
        SetCollide(true);
    }

    m_mesh = vshape->GetMesh();
}

ChBodyEasyMesh::ChBodyEasyMesh(const std::string filename,
                               double mdensity,
                               bool compute_mass,
                               bool visualize,
                               bool collide,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept,
                               std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBodyAuxRef(collision_model) {
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(filename, true, true);
    SetupBody(trimesh, filename, mdensity, compute_mass, visualize, collide, material, sphere_swept, collision_model);
}

ChBodyEasyMesh::ChBodyEasyMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,
                               double mdensity,
                               bool compute_mass,
                               bool visualize,
                               bool collide,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept,
                               std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBodyAuxRef(collision_model) {
    SetupBody(mesh, "EasyMesh", mdensity, compute_mass, visualize, collide, material, sphere_swept, collision_model);
}

void ChBodyEasyMesh::SetupBody(std::shared_ptr<geometry::ChTriangleMeshConnected> trimesh,
                               const std::string& name,
                               double mdensity,
                               bool compute_mass,
                               bool visualize,
                               bool collide,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept,
                               std::shared_ptr<collision::ChCollisionModel> collision_model) {

    if (visualize) {
        auto vshape = chrono_types::make_shared<ChTriangleMeshShape>();
        vshape->SetMesh(trimesh);
        vshape->SetName(name);
        AddAsset(vshape);  // assets are respect to REF c.sys
    }

    this->SetDensity((float)mdensity);
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

        SetMass(mass * mdensity);
        SetInertiaXX(ChVector<>(principal_I) * mdensity);

        // Set the COG coordinates to barycenter, without displacing the REF reference
        SetFrame_COG_to_REF(ChFrame<>(baricenter, principal_inertia_csys));
    }

    if (collide) {
        assert(material);
        // coll.model is respect to REF c.sys
        GetCollisionModel()->ClearModel();
        GetCollisionModel()->AddTriangleMesh(material, trimesh, false, false, VNULL, ChMatrix33<>(1), sphere_swept);
        GetCollisionModel()->BuildModel();
        SetCollide(true);
    }
}

ChBodyEasyClusterOfSpheres::ChBodyEasyClusterOfSpheres(std::vector<ChVector<>>& positions,
                                                       std::vector<double>& radii,
                                                       double mdensity,
                                                       bool visualize,
                                                       bool collide,
                                                       std::shared_ptr<ChMaterialSurface> material,
                                                       std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    assert(positions.size() == radii.size());

    double totmass = 0;
    ChMatrix33<> totinertia;
    ChVector<> baricenter = VNULL;
    totinertia.setZero();
    for (unsigned int i = 0; i < positions.size(); ++i) {
        double sphmass = mdensity * ((4.0 / 3.0) * CH_C_PI * pow(radii[i], 3));
        baricenter = (baricenter * totmass + positions[i] * sphmass) / (totmass + sphmass);
        totmass += sphmass;
    }
    for (unsigned int i = 0; i < positions.size(); ++i) {
        double sphmass = mdensity * ((4.0 / 3.0) * CH_C_PI * pow(radii[i], 3));
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

    this->SetDensity((float)mdensity);
    this->SetMass(totmass);
    this->SetInertia(totinertia);

    // Translate the cluster baricenter so that body origin is also baricenter
    std::vector<ChVector<>> offset_positions = positions;
    for (unsigned int i = 0; i < positions.size(); ++i)
        offset_positions[i] -= baricenter;

    if (collide) {
        assert(material);
        GetCollisionModel()->ClearModel();
        for (unsigned int i = 0; i < positions.size(); ++i) {
            GetCollisionModel()->AddSphere(material, radii[i], offset_positions[i]);  // radius, radius, height on y
        }
        GetCollisionModel()->BuildModel();
        SetCollide(true);
    }
    if (visualize) {
        for (unsigned int i = 0; i < positions.size(); ++i) {
            auto vshape = chrono_types::make_shared<ChSphereShape>();
            vshape->GetSphereGeometry().rad = radii[i];
            vshape->GetSphereGeometry().center = offset_positions[i];
            this->AddAsset(vshape);
        }
    }
}

}  // end namespace chrono
