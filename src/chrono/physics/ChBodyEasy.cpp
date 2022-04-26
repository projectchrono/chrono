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

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChObjShapeFile.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/collision/ChCollisionUtilsBullet.h"

namespace chrono {

ChBodyEasySphere::ChBodyEasySphere(double radius,
                                   double density,
                                   bool visualize,
                                   bool collide,
                                   std::shared_ptr<ChMaterialSurface> material,
                                   std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    SetupBody(radius, density, visualize, collide, material);
}

ChBodyEasySphere::ChBodyEasySphere(double radius,
                                   double density,
                                   std::shared_ptr<ChMaterialSurface> material,
                                   collision::ChCollisionSystemType collision_type)
    : ChBody(collision_type) {
    SetupBody(radius, density, true, true, material);
}

void ChBodyEasySphere::SetupBody(double radius,
                                 double density,
                                 bool visualize,
                                 bool collide,
                                 std::shared_ptr<ChMaterialSurface> material) {
    double mmass = density * ((4.0 / 3.0) * CH_C_PI * pow(radius, 3));
    double inertia = (2.0 / 5.0) * mmass * pow(radius, 2);

    this->SetDensity((float)density);
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
        auto vshape = chrono_types::make_shared<ChSphereShape>();
        vshape->GetSphereGeometry().rad = radius;
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        vmodel->AddShape(vshape);
        this->AddVisualModel(vmodel);
    }
}

// -----------------------------------------------------------------------------

ChBodyEasyEllipsoid::ChBodyEasyEllipsoid(ChVector<> radius,
                                         double density,
                                         bool visualize,
                                         bool collide,
                                         std::shared_ptr<ChMaterialSurface> material,
                                         std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    SetupBody(radius, density, visualize, collide, material);
}

ChBodyEasyEllipsoid::ChBodyEasyEllipsoid(ChVector<> radius,
                                         double density,
                                         std::shared_ptr<ChMaterialSurface> material,
                                         collision::ChCollisionSystemType collision_type)
    : ChBody(collision_type) {
    SetupBody(radius, density, true, true, material);
}

void ChBodyEasyEllipsoid::SetupBody(ChVector<> radius,
                                    double density,
                                    bool visualize,
                                    bool collide,
                                    std::shared_ptr<ChMaterialSurface> material) {
    double mmass = density * ((4.0 / 3.0) * CH_C_PI * radius.x() * radius.y() * radius.z());
    double inertiax = (1.0 / 5.0) * mmass * (pow(radius.y(), 2) + pow(radius.z(), 2));
    double inertiay = (1.0 / 5.0) * mmass * (pow(radius.x(), 2) + pow(radius.z(), 2));
    double inertiaz = (1.0 / 5.0) * mmass * (pow(radius.x(), 2) + pow(radius.y(), 2));

    this->SetDensity((float)density);
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
        auto vshape = chrono_types::make_shared<ChEllipsoidShape>();
        vshape->GetEllipsoidGeometry().rad = radius;
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        vmodel->AddShape(vshape);
        this->AddVisualModel(vmodel);
    }
}

// -----------------------------------------------------------------------------

ChBodyEasyCylinder::ChBodyEasyCylinder(double radius,
                                       double height,
                                       double density,
                                       bool visualize,
                                       bool collide,
                                       std::shared_ptr<ChMaterialSurface> material,
                                       std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    SetupBody(radius, height, density, visualize, collide, material);
}

ChBodyEasyCylinder::ChBodyEasyCylinder(double radius,
                                       double height,
                                       double density,
                                       std::shared_ptr<ChMaterialSurface> material,
                                       collision::ChCollisionSystemType collision_type)
    : ChBody(collision_type) {
    SetupBody(radius, height, density, true, true, material);
}

void ChBodyEasyCylinder::SetupBody(double radius,
                                   double height,
                                   double density,
                                   bool visualize,
                                   bool collide,
                                   std::shared_ptr<ChMaterialSurface> material) {
    double mmass = density * (CH_C_PI * pow(radius, 2) * height);

    this->SetDensity((float)density);
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
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        vmodel->AddShape(vshape);
        this->AddVisualModel(vmodel);
    }
}

// -----------------------------------------------------------------------------

ChBodyEasyBox::ChBodyEasyBox(double Xsize,
                             double Ysize,
                             double Zsize,
                             double density,
                             bool visualize,
                             bool collide,
                             std::shared_ptr<ChMaterialSurface> material,
                             std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    SetupBody(Xsize, Ysize, Zsize, density, visualize, collide, material);
}

ChBodyEasyBox::ChBodyEasyBox(double Xsize,
                             double Ysize,
                             double Zsize,
                             double density,
                             std::shared_ptr<ChMaterialSurface> material,
                             collision::ChCollisionSystemType collision_type)
    : ChBody(collision_type) {
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

    this->SetDensity((float)density);
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
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        vmodel->AddShape(vshape);
        this->AddVisualModel(vmodel);
    }
}

// -----------------------------------------------------------------------------

ChBodyEasyConvexHull::ChBodyEasyConvexHull(std::vector<ChVector<>>& points,
                                           double density,
                                           bool visualize,
                                           bool collide,
                                           std::shared_ptr<ChMaterialSurface> material,
                                           std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    SetupBody(points, density, visualize, collide, material);
}

ChBodyEasyConvexHull::ChBodyEasyConvexHull(std::vector<ChVector<>>& points,
                                           double density,
                                           std::shared_ptr<ChMaterialSurface> material,
                                           collision::ChCollisionSystemType collision_type)
    : ChBody(collision_type) {
    SetupBody(points, density, true, true, material);
}

void ChBodyEasyConvexHull::SetupBody(std::vector<ChVector<>>& points,
                                     double density,
                                     bool visualize,
                                     bool collide,
                                     std::shared_ptr<ChMaterialSurface> material) {
    auto vshape = chrono_types::make_shared<ChTriangleMeshShape>();
    collision::bt_utils::ChConvexHullLibraryWrapper lh;
    lh.ComputeHull(points, *vshape->GetMesh());
    if (visualize) {
        vshape->SetName("chull_mesh_" + std::to_string(GetIdentifier()));
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        vmodel->AddShape(vshape);
        this->AddVisualModel(vmodel);
    }

    double mass;
    ChVector<> baricenter;
    ChMatrix33<> inertia;
    vshape->GetMesh()->ComputeMassProperties(true, mass, baricenter, inertia);

    // Translate the convex hull baricenter so that body origin is also baricenter
    for (unsigned int i = 0; i < vshape->GetMesh()->getCoordsVertices().size(); ++i)
        vshape->GetMesh()->getCoordsVertices()[i] -= baricenter;

    this->SetDensity((float)density);
    this->SetMass(mass * density);
    this->SetInertia(inertia * density);

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

// -----------------------------------------------------------------------------

ChBodyEasyConvexHullAuxRef::ChBodyEasyConvexHullAuxRef(std::vector<ChVector<>>& points,
                                                       double density,
                                                       bool visualize,
                                                       bool collide,
                                                       std::shared_ptr<ChMaterialSurface> material,
                                                       std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBodyAuxRef(collision_model) {
    SetupBody(points, density, visualize, collide, material);
}

ChBodyEasyConvexHullAuxRef::ChBodyEasyConvexHullAuxRef(std::vector<ChVector<>>& points,
                                                       double density,
                                                       std::shared_ptr<ChMaterialSurface> material,
                                                       collision::ChCollisionSystemType collision_type)
    : ChBodyAuxRef(collision_type) {
    SetupBody(points, density, true, true, material);
}

void ChBodyEasyConvexHullAuxRef::SetupBody(std::vector<ChVector<>>& points,
                                           double density,
                                           bool visualize,
                                           bool collide,
                                           std::shared_ptr<ChMaterialSurface> material) {
    auto vshape = chrono_types::make_shared<ChTriangleMeshShape>();
    collision::bt_utils::ChConvexHullLibraryWrapper lh;
    lh.ComputeHull(points, *vshape->GetMesh());
    if (visualize) {
        vshape->SetName("chull_mesh_" + std::to_string(GetIdentifier()));
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        vmodel->AddShape(vshape);
        this->AddVisualModel(vmodel);
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

    SetDensity((float)density);
    SetMass(mass * density);
    // this->SetInertia(inertia * density);
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

        GetCollisionModel()->ClearModel();
        GetCollisionModel()->AddConvexHull(material, points_reduced);
        GetCollisionModel()->BuildModel();
        SetCollide(true);
    }

    m_mesh = vshape->GetMesh();
}

// -----------------------------------------------------------------------------

ChBodyEasyMesh::ChBodyEasyMesh(const std::string& filename,
                               double density,
                               bool compute_mass,
                               bool visualize,
                               bool collide,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept,
                               std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBodyAuxRef(collision_model) {
    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    SetupBody(trimesh, filename, density, compute_mass, visualize, collide, material, sphere_swept);
}

ChBodyEasyMesh::ChBodyEasyMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,
                               double density,
                               bool compute_mass,
                               bool visualize,
                               bool collide,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept,
                               std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBodyAuxRef(collision_model) {
    SetupBody(mesh, "EasyMesh", density, compute_mass, visualize, collide, material, sphere_swept);
}

ChBodyEasyMesh::ChBodyEasyMesh(const std::string& filename,
                               double density,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept,
                               collision::ChCollisionSystemType collision_type)
    : ChBodyAuxRef(collision_type) {
    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true);
    SetupBody(trimesh, filename, density, true, true, true, material, sphere_swept);
}

ChBodyEasyMesh::ChBodyEasyMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,
                               double density,
                               std::shared_ptr<ChMaterialSurface> material,
                               double sphere_swept,
                               collision::ChCollisionSystemType collision_type)
    : ChBodyAuxRef(collision_type) {
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
        auto vshape = chrono_types::make_shared<ChTriangleMeshShape>();
        vshape->SetMesh(trimesh);
        vshape->SetName(name);
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        vmodel->AddShape(vshape);
        this->AddVisualModel(vmodel);
    }

    this->SetDensity((float)density);
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
        GetCollisionModel()->ClearModel();
        GetCollisionModel()->AddTriangleMesh(material, trimesh, false, false, VNULL, ChMatrix33<>(1), sphere_swept);
        GetCollisionModel()->BuildModel();
        SetCollide(true);
    }
}

// -----------------------------------------------------------------------------

ChBodyEasyClusterOfSpheres::ChBodyEasyClusterOfSpheres(std::vector<ChVector<>>& positions,
                                                       std::vector<double>& radii,
                                                       double density,
                                                       bool visualize,
                                                       bool collide,
                                                       std::shared_ptr<ChMaterialSurface> material,
                                                       std::shared_ptr<collision::ChCollisionModel> collision_model)
    : ChBody(collision_model) {
    SetupBody(positions, radii, density, visualize, collide, material);
}

ChBodyEasyClusterOfSpheres::ChBodyEasyClusterOfSpheres(std::vector<ChVector<>>& positions,
                                                       std::vector<double>& radii,
                                                       double density,
                                                       std::shared_ptr<ChMaterialSurface> material,
                                                       collision::ChCollisionSystemType collision_type)
    : ChBody(collision_type) {
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

    this->SetDensity((float)density);
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
        auto vmodel = chrono_types::make_shared<ChVisualModel>();
        for (unsigned int i = 0; i < positions.size(); ++i) {
            auto vshape = chrono_types::make_shared<ChSphereShape>();
            vshape->GetSphereGeometry().rad = radii[i];
            vmodel->AddShape(vshape, ChFrame<>(offset_positions[i]));
        }
        this->AddVisualModel(vmodel);
    }
}

}  // end namespace chrono
