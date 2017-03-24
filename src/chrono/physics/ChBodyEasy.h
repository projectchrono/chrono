// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban, Arman Pazouki
// =============================================================================

#ifndef CHBODYEASY_H
#define CHBODYEASY_H

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChObjShapeFile.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/collision/ChCCollisionUtils.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"

namespace chrono {

// This header includes some easy-to-use bodies, that can be used to create body
// objects that optionally include contact and visualization shapes.

/// Easy-to-use class for quick creation of rigid bodies with a spherical shape.
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
class ChBodyEasySphere : public ChBody {
  public:
    /// Creates a ChBody plus adds a visualization shape and, optionally,
    /// a collision shape. Mass and inertia are set automatically depending
    /// on density.
    /// Sphere is assumed with center at body reference coordsystem.
    ChBodyEasySphere(double radius, double mdensity, bool collide = false, bool visual_asset = true,
			  ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI) : ChBody(contact_method) {
        double mmass = mdensity * ((4.0 / 3.0) * CH_C_PI * pow(radius, 3));
        double inertia = (2.0 / 5.0) * mmass * pow(radius, 2);

        this->SetDensity((float)mdensity);
        this->SetMass(mmass);
        this->SetInertiaXX(ChVector<>(inertia, inertia, inertia));

        if (collide) {
            GetCollisionModel()->ClearModel();
            GetCollisionModel()->AddSphere(radius);  // radius, radius, height on y
            GetCollisionModel()->BuildModel();
            SetCollide(true);
        }
        if (visual_asset) {
            std::shared_ptr<ChSphereShape> vshape(new ChSphereShape());
            vshape->GetSphereGeometry().rad = radius;
            this->AddAsset(vshape);
        }
    }
};


/// Easy-to-use class for quick creation of rigid bodies with an ellipsoid shape.
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
class ChBodyEasyEllipsoid : public ChBody {
  public:
    /// Creates a ChBody plus adds a visualization shape and, optionally,
    /// a collision shape. Mass and inertia are set automatically depending
    /// on density.
    /// Ellipsoid is assumed with center at body reference coordsystem.
    ChBodyEasyEllipsoid(ChVector<> radius, double mdensity, bool collide = false, bool visual_asset = true,
			  ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI) : ChBody(contact_method) {
        double mmass = mdensity * ((4.0 / 3.0) * CH_C_PI * radius.x()*radius.y()*radius.z());
        double inertiax = (1.0 / 5.0) * mmass * (pow(radius.y(), 2)+pow(radius.z(),2));
        double inertiay = (1.0 / 5.0) * mmass * (pow(radius.x(), 2)+pow(radius.z(),2));
        double inertiaz = (1.0 / 5.0) * mmass * (pow(radius.x(), 2)+pow(radius.y(),2));

        this->SetDensity((float)mdensity);
        this->SetMass(mmass);
        this->SetInertiaXX(ChVector<>(inertiax, inertiay, inertiaz));

        if (collide) {
            GetCollisionModel()->ClearModel();
            GetCollisionModel()->AddEllipsoid(radius.x(),radius.y(),radius.z()); 
            GetCollisionModel()->BuildModel();
            SetCollide(true);
        }
        if (visual_asset) {
            std::shared_ptr<ChEllipsoidShape> vshape(new ChEllipsoidShape());
            vshape->GetEllipsoidGeometry().rad = radius;
            this->AddAsset(vshape);
        }
    }
};


/// Easy-to-use class for quick creation of rigid bodies with a cylindrical shape.
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
class ChBodyEasyCylinder : public ChBody {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally,
    /// a collision shape. Mass and inertia are set automatically depending
    /// on density.
    /// Cylinder is assumed with body Y axis as vertical, and reference is at half height.
    ChBodyEasyCylinder(double radius, double height, double mdensity, bool collide = false, bool visual_asset = true,
			  ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI) : ChBody(contact_method) {
        double mmass = mdensity * (CH_C_PI * pow(radius, 2) * height);

        this->SetDensity((float)mdensity);
        this->SetMass(mmass);
        this->SetInertiaXX(ChVector<>((1.0 / 12.0) * mmass * (3 * pow(radius, 2) + pow(height, 2)),
                                      0.5 * mmass * pow(radius, 2),
                                      (1.0 / 12.0) * mmass * (3 * pow(radius, 2) + pow(height, 2))));
        if (collide) {
            GetCollisionModel()->ClearModel();
            GetCollisionModel()->AddCylinder(radius, radius, height * 0.5);  // radius x, radiusz, height on y
            GetCollisionModel()->BuildModel();
            SetCollide(true);
        }
        if (visual_asset) {
            auto vshape = std::make_shared<ChCylinderShape>();
            vshape->GetCylinderGeometry().p1 = ChVector<>(0, -height * 0.5, 0);
            vshape->GetCylinderGeometry().p2 = ChVector<>(0, height * 0.5, 0);
            vshape->GetCylinderGeometry().rad = radius;
            this->AddAsset(vshape);
        }
    }
};

/// Easy-to-use class for quick creation of rigid bodies with a box shape.
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
class ChBodyEasyBox : public ChBody {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally,
    /// a collision shape. Mass and inertia are set automatically depending
    /// on density.
    /// Box is assumed centered, ie. and reference of body in the middle.
    ChBodyEasyBox(double Xsize,
                  double Ysize,
                  double Zsize,
                  double mdensity,
                  bool collide = false,
                  bool visual_asset = true,
				  ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI) : ChBody(contact_method) {
        double mmass = mdensity * (Xsize * Ysize * Zsize);

        this->SetDensity((float)mdensity);
        this->SetMass(mmass);
        this->SetInertiaXX(ChVector<>((1.0 / 12.0) * mmass * (pow(Ysize, 2) + pow(Zsize, 2)),
                                      (1.0 / 12.0) * mmass * (pow(Xsize, 2) + pow(Zsize, 2)),
                                      (1.0 / 12.0) * mmass * (pow(Xsize, 2) + pow(Ysize, 2))));
        if (collide) {
            GetCollisionModel()->ClearModel();
            GetCollisionModel()->AddBox(Xsize * 0.5, Ysize * 0.5, Zsize * 0.5);  // radius x, radius z, height on y
            GetCollisionModel()->BuildModel();
            SetCollide(true);
        }
        if (visual_asset) {
            auto vshape = std::make_shared<ChBoxShape>();
            vshape->GetBoxGeometry().SetLengths(ChVector<>(Xsize, Ysize, Zsize));
            this->AddAsset(vshape);
        }
    }
};

/// Easy-to-use class for quick creation of rigid bodies with a convex hull shape.
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
/// Note that the convex hull points are automatically displaced so that the
/// baricenter of the hull is the body reference coordsys.
class ChBodyEasyConvexHull : public ChBody {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally,
    /// a collision shape. Mass and inertia are set automatically depending
    /// on density.
    /// Convex hull is defined with a set of points.
    ChBodyEasyConvexHull(std::vector<ChVector<> >& points,
                         double mdensity,
                         bool collide = false,
                         bool visual_asset = true,
						 ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI) : ChBody(contact_method) {
        auto vshape = std::make_shared<ChTriangleMeshShape>();
        collision::ChConvexHullLibraryWrapper lh;
        lh.ComputeHull(points, vshape->GetMesh());
        if (visual_asset) {
            this->AddAsset(vshape);
        }

        double mass;
        ChVector<> baricenter;
        ChMatrix33<> inertia;
        vshape->GetMesh().ComputeMassProperties(true, mass, baricenter, inertia);

        // Translate the convex hull baricenter so that body origin is also baricenter
        for (unsigned int i = 0; i < vshape->GetMesh().getCoordsVertices().size(); ++i)
            vshape->GetMesh().getCoordsVertices()[i] -= baricenter;

        this->SetDensity((float)mdensity);
        this->SetMass(mass * mdensity);
        this->SetInertia(inertia * mdensity);

        if (collide) {
            // avoid passing to collision the inner points discarded by convex hull
            // processor, so use mesh vertexes instead of all argument points
            std::vector<ChVector<> > points_reduced;
            points_reduced.resize(vshape->GetMesh().getCoordsVertices().size());
            for (unsigned int i = 0; i < vshape->GetMesh().getCoordsVertices().size(); ++i)
                points_reduced[i] = vshape->GetMesh().getCoordsVertices()[i];

            GetCollisionModel()->ClearModel();
            GetCollisionModel()->AddConvexHull(points_reduced);
            GetCollisionModel()->BuildModel();
            SetCollide(true);
        }
    }
};

/// Easy-to-use class for quick creation of rigid bodies with a convex hull shape,
/// that has a REF csys distinct from the COG cys (this is helpful because in many
/// cases the convex hull might have an offset barycenter with respect to the reference
/// that we want to use for the body - otherwise use the simplier ChBodyEasyConvexHull)
/// This class does automatically, at object creation:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
/// Note:  this inherits the ChBodyAuxRef, that allow to have the barycenter (COG) and
/// the body reference (REF) in two different positions. So the REF will remain unchanged,
/// while the COG reference is moved to the computed barycenter of convex hull and its rotation
/// is automatically aligned to the main inertia axes of the tensor of inertia.
class ChBodyEasyConvexHullAuxRef : public ChBodyAuxRef {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally,
    /// a collision shape. Mass and inertia are set automatically depending
    /// on density.
    /// Convex hull is defined with a set of points.
    ChBodyEasyConvexHullAuxRef(
        std::vector<ChVector<> >& points,  ///< points defined respect REF c.sys of body (initially REF=0,0,0 pos.)
        double mdensity,
        bool collide = false,
        bool visual_asset = true,
		ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI) : ChBodyAuxRef(contact_method) {
        auto vshape = std::make_shared<ChTriangleMeshShape>();
        collision::ChConvexHullLibraryWrapper lh;
        lh.ComputeHull(points, vshape->GetMesh());
        if (visual_asset) {
            this->AddAsset(vshape);  // assets are respect to REF c.sys
        }

        double mass;
        ChVector<> baricenter;
        ChMatrix33<> inertia;
        vshape->GetMesh().ComputeMassProperties(true, mass, baricenter, inertia);
        ChMatrix33<> principal_inertia_csys;
        double principal_I[3];
        inertia.FastEigen(principal_inertia_csys, principal_I);

        this->SetDensity((float)mdensity);
        this->SetMass(mass * mdensity);
        // this->SetInertia(inertia * mdensity);
        this->SetInertiaXX(ChVector<>(principal_I[0] * mdensity, principal_I[1] * mdensity, principal_I[2] * mdensity));

        // Set the COG coordinates to barycenter, without displacing the REF reference
        this->SetFrame_COG_to_REF(ChFrame<>(baricenter, principal_inertia_csys));

        if (collide) {
            // avoid passing to collision the inner points discarded by convex hull
            // processor, so use mesh vertexes instead of all argument points
            std::vector<ChVector<> > points_reduced;
            points_reduced.resize(vshape->GetMesh().getCoordsVertices().size());
            for (unsigned int i = 0; i < vshape->GetMesh().getCoordsVertices().size(); ++i)
                points_reduced[i] = vshape->GetMesh().getCoordsVertices()[i];

            GetCollisionModel()->ClearModel();
            GetCollisionModel()->AddConvexHull(points_reduced);  // coll.model is respect to REF c.sys
            GetCollisionModel()->BuildModel();
            SetCollide(true);
        }
    }
};

/// Easy-to-use class for quick creation of rigid bodies with a triangle mesh shape,
/// that has a REF csys distinct from the COG cys (this is helpful because in many cases
/// the mesh might have an offset barycenter with respect to the reference that we want
/// to use for the body)
/// This class does automatically, at object creation:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry (this
///   requires the mesh to be closed, watertight, with proper triangle orientation)
/// Note:  this inherits the ChBodyAuxRef, that allow to have the barycenter (COG) and
/// the body reference (REF) in two different positions. So the REF will remain unchanged,
/// while the COG reference is moved to the computed barycenter of convex hull and its rotation
/// is automatically aligned to the main inertia axes of the tensor of inertia.
class ChBodyEasyMesh : public ChBodyAuxRef {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally,
    /// a collision shape. Mass and inertia are set automatically depending
    /// on density.
    ChBodyEasyMesh(
        const std::string filename,  ///< .OBJ mesh defined respect REF c.sys of body (initially REF=0,0,0 pos.)
        double mdensity,
        bool compute_mass = true,
        bool collide = false,
        double sphere_swept = 0.001,  ///< radius of 'inflating' of mesh, leads to more robust collision detection
        bool visual_asset = true,
		ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI) : ChBodyAuxRef(contact_method) {
        auto vshape = std::make_shared<ChTriangleMeshShape>();
        vshape->GetMesh().LoadWavefrontMesh(filename, true, true);
        this->AddAsset(vshape);  // assets are respect to REF c.sys

        if (!visual_asset) {
            vshape->SetVisible(false);
        }

        this->SetDensity((float)mdensity);
        if (compute_mass) {
            double mass;
            ChVector<> baricenter;
            ChMatrix33<> inertia;
            vshape->GetMesh().ComputeMassProperties(true, mass, baricenter, inertia);
            ChMatrix33<> principal_inertia_csys;
            double principal_I[3];
            inertia.FastEigen(principal_inertia_csys, principal_I);
            this->SetMass(mass * mdensity);
            this->SetInertiaXX(
                ChVector<>(principal_I[0] * mdensity, principal_I[1] * mdensity, principal_I[2] * mdensity));
            // Set the COG coordinates to barycenter, without displacing the REF reference
            this->SetFrame_COG_to_REF(ChFrame<>(baricenter, principal_inertia_csys));
        }

        if (collide) {
            GetCollisionModel()->ClearModel();
            GetCollisionModel()->AddTriangleMesh(vshape->GetMesh(), false, false, VNULL, ChMatrix33<>(1),
                                                 sphere_swept);  // coll.model is respect to REF c.sys
            GetCollisionModel()->BuildModel();
            SetCollide(true);
        }
    }
};

/// Easy-to-use class for quick creation of rigid bodies with a shape made of a cluster
/// of spheres.
/// Compared to the base ChBody class, this class also does automatically, at object creation,
/// the following tasks that you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
/// NOTE! mass and inertia are computed as if spheres are not intersecting! If you
/// need a more precise mass/inertia estimation when spheres are intersecting, change
/// mass and inertia after creation using more advanced formulas.
class ChBodyEasyClusterOfSpheres : public ChBody {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally,
    /// a collision shape. Mass and inertia are set automatically depending
    /// on density.
    /// The cluster of spheres will be displaced so that their center of mass
    /// corresponds to the origin of the ChBody.
    ChBodyEasyClusterOfSpheres(std::vector<ChVector<> >& positions,
                               std::vector<double>& radii,
                               double mdensity,
                               bool collide = false,
                               bool visual_asset = true,
							   ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI) : ChBody(contact_method) {
        assert(positions.size() == radii.size());

        double totmass = 0;
        ChMatrix33<> totinertia;
        ChVector<> baricenter = VNULL;
        totinertia.Reset();
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
        std::vector<ChVector<> > offset_positions = positions;
        for (unsigned int i = 0; i < positions.size(); ++i)
            offset_positions[i] -= baricenter;

        if (collide) {
            GetCollisionModel()->ClearModel();
            for (unsigned int i = 0; i < positions.size(); ++i) {
                GetCollisionModel()->AddSphere(radii[i], offset_positions[i]);  // radius, radius, height on y
            }
            GetCollisionModel()->BuildModel();
            SetCollide(true);
        }
        if (visual_asset) {
            for (unsigned int i = 0; i < positions.size(); ++i) {
                auto vshape = std::make_shared<ChSphereShape>();
                vshape->GetSphereGeometry().rad = radii[i];
                vshape->GetSphereGeometry().center = offset_positions[i];
                this->AddAsset(vshape);
            }
        }
    }
};

}  // end namespace chrono

#endif
