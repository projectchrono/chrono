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

// Classes for creating easy-to-use bodies that optionally include contact and visualization shapes.

#ifndef CHBODYEASY_H
#define CHBODYEASY_H

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/collision/ChCollisionModelBullet.h"

namespace chrono {

/// Easy-to-use class for quick creation of rigid bodies with a spherical shape.
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
class ChApi ChBodyEasySphere : public ChBody {
  public:
    /// Creates a ChBody plus adds a visualization shape and, optionally, a collision shape.
    /// Mass and inertia are set automatically depending on density.
    /// Sphere is assumed with center at body reference coordsystem.
    ChBodyEasySphere(double radius,                                          ///< radius of the sphere
                     double mdensity,                                        ///< density of the body
                     bool visualize = true,                                  ///< create visualization asset
                     bool collide = false,                                   ///< enable collision
                     std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                     std::shared_ptr<collision::ChCollisionModel> collision_model =
                         chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );
};

/// Easy-to-use class for quick creation of rigid bodies with an ellipsoid shape.
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
class ChApi ChBodyEasyEllipsoid : public ChBody {
  public:
    /// Creates a ChBody plus adds a visualization shape and, optionally, a collision shape.
    /// Mass and inertia are set automatically depending on density.
    /// Ellipsoid is assumed with center at body reference coordsystem.
    ChBodyEasyEllipsoid(ChVector<> radius,                                      ///< radii of the ellipsoid
                        double mdensity,                                        ///< density of the body
                        bool visualize = true,                                  ///< create visualization asset
                        bool collide = false,                                   ///< enable collision
                        std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                        std::shared_ptr<collision::ChCollisionModel> collision_model =
                            chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );
};

/// Easy-to-use class for quick creation of rigid bodies with a cylindrical shape.
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
class ChApi ChBodyEasyCylinder : public ChBody {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally, a collision shape.
    /// Mass and inertia are set automatically depending on density.
    /// Cylinder is assumed with body Y axis as vertical, and reference at half height.
    ChBodyEasyCylinder(double radius,                                          ///< radius of the cylinder
                       double height,                                          ///< height of the cylinder
                       double mdensity,                                        ///< density of the body
                       bool visualize = true,                                  ///< create visualization asset
                       bool collide = false,                                   ///< enable collision
                       std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                       std::shared_ptr<collision::ChCollisionModel> collision_model =
                           chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );
};

/// Easy-to-use class for quick creation of rigid bodies with a box shape.
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
class ChApi ChBodyEasyBox : public ChBody {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally, a collision shape.
    /// Mass and inertia are set automatically depending on density.
    /// Box is assumed centered, i.e. and reference of body in the middle.
    ChBodyEasyBox(double Xsize,                                           ///< size along the X dimension
                  double Ysize,                                           ///< size along the Y dimension
                  double Zsize,                                           ///< size along the Z dimension
                  double mdensity,                                        ///< density of the body
                  bool visualize = true,                                  ///< create visualization asset
                  bool collide = false,                                   ///< enable collision
                  std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                  std::shared_ptr<collision::ChCollisionModel> collision_model =
                      chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );
};

/// Easy-to-use class for quick creation of rigid bodies with a convex hull shape.
/// Compared to the base ChBody class, this class also does
/// automatically, at object creation, the following tasks that
/// you would normally do by hand if using ChBody:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
/// Note that the convex hull points are automatically displaced so that the
/// barycenter of the hull is the body reference coordsys.
class ChApi ChBodyEasyConvexHull : public ChBody {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally, a collision shape.
    /// Mass and inertia are set automatically depending on density.
    /// Convex hull is defined with a set of points.
    ChBodyEasyConvexHull(std::vector<ChVector<> >& points,                       ///< points of the convex hull
                         double mdensity,                                        ///< density of the body
                         bool visualize = true,                                  ///< create visualization asset
                         bool collide = false,                                   ///< enable collision
                         std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                         std::shared_ptr<collision::ChCollisionModel> collision_model =
                             chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );
};

/// Easy-to-use class for quick creation of rigid bodies with a convex hull shape,
/// that has a REF csys distinct from the COG cys (this is helpful because in many
/// cases the convex hull might have an offset barycenter with respect to the reference
/// that we want to use for the body - otherwise use the simpler ChBodyEasyConvexHull)
/// This class does automatically, at object creation:
/// - a visualization shape is created and added, if visualization asset is desired
/// - a collision shape is created and added, if collision is desired,
/// - mass and moment of inertia is automatically set, according to the geometry.
/// Note:  this inherits the ChBodyAuxRef, that allow to have the barycenter (COG) and
/// the body reference (REF) in two different positions. So the REF will remain unchanged,
/// while the COG reference is moved to the computed barycenter of convex hull and its rotation
/// is automatically aligned to the main inertia axes of the tensor of inertia.
class ChApi ChBodyEasyConvexHullAuxRef : public ChBodyAuxRef {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally, a collision shape.
    /// Mass and inertia are set automatically depending on density.
    /// Convex hull is defined with a set of points (specified with respect to the body reference frame)
    ChBodyEasyConvexHullAuxRef(std::vector<ChVector<> >& points,                       ///< convex hull points
                               double mdensity,                                        ///< density of the body
                               bool visualize = true,                                  ///< create visualization asset
                               bool collide = false,                                   ///< enable collision
                               std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                               std::shared_ptr<collision::ChCollisionModel> collision_model =
                                   chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );
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
class ChApi ChBodyEasyMesh : public ChBodyAuxRef {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally, a collision shape.
    /// The mesh is assumed to be provided in an OBJ Wavefront file and defined with respect to the body reference
    /// frame. Mass and inertia are set automatically depending on density.
    ChBodyEasyMesh(
        const std::string filename,                             ///< file name for OBJ Wavefront mesh
        double mdensity,                                        ///< density of the body
        bool compute_mass = true,                               ///< automatic evaluation of inertia properties
        bool visualize = true,                                  ///< create visualization asset
        bool collide = false,                                   ///< enable collision
        std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
        double sphere_swept = 0.001,  ///< radius of 'inflating' of mesh, leads to more robust collision detection
        std::shared_ptr<collision::ChCollisionModel> collision_model =
            chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );
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
class ChApi ChBodyEasyClusterOfSpheres : public ChBody {
  public:
    /// Creates a ChBody plus adds an optional visualization shape and, optionally, a collision shape.
    /// Mass and inertia are set automatically depending on density.
    /// The cluster of spheres will be displaced so that their center of mass corresponds to the origin of the ChBody.
    ChBodyEasyClusterOfSpheres(std::vector<ChVector<> >& positions,                    ///< position of the spheres
                               std::vector<double>& radii,                             ///< sphere radius
                               double mdensity,                                        ///< density of the body
                               bool visualize = true,                                  ///< create visualization asset
                               bool collide = false,                                   ///< enable collision
                               std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                               std::shared_ptr<collision::ChCollisionModel> collision_model =
                                   chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );
};

}  // end namespace chrono

#endif
