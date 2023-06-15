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

#ifndef CHBODYEASY_H
#define CHBODYEASY_H

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/collision/ChCollisionModelBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {

/// Create rigid bodies with a spherical shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasySphere : public ChBody {
  public:
    /// Create a ChBody with optional sphere visualization and/or collision shape. The sphere is created centered at the
    /// center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasySphere(double radius,                                          ///< radius of the sphere
                     double density,                                         ///< density of the body
                     bool visualize = true,                                  ///< create visualization asset
                     bool collide = false,                                   ///< enable collision
                     std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                     std::shared_ptr<collision::ChCollisionModel> collision_model =
                         chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );

    /// Create a ChBody with a sphere visualization and collision shape using the specified collision model type. The
    /// sphere is created centered at the center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasySphere(double radius,                                   ///< radius of the sphere
                     double density,                                  ///< density of the body
                     std::shared_ptr<ChMaterialSurface> material,     ///< surface contact material
                     collision::ChCollisionSystemType collision_type  ///< collision model type
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& marchive);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& marchive);

  private:
    void SetupBody(double radius,
                   double density,
                   bool visualize,
                   bool collide,
                   std::shared_ptr<ChMaterialSurface> material);

    ChBodyEasySphere(){}
};

/// Create rigid bodies with an ellipsoid shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyEllipsoid : public ChBody {
  public:
    /// Create a ChBody with optional ellipsoid visualization and/or collision shape. The ellipsoid is created centered
    /// at the center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasyEllipsoid(ChVector<> axes,                                        ///< ellipsoid axis lengths
                        double density,                                         ///< density of the body
                        bool visualize = true,                                  ///< create visualization asset
                        bool collide = false,                                   ///< enable collision
                        std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                        std::shared_ptr<collision::ChCollisionModel> collision_model =
                            chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );

    /// Create a ChBody with an ellipsoid visualization and collision shape using the specified collision model type.
    /// The ellipsoid is created centered at the center of mass. Mass and inertia are set automatically depending on
    /// density.
    ChBodyEasyEllipsoid(ChVector<> axes,                                 ///< ellipsoid axis lengths
                        double density,                                  ///< density of the body
                        std::shared_ptr<ChMaterialSurface> material,     ///< surface contact material
                        collision::ChCollisionSystemType collision_type  ///< collision model type
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& marchive);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& marchive);

  private:
    void SetupBody(ChVector<> axes,
                   double density,
                   bool visualize,
                   bool collide,
                   std::shared_ptr<ChMaterialSurface> material);

    ChBodyEasyEllipsoid(){}
};

/// Create rigid bodies with a cylinder shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyCylinder : public ChBody {
  public:
    /// Create a ChBody with optional cylinder visualization and/or collision shape.
    /// The cylinder is created along the specified axis and centered at the center of mass.
    /// Mass and inertia are set automatically depending on density.
    ChBodyEasyCylinder(geometry::ChAxis direction,                             ///< cylinder direction
                       double radius,                                          ///< radius of the cylinder
                       double height,                                          ///< height of the cylinder
                       double density,                                         ///< density of the body
                       bool visualize = true,                                  ///< create visualization asset
                       bool collide = false,                                   ///< enable collision
                       std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                       std::shared_ptr<collision::ChCollisionModel> collision_model =
                           chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );

    /// Create a ChBody with a cylinder visualization and collision shape using the specified collision model type.
    /// The cylinder is created along the specified axis and centered at the center of mass.
    /// Mass and inertia are set automatically depending on density.
    ChBodyEasyCylinder(geometry::ChAxis direction,                      ///< cylinder direction
                       double radius,                                   ///< radius of the cylinder
                       double height,                                   ///< height of the cylinder
                       double density,                                  ///< density of the body
                       std::shared_ptr<ChMaterialSurface> material,     ///< surface contact material
                       collision::ChCollisionSystemType collision_type  ///< collision model type
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& marchive);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& marchive);

  private:
    void SetupBody(geometry::ChAxis direction,
                   double radius,
                   double height,
                   double density,
                   bool visualize,
                   bool collide,
                   std::shared_ptr<ChMaterialSurface> material);

    ChBodyEasyCylinder(){}
};

/// Create rigid bodies with a box shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyBox : public ChBody {
  public:
    /// Create a ChBody with optional box visualization and/or collision shape. The box is created centered at the
    /// center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasyBox(double Xsize,                                           ///< size along the X dimension
                  double Ysize,                                           ///< size along the Y dimension
                  double Zsize,                                           ///< size along the Z dimension
                  double density,                                         ///< density of the body
                  bool visualize = true,                                  ///< create visualization asset
                  bool collide = false,                                   ///< enable collision
                  std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                  std::shared_ptr<collision::ChCollisionModel> collision_model =
                      chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );

    /// Create a ChBody with a box visualization and collision shape using the specified collision model type. The box
    /// is created centered at the center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasyBox(double Xsize,                                    ///< size along the X dimension
                  double Ysize,                                    ///< size along the Y dimension
                  double Zsize,                                    ///< size along the Z dimension
                  double density,                                  ///< density of the body
                  std::shared_ptr<ChMaterialSurface> material,     ///< surface contact material
                  collision::ChCollisionSystemType collision_type  ///< collision model type
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& marchive);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& marchive);

private:
    void SetupBody(double Xsize,
                   double Ysize,
                   double Zsize,
                   double density,
                   bool visualize,
                   bool collide,
                   std::shared_ptr<ChMaterialSurface> material);

    ChBodyEasyBox(){}
};

/// Create rigid bodies with a convex hull shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyConvexHull : public ChBody {
  public:
    /// Create a ChBody with optional convex hull visualization and/or collision shape.
    /// The convex hull is defined with a set of points, expressed in a local frame.
    /// Mass and inertia are set automatically depending on density. The convex hull vertices are translated so that the
    /// barycenter coincides with the center of mass.
    ChBodyEasyConvexHull(std::vector<ChVector<>>& points,                        ///< points of the convex hull
                         double density,                                         ///< density of the body
                         bool visualize = true,                                  ///< create visualization asset
                         bool collide = false,                                   ///< enable collision
                         std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                         std::shared_ptr<collision::ChCollisionModel> collision_model =
                             chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );

    /// Create a ChBody with a convex hull visualization and collision shape using the specified collision model type.
    /// The convex hull is defined with a set of points, expressed in a local frame.
    /// Mass and inertia are set automatically depending on density. The convex hull vertices are translated so that the
    /// barycenter coincides with the center of mass.
    ChBodyEasyConvexHull(std::vector<ChVector<>>& points,                 ///< points of the convex hull
                         double density,                                  ///< density of the body
                         std::shared_ptr<ChMaterialSurface> material,     ///< surface contact material
                         collision::ChCollisionSystemType collision_type  ///< collision model type
    );

    std::shared_ptr<geometry::ChTriangleMeshConnected> GetMesh() const { return m_mesh; }

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& marchive);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& marchive);

  private:
    void SetupBody(std::vector<ChVector<>>& points,
                   double density,
                   bool visualize,
                   bool collide,
                   std::shared_ptr<ChMaterialSurface> material);

    std::shared_ptr<geometry::ChTriangleMeshConnected> m_mesh;

    ChBodyEasyConvexHull(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh) : m_mesh(mesh){}
};

/// Create rigid body with a convex hull shape, with a reference frame distinct from the centroidal frame.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyConvexHullAuxRef : public ChBodyAuxRef {
  public:
    /// Create a ChBodyAuxRef with optional convex hull visualization and/or collision shape. The convex hull is defined
    /// with a set of points, expressed in a local frame. Mass and inertia are set automatically depending on density.
    /// The center of mass is set at the barycenter.
    ChBodyEasyConvexHullAuxRef(std::vector<ChVector<>>& points,                        ///< convex hull points
                               double density,                                         ///< density of the body
                               bool visualize = true,                                  ///< create visualization asset
                               bool collide = false,                                   ///< enable collision
                               std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                               std::shared_ptr<collision::ChCollisionModel> collision_model =
                                   chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );

    /// Create a ChBodyAuxRef with a convex hull visualization and collision shape using the specified collision model
    /// type. The convex hull is defined with a set of points, expressed in a local frame. Mass and inertia are set
    /// automatically depending on density. The center of mass is set at the barycenter.
    ChBodyEasyConvexHullAuxRef(std::vector<ChVector<>>& points,                 ///< convex hull points
                               double density,                                  ///< density of the body
                               std::shared_ptr<ChMaterialSurface> material,     ///< surface contact material
                               collision::ChCollisionSystemType collision_type  ///< collision model type
    );

    std::shared_ptr<geometry::ChTriangleMeshConnected> GetMesh() const { return m_mesh; }

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& marchive);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& marchive);

  private:
    void SetupBody(std::vector<ChVector<>>& points,
                   double density,
                   bool visualize,
                   bool collide,
                   std::shared_ptr<ChMaterialSurface> material);

    std::shared_ptr<geometry::ChTriangleMeshConnected> m_mesh;

    ChBodyEasyConvexHullAuxRef(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh) : m_mesh(mesh) {}
};

/// Create rigid bodies with a mesh shape, with a reference frame distinct from the centroidal frame.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyMesh : public ChBodyAuxRef {
  public:
    /// Create a ChBodyAuxRef with optional mesh visualization and/or collision shape. The mesh is assumed to be
    /// provided in a Wavefront OBJ file and defined with respect to the body reference frame. Mass and inertia are set
    /// automatically depending on density.
    ChBodyEasyMesh(const std::string& filename,  ///< name of the Wavefront OBJ file
                   double density,               ///< density of the body
                   bool compute_mass = true,     ///< automatic evaluation of inertia properties
                   bool visualize = true,        ///< create visualization asset
                   bool collide = false,         ///< enable collision
                   std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                   double sphere_swept = 0.001,  ///< thickness (collision detection robustness)
                   std::shared_ptr<collision::ChCollisionModel> collision_model =
                       chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );

    /// Create a ChBodyAuxRef with optional mesh visualization and/or collision shape. The mesh is defined with respect
    /// to the body reference frame. Mass and inertia are set automatically depending on density.
    ChBodyEasyMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,  ///< triangular mesh
                   double density,                                           ///< density of the body
                   bool compute_mass = true,  ///< automatic evaluation of inertia properties
                   bool visualize = true,     ///< create visualization asset
                   bool collide = false,      ///< enable collision
                   std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                   double sphere_swept = 0.001,  ///< thickness (collision detection robustness)
                   std::shared_ptr<collision::ChCollisionModel> collision_model =
                       chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );

    /// Create a ChBodyAuxRef with a mesh visualization and collision shape using the specified collision model type.
    /// The mesh is assumed to be provided in a Wavefront OBJ file and defined with respect to the body reference
    /// frame. Mass and inertia are set automatically depending on density.
    ChBodyEasyMesh(const std::string& filename,                     ///< name of the Wavefront OBJ file
                   double density,                                  ///< density of the body
                   std::shared_ptr<ChMaterialSurface> material,     ///< surface contact material
                   double sphere_swept,                             ///< thickness (collision detection robustness)
                   collision::ChCollisionSystemType collision_type  ///< collision model type
    );

    /// Create a ChBodyAuxRef with a convex hull visualization and collision shape using the specified collision model
    /// type. The mesh is defined with respect to the body reference frame. Mass and inertia are set automatically
    /// depending on density.
    ChBodyEasyMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,  ///< triangular mesh
                   double density,                                           ///< density of the body
                   std::shared_ptr<ChMaterialSurface> material,              ///< surface contact material
                   double sphere_swept,                             ///< thickness (collision detection robustness)
                   collision::ChCollisionSystemType collision_type  ///< collision model type
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& marchive);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& marchive);

  private:
    void SetupBody(std::shared_ptr<geometry::ChTriangleMeshConnected> trimesh,
                   const std::string& name,
                   double density,
                   bool compute_mass,
                   bool visualize,
                   bool collide,
                   std::shared_ptr<ChMaterialSurface> material,
                   double sphere_swept);

    ChBodyEasyMesh(){}
};

/// Create rigid bodies with a shape made of a cluster of spheres.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
/// Note that the mass and inertia are computed as if spheres are not intersecting! If a more precise mass/inertia
/// estimation is needed when spheres are intersecting, change mass and inertia after creation using more advanced
/// formulas.
class ChApi ChBodyEasyClusterOfSpheres : public ChBody {
  public:
    /// Create a ChBody with optional sphere cluster mesh visualization and/or collision shapes. The cluster of spheres
    /// will be displaced so that their center of mass corresponds to the origin of the ChBody. Mass and inertia are set
    /// automatically depending on density.
    ChBodyEasyClusterOfSpheres(std::vector<ChVector<>>& positions,                     ///< position of the spheres
                               std::vector<double>& radii,                             ///< sphere radius
                               double density,                                         ///< density of the body
                               bool visualize = true,                                  ///< create visualization asset
                               bool collide = false,                                   ///< enable collision
                               std::shared_ptr<ChMaterialSurface> material = nullptr,  ///< surface contact material
                               std::shared_ptr<collision::ChCollisionModel> collision_model =
                                   chrono_types::make_shared<collision::ChCollisionModelBullet>()  ///< collision model
    );

    /// Create a ChBody with a sphere cluster mesh visualization and collision shapes using the specified collision
    /// model type. The cluster of spheres will be displaced so that their center of mass corresponds to the origin of
    /// the ChBody. Mass and inertia are set automatically depending on density.
    ChBodyEasyClusterOfSpheres(std::vector<ChVector<>>& positions,              ///< position of the spheres
                               std::vector<double>& radii,                      ///< sphere radius
                               double density,                                  ///< density of the body
                               std::shared_ptr<ChMaterialSurface> material,     ///< surface contact material
                               collision::ChCollisionSystemType collision_type  ///< collision model type
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& marchive);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& marchive);

  private:
    void SetupBody(std::vector<ChVector<>>& positions,
                   std::vector<double>& radii,
                   double density,
                   bool visualize,
                   bool collide,
                   std::shared_ptr<ChMaterialSurface> material);

    ChBodyEasyClusterOfSpheres(){}
};

}  // end namespace chrono

#endif
