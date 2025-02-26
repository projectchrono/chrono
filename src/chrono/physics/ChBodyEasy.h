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
#include "chrono/collision/bullet/ChCollisionModelBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {

/// Create rigid bodies with a spherical shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasySphere : public ChBody {
  public:
    /// Create a rigid body with optional sphere visualization and/or collision shape.
    /// The sphere is created at the center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasySphere(double radius,                                         ///< radius of the sphere
                     double density,                                        ///< density of the body
                     bool create_visualization = true,                      ///< create visualization asset
                     bool create_collision = false,                         ///< enable collision
                     std::shared_ptr<ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a rigid body with a sphere visualization and collision shape.
    /// The sphere is created at the center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasySphere(double radius,                               ///< radius of the sphere
                     double density,                              ///< density of the body
                     std::shared_ptr<ChContactMaterial> material  ///< surface contact material
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& archive_in);

  private:
    void SetupBody(double radius,
                   double density,
                   bool create_visualization,
                   bool create_collision,
                   std::shared_ptr<ChContactMaterial> material);

    ChBodyEasySphere() {}
};

/// Create rigid bodies with an ellipsoid shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyEllipsoid : public ChBody {
  public:
    /// Create a rigid body with optional ellipsoid visualization and/or collision shape.
    /// The ellipsoid is created at the center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasyEllipsoid(ChVector3d axes,                                       ///< ellipsoid axis lengths
                        double density,                                        ///< density of the body
                        bool create_visualization = true,                      ///< create visualization asset
                        bool create_collision = false,                         ///< enable collision
                        std::shared_ptr<ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a rigid body with an ellipsoid visualization and collision shape.
    /// The ellipsoid is created at the center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasyEllipsoid(ChVector3d axes,                             ///< ellipsoid axis lengths
                        double density,                              ///< density of the body
                        std::shared_ptr<ChContactMaterial> material  ///< surface contact material
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& archive_in);

  private:
    void SetupBody(ChVector3d axes,
                   double density,
                   bool create_visualization,
                   bool create_collision,
                   std::shared_ptr<ChContactMaterial> material);

    ChBodyEasyEllipsoid() {}
};

/// Create rigid bodies with a cylinder shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyCylinder : public ChBody {
  public:
    /// Create a rigid body with optional cylinder visualization and/or collision shape.
    /// The cylinder is created along the specified axis and centered at the center of mass.
    /// Mass and inertia are set automatically depending on density.
    ChBodyEasyCylinder(ChAxis direction,                                      ///< cylinder direction
                       double radius,                                         ///< radius of the cylinder
                       double height,                                         ///< height of the cylinder
                       double density,                                        ///< density of the body
                       bool create_visualization = true,                      ///< create visualization asset
                       bool create_collision = false,                         ///< enable collision
                       std::shared_ptr<ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a rigid body with a cylinder visualization and collision shape.
    /// The cylinder is created along the specified axis and centered at the center of mass.
    /// Mass and inertia are set automatically depending on density.
    ChBodyEasyCylinder(ChAxis direction,                            ///< cylinder direction
                       double radius,                               ///< radius of the cylinder
                       double height,                               ///< height of the cylinder
                       double density,                              ///< density of the body
                       std::shared_ptr<ChContactMaterial> material  ///< surface contact material
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& archive_in);

  private:
    void SetupBody(ChAxis direction,
                   double radius,
                   double height,
                   double density,
                   bool create_visualization,
                   bool create_collision,
                   std::shared_ptr<ChContactMaterial> material);

    ChBodyEasyCylinder() {}
};

/// Create rigid bodies with a box shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyBox : public ChBody {
  public:
    /// Create a rigid body with optional box visualization and/or collision shape.
    /// The box is created at the center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasyBox(double Xsize,                                          ///< size along the X dimension
                  double Ysize,                                          ///< size along the Y dimension
                  double Zsize,                                          ///< size along the Z dimension
                  double density,                                        ///< density of the body
                  bool create_visualization = true,                      ///< create visualization asset
                  bool create_collision = false,                         ///< enable collision
                  std::shared_ptr<ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a rigid body with a box visualization and collision shape.
    /// The box is created at the center of mass. Mass and inertia are set automatically depending on density.
    ChBodyEasyBox(double Xsize,                                ///< size along the X dimension
                  double Ysize,                                ///< size along the Y dimension
                  double Zsize,                                ///< size along the Z dimension
                  double density,                              ///< density of the body
                  std::shared_ptr<ChContactMaterial> material  ///< surface contact material
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& archive_in);

  private:
    void SetupBody(double Xsize,
                   double Ysize,
                   double Zsize,
                   double density,
                   bool create_visualization,
                   bool create_collision,
                   std::shared_ptr<ChContactMaterial> material);

    ChBodyEasyBox() {}
};

/// Create rigid bodies with a convex hull shape.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyConvexHull : public ChBody {
  public:
    /// Create a rigid body with optional convex hull visualization and/or collision shape.
    /// The convex hull is defined with a set of points, expressed in a local frame. Mass and inertia are set
    /// automatically depending on density. The convex hull vertices are translated so that the barycenter coincides
    /// with the center of mass.
    ChBodyEasyConvexHull(std::vector<ChVector3d>& points,                       ///< points of the convex hull
                         double density,                                        ///< density of the body
                         bool create_visualization = true,                      ///< create visualization asset
                         bool create_collision = false,                         ///< enable collision
                         std::shared_ptr<ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a rigid body with a convex hull visualization and collision shape.
    /// The convex hull is defined with a set of points, expressed in a local frame. Mass and inertia are set
    /// automatically depending on density. The convex hull vertices are translated so that the barycenter coincides
    /// with the center of mass.
    ChBodyEasyConvexHull(std::vector<ChVector3d>& points,             ///< points of the convex hull
                         double density,                              ///< density of the body
                         std::shared_ptr<ChContactMaterial> material  ///< surface contact material
    );

    std::shared_ptr<ChTriangleMeshConnected> GetMesh() const { return m_mesh; }

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& archive_in);

  private:
    void SetupBody(std::vector<ChVector3d>& points,
                   double density,
                   bool create_visualization,
                   bool create_collision,
                   std::shared_ptr<ChContactMaterial> material);

    std::shared_ptr<ChTriangleMeshConnected> m_mesh;

    ChBodyEasyConvexHull(std::shared_ptr<ChTriangleMeshConnected> mesh) : m_mesh(mesh) {}
};

/// Create rigid body with a convex hull shape, with a reference frame distinct from the centroidal frame.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyConvexHullAuxRef : public ChBodyAuxRef {
  public:
    /// Create a ChBodyAuxRef with optional convex hull visualization and/or collision shape.
    /// The convex hull is defined with a set of points, expressed in a local frame. Mass and inertia are set
    /// automatically depending on density. The center of mass is set at the barycenter.
    ChBodyEasyConvexHullAuxRef(std::vector<ChVector3d>& points,                       ///< convex hull points
                               double density,                                        ///< density of the body
                               bool create_visualization = true,                      ///< create visualization asset
                               bool create_collision = false,                         ///< enable collision
                               std::shared_ptr<ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a ChBodyAuxRef with a convex hull visualization and collision shape.
    /// The convex hull is defined with a set of points, expressed in a local frame. Mass and inertia are set
    /// automatically depending on density. The center of mass is set at the barycenter.
    ChBodyEasyConvexHullAuxRef(std::vector<ChVector3d>& points,             ///< convex hull points
                               double density,                              ///< density of the body
                               std::shared_ptr<ChContactMaterial> material  ///< surface contact material
    );

    std::shared_ptr<ChTriangleMeshConnected> GetMesh() const { return m_mesh; }

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& archive_in);

  private:
    void SetupBody(std::vector<ChVector3d>& points,
                   double density,
                   bool create_visualization,
                   bool create_collision,
                   std::shared_ptr<ChContactMaterial> material);

    std::shared_ptr<ChTriangleMeshConnected> m_mesh;

    ChBodyEasyConvexHullAuxRef(std::shared_ptr<ChTriangleMeshConnected> mesh) : m_mesh(mesh) {}
};

/// Create rigid bodies with a mesh shape, with a reference frame distinct from the centroidal frame.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
class ChApi ChBodyEasyMesh : public ChBodyAuxRef {
  public:
    /// Create a ChBodyAuxRef with optional mesh visualization and/or collision shape.
    /// The mesh is assumed to be provided in a Wavefront OBJ file and defined with respect to the body reference frame.
    /// Mass and inertia are set automatically depending on density.
    ChBodyEasyMesh(const std::string& filename,       ///< name of the Wavefront OBJ file
                   double density,                    ///< density of the body
                   bool compute_mass = true,          ///< automatic evaluation of inertia properties
                   bool create_visualization = true,  ///< create visualization asset
                   bool create_collision = false,     ///< enable collision
                   std::shared_ptr<ChContactMaterial> material = nullptr,  ///< surface contact material
                   double sphere_swept = 0.001  ///< thickness (collision detection robustness)
    );

    /// Create a ChBodyAuxRef with optional mesh visualization and/or collision shape.
    /// The mesh is defined with respect to the body reference frame. Mass and inertia are set automatically depending
    /// on density.
    ChBodyEasyMesh(std::shared_ptr<ChTriangleMeshConnected> mesh,  ///< triangular mesh
                   double density,                                 ///< density of the body
                   bool compute_mass = true,                       ///< automatic evaluation of inertia properties
                   bool create_visualization = true,               ///< create visualization asset
                   bool create_collision = false,                  ///< enable collision
                   std::shared_ptr<ChContactMaterial> material = nullptr,  ///< surface contact material
                   double sphere_swept = 0.001  ///< thickness (collision detection robustness)
    );

    /// Create a ChBodyAuxRef with a mesh visualization and collision shape.
    /// The mesh is assumed to be provided in a Wavefront OBJ file and defined with respect to the body reference frame.
    /// Mass and inertia are set automatically depending on density.
    ChBodyEasyMesh(const std::string& filename,                  ///< name of the Wavefront OBJ file
                   double density,                               ///< density of the body
                   std::shared_ptr<ChContactMaterial> material,  ///< surface contact material
                   double sphere_swept                           ///< thickness (collision detection robustness)
    );

    /// Create a ChBodyAuxRef with a convex hull visualization and collision shape.
    /// The mesh is defined with respect to the body reference frame. Mass and inertia are set automatically depending
    /// on density.
    ChBodyEasyMesh(std::shared_ptr<ChTriangleMeshConnected> mesh,  ///< triangular mesh
                   double density,                                 ///< density of the body
                   std::shared_ptr<ChContactMaterial> material,    ///< surface contact material
                   double sphere_swept                             ///< thickness (collision detection robustness)
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& archive_in);

  private:
    void SetupBody(std::shared_ptr<ChTriangleMeshConnected> trimesh,
                   const std::string& name,
                   double density,
                   bool compute_mass,
                   bool create_visualization,
                   bool create_collision,
                   std::shared_ptr<ChContactMaterial> material,
                   double sphere_swept);

    ChBodyEasyMesh() {}
};

/// Create rigid bodies with a shape made of a cluster of spheres.
/// Optionally sets the visualization and/or collision geometry and automatically calculates intertia properties based
/// on the geometry.
/// Note that the mass and inertia are computed as if spheres are not intersecting! If a more precise mass/inertia
/// estimation is needed when spheres are intersecting, change mass and inertia after creation using more advanced
/// formulas.
class ChApi ChBodyEasyClusterOfSpheres : public ChBody {
  public:
    /// Create a rigid body with optional sphere cluster mesh visualization and/or collision shapes.
    /// The cluster of spheres will be displaced so that their center of mass corresponds to the origin of the body.
    /// Mass and inertia are set automatically depending on density.
    ChBodyEasyClusterOfSpheres(std::vector<ChVector3d>& positions,                    ///< position of the spheres
                               std::vector<double>& radii,                            ///< sphere radius
                               double density,                                        ///< density of the body
                               bool create_visualization = true,                      ///< create visualization asset
                               bool create_collision = false,                         ///< enable collision
                               std::shared_ptr<ChContactMaterial> material = nullptr  ///< surface contact material
    );

    /// Create a ChBody with a sphere cluster mesh visualization and collision shapes.
    /// The cluster of spheres will be displaced so that their center of mass corresponds to the origin of the ChBody.
    /// Mass and inertia are set automatically depending on density.
    ChBodyEasyClusterOfSpheres(std::vector<ChVector3d>& positions,          ///< position of the spheres
                               std::vector<double>& radii,                  ///< sphere radius
                               double density,                              ///< density of the body
                               std::shared_ptr<ChContactMaterial> material  ///< surface contact material
    );

    /// Deserialization for non-default constructor classes.
    virtual void ArchiveOutConstructor(ChArchiveOut& archive_out);

    /// Serialization for non-default constructor classes.
    static void* ArchiveInConstructor(ChArchiveIn& archive_in);

  private:
    void SetupBody(std::vector<ChVector3d>& positions,
                   std::vector<double>& radii,
                   double density,
                   bool create_visualization,
                   bool create_collision,
                   std::shared_ptr<ChContactMaterial> material);

    ChBodyEasyClusterOfSpheres() {}
};

}  // end namespace chrono

#endif
