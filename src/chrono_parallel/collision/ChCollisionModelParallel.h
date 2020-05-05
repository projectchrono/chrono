// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
// Description: class for a parallel collision model
// =============================================================================

#pragma once

#include "chrono/collision/ChCollisionModel.h"

#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {

// Forward references
class ChBody;

namespace collision {

/// @addtogroup parallel_collision
/// @{

/// Class to encapsulate description of a convex collision shape.
class ChCollisionShapeParallel : public ChCollisionShape {
  public:
    ChCollisionShapeParallel(Type t, std::shared_ptr<ChMaterialSurface> material) : ChCollisionShape(t, material) {}

    real3 A;          ///< location
    real3 B;          ///< dimensions
    real3 C;          ///< extra
    quaternion R;     ///< rotation
    real3* convex;    ///< pointer to convex data;
};

/// Class for geometric model for collision detection.
/// A rigid body that interacts through contact must have a collision model.
class CH_PARALLEL_API ChCollisionModelParallel : public ChCollisionModel {
  public:
    ChCollisionModelParallel();
    virtual ~ChCollisionModelParallel();

    /// Deletes all inserted geometries.
    /// Also, if you begin the definition of a model, AFTER adding
    /// the geometric description, remember to call the ClearModel().
    /// MUST be inherited by child classes! (ex for resetting also BV hierarchies)
    virtual int ClearModel() override;

    /// Builds the BV hierarchy.
    /// Call this function AFTER adding the geometric description.
    /// MUST be inherited by child classes! (ex for bulding BV hierarchies)
    virtual int BuildModel() override;

    /// Sets the position and orientation of the collision
    /// model as the rigid body current position.
    virtual void SyncPosition() override;

    /// Sets the pointer to the contactable object.
    virtual void SetContactable(ChContactable* mc) override;

    //
    // GEOMETRY DESCRIPTION
    //
    //  The following functions must be called inbetween
    //  the ClearModel() BuildModel() pair.

    /// Add a sphere shape to this collision model.
    virtual bool AddSphere(                           //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< sphere radius
        const ChVector<>& pos = ChVector<>()          ///< center position in model coordinates
        ) override;

    /// Add an ellipsoid shape to this collision model.
    virtual bool AddEllipsoid(                        //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double rx,                                    ///< x semi-axis
        double ry,                                    ///< y semi-axis
        double rz,                                    ///< z semi-axis
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a box shape to this collision model.
    virtual bool AddBox(                              //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double hx,                                    ///< x half-dimension
        double hy,                                    ///< y half-dimension
        double hz,                                    ///< z half-dimension
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a rounded box shape to this collision model.
    virtual bool AddRoundedBox(                       //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double hx,                                    ///< x half-dimension
        double hy,                                    ///< y half-dimension
        double hz,                                    ///< z half-dimension
        double sphere_r,                              ///< radius of sweeping sphere
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a triangle shape to this model, for collision purposes.
    virtual bool AddTriangle(                         //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        ChVector<> A,                                 ///< triangle vertex A
        ChVector<> B,                                 ///< triangle vertex B
        ChVector<> C,                                 ///< triangle vertex C
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
    );

    /// Add a cylinder to this collision model (default axis on Y direction).
    virtual bool AddCylinder(                         //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double rx,                                    ///< radius (X direction)
        double rz,                                    ///< radius (Z direction)
        double hy,                                    ///< half length
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a rounded cylinder to this collision model (default axis on Y direction).
    virtual bool AddRoundedCylinder(                  //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double rx,                                    ///< radius (X direction)
        double rz,                                    ///< radius (Z direction)
        double hy,                                    ///< half length
        double sphere_r,                              ///< radius of sweeping sphere
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a cone to this collision model (default axis on Y direction).
    virtual bool AddCone(                             //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double rx,                                    ///< radius (X direction)
        double rz,                                    ///< radius (Z direction)
        double hy,                                    ///< half length
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a rounded cone to this collision model (default axis on Y direction).
    virtual bool AddRoundedCone(                      //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double rx,                                    ///< radius (X direction)
        double rz,                                    ///< radius (Z direction)
        double hy,                                    ///< half length
        double sphere_r,                              ///< radius of sweeping sphere
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a capsule to this collision model (default axis in Y direction).
    virtual bool AddCapsule(                          //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< radius
        double hlen,                                  ///< half-length of capsule axis
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a convex hull to this collision model. A convex hull is simply a point cloud that describe a convex
    /// polytope. Connectivity between the vertexes, as faces/edges in triangle meshes is not necessary.
    /// Points are passed as a list which is then copied into the model.
    virtual bool AddConvexHull(                          //
        std::shared_ptr<ChMaterialSurface> material,     ///< surface contact material
        const std::vector<ChVector<double>>& pointlist,  ///< list of hull points
        const ChVector<>& pos = ChVector<>(),            ///< origin position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)        ///< rotation in model coordinates
        ) override;

    /// Add a triangle mesh to this collision model.
    /// Note: if possible, for better performance, avoid triangle meshes and prefer simplified
    /// representations as compounds of primitive convex shapes (boxes, sphers, etc).
    virtual bool AddTriangleMesh(                           //
        std::shared_ptr<ChMaterialSurface> material,        ///< surface contact material
        std::shared_ptr<geometry::ChTriangleMesh> trimesh,  ///< the triangle mesh
        bool is_static,                                     ///< true if model doesn't move. May improve performance.
        bool is_convex,                                     ///< if true, a convex hull is used. May improve robustness.
        const ChVector<>& pos = ChVector<>(),               ///< origin position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1),          ///< rotation in model coordinates
        double sphereswept_thickness = 0.0                  ///< outward sphere-swept layer (when supported)
        ) override;

    /// Add a barrel-like shape to this collision model (main axis on Y direction).
    /// The barrel shape is made by lathing an arc of an ellipse around the vertical Y axis.
    /// The center of the ellipse is on Y=0 level, and it is offsetted by R_offset from
    /// the Y axis in radial direction. The two radii of the ellipse are R_vert (for the
    /// vertical direction, i.e. the axis parallel to Y) and R_hor (for the axis that
    /// is perpendicular to Y). Also, the solid is clamped with two discs on the top and
    /// the bottom, at levels Y_low and Y_high.
    virtual bool AddBarrel(                           //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double Y_low,                                 ///<
        double Y_high,                                ///<
        double R_vert,                                ///<
        double R_hor,                                 ///<
        double R_offset,                              ///<
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add all shapes already contained in another model.
    virtual bool AddCopyOfAnotherModel(ChCollisionModel* another) override;

    /// Return the axis aligned bounding box for this collision model.
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const override;

    /// Return the position and orientation of the collision shape with specified index, relative to the model frame.
    virtual ChCoordsys<> GetShapePos(int index) const override;

    /// Return shape characteristic dimensions.
    /// The following collision shapes are supported:
    /// <pre>
    /// SPHERE       radius
    /// BOX          x-halfdim y-halfdim z-halfdim
    /// ELLIPSOID    x-radius y-radius z-radius
    /// CYLINDER     x-radius z-radius halflength
    /// CONE         x-radius z-radius halfheight
    /// CAPSULE      radius halflength
    /// ROUNDEDBOX   x-halfdim y-halfdim z-halfdim sphere_rad
    /// ROUNDEDCYL   x-radius z-radius halflength sphere_rad
    /// </pre>
    virtual std::vector<double> GetShapeDimensions(int index) const override;

    /// Return a pointer to the associated body.
    ChBody* GetBody() const { return mbody; }

    /// Set the pointer to the owner rigid body.
    void SetBody(ChBody* body) { mbody = body; }

    std::vector<real3> local_convex_data;

    real3 aabb_min;
    real3 aabb_max;

  protected:
    ChBody* mbody;
};

/// @} parallel_collision

}  // end namespace collision
}  // end namespace chrono
