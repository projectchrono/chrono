// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Hammad Mazhar
// =============================================================================
// Geometric model for the custom multicore Chrono collision system
// =============================================================================

#ifndef CH_COLLISION_MODEL_CHRONO_H
#define CH_COLLISION_MODEL_CHRONO_H

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionShapeChrono.h"

#include "chrono/multicore_math/ChMulticoreMath.h"

namespace chrono {

// Forward references
class ChBody;

namespace collision {

/// @addtogroup collision_mc
/// @{

/// Geometric model for the custom multicore Chrono collision system.
class ChApi ChCollisionModelChrono : public ChCollisionModel {
  public:
    ChCollisionModelChrono();
    virtual ~ChCollisionModelChrono();

    /// Return the type of this collision model.
    virtual ChCollisionSystemType GetType() const override { return ChCollisionSystemType::CHRONO; }

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
        double axis_x,                                ///< x axis length
        double axis_y,                                ///< y axis length
        double axis_z,                                ///< z axis length
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a box shape to this collision model.
    virtual bool AddBox(                              //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double size_x,                                ///< x dimension
        double size_y,                                ///< y dimension
        double size_z,                                ///< z dimension
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a rounded box shape to this collision model.
    virtual bool AddRoundedBox(                       //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double size_x,                                ///< x dimension
        double size_y,                                ///< y dimension
        double size_z,                                ///< z dimension
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

    /// Add a cylinder to this collision model (axis in Z direction).
    virtual bool AddCylinder(                         //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< radius
        double height,                                ///< height
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a cylindrical shell to this collision model (axis in Z direction).
    virtual bool AddCylindricalShell(                 //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< radius
        double height,                                ///< height
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a rounded cylinder to this collision model (axis in Z direction).
    virtual bool AddRoundedCylinder(                  //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< radius
        double height,                                ///< height
        double sphere_r,                              ///< radius of sweeping sphere
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a cone to this collision model (axis in Z direction).
    virtual bool AddCone(                             //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< radius
        double height,                                ///< height
        const ChVector<>& pos = ChVector<>(),         ///< base center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a capsule to this collision model (axis in Z direction).
    virtual bool AddCapsule(                          //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< radius
        double height,                                ///< height of cylindrical portion
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
    /// the Y axis in radial direction. The two axes of the ellipse are axis_vert (for the
    /// vertical direction, i.e. the axis parallel to Y) and axis_hor (for the axis that
    /// is perpendicular to Y). Also, the solid is clamped with two discs on the top and
    /// the bottom, at levels Y_low and Y_high.
    /// Currently not supported.
    virtual bool AddBarrel(                           //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double Y_low,                                 ///< bottom level
        double Y_high,                                ///< top level
        double axis_vert,                             ///< ellipse axis in vertical direction
        double axis_hor,                              ///< ellipse axis in horizontal direction
        double R_offset,                              ///< lateral offset (radius at top and bottom)
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
    /// CYLSHELL     radius halflength
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

    ChVector<> aabb_min;
    ChVector<> aabb_max;

  protected:
    ChBody* mbody;
};

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono

#endif
