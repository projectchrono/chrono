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
// Authors: Hammad Mazhar
// =============================================================================
// Description: class for a parallel collision model
// =============================================================================

#pragma once

#include "collision/ChCCollisionModel.h"

#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/collision/ChCDataStructures.h"
namespace chrono {
// forward references
class ChBody;

namespace collision {

/// @addtogroup parallel_module
/// @{


///  A wrapper to uses GPU collision detection

class CH_PARALLEL_API ChCollisionModelParallel : public ChCollisionModel {
  public:
    ChCollisionModelParallel();
    virtual ~ChCollisionModelParallel();

    /// Deletes all inserted geometries.
    /// Also, if you begin the definition of a model, AFTER adding
    /// the geometric description, remember to call the ClearModel().
    /// MUST be inherited by child classes! (ex for resetting also BV hierarchies)
    virtual int ClearModel();

    /// Builds the BV hierarchy.
    /// Call this function AFTER adding the geometric description.
    /// MUST be inherited by child classes! (ex for bulding BV hierarchies)
    virtual int BuildModel();

    /// Sets the position and orientation of the collision
    /// model as the rigid body current position.
    virtual void SyncPosition();

    /// Sets the pointer to the contactable object.
    virtual void SetContactable(ChContactable* mc);

    //
    // GEOMETRY DESCRIPTION
    //
    //  The following functions must be called inbetween
    //  the ClearModel() BuildModel() pair.

    /// Add a sphere shape to this model, for collision purposes
    virtual bool AddSphere(double radius, const ChVector<>& pos = ChVector<>());

    /// Add an ellipsoid shape to this model, for collision purposes
    virtual bool AddEllipsoid(double rx,
                              double ry,
                              double rz,
                              const ChVector<>& pos = ChVector<>(),
                              const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a box shape to this model, for collision purposes. hx, hy, and hz are half lengths.
    virtual bool AddBox(double hx,
                        double hy,
                        double hz,
                        const ChVector<>& pos = ChVector<>(),
                        const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a rounded box shape to this model, for collision purposes
    virtual bool AddRoundedBox(double hx,
                               double hy,
                               double hz,
                               double sphere_r,
                               const ChVector<>& pos = ChVector<>(),
                               const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a triangle shape to this model, for collision purposes
    virtual bool AddTriangle(ChVector<> A,
                             ChVector<> B,
                             ChVector<> C,
                             const ChVector<>& pos = ChVector<>(),
                             const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a cylinder to this model (default axis on Y direction), for collision purposes.
    /// rx and rz are the radii and hy is the cylinder half-height. The cylinder axis is
    /// parallel to the y-direction.
    virtual bool AddCylinder(double rx,
                             double rz,
                             double hy,
                             const ChVector<>& pos = ChVector<>(),
                             const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a rounded cylinder to this model (default axis on Y direction), for collision
    /// purposes. rx and rz are the radii and hy is the cylinder half-height. The cylinder
    /// axis is parallel to the y-direction.
    virtual bool AddRoundedCylinder(double rx,
                                    double rz,
                                    double hy,
                                    double sphere_r,
                                    const ChVector<>& pos = ChVector<>(),
                                    const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a cone to this model (default axis on Y direction), for collision purposes.
    /// rx and rz are the radii of the base and hy is the cone height.
    virtual bool AddCone(double rx,
                         double rz,
                         double hy,
                         const ChVector<>& pos = ChVector<>(),
                         const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a rounded cone to this model (default axis on Y direction), for collision purposes.
    /// rx and rz are the radii of the base and hy is the cone height.
    virtual bool AddRoundedCone(double rx,
                                double rz,
                                double hy,
                                double sphere_r,
                                const ChVector<>& pos = ChVector<>(),
                                const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a capsule to this model (default axis in Y direction), for collision purposes.
    /// radius is the radius of the spherical ends, and hlen is half of the cylinder height
    /// (total end-to-end length would be 2 * hlen + 2 * radius)
    virtual bool AddCapsule(double radius,
                            double hlen,
                            const ChVector<>& pos = ChVector<>(),
                            const ChMatrix33<>& rot = ChMatrix33<>(1));

    virtual bool AddConvexHull(std::vector<ChVector<double> >& pointlist,
                               const ChVector<>& pos = ChVector<>(),
                               const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a triangle mesh to this model, passing a triangle mesh (do not delete the triangle mesh
    /// until the collision model, because depending on the implementation of inherited ChCollisionModel
    /// classes, maybe the triangle is referenced via a striding interface or just copied)
    /// Note: if possible, in sake of high performance, avoid triangle meshes and prefer simplified
    /// representations as compounds of convex shapes of boxes/spheres/etc.. type.
    virtual bool AddTriangleMesh(
        const geometry::ChTriangleMesh& trimesh,  ///< the triangle mesh
        bool is_static,  ///< true only if model doesn't move (es.a terrain). May improve performance
        bool is_convex,  ///< true if mesh convex hull is used (only for simple mesh). May improve robustness
        const ChVector<>& pos = ChVector<>(),       ///< displacement respect to COG (optional)
        const ChMatrix33<>& rot = ChMatrix33<>(1),  ///< the rotation of the mesh - matrix must be orthogonal
        double sphereswept_thickness = 0.0          ///< optional: outward sphereswept layer (when supported)
        );

    /// Add a barrel-like shape to this model (main axis on Y direction), for collision purposes.
    /// The barrel shape is made by lathing an arc of an ellipse around the vertical Y axis.
    /// The center of the ellipse is on Y=0 level, and it is ofsetted by R_offset from
    /// the Y axis in radial direction. The two radii of the ellipse are R_vert (for the
    /// vertical direction, i.e. the axis parellel to Y) and R_hor (for the axis that
    /// is perpendicular to Y). Also, the solid is clamped with two discs on the top and
    /// the bottom, at levels Y_low and Y_high.
    virtual bool AddBarrel(double Y_low,
                           double Y_high,
                           double R_vert,
                           double R_hor,
                           double R_offset,
                           const ChVector<>& pos = ChVector<>(),
                           const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add all shapes already contained in another model.
    /// Thank to the adoption of shared pointers, underlying shapes are
    /// shared (not copied) among the models; this will save memory when you must
    /// simulate thousands of objects with the same collision shape.
    /// The 'another' model must be of ChModelBullet subclass.
    virtual bool AddCopyOfAnotherModel(ChCollisionModel* another);

    /// Return the axis aligned bounding box for this collision model.
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const;

    /// Return a pointer to the body
    ChBody* GetBody() const { return mbody; }

    /// Set the pointer to the owner rigid body.
    void SetBody(ChBody* body) { mbody = body; }

    /// Return the number of objects in this model.
    int GetNObjects() const { return nObjects; }

    float getVolume();

    std::vector<ConvexShape> mData;
    std::vector<real3> local_convex_data;

  protected:
    ChBody* mbody;

    unsigned int nObjects;
    float total_volume;
};

/// @} parallel_module
}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
