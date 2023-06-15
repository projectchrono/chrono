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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

//// TODO
//// - implement ArchiveIn & ArchiveOut

#ifndef CH_COLLISION_MODEL_BULLET_H
#define CH_COLLISION_MODEL_BULLET_H

#include <memory>
#include <vector>

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionShapeBullet.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCompoundShape.h"

// forward references
class cbtCollisionObject;
// class cbtCollisionShape;

namespace chrono {

// forward references
class ChBody;

namespace collision {

class ChConvexDecomposition;

/// @addtogroup collision_bullet
/// @{

/// Class defining the Bullet geometric model for collision detection.
class ChApi ChCollisionModelBullet : public ChCollisionModel {
  protected:
    std::unique_ptr<cbtCollisionObject> bt_collision_object;  ///< Bullet collision object containing Bullet geometries
    std::shared_ptr<cbtCompoundShape> bt_compound_shape;  ///< Compound for models with more than one collision shape

  public:
    ChCollisionModelBullet();
    virtual ~ChCollisionModelBullet();

    /// Return the type of this collision model.
    virtual ChCollisionSystemType GetType() const override { return ChCollisionSystemType::BULLET; }

    /// Delete all inserted geometries.
    /// Addition of collision shapes must be done between calls to ClearModel() and BuildModel().
    /// This function must be invoked before adding geometric collision shapes.
    virtual int ClearModel() override;

    /// Complete the construction of the collision model (build the BV hierarchy).
    /// Addition of collision shapes must be done between calls to ClearModel() and BuildModel().
    /// This function must be invoked after all geometric collision shapes have been added.
    virtual int BuildModel() override;

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

    /// Add a cone to this collision model (default axis on Y direction).
    /// Currently not supported.
    virtual bool AddCone(                             //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< radius
        double height,                                ///< height
        const ChVector<>& pos = ChVector<>(),         ///< base center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override {
        return false;
    }

    /// Add a capsule to this collision model (axis in Z direction).
    virtual bool AddCapsule(                          //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< radius
        double height,                                ///< height of cylindrical portion
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override;

    /// Add a rounded box shape to this collision model.
    /// Currently not supported.
    virtual bool AddRoundedBox(                       //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double size_x,                                ///< x dimension
        double size_y,                                ///< y dimension
        double size_z,                                ///< z dimension
        double sphere_r,                              ///< radius of sweeping sphere
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override {
        return false;
    }

    /// Add a rounded cylinder to this collision model (axis in Z direction).
    /// Currently not supported.
    virtual bool AddRoundedCylinder(                  //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< radius
        double height,                                ///< height
        double sphere_r,                              ///< radius of sweeping sphere
        const ChVector<>& pos = ChVector<>(),         ///< center position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)     ///< rotation in model coordinates
        ) override {
        return false;
    }

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

    /// CUSTOM for this class only: add a concave triangle mesh that will be managed
    /// by GImpact mesh-mesh algorithm. Note that, despite this can work with
    /// arbitrary meshes, there could be issues of robustness and precision, so
    /// when possible, prefer simplified representations as compounds of convex
    /// shapes of boxes/spheres/etc.. type.
    virtual bool AddTriangleMeshConcave(                    //
        std::shared_ptr<ChMaterialSurface> material,        ///< surface contact material
        std::shared_ptr<geometry::ChTriangleMesh> trimesh,  ///< the triangle mesh
        const ChVector<>& pos = ChVector<>(),               ///< origin position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)           ///< rotation in model coordinates
    );

    /// CUSTOM for this class only: add a concave triangle mesh that will be decomposed
    /// into a compound of convex shapes. Decomposition could be more efficient than
    /// AddTriangleMeshConcave(), but preprocessing decomposition might take a while, and
    /// decomposition result is often approximate. Therefore, despite this can work with
    /// arbitrary meshes, there could be issues of robustness and precision, so
    /// when possible, prefer simplified representations as compounds of convex
    /// shapes of boxes/spheres/etc.. type.
    virtual bool AddTriangleMeshConcaveDecomposed(               //
        std::shared_ptr<ChMaterialSurface> material,             ///< surface contact material
        std::shared_ptr<ChConvexDecomposition> mydecomposition,  ///< convex mesh decomposition
        const ChVector<>& pos = ChVector<>(),                    ///< origin position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1)                ///< rotation in model coordinates
    );

    /// Add a barrel-like shape to this collision model (main axis on Y direction).
    /// The barrel shape is made by lathing an arc of an ellipse around the vertical Y axis.
    /// The center of the ellipse is on Y=0 level, and it is offsetted by R_offset from
    /// the Y axis in radial direction. The two axes of the ellipse are axis_vert (for the
    /// vertical direction, i.e. the axis parallel to Y) and axis_hor (for the axis that
    /// is perpendicular to Y). Also, the solid is clamped with two discs on the top and
    /// the bottom, at levels Y_low and Y_high.
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

    /// Add a 2D closed line, defined on the XY plane passing by pos and aligned as rot,
    /// that defines a 2D collision shape that will collide with another 2D line of the same type
    /// if aligned on the same plane. This is useful for mechanisms that work on a plane, and that
    /// require more precise collision that is not possible with current 3D shapes. For example,
    /// the line can contain concave or convex round fillets.
    /// Requirements:
    /// - the line must be clockwise for inner material, (counterclockwise=hollow, material outside)
    /// - the line must contain only ChLineSegment and ChLineArc sub-lines
    /// - the sublines must follow in the proper order, with coincident corners, and must be closed.
    virtual bool Add2Dpath(                           //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        std::shared_ptr<geometry::ChLinePath> mpath,  ///< 2D curve path
        const ChVector<>& pos = ChVector<>(),         ///< origin position in model coordinates
        const ChMatrix33<>& rot = ChMatrix33<>(1),    ///< rotation in model coordinates
        const double thickness = 0.001                ///< line thickness
        ) override;

    /// Add a point-like sphere, that will collide with other geometries, but won't ever create contacts between them.
    virtual bool AddPoint(                            //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius = 0,                            ///< node radius
        const ChVector<>& pos = ChVector<>()          ///< center position in model coordinates
        ) override;

    /// Add a triangle from  mesh.
    /// For efficiency, points are stored as pointers. Thus, the user must
    /// take care of memory management and of dangling pointers.
    virtual bool AddTriangleProxy(                    //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        ChVector<>* p1,                               ///< points to vertex1 coords
        ChVector<>* p2,                               ///< points to vertex2 coords
        ChVector<>* p3,                               ///< points to vertex3 coords
        ChVector<>* ep1,                              ///< points to neighbouring vertex at edge1 if any
        ChVector<>* ep2,                              ///< points to neighbouring vertex at edge1 if any
        ChVector<>* ep3,                              ///< points to neighbouring vertex at edge1 if any
        bool mowns_vertex_1,         ///< vertex is owned by this triangle (otherwise, owned by neighbour)
        bool mowns_vertex_2,         ///< vertex is owned by this triangle (otherwise, owned by neighbour)
        bool mowns_vertex_3,         ///< vertex is owned by this triangle (otherwise, owned by neighbour)
        bool mowns_edge_1,           ///< edge is owned by this triangle (otherwise, owned by neighbour)
        bool mowns_edge_2,           ///< edge is owned by this triangle (otherwise, owned by neighbour)
        bool mowns_edge_3,           ///< edge is owned by this triangle (otherwise, owned by neighbour)
        double msphereswept_rad = 0  ///< sphere swept triangle ('fat' triangle, improves robustness)
    );

    /// Add all shapes already contained in another model.
    /// The 'another' model must be of ChCollisionModelBullet subclass.
    virtual bool AddCopyOfAnotherModel(ChCollisionModel* another) override;

    virtual void SetFamily(int mfamily) override;
    virtual int GetFamily() override;
    virtual void SetFamilyMaskNoCollisionWithFamily(int mfamily) override;
    virtual void SetFamilyMaskDoCollisionWithFamily(int mfamily) override;
    virtual bool GetFamilyMaskDoesCollisionWithFamily(int mfamily) override;

    /// Set the collision family group of this model.
    /// This is an alternative way of specifying the collision family for this
    /// object.  The value family_group must have a single bit set (i.e. it must
    /// be a power of 2). The corresponding family is then the bit position.
    virtual void SetFamilyGroup(short int group) override;

    /// Set the collision mask for this model.
    /// Any set bit in the specified mask indicates that this model collides with
    /// all objects whose family is equal to the bit position.
    virtual void SetFamilyMask(short int mask) override;

    /// Returns the axis aligned bounding box (AABB) of the collision model,
    /// i.e. max-min along the x,y,z world axes. Remember that SyncPosition()
    /// should be invoked before calling this.
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const override;

    /// Sets the position and orientation of the collision
    /// model as the current position of the corresponding ChContactable
    virtual void SyncPosition() override;

    /// If the collision shape is a sphere, resize it and return true (if no
    /// sphere is found in this collision shape, return false).
    /// It can also change the outward envelope; the inward margin is automatically the radius of the sphere.
    bool SetSphereRadius(double coll_radius, double out_envelope);

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
    /// </pre>
    virtual std::vector<double> GetShapeDimensions(int index) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    void injectShape(const ChVector<>& pos, const ChMatrix33<>& rot, ChCollisionShapeBullet* shape);

    void onFamilyChange();

    cbtCollisionObject* GetBulletModel() { return bt_collision_object.get(); }

    std::vector<std::shared_ptr<geometry::ChTriangleMesh>> m_trimeshes;

    friend class ChCollisionSystemBullet;
    friend class ChCollisionSystemBulletMulticore;
};

/// @} collision_bullet

}  // end namespace collision

CH_CLASS_VERSION(collision::ChCollisionModelBullet, 0)

}  // end namespace chrono

#endif
