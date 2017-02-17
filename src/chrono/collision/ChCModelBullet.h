//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_MODELBULLET_H
#define CHC_MODELBULLET_H

#include <vector>
#include <memory>

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "chrono/geometry/ChLinePath.h"

// forward references
class btCollisionObject;
// class btCollisionShape;

namespace chrono {

// forward references
class ChBody;

namespace collision {

class ChConvexDecomposition;

///  A wrapper to use the Bullet collision detection
///  library

class ChApi ChModelBullet : public ChCollisionModel {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChModelBullet)

  protected:
    // The Bullet collision object containing Bullet geometries
    btCollisionObject* bt_collision_object;

    // Vector of shared pointers to geometric objects.
    std::vector<std::shared_ptr<btCollisionShape>> shapes;

  public:
    ChModelBullet();
    virtual ~ChModelBullet();

    /// Deletes all inserted geometries.
    /// Also, if you begin the definition of a model, AFTER adding
    /// the geometric description, remember to call the ClearModel().
    /// MUST be inherited by child classes! (ex for resetting also BV hierarchies)
    virtual int ClearModel();

    /// Builds the BV hierarchy.
    /// Call this function AFTER adding the geometric description.
    /// MUST be inherited by child classes! (ex for bulding BV hierarchies)
    virtual int BuildModel();

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

    /// Add a box shape to this model, for collision purposes
    virtual bool AddBox(double hx,
                        double hy,
                        double hz,
                        const ChVector<>& pos = ChVector<>(),
                        const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a cylinder to this model (default axis on Y direction), for collision purposes
    virtual bool AddCylinder(double rx,
                             double rz,
                             double hy,
                             const ChVector<>& pos = ChVector<>(),
                             const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a cylinder to this model (default axis on Y direction), for collision purposes
    virtual bool AddCone(double rx,
                         double rz,
                         double hy,
                         const ChVector<>& pos = ChVector<>(),
                         const ChMatrix33<>& rot = ChMatrix33<>(1)) {
        return false;
    }

    /// Add a capsule to this model (default axis in Y direction), for collision purposes
    virtual bool AddCapsule(double radius,
                            double hlen,
                            const ChVector<>& pos = ChVector<>(),
                            const ChMatrix33<>& rot = ChMatrix33<>(1)) {
        return false;
    }

    /// Add a rounded box shape to this model, for collision purposes
    virtual bool AddRoundedBox(double hx,
                               double hy,
                               double hz,
                               double sphere_r,
                               const ChVector<>& pos = ChVector<>(),
                               const ChMatrix33<>& rot = ChMatrix33<>(1)) {
        return false;
    }

    /// Add a rounded cylinder to this model (default axis on Y direction), for collision purposes
    virtual bool AddRoundedCylinder(double rx,
                                    double rz,
                                    double hy,
                                    double sphere_r,
                                    const ChVector<>& pos = ChVector<>(),
                                    const ChMatrix33<>& rot = ChMatrix33<>(1)) {
        return false;
    }

    /// Add a rounded cone to this model (default axis on Y direction), for collision purposes
    virtual bool AddRoundedCone(double rx,
                                double rz,
                                double hy,
                                double sphere_r,
                                const ChVector<>& pos = ChVector<>(),
                                const ChMatrix33<>& rot = ChMatrix33<>(1)) {
        return false;
    }

    /// Add a convex hull to this model. A convex hull is simply a point cloud that describe
    /// a convex polytope. Connectivity between the vertexes, as faces/edges in triangle meshes is not necessary.
    /// Points are passed as a list, that is instantly copied into the model.
    virtual bool AddConvexHull(std::vector<ChVector<double> >& pointlist,
                               const ChVector<>& pos = ChVector<>(),
                               const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// Add a triangle mesh to this model, passing a triangle mesh (do not delete the triangle mesh
    /// until the collision model, because depending on the implementation of inherited ChCollisionModel
    /// classes, maybe the triangle is referenced via a striding interface or just copied)
    /// Note: if possible, in sake of high performance, avoid triangle meshes and prefer simplified
    /// representations as compounds of convex shapes of boxes/spheres/etc.. type.
    virtual bool AddTriangleMesh(const geometry::ChTriangleMesh& trimesh,
                                 bool is_static,
                                 bool is_convex,
                                 const ChVector<>& pos = ChVector<>(),
                                 const ChMatrix33<>& rot = ChMatrix33<>(1),
                                 double sphereswept_thickness = 0.0);

    /// CUSTOM for this class only: add a concave triangle mesh that will be managed
    /// by GImpact mesh-mesh algorithm. Note that, despite this can work with
    /// arbitrary meshes, there could be issues of robustness and precision, so
    /// when possible, prefer simplified representations as compounds of convex
    /// shapes of boxes/spheres/etc.. type.
    virtual bool AddTriangleMeshConcave(const geometry::ChTriangleMesh& trimesh,
                                        const ChVector<>& pos = ChVector<>(),
                                        const ChMatrix33<>& rot = ChMatrix33<>(1));

    /// CUSTOM for this class only: add a concave triangle mesh that will be decomposed
    /// into a compound of convex shapes. Decomposition could be more efficient than
    /// AddTriangleMeshConcave(), but preprocessing decomposition might take a while, and
    /// decomposition result is often approximate. Therefore, despite this can work with
    /// arbitrary meshes, there could be issues of robustness and precision, so
    /// when possible, prefer simplified representations as compounds of convex
    /// shapes of boxes/spheres/etc.. type.
    virtual bool AddTriangleMeshConcaveDecomposed(ChConvexDecomposition& mydecomposition,
                                                  const ChVector<>& pos = ChVector<>(),
                                                  const ChMatrix33<>& rot = ChMatrix33<>(1));

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

    /// Add a 2D closed line, defined on the XY plane passing by pos and alinged as rot,
    /// that defines a 2D collision shape that will collide with another 2D line of the same type
    /// if aligned on the same plane. This is useful for mechanisms that work on a plane, and that
    /// require more precise collision that is not possible with current 3D shapes. For example,
    /// the line can contain concave or convex round fillets. 
    /// Requirements: 
    /// - the line must be clockwise for inner material, (counterclockwise=hollow, material outside)
    /// - the line must contain only ChLineSegment and ChLineArc sub-lines
    /// - the sublines must follow in the proper order, with cohincident corners, and must be closed.
    virtual bool Add2Dpath(geometry::ChLinePath& mpath,
                           const ChVector<>& pos = ChVector<>(),
                           const ChMatrix33<>& rot = ChMatrix33<>(1),
                           const double thickness = 0.001);

    /// Add a point-like sphere, that will collide with other geometries,
    /// but won't ever create contacts between them.
    virtual bool AddPoint(  double radius = 0,                     ///< the radius of the node 
                            const ChVector<>& pos = ChVector<>()   ///< the position of the node in model coordinates
                           );

    /// Add a triangle from  mesh. 
    /// For efficiency, points are stored as pointers. Thus, the user must 
    /// take care of memory management and of dangling pointers.
    virtual bool AddTriangleProxy(  ChVector<>* p1,                 ///< points to vertex1 coords
                                    ChVector<>* p2,                 ///< points to vertex2 coords
                                    ChVector<>* p3,                 ///< points to vertex3 coords
                                    ChVector<>* ep1,                ///< points to neighbouring vertex at edge1 if any
                                    ChVector<>* ep2,                ///< points to neighbouring vertex at edge1 if any
                                    ChVector<>* ep3,                ///< points to neighbouring vertex at edge1 if any
                                    bool mowns_vertex_1,            ///< vertex is owned by this triangle (otherwise, owned by neighbour)
                                    bool mowns_vertex_2,
                                    bool mowns_vertex_3,
                                    bool mowns_edge_1,              ///< edge is owned by this triangle (otherwise, owned by neighbour)
                                    bool mowns_edge_2,
                                    bool mowns_edge_3,
                                    double msphereswept_rad=0       ///< sphere swept triangle ('fat' triangle, improves robustness)
                                  );

    /// Add all shapes already contained in another model.
    /// Thank to the adoption of shared pointers, underlying shapes are
    /// shared (not copied) among the models; this will save memory when you must
    /// simulate thousands of objects with the same collision shape.
    /// The 'another' model must be of ChModelBullet subclass.
    virtual bool AddCopyOfAnotherModel(ChCollisionModel* another);

    virtual void SetFamily(int mfamily);
    virtual int GetFamily();
    virtual void SetFamilyMaskNoCollisionWithFamily(int mfamily);
    virtual void SetFamilyMaskDoCollisionWithFamily(int mfamily);
    virtual bool GetFamilyMaskDoesCollisionWithFamily(int mfamily);

    /// Set the collision family group of this model.
    /// This is an alternative way of specifying the collision family for this
    /// object.  The value family_group must have a single bit set (i.e. it must
    /// be a power of 2). The corresponding family is then the bit position.
    virtual void SetFamilyGroup(short group);

    /// Set the collision mask for this model.
    /// Any set bit in the specified mask indicates that this model collides with
    /// all objects whose family is equal to the bit position.
    virtual void SetFamilyMask(short mask);

    /// Returns the axis aligned bounding box (AABB) of the collision model,
    /// i.e. max-min along the x,y,z world axes. Remember that SyncPosition()
    /// should be invoked before calling this.
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const;

    /// Sets the position and orientation of the collision
    /// model as the current position of the corresponding ChContactable
    virtual void SyncPosition();

    /// If the collision shape is a sphere, resize it and return true (if no
    /// sphere is found in this collision shape, return false).
    /// It can also change the outward envelope; the inward margin is automatically the radius of the sphere.
    bool SetSphereRadius(double coll_radius, double out_envelope);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);


    //
    // CUSTOM BULLET
    //

    /// Return the pointer to the Bullet model
    btCollisionObject* GetBulletModel() { return this->bt_collision_object; }

  private:
    void _injectShape(const ChVector<>& pos, const ChMatrix33<>& rot, btCollisionShape* mshape);

    void onFamilyChange();
};


}  // end namespace collision

CH_CLASS_VERSION(collision::ChModelBullet,0)

}  // end namespace chrono

#endif
