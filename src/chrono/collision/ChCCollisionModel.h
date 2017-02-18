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

#ifndef CHC_COLLISIONMODEL_H
#define CHC_COLLISIONMODEL_H

#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/physics/ChContactable.h"

namespace chrono {

// forward references
class ChPhysicsItem;


namespace collision {
/// Shape types that can be created.
enum ShapeType {
    SPHERE,
    ELLIPSOID,
    BOX,
    CYLINDER,
    CONVEXHULL,
    TRIANGLEMESH,
    BARREL,
    CAPSULE,      // Currently implemented in parallel only
    CONE,         // Currently implemented in parallel only
    ROUNDEDBOX,   // Currently implemented in parallel only
    ROUNDEDCYL,   // Currently implemented in parallel only
    ROUNDEDCONE,  // Currently implemented in parallel only
    CONVEX,       // Currently implemented in parallel only
    TETRAHEDRON   // Currently implemented in parallel only
};

///
/// Class containing the geometric model ready for collision detection.
/// Each rigid body will have a ChCollisionModel.
/// A ChCollisionModel will contain all the geometric description(s)
/// of the shape of the rigid body, for collision purposes.
///

class ChApi ChCollisionModel {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChCollisionModel)

  public:
    ChCollisionModel();

    virtual ~ChCollisionModel(){};

    /// Deletes all inserted geometries.
    /// Also, if you begin the definition of a model, AFTER adding
    /// the geometric description, remember to call the ClearModel().
    /// MUST be inherited by child classes! (ex for resetting also BV hierarchies)
    virtual int ClearModel() = 0;

    /// Builds the BV hierarchy.
    /// Call this function AFTER adding the geometric description.
    /// MUST be inherited by child classes! (ex for bulding BV hierarchies)
    virtual int BuildModel() = 0;

    //
    // GEOMETRY DESCRIPTION
    //
    //  The following functions must be called inbetween
    //  the ClearModel() BuildModel() pair.
    //  The class must implement automatic deletion of the created
    //  geometries at class destruction time and at ClearModel()
    //  Return value is true if the child class implements the
    //  corresponding type of geometry.

    /// Add a sphere shape to this model, for collision purposes
    virtual bool AddSphere(double radius,                        ///< the radius of the sphere
                           const ChVector<>& pos = ChVector<>()  ///< the position of the sphere in model coordinates
                           ) = 0;

    /// Add an ellipsoid shape to this model, for collision purposes
    virtual bool AddEllipsoid(double rx,                                 ///< the rad on x axis
                              double ry,                                 ///< the rad on y axis
                              double rz,                                 ///< the rad on z axis
                              const ChVector<>& pos = ChVector<>(),      ///< the position of the ellipsoid
                              const ChMatrix33<>& rot = ChMatrix33<>(1)  ///< the matrix defining rotation (orthogonal)
                              ) = 0;

    /// Add a box shape to this model, for collision purposes
    virtual bool AddBox(
        double hx,                                 ///< the halfsize on x axis
        double hy,                                 ///< the halfsize on y axis
        double hz,                                 ///< the halfsize on z axis
        const ChVector<>& pos = ChVector<>(),      ///< the position of the box COG
        const ChMatrix33<>& rot = ChMatrix33<>(1)  ///< the rotation of the box - matrix must be orthogonal
        ) = 0;

    /// Add a cylinder to this model (default axis on Y direction), for collision purposes
    virtual bool AddCylinder(double rx,
                             double rz,
                             double hy,
                             const ChVector<>& pos = ChVector<>(),
                             const ChMatrix33<>& rot = ChMatrix33<>(1)) = 0;

    /// Add a cone to this model (default axis on Y direction), for collision purposes
    virtual bool AddCone(double rx,
                         double rz,
                         double hy,
                         const ChVector<>& pos = ChVector<>(),
                         const ChMatrix33<>& rot = ChMatrix33<>(1)) = 0;

    /// Add a capsule to this model (default axis in Y direction), for collision purposes
    virtual bool AddCapsule(double radius,                             ///< capsule radius
                            double hlen,                               ///< half-length of capsule axis
                            const ChVector<>& pos = ChVector<>(),      ///< the position of the ellipsoid
                            const ChMatrix33<>& rot = ChMatrix33<>(1)  ///< the matrix defining rotation (orthogonal)
                            ) = 0;

    /// Add a rounded box shape to this model, for collision purposes
    virtual bool AddRoundedBox(double hx,
                               double hy,
                               double hz,
                               double sphere_r,
                               const ChVector<>& pos = ChVector<>(),
                               const ChMatrix33<>& rot = ChMatrix33<>(1)) = 0;

    /// Add a rounded cylinder to this model (default axis on Y direction), for collision purposes
    virtual bool AddRoundedCylinder(double rx,
                                    double rz,
                                    double hy,
                                    double sphere_r,
                                    const ChVector<>& pos = ChVector<>(),
                                    const ChMatrix33<>& rot = ChMatrix33<>(1)) = 0;

    /// Add a rounded cone to this model (default axis on Y direction), for collision purposes
    virtual bool AddRoundedCone(double rx,
                                double rz,
                                double hy,
                                double sphere_r,
                                const ChVector<>& pos = ChVector<>(),
                                const ChMatrix33<>& rot = ChMatrix33<>(1)) = 0;

    /// Add a convex hull to this model. A convex hull is simply a point cloud that describe
    /// a convex polytope. Connectivity between the vertexes, as faces/edges in triangle meshes is not necessary.
    /// Points are passed as a list, that is instantly copied into the model.
    virtual bool AddConvexHull(std::vector<ChVector<double> >& pointlist,
                               const ChVector<>& pos = ChVector<>(),
                               const ChMatrix33<>& rot = ChMatrix33<>(1)) = 0;

    /// Add a triangle mesh to this model, passing a triangle mesh (do not delete the triangle mesh
    /// until the collision model, because depending on the implementation of inherited ChCollisionModel
    /// classes, maybe the triangle is referenced via a striding interface or just copied)
    /// Note: if possible, in sake of high performance, avoid triangle meshes and prefer simplified
    /// representations as compounds of convex shapes of boxes/spheres/etc.. type. See functions above.
    virtual bool AddTriangleMesh(
        const geometry::ChTriangleMesh& trimesh,  ///< the triangle mesh
        bool is_static,  ///< true only if model doesn't move (es.a terrain). May improve performance
        bool is_convex,  ///< true if mesh convex hull is used (only for simple mesh). May improve robustness
        const ChVector<>& pos = ChVector<>(),      ///< displacement respect to COG (optional)
        const ChMatrix33<>& rot = ChMatrix33<>(1),  ///< the rotation of the mesh - matrix must be orthogonal
        double sphereswept_thickness = 0.0      ///< optional: outward sphereswept layer (when supported)
        ) = 0;

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
                           const ChMatrix33<>& rot = ChMatrix33<>(1)) = 0;

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
                           const double thickness = 0.001) { return true; };

    /// Add a point-like sphere, that will collide with other geometries,
    /// but won't ever create contacts between them.
    virtual bool AddPoint(  double radius = 0,                     ///< the radius of the node 
                            const ChVector<>& pos = ChVector<>()   ///< the position of the node in model coordinates
                           ) { this->AddSphere(radius,pos); return true;}

    /// Add all shapes already contained in another model.
    /// If possible, child classes implement this so that underlying shapes are
    /// shared (not copied) among the models.
    virtual bool AddCopyOfAnotherModel(ChCollisionModel* another) = 0;

    /// Add a cluster of convex hulls by a '.chulls' file description. The file is an ascii text that contains
    /// many lines with "[x] [y] [z]" coordinates of the convex hulls. Hulls are separated by lines with "hull".
    /// Inherited classes should not need to implement/overload this, because this base implementation
    /// basically calls AddConvexHull() n times while parsing the file, that is enough.
    virtual bool AddConvexHullsFromFile(ChStreamInAscii& mstream,
                                        const ChVector<>& pos = ChVector<>(),
                                        const ChMatrix33<>& rot = ChMatrix33<>(1));

    // OTHER FUNCTIONS
    //

    /// Gets the pointer to the contactable object 
    ChContactable* GetContactable() {return mcontactable;}

    /// Sets the pointer to the contactable object.
    /// A derived class may override this, but should always invoke this base class implementation.
    virtual void SetContactable(ChContactable* mc) { mcontactable = mc;}

    /// Gets the pointer to the client owner ChPhysicsItem.
    /// Default: just casts GetContactable(). Just for backward compatibility.
    /// It might return null if contactable not inherited by  ChPhysicsItem.
    /// ***TODO*** remove the need of ChPhysicsItem*, just use ChContactable* in all code
    virtual ChPhysicsItem* GetPhysicsItem(); 

    /// Sets the position and orientation of the collision
    /// model as the rigid body current position.
    /// By default it uses GetCsysForCollisionModel 
    virtual void SyncPosition() =0;

    /// By default, all collsion objects belong to family n.0,
    /// but you can set family in range 0..15. This is used when
    /// the objects collided with another: the contact is created
    /// only if the family is within the 'family mask' of the other,
    /// and viceversa.
    /// NOTE: these functions have NO effect if used before you add
    ///       the body to a ChSystem, using AddBody(). Use after AddBody().
    /// These default implementations use the family group.
    virtual void SetFamily(int mfamily);
    virtual int GetFamily();

    /// By default, family mask is all turned on, so all families
    /// can collide with this object, but you can turn on-off some bytes
    /// of this mask so that some families do not collide.
    /// When two objects collide, the contact is created
    /// only if the family is within the 'family mask' of the other,
    /// and viceversa.
    /// NOTE: these functions have NO effect if used before you add
    ///       the body to a ChSystem, using AddBody(). Use after AddBody().
    /// These default implementations use the family mask.
    virtual void SetFamilyMaskNoCollisionWithFamily(int mfamily);
    virtual void SetFamilyMaskDoCollisionWithFamily(int mfamily);

    /// Tells if the family mask of this collision object allows
    /// for the collision with another collision object belonging to
    /// a given family.
    /// NOTE: this function has NO effect if used before you add
    ///       the body to a ChSystem, using AddBody(). Use after AddBody().
    /// This default implementation uses the family mask.
    virtual bool GetFamilyMaskDoesCollisionWithFamily(int mfamily);

    /// Return the collision family group of this model.
    /// The collision family of this model is the position of the single set bit
    /// in the return value.
    virtual short int GetFamilyGroup() const { return family_group; }
    /// Set the collision family group of this model.
    /// This is an alternative way of specifying the collision family for this
    /// object.  The value family_group must have a single bit set (i.e. it must
    /// be a power of 2). The corresponding family is then the bit position.
    virtual void SetFamilyGroup(short int group);

    /// Return the collision mask for this model.
    /// Each bit of the return value indicates whether this model collides with
    /// the corresponding family (bit set) or not (bit unset).
    virtual short int GetFamilyMask() const { return family_mask; }
    /// Set the collision mask for this model.
    /// Any set bit in the specified mask indicates that this model collides with
    /// all objects whose family is equal to the bit position.
    virtual void SetFamilyMask(short int mask);


    // TOLERANCES, ENVELOPES, THRESHOLDS
    //

    /// Sets the suggested collision 'inward safe margin' for the
    /// shapes to be added from now on, using the AddBox,
    /// AddCylinder etc (where, if this margin is too high for some
    /// thin or small shapes, it may be clamped)..
    /// If dist<0 and interpenetation occurs (ex.for numerical errors) within
    /// this 'safe margin' inward range, collision detection is still fast
    /// and reliable (beyond this, for deep penetrations, CD still works,
    /// but might be slower and less reliable)
    /// Call this BEFORE adding the shapes into the model.
    /// Side effect: think at the margin as a radius of a 'smoothing' fillet
    /// on all corners of the shapes - that's why you cannot exceed with this...
    virtual void SetSafeMargin(double amargin) { model_safe_margin = (float)amargin; }
    /// Returns the inward safe margin (see SetSafeMargin() )
    virtual float GetSafeMargin() { return model_safe_margin; }

    /// Sets the suggested collision outward 'envelope' (used from shapes
    /// added, from now on, to this collision model).  This 'envelope' is a
    /// surrounding invisible volume which extends outward from the
    /// surface, and it is used to detect contacts a bit before shapes
    /// come into contact, i.e. when dist>0. However contact points will stay
    /// on the true surface of the geometry, not on the external surface of the
    /// envelope.
    /// Call this BEFORE adding the shapes into the model.
    /// Side effect: AABB are 'expanded' outward by this amount, so if you
    /// exagerate with this value, CD might be slower and too sensible.
    /// On the other hand, if you set this value to 0, contacts are detected
    /// only for dist<=0, thus causing unstable simulation.
    virtual void SetEnvelope(double amargin) { model_envelope = (float)amargin; }
    /// Returns the outward safe margin (see SetEnvelope() )
    virtual float GetEnvelope() { return model_envelope; }

    /// Using this function BEFORE you start creating collision shapes,
    /// it will make all following collision shapes to take this collision
    /// envelope (safe outward layer) as default.
    /// Easier than calling SetEnvelope() all the times.
    static void SetDefaultSuggestedEnvelope(double menv);

    /// Using this function BEFORE you start creating collision shapes,
    /// it will make all following collision shapes to take this collision
    /// margin (inward penetration layer) as default. If you call it again later, it will have no effect,
    /// except for shapes created later.
    /// Easier than calling SetMargin() all the times.
    static void SetDefaultSuggestedMargin(double mmargin);

    static double GetDefaultSuggestedEnvelope();
    static double GetDefaultSuggestedMargin();

    /// Returns the axis aligned bounding box (AABB) of the collision model,
    /// i.e. max-min along the x,y,z world axes. Remember that SyncPosition()
    /// should be invoked before calling this.
    /// MUST be implemented by child classes!
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const = 0;


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChCollisionModel>();

        // serialize all member data:
        marchive << CHNVP(model_envelope);
        marchive << CHNVP(model_safe_margin);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChCollisionModel>();

        // stream in all member data:
        marchive >> CHNVP(model_envelope);
        marchive >> CHNVP(model_safe_margin);
    }


  protected:
    virtual float GetSuggestedFullMargin() { return model_envelope + model_safe_margin; }

    // Maximum envelope: surrounding volume from surface to the exterior
    float model_envelope;

    // This is the max.value to be used for fast penetration contact detection.
    float model_safe_margin;

    // Pointer to the contactable object
    ChContactable* mcontactable;

    // Collision family group and mask
    short int family_group;
    short int family_mask;
};

}  // end namespace collision

CH_CLASS_VERSION(collision::ChCollisionModel,0)

}  // end namespace chrono

#endif
