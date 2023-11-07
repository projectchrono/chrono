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
//// - eliminate GetPhysicsItem
//// - change signature for ClearModel / BuildModel; no need for return value?

#ifndef CH_COLLISION_MODEL_H
#define CH_COLLISION_MODEL_H

#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/collision/ChCollisionShape.h"
#include "chrono/collision/ChCollisionShapes.h"

namespace chrono {

// forward references
class ChPhysicsItem;


/// @addtogroup chrono_collision
/// @{

/// Collision engine type.
enum class ChCollisionSystemType {
    BULLET,  ///< Bullet-based collision detection system
    CHRONO,  ///< Chrono multicore collision detection system
    OTHER    ///< other type (external)
};

/// Class defining the geometric model for collision detection.
/// A ChCollisionModel contains all geometric shapes on a rigid body, for collision purposes.
class ChApi ChCollisionModel {
  public:
    /// A ShapeInstance is a pair of a collision shape and its position in the model.
    typedef std::pair<std::shared_ptr<ChCollisionShape>, ChFrame<>> ShapeInstance;

    ChCollisionModel();
    virtual ~ChCollisionModel();

    /// Return the type of this collision model.
    virtual ChCollisionSystemType GetType() const = 0;

    /// Delete all inserted collision shapes.
    /// This function also removes this model from the collsion system (if applicable).
    void Clear();

    /// Complete the construction of the collision model.
    /// This function also inserts this model into the collision system (if applicable).
    /// This function must be invoked after all geometric collision shapes have been added to the model.
    void Build();

    /// Add a collision shape with specified position within the model.
    void AddShape(std::shared_ptr<ChCollisionShape> shape,  ///< visualization shape
                  const ChFrame<>& frame = ChFrame<>()      ///< shape frame in model
    );

    /// Convenience function to add a cylinder specified through a radius and end points.
    void AddCylinder(std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
                     double radius,                                ///< radius
                     const ChVector<>& p1,                         ///< first end point
                     const ChVector<>& p2                          ///< second end point
    );

    /// Add all shapes already contained in another model. If possible, derived classes should implement this so that
    /// underlying shapes are shared (not copied) among the models.
    /// TODO: keep? implement in this base class?
    virtual bool AddCopyOfAnotherModel(ChCollisionModel* another) = 0;

    // OTHER FUNCTIONS

    /// Get the pointer to the contactable object.
    ChContactable* GetContactable() { return mcontactable; }

    /// Set the pointer to the contactable object.
    /// A derived class may override this, but should always invoke this base class implementation.
    virtual void SetContactable(ChContactable* mc) { mcontactable = mc; }

    /// Get the pointer to the client owner ChPhysicsItem.
    /// Default: just casts GetContactable(). Just for backward compatibility.
    /// It might return null if contactable not inherited by  ChPhysicsItem.
    /// ***TODO*** remove the need of ChPhysicsItem*, just use ChContactable* in all code
    virtual ChPhysicsItem* GetPhysicsItem();

    /// Synchronize the position and orientation of the collision model to the associated contactable.
    virtual void SyncPosition() = 0;

    /// By default, all collision objects belong to family n.0,
    /// but you can set family in range 0..15. This is used when
    /// the objects collided with another: the contact is created
    /// only if the family is within the 'family mask' of the other,
    /// and viceversa.
    /// These default implementations use the family group.
    virtual void SetFamily(int mfamily);
    virtual int GetFamily();

    /// By default, family mask is all turned on, so all families
    /// can collide with this object, but you can turn on-off some bytes
    /// of this mask so that some families do not collide.
    /// When two objects collide, the contact is created
    /// only if the family is within the 'family mask' of the other,
    /// and viceversa.
    /// These default implementations use the family mask.
    virtual void SetFamilyMaskNoCollisionWithFamily(int mfamily);
    virtual void SetFamilyMaskDoCollisionWithFamily(int mfamily);

    /// Tells if the family mask of this collision object allows
    /// for the collision with another collision object belonging to
    /// a given family.
    /// This default implementation uses the family mask.
    virtual bool GetFamilyMaskDoesCollisionWithFamily(int mfamily);

    /// Return the collision family group of this model.
    /// The collision family of this model is the position of the single set bit in the return value.
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

    /// Set the suggested collision 'inward safe margin' for the shapes to be added from now on.
    /// If this margin is too high for some thin or small shapes, it may be clamped.
    /// If dist\<0 and inter-penetration occurs (e.g. due to numerical errors) within this 'safe margin' inward range,
    /// collision detection is still fast and reliable (beyond this, for deep penetrations, CD still works, but might be
    /// slower and less reliable) Call this BEFORE adding the shapes into the model. Side effect: think of the margin as
    /// a radius of a 'smoothing' fillet on all corners of the shapes - that's why you cannot exceed with this.
    virtual void SetSafeMargin(double amargin) { model_safe_margin = (float)amargin; }

    /// Returns the inward safe margin (see SetSafeMargin() )
    virtual float GetSafeMargin() { return model_safe_margin; }

    /// Set the suggested collision outward 'envelope' used from shapes added from now on.
    /// This 'envelope' is a surrounding invisible volume which extends outward from the surface, and it is used to
    /// detect contacts a bit before shapes come into contact, i.e. when dist\>0. However contact points will stay on
    /// the true surface of the geometry, not on the external surface of the envelope. Call this BEFORE adding the
    /// shapes into the model. Side effect: AABB are 'expanded' outward by this amount, so if you exaggerate with this
    /// value, CD might be slower and too sensible. On the other hand, if you set this value to 0, contacts are detected
    /// only for dist<=0, thus causing unstable simulation.
    virtual void SetEnvelope(double amargin) { model_envelope = (float)amargin; }

    /// Return the outward safe margin (see SetEnvelope() )
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

    /// Return the current axis aligned bounding box (AABB) of the collision model.
    /// The two return vectors represent the min.max corners along the x,y,z world axes.
    /// Note that SyncPosition() should be invoked before calling this.
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const = 0;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive);

    /// Return the number of collision shapes in this model.
    int GetNumShapes() const { return (int)m_shapes.size(); }

    /// Get the list of collision shapes in this model.
    const std::vector<std::shared_ptr<ChCollisionShape>>& GetShapes() const { return m_shapes; }

    /// Get the collision shape with specified index.
    std::shared_ptr<ChCollisionShape> GetShape(int index) { return m_shapes[index]; }

    /// Return the position and orientation of the collision shape with specified index, relative to the model frame.
    virtual ChCoordsys<> GetShapePos(int index) const = 0;

    /// Return shape characteristic dimensions.
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
    std::vector<double> GetShapeDimensions(int index) const;

    /// Set the contact material for the collision shape with specified index.
    void SetShapeMaterial(int index, std::shared_ptr<ChMaterialSurface> mat);

    /// Set the contact material for all collision shapes in the model (all shapes will share the material).
    /// This function is useful in adjusting contact material properties for objects imported from outside (e.g., from SolidWorks).
    void SetAllShapesMaterial(std::shared_ptr<ChMaterialSurface> mat);

  protected:
    /// Remove this model from the collision system (if applicable).
    virtual void Dissociate() = 0;

    /// Insert this model into the collision system (if applicable).
    virtual void Associate() = 0;

    /// Populate the collision system with the collision shapes defined in this model.
    virtual void Populate() = 0;

    /// Copy the collision shapes from another model.
    void CopyShapes(ChCollisionModel* other);

    virtual float GetSuggestedFullMargin() { return model_envelope + model_safe_margin; }

    float model_envelope;         ///< Maximum envelope: surrounding volume from surface to the exterior
    float model_safe_margin;      ///< Maximum margin value to be used for fast penetration contact detection
    ChContactable* mcontactable;  ///< Pointer to the contactable object

    short int family_group;  ///< Collision family group
    short int family_mask;   ///< Collision family mask

    std::vector<ShapeInstance> m_shape_instances;             ///< list of collision shapes and positions in model
    std::vector<std::shared_ptr<ChCollisionShape>> m_shapes;  ///< extended list of collision shapes
};

/// @} chrono_collision


CH_CLASS_VERSION(ChCollisionModel, 0)

}  // end namespace chrono

#endif
