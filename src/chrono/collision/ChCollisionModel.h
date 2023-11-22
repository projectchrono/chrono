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

#ifndef CH_COLLISION_MODEL_H
#define CH_COLLISION_MODEL_H

#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/collision/ChCollisionShape.h"
#include "chrono/collision/ChCollisionShapes.h"

namespace chrono {

// forward references
class ChPhysicsItem;
class ChContactable;
class ChCollisionModelImpl;

/// @addtogroup chrono_collision
/// @{

/// Class defining the geometric model for collision detection.
class ChApi ChCollisionModel {
  public:
    /// A ShapeInstance is a pair of a collision shape and its position in the model.
    typedef std::pair<std::shared_ptr<ChCollisionShape>, ChFrame<>> ShapeInstance;

    ChCollisionModel();
    ChCollisionModel(const ChCollisionModel& other);
    ~ChCollisionModel();

    /// Delete all inserted collision shapes.
    void Clear();

    /// Add a collision shape with specified position within the model.
    void AddShape(std::shared_ptr<ChCollisionShape> shape,  ///< collision shape
                  const ChFrame<>& frame = ChFrame<>()      ///< shape frame in model
    );

    /// Convenience function to add a cylinder collision shape specified through a radius and end points.
    void AddCylinder(std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
                     double radius,                                ///< radius
                     const ChVector<>& p1,                         ///< first end point
                     const ChVector<>& p2                          ///< second end point
    );

    /// Add copies of the collision shapes in the provided model to this collision model.
    void AddShapes(std::shared_ptr<ChCollisionModel> model,  ///< collision model
                   const ChFrame<>& frame = ChFrame<>()      ///< model frame in model
    );

    /// Set the pointer to the contactable object.
    void SetContactable(ChContactable* contactable);

    /// Get the pointer to the contactable object.
    ChContactable* GetContactable() { return contactable; }

    /// Get the pointer to the client owner ChPhysicsItem.
    ///
    //// TODO OBSOLETE
    ///
    ChPhysicsItem* GetPhysicsItem();

    /// Synchronize the position and orientation of the collision model to the associated contactable.
    void SyncPosition();

    /// Set the collision family for this model (0...15).
    /// By default, all collision objects belong to family 0.
    void SetFamily(int family);
    int GetFamily();

    /// By default, family mask is all turned on, so all families can collide with this object, but you can turn on-off
    /// some bytes of this mask so that some families do not collide. When two objects collide, the contact is created
    /// only if the family is within the 'family mask' of the other, and viceversa.
    void SetFamilyMaskNoCollisionWithFamily(int family);
    void SetFamilyMaskDoCollisionWithFamily(int family);

    /// Indicate if the family mask of this collision object allows for the collision with another collision object
    /// belonging to a given family.
    bool GetFamilyMaskDoesCollisionWithFamily(int family);

    /// Return the collision family group of this model.
    /// The collision family of this model is the position of the single set bit in the return value.
    short int GetFamilyGroup() const { return family_group; }

    /// Set the collision family group of this model.
    /// This is an alternative way of specifying the collision family for this object. The value family_group must have
    /// a single bit set (i.e. it must be a power of 2). The corresponding family is then the bit position.
    void SetFamilyGroup(short int group);

    /// Return the collision mask for this model.
    /// Each bit of the return value indicates whether this model collides with the corresponding family (bit set) or
    /// not (bit unset).
    short int GetFamilyMask() const { return family_mask; }

    /// Set the collision mask for this model.
    /// Any set bit in the specified mask indicates that this model collides with all objects whose family is equal to
    /// the bit position.
    void SetFamilyMask(short int mask);

    // TOLERANCES, ENVELOPES, THRESHOLDS

    /// Set the suggested collision 'inward safe margin' for the shapes to be added from now on.
    /// If this margin is too high for some thin or small shapes, it may be clamped.
    /// If dist<0 and inter-penetration occurs (e.g. due to numerical errors) within this 'safe margin' inward range,
    /// collision detection is still fast and reliable (beyond this, for deep penetrations, CD still works, but might be
    /// slower and less reliable) Call this BEFORE adding the shapes into the model. Side effect: think of the margin as
    /// a radius of a 'smoothing' fillet on all corners of the shapes - that's why you cannot exceed with this.
    void SetSafeMargin(float margin) { model_safe_margin = margin; }

    /// Return the inward safe margin (see SetSafeMargin).
    float GetSafeMargin() { return model_safe_margin; }

    /// Set the suggested collision outward 'envelope' used from shapes added from now on.
    /// This 'envelope' is a surrounding invisible volume which extends outward from the surface, and it is used to
    /// detect contacts a bit before shapes come into contact, i.e. when dist\>0. However contact points will stay on
    /// the true surface of the geometry, not on the external surface of the envelope. Call this BEFORE adding the
    /// shapes into the model. Side effect: AABB are 'expanded' outward by this amount, so if you exaggerate with this
    /// value, CD might be slower and too sensible. On the other hand, if you set this value to 0, contacts are detected
    /// only for dist<=0, thus causing unstable simulation.
    void SetEnvelope(float envelope) { model_envelope = envelope; }

    /// Return the outward safe margin (see SetEnvelope).
    float GetEnvelope() { return model_envelope; }

    /// Set the default envelope value.
    /// All collision shapes in all collision models created after the call to this function will use this value.
    /// A particular collision system may ignore this suggested value.
    static void SetDefaultSuggestedEnvelope(double envelope);

    /// Set the default margin (inward penetration).
    /// All collision shapes in all collision models created after the call to this function will use this value.
    /// A particular collision system may ignore this suggested value.
    static void SetDefaultSuggestedMargin(double margin);

    static double GetDefaultSuggestedEnvelope();
    static double GetDefaultSuggestedMargin();

    /// Return the current axis aligned bounding box (AABB) of the collision model.
    /// Note that SyncPosition() should be invoked before calling this.
    geometry::ChAABB GetBoundingBox() const;

    /// Method to allow serialization of transient data to archives.
    void ArchiveOut(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& marchive);

    /// Return the number of collision shapes in this model.
    int GetNumShapes() const { return (int)m_shape_instances.size(); }

    /// Get the list of collision shapes in this model.
    const std::vector<ShapeInstance>& GetShapes() const { return m_shape_instances; }

    /// Get the collision shape with specified index.
    const ShapeInstance& GetShape(int index) const { return m_shape_instances[index]; }

    /// Set the contact material for all collision shapes in the model (all shapes will share the material).
    /// This function is useful in adjusting contact material properties for objects imported from outside (e.g., from
    /// SolidWorks).
    void SetAllShapesMaterial(std::shared_ptr<ChMaterialSurface> mat);

    // Get direct access to the concrete implementation object.
    // These functions are provided only for implementation of a concrete collision system.
    bool HasImplementation() const { return impl != nullptr; }
    ChCollisionModelImpl* GetImplementation() const { return impl; }
    void RemoveImplementation() { impl = nullptr; }

  private:
    float model_envelope;        ///< Maximum envelope: surrounding volume from surface to the exterior
    float model_safe_margin;     ///< Maximum margin value to be used for fast penetration contact detection
    ChContactable* contactable;  ///< Pointer to the contactable object

    short int family_group;  ///< Collision family group
    short int family_mask;   ///< Collision family mask

    std::vector<ShapeInstance> m_shape_instances;  ///< list of collision shapes and positions in model

    ChCollisionModelImpl* impl;  ///< concrete implementation of the collision model

    friend class ChCollisionModelImpl;
};

// Base class for a concrete collision model, specific to a particular collision detection system.
class ChCollisionModelImpl {
  public:
    virtual ~ChCollisionModelImpl() {}

  protected:
    ChCollisionModelImpl(ChCollisionModel* collision_model);

    /// Get the pointer to the contactable object.
    ChContactable* GetContactable();

    /// Synchronize the position and orientation of the collision model to the associated contactable.
    virtual void SyncPosition() = 0;

    /// Additional operations to be performed on a change in collision family.
    virtual void OnFamilyChange(short int family_group, short int family_mask) {}

    /// Return the current axis aligned bounding box (AABB) of the collision model.
    /// The two return vectors represent the min.max corners along the x,y,z world axes.
    /// Note that SyncPosition() should be invoked before calling this.
    virtual geometry::ChAABB GetBoundingBox() const = 0;

    ChCollisionModel* model;  // associated collision model

    friend class ChCollisionModel;
};

/// @} chrono_collision

CH_CLASS_VERSION(ChCollisionModel, 0)

}  // end namespace chrono

#endif
