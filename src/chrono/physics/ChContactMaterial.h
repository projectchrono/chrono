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
// Classes:
//   ChContactMaterialData
//   ChContactMaterial
//   ChContactMaterialComposite
//   ChContactMaterialCompositionStrategy
// =============================================================================

#ifndef CH_MATERIAL_SURFACE_H
#define CH_MATERIAL_SURFACE_H

#include <algorithm>

#include "chrono/core/ChClassFactory.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Enumeration of contact methods.
enum class ChContactMethod {
    NSC,  ///< non-smooth, constraint-based (a.k.a. rigid-body) contact
    SMC   ///< smooth, penalty-based (a.k.a. soft-body) contact
};

/// Base class for specifying material properties for contact force generation.
class ChApi ChContactMaterial {
  public:
    virtual ~ChContactMaterial() {}

    /// "Virtual" copy constructor.
    virtual ChContactMaterial* Clone() const = 0;

    virtual ChContactMethod GetContactMethod() const = 0;

    /// Set the static friction coefficient (default 0.6).
    /// Usually in the range [0, 1], rarely above.
    void SetStaticFriction(float val) { static_friction = val; }

    /// Get the static friction coefficient.
    float GetStaticFriction() const { return static_friction; }

    /// Set the sliding (kinetic) friction coefficient (default: 0.6).
    void SetSlidingFriction(float val) { sliding_friction = val; }

    /// Get the sliding friction coefficient.
    float GetSlidingFriction() const { return sliding_friction; }

    /// Set both static friction and sliding friction at once, with same value.
    void SetFriction(float val);

    /// Set the rolling friction coefficient (default: 0).
    /// Usually around 1E-3.
    void SetRollingFriction(float val) { rolling_friction = val; }

    /// Get the rolling friction coefficient.
    float GetRollingFriction() const { return rolling_friction; }

    /// Set the spinning friction coefficient (default: 0).
    /// Usually around 1E-3.
    void SetSpinningFriction(float val) { spinning_friction = val; }

    /// Get the roliung friction coefficient.
    float GetSpinningFriction() const { return spinning_friction; }

    /// Set the normal coefficient of restitution (default: 0.4).
    /// In the range [0, 1].
    void SetRestitution(float val) { restitution = val; }

    /// Get the coefficient of restitution.
    float GetRestitution() const { return restitution; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

    /// Construct and return a contact material of the specified type with default properties.
    static std::shared_ptr<ChContactMaterial> DefaultMaterial(ChContactMethod contact_method);

    // Properties common to both NSC and SMC materials
    float static_friction;    ///< static coefficient of friction
    float sliding_friction;   ///< sliding coefficient of friction
    float rolling_friction;   ///< rolling coefficient of friction
    float spinning_friction;  ///< spinning coefficient of friction
    float restitution;        ///< coefficient of restitution

  protected:
    ChContactMaterial();
    ChContactMaterial(const ChContactMaterial& other);
};

CH_CLASS_VERSION(ChContactMaterial, 0)

/// Material information for a collision shape.
/// Provides mechanism for creating a contact material of appropriate type for a particular contact formulation
/// (SMC or NSC).
class ChApi ChContactMaterialData {
  public:
    float mu;  ///< coefficient of friction
    float cr;  ///< coefficient of restitution
    float Y;   ///< Young's modulus
    float nu;  ///< Poisson ratio
    float kn;  ///< normal stiffness
    float gn;  ///< normal viscous damping
    float kt;  ///< tangential stiffness
    float gt;  ///< tangential viscous damping

    /// Define default properties for contact material information.
    ChContactMaterialData();

    /// Define contact material data with provided properties.
    ChContactMaterialData(float mu, float cr, float Y, float nu, float kn, float gn, float kt, float gt);

    /// Construct a contact material, consistent with the specified method, using the current data.
    std::shared_ptr<ChContactMaterial> CreateMaterial(ChContactMethod contact_method) const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);
};

CH_CLASS_VERSION(ChContactMaterialData, 0)

/// Base class for composite material for a contact pair.
class ChApi ChContactMaterialComposite {
  public:
    virtual ~ChContactMaterialComposite() {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);
};

CH_CLASS_VERSION(ChContactMaterialComposite, 0)

/// Base class for material composition strategy.
/// Implements the default combination laws for coefficients of friction, cohesion, compliance, etc.
/// Derived classes can override one or more of these combination laws.
/// Enabling the use of a customized composition strategy is system type-dependent.
class ChApi ChContactMaterialCompositionStrategy {
  public:
    virtual ~ChContactMaterialCompositionStrategy() {}

    virtual float CombineFriction(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineCohesion(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineRestitution(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineDamping(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineCompliance(float a1, float a2) const { return a1 + a2; }

    virtual float CombineAdhesionMultiplier(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineStiffnessCoefficient(float a1, float a2) const { return (a1 + a2) / 2; }
    virtual float CombineDampingCoefficient(float a1, float a2) const { return (a1 + a2) / 2; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);
};

CH_CLASS_VERSION(ChContactMaterialCompositionStrategy, 0)

typedef std::shared_ptr<ChContactMaterial> ChContactMaterialSharedPtr;

}  // end namespace chrono

#endif
