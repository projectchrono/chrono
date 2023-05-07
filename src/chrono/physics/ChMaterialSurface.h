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
class ChApi ChMaterialSurface {
  public:
    virtual ~ChMaterialSurface() {}

    /// "Virtual" copy constructor.
    virtual ChMaterialSurface* Clone() const = 0;

    virtual ChContactMethod GetContactMethod() const = 0;

    /// Static sliding friction coefficient.
    /// Usually in 0..1 range, rarely above. Default 0.6.
    void SetSfriction(float val) { static_friction = val; }
    float GetSfriction() const { return static_friction; }

    /// Kinetic sliding friction coefficient.
    void SetKfriction(float val) { sliding_friction = val; }
    float GetKfriction() const { return sliding_friction; }

    /// Set both static friction and kinetic friction at once, with same value.
    void SetFriction(float val);

    /// Rolling friction coefficient. Usually around 1E-3. Default 0.
    void SetRollingFriction(float val) { rolling_friction = val; }
    float GetRollingFriction() const { return rolling_friction; }

    /// Spinning friction coefficient. Usually around 1E-3. Default 0.
    void SetSpinningFriction(float val) { spinning_friction = val; }
    float GetSpinningFriction() const { return spinning_friction; }

    /// Normal coefficient of restitution. In the range [0,1]. Default 0.
    void SetRestitution(float val) { restitution = val; }
    float GetRestitution() const { return restitution; }

    virtual void ArchiveOUT(ChArchiveOut& marchive);
    virtual void ArchiveIN(ChArchiveIn& marchive);

    /// Construct and return a contact material of the specified type with default properties.
    static std::shared_ptr<ChMaterialSurface> DefaultMaterial(ChContactMethod contact_method);

    // Properties common to both NSC and SMC materials
    float static_friction;    ///< Static coefficient of friction
    float sliding_friction;   ///< Kinetic coefficient of friction
    float rolling_friction;   ///< Rolling coefficient of friction
    float spinning_friction;  ///< Spinning coefficient of friction
    float restitution;        ///< Coefficient of restitution

  protected:
    ChMaterialSurface();
    ChMaterialSurface(const ChMaterialSurface& other);
};

CH_CLASS_VERSION(ChMaterialSurface, 0)

/// Material information for a collision shape.
/// Provides mechanism for creating a contact material of type to a particular contact formulation (SMC or NSC).
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
    std::shared_ptr<ChMaterialSurface> CreateMaterial(ChContactMethod contact_method) const;
};

/// Base class for composite material for a contact pair.
class ChApi ChMaterialComposite {
  public:
    virtual ~ChMaterialComposite() {}
};

/// Base class for material composition strategy.
/// Implements the default combination laws for coefficients of friction, cohesion, compliance, etc.
/// Derived classes can override one or more of these combination laws.
/// Enabling the use of a customized composition strategy is system type-dependent.
class ChApi ChMaterialCompositionStrategy {
  public:
    virtual ~ChMaterialCompositionStrategy() {}

    virtual float CombineFriction(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineCohesion(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineRestitution(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineDamping(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineCompliance(float a1, float a2) const { return a1 + a2; }

    virtual float CombineAdhesionMultiplier(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineStiffnessCoefficient(float a1, float a2) const { return (a1 + a2) / 2; }
    virtual float CombineDampingCoefficient(float a1, float a2) const { return (a1 + a2) / 2; }
};


typedef std::shared_ptr<ChMaterialSurface> ChMaterialSurfaceSharedPtr;

}  // end namespace chrono

#endif
