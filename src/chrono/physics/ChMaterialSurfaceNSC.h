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

#ifndef CH_MATERIALSURFACE_NSC_H
#define CH_MATERIALSURFACE_NSC_H

#include "chrono/physics/ChMaterialSurface.h"

namespace chrono {

/// Material data for a collision surface for use with non-smooth (complementarity) contact method.
/// Notes:
/// \li Rolling friction \n
///   Rolling resistant torque is Tr <= (normal force) * (rolling friction parameter) \n
///   The rolling friction parameter is usually a very low value. Measuring unit: m. \n
///   A non-zero value will make the simulation 2x slower!
/// \li Spinning friction \n
///   Spinning resistant torque is Ts <= (normal force) * (spinning friction parameter) \n
///   The spinning friction parameter is usually a very low value.  Measuring unit: m. \n
///   A non-zero value will make the simulation 2x slower!
class ChApi ChMaterialSurfaceNSC : public ChMaterialSurface {
  public:
    ChMaterialSurfaceNSC();
    ChMaterialSurfaceNSC(const ChMaterialSurfaceNSC& other);
    ~ChMaterialSurfaceNSC() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChMaterialSurfaceNSC* Clone() const override { return new ChMaterialSurfaceNSC(*this); }

    virtual ChContactMethod GetContactMethod() const override { return ChContactMethod::NSC; }

    /// The cohesion max. force for normal pulling traction in contacts. Measuring unit: N. Default 0.
    float GetCohesion() { return cohesion; }
    void SetCohesion(float mval) { cohesion = mval; }

    /// The damping in contact, as a factor 'f': damping is a multiple of stiffness [K], that is: [R]=f*[K]
    /// Measuring unit: time, s. Default 0.
    float GetDampingF() { return dampingf; }
    void SetDampingF(float mval) { dampingf = mval; }

    /// Compliance of the contact, in normal direction.
    /// It is the inverse of the stiffness [K] , so for zero value one has a perfectly rigid contact.
    /// Measuring unit: m/N. Default 0.
    float GetCompliance() { return compliance; }
    void SetCompliance(float mval) { compliance = mval; }

    /// Compliance of the contact, in tangential direction.
    /// Measuring unit: m/N. Default 0.
    float GetComplianceT() { return complianceT; }
    void SetComplianceT(float mval) { complianceT = mval; }

    /// Rolling compliance of the contact, if using a nonzero rolling friction.
    /// (If there is no rolling friction, this has no effect.)
    /// Measuring unit: rad/Nm. Default 0.
    float GetComplianceRolling() { return complianceRoll; }
    void SetComplianceRolling(float mval) { complianceRoll = mval; }

    /// Spinning compliance of the contact, if using a nonzero rolling friction.
    /// (If there is no spinning friction, this has no effect.)
    /// Measuring unit: rad/Nm. Default 0.
    float GetComplianceSpinning() { return complianceSpin; }
    void SetComplianceSpinning(float mval) { complianceSpin = mval; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    float cohesion;
    float dampingf;
    float compliance;
    float complianceT;
    float complianceRoll;
    float complianceSpin;
};

CH_CLASS_VERSION(ChMaterialSurfaceNSC, 0)

/// Composite NSC material data for a contact pair.
class ChApi ChMaterialCompositeNSC : public ChMaterialComposite {
  public:
    float static_friction;
    float sliding_friction;
    float rolling_friction;
    float spinning_friction;
    float restitution;
    float cohesion;
    float dampingf;
    float compliance;
    float complianceT;
    float complianceRoll;
    float complianceSpin;

    ChMaterialCompositeNSC();

    ChMaterialCompositeNSC(ChMaterialCompositionStrategy* strategy,
                           std::shared_ptr<ChMaterialSurfaceNSC> mat1,
                           std::shared_ptr<ChMaterialSurfaceNSC> mat2);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChMaterialCompositeNSC, 0)


}  // end namespace chrono

#endif
