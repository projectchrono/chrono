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

#ifndef CH_CONTACT_CONTAINER_NSC_H
#define CH_CONTACT_CONTAINER_NSC_H

#include <list>

#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactNSC.h"
#include "chrono/physics/ChContactNSCrolling.h"
#include "chrono/physics/ChContactable.h"

namespace chrono {

/// Class representing a container of many non-smooth contacts.
/// Implemented using linked lists of ChContactNSC objects (that is, contacts between two ChContactable objects, with 3
/// reactions). It might also contain ChContactNSCrolling objects (extended versions of ChContactNSC, with 6 reactions,
/// that account also for rolling and spinning resistance), but also for '6dof vs 6dof' contactables.
class ChApi ChContactContainerNSC : public ChContactContainer {
  public:
    ChContactContainerNSC();
    ChContactContainerNSC(const ChContactContainerNSC& other);
    virtual ~ChContactContainerNSC();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChContactContainerNSC* Clone() const override { return new ChContactContainerNSC(*this); }

    /// Report the number of added contacts.
    virtual unsigned int GetNumContacts() const override { return n_added + n_added_rolling; }

    /// Remove (delete) all contained contact data.
    virtual void RemoveAllContacts() override;

    /// The collision system will call BeginAddContact() before adding all contacts (for example with AddContact() or
    /// similar). Instead of simply deleting all list of the previous contacts, this optimized implementation rewinds
    /// the link iterator to begin and tries to reuse previous contact objects until possible, to avoid too much
    /// allocation/deallocation.
    virtual void BeginAddContact() override;

    /// Add a contact between two collision shapes, storing it into this container.
    /// A compositecontact material is created from the two given materials.
    /// In this case, the collision info object may have null pointers to collision shapes.
    virtual void AddContact(const ChCollisionInfo& cinfo,
                            std::shared_ptr<ChContactMaterial> mat1,
                            std::shared_ptr<ChContactMaterial> mat2) override;

    /// Add a contact between two collision shapes, storing it into this container.
    /// The collision info object is assumed to contain valid pointers to the two colliding shapes.
    /// A composite contact material is created from their material properties.
    virtual void AddContact(const ChCollisionInfo& cinfo) override;

    /// The collision system will call BeginAddContact() after adding all contacts (for example with AddContact() or
    /// similar). This optimized version purges the end of the list of contacts that were not reused (if any).
    virtual void EndAddContact() override;

    /// Scan all the contacts and for each contact executes the OnReportContact() function of the provided callback
    /// object.
    virtual void ReportAllContacts(std::shared_ptr<ReportContactCallback> callback) override;

    /// Report the number of scalar unilateral constraints.
    /// Note: friction constraints aren't exactly unilaterals, but they are still counted.
    virtual unsigned int GetNumConstraintsUnilateral() override { return 3 * n_added + 6 * n_added_rolling; }

    /// Objects will rebounce only if their relative colliding speed is above this threshold.
    double GetMinBounceSpeed() const { return min_bounce_speed; }

    /// Update state of this contact container: compute jacobians, violations, etc.
    /// and store results in inner structures of contacts.
    virtual void Update(double mtime, bool update_assets) override;

    /// Compute contact forces on all contactable objects in this container.
    /// This function caches contact forces in a map.
    virtual void ComputeContactForces() override;

    /// Return the resultant contact force acting on the specified contactable object.
    virtual ChVector3d GetContactableForce(ChContactable* contactable) override;

    /// Return the resultant contact torque acting on the specified contactable object.
    virtual ChVector3d GetContactableTorque(ChContactable* contactable) override;

    // STATE FUNCTIONS

    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    // SOLVER INTERFACE

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void LoadConstraintJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    int n_added;
    std::list<ChContactNSC*> contacts;
    std::list<ChContactNSC*>::iterator last_contact;

    int n_added_rolling;
    std::list<ChContactNSCrolling*> contacts_rolling;
    std::list<ChContactNSCrolling*>::iterator last_contact_rolling;

    std::unordered_map<ChContactable*, ForceTorque> contact_forces;

  private:
    void InsertContact(const ChCollisionInfo& cinfo, const ChContactMaterialCompositeNSC& cmat);

    double min_bounce_speed;  ///< minimum speed for rebounce after impacts. Lower speeds are clamped to 0

    friend class ChSystemNSC;
};

CH_CLASS_VERSION(ChContactContainerNSC, 0)

}  // end namespace chrono

#endif
