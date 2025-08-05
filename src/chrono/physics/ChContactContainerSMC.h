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

#ifndef CH_CONTACT_CONTAINER_SMC_H
#define CH_CONTACT_CONTAINER_SMC_H

#include <algorithm>
#include <cmath>
#include <list>

#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactSMC.h"
#include "chrono/physics/ChContactable.h"

namespace chrono {

/// Class representing a container of many smooth (penalty) contacts.
/// Implemented using linked lists of ChContactSMC objects (that is, contacts between two ChContactable objects).
class ChApi ChContactContainerSMC : public ChContactContainer {
  public:
    ChContactContainerSMC();
    ChContactContainerSMC(const ChContactContainerSMC& other);
    virtual ~ChContactContainerSMC();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChContactContainerSMC* Clone() const override { return new ChContactContainerSMC(*this); }

    /// Report the number of added contacts.
    virtual unsigned int GetNumContacts() const override { return n_added; }

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

    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override;
    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override;

    // SOLVER INTERFACE

    virtual void ConstraintsFbLoadForces(double factor) override;

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    int n_added;
    std::list<ChContactSMC*> contacts;
    std::list<ChContactSMC*>::iterator last_contact;

    std::unordered_map<ChContactable*, ForceTorque> contact_forces;


  private:
    void InsertContact(const ChCollisionInfo& cinfo, const ChContactMaterialCompositeSMC& cmat);
};

CH_CLASS_VERSION(ChContactContainerSMC, 0)

}  // end namespace chrono

#endif
