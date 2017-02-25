#pragma once

#include <list>

#include "chrono/physics/ChContactContainerBase.h"
#include "chrono/physics/ChContactTuple.h"

#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChDataManager.h"

namespace chrono {

/// @addtogroup parallel_module
/// @{

/// Class representing a container of many contacts, implemented as a linked list of contact tuples.
/// Notes:
/// * This container is used only for reporting geometric information about the contact pairs
///   and is therefore suitable for both DVI and DEM systems.
/// * Currently, only contacts between rigid bodies are considered

class CH_PARALLEL_API ChContactContainerParallel : public ChContactContainerBase {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChContactContainerParallel)

  public:
    typedef ChContactTuple<ChContactable_1vars<6>, ChContactable_1vars<6> > ChContact_6_6;

    ChContactContainerParallel(ChParallelDataManager* dc);
    ChContactContainerParallel(const ChContactContainerParallel& other);
    ~ChContactContainerParallel();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChContactContainerParallel* Clone() const override { return new ChContactContainerParallel(*this); }

    virtual int GetNcontacts() const override { return data_manager->num_rigid_contacts; }

    virtual void RemoveAllContacts() override;
    virtual void BeginAddContact() override;
    virtual void AddContact(const collision::ChCollisionInfo& mcontact) override;
    virtual void EndAddContact() override;

    /// Return the list of contacts between rigid bodies
    const std::list<ChContact_6_6*>& GetContactList() const { return contactlist_6_6; }

    ChParallelDataManager* data_manager;

  private:
    int n_added_6_6;
    std::list<ChContact_6_6*> contactlist_6_6;
    std::list<ChContact_6_6*>::iterator lastcontact_6_6;
};

/// @} parallel_module
}
