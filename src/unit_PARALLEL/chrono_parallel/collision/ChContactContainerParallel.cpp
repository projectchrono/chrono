#include "chrono_parallel/collision/ChContactContainerParallel.h"

#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChParticlesClones.h"
#include "collision/ChCModelBullet.h"

namespace chrono {

using namespace collision;
using namespace geometry;

ChContactContainerParallel::ChContactContainerParallel(ChParallelDataManager* dc) : data_manager(dc) {
  contactlist_6_6.clear();
  n_added_6_6 = 0;
}

ChContactContainerParallel::~ChContactContainerParallel() {
  RemoveAllContacts();
}

void ChContactContainerParallel::RemoveAllContacts() {
  std::list<ChContact_6_6*>::iterator itercontact = contactlist_6_6.begin();
  while (itercontact != contactlist_6_6.end()) {
    delete (*itercontact);
    (*itercontact) = 0;
    ++itercontact;
  }
  contactlist_6_6.clear();
  lastcontact_6_6 = contactlist_6_6.begin();
  n_added_6_6 = 0;
}

void ChContactContainerParallel::BeginAddContact() {
  lastcontact_6_6 = contactlist_6_6.begin();
  n_added_6_6 = 0;
}

void ChContactContainerParallel::EndAddContact() {
  // remove contacts that are beyond last contact
  while (lastcontact_6_6 != contactlist_6_6.end()) {
    delete (*lastcontact_6_6);
    lastcontact_6_6 = contactlist_6_6.erase(lastcontact_6_6);
  }
}

void ChContactContainerParallel::AddContact(const collision::ChCollisionInfo& mcontact) {
  assert(mcontact.modelA->GetContactable());
  assert(mcontact.modelB->GetContactable());

  // Return now if both objects are inactive
  bool inactiveA = !mcontact.modelA->GetContactable()->IsContactActive();
  bool inactiveB = !mcontact.modelB->GetContactable()->IsContactActive();

  if (inactiveA && inactiveB)
    return;

  // Currently, we only consider contacts between rigid bodies
  ChContactable_1vars<6>* mmboA = dynamic_cast<ChContactable_1vars<6>*>(mcontact.modelA->GetContactable());
  ChContactable_1vars<6>* mmboB = dynamic_cast<ChContactable_1vars<6>*>(mcontact.modelB->GetContactable());

  if (mmboA && mmboB) {
    if (lastcontact_6_6 != contactlist_6_6.end()) {
      // reuse old contacts
      (*lastcontact_6_6)->Reset(mmboA, mmboB, mcontact);
      lastcontact_6_6++;
    } else {
      // add new contact
      ChContact_6_6* mc = new ChContact_6_6(this, mmboA, mmboB, mcontact);
      contactlist_6_6.push_back(mc);
      lastcontact_6_6 = contactlist_6_6.end();
    }
    n_added_6_6++;
  }
}

}  // END_OF_NAMESPACE____
