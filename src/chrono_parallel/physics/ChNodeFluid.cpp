#include <stdlib.h>
#include <algorithm>

#include "chrono_parallel/physics/ChNodeFluid.h"
#include "physics/ChSystem.h"
#include "core/ChLinearAlgebra.h"

namespace chrono {

using namespace collision;
using namespace geometry;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A 3DOF FLUID NODE

ChNodeFluid::ChNodeFluid(real r) {
  body_id = 0;
  data_manager = 0;
}

ChNodeFluid::ChNodeFluid(const ChNodeFluid& other) : ChPhysicsItem(other) {
  this->body_id = other.body_id;
  this->data_manager = other.data_manager;
}

ChNodeFluid::~ChNodeFluid() {
}

ChNodeFluid& ChNodeFluid::operator=(const ChNodeFluid& other) {
  if (&other == this)
    return *this;

  ChPhysicsItem::operator=(other);
  return *this;
}

}  // END_OF_NAMESPACE____

/////////////////////
