#include <stdlib.h>
#include <algorithm>

#include "chrono_parallel/physics/ChNodeFluid.h"
#include "physics/ChSystem.h"
#include "core/ChLinearAlgebra.h"
#include "core/ChMemory.h"  // must be last include (memory leak debugger). In .cpp only.

namespace chrono {

using namespace collision;
using namespace geometry;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A 3DOF FLUID NODE

ChNodeFluid::ChNodeFluid(real r) {
  this->coll_rad = r;
  this->pressure = 0;
  this->density = 1000;
  body_id = 0;
}

ChNodeFluid::~ChNodeFluid() {
}

ChNodeFluid::ChNodeFluid(const ChNodeFluid& other) : ChPhysicsItem(other) {
  this->pos = other.pos;
  this->pos_dt = other.pos_dt;
  this->pos_dtdt = other.pos_dtdt;
  this->body_id = other.body_id;

  this->SetCollisionRadius(other.coll_rad);
  // this->mass = other.mass;
  this->density = other.density;
  this->pressure = other.pressure;
  this->coll_rad = other.coll_rad;
}

ChNodeFluid& ChNodeFluid::operator=(const ChNodeFluid& other) {
  if (&other == this)
    return *this;

  ChPhysicsItem::operator=(other);

  this->pos = other.pos;
  this->pos_dt = other.pos_dt;
  this->pos_dtdt = other.pos_dtdt;

  this->SetCollisionRadius(other.coll_rad);

  this->density = other.density;
  // this->mass = other.mass;
  return *this;
}

void ChNodeFluid::SetCollisionRadius(real mr) {
  coll_rad = mr;
}

void ChNodeFluid::AddCollisionModelsToSystem() {
}

}  // END_OF_NAMESPACE____

/////////////////////
