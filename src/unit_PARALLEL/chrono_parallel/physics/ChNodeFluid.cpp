#include <stdlib.h>
#include <algorithm>

#include "chrono_parallel/physics/ChNodeFluid.h"
#include "chrono_parallel/collision/ChCCollisionModelSphere.h"

#include "physics/ChSystem.h"

#include "core/ChLinearAlgebra.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono {

using namespace collision;
using namespace geometry;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A SPH NODE

ChNodeFluid::ChNodeFluid() {
   this->collision_model = new ChCollisionModelSphere;

   this->UserForce = VNULL;
   this->h_rad = 0.1;
   this->coll_rad = 0.001;
   this->SetMass(0.01);
   this->volume = 0.01;
   this->density = this->GetMass() / this->volume;
   this->pressure = 0;

}

ChNodeFluid::~ChNodeFluid() {
   delete collision_model;
}

ChNodeFluid::ChNodeFluid(const ChNodeFluid& other)
      : ChNodeXYZ(other) {
   this->collision_model = new ChCollisionModelSphere;
   this->collision_model->AddSphere(other.coll_rad);

   this->UserForce = other.UserForce;
   this->SetKernelRadius(other.h_rad);
   this->SetCollisionRadius(other.coll_rad);
   this->SetMass(other.GetMass());
   this->volume = other.volume;
   this->density = other.density;
   this->pressure = other.pressure;
   this->h_rad = other.h_rad;
   this->coll_rad = other.coll_rad;
   this->variables = other.variables;
}

ChNodeFluid& ChNodeFluid::operator=(const ChNodeFluid& other) {
   if (&other == this)
      return *this;

   ChNodeXYZ::operator=(other);

   this->collision_model->ClearModel();
   this->collision_model->AddSphere(other.coll_rad);

   this->UserForce = other.UserForce;
   this->SetKernelRadius(other.h_rad);
   this->SetCollisionRadius(other.coll_rad);
   this->SetMass(other.GetMass());
   this->volume = other.volume;
   this->density = other.density;

   this->variables = other.variables;

   return *this;
}

void ChNodeFluid::SetKernelRadius(real mr) {
   h_rad = mr;
}

void ChNodeFluid::SetCollisionRadius(real mr) {
   coll_rad = mr;
   ((ChCollisionModelSphere*) this->collision_model)->SetSphereRadius(coll_rad);
}

}  // END_OF_NAMESPACE____

/////////////////////
