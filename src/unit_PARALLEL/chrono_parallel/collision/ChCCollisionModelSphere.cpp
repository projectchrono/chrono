//////////////////////////////////////////////////
//
//   ChCModelGPU.cpp
//
// ------------------------------------------------
//       Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_parallel/collision/ChCCollisionModelSphere.h"
#include "physics/ChBody.h"
#include "physics/ChSystem.h"

namespace chrono {
namespace collision {

ChCollisionModelSphere::ChCollisionModelSphere(real rad) {
   nObjects = 0;
   colFam = -1;
   noCollWith = -2;
   inertia = R3(0);
   total_volume = 0;
   mData.resize(1);

   model_type = SPHERE;
   nObjects = 1;
   radius = rad;
   bData tData;
   tData.A = R3(0);
   tData.B = R3(radius, 0, 0);
   tData.C = R3(0, 0, 0);
   tData.R = R4(1, 0, 0, 0);
   tData.type = SPHERE;
   mData[0] = tData;
   total_volume = 4.0 / 3.0 * CH_C_PI * pow(radius, 3.0);

}

ChCollisionModelSphere::~ChCollisionModelSphere() {
   mData.clear();
}

bool ChCollisionModelSphere::AddSphere(real radius,
                                       const ChVector<> &pos) {

   model_type = SPHERE;
   nObjects = 1;
   bData tData;
   tData.A = R3(0);
   tData.B = R3(radius, 0, 0);
   tData.C = R3(0, 0, 0);
   tData.R = R4(1, 0, 0, 0);
   tData.type = SPHERE;
   mData[0] = tData;
   total_volume = 4.0 / 3.0 * CH_C_PI * pow(radius, 3.0);

   return true;
}

void ChCollisionModelSphere::SetSphereRadius(real sphere_radius) {
   mData[0].B.x = sphere_radius;
}

}     // END_OF_NAMESPACE____
}     // END_OF_NAMESPACE____

