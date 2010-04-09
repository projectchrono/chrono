#ifndef CHC_MODELGPUBODY_H
#define CHC_MODELGPUBODY_H
 
//////////////////////////////////////////////////
//  
//   ChCModelGPUBody.h
//
//   A wrapper to use the GPU collision detection
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "collision/ChCModelGPU.h"


namespace chrono 
{

// forward references
class ChBody;

namespace collision 
{


///  Class for the collision model to be used in ChBody shapes,
///  using features of the Bullet library.

class ChModelGPUBody : public ChModelGPU
{

public:

  ChModelGPUBody();
  virtual ~ChModelGPUBody();
  

    	/// Gets the pointer to the client owner rigid body. 
  ChBody* GetBody() const {return mbody;};
		/// Sets the pointer to the client owner rigid body
  void SetBody(ChBody* mbo) {mbody = mbo;};

  
	// Overrides and implementations of base members:

		/// Sets the position and orientation of the collision
		/// model as the current position of the corresponding ChBody
  virtual void SyncPosition();

  		/// Gets the pointer to the client owner ChPhysicsItem. 
  virtual ChPhysicsItem* GetPhysicsItem() {return (ChPhysicsItem*)GetBody();};

private:
	ChBody* mbody;
};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
