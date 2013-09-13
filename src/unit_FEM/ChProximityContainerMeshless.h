#ifndef CHPROXIMITYCONTAINERMESHLESS_H
#define CHPROXIMITYCONTAINERMESHLESS_H

///////////////////////////////////////////////////
//
//   ChProximityContainerMeshless.h
//
//   Class for container of many proximity pairs for SPH (Smooth 
//   Particle Hydrodinamics and similar meshless force computations), 
//   as CPU typical linked list of ChProximitySPH objects
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChProximityContainerBase.h"
#include "collision/ChCModelBulletNode.h"
#include <list>

namespace chrono
{



///
/// Class for a proximity pair information in a meshless deformable continumm,
/// made with a cluster of particles - that is, an 'edge' topological connectivity in
/// in a meshless FEM approach, similar to the Smoothed Particle Hydrodynamics.
/// 

class ChApiFem ChProximityMeshless
{
public:
	ChProximityMeshless(collision::ChModelBulletNode* mmodA,	///< model A
				   collision::ChModelBulletNode* mmodB)	///< model B
	{
		Reset(mmodA, mmodB);
	}

	virtual ~ChProximityMeshless () {};

				//
	  			// FUNCTIONS
				//

					/// Initialize again this constraint.
	virtual void Reset(	collision::ChModelBulletNode* mmodA,	///< model A
						collision::ChModelBulletNode* mmodB) ///< model B
	{
		assert (mmodA);
		assert (mmodB);

		this->modA = mmodA;
		this->modB = mmodB;
	}

					/// Get the collision model A, with point P1
	virtual collision::ChCollisionModel* GetModelA() {return this->modA;}
					/// Get the collision model B, with point P2
	virtual collision::ChCollisionModel* GetModelB() {return this->modB;}

private:
				//
	  			// DATA
				//
	collision::ChCollisionModel* modA;	///< model A
	collision::ChCollisionModel* modB;  ///< model B
};



///
/// Class for container of many proximity pairs for a meshless
/// deformable continuum (necessary for inter-particle material forces), 
/// as CPU typical linked list of ChProximityMeshless objects.
/// Such an item must be addd to the physical system if you added
/// an object of class ChMatterMeshless.
///

class ChApiFem ChProximityContainerMeshless : public ChProximityContainerBase {

	CH_RTTI(ChProximityContainerMeshless,ChProximityContainerBase);

protected:
				//
	  			// DATA
				//

	std::list<ChProximityMeshless*>   proximitylist; 

	int n_added;

	std::list<ChProximityMeshless*>::iterator lastproximity;


public:
				//
	  			// CONSTRUCTORS
				//

	ChProximityContainerMeshless ();

	virtual ~ChProximityContainerMeshless ();



				//
	  			// FUNCTIONS
				//


					/// Tell the number of added contacts
	virtual int GetNproximities  () {return n_added;}

					/// Remove (delete) all contained contact data.
	virtual void RemoveAllProximities();

					/// The collision system will call BeginAddProximities() before adding
					/// all pairs (for example with AddProximity() or similar). Instead of
					/// simply deleting all list of the previous pairs, this optimized implementation
					/// rewinds the link iterator to begin and tries to reuse previous pairs objects
					/// until possible, to avoid too much allocation/deallocation.
	virtual void BeginAddProximities();

					/// Add a proximity SPH data between two collision models, if possible.
	virtual void AddProximity(collision::ChCollisionModel* modA, ///< get contact model 1
							  collision::ChCollisionModel* modB  ///< get contact model 2
							  );

					/// The collision system will call BeginAddContact() after adding
					/// all contacts (for example with AddContact() or similar). This optimized version
					/// purges the end of the list of contacts that were not reused (if any).
	virtual void EndAddProximities();

					/// Scans all the proximity pairs of SPH type and for each pair executes the ReportProximityCallback()
					/// function of the user object inherited from ChReportProximityCallback.
	virtual void ReportAllProximities(ChReportProximityCallback* mcallback);



					// Perform some SPH per-edge initializations and accumulations of values 
					// into the connected pairs of particles (summation into partcle's  J, Amoment, m_v, UserForce -viscous only- )
					// Will be called by the ChMatterMeshless item.
	virtual void AccumulateStep1();

					// Perform some SPH per-edge transfer of forces, given stress tensors in A B nodes
					// Will be called by the ChMatterMeshless item.
	virtual void AccumulateStep2();
	

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
