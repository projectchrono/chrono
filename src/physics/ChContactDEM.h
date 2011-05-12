#ifndef CHCONTACTDEM_H
#define CHCONTACTDEM_H

///////////////////////////////////////////////////
//
//   ChContactDEM.h
//
//   Classes for enforcing constraints between DEM bodies
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChFrame.h"
#include "lcp/ChLcpConstraintTwoContactN.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "collision/ChCCollisionModel.h"

namespace chrono
{


///
/// Class representing a contact between DEM bodies
///

class ChApi ChContactDEM {

protected:
				//
	  			// DATA
				//
	collision::ChCollisionModel* modA;	///< model A
	collision::ChCollisionModel* modB;  ///< model B

	ChVector<> p1;			///< max penetration point on geo1, after refining, in abs space
	ChVector<> p2;			///< max penetration point on geo2, after refining, in abs space
	ChVector<float> normal;	///< normal, on surface of master reference (geo1)

							///< the plane of contact (X is normal direction)
	ChMatrix33<float> contact_plane;
	
	double norm_dist;	    ///< penetration distance (negative if going inside) after refining
	double knn;  ///< spring coefficient for this contact
	double gnn;  ///< damping coefficient for this contact

	ChVector<> react_force;

public:
				//
	  			// CONSTRUCTORS
				//

	ChContactDEM ();

	ChContactDEM (		collision::ChCollisionModel* mmodA,	///< model A
						collision::ChCollisionModel* mmodB,	///< model B
						const ChLcpVariablesBody* varA, ///< pass A vars
						const ChLcpVariablesBody* varB, ///< pass B vars
						const ChFrame<>* frameA,		///< pass A frame
						const ChFrame<>* frameB,		///< pass B frame
						const ChVector<>& vpA,			///< pass coll.point on A, in absolute coordinates
						const ChVector<>& vpB,			///< pass coll.point on B, in absolute coordinates
						const ChVector<>& vN, 			///< pass coll.normal, respect to A, in absolute coordinates
						double mdistance,				///< pass the distance (negative for penetration)
						float* mreaction_cache,			///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						double  mkn,				///< spring coeff.
						double  mgn				///< damping coeff.
				);

	virtual ~ChContactDEM ();


				//
	  			// FUNCTIONS
				//

					/// Initialize again this constraint.
	virtual void Reset(	collision::ChCollisionModel* mmodA,	///< model A
						collision::ChCollisionModel* mmodB,	///< model B
						const ChLcpVariablesBody* varA, ///< pass A vars
						const ChLcpVariablesBody* varB, ///< pass B vars
						const ChFrame<>* frameA,		///< pass A frame
						const ChFrame<>* frameB,		///< pass B frame
						const ChVector<>& vpA,			///< pass coll.point on A, in absolute coordinates
						const ChVector<>& vpB,			///< pass coll.point on B, in absolute coordinates
						const ChVector<>& vN, 			///< pass coll.normal, respect to A, in absolute coordinates
						double mdistance,				///< pass the distance (negative for penetration)
						float* mreaction_cache,			///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						double  mkn,				///< spring coeff.
						double  mgn				///< damping coeff.
);


					/// Get the contact coordinate system, expressed in absolute frame.
					/// This represents the 'main' reference of the link: reaction forces 
					/// are expressed in this coordinate system. Its origin is point P2.
					/// (It is the coordinate system of the contact plane and normal)
	virtual ChCoordsys<> GetContactCoords();

					/// Returns the pointer to a contained 3x3 matrix representing the UV and normal
					/// directions of the contact. In detail, the X versor (the 1s column of the 
					/// matrix) represents the direction of the contact normal.
	ChMatrix33<float>* GetContactPlane() {return &contact_plane;};


					/// Get the contact point 1, in absolute coordinates
	virtual ChVector<> GetContactP1() {return p1; };

					/// Get the contact point 2, in absolute coordinates
	virtual ChVector<> GetContactP2() {return p2; };

					/// Get the contact normal, in absolute coordinates
	virtual ChVector<float> GetContactNormal()  {return normal; };

					/// Get the contact distance
	virtual double	   GetContactDistance()  {return norm_dist; };
	
					/// Get the contact force, if computed, in absolute coordinates
	virtual ChVector<> GetContactForce() {return react_force; };

					/// Get the spring coefficient
	virtual double GetSpringStiffness() {return knn; };
					/// Set the spring coefficient
	virtual void SetSpringStiffness(float mk) { knn=mk; };
					/// Get the damping coefficient
	virtual double GetDampingStiffness() {return gnn; };
					/// Set the damping coefficient
	virtual void SetDampingStiffness(float mg) { gnn=mg; };

					/// Get the collision model A, with point P1
	virtual collision::ChCollisionModel* GetModelA() {return this->modA;};
					/// Get the collision model B, with point P2
	virtual collision::ChCollisionModel* GetModelB() {return this->modB;};

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
