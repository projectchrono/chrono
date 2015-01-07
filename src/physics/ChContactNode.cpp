//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChContactNode.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChContactNode.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpConstraintNodeContactN.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;


ChContactNode::ChContactNode ()
{ 
	Nx.SetTangentialConstraintU(&Tu);
	Nx.SetTangentialConstraintV(&Tv);
}

ChContactNode::ChContactNode (		collision::ChCollisionModel* mmodA,	///< model A (body)
						collision::ChCollisionModel* mmodB,	///< model B (node)
						const ChLcpVariablesBody* varA, ///< pass B vars (body)
						const ChLcpVariablesNode* varB, ///< pass A vars (node)
						const ChFrame<>*  frameA,		///< pass A frame
						const ChVector<>* posB,			///< pass B position
						const ChVector<>& vpA,			///< pass coll.point on A, in absolute coordinates
						const ChVector<>& vpB,			///< pass coll.point on B, in absolute coordinates
						const ChVector<>& vN, 			///< pass coll.normal, respect to A, in absolute coordinates
						double mdistance,				///< pass the distance (negative for penetration)
						float* mreaction_cache,			///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						float  mfriction				///< friction coeff.
				)
{ 
	Nx.SetTangentialConstraintU(&Tu);
	Nx.SetTangentialConstraintV(&Tv);

	this->Reset(	mmodA, mmodB,
			varA, ///< pass A vars
			varB, ///< pass B vars
			frameA,		  ///< pass A frame
			posB,		  ///< pass B frame
			vpA,		  ///< pass coll.point on A
			vpB,		  ///< pass coll.point on B
			vN, 		  ///< pass coll.normal, respect to A
			mdistance,		  ///< pass the distance (negative for penetration)
			mreaction_cache,	  ///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
			mfriction			  ///< friction coeff.
				);
}

ChContactNode::~ChContactNode ()
{

}

void ChContactNode::Reset(		collision::ChCollisionModel* mmodA,	///< model A (body)
						collision::ChCollisionModel* mmodB,	///< model B (node)
						const ChLcpVariablesBody* varA, ///< pass B vars (body)
						const ChLcpVariablesNode* varB, ///< pass A vars (node)
						const ChFrame<>*  frameA,		///< pass A frame
						const ChVector<>* posB,			///< pass B position
						const ChVector<>& vpA,			///< pass coll.point on A, in absolute coordinates
						const ChVector<>& vpB,			///< pass coll.point on B, in absolute coordinates
						const ChVector<>& vN, 			///< pass coll.normal, respect to A, in absolute coordinates
						double mdistance,				///< pass the distance (negative for penetration)
						float* mreaction_cache,			///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						float  mfriction				///< friction coeff.
				)
{
	assert (varA);
	assert (varB);
	assert (frameA);
	assert (posB);


	this->modA = mmodA;
	this->modB = mmodB;

	Nx.SetVariables(const_cast<ChLcpVariablesBody*>(varA),const_cast<ChLcpVariablesNode*>(varB));
	Tu.SetVariables(const_cast<ChLcpVariablesBody*>(varA),const_cast<ChLcpVariablesNode*>(varB));
	Tv.SetVariables(const_cast<ChLcpVariablesBody*>(varA),const_cast<ChLcpVariablesNode*>(varB));

	Nx.SetFrictionCoefficient(mfriction);

	ChVector<> VN = vN;
	ChVector<double> Vx, Vy, Vz;
	ChVector<double> singul(VECT_Y);
	XdirToDxDyDz(&VN, &singul, &Vx,  &Vy, &Vz);
	contact_plane.Set_A_axis(Vx,Vy,Vz);

	this->p1 = vpA;
	this->p2 = vpB;
	this->normal = vN;
	this->norm_dist = mdistance;
	this->reactions_cache = mreaction_cache;

	ChVector<> Pl1 = frameA->TransformParentToLocal(p1);
	ChVector<> Pl2 = p2  - *posB; 

			// compute jacobians
	ChMatrix33<> Jx1, Jx2, Jr1;
	ChMatrix33<> Ps1, Ps2, Jtemp;
	Ps1.Set_X_matrix(Pl1);
	Ps2.Set_X_matrix(Pl2);

	Jx1.CopyFromMatrixT(this->contact_plane);
	Jx2.CopyFromMatrixT(this->contact_plane);
	Jx1.MatrNeg();

	Jtemp.MatrMultiply(*( const_cast<ChFrame<>*>(frameA)->GetA()), Ps1);
	Jr1.MatrTMultiply(this->contact_plane, Jtemp);

	Nx.Get_Cq_a()->PasteClippedMatrix(&Jx1, 0,0, 1,3, 0,0);
	Tu.Get_Cq_a()->PasteClippedMatrix(&Jx1, 1,0, 1,3, 0,0);
	Tv.Get_Cq_a()->PasteClippedMatrix(&Jx1, 2,0, 1,3, 0,0);
	Nx.Get_Cq_a()->PasteClippedMatrix(&Jr1, 0,0, 1,3, 0,3);
	Tu.Get_Cq_a()->PasteClippedMatrix(&Jr1, 1,0, 1,3, 0,3);
	Tv.Get_Cq_a()->PasteClippedMatrix(&Jr1, 2,0, 1,3, 0,3);

	Nx.Get_Cq_b()->PasteClippedMatrix(&Jx2, 0,0, 1,3, 0,0);
	Tu.Get_Cq_b()->PasteClippedMatrix(&Jx2, 1,0, 1,3, 0,0);
	Tv.Get_Cq_b()->PasteClippedMatrix(&Jx2, 2,0, 1,3, 0,0);

	react_force = VNULL;
}



ChCoordsys<> ChContactNode::GetContactCoords()
{
	ChCoordsys<> mcsys;
	ChQuaternion<float> mrot = this->contact_plane.Get_A_quaternion();
	mcsys.rot.Set(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
	mcsys.pos = this->p2;
	return mcsys;
}


 


void ChContactNode::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	mdescriptor.InsertConstraint(&Nx);
	mdescriptor.InsertConstraint(&Tu);
	mdescriptor.InsertConstraint(&Tv); 
}

void ChContactNode::ConstraintsBiReset()
{
	Nx.Set_b_i(0.);
	Tu.Set_b_i(0.);
	Tv.Set_b_i(0.);
}
 
void ChContactNode::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	if (do_clamp)
		Nx.Set_b_i( Nx.Get_b_i() + ChMax (factor * this->norm_dist, -recovery_clamp)  );
	else
		Nx.Set_b_i( Nx.Get_b_i() + factor * this->norm_dist  );
}

void ChContactNode::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	react_force.x = Nx.Get_l_i() * factor;  
	react_force.y = Tu.Get_l_i() * factor;
	react_force.z = Tv.Get_l_i() * factor;
}


void  ChContactNode::ConstraintsLiLoadSuggestedSpeedSolution()
{
	// Fetch the last computed impulsive reactions from the persistent contact manifold (could
	// be used for warm starting the CCP speed solver): 
	if (this->reactions_cache)
	{
		Nx.Set_l_i(reactions_cache[0]);
		Tu.Set_l_i(reactions_cache[1]);  
		Tv.Set_l_i(reactions_cache[2]);
	} 
}

void  ChContactNode::ConstraintsLiLoadSuggestedPositionSolution()
{
	// Fetch the last computed 'positional' reactions from the persistent contact manifold (could
	// be used for warm starting the CCP position stabilization solver):
	if (this->reactions_cache)
	{
		Nx.Set_l_i(reactions_cache[3]);
		Tu.Set_l_i(reactions_cache[4]);  
		Tv.Set_l_i(reactions_cache[5]);
	}
}

void  ChContactNode::ConstraintsLiFetchSuggestedSpeedSolution()
{
	// Store the last computed reactions into the persistent contact manifold (might
	// be used for warm starting CCP the speed solver):
	if (reactions_cache)
	{
		reactions_cache[0] = (float)Nx.Get_l_i();
		reactions_cache[1] = (float)Tu.Get_l_i();
		reactions_cache[2] = (float)Tv.Get_l_i();
	}
}

void  ChContactNode::ConstraintsLiFetchSuggestedPositionSolution()
{
	// Store the last computed 'positional' reactions into the persistent contact manifold (might
	// be used for warm starting the CCP position stabilization solver):
	if (reactions_cache)
	{
		reactions_cache[3] = (float)Nx.Get_l_i();
		reactions_cache[4] = (float)Tu.Get_l_i();
		reactions_cache[5] = (float)Tv.Get_l_i();
	}
}












} // END_OF_NAMESPACE____


