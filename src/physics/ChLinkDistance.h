#ifndef CHLINKDISTANCE_H
#define CHLINKDISTANCE_H

///////////////////////////////////////////////////
//
//   ChLinkDistance.h
//
//   Class for enforcing a fixed polar distance 
//   between two points on two ChBody objects.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChLinkGeometric.h"
#include "lcp/ChLcpConstraintTwoBodies.h"

namespace chrono
{

// Unique link identifier, for detecting type faster than with rtti.
#define LNK_GEOMETRICDISTANCE	37


/// Class for enforcing a fixed polar distance 
/// between two points on two ChBody objects.
/// The two points which are used to define the end points
/// of the distance are assumed not to move respect to the
/// two owner ChBody, as well as the amount of the distance
/// is assumed not to change during the simulation. If you
/// need to have a time-varying distance, or distance between
/// two points which move respect to the bodies, please use 
/// the more advanced ChLinkLinActuator.

class ChLinkDistance : public ChLinkGeometric {

	CH_RTTI(ChLinkDistance,ChLinkGeometric);

protected:
				//
	  			// DATA
				//
							// the imposed distance
	double	distance;	
							// the distance endpoints, in body rel.coords
	ChVector<> pos1;
	ChVector<> pos2;
							// the constraint object
	ChLcpConstraintTwoBodies Cx;

	double curr_dist;		// used for internal optimizations
	
	float cache_li_speed;	// used to cache the last computed value of multiplier (solver warm starting)
	float cache_li_pos;		// used to cache the last computed value of multiplier (solver warm starting)

public:
				//
	  			// CONSTRUCTORS
				//

	ChLinkDistance ();

	virtual ~ChLinkDistance ();
	virtual void Copy(ChLinkDistance* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer


				//
	  			// FUNCTIONS
				//

	virtual int GetType	() {return LNK_GEOMETRICDISTANCE;}

					/// Initialize this constraint, given the two bodies to be connected, the
					/// positions of the two anchor endpoints of the distance (each expressed
					/// in body or abs. coordinates) and the imposed distance.
	virtual int Initialize(ChSharedBodyPtr& mbody1,		 ///< first body to link
						   ChSharedBodyPtr& mbody2,		 ///< second body to link
						   bool pos_are_relative,///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChVector<> mpos1,	 ///< position of distance endpoint, for 1st body (rel. or abs., see flag above)
						   ChVector<> mpos2,	 ///< position of distance endpoint, for 2nd body (rel. or abs., see flag above) 
						   bool auto_distance =true,///< if true, initializes the imposed distance as the distance between mpos1 and mpos2
						   double mdistance =0   ///< imposed distance (no need to define, if auto_distance=true.)
						   );

					/// Get the number of scalar constraints imposed by this link (only unilateral constr.)
	virtual int GetDOC_d  () {return 1;}

					/// Get the link coordinate system, expressed relative to Body2 (the 'master'
					/// body). This represents the 'main' reference of the link: reaction forces 
					/// are expressed in this coordinate system.
					/// (It is the coordinate system of the contact plane relative to Body2)
	ChCoordsys<> GetLinkRelativeCoords();

					/// Get the 1st anchor endpoint for the distance (expressed in Body1 coordinate system)
	ChVector<> GetEndPoint1Rel() {return pos1;}
					/// Set the 1st anchor endpoint for the distance (expressed in Body1 coordinate system)
	void       SetEndPoint1Rel(const ChVector<>& mset) {pos1 = mset;}
					/// Get the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
	ChVector<> GetEndPoint1Abs() {return Body1->Point_Body2World(&pos1);}
					/// Set the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
	void       SetEndPoint1Abs(ChVector<>& mset) {pos1 = Body1->Point_World2Body(&mset);}

					/// Get the 2nd anchor endpoint for the distance (expressed in Body2 coordinate system)
	ChVector<> GetEndPoint2Rel() {return pos2;};
					/// Set the 2nd anchor endpoint for the distance (expressed in Body2 coordinate system)
	void       SetEndPoint2Rel(const ChVector<>& mset) {pos2 = mset;}
					/// Get the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
	ChVector<> GetEndPoint2Abs() {return Body2->Point_Body2World(&pos2);}
					/// Set the 1st anchor endpoint for the distance (expressed in absolute coordinate system)
	void       SetEndPoint2Abs(ChVector<>& mset) {pos2 = Body2->Point_World2Body(&mset);}


					/// Get the imposed distance
	double GetImposedDistance() {return distance;};
					/// Set the imposed distance
	void   SetImposedDistance(const double mset) {distance = mset;}
					/// Get the distance currently existing between the two endpoints
	double GetCurrentDistance() {return (Body1->Point_Body2World(&pos1)-Body2->Point_Body2World(&pos2)).Length(); };


				//
				// UPDATING FUNCTIONS
				//

					/// Override _all_ time, jacobian etc. updating.
					/// In detail, it computes jacobians, violations, etc. and stores 
					/// results in inner structures.
	virtual void Update (double mtime);


				//
				// LCP INTERFACE
				//

	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
	virtual void ConstraintsBiReset();
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);
	virtual void ConstraintsLoadJacobians();
	virtual void ConstraintsFetch_react(double factor=1.);
	virtual void ConstraintsLiLoadSuggestedSpeedSolution();
	virtual void ConstraintsLiLoadSuggestedPositionSolution();
	virtual void ConstraintsLiFetchSuggestedSpeedSolution();
	virtual void ConstraintsLiFetchSuggestedPositionSolution();

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
