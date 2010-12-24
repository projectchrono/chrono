#ifndef CHLINKMARKERS_H
#define CHLINKMARKERS_H

///////////////////////////////////////////////////
//
//   ChLinkMarkers.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChLink.h"
#include "physics/ChMarker.h"



namespace chrono
{



///
/// Class for links which connect two 'markers'. The markers are two
/// ChMarker objects each belonging to the two linked ChBody parts.
/// Many specialized classes are based on this ChLinkMarkers class, for example
/// the ChLinkSpring and all the family of the ChLinkLock classes - see them-.
/// Also, ChLinkMarkers class allows an optional force vector and torque vector
/// to be set between the two connected markers.
///

class ChApi ChLinkMarkers : public ChLink {

	CH_RTTI(ChLinkMarkers,ChLink);

protected:

				//
	  			// DATA
				//

	ChMarker* marker1;	// slave coordsys
	ChMarker* marker2;	// master coordsys, =0 if liked to ground

	int markID1;		// unique identifier for markers 1 & 2,
	int markID2;		// when using plugin dynamic hierarchies

	Coordsys relM;		// relative marker position 2-1
	Coordsys relM_dt;	// relative marker speed
	Coordsys relM_dtdt; // relative marker acceleration

	double relAngle;	// relative angle of rotation
	Vector relAxis;		// relative axis of rotation
	Vector relRotaxis;	// relative rotaion vector =angle*axis
	Vector relWvel;		// relative angular speed
	Vector relWacc;		// relative angular acceleration
	double dist;		// the distance between the two origins of markers,
	double dist_dt;		// the speed between the two  origins of markers

	Vector Scr_force;	// internal force  set by script only (just added to C_force)
	Vector Scr_torque;	// internal torque set by script only (just added to C_force)
	Vector C_force;		// internal force  applied by springs/dampers/actuators
	Vector C_torque;	// internal torque applied by springs/dampers/actuators


public:
				//
	  			// CONSTRUCTORS
				//
	ChLinkMarkers ();
	virtual ~ChLinkMarkers ();
	virtual void Copy(ChLinkMarkers* source);
	virtual ChLink* new_Duplicate ();  


public:
				//
	  			// FUNCTIONS
				//

					/// Get the type identifier of this link. Use if you don't want to use RTTI for performance.
	virtual int GetType	() {return LNK_BASE;}

					/// Return the 1st referenced marker (the 'slave' marker, owned by 1st body)
	ChMarker* GetMarker1 () {return marker1;}
					/// Return the 2nd referenced marker (the 'master' marker, owned by 2nd body)
	ChMarker* GetMarker2 () {return marker2;}
					/// Set the 1st referenced marker (the 'slave' marker, owned by 1st body)
	virtual void SetMarker1 (ChMarker* mark1);
					/// Set the 2nd referenced marker (the 'master' marker, owned by 2nd body)
	virtual void SetMarker2 (ChMarker* mark2);
					/// Set both constrained markers at once. Note that also Body1 and Body2 are
					/// automatically set (they are of course the owners of the two markers)
	void SetMarkers (ChMarker* mark1, ChMarker* mark2);
					/// Exchange the master and the slave marker. The same happens for 
					/// body1 and body2, automatically.
	void SwapMainSlaveMarkers();
					
	void SetMarkID1(int mid) {markID1 = mid;}
	void SetMarkID2(int mid) {markID2 = mid;}
	int GetMarkID1() {return markID1;}
	int GetMarkID2() {return markID2;}

					/// Shortcut: performs  SetMarkers(), and SetMarkID1() SetMarkID2() at once.
	int ReferenceMarkers(ChMarker* mark1, ChMarker* mark2);

					/// Use this function after link creation, to initialize the link from 
					/// two markers to join. 
					/// Each marker must belong to a rigid body, and both rigid bodies 
					/// must belong to the same ChSystem. 
					/// The position of mark2 is used as link's position and main reference.
	virtual int Initialize(ChSharedMarkerPtr& mark1, ///< first  marker to join
						   ChSharedMarkerPtr& mark2	 ///< second marker to join (master)
						   );

					/// Use this function after link creation, to initialize the link from 
					/// two joined rigid bodies. 
					/// Both rigid bodies must belong to the same ChSystem. 
					/// Two markers will be created and added to the rigid bodies (later,
					/// you can use GetMarker1() and GetMarker2() to access them.
					/// To specify the (absolute) position of link and markers, use 'mpos'.
	virtual int Initialize(ChSharedBodyPtr& mbody1, ///< first  body to join
						   ChSharedBodyPtr& mbody2, ///< second body to join 
						   ChCoordsys<> mpos		///< the current absolute pos.& alignment.
						   );

					/// Use this function after link creation, to initialize the link from 
					/// two joined rigid bodies. 
					/// Both rigid bodies must belong to the same ChSystem. 
					/// Two markers will be created and added to the rigid bodies (later,
					/// you can use GetMarker1() and GetMarker2() to access them.
					/// To specify the (absolute) position of link and markers, use 'mpos'.
	virtual int Initialize(ChSharedBodyPtr& mbody1, ///< first  body to join
						   ChSharedBodyPtr& mbody2, ///< second body to join  
						   bool pos_are_relative,	///< if =true, following two positions are relative to bodies. If false, are absolute.
						   ChCoordsys<> mpos1, 		///< the position & alignment of 1st marker (relative to body1 cords, or absolute)
						   ChCoordsys<> mpos2		///< the position & alignment of 2nd marker (relative to body2 cords, or absolute)
						   );

					/// Get the link coordinate system, expressed relative to Body2 (the 'master'
					/// body). This represents the 'main' reference of the link: reaction forces 
					/// and torques are expressed in this coordinate system.
					/// (It is the coordinate system of the 'master' marker2 relative to Body2)
	ChCoordsys<> GetLinkRelativeCoords() {return this->marker2->GetCoord();};


			//
			// UPDATING FUNCTIONS
			//



					/// Updates auxiliary vars relM, relM_dt, relM_dtdt, 
					/// dist, dist_dt et similia.
	virtual void UpdateRelMarkerCoords();

					///  Updates auxiliary forces caused by springs/dampers/etc. which may
					/// be connected between the two bodies of the link. 
					/// By default, it adds the forces which might have been added by the
					/// user using Set_Scr_force() and Set_Scr_torque(). Note, these forces 
					/// are considered in the reference coordsystem of marker2 (the MAIN marker), 
					/// and their application point is the origin of marker1 (the SLAVE marker).
	virtual void UpdateForces (double mytime);



					// -----------COMPLETE UPDATE.
					// sequence:
					//			UpdateTime;
					//          UpdateRelMarkerCoords;
					//			UpdateForces;

	virtual void Update (double mytime);



			//
			// LCP INTERFACE
			//

				/// Overrides the empty behaviour of the parent ChLink implementation, which
				/// does not consider any user-imposed force between the two bodies.
				/// It adds the current link-forces, if any, (caused by springs, etc.) to the 'fb' vectors
				/// of the ChLcpVariables referenced by encapsulated ChLcpConstraints.
				/// In details, it adds the effect caused by C_force and C_torque.
				/// Both C_force and C_torque these forces are considered expressed in the 
				/// reference coordsystem of marker2 (the MAIN marker), 
				/// and their application point is the origin of marker1 (the SLAVE marker).
	virtual void ConstraintsFbLoadForces(double factor=1.);


			//
			// LINK COORDINATES and other functions:
			// 

					/// Relative position of marker 1 respect to marker 2.
	Coordsys GetRelM () {return relM;}
					/// Relative speed of marker 1 respect to marker 2.
	Coordsys GetRelM_dt () {return relM_dt;}
					/// Relative acceleration of marker 1 respect to marker 2.
	Coordsys GetRelM_dtdt () {return relM_dtdt;}
					/// Relative rotation angle of marker 1 respect to marker 2 (best with revolute joints..).
	double	 GetRelAngle () {return relAngle;}
					/// Relative finite rotation axis of marker 1 respect to marker 2.
	Vector   GetRelAxis	() {return relAxis;}
	Vector   GetRelRotaxis	() {return relRotaxis;}
					/// Relative angular speed of marker 1 respect to marker 2.
	Vector   GetRelWvel () {return relWvel;}
					/// Relative angular acceleration of marker 1 respect to marker 2.
	Vector   GetRelWacc () {return relWacc;}
					/// Relative 'polar' distance of marker 1 respect to marker 2.
	double	 GetDist() {return dist;}
					/// Relative speed of marker 1 respect to marker 2, along the polar distance vector.
	double	 GetDist_dt() {return dist_dt;}



					/// To get & set the 'script' force buffers(only accessed by
					/// external scripts, so it's up to the script to remember
					/// to set& reset them -link class just add them to C_force etc.
	Vector* Get_Scr_force() {return &Scr_force;};
	Vector* Get_Scr_torque() {return &Scr_torque;};
	void Set_Scr_force(Vector mf) {Scr_force = mf;};
	void Set_Scr_torque(Vector mf) {Scr_torque = mf;};

					/// Get the total applied force accumulators (force, momentum) in link coords.
					/// These forces might be affected by additional springs, dampers, etc. but they do not
					/// include the reaction forces.
	Vector GetC_force() {return C_force;}
	Vector GetC_torque() {return C_torque;}




			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);

					/// Method to allow serialization of transient data in ascii,
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	virtual void StreamOUT(ChStreamOutAscii& mstream) {};

};





} // END_OF_NAMESPACE____

#endif
