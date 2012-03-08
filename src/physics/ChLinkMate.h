#ifndef CHLINKMATE_H
#define CHLINKMATE_H

///////////////////////////////////////////////////
//
//   ChLinkMate.h
//
//   Classes for enforcing constraints of simple
//   types (geometric mating)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

//***WORK IN PROGRESS***

#include "physics/ChLink.h"
#include "physics/ChLinkMask.h"


namespace chrono
{



// Unique link identifier, for detecting type faster than with rtti. (Obsolete)
#define LNK_MATE	41

///
/// Base class for all 'simple' constraints between 
/// two frames attached to two bodies. These constraints 
/// can correspond to the typical 'mating' conditions that
/// are created in assemblies of 3D CAD tools (parallel
/// axis, or face-to-face, etc.).
/// Note that most of the ChLinkMate constraints can be
/// done also with the contraints inherited from ChLinkLock...
/// but in case of links of the ChLinkLock class they 
/// reference two ChMarker objects, tht can also move, but
/// this is could be an unnecessary complication in most cases.
///

class ChApi ChLinkMate : public ChLink {

	CH_RTTI(ChLinkMate,ChLink);

protected:
				//
	  			// DATA
				//

public:
				//
	  			// CONSTRUCTORS
				//

	ChLinkMate () {};
	virtual ~ChLinkMate () {};
	virtual void Copy(ChLinkMate* source) {};
	virtual ChLink* new_Duplicate ();	// always return base link class pointer


				//
	  			// FUNCTIONS
				//

	virtual int GetType	() {return LNK_MATE;}

				//
				// STREAMING
				//

	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutBinary& mstream);
	virtual void StreamOUT(ChStreamOutAscii& mstream) {};
};



//////////////////////////////////////////////////////
//////////////////////////////////////////////////////



// Unique link identifier, for detecting type faster than with rtti. (Obsolete)
#define LNK_MATEGENERIC	42

///
/// Generic mate constraint, where one can select which 
/// DOF must be constrained between two frames attached to 
/// the two bodies.
///


class ChApi ChLinkMateGeneric : public ChLinkMate {

	CH_RTTI(ChLinkMateGeneric,ChLinkMate);

protected:
				//
	  			// DATA
				//

	ChFrame<> frameA;
	ChFrame<> frameB;

	bool c_x;
	bool c_y;
	bool c_z;
	bool c_rx;
	bool c_ry;
	bool c_rz;

	int ndoc;			// number of DOC, degrees of costraint
	int ndoc_c;			// number of DOC, degrees of costraint (only bilaterals)
	int ndoc_d;			// number of DOC, degrees of costraint (only unilaterals)

	ChLinkMask* mask;
	
	ChMatrix<>* C; // residuals

	ChMatrix<>* cache_li_pos;
	ChMatrix<>* cache_li_speed;

public:
				//
	  			// CONSTRUCTORS
				//

	ChLinkMateGeneric (bool mc_x=true, bool mc_y=true, bool mc_z=true, bool mc_rx=true, bool mc_ry=true, bool mc_rz=true);
	virtual ~ChLinkMateGeneric ();
	virtual void Copy(ChLinkMateGeneric* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer

				//
	  			// FUNCTIONS
				//

	virtual int GetType	() {return LNK_MATEGENERIC;}

					/// Get the link coordinate system, expressed relative to Body2 (the 'master'
					/// body). This represents the 'main' reference of the link: reaction forces 
					/// are expressed in this coordinate system.
					/// (It is the coordinate system of the contact plane relative to Body2)
	virtual ChCoordsys<> GetLinkRelativeCoords() {return frameB.GetCoord();};

					/// Access the coordinate system considered attached to body1.
					/// Its position is expressed in the coordinate system of body1.
	ChFrame<>& GetFrameA() {return frameA;};

					/// Access the coordinate system considered attached to body1.
					/// Its position is expressed in the coordinate system of body1.
	ChFrame<>& GetFrameB() {return frameB;};

	bool IsConstrainedX() {return c_x;}
	bool IsConstrainedY() {return c_y;}
	bool IsConstrainedZ() {return c_z;}
	bool IsConstrainedRx() {return c_rx;}
	bool IsConstrainedRy() {return c_ry;}
	bool IsConstrainedRz() {return c_rz;}

					/// Sets which movements (of frame 1 respect to frame 2) are constrained
	virtual void SetConstrainedCoords(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz);

					/// Specialized initialization for generic mate, given the two bodies to be connected, the
					/// positions of the two frames to connect on the bodies (each expressed
					/// in body or abs. coordinates).
	virtual int Initialize(ChSharedBodyPtr& mbody1,	///< first body to link
						   ChSharedBodyPtr& mbody2, ///< second body to link
						   bool pos_are_relative,	///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChFrame<> mframe1,		///< mate frame (slave), for 1st body (rel. or abs., see flag above)
						   ChFrame<> mframe2  		///< mate frame (master), for 2nd body (rel. or abs., see flag above) 
						   );


					/// Initialization based on passing two vectors (point + dir) on the 
					/// two bodies, they will represent the X axes of the two frames (Y and Z will
					/// be built from the X vector via Gramm Schmidt orthonomralization). 
					/// Use the other ChLinkMateGeneric::Initialize() if you want to set the two frames directly.
	virtual int Initialize(ChSharedBodyPtr& mbody1,	///< first body to link
						   ChSharedBodyPtr& mbody2, ///< second body to link
						   bool pos_are_relative,	///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChVector<> mpt1,			///< origin of slave frame 1, for 1st body (rel. or abs., see flag above)
						   ChVector<> mpt2,  		///< origin of master frame 2, for 2nd body (rel. or abs., see flag above)
						   ChVector<> mnorm1,		///< X axis of slave plane, for 1st body (rel. or abs., see flag above)
						   ChVector<> mnorm2  		///< X axis of master plane, for 2nd body (rel. or abs., see flag above)
						   );



				//
				// UPDATING FUNCTIONS
				//

					/// Override _all_ time, jacobian etc. updating.
	virtual void Update (double mtime);

						/// If some constraint is redundant, return to normal state
	virtual int  RestoreRedundant();		   ///< \return number of changed states

					/// Set the status of link validity
	virtual void SetValid(bool mon) {valid = mon;}

					/// User can use this to enable/disable all the constraint of
					/// the link as desired.
	virtual void SetDisabled(bool mdis);

					/// Ex:3rd party software can set the 'broken' status via this method
	virtual void SetBroken(bool mon);

	virtual int GetDOC  () {return ndoc;}
	virtual int GetDOC_c  () {return ndoc_c;}
	virtual int GetDOC_d  () {return ndoc_d;}

				//
				// LCP INTERFACE
				//

	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
	virtual void ConstraintsBiReset();
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);
	virtual void ConstraintsBiLoad_Ct(double factor=1.);
	virtual void ConstraintsLoadJacobians();
	virtual void ConstraintsLiLoadSuggestedSpeedSolution();
	virtual void ConstraintsLiLoadSuggestedPositionSolution();
	virtual void ConstraintsLiFetchSuggestedSpeedSolution();
	virtual void ConstraintsLiFetchSuggestedPositionSolution();
	virtual void ConstraintsFetch_react(double factor=1.);

protected:
	void SetupLinkMask();
	void ChangedLinkMask();

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////






// Unique link identifier, for detecting type faster than with rtti. (Obsolete)
#define LNK_MATEPLANE	43

///
/// Mate constraint of plane-to-plane type. This correspond to the
/// typical planar face vs planar face mating used in 3D CAD assemblies.
/// The planes are defined by the Y and Z axes of the two frames.
///

class ChApi ChLinkMatePlane : public ChLinkMateGeneric {

	CH_RTTI(ChLinkMatePlane,ChLinkMateGeneric);

protected:

	bool	flipped;
	double	separation;

public:
				//
	  			// CONSTRUCTORS
				//

	ChLinkMatePlane () : ChLinkMateGeneric(true, false, false, false, true,true) {flipped = false; separation=0;};
	virtual ~ChLinkMatePlane () {};
	virtual void Copy(ChLinkMatePlane* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer

				//
	  			// FUNCTIONS
				//

	virtual int GetType	() {return LNK_MATEPLANE;}

					/// Tell if the two normals must be opposed (flipped=false) or must have the same verse (flipped=true)
	void SetFlipped(bool doflip);
	bool IsFlipped() {return flipped;}

					/// Set the distance between the two planes, in normal direction
	void SetSeparation(double msep) {separation = msep;}
					/// Get the requested distance between the two planes, in normal direction
	double GetSeparation() {return separation;}


					/// Specialized initialization for plane-plane mate, given the two bodies to be connected,
					/// two points on the two faces, two normals on the faces (each expressed
					/// in body or abs. coordinates). 
					/// Use ChLinkMateGeneric::Initialize() if you want to set the two frames directly.
	virtual int Initialize(ChSharedBodyPtr& mbody1,	///< first body to link
						   ChSharedBodyPtr& mbody2, ///< second body to link
						   bool pos_are_relative,	///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChVector<> mpt1,			///< point on slave plane, for 1st body (rel. or abs., see flag above)
						   ChVector<> mpt2,  		///< point on master plane, for 2nd body (rel. or abs., see flag above)
						   ChVector<> mnorm1,		///< normal of slave plane, for 1st body (rel. or abs., see flag above)
						   ChVector<> mnorm2  		///< normal of master plane, for 2nd body (rel. or abs., see flag above)
						   );

					/// Override _all_ time, jacobian etc. updating, inheriting parent but also adding the effect of separation
	virtual void Update (double mtime);

};







//////////////////////////////////////////////////////
//////////////////////////////////////////////////////






// Unique link identifier, for detecting type faster than with rtti. (Obsolete)
#define LNK_MATECOAXIAL	44

///
/// Mate constraint of coaxial type. This correspond to the
/// typical cylinder-vs-cylinder mating used in 3D CAD assemblies.
/// The two coaxial axes are the X axes of the two frames.
///

class ChApi ChLinkMateCoaxial : public ChLinkMateGeneric {

	CH_RTTI(ChLinkMateCoaxial,ChLinkMateGeneric);

protected:

	bool	flipped;

public:
				//
	  			// CONSTRUCTORS
				//

	ChLinkMateCoaxial () : ChLinkMateGeneric(false, true, true, false, true,true) {flipped = false;};
	virtual ~ChLinkMateCoaxial () {};
	virtual void Copy(ChLinkMateCoaxial* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer

				//
	  			// FUNCTIONS
				//

	virtual int GetType	() {return LNK_MATECOAXIAL;}

					/// Tell if the two axes must be opposed (flipped=false) or must have the same verse (flipped=true)
	void SetFlipped(bool doflip);
	bool IsFlipped() {return flipped;}

					/// Specialized initialization for coaxial mate, given the two bodies to be connected,
					/// two points, two directions (each expressed in body or abs. coordinates). 
					/// Use ChLinkMateGeneric::Initialize() if you want to set the two frames directly.
	virtual int Initialize(ChSharedBodyPtr& mbody1,	///< first body to link
						   ChSharedBodyPtr& mbody2, ///< second body to link
						   bool pos_are_relative,	///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChVector<> mpt1,			///< point on slave axis, for 1st body (rel. or abs., see flag above)
						   ChVector<> mpt2,  		///< point on master axis, for 2nd body (rel. or abs., see flag above)
						   ChVector<> mdir1,		///< direction of slave axis, for 1st body (rel. or abs., see flag above)
						   ChVector<> mdir2  		///< direction of master axis, for 2nd body (rel. or abs., see flag above)
						   );

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////





// Unique link identifier, for detecting type faster than with rtti. (Obsolete)
#define LNK_MATECOINCIDENT	45

///
/// Mate constraint of coincident type. This correspond to the
/// typical point-on-point or spherical joint mating used in 3D CAD assemblies.
///

class ChApi ChLinkMateCoincident : public ChLinkMateGeneric {

	CH_RTTI(ChLinkMateCoincident,ChLinkMateGeneric);

protected:


public:
				//
	  			// CONSTRUCTORS
				//

	ChLinkMateCoincident () : ChLinkMateGeneric(true, true, true, false, false, false) {} ;
	virtual ~ChLinkMateCoincident () {};
	virtual void Copy(ChLinkMateCoincident* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer

				//
	  			// FUNCTIONS
				//

	virtual int GetType	() {return LNK_MATECOINCIDENT;}


					/// Specialized initialization for coincident mate, given the two bodies to be connected,
					/// and two points (each expressed in body or abs. coordinates). 
					/// Use ChLinkMateGeneric::Initialize() if you want to set the two frames directly.
	virtual int Initialize(ChSharedBodyPtr& mbody1,	///< first body to link
						   ChSharedBodyPtr& mbody2, ///< second body to link
						   bool pos_are_relative,	///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChVector<> mpt1,			///< point, slave, for 1st body (rel. or abs., see flag above)
						   ChVector<> mpt2  		///< point, master, for 2nd body (rel. or abs., see flag above)
						   );

};






//////////////////////////////////////////////////////
//////////////////////////////////////////////////////





// Unique link identifier, for detecting type faster than with rtti. (Obsolete)
#define LNK_MATEPARALLEL	46

///
/// Mate constraint of parallel type. This correspond to the
/// typical axis-is-parallel-to-axis (or edge to edge, etc.) mating 
/// used in 3D CAD assemblies. The axes to be kept parallel are
/// the two X axes of the two frames.
///

class ChApi ChLinkMateParallel : public ChLinkMateGeneric {

	CH_RTTI(ChLinkMateParallel,ChLinkMateGeneric);

protected:

	bool flipped;

public:
				//
	  			// CONSTRUCTORS
				//

	ChLinkMateParallel () : ChLinkMateGeneric(false, false, false, false, true, true) { flipped = false;} ;
	virtual ~ChLinkMateParallel () {};
	virtual void Copy(ChLinkMateParallel* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer

				//
	  			// FUNCTIONS
				//

	virtual int GetType	() {return LNK_MATEPARALLEL;}

					/// Tell if the two axes must be opposed (flipped=false) or must have the same verse (flipped=true)
	void SetFlipped(bool doflip);
	bool IsFlipped() {return flipped;}

					/// Specialized initialization for parallel mate, given the two bodies to be connected,
					/// two points and two directions (each expressed in body or abs. coordinates). 
					/// Use ChLinkMateGeneric::Initialize() if you want to set the two frames directly.
	virtual int Initialize(ChSharedBodyPtr& mbody1,	///< first body to link
						   ChSharedBodyPtr& mbody2, ///< second body to link
						   bool pos_are_relative,	///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChVector<> mpt1,			///< point on slave axis, for 1st body (rel. or abs., see flag above)
						   ChVector<> mpt2,  		///< point on master axis, for 2nd body (rel. or abs., see flag above)
						   ChVector<> mdir1,		///< direction of slave axis, for 1st body (rel. or abs., see flag above)
						   ChVector<> mdir2  		///< direction of master axis, for 2nd body (rel. or abs., see flag above
						   );

};






} // END_OF_NAMESPACE____

#endif
