#ifndef CHLINKSPRING_H
#define CHLINKSPRING_H

///////////////////////////////////////////////////
//
//   ChLinkSpring.h
//
//
//   Classes for spring-dampers joints.
//
//   This class is inherited from the base ChLink()
//   class, used by all joints in 3D.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChLinkMarkers.h"



namespace chrono
{

// Unique link identifier, for detecting type faster than with rtti.
#define LNK_SPRING		25


///
/// Class for spring-damper systems, acting along the polar
/// distance of two markers
///

class ChApi ChLinkSpring : public ChLinkMarkers {

	CH_RTTI(ChLinkSpring,ChLinkMarkers);

protected:
	double spr_restlenght;
	double spr_k;
	double spr_r;
	double spr_f;
	ChFunction* mod_f_time;		// f(t)
	ChFunction* mod_k_d;		// k(d)
	ChFunction* mod_r_d;		// r(d)
	ChFunction* mod_r_speed;	// k(speed)
	ChFunction* mod_k_speed;	// r(speed)
	double spr_react;		// resulting force in dist. coord / readonly
public:

				//
	  			// FUNCTIONS
				//
						// builders and destroyers
	ChLinkSpring ();
	virtual ~ChLinkSpring ();
	virtual void Copy(ChLinkSpring* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer, Link*

	virtual int GetType	() {return LNK_SPRING;}

						// data fetch/store
	double Get_SpringRestLenght() {return spr_restlenght;};
	void   Set_SpringRestLenght(double m_r) {spr_restlenght = m_r;};
	double Get_SpringDeform() {return (dist - spr_restlenght);};
	double Get_SpringK() {return spr_k;};
	void   Set_SpringK(double m_r) {spr_k = m_r;};
	double Get_SpringR() {return spr_r;};
	void   Set_SpringR(double m_r) {spr_r = m_r;};
	double Get_SpringF() {return spr_f;};
	void   Set_SpringF(double m_r) {spr_f = m_r;};
	double Get_SpringReact() {return spr_react;};

	ChFunction* Get_mod_f_time() {return mod_f_time;};
	ChFunction* Get_mod_k_d() {return mod_k_d;};
	ChFunction* Get_mod_r_d() {return mod_r_d;};
	ChFunction* Get_mod_k_speed() {return mod_k_speed;};
	ChFunction* Get_mod_r_speed() {return mod_r_speed;};
	void Set_mod_f_time	(ChFunction* m_funct);
	void Set_mod_k_d	(ChFunction* m_funct);
	void Set_mod_r_d	(ChFunction* m_funct);
	void Set_mod_k_speed(ChFunction* m_funct);
	void Set_mod_r_speed(ChFunction* m_funct);

					/// Specialized initialization for springs, given the two bodies to be connected, the
					/// positions of the two anchor endpoints of the spring (each expressed
					/// in body or abs. coordinates) and the imposed rest length of the spring.
					/// NOTE! As in ChLinkMarkers::Initialize(), the two markers are automatically 
					/// created and placed inside the two connected bodies.
	virtual int Initialize(ChSharedBodyPtr& mbody1,	///< first body to link
						   ChSharedBodyPtr& mbody2, ///< second body to link
						   bool pos_are_relative,	///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChVector<> mpos1,		///< position of spring endpoint, for 1st body (rel. or abs., see flag above)
						   ChVector<> mpos2,		///< position of spring endpoint, for 2nd body (rel. or abs., see flag above) 
						   bool auto_rest_length =true,///< if true, initializes the rest-length as the distance between mpos1 and mpos2
						   double mrest_length =0   ///< imposed rest_length (no need to define, if auto_rest_length=true.)
						   );

					/// Get the 1st spring endpoint (expressed in Body1 coordinate system)
	ChVector<> GetEndPoint1Rel() {return marker1->GetPos();}
					/// Set the 1st spring endpoint (expressed in Body1 coordinate system)
	void       SetEndPoint1Rel(const ChVector<>& mset) { marker1->Impose_Rel_Coord(ChCoordsys<>(mset,QUNIT));}
					/// Get the 1st spring endpoint (expressed in absolute coordinate system)
	ChVector<> GetEndPoint1Abs() {return marker1->GetAbsCoord().pos;}
					/// Set the 1st spring endpoint (expressed in absolute coordinate system)
	void       SetEndPoint1Abs(ChVector<>& mset) { marker1->Impose_Abs_Coord(ChCoordsys<>(mset,QUNIT));}

					/// Get the 2nd spring endpoint (expressed in Body2 coordinate system)
	ChVector<> GetEndPoint2Rel() {return marker2->GetPos();};
					/// Set the 2nd spring endpoint (expressed in Body2 coordinate system)
	void       SetEndPoint2Rel(const ChVector<>& mset) { marker2->Impose_Rel_Coord(ChCoordsys<>(mset,QUNIT));}
					/// Get the 1st spring endpoint (expressed in absolute coordinate system)
	ChVector<> GetEndPoint2Abs() {return marker2->GetAbsCoord().pos;}
					/// Set the 1st spring endpoint (expressed in absolute coordinate system)
	void       SetEndPoint2Abs(ChVector<>& mset) { marker2->Impose_Abs_Coord(ChCoordsys<>(mset,QUNIT));}

			//
			// UPDATING FUNCTIONS
			//

						/// Inherits, then also adds the spring custom forces to
						/// the C_force and C_torque. 
	virtual void UpdateForces (double mytime);


			//
			// STREAMING
			//


	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutBinary& mstream);


};





//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
