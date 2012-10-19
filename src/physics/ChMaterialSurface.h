#ifndef CHMATERIALSURFACE_H
#define CHMATERIALSURFACE_H

///////////////////////////////////////////////////
//
//   ChMaterialSurface.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "core/ChShared.h"


namespace chrono
{

/// Material data for a surface: friction, compliance, etc.
/// This data is used to define surface properties owned by 
/// ChBody rigid bodies and similar things; it carries information
/// that is used to make contacts.

class ChApi ChMaterialSurface : public ChShared
{
public:

			//
			// DATA
			//

	float  static_friction;	
	float  sliding_friction;	
	float  rolling_friction;	
	float  spinning_friction;			
	float  restitution;	
	float  cohesion;
	float  dampingf;
	float  compliance;
	float  complianceT;
	float  complianceRoll;
	float  complianceSpin;


			//
			// CONSTRUCTORS
			//

	ChMaterialSurface() :static_friction(0.6f),
						sliding_friction(0.6f),
						rolling_friction(0),
						spinning_friction(0),
						restitution(0),
						cohesion(0),
						dampingf(0),
						compliance(0),
						complianceT(0),
						complianceRoll(0),
						complianceSpin(0)
					{
					};

	~ChMaterialSurface() 
					{
					};

						// Copy constructor 
	ChMaterialSurface(const ChMaterialSurface& other) 
					{ 
						static_friction=other.static_friction;
						sliding_friction=other.sliding_friction;
						rolling_friction=other.rolling_friction;
						spinning_friction=other.spinning_friction;
						restitution=other.restitution;
						cohesion=other.cohesion;
						dampingf=other.dampingf;
						compliance=other.compliance;
						complianceT=other.complianceT;
						complianceRoll=other.complianceRoll;
						complianceSpin=other.complianceSpin;
					}

			//
			// FUNCTIONS
			//

				/// The static friction coefficient. 
				/// Usually in 0..1 range, rarely above.
				/// Default 0.6
	float  GetSfriction() {return static_friction;}
	void   SetSfriction(float mval) {static_friction = mval;}

				/// The sliding ('kinetic')friction coefficient. Default 0.6
				/// Usually in 0..1 range, rarely above. 
				/// Note: currently the static friction will be used instead, anyway, because of an issue in the solver.
	float  GetKfriction() {return sliding_friction;}
	void   SetKfriction(float mval) {sliding_friction = mval;}

				/// Set both static friction and kinetic friction at once, with same value.
	void   SetFriction(float mval) {SetSfriction(mval); SetKfriction(mval);}

				/// The rolling friction (rolling parameter, it has the dimension of a length). 
				/// Rolling resistant torque is Tr <= (normal force) * (this parameter)
				/// Usually a very low value. Measuring unit: m
				/// Default =0.
				/// Note! a non-zero value will make the simulation 2x slower! Also, the
				/// GPU solver currently does not support rolling friction. Default: 0.
	float  GetRollingFriction() {return rolling_friction;}
	void   SetRollingFriction(float mval) {rolling_friction = mval;}

				/// The spinning friction (it has the dimension of a length). 
				/// Spinning resistant torque is Ts <= (normal force) * (this parameter)
				/// Usually a very low value.  Measuring unit: m
				/// Default =0.
				/// Note! a non-zero value will make the simulation 2x slower! Also, the
				/// GPU solver currently does not support spinning friction. Default: 0.
	float  GetSpinningFriction() {return spinning_friction;}
	void   SetSpinningFriction(float mval) {spinning_friction = mval;}

				/// The normal restitution coefficient, for collisions.
				/// Should be in 0..1 range. Default =0.
	float  GetRestitution() {return restitution;}
	void   SetRestitution(float mval) {restitution = mval;}

				/// The cohesion max. force for normal pulling traction in 
				/// contacts. Measuring unit: N
				/// Default =0.
	float  GetCohesion() {return cohesion;}
	void   SetCohesion(float mval) {cohesion = mval;}

				/// The damping in contact, as a factor 'f': damping is a 
				/// multiple of stiffness [K], that is: [R]=f*[K] 
				/// Measuring unit: time, s. Default =0.
	float  GetDampingF() {return dampingf;}
	void   SetDampingF(float mval) {dampingf = mval;}

				/// Compliance of the contact, in normal direction.
				/// It is the inverse of the stiffness [K] , so for zero
				/// value one has a perfectly rigid contact.
				/// Measuring unit: m/N  
				/// Default =0.
	float  GetCompliance() {return compliance;}
	void   SetCompliance(float mval) {compliance = mval;}

				/// Compliance of the contact, in tangential direction.
				/// Measuring unit: m/N
				/// Default =0.
	float  GetComplianceT() {return complianceT;}
	void   SetComplianceT(float mval) {complianceT = mval;}

				/// Rolling compliance of the contact, if using a nonzero rolling friction.
				/// (If there is no rolling friction, this has no effect.)
				/// Measuring unit: rad/Nm
				/// Default =0.
	float  GetComplianceRolling() {return complianceRoll;}
	void   SetComplianceRolling(float mval) {complianceRoll = mval;}
				
				/// Spinning compliance of the contact, if using a nonzero rolling friction.
				/// (If there is no spinning friction, this has no effect.)
				/// Measuring unit: rad/Nm
				/// Default =0.
	float  GetComplianceSpinning() {return complianceSpin;}
	void   SetComplianceSpinning(float mval) {complianceSpin = mval;}


			//
			// STREAMING
			//
					/// Method to allow serializing transient data into in ascii
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	virtual void StreamOUT(ChStreamOutAscii& mstream)
						{
							mstream << "Material \n";
						}

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream)
						{
							// class version number
							mstream.VersionWrite(1);

							// deserialize parent class too
							//ChShared::StreamOUT(mstream); // nothing 

							// stream out all member data
							mstream <<   static_friction;	
							mstream <<   sliding_friction;	
							mstream <<   rolling_friction;	
							mstream <<   spinning_friction;			
							mstream <<   restitution;	
							mstream <<   cohesion;
							mstream <<   dampingf;
							mstream <<   compliance;
							mstream <<   complianceT;
							mstream <<   complianceRoll;
							mstream <<   complianceSpin;
						}

					/// Operator to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream)
						{
							// class version number
							//int version = mstream.VersionRead(); //unused

							// deserialize parent class too
							//ChShared::StreamIN(mstream); // nothing 

							// stream in all member data
							mstream >>   static_friction;	
							mstream >>   sliding_friction;	
							mstream >>   rolling_friction;	
							mstream >>   spinning_friction;			
							mstream >>   restitution;	
							mstream >>   cohesion;
							mstream >>   dampingf;
							mstream >>   compliance;
							mstream >>   complianceT;
							mstream >>   complianceRoll;
							mstream >>   complianceSpin;
						}

};



} // END_OF_NAMESPACE____

#endif
