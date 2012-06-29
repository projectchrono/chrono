%{

/* Includes the header in the wrapper code */
#include "physics/ChBody.h"

%}
 

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


//%import "ChPhysicsItem.i"
%import "ChMaterialSurface.i"
%import "ChCollisionModel.i"
//%import "ChMarker.i" // already tricked by adding chrono:: in front of all ChMarker...
//%import "ChForce.i"  // "  "  "


/* Parse the header file to generate wrappers */
//%include "../physics/ChBody.h"  

namespace chrono
{
class ChBody : public ChPhysicsItem , public ChFrameMoving<double> {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChBody,ChPhysicsItem);

public:

	ChBody ();
	~ChBody ();
	void Copy(ChBody* source);

	void SetBodyFixed (bool mev);
	bool GetBodyFixed()	   {return BFlagGet(BF_FIXED);}

	void  SetCollide (bool mcoll);
	bool  GetCollide() {return BFlagGet(BF_COLLIDE);}

	void SetShowCollisionMesh    (bool mcoll) { BFlagSet(BF_SHOW_COLLMESH, mcoll);};
	bool GetShowCollisionMesh () {return BFlagGet(BF_SHOW_COLLMESH);};

	void SetLimitSpeed    (bool mlimit) { BFlagSet(BF_LIMITSPEED, mlimit);};
	bool GetLimitSpeed()  {return BFlagGet(BF_LIMITSPEED);};

	void SetUseSleeping    (bool ms) { BFlagSet(BF_USESLEEPING, ms);};
	bool GetUseSleeping()  {return BFlagGet(BF_USESLEEPING);};

	void SetSleeping    (bool ms) { BFlagSet(BF_SLEEPING, ms);};
	bool GetSleeping()  {return BFlagGet(BF_SLEEPING);};

	bool TrySleeping();
	bool IsActive() {return !BFlagGet(BF_SLEEPING | BF_FIXED);}

	virtual int GetDOF  ()   {return 6;}

	ChLcpVariablesBodyOwnMass& Variables() {return variables;}
	void VariablesFbReset();

	void VariablesFbLoadForces(double factor=1.);
	void VariablesQbLoadSpeed();
	void VariablesQbSetSpeed(double step=0.);
	void VariablesQbIncrementPosition(double step);
	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);
	virtual chrono::collision::ChCollisionModel* InstanceCollisionModel();
	void SetNoSpeedNoAcceleration();
	chrono::collision::ChCollisionModel* GetCollisionModel() {return collision_model;}

	virtual void SyncCollisionModels();
	virtual void AddCollisionModelsToSystem();
	virtual void RemoveCollisionModelsFromSystem();

	int RecomputeCollisionModel();

	ChCoordsys<> GetLastCollPos () { return last_coll_pos; }

	void SynchronizeLastCollPos() {last_coll_pos = this->coord;}

	virtual ChFrame<>& GetFrame_COG_to_abs() {return *this;}

	virtual ChFrame<>& GetFrame_REF_to_abs() {return *this;}

	virtual void GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax);

	virtual void StreamINstate(ChStreamInBinary& mstream);

	virtual void StreamOUTstate(ChStreamOutBinary& mstream);	

	chrono::ChSharedPtr<ChMaterialSurface>& GetMaterialSurface() {return this->matsurface;}

	void SetMaterialSurface(chrono::ChSharedPtr<ChMaterialSurface>& mnewsurf) {this->matsurface = mnewsurf;}

				
	void AddMarker (chrono::ChSharedPtr<ChMarker> amarker);
	void AddForce (chrono::ChSharedPtr<ChForce> aforce);

	void RemoveMarker (chrono::ChSharedPtr<ChMarker> amarker);
	void RemoveForce  (chrono::ChSharedPtr<ChForce> aforce);

	void RemoveAllForces();
	void RemoveAllMarkers();

	chrono::ChSharedPtr<ChMarker> SearchMarker (char* m_name);
	chrono::ChSharedPtr<ChForce> SearchForce(char* m_name);

	std::vector<ChMarker*>* GetMarkerList() {return &marklist;} 
	std::vector<ChForce*>* GetForceList() {return &forcelist;}

	ChVector<> Point_World2Body (ChVector<>* mpoint);
	ChVector<> Point_Body2World (ChVector<>* mpoint);
	ChVector<> Dir_World2Body (ChVector<>* mpoint);
	ChVector<> Dir_Body2World (ChVector<>* mpoint);
	ChVector<> RelPoint_AbsSpeed(ChVector<>* mrelpoint);
	ChVector<> RelPoint_AbsAcc(ChVector<>* mrelpoint);

	
	void   SetMass (double newmass) { if (newmass>0.) variables.SetBodyMass(newmass);}
	double GetMass() {return variables.GetBodyMass();}

	void SetInertia (ChMatrix33<>* newXInertia);
	void SetInertiaXX (ChVector<> iner);
	ChVector<> GetInertiaXX();
	void SetInertiaXY (ChVector<> iner);
	ChVector<> GetInertiaXY();

	void   SetMaxSpeed(float m_max_speed) {max_speed = m_max_speed;}
	float  GetMaxSpeed () {return max_speed;}
	void   SetMaxWvel(float m_max_wvel) {max_wvel = m_max_wvel;}
	float  GetMaxWvel () {return max_wvel;}
	void ClampSpeed();

	void   SetSleepTime(float m_t) {sleep_time = m_t;}
	float GetSleepTime () {return sleep_time;}
	void   SetSleepMinSpeed(float m_t) {sleep_minspeed = m_t;}
	float GetSleepMinSpeed () {return sleep_minspeed;}
	void   SetSleepMinWvel(float m_t) {sleep_minwvel = m_t;}
	float GetSleepMinWvel () {return sleep_minwvel;}

	void ComputeGyro ();

	void Add_as_lagrangian_force(ChVector<> force, ChVector<> appl_point, int local, ChMatrixNM<double,7,1>* mQf);
	void Add_as_lagrangian_torque(ChVector<> torque, int local, ChMatrixNM<double,7,1>* mQf);
	void From_lagrangian_to_forcetorque(ChMatrixNM<double,7,1>* mQf, ChVector<>* mforce, ChVector<>* mtorque);
	void From_forcetorque_to_lagrangian(ChVector<>* mforce, ChVector<>* mtorque, ChMatrixNM<double,7,1>* mQf);

	void To_abs_forcetorque  (ChVector<> force, ChVector<> appl_point, int local, ChVector<>& resultforce, ChVector<>& resulttorque);
	void To_abs_torque (ChVector<> torque, int local, ChVector<>& resulttorque);

	void Accumulate_force  (ChVector<> force, ChVector<> appl_point, int local);
	void Accumulate_torque (ChVector<> torque, int local);
	ChVector<> Get_accumulated_force  () {return Force_acc;};
	ChVector<> Get_accumulated_torque () {return Torque_acc;};
	void Empty_forces_accumulators () {Force_acc = VNULL; Torque_acc = VNULL;};

	ChVector<>* Get_Scr_force() {return &Scr_force;};
	ChVector<>* Get_Scr_torque() {return &Scr_torque;};
	void Set_Scr_force(ChVector<> mf) {Scr_force = mf;};
	void Set_Scr_torque(ChVector<> mf) {Scr_torque = mf;};
	void Accumulate_script_force (ChVector<> force, ChVector<> appl_point, int local);
	void Accumulate_script_torque (ChVector<> torque, int local);

	ChVector<>  Get_gyro();
	ChVector<> Get_Xforce ();
	ChVector<> Get_Xtorque();
	ChMatrix33<>* GetXInertia ();

	void UpdateMarkers (double mytime);
	void UpdateForces (double mytime);
	void UpdateTime (double mytime);
	void UpdateState (ChCoordsys<> mypos, ChCoordsys<> mypos_dt);
	void UpdateStateTime (ChCoordsys<> mypos, ChCoordsys<> mypos_dt, double mytime);
	void Update (ChCoordsys<> mypos, ChCoordsys<> mypos_dt, double mytime);
	virtual void Update (double mytime);
	virtual void Update ();

	void UpdateExternalGeometry ();

	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);
	int StreamOUTall (ChStreamOutBinary& m_file);
	int StreamINall  (ChStreamInBinary&  m_file);
	void StreamOUT(ChStreamOutAscii& mstream);
	int  StreamOUTall  (ChStreamOutAscii& mstream);
};

}; // End namespace


// Define also the shared pointer chrono::ChShared<ChBody> 
// (renamed as 'ChBodyShared' in python)

%DefChSharedPtr(ChBodyShared, ChBody)

