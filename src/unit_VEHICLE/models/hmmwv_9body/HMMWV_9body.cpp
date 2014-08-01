#include "HMMWV_9body.h"
#include "assets/ChObjShapeFile.h"
#include "assets/ChAssetLevel.h"
#include "HMMWV_9body_config.h"


// DEBUG HACK
// 0:  no chassis visualization
// 1:  two boxes
// 2:  mesh
#define CHASSIS_VISUALIZATION 0


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV_9body::chassisMass = 7500.0 / 2.2;
const double HMMWV_9body::spindleMass = 100.0 / 2.2;

// Inertias, from my HMMWV model
// (roll, pitch, yaw) = (13320, 52680, 56280) lb-in-sec^2
const ChVector<> HMMWV_9body::chassisInertia(125.8, 497.4, 531.4);	// kg-m2;
const ChVector<> HMMWV_9body::spindleInertia = chassisInertia / 60;

const std::string HMMWV_9body::chassisMeshName = "hmmwv_chassis";
const std::string HMMWV_9body::chassisMeshFile = "../data/humvee4_scaled_rotated_decimated_centered.obj";

// helpful for unit conversions
double in_to_m = 1.0/39.3701;	// inches to meters
double inlb_to_Nm = 1.0/8.851;	// in-lb to N-m

// engine data
const double HMMWV_9body::max_torque = 2400*inlb_to_Nm;	// in-lb
const double HMMWV_9body::max_engine_n = 2000;	// engine speed, rpm 

// set up tire & vehicle geometry -------------------------------------------------
// effective radius of the tires
double tireRadius			= 18.5*in_to_m;
// width of the tires
double tireWidth			= 10.0*in_to_m;
double wheelMass			= 175.0/3.2;	//chassisMass/(150./3.0);
// for visualization, the size of some objects
ChVector<> bodySize(5.2, 2.0, 2.8);
ChVector<> spindleSize(0.2,0.2,0.1);


// for quick output of TMVector
std::ostream& operator << (std::ostream& output, const chrono::ChVector<>& v) {
	output << v.x << "," << v.y << "," << v.z;
	return output;
}

// -----------------------------------------------------------------------------
// HMMWV_9body constructor
//
// x+ points towards rear of vehicle
// y+ points towards right of vehicle
// z+ points vertical upwards
// the front two wheels need revolute joints at the wheel hub
// the rear (driven) wheels use a ChLinkEngine, which is a revolute joint w/ an
// engine attached to the DOF
// -----------------------------------------------------------------------------
HMMWV_9body::HMMWV_9body(ChSystem&             my_system,
                         const ChVector<>&     chassisCM,
                         const ChQuaternion<>& chassisRot,
                         const bool            fixed,
                         const std::string&    tireMeshFile)
: writeOutData(false),
  m_sys(&my_system),
  driver(new ChVehicleDriver)
{
	this->conic_tau = 0.2;
	this->gear_tau = 0.3;
	this->max_motor_torque = max_torque;
	this->max_motor_speed = max_engine_n;

  bool use_tireMesh = !tireMeshFile.empty();

  // -------------------------------------------
  // Create the chassis body
  // -------------------------------------------

  chassis = ChSharedPtr<ChBody>(new ChBody);

  chassis->SetIdentifier(0);
  chassis->SetName("chassis");
  chassis->SetMass(chassisMass);
  chassis->SetInertiaXX(chassisInertia);
  chassis->SetPos(chassisCM);
  chassis->SetRot(chassisRot);
  chassis->SetBodyFixed(fixed);

#if CHASSIS_VISUALIZATION == 1
  ChSharedPtr<ChBoxShape> box1(new ChBoxShape);
  box1->GetBoxGeometry().SetLenghts(ChVector<>(5, 1.7, 0.4));
  box1->Pos = ChVector<>(0, 0, -0.4);
  chassis->AddAsset(box1);
  ChSharedPtr<ChBoxShape> box2(new ChBoxShape);
  box2->GetBoxGeometry().SetLenghts(ChVector<>(4, 1.7, 0.4));
  box2->Pos = ChVector<>(0.5, 0, 0);
  chassis->AddAsset(box2);
#elif CHASSIS_VISUALIZATION == 2
  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(chassisMeshFile, false, false);
  ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
  trimesh_shape->SetMesh(trimesh);
  trimesh_shape->SetName(chassisMeshName);
  chassis->AddAsset(trimesh_shape);
#endif

  my_system.Add(chassis);






	// position of spindle, wheel CMs
	double offset= 4.0*in_to_m;	// offset in lateral length between CM of spindle, wheel

	// wheel and spindle positions, relative to the chassis CM position
	// x-forward, z-lateral, w.r.t. chassis coord-sys
	ChVector<> wheelRF_cm_bar = ChVector<>(-83.8, 35.82, -33.325)*in_to_m;	// right front wheel, in body coords
	ChVector<> spindleRF_cm_bar	= ChVector<>(wheelRF_cm_bar);	// right front spindle
	spindleRF_cm_bar.y -= offset; 
	ChVector<> wheelLF_cm_bar = ChVector<>(-83.8, -35.82, -33.325)*in_to_m;	// left front wheel, in body coords
	ChVector<> spindleLF_cm_bar = ChVector<>(wheelLF_cm_bar);	// left front spnidle
	spindleLF_cm_bar.y += offset;
	ChVector<> wheelRB_cm_bar = ChVector<>(46.2, 35.82, -33.325 )*in_to_m;	// right back wheel
	ChVector<> spindleRB_cm_bar	= ChVector<>(wheelRB_cm_bar);	// right back spindle
	spindleRB_cm_bar.y -= offset;
	ChVector<> wheelLB_cm_bar = ChVector<>(46.2, -35.82, -33.325)*in_to_m;	// left back wheel
	ChVector<> spindleLB_cm_bar = ChVector<>(wheelLB_cm_bar);	// left back spindle
	spindleLB_cm_bar.y += offset;

	// 0) --- LF Wheel
	ChVector<> wheelLF_cm = chassis->GetCoord().TrasformLocalToParent( wheelLF_cm_bar);
    use_tireMesh = true;
  if (use_tireMesh) {
		// use a nice looking .obj mesh for the wheel visuals
		this->wheelLF = new ChWheel(my_system, wheelLF_cm, chrono::QUNIT,
			wheelMass, tireWidth, tireRadius*2.0, tireRadius*0.8,
			true,
      "../data/wheel_centered_rotated.obj", QUNIT);
	} else {
		// use a cylinder, with inertia based on the inner and outer radii
		this->wheelLF = new ChWheel(my_system, wheelLF_cm,chrono::QUNIT,
			wheelMass, tireWidth, tireRadius*2.0, tireRadius*0.8, true);
	}
    use_tireMesh = false;
	GetLog() << " \n ---- system heiarchy after BEFORE first suspension subsystem : \n\n\n ";
	my_system.ShowHierarchy( GetLog() );

	// Each suspension has its marker joint locations hardcoded, for now, for each of the four units
	// --- Left front suspension
	this->suspension_LF = new DoubleAarm(my_system, 0, chassis, wheelLF->GetBody(), spindleLF_cm_bar);

	GetLog() << " \n ---- system heiarchy after AFTER first suspension subsystem : \n\n\n ";
	my_system.ShowHierarchy( GetLog() );

	// 1) --- RF wheel
	ChVector<> wheelRF_cm = chassis->GetCoord().TrasformLocalToParent( wheelRF_cm_bar);
  if (use_tireMesh) {
		// use a nice looking .obj mesh for the wheel visuals
		this->wheelRF = new ChWheel(my_system, wheelRF_cm, chrono::QUNIT,
			wheelMass, tireWidth, tireRadius*2.0, tireRadius*0.8,
			true,
      tireMeshFile, QUNIT);
	} else {
		// use a cylinder, with inertia based on the inner and outer radii
		this->wheelRF = new ChWheel(my_system, wheelRF_cm,chrono::QUNIT,
			wheelMass, tireWidth, tireRadius*2.0, tireRadius*0.8, true);
	}

	// --- Right front suspension
	this->suspension_RF = new DoubleAarm(my_system, 1, chassis, wheelRF->GetBody(), spindleRF_cm_bar);

	// 2) ---	LB Wheel
	ChVector<> wheelLB_cm = chassis->GetCoord().TrasformLocalToParent( wheelLB_cm_bar);
  if (use_tireMesh) {
		// use a nice looking .obj mesh for the wheel visuals
		this->wheelLB =  new ChWheel(my_system, wheelLB_cm, chrono::QUNIT,
			wheelMass, tireWidth, tireRadius*2.0, tireRadius*0.8,
			true,
      tireMeshFile, QUNIT);
	} else {
		// use a cylinder, with inertia based on the inner and outer radii
		this->wheelLB = new ChWheel(my_system, wheelLB_cm,chrono::QUNIT,
			wheelMass, tireWidth, tireRadius*2.0, tireRadius*0.8, true);
	}

	// --- Left back suspension. Does not include ChLinkEngine; create it here
	this->suspension_LB = new DoubleAarm(my_system, 2, chassis, wheelLB->GetBody(), spindleLB_cm_bar);

	/// --- LB engine
	this->link_engineL = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); 
	link_engineL->Initialize(wheelLB->GetBody(), chassis, 
		ChCoordsys<>( wheelLB_cm, chrono::Q_from_AngAxis(CH_C_PI/2.0, VECT_X) ) );
	link_engineL->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_CARDANO); // approx as a double Rzeppa joint
	link_engineL->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
	link_engineL->SetName("engine_L");
	my_system.AddLink(link_engineL);

	// 3) ---	RB Wheel
	ChVector<> wheelRB_cm = chassis->GetCoord().TrasformLocalToParent( wheelRB_cm_bar);
  if (use_tireMesh) {
		// use a nice looking .obj mesh for the wheel visuals
		this->wheelRB = new ChWheel(my_system, wheelRB_cm, chrono::QUNIT,
			wheelMass, tireWidth, tireRadius*2.0, tireRadius*0.8,
			true,
      tireMeshFile, QUNIT);
	} else {
		// use a cylinder, with inertia based on the inner and outer radii
		this->wheelRB = new ChWheel(my_system, wheelRB_cm,chrono::QUNIT,
			wheelMass, tireWidth, tireRadius*2.0, tireRadius*0.8, true);
	}

	// --- Right back suspension. Does not include ChLinkEngine; create it here
	this->suspension_RB = new DoubleAarm(my_system, 3, chassis, wheelRB->GetBody(), spindleRB_cm_bar);

	// --- RB engine
	this->link_engineR = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); 
	link_engineR->Initialize(wheelRB->GetBody(), chassis, 
		ChCoordsys<>( wheelRB_cm, chrono::Q_from_AngAxis(-CH_C_PI/2.0, VECT_X) ) );
	link_engineR->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_CARDANO); // approx as a double Rzeppa joint
	link_engineR->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
	link_engineR->SetName("engine_R");
	my_system.AddLink(link_engineR);
}


double HMMWV_9body::ComputeWheelTorque()
{
	// Assume clutch is never used. Given the kinematics of differential,
	// the speed of the engine transmission shaft is the average of the two wheel speeds,
	// multiplied the conic gear transmission ratio inversed:
	double shaftspeed = (1.0/this->conic_tau) * 0.5 *
		(this->link_engineL->Get_mot_rot_dt()+this->link_engineR->Get_mot_rot_dt());
	// The motorspeed is the shaft speed multiplied by gear ratio inversed:
	double motorspeed = (1.0/this->gear_tau)*shaftspeed;
	// The torque depends on speed-torque curve of the motor: here we assume a 
	// very simplified model a bit like in DC motors:
	double motortorque = max_motor_torque - motorspeed*(max_motor_torque/max_motor_speed) ;
	// Motor torque is linearly modulated by throttle gas value:
	motortorque = motortorque *  this->driver->getThrottle();
	// The torque at motor shaft:
	double shafttorque =  motortorque * (1.0/this->gear_tau);
	// The torque at wheels - for each wheel, given the differential transmission, 
	// it is half of the shaft torque  (multiplied the conic gear transmission ratio)
	double singlewheeltorque = 0.5 * shafttorque * (1.0/this->conic_tau);
	// Set the wheel torque in both 'engine' links, connecting the wheels to the chassis;
	if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(this->link_engineL->Get_tor_funct()))
		mfun->Set_yconst(singlewheeltorque);
	if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(this->link_engineR->Get_tor_funct()))
		mfun->Set_yconst(-singlewheeltorque);
	// debug:print infos on screen:
	GetLog() << "motor torque="<< motortorque<< "  speed=" << motorspeed << "  wheel torqe=" << singlewheeltorque <<"\n";
	// If needed, return also the value of wheel torque:
	this->currTorque = singlewheeltorque;
	return singlewheeltorque;
}


double HMMWV_9body::ComputeSteerDisplacement()
{
	// between -1 and 1
	double steer_val = this->driver->getSteer();
	// apply the steer gain to convert to meters
	double steer_disp = 0.08 * steer_val;
	// apply the displacement to the initial tierod marker 1
	ChVector<> r_bar_L = this->suspension_LF->tierod_marker1_IC;
	ChVector<> r_bar_R = this->suspension_RF->tierod_marker1_IC;
	r_bar_L.y += steer_disp;
	r_bar_R.y += steer_disp;
	this->suspension_LF->tierod->SetEndPoint1Rel(r_bar_L);
	this->suspension_RF->tierod->SetEndPoint1Rel(r_bar_R);
	return steer_disp;
}

void HMMWV_9body::applyHub_FM(const std::vector<ChVector<>>& F_hub, const std::vector<ChVector<>>& M_hub){

	// empty force accumulators
	this->wheelLB->wheelBody->Empty_forces_accumulators();
	this->wheelLF->wheelBody->Empty_forces_accumulators();
	this->wheelRB->wheelBody->Empty_forces_accumulators();
	this->wheelRF->wheelBody->Empty_forces_accumulators();

	// add the force and moment to each
	// TODO: reverse signs?
	// RF
	this->wheelRF->wheelBody->Accumulate_force(F_hub[0], ChVector<>(0,0,0),0);
	this->wheelRF->wheelBody->Accumulate_torque(M_hub[0],0);
	// LF
	this->wheelLF->wheelBody->Accumulate_force(F_hub[0], ChVector<>(0,0,0),0);
	this->wheelLF->wheelBody->Accumulate_torque(M_hub[0],0);
	// RB
	this->wheelRB->wheelBody->Accumulate_force(F_hub[0], ChVector<>(0,0,0),0);
	this->wheelRB->wheelBody->Accumulate_torque(M_hub[0],0);
	// LB
	this->wheelLB->wheelBody->Accumulate_force(F_hub[0], ChVector<>(0,0,0),0);
	this->wheelLB->wheelBody->Accumulate_torque(M_hub[0],0);

}

// helper funtions
// RF, LF, RB, LB
ChVector<> HMMWV_9body::getCM_pos(int tire_idx){
	ChVector<> CM_pos = ChVector<>();
	// RF, LF, RB, LB
	if( tire_idx == 0 ) {
		// RF
		CM_pos = this->wheelRF->wheelBody->GetPos();
	}
	if( tire_idx == 1) {
		// LF
		CM_pos = this->wheelLF->wheelBody->GetPos();
	}
	if( tire_idx == 2) {
		// RB
		CM_pos = this->wheelRB->wheelBody->GetPos();
	}
	if( tire_idx == 3) {
		// LB
		CM_pos = this->wheelLB->wheelBody->GetPos();
	}
	if( tire_idx > 3 ) {
		GetLog()<< "warning, tire_idx larger than the max # of tires\n";
	}

	return CM_pos;
}


ChQuaternion<> HMMWV_9body::getCM_q(int tire_idx){
	ChQuaternion<> CM_q = ChQuaternion<>();
	// RF, LF, RB, LB
	if( tire_idx == 0 ) {
		// RF
		CM_q = this->wheelRF->wheelBody->GetRot();
	}
	if( tire_idx == 1) {
		// LF
		CM_q = this->wheelLF->wheelBody->GetRot();
	}
	if( tire_idx == 2) {
		// RB
		CM_q = this->wheelRB->wheelBody->GetRot();
	}
	if( tire_idx == 3) {
		// LB
		CM_q = this->wheelLB->wheelBody->GetRot();
	}
	if( tire_idx > 3 ) {
		GetLog()<< "warning, tire_idx larger than the max # of tires\n";
	}

	return CM_q;
}

// get the chassis CM coords
ChVector<> HMMWV_9body::getCM_pos_chassis() {
	return this->chassis->GetPos();
}

// get the chassis CM velocity
ChVector<> HMMWV_9body::getCM_vel_chassis() {
	return this->chassis->GetPos_dt();
}

ChVector<> HMMWV_9body::getCM_vel(int tire_idx){
	ChVector<> CM_vel = ChVector<>();
		// RF, LF, RB, LB
	if( tire_idx == 0 ) {
		// RF
		this->wheelRF->wheelBody->GetPos_dt();
	}
	if( tire_idx == 1) {
		// LF
		this->wheelLF->wheelBody->GetPos_dt();
	}
	if( tire_idx == 2) {
		// RB
		this->wheelRB->wheelBody->GetPos_dt();
	}
	if( tire_idx == 3) {
		// LB
		this->wheelLB->wheelBody->GetPos_dt();
	}
	if( tire_idx > 3 ) {
		GetLog()<< "warning, tire_idx larger than the max # of tires\n";
	}
	return CM_vel;
}

ChVector<> HMMWV_9body::getCM_w(int tire_idx){
	ChVector<> CM_w = ChVector<>();
		// RF, LF, RB, LB
	if( tire_idx == 0 ) {
		// RF
		this->wheelRF->wheelBody->GetWvel_loc();
	}
	if( tire_idx == 1) {
		// LF
		this->wheelLF->wheelBody->GetWvel_loc();
	}
	if( tire_idx == 2) {
		// RB
		this->wheelRB->wheelBody->GetWvel_loc();
	}
	if( tire_idx == 3) {
		// LB
		this->wheelLB->wheelBody->GetWvel_loc();
	}
	if( tire_idx > 3 ) {
		GetLog()<< "warning, tire_idx larger than the max # of tires\n";
	}
	return CM_w;
}

ChVector<> HMMWV_9body::getCM_acc(int tire_idx){
	ChVector<> CM_acc = ChVector<>();
		// RF, LF, RB, LB
	if( tire_idx == 0 ) {
		// RF
		this->wheelRF->wheelBody->GetPos_dtdt();
	}
	if( tire_idx == 1) {
		// LF
		this->wheelLF->wheelBody->GetPos_dtdt();
	}
	if( tire_idx == 2) {
		// RB
		this->wheelRB->wheelBody->GetPos_dtdt();
	}
	if( tire_idx == 3) {
		// LB
		this->wheelLB->wheelBody->GetPos_dtdt();
	}
	if( tire_idx > 3 ) {
		GetLog()<< "warning, tire_idx larger than the max # of tires\n";
	}
	return CM_acc;
}



// I want to output:
// 1) CM pos
// 2) CM orientation (nasa angles) YAW PITCH ROLL ????????
// 3) CM linear velocity
// 4) CM rotational vel
// 5) motion Force
void HMMWV_9body::write_OutData(double simtime) {
	// first time thru, write the headers
	if( this->outWritten_nTimes == 0 ) {
		this->outFile = std::ofstream( outFilename, std::ios::out);
		if( !outFile.is_open() ) {
			std::cout << "couldn't open file: " << outFilename << " to write TestMech data to!!!! " << std::endl;
		} else {
			// write the headers
			// NOTE: CHECK THE OUTPUT OF NASA ANGS, MIGHT BE ROLL PITCH YAW!!!!
			outFile << "time,cm_x,cm_y,cm_z,Yaw,Pitch,Roll,vel_x,vel_y,vel_z,w_x,w_y,w_z,F_x,F_y,F_z,T_a" << std::endl;
			this->outWritten_nTimes++;
			outFile.close();
		}
	} else {
		// open file for append
		this->outFile.open( outFilename, std::ios::app);
		ChVector<> cm_pos = this->chassis->GetPos();
		ChVector<> nasa_angs = chrono::Q_to_NasaAngles( this->chassis->GetRot() );
		ChVector<> cm_vel = chassis->GetPos_dt();
		// ChVector<> vel_global = locFrame.TrasformLocalToParent( mWheelBody->GetPos_dt() );
		ChVector<> omega = chrono::Q_to_NasaAngles( chassis->GetRot_dt() );
		// write the data

//		outFile << simtime << "," << cm_pos << "," << nasa_angs << "," << cm_vel << "," << omega << std::endl;
		this->outWritten_nTimes++; // increment the counter
		outFile.close();	// close the file for writing

	}
	
}

HMMWV_9body::~HMMWV_9body()
{
	// When a ChBodySceneNode is removed via ->remove() from Irrlicht 3D scene manager,
	// it is also automatically removed from the ChSystem (the ChSystem::RemoveBody() is
	// automatically called at Irrlicht node deletion - see ChBodySceneNode.h ).
	ChSystem* sys = chassis->GetSystem();
	// For links, just remove them from the ChSystem using ChSystem::RemoveLink()
	sys->RemoveBody(chassis);
	sys->RemoveLink(link_engineL);
	sys->RemoveLink(link_engineR);

	// created the wheels on the stack
	delete wheelRF;
	delete wheelLF;
	delete wheelRB;
	delete wheelLB;

	// create suspension on the stack
	delete suspension_RF;
	delete suspension_LF;
	delete suspension_RB;
	delete suspension_LB;

	// delete the driver
	delete driver;

}

