#include "DoubleAarm.h"

using namespace chrono;
// JCM 11/7/12, helpful for unit conversions
// double inch_to_m = inch_to_m;	// inches to meters
// double inlb_to_Nm = 1.0/8.851;	// in-lb to N-m

// set up tire & vehicle geometry -------------------------------------------------
// effective radius of the tires
// double tireRadius			= 18.5*inch_to_m;
// width of the tires
// double tireWidth			= 10.0*inch_to_m;
// double chassisMass			= 7500.0/2.2;	// chassis mass in kg
// double spindleMass			= 100.0/2.2;	//chassisMass/(150./8.0);
// double wheelMass			= 175.0/3.2;	//chassisMass/(150./3.0);
// for visualization, the size of some objects
// ChVector<> bodySize(5.2, 2.0, 2.8);
// ChVector<> spindleSize(0.2,0.2,0.1);
// Inertias, from my HMMWV model
// ChVector<> carInertia		= ChVector<>(10.0, 20.0, 20.0);	// kg-m2
// ChVector<> wheelInertia		= carInertia/20.0;	// [kg-m^2]
// ChVector<> spindleInertia	= carInertia/40.0;	// guesses, for now
// spring stiffness and damping, HMMWV M 1037 data
double springK_F = 168822.0;		// lb/in
double springK_R = 302619;			// lb/in
double damperC_F = 16987;			// lb-sec/in
double damperC_R = 33974;			// lb-sec/in
double inch_to_m = 0.0254;	// inches to meters
// engine data
// double max_torque = 8600*1.0/8.851;	// in-lb
// double max_engine_n = 2000;	// engine speed, rpm 

/*
DoubleAarm::DoubleAarm() {

}
*/

DoubleAarm::DoubleAarm(ChSystem&  my_system, const int susp_type, ChSharedPtr<ChBody>& chassis,
	ChSharedPtr<ChBody>& wheel,
	const ChVector<>& spindle_r_bar,
	const std::string& data_file_name,
	double spindleMass, ChVector<>& spindleInertia, ChVector<>& spindleSize) 
{
	// for now, assume HMMWV is oriented directly forward
	ChVector<> r0 = chassis->GetPos();

	// *** suspension hardpoints
	ChVector<> HP_U1	= ChVector<>();	// top front, 
	ChVector<> HP_U2	= ChVector<>();	// outer, on spindle
	ChVector<> HP_U3	= ChVector<>();	// top back
	ChVector<> HP_L1 = ChVector<>();	// chassis, bottom front
	ChVector<> HP_L2 = ChVector<>();	// outer, on spindle
	ChVector<> HP_L3 = ChVector<>();	// bottom rear
	ChVector<> HP_KD_U	= ChVector<>();	// springDamper, top, avg. of UCA HP 1,3
	ChVector<> HP_KD_L	= ChVector<>();	// sringDamper, lower = LCA HP2
	ChVector<> HP_St_1	= ChVector<>();	// steer, chassis 
	ChVector<> HP_St_2	= ChVector<>();	// steer, spindle

	// all numbers are w.r.t. chassis CM
	switch ( susp_type )
	{
		case 0:	// "left front, (x,y) (-,-)":
		{	
			HP_U1 = r0 + ChVector<>(-44.13, -11.13, -17.56)*inch_to_m;		// on chassis, top front
			HP_U2 = r0 + ChVector<>(-43.93, -10.43, -28.17)*inch_to_m;		// on spindle
			HP_U3 = r0 + ChVector<>(-35.46,  -11.13, -18.81)*inch_to_m;		// on chassis, top back
			HP_L1 = r0 + ChVector<>(-54.81, -18.91, -12.1)*inch_to_m;	// chassis, bottom front
			HP_L2 = r0 + ChVector<>(-44.62, -23.56, -30.97)*inch_to_m;	// spindle, bottom
			HP_L3 = r0 + ChVector<>(-37.23, -18.91, -12.1)*inch_to_m;	// spindle, bottom rear
			HP_KD_U	= r0 + ChVector<>(-39.795, -11.13, -27.87)*inch_to_m;		// springDamper, top, avg of UCA HP1,3
			HP_KD_L	= r0 + ChVector<>(-44.62, -23.56, -30.97)*inch_to_m;	// sringDamper, lower, = HP L2
			HP_St_1	= r0 + ChVector<>(-25.8, -16.6, -9.81)*inch_to_m;	// steer, chassis // y -= 9.81
			HP_St_2	= r0 + ChVector<>(-31.1, -19.55, -32.33)*inch_to_m;	// steer, spindle
	
			break;
		}
		case 1:	// "right front, (x,y) (-,+)":
		{
			HP_U1 = r0 + ChVector<>(-44.13, 11.13, -17.56)*inch_to_m;	// top front, 
			HP_U2 = r0 + ChVector<>(-43.93, 10.43, -28.17)*inch_to_m;	// outer, on spindle
			HP_U3 = r0 + ChVector<>(-35.46, 11.13, -18.81)*inch_to_m;	// top back
			HP_L1 = r0 + ChVector<>(-54.81, 18.91, -12.1 )*inch_to_m;	// chassis, bottom front
			HP_L2 = r0 + ChVector<>(-44.62, 23.56, -30.97)*inch_to_m;	// outer, on spindle
			HP_L3 = r0 + ChVector<>(-37.23, 18.91, -12.1)*inch_to_m;	// bottom rear
			HP_KD_U	= r0 + ChVector<>(-39.795, 11.13, -27.87)*inch_to_m;	// springDamper, top, avg. of UCA HP 1,3
			HP_KD_L	= r0 + ChVector<>(-44.62, 23.56, -30.97)*inch_to_m;	// sringDamper, lower = LCA HP2
			HP_St_1	= r0 + ChVector<>(-25.8, 16.6, -9.81)*inch_to_m;	// steer, chassis 
			HP_St_2	= r0 + ChVector<>(-31.1, 19.55, -32.33)*inch_to_m;	// steer, spindle
			break; 
		}
		case 2:	// "left back, (x,y) (+,-)":
		{
			HP_U1	= r0 + ChVector<>(83.97, -10.02, -18.2)*inch_to_m;	// on chassis, top rear
			HP_U2	= r0 + ChVector<>(85.57, -10.41, -28.17)*inch_to_m;	// outer, on spindle
			HP_U3	= r0 +ChVector<>(73.2, -10.02, -18.2)*inch_to_m;		// top front
			HP_L1 = r0 +ChVector<>(95.76, -18.91, -12.1)*inch_to_m;	// chassis, bottom rear
			HP_L2 = r0 +ChVector<>(85.57, -23.56, -30.97)*inch_to_m;	// outer, spindle, bottom
			HP_L3 = r0 +ChVector<>(78.18, -18.91, -12.1)*inch_to_m;	// bottom front
			HP_KD_U	= r0 +ChVector<>(85.57, -10.02, -28.2)*inch_to_m;		// springDamper, top (chassis) = avg. of UCA HP 1,3
			HP_KD_L	= r0 +ChVector<>(85.57, -23.56, -30.97)*inch_to_m;	// sringDamper, lower (spindle) = LCA HP2
			HP_St_1	= r0 +ChVector<>(70.18, -16.6, -16.38)*inch_to_m;	// steer, chassis 
			HP_St_2	= r0 + ChVector<>(72.27, -19.28, -32.33)*inch_to_m;	// steer, spindle
			break;
		}
		case 3:	// "right back, (x,y) (+,+)":
		{
			HP_U1	= r0 + ChVector<>(83.97, 10.02, -18.2)*inch_to_m;	// on chassis, top rear
			HP_U2	= r0 + ChVector<>(85.57, 10.41, -28.17)*inch_to_m;	// outer, on spindle
			HP_U3	= r0 + ChVector<>(73.2, 10.02, -18.2)*inch_to_m;		// top front
			HP_L1 = r0 + ChVector<>(95.76, 18.91, -12.1 )*inch_to_m;	// chassis, bottom rear
			HP_L2 = r0 + ChVector<>(85.57, 23.56, -30.97)*inch_to_m;	// outer, on spindle, bottom
			HP_L3 = r0 + ChVector<>(78.18, 18.91, -12.1)*inch_to_m;	// bottom front
			HP_KD_U	= r0 + ChVector<>(85.57, 10.02, -28.2)*inch_to_m;		// springDamper, top = avg. of UCA HP1,3
			HP_KD_L	= r0 + ChVector<>(85.57, 23.56, -30.97)*inch_to_m;	// sringDamper, lower = LCA HP2
			HP_St_1	= r0 + ChVector<>(70.18, 16.6, -16.38)*inch_to_m;	// steer, chassis 
			HP_St_2	= r0 + ChVector<>(72.27, 19.28, -32.33)*inch_to_m;	// steer, spindle
			break;
		}
		default:
		{
			std::cout << "couldn't recognize susp_type " << susp_type << std::endl;
			break;
		}
	}

	// now the suspension hardpoints have been ID'd, create the bodies and links
	// 1) create a new ChBody or ChLink as a ChSharedPtr
	// 2) Initialize the object
	// 3) (optional) add collision geometry
	// 4) finalize or add to system
	// 5) add ChSharedPtr to the class list, and the name also
	// ---------- spindle.  Initialize, add to the system, add to ref array.
	spindle = ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(spindleSize.x, spindleSize.y, spindleSize.z, 1000));
	spindle->SetPos(chassis->GetCoord().TrasformLocalToParent( spindle_r_bar) );
	my_system.Add(spindle);

	this->body_list.push_back(spindle);
	this->body_names.push_back("spindle");

	// either front suspension needs a revolute joint between wheel and spindle, due to lack of ChLinkEngine
	// on Rear Wheel Drive vehicles
	if(susp_type == 0 || susp_type == 1 ) {
		ChSharedPtr<ChLinkLockRevolute> spindle_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); 
		spindle_revolute->Initialize(wheel, spindle, ChCoordsys<>( wheel->GetPos(),
			chrono::Q_from_AngAxis(CH_C_PI/2.0, VECT_X) ) );
		my_system.AddLink(spindle_revolute);
		this->joint_list.push_back(spindle_revolute);
		this->joint_names.push_back("spindle_revolute");
	}

	// --- suspension joints. Initialize, add to the system, add to ref array.
	ChSharedPtr<ChLinkDistance> link_distU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, upper, 1
	link_distU1->Initialize(chassis, spindle, false, HP_U1, HP_U2);
	my_system.AddLink(link_distU1);
	this->link_list.push_back(link_distU1);
	this->link_names.push_back("link_distU1");

	ChSharedPtr<ChLinkDistance> link_distU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, upper, 2
	link_distU2->Initialize(chassis, spindle, false, HP_U3, HP_U2 );
	my_system.AddLink(link_distU2);
	this->link_list.push_back(link_distU2);
	this->link_names.push_back("link_distU2)");

	ChSharedPtr<ChLinkDistance> link_distL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, lower, 1
	link_distL1->Initialize(chassis, spindle, false, HP_L1, HP_L2 );
	my_system.AddLink(link_distL1);
	this->link_list.push_back(link_distL1);
	this->link_names.push_back("link_distL1");

	ChSharedPtr<ChLinkDistance> link_distL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, lower, 2
	link_distL2->Initialize(chassis, spindle, false, HP_L3, HP_L2 );
	my_system.AddLink(link_distL2);
	this->link_list.push_back(link_distL2);
	this->link_names.push_back("link_distL2");

	//	--- Spring/damper
	ChSharedPtr<ChLinkSpring> spring = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
	spring->Initialize(chassis, spindle, false, HP_KD_U, HP_KD_L );
	spring->Set_SpringK(springK_F);	
	spring->Set_SpringR(damperC_F);	
	my_system.AddLink(spring);
	this->shock = spring;
	// double m_springRestLen = shock->Get_SpringRestLenght();
	// shock->Set_SpringRestLenght

	//	--- Steering
 	ChSharedPtr<ChLinkDistance> link_distSTEER = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right steer
	link_distSTEER->Initialize(chassis, spindle, false, HP_St_1, HP_St_2 );
	my_system.AddLink(link_distSTEER);
	this->link_list.push_back(link_distSTEER);
	this->link_names.push_back("link_distSTEER");
	this->tierod = link_distSTEER;

}

DoubleAarm::~DoubleAarm() {
	// remove the links, forces, etc., then the bodies last
	if( this->link_list.size() > 0 ) {
		ChSystem* m_sys = this->body_list[0]->GetSystem();
		for(int i = 0; i < this->link_list.size(); i++){
			m_sys->RemoveLink(this->link_list[i]);
		}
		for(int j = 0; j < this->body_list.size(); j++){
			m_sys->RemoveBody(this->body_list[j]);
		}
		this->body_names.clear();
		this->joint_names.clear();
	}
}

/*

// get the list of bodies as const refs
std::vector<ChSharedPtr<ChBody>>& DoubleAarm::get_body_ptrs() {
	return this->body_list;
}

// get the list of joints/links as const refs
std::vector<ChSharedPtr<ChLinkLock>>& DoubleAarm::get_link_ptrs() {
	return this->link_list;
}

*/