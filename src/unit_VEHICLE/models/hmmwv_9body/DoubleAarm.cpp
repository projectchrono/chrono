#include "DoubleAarm.h"

using namespace chrono;

// set up tire & vehicle geometry -------------------------------------------------
// spring stiffness and damping, HMMWV M 1037 data
double springK_F = 168822.0;		// lb/in
double springK_R = 302619;			// lb/in
double damperC_F = 16987;			// lb-sec/in
double damperC_R = 33974;			// lb-sec/in
double inch_to_m = 0.0254;	// inches to meters


DoubleAarm::DoubleAarm(ChSystem&  my_system, const int susp_type, ChSharedPtr<ChBody>& chassis,
	ChSharedPtr<ChBody>& wheel,
	const ChVector<>& upright_r_bar,
	const std::string& data_file_name,
	double uprightMass, ChVector<>& uprightInertia, ChVector<>& uprightSize) 
{
	// for now, assume HMMWV is oriented directly forward
	ChVector<> r0 = chassis->GetPos();

	// *** suspension hardpoints
	ChVector<> HP_U1	= ChVector<>();	// top front, 
	ChVector<> HP_U2	= ChVector<>();	// outer, on upright
	ChVector<> HP_U3	= ChVector<>();	// top back
	ChVector<> HP_L1 = ChVector<>();	// chassis, bottom front
	ChVector<> HP_L2 = ChVector<>();	// outer, on upright
	ChVector<> HP_L3 = ChVector<>();	// bottom rear
	ChVector<> HP_KD_U	= ChVector<>();	// springDamper, top, avg. of UCA HP 1,3
	ChVector<> HP_KD_L	= ChVector<>();	// sringDamper, lower = LCA HP2
	ChVector<> HP_St_1	= ChVector<>();	// steer, chassis 
	ChVector<> HP_St_2	= ChVector<>();	// steer, upright

	std::string susp_type_string;

	// all numbers are w.r.t. chassis CM
	switch ( susp_type )
	{
		case 0:	// "left front, (x,y) (-,-)":
		{	
			HP_U1 = r0 + ChVector<>(-44.13, -11.13, -17.56)*inch_to_m;		// on chassis, top front
			HP_U2 = r0 + ChVector<>(-43.93, -10.43, -28.17)*inch_to_m;		// on upright
			HP_U3 = r0 + ChVector<>(-35.46,  -11.13, -18.81)*inch_to_m;		// on chassis, top back
			HP_L1 = r0 + ChVector<>(-54.81, -18.91, -12.1)*inch_to_m;	// chassis, bottom front
			HP_L2 = r0 + ChVector<>(-44.62, -23.56, -30.97)*inch_to_m;	// upright, bottom
			HP_L3 = r0 + ChVector<>(-37.23, -18.91, -12.1)*inch_to_m;	// upright, bottom rear
			HP_KD_U	= r0 + ChVector<>(-39.795, -11.13, -27.87)*inch_to_m;		// springDamper, top, avg of UCA HP1,3
			HP_KD_L	= r0 + ChVector<>(-44.62, -23.56, -30.97)*inch_to_m;	// sringDamper, lower, = HP L2
			HP_St_1	= r0 + ChVector<>(-25.8, -16.6, -9.81)*inch_to_m;	// steer, chassis // y -= 9.81
			HP_St_2	= r0 + ChVector<>(-31.1, -19.55, -32.33)*inch_to_m;	// steer, upright
			susp_type_string = "left front";
			break;
		}
		case 1:	// "right front, (x,y) (-,+)":
		{
			HP_U1 = r0 + ChVector<>(-44.13, 11.13, -17.56)*inch_to_m;	// top front, 
			HP_U2 = r0 + ChVector<>(-43.93, 10.43, -28.17)*inch_to_m;	// outer, on upright
			HP_U3 = r0 + ChVector<>(-35.46, 11.13, -18.81)*inch_to_m;	// top back
			HP_L1 = r0 + ChVector<>(-54.81, 18.91, -12.1 )*inch_to_m;	// chassis, bottom front
			HP_L2 = r0 + ChVector<>(-44.62, 23.56, -30.97)*inch_to_m;	// outer, on upright
			HP_L3 = r0 + ChVector<>(-37.23, 18.91, -12.1)*inch_to_m;	// bottom rear
			HP_KD_U	= r0 + ChVector<>(-39.795, 11.13, -27.87)*inch_to_m;	// springDamper, top, avg. of UCA HP 1,3
			HP_KD_L	= r0 + ChVector<>(-44.62, 23.56, -30.97)*inch_to_m;	// sringDamper, lower = LCA HP2
			HP_St_1	= r0 + ChVector<>(-25.8, 16.6, -9.81)*inch_to_m;	// steer, chassis 
			HP_St_2	= r0 + ChVector<>(-31.1, 19.55, -32.33)*inch_to_m;	// steer, upright
			susp_type_string = "right front";
			break; 
		}
		case 2:	// "left back, (x,y) (+,-)":
		{
			HP_U1	= r0 + ChVector<>(83.97, -10.02, -18.2)*inch_to_m;	// on chassis, top rear
			HP_U2	= r0 + ChVector<>(85.57, -10.41, -28.17)*inch_to_m;	// outer, on upright
			HP_U3	= r0 +ChVector<>(73.2, -10.02, -18.2)*inch_to_m;		// top front
			HP_L1 = r0 +ChVector<>(95.76, -18.91, -12.1)*inch_to_m;	// chassis, bottom rear
			HP_L2 = r0 +ChVector<>(85.57, -23.56, -30.97)*inch_to_m;	// outer, upright, bottom
			HP_L3 = r0 +ChVector<>(78.18, -18.91, -12.1)*inch_to_m;	// bottom front
			HP_KD_U	= r0 +ChVector<>(85.57, -10.02, -28.2)*inch_to_m;		// springDamper, top (chassis) = avg. of UCA HP 1,3
			HP_KD_L	= r0 +ChVector<>(85.57, -23.56, -30.97)*inch_to_m;	// sringDamper, lower (upright) = LCA HP2
			HP_St_1	= r0 +ChVector<>(70.18, -16.6, -16.38)*inch_to_m;	// steer, chassis 
			HP_St_2	= r0 + ChVector<>(72.27, -19.28, -32.33)*inch_to_m;	// steer, upright
			susp_type_string = "left back";
			break;
		}
		case 3:	// "right back, (x,y) (+,+)":
		{
			HP_U1	= r0 + ChVector<>(83.97, 10.02, -18.2)*inch_to_m;	// on chassis, top rear
			HP_U2	= r0 + ChVector<>(85.57, 10.41, -28.17)*inch_to_m;	// outer, on upright
			HP_U3	= r0 + ChVector<>(73.2, 10.02, -18.2)*inch_to_m;		// top front
			HP_L1 = r0 + ChVector<>(95.76, 18.91, -12.1 )*inch_to_m;	// chassis, bottom rear
			HP_L2 = r0 + ChVector<>(85.57, 23.56, -30.97)*inch_to_m;	// outer, on upright, bottom
			HP_L3 = r0 + ChVector<>(78.18, 18.91, -12.1)*inch_to_m;	// bottom front
			HP_KD_U	= r0 + ChVector<>(85.57, 10.02, -28.2)*inch_to_m;		// springDamper, top = avg. of UCA HP1,3
			HP_KD_L	= r0 + ChVector<>(85.57, 23.56, -30.97)*inch_to_m;	// sringDamper, lower = LCA HP2
			HP_St_1	= r0 + ChVector<>(70.18, 16.6, -16.38)*inch_to_m;	// steer, chassis 
			HP_St_2	= r0 + ChVector<>(72.27, 19.28, -32.33)*inch_to_m;	// steer, upright
			susp_type_string = "right back";
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
	// ---------- upright.  Initialize, add to the system, add to ref array.
	this->upright = ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(uprightSize.x, uprightSize.y, uprightSize.z, 1000, false, true));
	upright->SetPos(chassis->GetCoord().TrasformLocalToParent( upright_r_bar) );
	upright->SetNameString(susp_type_string + " upright" );
	my_system.Add(upright);
	this->body_list.push_back(&upright.DynamicCastTo<ChBody>());

	// ---- revolute joint between wheel and upright. Joint location based on input position, upright_r_bar
	ChSharedPtr<ChLinkLockRevolute> spindle_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); 
	spindle_revolute->Initialize(wheel, upright, ChCoordsys<>( chassis->GetCoord().TrasformLocalToParent(upright_r_bar),
		chrono::Q_from_AngAxis(CH_C_PI/2.0, VECT_X) ) );
	spindle_revolute->SetNameString(susp_type_string + " spindle revolute");
	my_system.AddLink(spindle_revolute);
	this->joint_list.push_back(&spindle_revolute.DynamicCastTo<ChLinkLock>());

	// --- distance constraints to idealize upper and lower control arms
	ChSharedPtr<ChLinkDistance> link_distU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // upper 1, (frontward)
	link_distU1->Initialize(chassis, upright, false, HP_U1, HP_U2);
	link_distU1->SetNameString(susp_type_string + " dist U1 front");
	my_system.AddLink(link_distU1);
	this->link_list.push_back(&link_distU1);

	ChSharedPtr<ChLinkDistance> link_distU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); //   upper 2, (rearward)
	link_distU2->Initialize(chassis, upright, false, HP_U3, HP_U2 );
	link_distU2->SetNameString(susp_type_string + " dist U2 rear");
	my_system.AddLink(link_distU2);
	this->link_list.push_back(&link_distU2);

	ChSharedPtr<ChLinkDistance> link_distL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); //   lower 1, frontward
	link_distL1->Initialize(chassis, upright, false, HP_L1, HP_L2 );
	link_distL1->SetNameString(susp_type_string + " dist L1 front");
	my_system.AddLink(link_distL1);
	this->link_list.push_back(&link_distL1);

	ChSharedPtr<ChLinkDistance> link_distL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // lower 2, rearward
	link_distL2->Initialize(chassis, upright, false, HP_L3, HP_L2 );
	link_distL2->SetNameString(susp_type_string + " dist L2 rear");
	my_system.AddLink(link_distL2);
	this->link_list.push_back(&link_distL2);

	//	--- Spring/damper, between upright and chassis
	this->shock = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
	shock->Initialize(upright, chassis, false, HP_KD_U, HP_KD_L );
	shock->Set_SpringK(springK_F);	
	shock->Set_SpringR(damperC_F);
	shock->SetNameString(susp_type_string + " shock");
	my_system.AddLink(shock);
	// double m_springRestLen = shock->Get_SpringRestLenght();
	// shock->Set_SpringRestLenght

	//	--- Steering
 	this->tierod = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right steer
	tierod->Initialize(chassis, chassis, false, HP_St_1, HP_St_2 );
	tierod->SetNameString(susp_type_string + " dist tierod");
	my_system.AddLink(tierod);
	this->link_list.push_back(&tierod);

}

DoubleAarm::~DoubleAarm() {
	// remove the links, forces, etc., then the bodies last
	if( this->link_list.size() > 0 ) {
		ChSystem* m_sys = (*(this->body_list[0]))->GetSystem();
		for(int i = 0; i < this->link_list.size(); i++){
			m_sys->RemoveLink(*(this->link_list[i]));
		}
		for(int j = 0; j < this->body_list.size(); j++){
			m_sys->RemoveBody(*(this->body_list[j]) );
		}
	}
}
