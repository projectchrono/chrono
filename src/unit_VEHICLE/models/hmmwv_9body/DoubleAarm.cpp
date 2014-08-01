#include "DoubleAarm.h"

using namespace chrono;

// set up tire & vehicle geometry -------------------------------------------------
// spring stiffness and damping, HMMWV M 1037 data
double springK_F = 954.0;		// lb/in, spring type 5579473, standard front springs
double springK_R = 2108.0;			// lb/in, spring type 5597913, heavy duty rear springs
double damperC_F = 128.25;			// lb-sec/in, Fig 5.9 force/rebound rate slope
double damperC_R = 200.0;			// lb-sec/in, Fig 5.10, heavy duty shock
// Shock connection point differs from the spring by 4.6" in both front and rear units.
// No way to account for this currently, so adding this to the spring free lengths 
double spring_rest_len_F = 17.96;	// inches, front spring, 13.36" from tech report.
double spring_rest_len_R = 17.96;	// inches, rear heavy duty spring, 15.03" from tech report.
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
	double m_spring_K, m_damper_C, m_rest_length = 0;
	// all numbers are w.r.t. chassis CM
	switch ( susp_type )
	{
		// hardpoints 1,2,3: forward, outer (ball-joint), rear-ward
		case 0:	// "left front, (x,y) (-,-)":
		{	
			HP_U1 = r0 + ChVector<>(-83.5, -17.56, -22.66)*inch_to_m;	// on chassis, top front
			HP_U2 = r0 + ChVector<>(-83.3, -28.17, -23.81)*inch_to_m;	// on upright
			HP_U3 = r0 + ChVector<>(-74.83,  -17.56, -24.6)*inch_to_m;	// on chassis, top back
			HP_L1 = r0 + ChVector<>(-94.18, -12.1, -32.29)*inch_to_m;	// chassis, bottom front
			HP_L2 = r0 + ChVector<>(-83.99, -30.97, -36.94)*inch_to_m;	// upright, bottom
			HP_L3 = r0 + ChVector<>(-76.6, -12.1, -32.29)*inch_to_m;	// upright, bottom rear
//			HP_KD_U	= r0 + ChVector<>(-83.8, -27.87, -19.57)*inch_to_m;	// shock, top, Aligned with spindle CM
//			HP_KD_L	= r0 + ChVector<>(-83.8, -30.97, -33.81)*inch_to_m;	// shock, bottom, Aligned with spindle CM
			HP_KD_U	= r0 + ChVector<>(-89.49, -27.87, -19.57)*inch_to_m;	// shock, top
			HP_KD_L	= r0 + ChVector<>(-89.22, -30.97, -33.81)*inch_to_m;	// shock, bottom
			HP_St_1	= r0 + ChVector<>(-72.0, -9.81, -33.325)*inch_to_m;	// steer, output to rack // y -= 9.81
			HP_St_2	= r0 + ChVector<>(-78.47, -32.32, -33.325)*inch_to_m;	// steer, upright ball joint
			susp_type_string = "left front";
			m_spring_K = springK_F/inch_to_m * 4.448;	// lb/in to N/m
			m_damper_C = damperC_F/inch_to_m * 4.448;	// lb/in/sec to N/m/sec
			m_rest_length = spring_rest_len_F * inch_to_m;	// inches to meters
			break;
		}
		case 1:	// "right front, (x,y) (-,+)":
		{
			HP_U1 = r0 + ChVector<>(-83.5, 17.56, -22.66)*inch_to_m;	// on chassis, top front
			HP_U2 = r0 + ChVector<>(-83.3, 28.17, -23.81)*inch_to_m;	// on upright
			HP_U3 = r0 + ChVector<>(-74.83,  17.56, -24.6)*inch_to_m;	// on chassis, top back
			HP_L1 = r0 + ChVector<>(-94.18, 12.1, -32.29)*inch_to_m;	// chassis, bottom front
			HP_L2 = r0 + ChVector<>(-83.99, 30.97, -36.94)*inch_to_m;	// upright, bottom
			HP_L3 = r0 + ChVector<>(-76.6, 12.1, -32.29)*inch_to_m;	// upright, bottom rear
			HP_KD_U	= r0 + ChVector<>(-89.49, 27.87, -19.57)*inch_to_m;	// springDamper, top, avg of UCA HP1,3
			HP_KD_L	= r0 + ChVector<>(-89.22, 30.97, -33.81)*inch_to_m;	// sringDamper, lower, = HP L2
			HP_St_1	= r0 + ChVector<>(-72.0, 9.81, -33.325)*inch_to_m;	// steer, output to rack // y -= 9.81
			HP_St_2	= r0 + ChVector<>(-78.47, 32.32, -33.325)*inch_to_m;	// steer, upright ball joint
			susp_type_string = "right front";
			m_spring_K = springK_F/inch_to_m * 4.448;	// lb/in to N/m
			m_damper_C = damperC_F/inch_to_m * 4.448;	// lb/in/sec to N/m/sec
			m_rest_length = spring_rest_len_F * inch_to_m;	// inches to meters
			break; 
		}
		case 2:	// "left back, (x,y) (+,-)":
		{
			HP_U1	= r0 + ChVector<>(33.82, -18.2, -23.41)*inch_to_m;	// on chassis, top rear
			HP_U2	= r0 + ChVector<>(46.2, -28.17, -23.01)*inch_to_m;	// outer, on upright
			HP_U3	= r0 +ChVector<>(44.53, -18.2, -23.41)*inch_to_m;		// top front
			HP_L1 = r0 +ChVector<>(38.81, -12.1, -32.29 )*inch_to_m;	// chassis, bottom rear
			HP_L2 = r0 +ChVector<>(46.2, -30.97, -36.94)*inch_to_m;	// outer, upright, bottom
			HP_L3 = r0 +ChVector<>(56.39, -12.1, -32.29)*inch_to_m;	// bottom front
			HP_KD_U	= r0 +ChVector<>(51.69, -28.2, -19.57)*inch_to_m;		// springDamper, top (chassis) = avg. of UCA HP 1,3
			HP_KD_L	= r0 +ChVector<>(51.69, -30.97, -33.8)*inch_to_m;	// sringDamper, lower (upright) = LCA HP2
			HP_St_1	= r0 +ChVector<>(34.9, -16.38, -32.66)*inch_to_m;	// steer, chassis 
			HP_St_2	= r0 + ChVector<>(40.9, -32.33, -32.66)*inch_to_m;	// steer, upright
			susp_type_string = "left back";
			m_spring_K = springK_R/inch_to_m * 4.448;	// lb/in to N/m
			m_damper_C = damperC_R/inch_to_m * 4.448;	// lb/in/sec to N/m/sec
			m_rest_length = spring_rest_len_R * inch_to_m;	// inches to meters
			break;
		}
		case 3:	// "right back, (x,y) (+,+)":
		{
			HP_U1	= r0 + ChVector<>(33.82, 18.2, -23.41)*inch_to_m;	// on chassis, top rear
			HP_U2	= r0 + ChVector<>(46.2, 28.17, -23.01)*inch_to_m;	// outer, on upright
			HP_U3	= r0 +ChVector<>(44.53, 18.2, -23.41)*inch_to_m;		// top front
			HP_L1 = r0 +ChVector<>(38.81, 12.1, -32.29 )*inch_to_m;	// chassis, bottom rear
			HP_L2 = r0 +ChVector<>(46.2, 30.97, -36.94)*inch_to_m;	// outer, upright, bottom
			HP_L3 = r0 +ChVector<>(56.39, 12.1, -32.29)*inch_to_m;	// bottom front
			HP_KD_U	= r0 +ChVector<>(51.69, 28.2, -19.57)*inch_to_m;		// springDamper, top (chassis) = avg. of UCA HP 1,3
			HP_KD_L	= r0 +ChVector<>(51.69, 30.97, -33.8)*inch_to_m;	// sringDamper, lower (upright) = LCA HP2
			// HP_KD_U	= r0 +ChVector<>(46.2, 28.2, -19.57)*inch_to_m;	// keep the connection of the shock in-line with upright CM
			// HP_KD_L	= r0 +ChVector<>(46.2, 30.97, -33.8)*inch_to_m;	// keep the connection of the shock in-line with upright CM
			HP_St_1	= r0 +ChVector<>(34.9, 16.38, -32.66)*inch_to_m;	// steer, chassis 
			HP_St_2	= r0 + ChVector<>(40.9, 32.33, -32.66)*inch_to_m;	// steer, upright
			susp_type_string = "right back";
			m_spring_K = springK_R/inch_to_m * 4.448;	// lb/in to N/m
			m_damper_C = damperC_R/inch_to_m * 4.448;	// lb/in/sec to N/m/sec
			m_rest_length = spring_rest_len_R * inch_to_m;	// inches to meters
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
	this->upright = ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(uprightSize.x, uprightSize.y, uprightSize.z, 1000,
		false, true));
	ChVector<> spindlePos = chassis->GetCoord().TrasformLocalToParent( upright_r_bar);
	upright->SetPos( spindlePos );
	upright->SetNameString(susp_type_string + " upright" );
	my_system.Add(upright);
	this->body_list.push_back(upright.DynamicCastTo<ChBody>().get_ptr());

	// ---- revolute joint between wheel and upright. Joint location based on input position, upright_r_bar
	this->spindle_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); 
	spindle_revolute->Initialize(wheel, upright, ChCoordsys<>( chassis->GetCoord().TrasformLocalToParent(upright_r_bar),
		chrono::Q_from_AngAxis(CH_C_PI/2.0, VECT_X) ) );
	spindle_revolute->SetNameString(susp_type_string + " spindle revolute");
	my_system.AddLink(spindle_revolute);
	this->joint_list.push_back(spindle_revolute.DynamicCastTo<ChLinkLock>().get_ptr() );

	// --- distance constraints to idealize upper and lower control arms
	ChSharedPtr<ChLinkDistance> link_distU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // upper 1, (frontward)
	link_distU1->Initialize(chassis, upright, false, HP_U1, HP_U2);
	link_distU1->SetNameString(susp_type_string + " dist U1 front");
	my_system.AddLink(link_distU1);
	this->link_list.push_back(link_distU1.get_ptr() );

	ChSharedPtr<ChLinkDistance> link_distU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); //   upper 2, (rearward)
	link_distU2->Initialize(chassis, upright, false, HP_U3, HP_U2 );
	link_distU2->SetNameString(susp_type_string + " dist U2 rear");
	my_system.AddLink(link_distU2);
	this->link_list.push_back(link_distU2.get_ptr() );

	ChSharedPtr<ChLinkDistance> link_distL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); //   lower 1, frontward
	link_distL1->Initialize(chassis, upright, false, HP_L1, HP_L2 );
	link_distL1->SetNameString(susp_type_string + " dist L1 front");
	my_system.AddLink(link_distL1);
	this->link_list.push_back(link_distL1.get_ptr() );

	ChSharedPtr<ChLinkDistance> link_distL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // lower 2, rearward
	link_distL2->Initialize(chassis, upright, false, HP_L3, HP_L2 );
	link_distL2->SetNameString(susp_type_string + " dist L2 rear");
	my_system.AddLink(link_distL2);
	this->link_list.push_back(link_distL2.get_ptr() );

	//	--- Spring/damper, between upright and chassis
	this->shock = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
	shock->Initialize(upright, chassis, false, HP_KD_L, HP_KD_U );
	shock->Set_SpringK(m_spring_K);	// lb-in to N-m
	shock->Set_SpringR(m_damper_C);	// lb-in/sec to N-m/sec
	shock->SetNameString(susp_type_string + " shock");
	double marker_springRestLen = shock->Get_SpringRestLenght();	// 0.37 m 
	shock->Set_SpringRestLenght(m_rest_length );	// FRONT 0.456 m, so dx*k = 14 kN, supporting 1425 kg, front
													// REAR,  dx*k =  32 kN, 3250 kg
	my_system.AddLink(shock);

	//	--- Steering
 	this->tierod = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right steer
	tierod->Initialize(chassis, upright, false, HP_St_1, HP_St_2 );
	tierod->SetNameString(susp_type_string + " dist tierod");
	my_system.AddLink(tierod);
	this->link_list.push_back(tierod.get_ptr());
	// initial relative position of marker 1, attached to chassis, manipulated by steer input
	this->tierod_marker1_IC = tierod->GetEndPoint1Rel();

}

DoubleAarm::~DoubleAarm() {
	// remove the links, forces, etc., then the bodies last
	if( this->link_list.size() > 0 ) {
		ChSystem* m_sys = this->link_list.front()->GetSystem();
		std::list<ChLink*>::iterator link_iter =  this->link_list.begin();
		m_sys->RemoveLinkIter(link_iter);
	}
	if( this->joint_list.size() > 0 ) {
		ChSystem* m_sys = this->joint_list.front()->GetSystem();
		std::list<ChLink*>::iterator joint_iter = this->joint_list.begin();
		m_sys->RemoveLinkIter(joint_iter);
	}
	if( this->body_list.size() > 0 ) {
		ChSystem* m_sys = this->body_list.front()->GetSystem();
		std::list<ChBody*>::iterator body_iter = this->body_list.begin();
		while(1) {
			ChSharedPtr<ChBody> curr_body(*body_iter);
			m_sys->RemoveBody(curr_body);
			if(body_iter == this->body_list.end() ) {
				break;
			}
		}
	}
}
