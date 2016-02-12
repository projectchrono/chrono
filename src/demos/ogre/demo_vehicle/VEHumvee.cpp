#include "VEHumvee.h"

double const in2m = 0.0254;

namespace VehicleEnvironment {

	VEHumvee::VEHumvee() {

	}

	VEHumvee::~VEHumvee() {

	}

	VEHumvee::VEHumvee(EnvironmentCore::EnvironmentCoreApplication* App) : VESuspensionDemo(App) {

	}

	void VEHumvee::build(chrono::ChVector<>& Pos) {

		double wheel_fric = .9;
		chrono::ChVector<> r;
		double chassis_mass = 140.0;
		double spindle_mass = 3.3;
		double wheel_mass = 1.8;
		r = chrono::ChVector<>(1.0, 0.5, 3)*.5;
		chrono::ChVector<> inertia_chassis(1 / 12.0 * chassis_mass * (r.y * r.y + r.z * r.z), 1 / 12.0 * chassis_mass * (r.x * r.x + r.z * r.z), 1 / 12.0 * chassis_mass * (r.x * r.x + r.y * r.y));;
		r = chrono::ChVector<>(0.1, 0.4, 0.4)*.5;
		chrono::ChVector<> inertia_spindle(1 / 12.0 * spindle_mass * (r.y * r.y + r.z * r.z), 1 / 12.0 * spindle_mass * (r.x * r.x + r.z * r.z), 1 / 12.0 * spindle_mass * (r.x * r.x + r.y * r.y));;
		r = chrono::ChVector<>(0.9, 0.3, 0.9)*0.5;
		chrono::ChVector<> inertia_wheel(1 / 5.0 * wheel_mass * (r.y * r.y + r.z * r.z), 1 / 5.0 * wheel_mass * (r.x * r.x + r.z * r.z), 1 / 5.0 * wheel_mass * (r.x * r.x + r.y * r.y));

		std::vector<chrono::ChVector<>> _points_front_right;
		_points_front_right.resize(NUM_POINTS);
		std::vector<chrono::ChVector<>> _points_front_left;
		_points_front_left.resize(NUM_POINTS);
		std::vector<chrono::ChVector<>> _points_rear_right;
		_points_rear_right.resize(NUM_POINTS);
		std::vector<chrono::ChVector<>> _points_rear_left;
		_points_rear_left.resize(NUM_POINTS);

		bool _fixed = true;

		truss = &m_pApp->getScene()->spawnBox("Truss", chassis_mass, chrono::ChVector<>(0, 1, 0) + Pos, chrono::ChVector<>(1.7, 0.4, 5)*.5, chrono::ChQuaternion<>(1, 0, 0, 0), _fixed);
		truss->getChBody()->SetInertiaXX(inertia_chassis);
		truss->getChBody()->SetCollide(false);
		truss->deletable = false;

		for (unsigned int i = 0; i < NUM_POINTS; i++) {
			chrono::ChVector<> rel_pos = getFrontLocation(static_cast<PointId>(i));
			_points_front_right[i] = truss->getChBody()->GetCoord().TransformLocalToParent(in2m * chrono::ChVector<>(12.10, -18.914, -85.39) + rel_pos);
		}
		for (unsigned int i = 0; i < NUM_POINTS; i++) {
			chrono::ChVector<> rel_pos = getFrontLocation(static_cast<PointId>(i));
			rel_pos.x = -rel_pos.x;
			_points_front_left[i] = truss->getChBody()->GetCoord().TransformLocalToParent(in2m * chrono::ChVector<>(-12.10, -18.914, -85.39) + rel_pos);
		}
		for (unsigned int i = 0; i < NUM_POINTS; i++) {
			chrono::ChVector<> rel_pos = getRearLocation(static_cast<PointId>(i));
			_points_rear_right[i] = truss->getChBody()->GetCoord().TransformLocalToParent(in2m * chrono::ChVector<>(12.10, -18.914, 47.60) + rel_pos);
		}
		for (unsigned int i = 0; i < NUM_POINTS; i++) {
			chrono::ChVector<> rel_pos = getRearLocation(static_cast<PointId>(i));
			rel_pos.x = -rel_pos.x;
			_points_rear_left[i] = truss->getChBody()->GetCoord().TransformLocalToParent(in2m * chrono::ChVector<>(-12.10, -18.914, 47.60) + rel_pos);
		}

		spindleRF = &m_pApp->getScene()->spawnBox("SpindleRF", 33.0, _points_front_right[UPRIGHT], chrono::ChVector<>(0.1, 0.4, 0.4)*.5, chrono::ChQuaternion<>(1, 0, 0, 0), _fixed);
		r = chrono::ChVector<>(0.1, 0.4, 0.4)*.5;
		spindleRF->getChBody()->SetInertiaXX(inertia_spindle);
		spindleRF->getChBody()->SetCollide(false);
		spindleRF->deletable = false;

		wheelRF = &m_pApp->getScene()->spawnEllipsoid("WheelRF", 18.0, _points_front_right[SPINDLE], chrono::ChVector<>(0.9, 0.3, 0.9)*0.5, chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Z), _fixed);
		wheelRF->getChBody()->SetInertiaXX(inertia_wheel);
		wheelRF->getChBody()->SetFriction(wheel_fric);
		wheelRF->deletable = false;

		spindleLF = &m_pApp->getScene()->spawnBox("SpindleLF", 33.0, _points_front_left[UPRIGHT], chrono::ChVector<>(0.1, 0.4, 0.4)*.5, chrono::ChQuaternion<>(1, 0, 0, 0), _fixed);
		spindleLF->getChBody()->SetInertiaXX(inertia_spindle);
		spindleLF->getChBody()->SetCollide(false);
		spindleLF->deletable = false;

		wheelLF = &m_pApp->getScene()->spawnEllipsoid("WheelLF", 18.0, _points_front_left[SPINDLE], chrono::ChVector<>(0.9, 0.3, 0.9)*0.5, chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Z), _fixed);
		wheelLF->getChBody()->SetInertiaXX(inertia_wheel);
		wheelLF->getChBody()->SetFriction(wheel_fric);
		wheelLF->deletable = false;

		spindleRB = &m_pApp->getScene()->spawnBox("SpindleRB", 33.0, _points_rear_right[UPRIGHT], chrono::ChVector<>(0.1, 0.4, 0.4)*.5, chrono::ChQuaternion<>(1, 0, 0, 0), _fixed);
		spindleRB->getChBody()->SetInertiaXX(inertia_spindle);
		spindleRB->getChBody()->SetCollide(false);
		spindleRB->deletable = false;

		wheelRB = &m_pApp->getScene()->spawnEllipsoid("WheelRB", 18.0, _points_rear_right[SPINDLE], chrono::ChVector<>(0.9, 0.3, 0.9)*0.5, chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Z), _fixed);
		wheelRB->getChBody()->SetInertiaXX(inertia_wheel);
		wheelRB->getChBody()->SetFriction(wheel_fric);
		wheelRB->deletable = false;

		spindleLB = &m_pApp->getScene()->spawnBox("SpindleLB", 33.0, _points_rear_left[UPRIGHT], chrono::ChVector<>(0.1, 0.4, 0.4)*.5, chrono::ChQuaternion<>(1, 0, 0, 0), _fixed);
		spindleLB->getChBody()->SetInertiaXX(inertia_spindle);
		spindleLB->getChBody()->SetCollide(false);
		spindleLB->deletable = false;

		wheelLB = &m_pApp->getScene()->spawnEllipsoid("WheelLB", 18.0, _points_rear_left[SPINDLE], chrono::ChVector<>(0.9, 0.3, 0.9)*0.5, chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Z), _fixed);
		wheelLB->getChBody()->SetInertiaXX(inertia_wheel);
		wheelLB->getChBody()->SetFriction(wheel_fric);
		wheelLB->deletable = false;

		double spring_k_front = 23800;//9393;
		double spring_k_rear = 23800;//12750;
		//Right front wheel

		link_revoluteRF->Initialize(wheelRF->getChBody(), spindleRF->getChBody(),
			chrono::ChCoordsys<>(chrono::ChVector<>(1.5, 1, 1) + Pos, chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Y)));
		m_pApp->getChSystem()->AddLink(link_revoluteRF);

		link_distRFU1->Initialize(truss->getChBody(), spindleRF->getChBody(), false, chrono::ChVector<>(0.5, 1.2, 1.2) + Pos, chrono::ChVector<>(1.25, 1.2, 1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distRFU1);

		link_distRFU2->Initialize(truss->getChBody(), spindleRF->getChBody(), false, chrono::ChVector<>(0.5, 1.2, 0.8) + Pos, chrono::ChVector<>(1.25, 1.2, 1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distRFU2);

		link_distRFL1->Initialize(truss->getChBody(), spindleRF->getChBody(), false, chrono::ChVector<>(0.5, 0.8, 1.2) + Pos, chrono::ChVector<>(1.25, 0.8, 1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distRFL1);

		link_distRFL2->Initialize(truss->getChBody(), spindleRF->getChBody(), false, chrono::ChVector<>(0.5, 0.8, 0.8) + Pos, chrono::ChVector<>(1.25, 0.8, 1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distRFL2);

		link_springRF->Initialize(truss->getChBody(), spindleRF->getChBody(), false, chrono::ChVector<>(0.5, 1.2, 1.0) + Pos, chrono::ChVector<>(1.25, 0.8, 1) + Pos);
		link_springRF->Set_SpringK(spring_k_front);
		link_springRF->Set_SpringR(80);
		m_pApp->getChSystem()->AddLink(link_springRF);

		link_distRSTEER->Initialize(truss->getChBody(), spindleRF->getChBody(), false, chrono::ChVector<>(0.5, 1.21, 1.4) + Pos, chrono::ChVector<>(1.25, 1.21, 1.3) + Pos);
		m_pApp->getChSystem()->AddLink(link_distRSTEER);

		//Left front wheel

		link_revoluteLF->Initialize(wheelLF->getChBody(), spindleLF->getChBody(),
			chrono::ChCoordsys<>(chrono::ChVector<>(-1.5, 1, 1) + Pos, chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Y)));
		m_pApp->getChSystem()->AddLink(link_revoluteLF);

		link_distLFU1->Initialize(truss->getChBody(), spindleLF->getChBody(), false, chrono::ChVector<>(-0.5, 1.2, 1.2) + Pos, chrono::ChVector<>(-1.25, 1.2, 1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distLFU1);

		link_distLFU2->Initialize(truss->getChBody(), spindleLF->getChBody(), false, chrono::ChVector<>(-0.5, 1.2, 0.8) + Pos, chrono::ChVector<>(-1.25, 1.2, 1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distLFU2);

		link_distLFL1->Initialize(truss->getChBody(), spindleLF->getChBody(), false, chrono::ChVector<>(-0.5, 0.8, 1.2) + Pos, chrono::ChVector<>(-1.25, 0.8, 1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distLFL1);

		link_distLFL2->Initialize(truss->getChBody(), spindleLF->getChBody(), false, chrono::ChVector<>(-0.5, 0.8, 0.8) + Pos, chrono::ChVector<>(-1.25, 0.8, 1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distLFL2);

		link_springLF->Initialize(truss->getChBody(), spindleLF->getChBody(), false, chrono::ChVector<>(-0.5, 1.2, 1.0) + Pos, chrono::ChVector<>(-1.25, 0.8, 1) + Pos);
		link_springLF->Set_SpringK(spring_k_front);
		link_springLF->Set_SpringR(80);
		m_pApp->getChSystem()->AddLink(link_springLF);

		link_distLSTEER->Initialize(truss->getChBody(), spindleLF->getChBody(), false, chrono::ChVector<>(-0.5, 1.21, 1.4) + Pos, chrono::ChVector<>(-1.25, 1.21, 1.3) + Pos);
		m_pApp->getChSystem()->AddLink(link_distLSTEER);

		//Right back wheel

		link_revoluteRB->Initialize(wheelRB->getChBody(), spindleRB->getChBody(),
			chrono::ChCoordsys<>(chrono::ChVector<>(1.5, 1, -1) + Pos, chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Y)));
		m_pApp->getChSystem()->AddLink(link_revoluteRB);

		link_engineR->Initialize(wheelRB->getChBody(), truss->getChBody(),
			chrono::ChCoordsys<>(chrono::ChVector<>(1.5, 1, -1) + Pos, chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Y)));
		link_engineR->Set_shaft_mode(chrono::ChLinkEngine::ENG_SHAFT_CARDANO);
		link_engineR->Set_eng_mode(chrono::ChLinkEngine::ENG_MODE_TORQUE);
		m_pApp->getChSystem()->AddLink(link_engineR);

		link_distRBU1->Initialize(truss->getChBody(), spindleRB->getChBody(), false, chrono::ChVector<>(0.5, 1.2, -1.2) + Pos, chrono::ChVector<>(1.25, 1.2, -1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distRBU1);

		link_distRBU2->Initialize(truss->getChBody(), spindleRB->getChBody(), false, chrono::ChVector<>(0.5, 1.2, -0.8) + Pos, chrono::ChVector<>(1.25, 1.2, -1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distRBU2);

		link_distRBL1->Initialize(truss->getChBody(), spindleRB->getChBody(), false, chrono::ChVector<>(0.5, 0.8, -1.2) + Pos, chrono::ChVector<>(1.25, 0.8, -1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distRBL1);

		link_distRBL2->Initialize(truss->getChBody(), spindleRB->getChBody(), false, chrono::ChVector<>(0.5, 0.8, -0.8) + Pos, chrono::ChVector<>(1.25, 0.8, -1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distRBL2);

		link_springRB->Initialize(truss->getChBody(), spindleRB->getChBody(), false, chrono::ChVector<>(0.5, 1.2, -1.0) + Pos, chrono::ChVector<>(1.25, 0.8, -1) + Pos);
		link_springRB->Set_SpringK(spring_k_rear);
		link_springRB->Set_SpringR(80);
		m_pApp->getChSystem()->AddLink(link_springRB);

		link_distRBlat->Initialize(truss->getChBody(), spindleRB->getChBody(), false, chrono::ChVector<>(0.5, 1.21, -1.4) + Pos, chrono::ChVector<>(1.25, 1.21, -1.3) + Pos);
		m_pApp->getChSystem()->AddLink(link_distRBlat);

		//Left back wheel

		link_revoluteLB->Initialize(wheelLB->getChBody(), spindleLB->getChBody(),
			chrono::ChCoordsys<>(chrono::ChVector<>(-1.5, 1, -1) + Pos, chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Y)));
		m_pApp->getChSystem()->AddLink(link_revoluteLB);

		link_engineL->Initialize(wheelLB->getChBody(), truss->getChBody(),
			chrono::ChCoordsys<>(chrono::ChVector<>(-1.5, 1, -1) + Pos, chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Y)));
		link_engineL->Set_shaft_mode(chrono::ChLinkEngine::ENG_SHAFT_CARDANO);
		link_engineL->Set_eng_mode(chrono::ChLinkEngine::ENG_MODE_TORQUE);
		m_pApp->getChSystem()->AddLink(link_engineL);

		link_distLBU1->Initialize(truss->getChBody(), spindleLB->getChBody(), false, chrono::ChVector<>(-0.5, 1.2, -1.2) + Pos, chrono::ChVector<>(-1.25, 1.2, -1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distLBU1);

		link_distLBU2->Initialize(truss->getChBody(), spindleLB->getChBody(), false, chrono::ChVector<>(-0.5, 1.2, -0.8) + Pos, chrono::ChVector<>(-1.25, 1.2, -1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distLBU2);

		link_distLBL1->Initialize(truss->getChBody(), spindleLB->getChBody(), false, chrono::ChVector<>(-0.5, 0.8, -1.2) + Pos, chrono::ChVector<>(-1.25, 0.8, -1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distLBL1);

		link_distLBL2->Initialize(truss->getChBody(), spindleLB->getChBody(), false, chrono::ChVector<>(-0.5, 0.8, -0.8) + Pos, chrono::ChVector<>(-1.25, 0.8, -1) + Pos);
		m_pApp->getChSystem()->AddLink(link_distLBL2);

		link_springLB->Initialize(truss->getChBody(), spindleLB->getChBody(), false, chrono::ChVector<>(-0.5, 1.2, -1.0) + Pos, chrono::ChVector<>(-1.25, 0.8, -1) + Pos);
		link_springLB->Set_SpringK(spring_k_rear);
		link_springLB->Set_SpringR(80);
		m_pApp->getChSystem()->AddLink(link_springLB);

		link_distLBlat->Initialize(truss->getChBody(), spindleLB->getChBody(), false, chrono::ChVector<>(-0.5, 1.21, -1.4) + Pos, chrono::ChVector<>(-1.25, 1.21, -1.3) + Pos);
		m_pApp->getChSystem()->AddLink(link_distLBlat);
	}

	void VEHumvee::update() {

	}

	void VEHumvee::reset(chrono::ChVector<>& Pos) {
		truss->getChBody()->SetPos(chrono::ChVector<>(0, 1, 0) + Pos);
		truss->getChBody()->SetRot(chrono::QUNIT);
		truss->getChBody()->SetPos_dt(chrono::ChVector<>(0, 0, 0));
		truss->getChBody()->SetRot_dt(chrono::QUNIT);

		spindleRF->getChBody()->SetPos(chrono::ChVector<>(1.3, 1, 1) + Pos);
		spindleRF->getChBody()->SetRot(chrono::QUNIT);
		spindleRF->getChBody()->SetPos_dt(chrono::ChVector<>(0, 0, 0));
		spindleRF->getChBody()->SetRot_dt(chrono::QUNIT);

		wheelRF->getChBody()->SetPos(chrono::ChVector<>(1.5, 1, 1) + Pos);
		wheelRF->getChBody()->SetRot(chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Z));
		wheelRF->getChBody()->SetPos_dt(chrono::ChVector<>(0, 0, 0));
		wheelRF->getChBody()->SetRot_dt(chrono::QUNIT);

		spindleLF->getChBody()->SetPos(chrono::ChVector<>(-1.3, 1, 1) + Pos);
		spindleLF->getChBody()->SetRot(chrono::QUNIT);
		spindleLF->getChBody()->SetPos_dt(chrono::ChVector<>(0, 0, 0));
		spindleLF->getChBody()->SetRot_dt(chrono::QUNIT);

		wheelLF->getChBody()->SetPos(chrono::ChVector<>(-1.5, 1, 1) + Pos);
		wheelLF->getChBody()->SetRot(chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Z));
		wheelLF->getChBody()->SetPos_dt(chrono::ChVector<>(0, 0, 0));
		wheelLF->getChBody()->SetRot_dt(chrono::QUNIT);

		spindleRB->getChBody()->SetPos(chrono::ChVector<>(1.3, 1, -1) + Pos);
		spindleRB->getChBody()->SetRot(chrono::QUNIT);
		spindleRB->getChBody()->SetPos_dt(chrono::ChVector<>(0, 0, 0));
		spindleRB->getChBody()->SetRot_dt(chrono::QUNIT);

		wheelRB->getChBody()->SetPos(chrono::ChVector<>(1.5, 1, -1) + Pos);
		wheelRB->getChBody()->SetRot(chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Z));
		wheelRB->getChBody()->SetPos_dt(chrono::ChVector<>(0, 0, 0));
		wheelRB->getChBody()->SetRot_dt(chrono::QUNIT);

		spindleLB->getChBody()->SetPos(chrono::ChVector<>(-1.3, 1, -1) + Pos);
		spindleLB->getChBody()->SetRot(chrono::QUNIT);
		spindleLB->getChBody()->SetPos_dt(chrono::ChVector<>(0, 0, 0));
		spindleLB->getChBody()->SetRot_dt(chrono::QUNIT);

		wheelLB->getChBody()->SetPos(chrono::ChVector<>(-1.5, 1, -1) + Pos);
		wheelLB->getChBody()->SetRot(chrono::Q_from_AngAxis(chrono::CH_C_PI / 2, chrono::VECT_Z));
		wheelLB->getChBody()->SetPos_dt(chrono::ChVector<>(0, 0, 0));
		wheelLB->getChBody()->SetRot_dt(chrono::QUNIT);
	}

	void VEHumvee::shift(uint8_t gear) {

	}

	void VEHumvee::brake() {
		wheelRF->getChBody()->SetRot_dt(chrono::QUNIT);
		wheelLF->getChBody()->SetRot_dt(chrono::QUNIT);
		wheelRB->getChBody()->SetRot_dt(chrono::QUNIT);
		wheelLB->getChBody()->SetRot_dt(chrono::QUNIT);
	}

    chrono::ChSharedPtr<ChBody> VEHumvee::getChassis() {
		return truss->getChBody();
	}


	chrono::ChVector<> VEHumvee::getFrontLocation(PointId which) {
		switch (which) {
		case SPINDLE:  return in2m * chrono::ChVector<>(1.59, -1.0350, -23.72);
		case UPRIGHT:  return in2m * chrono::ChVector<>(1.59, -1.0350, -19.72);
		case UCA_F:    return in2m * chrono::ChVector<>(1.89, 9.63, -5.46);
		case UCA_B:    return in2m * chrono::ChVector<>(10.56, 7.69, -5.46);
		case UCA_U:    return in2m * chrono::ChVector<>(2.09, 8.48, -16.07);
		case LCA_F:    return in2m * chrono::ChVector<>(-8.79, 0, 0);
		case LCA_B:    return in2m * chrono::ChVector<>(8.79, 0, 0);
		case LCA_U:    return in2m * chrono::ChVector<>(1.40, -4.65, -18.87);
		case SHOCK_C:  return in2m * chrono::ChVector<>(-4.10, 12.72, -15.77);
		case SHOCK_U:  return in2m * chrono::ChVector<>(-3.83, -1.52, -18.87);
		case TIEROD_C: return in2m * chrono::ChVector<>(13.39, -1.0350, 2.29);
		case TIEROD_U: return in2m * chrono::ChVector<>(6.92, -1.0350, -20.22);
		default:       return chrono::ChVector<>(0, 0, 0);
		}
	}

	chrono::ChVector<> VEHumvee::getRearLocation(PointId which) {
		switch (which) {
		case SPINDLE:  return in2m * chrono::ChVector<>(-1.40, 23.72, -1.035);
		case UPRIGHT:  return in2m * chrono::ChVector<>(-1.40, 19.72, -1.035);
		case UCA_F:    return in2m * chrono::ChVector<>(-13.78, 6.10, 8.88);
		case UCA_B:    return in2m * chrono::ChVector<>(-3.07, 6.10, 8.88);
		case UCA_U:    return in2m * chrono::ChVector<>(-1.40, 16.07, 9.28);
		case LCA_F:    return in2m * chrono::ChVector<>(-8.79, 0, 0);
		case LCA_B:    return in2m * chrono::ChVector<>(8.79, 0, 0);
		case LCA_U:    return in2m * chrono::ChVector<>(-1.40, 18.87, -4.65);
		case SHOCK_C:  return in2m * chrono::ChVector<>(4.09, 16.10, 12.72);
		case SHOCK_U:  return in2m * chrono::ChVector<>(4.09, 18.87, -1.51);
		case TIEROD_C: return in2m * chrono::ChVector<>(-12.70, 4.28, -0.37);
		case TIEROD_U: return in2m * chrono::ChVector<>(-6.70, 20.23, -0.37);
		default:       return chrono::ChVector<>(0, 0, 0);
		}
	}

}