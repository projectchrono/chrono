// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban, Rainer Gericke
// =============================================================================
//
// 8WD driveline model template based on ChShaft objects.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline8WD.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// dir_motor_block specifies the direction of the driveshaft, i.e. the input of
// the conic gear pair, in chassis local coords.
//
// dir_axle specifies the direction of the axle, i.e. the output of the conic
// conic gear pair, in chassis local coords. This is needed because ChShaftsBody
// could transfer pitch torque to the chassis.
// -----------------------------------------------------------------------------
ChShaftsDriveline8WD::ChShaftsDriveline8WD(const std::string& name)
    : ChDrivelineWV(name), m_dir_motor_block(ChVector<>(1, 0, 0)), m_dir_axle(ChVector<>(0, 1, 0)) {}

ChShaftsDriveline8WD::~ChShaftsDriveline8WD() {
    auto sys = m_central_differential->GetSystem();
    if (sys) {
        sys->Remove(m_driveshaft);
        sys->Remove(m_central_differential);
        sys->Remove(m_central_clutch);

        for (int i = 0; i < 2; i++) {
            sys->Remove(m_GD_inshaft[i]);
            sys->Remove(m_GD_differential[i]);
            sys->Remove(m_GD_clutch[i]);
        }

        for (int i = 0; i < 4; i++) {
            sys->Remove(m_AD_inshaft[i]);
            sys->Remove(m_AD_conicalgear[i]);
            sys->Remove(m_AD_differential[i]);
            sys->Remove(m_AD_differentialbox[i]);
            sys->Remove(m_AD_clutch[i]);
        }
    }
}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline to the specified axles.
// -----------------------------------------------------------------------------
void ChShaftsDriveline8WD::Initialize(std::shared_ptr<ChChassis> chassis,
                                      const ChAxleList& axles,
                                      const std::vector<int>& driven_axles) {
    ChDriveline::Initialize(chassis);

    assert(axles.size() >= 4);
    assert(driven_axles.size() == 4);

    m_driven_axles = driven_axles;

    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Create the driveshaft for the connection of the driveline to the transmission box.
    m_driveshaft = chrono_types::make_shared<ChShaft>();
    m_driveshaft->SetInertia(GetDriveshaftInertia());
    sys->AddShaft(m_driveshaft);

    // Create the 7 differentials. For a differential, the transmission ratio in Willis formula must be set to -1.
    // For each differential, specify the input shaft (the carrier) and the two output shafts.
    // Attach a clutch to each differential to control differential locking.
    // For each of the 4 axle differentials, also create a shaft representing the inertia of the roatating box of the
    // differential and a conical gear to connect the potentially non-parallel wheel shafts.

    // Axle differentials and conical gears
    for (int i = 0; i < 4; i++) {
        // Input shaft
        m_AD_inshaft[i] = chrono_types::make_shared<ChShaft>();
        m_AD_inshaft[i]->SetInertia(GetAxleDiffInputShaftInertia());
        sys->AddShaft(m_AD_inshaft[i]);

        // Shaft representing inertia of the differential rotating box
        m_AD_differentialbox[i] = chrono_types::make_shared<ChShaft>();
        m_AD_differentialbox[i]->SetInertia(GetAxleDiffBoxInertia());
        sys->AddShaft(m_AD_differentialbox[i]);

        // Conical gear
        m_AD_conicalgear[i] = chrono_types::make_shared<ChShaftsGearboxAngled>();
        m_AD_conicalgear[i]->Initialize(m_AD_inshaft[i], m_AD_differentialbox[i], chassisBody, m_dir_motor_block,
                                        m_dir_axle);
        m_AD_conicalgear[i]->SetTransmissionRatio(-GetAxleDiffConicalGearRatio());
        sys->Add(m_AD_conicalgear[i]);

        // Differential
        m_AD_differential[i] = chrono_types::make_shared<ChShaftsPlanetary>();
        m_AD_differential[i]->Initialize(m_AD_differentialbox[i], axles[m_driven_axles[i]]->m_suspension->GetAxle(LEFT),
                                         axles[m_driven_axles[i]]->m_suspension->GetAxle(RIGHT));
        m_AD_differential[i]->SetTransmissionRatioOrdinary(-1.0);
        sys->Add(m_AD_differential[i]);

        // Clutch
        m_AD_clutch[i] = chrono_types::make_shared<ChShaftsClutch>();
        m_AD_clutch[i]->Initialize(axles[m_driven_axles[i]]->m_suspension->GetAxle(LEFT),
                                   axles[m_driven_axles[i]]->m_suspension->GetAxle(RIGHT));
        m_AD_clutch[i]->SetTorqueLimit(GetAxleDifferentialLockingLimit());
        m_AD_clutch[i]->SetModulation(0);
        sys->Add(m_AD_clutch[i]);
    }

    // Group differentials
    for (int i = 0; i < 2; i++) {
        // Input shaft
        m_GD_inshaft[i] = chrono_types::make_shared<ChShaft>();
        m_GD_inshaft[i]->SetInertia(GetGroupDiffInputShaftInertia());
        sys->AddShaft(m_GD_inshaft[i]);

        // Differential
        m_GD_differential[i] = chrono_types::make_shared<ChShaftsPlanetary>();
        m_GD_differential[i]->Initialize(m_GD_inshaft[i], m_AD_inshaft[2 * i + 1], m_AD_inshaft[2 * i]);
        m_GD_differential[i]->SetTransmissionRatioOrdinary(-1.0);
        sys->Add(m_GD_differential[i]);

        // Clutch
        m_GD_clutch[i] = chrono_types::make_shared<ChShaftsClutch>();
        m_GD_clutch[i]->Initialize(m_AD_inshaft[2 * i + 1], m_AD_inshaft[2 * i]);
        m_GD_clutch[i]->SetTorqueLimit(GetGroupDifferentialLockingLimit());
        m_GD_clutch[i]->SetModulation(0);
        sys->Add(m_GD_clutch[i]);
    }

    // Central differential
    m_central_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_central_differential->Initialize(m_driveshaft, m_GD_inshaft[1], m_GD_inshaft[0]);
    m_central_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_central_differential);

    // Clutch for central differential locking. By default, unlocked.
    m_central_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_central_clutch->Initialize(m_GD_inshaft[1], m_GD_inshaft[0]);
    m_central_clutch->SetTorqueLimit(GetCentralDifferentialLockingLimit());
    m_central_clutch->SetModulation(0);
    sys->Add(m_central_clutch);

    // Initialize shaft angular velocities based on the initial wheel angular velocities.
    // Propagate from wheels to axle differentials to group differentials and finally to the central differential.

    double omega_AD_inshaft[4];
    for (int i = 0; i < 4; i++) {
        double omega_L = axles[m_driven_axles[i]]->m_suspension->GetAxleSpeed(LEFT);
        double omega_R = axles[m_driven_axles[i]]->m_suspension->GetAxleSpeed(RIGHT);

        double omega_diffbox = 0.5 * (omega_L + omega_R);
        m_AD_differentialbox[i]->SetPos_dt(omega_diffbox);

        omega_AD_inshaft[i] = omega_diffbox / GetAxleDiffConicalGearRatio();
        m_AD_inshaft[i]->SetPos_dt(omega_AD_inshaft[i]);
    }

    double omega_GD_inshaft[2];
    for (int i = 0; i < 2; i++) {
        omega_GD_inshaft[i] = 0.5 * (omega_AD_inshaft[2 *i] + omega_AD_inshaft[2*i+1]);
        m_GD_inshaft[i]->SetPos_dt(omega_GD_inshaft[i]);
    }

    double omega_driveshaft = 0.5 * (omega_GD_inshaft[0] + omega_GD_inshaft[1]);
    m_driveshaft->SetPos_dt(omega_driveshaft);
}

// -----------------------------------------------------------------------------
void ChShaftsDriveline8WD::Synchronize(double time, const DriverInputs& driver_inputs, double driveshaft_torque) {
    m_driveshaft->SetAppliedTorque(driveshaft_torque);
}

// -----------------------------------------------------------------------------
// Differential locking
// -----------------------------------------------------------------------------
void ChShaftsDriveline8WD::LockAxleDifferential(int axle, bool lock) {
    if (axle == -1) {
        for (int i = 0; i < 4; i++)
            m_AD_clutch[i]->SetModulation(lock ? 1.0 : 0.0);
        return;
    }

    for (int i = 0; i < 4; i++) {
        if (axle == m_driven_axles[i]) {
            m_AD_clutch[i]->SetModulation(lock ? 1.0 : 0.0);
            return;
        }
    }
}

void ChShaftsDriveline8WD::LockCentralDifferential(int which, bool lock) {
    switch (which) {
        case 0:
            m_GD_clutch[0]->SetModulation(lock ? 1 : 0);
            return;
        case 1:
            m_GD_clutch[1]->SetModulation(lock ? 1 : 0);
            return;
        case 2:
            m_central_clutch->SetModulation(lock ? 1 : 0);
            return;
        case -1:
            m_GD_clutch[0]->SetModulation(lock ? 1 : 0);
            m_GD_clutch[1]->SetModulation(lock ? 1 : 0);
            m_central_clutch->SetModulation(lock ? 1 : 0);
            return;
    }
}

// -----------------------------------------------------------------------------
double ChShaftsDriveline8WD::GetSpindleTorque(int axle, VehicleSide side) const {
    for (int i = 0; i < 4; i++) {
        if (axle == m_driven_axles[i]) {
            switch (side) {
                case LEFT:
                    return -m_AD_differential[i]->GetTorqueReactionOn2() - m_AD_differential[i]->GetTorqueReactionOn1();
                case RIGHT:
                    return -m_AD_differential[i]->GetTorqueReactionOn3() - m_AD_differential[i]->GetTorqueReactionOn2();
            }       
        }
    }

    return 0;
}

// -----------------------------------------------------------------------------
void ChShaftsDriveline8WD::Disconnect() {
    for (int i = 0; i < 4; i++) {
        m_AD_differential[i]->SetDisabled(true);
        m_AD_clutch[i]->SetDisabled(true);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
