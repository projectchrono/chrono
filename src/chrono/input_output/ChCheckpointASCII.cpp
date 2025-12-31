// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a Chrono checkpoint database.
//
// =============================================================================

#include <iomanip>
#include <iostream>

#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearDriveline.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationDriveline.h"
#include "chrono/physics/ChShaftsMotorSpeed.h"

#include "chrono/input_output/ChCheckpointASCII.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {

ChCheckpointASCII::ChCheckpointASCII(Type type) : ChCheckpoint(type), m_np(0), m_nv(0) {
    m_csv.SetDelimiter(" ");

    constexpr auto precision{std::numeric_limits<long double>::digits10 + 1};
    m_csv.Stream() << std::setprecision(precision);
}

ChCheckpointASCII::~ChCheckpointASCII() {}

void ChCheckpointASCII::Initialize() {}

// -----------------------------------------------------------------------------

void ChCheckpointASCII::WriteFile(const std::string& filename) {
    std::ofstream ofile(filename);
    ofile << GetTypeAsString(m_type) << endl;
    ofile << m_np << " " << m_nv << endl;
    ofile << m_csv.Stream().str();
    ofile.close();
}

void ChCheckpointASCII::OpenFile(const std::string& filename) {
    try {
        m_ifile.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
        m_ifile.open(filename);
    } catch (const std::exception&) {
        cerr << "Error: Cannot open ASCII checkpoint file " << filename << endl;
        throw std::invalid_argument("Cannot open ASCII checkpoint file");
    }

    std::string line;

    // Read checkpoint type
    std::getline(m_ifile, line);
    {
        std::string type;

        std::istringstream iss(line);
        iss >> type;

        if (type != GetTypeAsString(m_type)) {
            cerr << "Error: incorrect checkpoint file type" << endl;
            throw std::runtime_error("Incorrect checkpoint file type");
        }
        cout << "Open file " << filename << " with " << type << " checkpoint." << endl;
    }

    // Read number of states
    std::getline(m_ifile, line);
    {
        std::istringstream iss(line);
        iss >> m_np >> m_nv;
    }
}

void ChCheckpointASCII::CheckIfOpen() const {
    if (!m_ifile.is_open()) {
        cerr << "Error: input checkpoint file not open" << endl;
        throw std::runtime_error("Input checkpoint file not open");
    }
}

// -----------------------------------------------------------------------------

void ChCheckpointASCII::WriteState(ChSystem* sys) {
    CheckIfSystemType();

    m_np = sys->GetNumCoordsPosLevel();
    m_nv = sys->GetNumCoordsVelLevel();
    ChState x(m_np, sys);
    ChStateDelta v(m_nv, sys);
    double time;
    sys->StateGather(x, v, time);
    m_csv << time << endl;
    m_csv << x << endl;
    m_csv << v << endl;
}

void ChCheckpointASCII::ReadState(ChSystem* sys) {
    CheckIfOpen();
    CheckIfSystemType();

    // Force a calculation of state counters
    sys->Setup();

    auto np = sys->GetNumCoordsPosLevel();
    auto nv = sys->GetNumCoordsVelLevel();

    // Check number of states
    if (m_np != np || m_nv != nv) {
        cerr << "Error: inconsistent number of states" << endl;
        throw std::runtime_error("Inconsistent number of states");
    }

    std::string line;

    // Read time
    double time;
    {
        std::getline(m_ifile, line);
        std::istringstream iss(line);
        iss >> time;
    }

    // Read states
    ChState x(np, sys);
    ChStateDelta v(nv, sys);
    for (size_t i = 0; i < m_np; i++) {
        std::getline(m_ifile, line);
        std::istringstream iss(line);
        iss >> x[i];
    }
    for (size_t i = 0; i < m_nv; i++) {
        std::getline(m_ifile, line);
        std::istringstream iss(line);
        iss >> v[i];
    }

    sys->StateScatter(x, v, time, true);
}

// -----------------------------------------------------------------------------

void ChCheckpointASCII::WriteTime(double time) {
    CheckIfComponentType();

    m_csv << time << endl;
}

void ChCheckpointASCII::WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
    CheckIfComponentType();

    for (const auto& body : bodies) {
        const auto& pos = body->GetPos();
        const auto& rot = body->GetRot();
        const auto& pos_dt = body->GetPosDt();
        const auto& rot_dt = body->GetRotDt();

        m_csv << pos << rot << pos_dt << rot_dt << endl;
    }

    m_np += 7 * bodies.size();
    m_nv += 6 * bodies.size();
}

void ChCheckpointASCII::WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) {
    CheckIfComponentType();

    for (const auto& shaft : shafts) {
        double pos = shaft->GetPos();
        double pos_dt = shaft->GetPosDt();

        m_csv << pos << pos_dt << endl;
    }

    m_np += shafts.size();
    m_nv += shafts.size();
}

void ChCheckpointASCII::WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) {
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    CheckIfComponentType();

    for (const auto& motor : couples) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChShaftsMotorSpeed>(motor)) {
            m_csv << motor_speed->Variables().State()(0, 0) << endl;
            m_np += 1;
        }
    }
}

void ChCheckpointASCII::WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    CheckIfComponentType();

    for (const auto& spring : springs) {
        auto num_states = spring->GetStates().size();
        if (num_states > 0)
            m_csv << spring->GetStates() << endl;
        m_np += num_states;
    }
}

void ChCheckpointASCII::WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
    CheckIfComponentType();

    for (const auto& motor : motors) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChLinkMotorLinearSpeed>(motor)) {
            m_csv << motor_speed->Variables().State()(0, 0) << endl;
            m_np += 1;
        } else if (auto motor_drvl = std::dynamic_pointer_cast<ChLinkMotorLinearDriveline>(motor)) {
            auto s1 = motor_drvl->GetInnerShaft1Lin();
            auto s2 = motor_drvl->GetInnerShaft2Lin();
            auto s3 = motor_drvl->GetInnerShaft2Rot();
            m_csv << s1->GetPos() << s2->GetPos() << s3->GetPos()                 //
                  << s1->GetPosDt() << s2->GetPosDt() << s3->GetPosDt() << endl;  //
            m_np += 3;
            m_nv += 3;
        }
    }
}

void ChCheckpointASCII::WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) {
    CheckIfComponentType();

    for (const auto& motor : motors) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChLinkMotorRotationSpeed>(motor)) {
            m_csv << motor_speed->Variables().State()(0, 0) << endl;
            m_np += 1;
        } else if (auto motor_drvl = std::dynamic_pointer_cast<ChLinkMotorRotationDriveline>(motor)) {
            auto s1 = motor_drvl->GetInnerShaft1();
            auto s2 = motor_drvl->GetInnerShaft2();
            m_csv << s1->GetPos() << s2->GetPos()               //
                  << s1->GetPosDt() << s2->GetPosDt() << endl;  //
            m_np += 2;
            m_nv += 2;
        }
    }
}

// -----------------------------------------------------------------------------

void ChCheckpointASCII::ReadTime(double& time) {
    CheckIfOpen();
    CheckIfComponentType();

    std::string line;
    std::getline(m_ifile, line);
    std::istringstream iss(line);
    iss >> time;
}

void ChCheckpointASCII::ReadBodies(std::vector<std::shared_ptr<ChBody>>& bodies) {
    CheckIfOpen();
    CheckIfComponentType();

    ChVector3d pos;
    ChQuaterniond rot;
    ChVector3d pos_dt;
    ChQuaterniond rot_dt;

    std::string line;
    for (auto& body : bodies) {
        std::getline(m_ifile, line);
        std::istringstream iss(line);
        iss >> pos.x() >> pos.y() >> pos.z() >> rot.e0() >> rot.e1() >> rot.e2() >> rot.e3()                        //
            >> pos_dt.x() >> pos_dt.y() >> pos_dt.z() >> rot_dt.e0() >> rot_dt.e1() >> rot_dt.e2() >> rot_dt.e3();  //
        body->SetPos(pos);
        body->SetRot(rot);
        body->SetPosDt(pos_dt);
        body->SetRotDt(rot_dt);
    }
}

void ChCheckpointASCII::ReadShafts(std::vector<std::shared_ptr<ChShaft>>& shafts) {
    CheckIfOpen();
    CheckIfComponentType();

    double pos;
    double pos_dt;

    std::string line;
    for (auto& shaft : shafts) {
        std::getline(m_ifile, line);
        std::istringstream iss(line);
        iss >> pos >> pos_dt;
        shaft->SetPos(pos);
        shaft->SetPosDt(pos_dt);
    }
}

void ChCheckpointASCII::ReadJoints(std::vector<std::shared_ptr<ChLink>>& joints) {
    CheckIfOpen();
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::ReadCouples(std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    CheckIfOpen();
    CheckIfComponentType();

    double state;

    std::string line;
    for (const auto& couple : couples) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChShaftsMotorSpeed>(couple)) {
            std::getline(m_ifile, line);
            std::istringstream iss(line);
            iss >> state;
            motor_speed->Variables().State()(0, 0) = state;
        }
    }
}

void ChCheckpointASCII::ReadLinSprings(std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    CheckIfOpen();
    CheckIfComponentType();

    std::string line;
    for (auto& spring : springs) {
        auto num_states = spring->GetStates().size();
        if (spring->GetStates().size() > 0) {
            std::getline(m_ifile, line);
            std::istringstream iss(line);
            ChVectorDynamic<> states(num_states);
            for (int i = 0; i < num_states; i++)
                iss >> states[i];
            spring->SetStates(states);
        }
    }
}

void ChCheckpointASCII::ReadRotSprings(std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    CheckIfOpen();
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::ReadBodyBodyLoads(std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    CheckIfOpen();
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::ReadLinMotors(std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
    CheckIfOpen();
    CheckIfComponentType();

    double state;

    double pos1, pos2, pos3;
    double pos1_dt, pos2_dt, pos3_dt;

    std::string line;
    for (const auto& motor : motors) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChLinkMotorLinearSpeed>(motor)) {
            std::getline(m_ifile, line);
            std::istringstream iss(line);
            iss >> state;
            motor_speed->Variables().State()(0, 0) = state;
        } else if (auto motor_drvl = std::dynamic_pointer_cast<ChLinkMotorLinearDriveline>(motor)) {
            std::getline(m_ifile, line);
            std::istringstream iss(line);
            iss >> pos1 >> pos2 >> pos3 >> pos1_dt >> pos2_dt >> pos3_dt;
            auto s1 = motor_drvl->GetInnerShaft1Lin();
            auto s2 = motor_drvl->GetInnerShaft2Lin();
            auto s3 = motor_drvl->GetInnerShaft2Rot();
            s1->SetPos(pos1);
            s2->SetPos(pos2);
            s3->SetPos(pos3);
            s1->SetPosDt(pos1_dt);
            s2->SetPosDt(pos2_dt);
            s3->SetPosDt(pos3_dt);
        }
    }
}

void ChCheckpointASCII::ReadRotMotors(std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) {
    CheckIfOpen();
    CheckIfComponentType();

    double state;

    double pos1, pos2;
    double pos1_dt, pos2_dt;

    std::string line;
    for (const auto& motor : motors) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChLinkMotorRotationSpeed>(motor)) {
            std::getline(m_ifile, line);
            std::istringstream iss(line);
            iss >> state;
            motor_speed->Variables().State()(0, 0) = state;
        } else if (auto motor_drvl = std::dynamic_pointer_cast<ChLinkMotorRotationDriveline>(motor)) {
            std::getline(m_ifile, line);
            std::istringstream iss(line);
            iss >> pos1 >> pos2 >> pos1_dt >> pos2_dt;
            auto s1 = motor_drvl->GetInnerShaft1();
            auto s2 = motor_drvl->GetInnerShaft2();
            s1->SetPos(pos1);
            s2->SetPos(pos2);
            s1->SetPosDt(pos1_dt);
            s2->SetPosDt(pos2_dt);
        }
    }
}

}  // namespace chrono
