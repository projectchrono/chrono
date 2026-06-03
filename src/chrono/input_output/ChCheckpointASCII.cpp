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

ChCheckpointASCII::ChCheckpointASCII(Type type) : ChCheckpoint(type), m_file_read(false), m_np(0), m_nv(0) {
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
    ofile << m_time << endl;
    ofile << m_np << " " << m_nv << endl;
    ofile << m_csv.Stream().str();
    ofile.close();
}

void ChCheckpointASCII::ReadFile(const std::string& filename) {
    std::ifstream ifile;

    try {
        ifile.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
        ifile.open(filename);
    } catch (const std::exception&) {
        cerr << "Error: Cannot open ASCII checkpoint file " << filename << endl;
        throw std::invalid_argument("Cannot open ASCII checkpoint file");
    }

    std::string line;

    // Load checkpoint type
    std::getline(ifile, line);
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

    // Load checkpoint time
    std::getline(ifile, line);
    {
        std::istringstream iss(line);
        iss >> m_time;
    }

    // Load number of states
    std::getline(ifile, line);
    {
        std::istringstream iss(line);
        iss >> m_np >> m_nv;
    }

    // Load the remainder lines in the ChWriterCSV object
    auto str = ifile.rdbuf();
    m_csv.Stream() << str;

    // Flag the checkpint as loaded from file
    m_file_read = true;
    m_istream = std::istringstream(m_csv.Stream().str());
}

void ChCheckpointASCII::CheckIfOpen() const {
    if (!m_file_read) {
        cerr << "Error: input checkpoint file not open" << endl;
        throw std::runtime_error("Input checkpoint file not open");
    }
}

// -----------------------------------------------------------------------------

void ChCheckpointASCII::SaveState(ChSystem* sys) {
    CheckIfSystemType();

    m_np = sys->GetNumCoordsPosLevel();
    m_nv = sys->GetNumCoordsVelLevel();
    ChState x(m_np, sys);
    ChStateDelta v(m_nv, sys);
    double time;
    sys->StateGather(x, v, time);
    assert(time == m_time);
    m_csv << x << endl;
    m_csv << v << endl;
}

void ChCheckpointASCII::LoadState(ChSystem* sys) {
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

    // Load states
    ChState x(np, sys);
    ChStateDelta v(nv, sys);
    for (size_t i = 0; i < m_np; i++) {
        std::getline(m_istream, line);
        std::istringstream iss(line);
        iss >> x[i];
    }
    for (size_t i = 0; i < m_nv; i++) {
        std::getline(m_istream, line);
        std::istringstream iss(line);
        iss >> v[i];
    }

    sys->StateScatter(x, v, m_time, UpdateFlags::UPDATE_ALL);
}

// -----------------------------------------------------------------------------

void ChCheckpointASCII::SaveBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
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

void ChCheckpointASCII::SaveShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) {
    CheckIfComponentType();

    for (const auto& shaft : shafts) {
        double pos = shaft->GetPos();
        double pos_dt = shaft->GetPosDt();

        m_csv << pos << pos_dt << endl;
    }

    m_np += shafts.size();
    m_nv += shafts.size();
}

void ChCheckpointASCII::SaveJoints(const std::vector<std::shared_ptr<ChLink>>& joints) {
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::SaveCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    CheckIfComponentType();

    for (const auto& motor : couples) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChShaftsMotorSpeed>(motor)) {
            m_csv << motor_speed->Variables().State()(0, 0) << endl;
            m_np += 1;
        }
    }
}

void ChCheckpointASCII::SaveLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    CheckIfComponentType();

    for (const auto& spring : springs) {
        auto num_states = spring->GetStates().size();
        if (num_states > 0)
            m_csv << spring->GetStates() << endl;
        m_np += num_states;
    }
}

void ChCheckpointASCII::SaveRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::SaveBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::SaveLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
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

void ChCheckpointASCII::SaveRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) {
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

void ChCheckpointASCII::SaveDouble(double value) {
    CheckIfComponentType();
    m_csv << value << endl;
}

void ChCheckpointASCII::SaveInteger(int value) {
    CheckIfComponentType();
    m_csv << value << endl;
}

void ChCheckpointASCII::SaveVector(const std::vector<double>& vector) {
    CheckIfComponentType();
    m_csv << vector.size();
    for (auto v : vector)
        m_csv << v;
    m_csv << endl;
}

void ChCheckpointASCII::SaveChVector3(const ChVector3d& vector) {
    CheckIfComponentType();
    m_csv << vector << endl;
}

void ChCheckpointASCII::SaveChQuaternion(const ChQuaterniond& quat) {
    CheckIfComponentType();
    m_csv << quat << endl;
}

// -----------------------------------------------------------------------------

void ChCheckpointASCII::LoadBodies(std::vector<std::shared_ptr<ChBody>>& bodies) {
    CheckIfOpen();
    CheckIfComponentType();

    ChVector3d pos;
    ChQuaterniond rot;
    ChVector3d pos_dt;
    ChQuaterniond rot_dt;

    std::string line;
    for (auto& body : bodies) {
        std::getline(m_istream, line);
        std::istringstream iss(line);
        iss >> pos.x() >> pos.y() >> pos.z() >> rot.e0() >> rot.e1() >> rot.e2() >> rot.e3()                        //
            >> pos_dt.x() >> pos_dt.y() >> pos_dt.z() >> rot_dt.e0() >> rot_dt.e1() >> rot_dt.e2() >> rot_dt.e3();  //
        body->SetPos(pos);
        body->SetRot(rot);
        body->SetPosDt(pos_dt);
        body->SetRotDt(rot_dt);
    }
}

void ChCheckpointASCII::LoadShafts(std::vector<std::shared_ptr<ChShaft>>& shafts) {
    CheckIfOpen();
    CheckIfComponentType();

    double pos;
    double pos_dt;

    std::string line;
    for (auto& shaft : shafts) {
        std::getline(m_istream, line);
        std::istringstream iss(line);
        iss >> pos >> pos_dt;
        shaft->SetPos(pos);
        shaft->SetPosDt(pos_dt);
    }
}

void ChCheckpointASCII::LoadJoints(std::vector<std::shared_ptr<ChLink>>& joints) {
    CheckIfOpen();
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::LoadCouples(std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    CheckIfOpen();
    CheckIfComponentType();

    double state;

    std::string line;
    for (const auto& couple : couples) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChShaftsMotorSpeed>(couple)) {
            std::getline(m_istream, line);
            std::istringstream iss(line);
            iss >> state;
            motor_speed->Variables().State()(0, 0) = state;
        }
    }
}

void ChCheckpointASCII::LoadLinSprings(std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    CheckIfOpen();
    CheckIfComponentType();

    std::string line;
    for (auto& spring : springs) {
        auto num_states = spring->GetStates().size();
        if (spring->GetStates().size() > 0) {
            std::getline(m_istream, line);
            std::istringstream iss(line);
            ChVectorDynamic<> states(num_states);
            for (int i = 0; i < num_states; i++)
                iss >> states[i];
            spring->SetStates(states);
        }
    }
}

void ChCheckpointASCII::LoadRotSprings(std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    CheckIfOpen();
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::LoadBodyBodyLoads(std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    CheckIfOpen();
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::LoadLinMotors(std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
    CheckIfOpen();
    CheckIfComponentType();

    double state;

    double pos1, pos2, pos3;
    double pos1_dt, pos2_dt, pos3_dt;

    std::string line;
    for (const auto& motor : motors) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChLinkMotorLinearSpeed>(motor)) {
            std::getline(m_istream, line);
            std::istringstream iss(line);
            iss >> state;
            motor_speed->Variables().State()(0, 0) = state;
        } else if (auto motor_drvl = std::dynamic_pointer_cast<ChLinkMotorLinearDriveline>(motor)) {
            std::getline(m_istream, line);
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

void ChCheckpointASCII::LoadRotMotors(std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) {
    CheckIfOpen();
    CheckIfComponentType();

    double state;

    double pos1, pos2;
    double pos1_dt, pos2_dt;

    std::string line;
    for (const auto& motor : motors) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChLinkMotorRotationSpeed>(motor)) {
            std::getline(m_istream, line);
            std::istringstream iss(line);
            iss >> state;
            motor_speed->Variables().State()(0, 0) = state;
        } else if (auto motor_drvl = std::dynamic_pointer_cast<ChLinkMotorRotationDriveline>(motor)) {
            std::getline(m_istream, line);
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

void ChCheckpointASCII::LoadDouble(double& value) {
    CheckIfOpen();
    CheckIfComponentType();

    std::string line;
    std::getline(m_istream, line);
    std::istringstream iss(line);

    iss >> value;
}

void ChCheckpointASCII::LoadInteger(int& value) {
    CheckIfOpen();
    CheckIfComponentType();

    std::string line;
    std::getline(m_istream, line);
    std::istringstream iss(line);

    iss >> value;
}

void ChCheckpointASCII::LoadVector(std::vector<double>& vector) {
    CheckIfOpen();
    CheckIfComponentType();

    std::string line;
    std::getline(m_istream, line);
    std::istringstream iss(line);

    size_t n;
    iss >> n;
    assert(n == vector.size());
    for (size_t i = 0; i < n; i++) {
        iss >> vector[i];
    }
}

void ChCheckpointASCII::LoadChVector3(ChVector3d& vector) {
    CheckIfOpen();
    CheckIfComponentType();

    std::string line;
    std::getline(m_istream, line);
    std::istringstream iss(line);
    iss >> vector.x() >> vector.y() >> vector.z();
}

void ChCheckpointASCII::LoadChQuaternion(ChQuaterniond& quat) {
    CheckIfOpen();
    CheckIfComponentType();

    std::string line;
    std::getline(m_istream, line);
    std::istringstream iss(line);
    iss >> quat.e0() >> quat.e1() >> quat.e2() >> quat.e3();
}

}  // namespace chrono
