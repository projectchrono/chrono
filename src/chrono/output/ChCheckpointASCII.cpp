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

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkUniversal.h"
#include "chrono/physics/ChLinkDistance.h"

#include "chrono/output/ChCheckpointASCII.h"

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

void ChCheckpointASCII::WriteFile(const std::string& filename, double time) {
    std::ofstream ofile(filename);
    ofile << GetTypeAsString(m_type) << endl;
    ofile << time << endl;
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
    std::getline(m_ifile, line);
    std::istringstream iss(line);

    std::string mode;
    iss >> mode;

    if (mode != GetTypeAsString(m_type)) {
        cerr << "Error: incorrect checkpoint file type" << endl;
        throw std::runtime_error("Incorrect checkpoint file type");
    }

    cout << "Open file " << filename << " with " << mode << " checkpoint." << endl;
}

void ChCheckpointASCII::WriteState(ChSystem* sys) {
    TestTypeSystem();

    m_np = sys->GetNumCoordsPosLevel();
    m_nv = sys->GetNumCoordsVelLevel();
    ChState x(m_np, sys);
    ChStateDelta v(m_nv, sys);
    double time;
    sys->StateGather(x, v, time);
    m_csv << x << endl;
    m_csv << v << endl;
}

void ChCheckpointASCII::ReadState(ChSystem* sys) {
    TestTypeSystem();

    // Force a calculation of state counters
    sys->Setup();

    auto np = sys->GetNumCoordsPosLevel();
    auto nv = sys->GetNumCoordsVelLevel();
    ChState x(np, sys);
    ChStateDelta v(nv, sys);
    double time;

    std::string line;

    // Read time
    std::getline(m_ifile, line);
    {
        std::istringstream iss(line);
        iss >> time;
        sys->SetChTime(time);
    }

    // Read number of states
    std::getline(m_ifile, line);
    {
        std::istringstream iss(line);
        iss >> m_np >> m_nv;
        if (m_np != np || m_nv != nv) {
            cerr << "Error: inconsistent number of states" << endl;
            throw std::runtime_error("Inconsistent number of states");
        }
    }

    // Read states
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

void ChCheckpointASCII::WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
    TestTypeComponent();

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
    TestTypeComponent();

    for (const auto& shaft : shafts) {
        double pos = shaft->GetPos();
        double pos_dt = shaft->GetPosDt();

        m_csv << pos << pos_dt << endl;
    }

    m_np += shafts.size();
    m_nv += shafts.size();
}

void ChCheckpointASCII::WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) {
    TestTypeComponent();

    //// TODO - any states?
}

void ChCheckpointASCII::WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    TestTypeComponent();

    //// TODO - ChShaftsMotorSpeed
}

void ChCheckpointASCII::WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    TestTypeComponent();

    for (const auto& spring : springs) {
        if (spring->GetStates().size() > 0)
            m_csv << spring->GetStates() << endl;
    }
}

void ChCheckpointASCII::WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    TestTypeComponent();

    // No states
}

void ChCheckpointASCII::WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    TestTypeComponent();

    //// TODO - any states?
}

void ChCheckpointASCII::WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
    TestTypeComponent();

    //// TODO - ChLinkMotorLinearSpeed, ChLinkMotorLinearDriveline
}

void ChCheckpointASCII::WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) {
    TestTypeComponent();

    //// TODO - ChLinkMotorRotationSpeed, ChLinkMotorRotationDriveline
}

}  // namespace chrono
