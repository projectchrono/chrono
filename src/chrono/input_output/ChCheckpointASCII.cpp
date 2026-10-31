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
// Implementation of an ASCII text Chrono checkpoint database.
//
// =============================================================================

#include <iomanip>
#include <iostream>
#include <limits>
#include <set>

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

// A speed motor (ChLinkMotorRotationSpeed, ChLinkMotorLinearSpeed, ChShaftsMotorSpeed) integrates the imposed speed into
// an internal state (an angle or a position), which it stores in the velocity-level state vector. Access it through
// the motor's state gather/scatter functions (the solver variable state only holds the result of the last solve).
static double GetSpeedMotorState(ChPhysicsItem& motor) {
    ChState x(motor.GetNumCoordsPosLevel(), nullptr);
    ChStateDelta v(motor.GetNumCoordsVelLevel(), nullptr);
    double time;
    motor.IntStateGather(0, x, 0, v, time);
    return v(0);
}

static void SetSpeedMotorState(ChPhysicsItem& motor, double state, double time) {
    ChState x(motor.GetNumCoordsPosLevel(), nullptr);
    ChStateDelta v(motor.GetNumCoordsVelLevel(), nullptr);
    double t;
    motor.IntStateGather(0, x, 0, v, t);
    v(0) = state;
    motor.IntStateScatter(0, x, 0, v, time, UpdateFlags::UPDATE_ALL);
}

ChCheckpointASCII::ChCheckpointASCII(Type type) : ChCheckpoint(type), m_file_read(false), m_np(0), m_nv(0) {
    m_csv.SetDelimiter(" ");

    // Write enough digits for doubles to be read back exactly
    m_csv.Stream() << std::setprecision(std::numeric_limits<double>::max_digits10);
}

ChCheckpointASCII::~ChCheckpointASCII() {}

void ChCheckpointASCII::Initialize() {}

// -----------------------------------------------------------------------------

void ChCheckpointASCII::WriteFile(const std::string& filename) {
    std::ofstream ofile(filename);
    ofile << GetTypeAsString(m_type) << endl;
    ofile << std::setprecision(std::numeric_limits<double>::max_digits10) << m_time << endl;
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

void ChCheckpointASCII::WriteBlock(const ChCheckpointWriter& block) {
    static const char* hex = "0123456789abcdef";

    auto& os = m_csv.Stream();
    const auto& entries = block.GetEntries();
    os << entries.size();
    for (const auto& entry : entries) {
        os << ' ';
        if (entry.is_bytes) {
            os << 'x';
            for (auto b : entry.bytes)
                os << hex[b >> 4] << hex[b & 0xf];
        } else {
            os << entry.value;
        }
    }
    os << '\n';
}

ChCheckpointReader ChCheckpointASCII::ParseBlock(const std::string& line) {
    std::istringstream iss(line);
    size_t n = 0;
    iss >> n;

    std::vector<ChCheckpointEntry> entries(n);
    std::string token;
    for (auto& entry : entries) {
        if (!(iss >> token)) {
            cerr << "Error: truncated checkpoint data block" << endl;
            throw std::runtime_error("Truncated checkpoint data block");
        }
        if (token[0] == 'x') {
            entry.is_bytes = true;
            entry.bytes.resize((token.size() - 1) / 2);
            for (size_t i = 0; i < entry.bytes.size(); i++)
                entry.bytes[i] = static_cast<std::uint8_t>(std::stoi(token.substr(1 + 2 * i, 2), nullptr, 16));
        } else {
            entry.value = std::stod(token);
        }
    }

    return ChCheckpointReader(std::move(entries));
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

    // Make sure state counters are current (e.g., for a system that was not yet simulated)
    sys->Setup();

    m_np = sys->GetNumCoordsPosLevel();
    m_nv = sys->GetNumCoordsVelLevel();
    ChState x(m_np, sys);
    ChStateDelta v(m_nv, sys);
    double time;
    sys->StateGather(x, v, time);
    assert(time == m_time);
    m_csv << x << endl;
    m_csv << v << endl;

    // Accelerations (used by some items, e.g., for inertial forces) and internal state of all physics items (data not
    // in the state vectors), in one block
    if (!sys->SupportsCheckpoint()) {
        cerr << "Error: the system contains physics items whose internal state cannot be checkpointed" << endl;
        throw std::runtime_error("System internal state cannot be checkpointed");
    }
    ChStateDelta a(m_nv, sys);
    sys->StateGatherAcceleration(a);
    ChCheckpointWriter internal;
    sys->SaveInternalState(internal);

    // Lagrange multipliers of the assembly constraints (contact multipliers are not saved, since contacts are
    // recreated at each step), used by some integrators (e.g., HHT) as part of their history
    ChVectorDynamic<> L(sys->GetNumConstraints());
    sys->StateGatherReactions(L);
    auto nc = sys->GetNumConstraints() - sys->GetContactContainer()->GetNumConstraints();

    // Timestepper internal state, tagged with the timestepper type
    ChCheckpointWriter timestepper;
    auto ts = sys->GetTimestepper();
    if (ts)
        ts->SaveInternalState(timestepper);

    ChCheckpointWriter writer;
    writer.Write(a);
    writer.Write(internal);
    writer.Write(L.head(nc));
    writer.Write(ts ? static_cast<int>(ts->GetType()) : -1);
    writer.Write(timestepper);
    WriteBlock(writer);
}

void ChCheckpointASCII::LoadState(ChSystem* sys) {
    CheckIfOpen();
    CheckIfSystemType();

    // Force a calculation of state counters.
    // This also initializes any items not yet initialized, so that they record their reference configuration before
    // the saved state is loaded.
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

    // Load accelerations and the internal state of all physics items, if present (older checkpoint files do not
    // include them)
    // Lagrange multipliers and timestepper state follow, if present (older checkpoint files do not include them)
    bool has_acc = false;
    bool has_reactions = false;
    ChStateDelta a(nv, sys);
    ChVectorDynamic<> L_saved;
    ChCheckpointReader internal;
    if (std::getline(m_istream, line) && !line.empty()) {
        auto reader = ParseBlock(line);
        reader.Read(a);
        reader.Read(internal);
        if (a.size() != nv) {
            cerr << "Error: inconsistent number of accelerations" << endl;
            throw std::runtime_error("Inconsistent number of accelerations");
        }
        has_acc = true;

        if (!reader.IsAtEnd()) {
            int ts_type;
            ChCheckpointReader timestepper;
            reader.Read(L_saved);
            reader.Read(ts_type);
            reader.Read(timestepper);
            reader.CheckAtEnd();

            auto nc = sys->GetNumConstraints() - sys->GetContactContainer()->GetNumConstraints();
            if (L_saved.size() != nc) {
                cerr << "Error: inconsistent number of constraints" << endl;
                throw std::runtime_error("Inconsistent number of constraints");
            }
            has_reactions = true;

            // Restore the timestepper state only for the same type of timestepper
            auto ts = sys->GetTimestepper();
            if (ts && static_cast<int>(ts->GetType()) == ts_type) {
                ts->LoadInternalState(timestepper);
                timestepper.CheckAtEnd();
            }
        }
    }

    sys->StateScatter(x, v, m_time, UpdateFlags::UPDATE_ALL);
    if (has_acc)
        sys->StateScatterAcceleration(a);
    if (has_reactions) {
        // Keep the current contact multipliers, if any; contacts are recreated at the next step
        ChVectorDynamic<> L(sys->GetNumConstraints());
        sys->StateGatherReactions(L);
        L.head(L_saved.size()) = L_saved;
        sys->StateScatterReactions(L);
    }

    // Internal state last, since it may refine data set by the scatter (e.g., quaternion derivatives of bodies); then
    // update the system with the complete state
    if (has_acc) {
        sys->LoadInternalState(internal);
        internal.CheckAtEnd();
        sys->Update(UpdateFlags::UPDATE_ALL);
    }
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
            m_csv << GetSpeedMotorState(*motor_speed) << endl;
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

    // No states, but an internal state (the number of full turns)
    for (const auto& spring : springs) {
        ChCheckpointWriter internal;
        spring->SaveInternalState(internal);
        WriteBlock(internal);
    }
}

void ChCheckpointASCII::SaveBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    CheckIfComponentType();

    // No states
}

void ChCheckpointASCII::SaveLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
    CheckIfComponentType();

    for (const auto& motor : motors) {
        if (auto motor_speed = std::dynamic_pointer_cast<ChLinkMotorLinearSpeed>(motor)) {
            m_csv << GetSpeedMotorState(*motor_speed) << endl;
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
            m_csv << GetSpeedMotorState(*motor_speed) << endl;
            m_np += 1;
        } else if (auto motor_drvl = std::dynamic_pointer_cast<ChLinkMotorRotationDriveline>(motor)) {
            auto s1 = motor_drvl->GetInnerShaft1();
            auto s2 = motor_drvl->GetInnerShaft2();
            m_csv << s1->GetPos() << s2->GetPos()               //
                  << s1->GetPosDt() << s2->GetPosDt() << endl;  //
            m_np += 2;
            m_nv += 2;
        }

        // Internal state (the multi-turn rotation angle)
        ChCheckpointWriter internal;
        motor->SaveInternalState(internal);
        WriteBlock(internal);
    }
}

void ChCheckpointASCII::SaveItems(const std::vector<std::shared_ptr<ChPhysicsItem>>& items) {
    CheckIfComponentType();

    for (const auto& item : items) {
        if (!item->SupportsCheckpoint()) {
            cerr << "Error: the internal state of physics item '" << item->GetName() << "' cannot be checkpointed" << endl;
            throw std::runtime_error("Physics item internal state cannot be checkpointed");
        }

        // Coordinates, accelerations and reactions, gathered with local offsets
        auto np = item->GetNumCoordsPosLevel();
        auto nv = item->GetNumCoordsVelLevel();
        auto nc = item->GetNumConstraints();
        ChState x(np, nullptr);
        ChStateDelta v(nv, nullptr);
        ChStateDelta a(nv, nullptr);
        ChVectorDynamic<> L(nc);
        double time;
        item->IntStateGather(0, x, 0, v, time);
        item->IntStateGatherAcceleration(0, a);
        item->IntStateGatherReactions(0, L);

        ChCheckpointWriter internal;
        item->SaveInternalState(internal);

        ChCheckpointWriter writer;
        writer.Write(x);
        writer.Write(v);
        writer.Write(a);
        writer.Write(L);
        writer.Write(internal);
        WriteBlock(writer);

        m_np += np;
        m_nv += nv;
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
            SetSpeedMotorState(*motor_speed, state, m_time);
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

    // No states, but an internal state (the number of full turns)
    std::string line;
    for (auto& spring : springs) {
        if (!std::getline(m_istream, line)) {
            cerr << "Error: missing checkpoint data for RSDA '" << spring->GetName() << "'" << endl;
            throw std::runtime_error("Missing checkpoint data for RSDA");
        }
        auto internal = ParseBlock(line);
        spring->LoadInternalState(internal);
        internal.CheckAtEnd();
    }
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
            SetSpeedMotorState(*motor_speed, state, m_time);
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
            SetSpeedMotorState(*motor_speed, state, m_time);
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

        // Internal state (the multi-turn rotation angle), loaded last since setting the motor state updates the motor
        if (!std::getline(m_istream, line)) {
            cerr << "Error: missing checkpoint data for rotation motor '" << motor->GetName() << "'" << endl;
            throw std::runtime_error("Missing checkpoint data for rotation motor");
        }
        auto internal = ParseBlock(line);
        motor->LoadInternalState(internal);
        internal.CheckAtEnd();
    }
}

void ChCheckpointASCII::LoadItems(std::vector<std::shared_ptr<ChPhysicsItem>>& items) {
    CheckIfOpen();
    CheckIfComponentType();

    // Set up the containing systems. This also initializes items not yet initialized, so that they record their
    // reference configuration before the saved state is loaded.
    std::set<ChSystem*> systems;
    for (const auto& item : items) {
        if (item->GetSystem())
            systems.insert(item->GetSystem());
    }
    for (auto sys : systems)
        sys->Setup();

    std::string line;
    for (const auto& item : items) {
        if (!std::getline(m_istream, line)) {
            cerr << "Error: missing checkpoint data for physics item '" << item->GetName() << "'" << endl;
            throw std::runtime_error("Missing checkpoint data for physics item");
        }
        auto reader = ParseBlock(line);

        ChState x(1, nullptr);
        ChStateDelta v(1, nullptr);
        ChStateDelta a(1, nullptr);
        ChVectorDynamic<> L;
        ChCheckpointReader internal;
        reader.Read(x);
        reader.Read(v);
        reader.Read(a);
        reader.Read(L);
        reader.Read(internal);
        reader.CheckAtEnd();

        if (x.size() != item->GetNumCoordsPosLevel() || v.size() != item->GetNumCoordsVelLevel() || a.size() != v.size() || L.size() != item->GetNumConstraints()) {
            cerr << "Error: inconsistent number of states or constraints for physics item '" << item->GetName() << "'" << endl;
            throw std::runtime_error("Inconsistent number of states or constraints");
        }

        item->IntStateScatter(0, x, 0, v, m_time, UpdateFlags::UPDATE_ALL);
        item->IntStateScatterAcceleration(0, a);
        item->IntStateScatterReactions(0, L);

        // Internal state last, since it may refine data set by the scatter; then update the item
        item->LoadInternalState(internal);
        internal.CheckAtEnd();
        item->Update(m_time, UpdateFlags::UPDATE_ALL);
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
