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
// Authors: Radu Serban
// =============================================================================
//
// Base class for a vehicle output database.
//
// =============================================================================

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkUniversal.h"

#include "chrono_vehicle/output/ChVehicleOutputHDF5.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

struct body_info {
    int id;                 // body identifier
    double x, y, z;         // position
    double e0, e1, e2, e3;  // orientation
    /*
    double xd, yd, zd;      // linear velocity
    double wx, wy, wz;      // angular velocity
    double xdd, ydd, zdd;   // linear acceleration
    double wxd, wyd, wzd;   // angular acceleration
    */
};

struct bodyaux_info {
    int id;                   // body identifier
    double x, y, z;           // position
    double e0, e1, e2, e3;    // orientation
    /*
    double xd, yd, zd;        // linear velocity
    double wx, wy, wz;        // angular velocity
    double xdd, ydd, zdd;     // linear acceleration
    double wxd, wyd, wzd;     // angular acceleration
    double rx, ry, rz;        // ref frame position
    double rxd, ryd, rzd;     // ref frame linear velocity
    double rxdd, rydd, rzdd;  // ref frame linear acceleration
    */
};

struct shaft_info {
    int id;             // shaft identifier
    double x, xd, xdd;  // angle, angular velocity, angular acceleration
    double t;           // applied torque
};

struct marker_info {
    int id;                // marker identifier
    double x, y, z;        // position
    double xd, yd, zd;     // linear velocity
    double xdd, ydd, zdd;  // linear acceleration
};

struct joint_info {
    int id;             // joint identifier
    double fx, fy, fz;  // joint reaction force
    double tx, ty, tz;  // joint reaction torque
};

struct couple_info {
    int id;             // couple identifier
    double x, xd, xdd;  // relative angle, angular velocity, angular acceleration
    double t1, t2;      // reaction torque on shaft 1 and on shaft 2
};

struct linspring_info {
    int id;        // spring identifier
    double x, xd;  // length and velocity
    double f;      // reaction force
};

struct rotspring_info {
    int id;        // spring identifier
    double x, xd;  // angle and velocity
    double t;      // reaction torque
};

struct bodyload_info {
    int id;             // joint identifier
    double fx, fy, fz;  // joint reaction force
    double tx, ty, tz;  // joint reaction torque
};

H5::CompType* ChVehicleOutputHDF5::m_body_type = nullptr;
H5::CompType* ChVehicleOutputHDF5::m_bodyaux_type = nullptr;
H5::CompType* ChVehicleOutputHDF5::m_shaft_type = nullptr;
H5::CompType* ChVehicleOutputHDF5::m_marker_type = nullptr;
H5::CompType* ChVehicleOutputHDF5::m_joint_type = nullptr;
H5::CompType* ChVehicleOutputHDF5::m_couple_type = nullptr;
H5::CompType* ChVehicleOutputHDF5::m_linspring_type = nullptr;
H5::CompType* ChVehicleOutputHDF5::m_rotspring_type = nullptr;
H5::CompType* ChVehicleOutputHDF5::m_bodyload_type = nullptr;

const H5::CompType& ChVehicleOutputHDF5::getBodyType() {
    if (!m_body_type) {
        struct Initializer {
            Initializer() {
                m_body_type = new H5::CompType(sizeof(body_info));
                m_body_type->insertMember("id", HOFFSET(body_info, id), H5::PredType::NATIVE_INT);
                m_body_type->insertMember("x", HOFFSET(body_info, x), H5::PredType::NATIVE_DOUBLE);
                m_body_type->insertMember("y", HOFFSET(body_info, y), H5::PredType::NATIVE_DOUBLE);
                m_body_type->insertMember("z", HOFFSET(body_info, z), H5::PredType::NATIVE_DOUBLE);
                m_body_type->insertMember("e0", HOFFSET(body_info, e0), H5::PredType::NATIVE_DOUBLE);
                m_body_type->insertMember("e1", HOFFSET(body_info, e1), H5::PredType::NATIVE_DOUBLE);
                m_body_type->insertMember("e2", HOFFSET(body_info, e2), H5::PredType::NATIVE_DOUBLE);
                m_body_type->insertMember("e3", HOFFSET(body_info, e3), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_body_type;
}

const H5::CompType& ChVehicleOutputHDF5::getBodyAuxType() {
    if (!m_bodyaux_type) {
        struct Initializer {
            Initializer() {
                m_bodyaux_type = new H5::CompType(sizeof(bodyaux_info));
                m_bodyaux_type->insertMember("id", HOFFSET(bodyaux_info, id), H5::PredType::NATIVE_INT);
                m_bodyaux_type->insertMember("x", HOFFSET(bodyaux_info, x), H5::PredType::NATIVE_DOUBLE);
                m_bodyaux_type->insertMember("y", HOFFSET(bodyaux_info, y), H5::PredType::NATIVE_DOUBLE);
                m_bodyaux_type->insertMember("z", HOFFSET(bodyaux_info, z), H5::PredType::NATIVE_DOUBLE);
                m_bodyaux_type->insertMember("e0", HOFFSET(bodyaux_info, e0), H5::PredType::NATIVE_DOUBLE);
                m_bodyaux_type->insertMember("e1", HOFFSET(bodyaux_info, e1), H5::PredType::NATIVE_DOUBLE);
                m_bodyaux_type->insertMember("e2", HOFFSET(bodyaux_info, e2), H5::PredType::NATIVE_DOUBLE);
                m_bodyaux_type->insertMember("e3", HOFFSET(bodyaux_info, e3), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_bodyaux_type;
}

const H5::CompType& ChVehicleOutputHDF5::getShaftType() {
    if (!m_shaft_type) {
        struct Initializer {
            Initializer() {
                m_shaft_type = new H5::CompType(sizeof(shaft_info));
                m_shaft_type->insertMember("id", HOFFSET(shaft_info, id), H5::PredType::NATIVE_INT);
                m_shaft_type->insertMember("x", HOFFSET(shaft_info, x), H5::PredType::NATIVE_DOUBLE);
                m_shaft_type->insertMember("xd", HOFFSET(shaft_info, xd), H5::PredType::NATIVE_DOUBLE);
                m_shaft_type->insertMember("xdd", HOFFSET(shaft_info, xdd), H5::PredType::NATIVE_DOUBLE);
                m_shaft_type->insertMember("torque", HOFFSET(shaft_info, t), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_shaft_type;
}

const H5::CompType& ChVehicleOutputHDF5::getMarkerType() {
    if (!m_marker_type) {
        struct Initializer {
            Initializer() {
                m_marker_type = new H5::CompType(sizeof(marker_info));
                m_marker_type->insertMember("id", HOFFSET(marker_info, id), H5::PredType::NATIVE_INT);
                m_marker_type->insertMember("x", HOFFSET(marker_info, x), H5::PredType::NATIVE_DOUBLE);
                m_marker_type->insertMember("y", HOFFSET(marker_info, y), H5::PredType::NATIVE_DOUBLE);
                m_marker_type->insertMember("z", HOFFSET(marker_info, z), H5::PredType::NATIVE_DOUBLE);
                m_marker_type->insertMember("xd", HOFFSET(marker_info, xd), H5::PredType::NATIVE_DOUBLE);
                m_marker_type->insertMember("yd", HOFFSET(marker_info, yd), H5::PredType::NATIVE_DOUBLE);
                m_marker_type->insertMember("zd", HOFFSET(marker_info, zd), H5::PredType::NATIVE_DOUBLE);
                m_marker_type->insertMember("xdd", HOFFSET(marker_info, xdd), H5::PredType::NATIVE_DOUBLE);
                m_marker_type->insertMember("ydd", HOFFSET(marker_info, ydd), H5::PredType::NATIVE_DOUBLE);
                m_marker_type->insertMember("zdd", HOFFSET(marker_info, zdd), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_marker_type;
}

const H5::CompType& ChVehicleOutputHDF5::getJointType() {
    if (!m_joint_type) {
        struct Initializer {
            Initializer() {
                m_joint_type = new H5::CompType(sizeof(joint_info));
                m_joint_type->insertMember("id", HOFFSET(joint_info, id), H5::PredType::NATIVE_INT);
                m_joint_type->insertMember("Fx", HOFFSET(joint_info, fx), H5::PredType::NATIVE_DOUBLE);
                m_joint_type->insertMember("Fy", HOFFSET(joint_info, fy), H5::PredType::NATIVE_DOUBLE);
                m_joint_type->insertMember("Fz", HOFFSET(joint_info, fz), H5::PredType::NATIVE_DOUBLE);
                m_joint_type->insertMember("Tx", HOFFSET(joint_info, tx), H5::PredType::NATIVE_DOUBLE);
                m_joint_type->insertMember("Ty", HOFFSET(joint_info, ty), H5::PredType::NATIVE_DOUBLE);
                m_joint_type->insertMember("Tz", HOFFSET(joint_info, tz), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_joint_type;
}

const H5::CompType& ChVehicleOutputHDF5::getCoupleType() {
    if (!m_couple_type) {
        struct Initializer {
            Initializer() {
                m_couple_type = new H5::CompType(sizeof(couple_info));
                m_couple_type->insertMember("id", HOFFSET(couple_info, id), H5::PredType::NATIVE_INT);
                m_couple_type->insertMember("x", HOFFSET(couple_info, x), H5::PredType::NATIVE_DOUBLE);
                m_couple_type->insertMember("xd", HOFFSET(couple_info, xd), H5::PredType::NATIVE_DOUBLE);
                m_couple_type->insertMember("xdd", HOFFSET(couple_info, xdd), H5::PredType::NATIVE_DOUBLE);
                m_couple_type->insertMember("torque1", HOFFSET(couple_info, t1), H5::PredType::NATIVE_DOUBLE);
                m_couple_type->insertMember("torque2", HOFFSET(couple_info, t1), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_couple_type;
}

const H5::CompType& ChVehicleOutputHDF5::getLinSpringType() {
    if (!m_linspring_type) {
        struct Initializer {
            Initializer() {
                m_linspring_type = new H5::CompType(sizeof(linspring_info));
                m_linspring_type->insertMember("id", HOFFSET(linspring_info, id), H5::PredType::NATIVE_INT);
                m_linspring_type->insertMember("x", HOFFSET(linspring_info, x), H5::PredType::NATIVE_DOUBLE);
                m_linspring_type->insertMember("xd", HOFFSET(linspring_info, xd), H5::PredType::NATIVE_DOUBLE);
                m_linspring_type->insertMember("force", HOFFSET(linspring_info, f), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_linspring_type;
}

const H5::CompType& ChVehicleOutputHDF5::getRotSpringType() {
    if (!m_rotspring_type) {
        struct Initializer {
            Initializer() {
                m_rotspring_type = new H5::CompType(sizeof(rotspring_info));
                m_rotspring_type->insertMember("id", HOFFSET(rotspring_info, id), H5::PredType::NATIVE_INT);
                m_rotspring_type->insertMember("x", HOFFSET(rotspring_info, x), H5::PredType::NATIVE_DOUBLE);
                m_rotspring_type->insertMember("xd", HOFFSET(rotspring_info, xd), H5::PredType::NATIVE_DOUBLE);
                m_rotspring_type->insertMember("force", HOFFSET(rotspring_info, t), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_rotspring_type;
}

const H5::CompType& ChVehicleOutputHDF5::getBodyLoadType() {
    if (!m_bodyload_type) {
        struct Initializer {
            Initializer() {
                m_bodyload_type = new H5::CompType(sizeof(bodyload_info));
                m_bodyload_type->insertMember("id", HOFFSET(bodyload_info, id), H5::PredType::NATIVE_INT);
                m_bodyload_type->insertMember("Fx", HOFFSET(bodyload_info, fx), H5::PredType::NATIVE_DOUBLE);
                m_bodyload_type->insertMember("Fy", HOFFSET(bodyload_info, fy), H5::PredType::NATIVE_DOUBLE);
                m_bodyload_type->insertMember("Fz", HOFFSET(bodyload_info, fz), H5::PredType::NATIVE_DOUBLE);
                m_bodyload_type->insertMember("Tx", HOFFSET(bodyload_info, tx), H5::PredType::NATIVE_DOUBLE);
                m_bodyload_type->insertMember("Ty", HOFFSET(bodyload_info, ty), H5::PredType::NATIVE_DOUBLE);
                m_bodyload_type->insertMember("Tz", HOFFSET(bodyload_info, tz), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_bodyload_type;
}

// -----------------------------------------------------------------------------

ChVehicleOutputHDF5::ChVehicleOutputHDF5(const std::string& filename)
    : m_frame_group(nullptr), m_section_group(nullptr) {
    m_fileHDF5 = new H5::H5File(filename, H5F_ACC_TRUNC);
    H5::Group frames_group(m_fileHDF5->createGroup("/Frames"));
}

ChVehicleOutputHDF5::~ChVehicleOutputHDF5() {
    if (m_section_group)
        m_section_group->close();
    if (m_frame_group)
        m_frame_group->close();
    m_fileHDF5->close();

    delete m_section_group;
    delete m_frame_group;
    delete m_fileHDF5;
    
    delete m_body_type;
    delete m_bodyaux_type;
    delete m_shaft_type;
    delete m_marker_type;
    delete m_joint_type;
    delete m_couple_type;
    delete m_linspring_type;
    delete m_rotspring_type;
}

// -----------------------------------------------------------------------------

std::string format_number(int num, int precision) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(precision) << num;
    return out.str();
}

// -----------------------------------------------------------------------------

void ChVehicleOutputHDF5::WriteTime(int frame, double time) {
    // Close the currently open section group
    if (m_section_group) {
        m_section_group->close();
        delete m_section_group;
        m_section_group = nullptr;
    }

    // Close the currently open frame group
    if (m_frame_group) {
        m_frame_group->close();
        delete m_frame_group;
        m_frame_group = nullptr;
    }

    // Open the frames group and create the group for this frame
    auto frame_name = std::string("Frame_") + format_number(frame, 6);
    H5::Group frames_group = m_fileHDF5->openGroup("/Frames");
    m_frame_group = new H5::Group(frames_group.createGroup(frame_name));

    // Create an attribute with timestamp
    {
        H5::DataSpace dataspace(H5S_SCALAR);
        H5::Attribute att = m_frame_group->createAttribute("Timestamp", H5::PredType::NATIVE_DOUBLE, dataspace);
        att.write(H5::PredType::NATIVE_DOUBLE, &time);
    }
}

void ChVehicleOutputHDF5::WriteSection(const std::string& name) {
    // Close the currently open section group
    if (m_section_group) {
        m_section_group->close();
        delete m_section_group;
        m_section_group = nullptr;
    }

    // Create the group for this section in the current frame group
    m_section_group = new H5::Group(m_frame_group->createGroup(name));
}

void ChVehicleOutputHDF5::WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
    if (bodies.empty())
        return;

    auto nbodies = bodies.size();
    hsize_t dim[] = {nbodies};
    H5::DataSpace dataspace(1, dim);
    std::vector<body_info> info(nbodies);
    for (auto i = 0; i < nbodies; i++) {
        const ChVector<>& p = bodies[i]->GetPos();
        const ChQuaternion<>& q = bodies[i]->GetRot();
        info[i] = {bodies[i]->GetIdentifier(), p.x(), p.y(), p.z(), q.e0(), q.e1(), q.e2(), q.e3()};
    }

    H5::DataSet set = m_section_group->createDataSet("Bodies", getBodyType(), dataspace);
    set.write(info.data(), getBodyType());
}

void ChVehicleOutputHDF5::WriteAuxRefBodies(const std::vector<std::shared_ptr<ChBodyAuxRef>>& bodies) {
    if (bodies.empty())
        return;

    auto nbodies = bodies.size();
    hsize_t dim[] = { nbodies };
    H5::DataSpace dataspace(1, dim);
    std::vector<bodyaux_info> info(nbodies);
    for (auto i = 0; i < nbodies; i++) {
        const ChVector<>& p = bodies[i]->GetPos();
        const ChQuaternion<>& q = bodies[i]->GetRot();
        info[i] = { bodies[i]->GetIdentifier(), p.x(), p.y(), p.z(), q.e0(), q.e1(), q.e2(), q.e3() };
    }

    H5::DataSet set = m_section_group->createDataSet("Bodies AuxRef", getBodyAuxType(), dataspace);
    set.write(info.data(), getBodyAuxType());
}

void ChVehicleOutputHDF5::WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) {
    if (markers.empty())
        return;

    auto nmarkers = markers.size();
    hsize_t dim[] = {nmarkers};
    H5::DataSpace dataspace(1, dim);
    std::vector<marker_info> info(nmarkers);
    for (auto i = 0; i < nmarkers; i++) {
        const ChVector<>& p = markers[i]->GetAbsCoord().pos;
        const ChVector<>& pd = markers[i]->GetAbsCoord_dt().pos;
        const ChVector<>& pdd = markers[i]->GetAbsCoord_dtdt().pos;
        info[i] = {markers[i]->GetIdentifier(), p.x(), p.y(), p.z(), pd.x(), pd.y(), pd.z(), pdd.x(), pdd.y(), pdd.z()};
    }

    H5::DataSet set = m_section_group->createDataSet("Markers", getMarkerType(), dataspace);
    set.write(info.data(), getMarkerType());
}

void ChVehicleOutputHDF5::WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) {
    if (shafts.empty())
        return;

    auto nshafts = shafts.size();
    hsize_t dim[] = {nshafts};
    H5::DataSpace dataspace(1, dim);
    std::vector<shaft_info> info(nshafts);
    for (auto i = 0; i < nshafts; i++) {
        info[i] = {shafts[i]->GetIdentifier(), shafts[i]->GetPos(), shafts[i]->GetPos_dt(), shafts[i]->GetPos_dtdt(),
                   shafts[i]->GetAppliedTorque()};
    }

    H5::DataSet set = m_section_group->createDataSet("Shafts", getShaftType(), dataspace);
    set.write(info.data(), getShaftType());
}

void ChVehicleOutputHDF5::WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) {
    if (joints.empty())
        return;

    auto njoints = joints.size();
    hsize_t dim[] = { njoints };
    H5::DataSpace dataspace(1, dim);
    std::vector<joint_info> info(njoints);
    for (auto i = 0; i < njoints; i++) {
        const ChVector<>& f = joints[i]->Get_react_force();
        const ChVector<>& t = joints[i]->Get_react_torque();
        info[i] = { joints[i]->GetIdentifier(), f.x(), f.y(), f.z(), t.x(), t.y(), t.z() };
    }

    H5::DataSet set = m_section_group->createDataSet("Joints", getJointType(), dataspace);
    set.write(info.data(), getJointType());
}

void ChVehicleOutputHDF5::WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    if (couples.empty())
        return;

    auto ncouples = couples.size();
    hsize_t dim[] = {ncouples};
    H5::DataSpace dataspace(1, dim);
    std::vector<couple_info> info(ncouples);
    for (auto i = 0; i < ncouples; i++) {
        info[i] = {couples[i]->GetIdentifier(),          couples[i]->GetRelativeRotation(),
                   couples[i]->GetRelativeRotation_dt(), couples[i]->GetRelativeRotation_dtdt(),
                   couples[i]->GetTorqueReactionOn1(),   couples[i]->GetTorqueReactionOn2()};
    }

    H5::DataSet set = m_section_group->createDataSet("Couples", getCoupleType(), dataspace);
    set.write(info.data(), getCoupleType());
}

void ChVehicleOutputHDF5::WriteLinSprings(const std::vector<std::shared_ptr<ChLinkSpringCB>>& springs) {
    if (springs.empty())
        return;

    auto nsprings = springs.size();
    hsize_t dim[] = {nsprings};
    H5::DataSpace dataspace(1, dim);
    std::vector<linspring_info> info(nsprings);
    for (auto i = 0; i < nsprings; i++) {
        info[i] = {springs[i]->GetIdentifier(), springs[i]->GetSpringLength(), springs[i]->GetSpringVelocity(),
                   springs[i]->GetSpringReact()};
    }

    H5::DataSet set = m_section_group->createDataSet("Lin Springs", getLinSpringType(), dataspace);
    set.write(info.data(), getLinSpringType());
}

void ChVehicleOutputHDF5::WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRotSpringCB>>& springs) {
    if (springs.empty())
        return;

    auto nsprings = springs.size();
    hsize_t dim[] = {nsprings};
    H5::DataSpace dataspace(1, dim);
    std::vector<rotspring_info> info(nsprings);
    for (auto i = 0; i < nsprings; i++) {
        info[i] = {springs[i]->GetIdentifier(), springs[i]->GetRotSpringAngle(), springs[i]->GetRotSpringSpeed(),
                   springs[i]->GetRotSpringTorque()};
    }

    H5::DataSet set = m_section_group->createDataSet("Rot Springs", getRotSpringType(), dataspace);
    set.write(info.data(), getRotSpringType());
}

void ChVehicleOutputHDF5::WriteBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    if (loads.empty())
        return;

    auto nloads = loads.size();
    hsize_t dim[] = { nloads };
    H5::DataSpace dataspace(1, dim);
    std::vector<bodyload_info> info(nloads);
    for (auto i = 0; i < nloads; i++) {
        ChVector<> f = loads[i]->GetForce();
        ChVector<> t = loads[i]->GetTorque();
        info[i] = { loads[i]->GetIdentifier(), f.x(), f.y(), f.z(), t.x(), t.y(), t.z() };
    }

    H5::DataSet set = m_section_group->createDataSet("Body-body Loads", getBodyLoadType(), dataspace);
    set.write(info.data(), getBodyLoadType());
}

}  // end namespace vehicle
}  // end namespace chrono
