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
// Implementation of an HDF5 Chrono output database.
//
// =============================================================================

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkUniversal.h"

#include "chrono/input_output/ChOutputHDF5.h"

namespace chrono {

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

struct linmotor_info {
    int id;        // motor identifier
    double x, xd;  // motor position and speed
    double f;      // motor force
};

struct rotmotor_info {
    int id;        // motor identifier
    double x, xd;  // motor angle and angular speed
    double t;      // motor torque
};

// -----------------------------------------------------------------------------

ChOutputHDF5::ChOutputHDF5(const std::string& out_dir, const std::string& out_file_stem, Mode mode) : ChOutput(mode), m_frame_group(nullptr) {
    auto filename = out_dir + "/" + out_file_stem + "." + GetModeAsString(mode) + ".h5";
    m_fileHDF5 = new H5::H5File(filename, H5F_ACC_TRUNC);

    switch (m_mode) {
        case Mode::FRAMES: {
            // Initialize the compound data types
            m_body_type = new H5::CompType(sizeof(body_info));
            m_body_type->insertMember("id", HOFFSET(body_info, id), H5::PredType::NATIVE_INT);
            m_body_type->insertMember("x", HOFFSET(body_info, x), H5::PredType::NATIVE_DOUBLE);
            m_body_type->insertMember("y", HOFFSET(body_info, y), H5::PredType::NATIVE_DOUBLE);
            m_body_type->insertMember("z", HOFFSET(body_info, z), H5::PredType::NATIVE_DOUBLE);
            m_body_type->insertMember("e0", HOFFSET(body_info, e0), H5::PredType::NATIVE_DOUBLE);
            m_body_type->insertMember("e1", HOFFSET(body_info, e1), H5::PredType::NATIVE_DOUBLE);
            m_body_type->insertMember("e2", HOFFSET(body_info, e2), H5::PredType::NATIVE_DOUBLE);
            m_body_type->insertMember("e3", HOFFSET(body_info, e3), H5::PredType::NATIVE_DOUBLE);

            m_joint_type = new H5::CompType(sizeof(joint_info));
            m_joint_type->insertMember("id", HOFFSET(joint_info, id), H5::PredType::NATIVE_INT);
            m_joint_type->insertMember("Fx", HOFFSET(joint_info, fx), H5::PredType::NATIVE_DOUBLE);
            m_joint_type->insertMember("Fy", HOFFSET(joint_info, fy), H5::PredType::NATIVE_DOUBLE);
            m_joint_type->insertMember("Fz", HOFFSET(joint_info, fz), H5::PredType::NATIVE_DOUBLE);
            m_joint_type->insertMember("Tx", HOFFSET(joint_info, tx), H5::PredType::NATIVE_DOUBLE);
            m_joint_type->insertMember("Ty", HOFFSET(joint_info, ty), H5::PredType::NATIVE_DOUBLE);
            m_joint_type->insertMember("Tz", HOFFSET(joint_info, tz), H5::PredType::NATIVE_DOUBLE);

            m_shaft_type = new H5::CompType(sizeof(shaft_info));
            m_shaft_type->insertMember("id", HOFFSET(shaft_info, id), H5::PredType::NATIVE_INT);
            m_shaft_type->insertMember("x", HOFFSET(shaft_info, x), H5::PredType::NATIVE_DOUBLE);
            m_shaft_type->insertMember("xd", HOFFSET(shaft_info, xd), H5::PredType::NATIVE_DOUBLE);
            m_shaft_type->insertMember("xdd", HOFFSET(shaft_info, xdd), H5::PredType::NATIVE_DOUBLE);
            m_shaft_type->insertMember("torque", HOFFSET(shaft_info, t), H5::PredType::NATIVE_DOUBLE);

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

            m_couple_type = new H5::CompType(sizeof(couple_info));
            m_couple_type->insertMember("id", HOFFSET(couple_info, id), H5::PredType::NATIVE_INT);
            m_couple_type->insertMember("x", HOFFSET(couple_info, x), H5::PredType::NATIVE_DOUBLE);
            m_couple_type->insertMember("xd", HOFFSET(couple_info, xd), H5::PredType::NATIVE_DOUBLE);
            m_couple_type->insertMember("xdd", HOFFSET(couple_info, xdd), H5::PredType::NATIVE_DOUBLE);
            m_couple_type->insertMember("torque1", HOFFSET(couple_info, t1), H5::PredType::NATIVE_DOUBLE);
            m_couple_type->insertMember("torque2", HOFFSET(couple_info, t2), H5::PredType::NATIVE_DOUBLE);

            m_linspring_type = new H5::CompType(sizeof(linspring_info));
            m_linspring_type->insertMember("id", HOFFSET(linspring_info, id), H5::PredType::NATIVE_INT);
            m_linspring_type->insertMember("x", HOFFSET(linspring_info, x), H5::PredType::NATIVE_DOUBLE);
            m_linspring_type->insertMember("xd", HOFFSET(linspring_info, xd), H5::PredType::NATIVE_DOUBLE);
            m_linspring_type->insertMember("force", HOFFSET(linspring_info, f), H5::PredType::NATIVE_DOUBLE);

            m_rotspring_type = new H5::CompType(sizeof(rotspring_info));
            m_rotspring_type->insertMember("id", HOFFSET(rotspring_info, id), H5::PredType::NATIVE_INT);
            m_rotspring_type->insertMember("x", HOFFSET(rotspring_info, x), H5::PredType::NATIVE_DOUBLE);
            m_rotspring_type->insertMember("xd", HOFFSET(rotspring_info, xd), H5::PredType::NATIVE_DOUBLE);
            m_rotspring_type->insertMember("torque", HOFFSET(rotspring_info, t), H5::PredType::NATIVE_DOUBLE);

            m_bodyload_type = new H5::CompType(sizeof(bodyload_info));
            m_bodyload_type->insertMember("id", HOFFSET(bodyload_info, id), H5::PredType::NATIVE_INT);
            m_bodyload_type->insertMember("Fx", HOFFSET(bodyload_info, fx), H5::PredType::NATIVE_DOUBLE);
            m_bodyload_type->insertMember("Fy", HOFFSET(bodyload_info, fy), H5::PredType::NATIVE_DOUBLE);
            m_bodyload_type->insertMember("Fz", HOFFSET(bodyload_info, fz), H5::PredType::NATIVE_DOUBLE);
            m_bodyload_type->insertMember("Tx", HOFFSET(bodyload_info, tx), H5::PredType::NATIVE_DOUBLE);
            m_bodyload_type->insertMember("Ty", HOFFSET(bodyload_info, ty), H5::PredType::NATIVE_DOUBLE);
            m_bodyload_type->insertMember("Tz", HOFFSET(bodyload_info, tz), H5::PredType::NATIVE_DOUBLE);

            m_linmotor_type = new H5::CompType(sizeof(linmotor_info));
            m_linmotor_type->insertMember("id", HOFFSET(linmotor_info, id), H5::PredType::NATIVE_INT);
            m_linmotor_type->insertMember("x", HOFFSET(linmotor_info, x), H5::PredType::NATIVE_DOUBLE);
            m_linmotor_type->insertMember("xd", HOFFSET(linmotor_info, xd), H5::PredType::NATIVE_DOUBLE);
            m_linmotor_type->insertMember("force", HOFFSET(linmotor_info, f), H5::PredType::NATIVE_DOUBLE);

            m_rotmotor_type = new H5::CompType(sizeof(rotmotor_info));
            m_rotmotor_type->insertMember("id", HOFFSET(rotmotor_info, id), H5::PredType::NATIVE_INT);
            m_rotmotor_type->insertMember("x", HOFFSET(rotmotor_info, x), H5::PredType::NATIVE_DOUBLE);
            m_rotmotor_type->insertMember("xd", HOFFSET(rotmotor_info, xd), H5::PredType::NATIVE_DOUBLE);
            m_rotmotor_type->insertMember("torque", HOFFSET(rotmotor_info, t), H5::PredType::NATIVE_DOUBLE);

            H5::Group frames_group(m_fileHDF5->createGroup("/Frames"));

            break;
        }

        case Mode::SERIES: {
            H5::Group time_group(m_fileHDF5->createGroup("/Time"));
            H5::Group body_group(m_fileHDF5->createGroup("/Bodies"));
            H5::Group shaft_group(m_fileHDF5->createGroup("/Shafts"));
            H5::Group joint_group(m_fileHDF5->createGroup("/Joints"));
            H5::Group tsda_group(m_fileHDF5->createGroup("/TSDAs"));
            H5::Group rsda_group(m_fileHDF5->createGroup("/RSDAs"));

            break;
        }
    }
}

ChOutputHDF5::~ChOutputHDF5() {
    switch (m_mode) {
        case Mode::FRAMES:
            if (m_frame_group) {
                m_frame_group->close();
                delete m_frame_group;
            }

            delete m_body_type;
            delete m_shaft_type;
            delete m_marker_type;
            delete m_joint_type;
            delete m_couple_type;
            delete m_linspring_type;
            delete m_rotspring_type;
            delete m_bodyload_type;
            delete m_linmotor_type;
            delete m_rotmotor_type;

            break;

        case Mode::SERIES:
            WriteBuffers();

            break;
    }

    m_fileHDF5->close();
    delete m_fileHDF5;
}

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

void ChOutputHDF5::WriteBuffers() {
    hsize_t N = m_time.size();
    std::array<hsize_t, 2> dims_N3 = {N, static_cast<hsize_t>(3)};
    H5::DataSpace data_space_N(1, &N);
    H5::DataSpace data_space_N3(2, dims_N3.data());

    {
        H5::Group tgroup = m_fileHDF5->openGroup("/Time");
        H5::DataSet tset = tgroup.createDataSet("time", H5::PredType::NATIVE_DOUBLE, data_space_N);
        tset.write(m_time.data(), H5::PredType::NATIVE_DOUBLE);
    }

    {
        H5::Group bgroup = m_fileHDF5->openGroup("/Bodies");
        for (const auto& buf : m_body_buf) {
            H5::Group group = bgroup.createGroup(buf.name.c_str());
            H5::DataSet pset = group.createDataSet("pos", H5::PredType::NATIVE_DOUBLE, data_space_N3);
            H5::DataSet rset = group.createDataSet("rot", H5::PredType::NATIVE_DOUBLE, data_space_N3);
            H5::DataSet vset = group.createDataSet("lin_vel", H5::PredType::NATIVE_DOUBLE, data_space_N3);
            H5::DataSet wset = group.createDataSet("ang_vel", H5::PredType::NATIVE_DOUBLE, data_space_N3);
            pset.write(buf.pos.data(), H5::PredType::NATIVE_DOUBLE);
            rset.write(buf.rot.data(), H5::PredType::NATIVE_DOUBLE);
            vset.write(buf.lin_vel.data(), H5::PredType::NATIVE_DOUBLE);
            wset.write(buf.ang_vel.data(), H5::PredType::NATIVE_DOUBLE);
        }
    }

    {
        H5::Group sgroup = m_fileHDF5->openGroup("/Shafts");
        for (const auto& buf : m_shaft_buf) {
            H5::Group group = sgroup.createGroup(buf.name.c_str());
            H5::DataSet pset = group.createDataSet("pos", H5::PredType::NATIVE_DOUBLE, data_space_N);
            H5::DataSet vset = group.createDataSet("vel", H5::PredType::NATIVE_DOUBLE, data_space_N);
            pset.write(buf.pos.data(), H5::PredType::NATIVE_DOUBLE);
            vset.write(buf.vel.data(), H5::PredType::NATIVE_DOUBLE);
        }
    }

    {
        H5::Group jgroup = m_fileHDF5->openGroup("/Joints");
        for (const auto& buf : m_joint_buf) {
            H5::Group group = jgroup.createGroup(buf.name.c_str());
            H5::DataSet f1set = group.createDataSet("force1", H5::PredType::NATIVE_DOUBLE, data_space_N3);
            H5::DataSet t1set = group.createDataSet("torque1", H5::PredType::NATIVE_DOUBLE, data_space_N3);
            H5::DataSet f2set = group.createDataSet("force2", H5::PredType::NATIVE_DOUBLE, data_space_N3);
            H5::DataSet t2set = group.createDataSet("torque2", H5::PredType::NATIVE_DOUBLE, data_space_N3);
            f1set.write(buf.force1.data(), H5::PredType::NATIVE_DOUBLE);
            t1set.write(buf.torque1.data(), H5::PredType::NATIVE_DOUBLE);
            f2set.write(buf.force2.data(), H5::PredType::NATIVE_DOUBLE);
            t2set.write(buf.torque2.data(), H5::PredType::NATIVE_DOUBLE);
        }
    }

    {
        H5::Group tgroup = m_fileHDF5->openGroup("/TSDAs");
        for (const auto& buf : m_tsda_buf) {
            H5::Group group = tgroup.createGroup(buf.name.c_str());
            H5::DataSet p1set = group.createDataSet("point1", H5::PredType::NATIVE_DOUBLE, data_space_N3);
            H5::DataSet p2set = group.createDataSet("point2", H5::PredType::NATIVE_DOUBLE, data_space_N3);
            H5::DataSet lset = group.createDataSet("len", H5::PredType::NATIVE_DOUBLE, data_space_N);
            H5::DataSet vset = group.createDataSet("vel", H5::PredType::NATIVE_DOUBLE, data_space_N);
            H5::DataSet fset = group.createDataSet("force", H5::PredType::NATIVE_DOUBLE, data_space_N);
            p1set.write(buf.point1.data(), H5::PredType::NATIVE_DOUBLE);
            p2set.write(buf.point2.data(), H5::PredType::NATIVE_DOUBLE);
            lset.write(buf.len.data(), H5::PredType::NATIVE_DOUBLE);
            vset.write(buf.vel.data(), H5::PredType::NATIVE_DOUBLE);
            fset.write(buf.force.data(), H5::PredType::NATIVE_DOUBLE);
        }
    }

    {
        H5::Group rgroup = m_fileHDF5->openGroup("/RSDAs");
        for (const auto& buf : m_rsda_buf) {
            H5::Group group = rgroup.createGroup(buf.name.c_str());
            H5::DataSet aset = group.createDataSet("ang", H5::PredType::NATIVE_DOUBLE, data_space_N);
            H5::DataSet vset = group.createDataSet("vel", H5::PredType::NATIVE_DOUBLE, data_space_N);
            H5::DataSet tset = group.createDataSet("torque", H5::PredType::NATIVE_DOUBLE, data_space_N);
            aset.write(buf.ang.data(), H5::PredType::NATIVE_DOUBLE);
            vset.write(buf.vel.data(), H5::PredType::NATIVE_DOUBLE);
            tset.write(buf.torque.data(), H5::PredType::NATIVE_DOUBLE);
        }
    }
}

// -----------------------------------------------------------------------------

// Open or create the data set with given name, data type, and data space in the provided group.
static H5::DataSet getDataSet(H5::Group* group, const char* dataset_name, const H5::DataType& data_type, const H5::DataSpace& data_space) {
    H5::DataSet set = (H5Lexists(group->getId(), dataset_name, H5P_DEFAULT))            //
                          ? group->openDataSet(dataset_name, H5P_DEFAULT)               //
                          : group->createDataSet(dataset_name, data_type, data_space);  //
    return set;
}

// Return a string with specified precision for the given integer.
static std::string format_number(int num, int precision) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(precision) << num;
    return out.str();
}

void ChOutputHDF5::WriteTimeStamp(int frame, double time) {
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

void ChOutputHDF5::WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
    if (bodies.empty())
        return;
    if (!m_frame_group)
        return;

    std::vector<body_info> info;
    for (const auto& body : bodies) {
        const auto& p = body->GetPos();
        const auto& q = body->GetRot();
        info.push_back({body->GetIdentifier(), p.x(), p.y(), p.z(), q.e0(), q.e1(), q.e2(), q.e3()});
    }

    hsize_t D = bodies.size();
    H5::DataSet set = getDataSet(m_frame_group, "Bodies", *m_body_type, H5::DataSpace(1, &D));
    set.write(info.data(), *m_body_type);
}

void ChOutputHDF5::WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) {
    if (markers.empty())
        return;
    if (!m_frame_group)
        return;

    std::vector<marker_info> info;
    for (const auto& marker : markers) {
        const ChVector3d& p = marker->GetAbsCoordsys().pos;
        const ChVector3d& pd = marker->GetAbsCoordsysDt().pos;
        const ChVector3d& pdd = marker->GetAbsCoordsysDt2().pos;
        info.push_back({marker->GetIdentifier(), p.x(), p.y(), p.z(), pd.x(), pd.y(), pd.z(), pdd.x(), pdd.y(), pdd.z()});
    }

    hsize_t D = markers.size();
    H5::DataSet set = getDataSet(m_frame_group, "Markers", *m_marker_type, H5::DataSpace(1, &D));
    set.write(info.data(), *m_marker_type);
}

void ChOutputHDF5::WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) {
    if (shafts.empty())
        return;
    if (!m_frame_group)
        return;

    std::vector<shaft_info> info;
    for (const auto& shaft : shafts) {
        info.push_back({shaft->GetIdentifier(), shaft->GetPos(), shaft->GetPosDt(), shaft->GetPosDt2(), shaft->GetAppliedLoad()});
    }

    hsize_t D = shafts.size();
    H5::DataSet set = getDataSet(m_frame_group, "Shafts", *m_shaft_type, H5::DataSpace(1, &D));
    set.write(info.data(), *m_shaft_type);
}

void ChOutputHDF5::WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) {
    if (joints.empty())
        return;
    if (!m_frame_group)
        return;

    std::vector<joint_info> info;
    for (const auto& joint : joints) {
        auto reaction = joint->GetReaction2();
        const ChVector3d& f = reaction.force;
        const ChVector3d& t = reaction.torque;
        info.push_back({joint->GetIdentifier(), f.x(), f.y(), f.z(), t.x(), t.y(), t.z()});
    }

    hsize_t D = joints.size();
    H5::DataSet set = getDataSet(m_frame_group, "Joints", *m_joint_type, H5::DataSpace(1, &D));
    set.write(info.data(), *m_joint_type);
}

void ChOutputHDF5::WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    if (couples.empty())
        return;
    if (!m_frame_group)
        return;

    std::vector<couple_info> info;
    for (const auto& couple : couples) {
        info.push_back({
            couple->GetIdentifier(), couple->GetRelativePos(),        //
            couple->GetRelativePosDt(), couple->GetRelativePosDt2(),  //
            couple->GetReaction1(), couple->GetReaction2()            //
        });
    }

    hsize_t D = couples.size();
    H5::DataSet set = getDataSet(m_frame_group, "Couples", *m_couple_type, H5::DataSpace(1, &D));
    set.write(info.data(), *m_couple_type);
}

void ChOutputHDF5::WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    if (springs.empty())
        return;
    if (!m_frame_group)
        return;

    std::vector<linspring_info> info;
    for (const auto& spring : springs) {
        info.push_back({spring->GetIdentifier(), spring->GetLength(), spring->GetVelocity(), spring->GetForce()});
    }

    hsize_t D = springs.size();
    H5::DataSet set = getDataSet(m_frame_group, "Lin Springs", *m_linspring_type, H5::DataSpace(1, &D));
    set.write(info.data(), *m_linspring_type);
}

void ChOutputHDF5::WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    if (springs.empty())
        return;
    if (!m_frame_group)
        return;

    std::vector<rotspring_info> info;
    for (const auto& spring : springs) {
        info.push_back({spring->GetIdentifier(), spring->GetAngle(), spring->GetVelocity(), spring->GetTorque()});
    }

    hsize_t D = springs.size();
    H5::DataSet set = getDataSet(m_frame_group, "Rot Springs", *m_rotspring_type, H5::DataSpace(1, &D));
    set.write(info.data(), *m_rotspring_type);
}

void ChOutputHDF5::WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    if (loads.empty())
        return;
    if (!m_frame_group)
        return;

    std::vector<bodyload_info> info;
    for (const auto& load : loads) {
        ChVector3d f = load->GetForce();
        ChVector3d t = load->GetTorque();
        info.push_back({load->GetIdentifier(), f.x(), f.y(), f.z(), t.x(), t.y(), t.z()});
    }

    hsize_t D = loads.size();
    H5::DataSet set = getDataSet(m_frame_group, "Body-body Loads", *m_bodyload_type, H5::DataSpace(1, &D));
    set.write(info.data(), *m_bodyload_type);
}

void ChOutputHDF5::WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
    if (motors.empty())
        return;
    if (!m_frame_group)
        return;

    std::vector<linmotor_info> info;
    for (const auto& motor : motors) {
        info.push_back({motor->GetIdentifier(), motor->GetMotorPos(), motor->GetMotorPosDt(), motor->GetMotorForce()});
    }

    hsize_t D = motors.size();
    H5::DataSet set = getDataSet(m_frame_group, "Lin Motors", *m_linmotor_type, H5::DataSpace(1, &D));
    set.write(info.data(), *m_linmotor_type);
}

void ChOutputHDF5::WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) {
    if (motors.empty())
        return;
    if (!m_frame_group)
        return;

    std::vector<linmotor_info> info;
    for (const auto& motor : motors) {
        info.push_back({motor->GetIdentifier(), motor->GetMotorAngle(), motor->GetMotorAngleDt(), motor->GetMotorTorque()});
    }

    hsize_t D = motors.size();
    H5::DataSet set = getDataSet(m_frame_group, "Rot Motors", *m_rotmotor_type, H5::DataSpace(1, &D));
    set.write(info.data(), *m_rotmotor_type);
}

}  // end namespace chrono
