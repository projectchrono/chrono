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
// Base class for a Chrono output database.
//
// =============================================================================

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkUniversal.h"

#include "chrono/output/ChOutputHDF5.h"

namespace chrono {

// -----------------------------------------------------------------------------

class ChOutputHDF5_impl {
  public:
    ChOutputHDF5_impl(H5::H5File* fileHDF5) : m_fileHDF5(fileHDF5) {}
     virtual ~ChOutputHDF5_impl() {}

     virtual void Initialize() = 0;

     virtual void WriteTime(int frame, double time) = 0;
     virtual void WriteSection(const std::string& name) = 0;
     virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) = 0;
     virtual void WriteAuxRefBodies(const std::vector<std::shared_ptr<ChBodyAuxRef>>& bodies) = 0;
     virtual void WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) = 0;
     virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) = 0;
     virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) = 0;
     virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) = 0;
     virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) = 0;
     virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) = 0;
     virtual void WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) = 0;
     virtual void WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) = 0;
     virtual void WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) = 0;
 
    H5::H5File* m_fileHDF5;
};

class ChOutputHDF5_series : public ChOutputHDF5_impl {
  public:
    ChOutputHDF5_series(H5::H5File* fileHDF5) : ChOutputHDF5_impl(fileHDF5) {}
    ~ChOutputHDF5_series() {}
    virtual void Initialize() override {}
    virtual void WriteTime(int frame, double time) override {}
    virtual void WriteSection(const std::string& name) override {}
    virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) override {}
    virtual void WriteAuxRefBodies(const std::vector<std::shared_ptr<ChBodyAuxRef>>& bodies) override {}
    virtual void WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) override {}
    virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) override {}
    virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) override {}
    virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) override {}
    virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) override {}
    virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) override {}
    virtual void WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) override {}
    virtual void WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) override {}
    virtual void WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) override {}
};

class ChOutputHDF5_frames : public ChOutputHDF5_impl {
  public:
    ChOutputHDF5_frames(H5::H5File* fileHDF5);
    ~ChOutputHDF5_frames();
    virtual void Initialize() override;
    virtual void WriteTime(int frame, double time) override;
    virtual void WriteSection(const std::string& name) override;
    virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) override;
    virtual void WriteAuxRefBodies(const std::vector<std::shared_ptr<ChBodyAuxRef>>& bodies) override;
    virtual void WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) override;
    virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) override;
    virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) override;
    virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) override;
    virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) override;
    virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) override;
    virtual void WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) override;
    virtual void WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) override;
    virtual void WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) override;

  private:
    H5::Group* m_frame_group;
    H5::Group* m_section_group;

    static H5::CompType* m_body_type;
    static H5::CompType* m_bodyaux_type;
    static H5::CompType* m_shaft_type;
    static H5::CompType* m_marker_type;
    static H5::CompType* m_joint_type;
    static H5::CompType* m_couple_type;
    static H5::CompType* m_linspring_type;
    static H5::CompType* m_rotspring_type;
    static H5::CompType* m_bodyload_type;
    static H5::CompType* m_linmotor_type;
    static H5::CompType* m_rotmotor_type;

    static const H5::CompType& getBodyType();
    static const H5::CompType& getBodyAuxType();
    static const H5::CompType& getShaftType();
    static const H5::CompType& getMarkerType();
    static const H5::CompType& getJointType();
    static const H5::CompType& getCoupleType();
    static const H5::CompType& getLinSpringType();
    static const H5::CompType& getRotSpringType();
    static const H5::CompType& getBodyLoadType();
    static const H5::CompType& getLinMotorType();
    static const H5::CompType& getRotMotorType();
};

// -----------------------------------------------------------------------------

ChOutputHDF5::ChOutputHDF5(const std::string& filename, Mode mode) : m_mode(mode), m_initialized(false) {
    m_fileHDF5 = new H5::H5File(filename, H5F_ACC_TRUNC);

    switch (mode) {
        case Mode::FRAMES:
            m_impl = chrono_types::make_unique<ChOutputHDF5_frames>(m_fileHDF5);
            break;
        case Mode::SERIES:
            m_impl = chrono_types::make_unique<ChOutputHDF5_series>(m_fileHDF5);
            break;
    }
}

ChOutputHDF5::~ChOutputHDF5() {
    m_fileHDF5->close();
}

void ChOutputHDF5::Initialize() {
    if (!m_initialized)
        m_impl->Initialize();
    m_initialized = true;
}

void ChOutputHDF5::WriteTime(int frame, double time) {
    m_impl->WriteTime(frame, time);
}
void ChOutputHDF5::WriteSection(const std::string& name) {
    m_impl->WriteSection(name);
}

void ChOutputHDF5::WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
    m_impl->WriteBodies(bodies);
}
void ChOutputHDF5::WriteAuxRefBodies(const std::vector<std::shared_ptr<ChBodyAuxRef>>& bodies) {
    m_impl->WriteAuxRefBodies(bodies);
}
void ChOutputHDF5::WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) {
    m_impl->WriteMarkers(markers);
}
void ChOutputHDF5::WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) {
    m_impl->WriteShafts(shafts);
}
void ChOutputHDF5::WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) {
    m_impl->WriteJoints(joints);
}
void ChOutputHDF5::WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    m_impl->WriteCouples(couples);
}
void ChOutputHDF5::WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    m_impl->WriteLinSprings(springs);
}
void ChOutputHDF5::WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    m_impl->WriteRotSprings(springs);
}
void ChOutputHDF5::WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    m_impl->WriteBodyBodyLoads(loads);
}
void ChOutputHDF5::WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
    m_impl->WriteLinMotors(motors);
}
void ChOutputHDF5::WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) {
    m_impl->WriteRotMotors(motors);
}

// -----------------------------------------------------------------------------

std::string format_number(int num, int precision) {
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(precision) << num;
    return out.str();
}

// -----------------------------------------------------------------------------

// ChOutputHDF5_frames implementation

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
    int id;                 // body identifier
    double x, y, z;         // position
    double e0, e1, e2, e3;  // orientation
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

H5::CompType* ChOutputHDF5_frames::m_body_type = nullptr;
H5::CompType* ChOutputHDF5_frames::m_bodyaux_type = nullptr;
H5::CompType* ChOutputHDF5_frames::m_shaft_type = nullptr;
H5::CompType* ChOutputHDF5_frames::m_marker_type = nullptr;
H5::CompType* ChOutputHDF5_frames::m_joint_type = nullptr;
H5::CompType* ChOutputHDF5_frames::m_couple_type = nullptr;
H5::CompType* ChOutputHDF5_frames::m_linspring_type = nullptr;
H5::CompType* ChOutputHDF5_frames::m_rotspring_type = nullptr;
H5::CompType* ChOutputHDF5_frames::m_bodyload_type = nullptr;
H5::CompType* ChOutputHDF5_frames::m_linmotor_type = nullptr;
H5::CompType* ChOutputHDF5_frames::m_rotmotor_type = nullptr;

const H5::CompType& ChOutputHDF5_frames::getBodyType() {
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

const H5::CompType& ChOutputHDF5_frames::getBodyAuxType() {
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

const H5::CompType& ChOutputHDF5_frames::getShaftType() {
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

const H5::CompType& ChOutputHDF5_frames::getMarkerType() {
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

const H5::CompType& ChOutputHDF5_frames::getJointType() {
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

const H5::CompType& ChOutputHDF5_frames::getCoupleType() {
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

const H5::CompType& ChOutputHDF5_frames::getLinSpringType() {
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

const H5::CompType& ChOutputHDF5_frames::getRotSpringType() {
    if (!m_rotspring_type) {
        struct Initializer {
            Initializer() {
                m_rotspring_type = new H5::CompType(sizeof(rotspring_info));
                m_rotspring_type->insertMember("id", HOFFSET(rotspring_info, id), H5::PredType::NATIVE_INT);
                m_rotspring_type->insertMember("x", HOFFSET(rotspring_info, x), H5::PredType::NATIVE_DOUBLE);
                m_rotspring_type->insertMember("xd", HOFFSET(rotspring_info, xd), H5::PredType::NATIVE_DOUBLE);
                m_rotspring_type->insertMember("torque", HOFFSET(rotspring_info, t), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_rotspring_type;
}

const H5::CompType& ChOutputHDF5_frames::getBodyLoadType() {
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

const H5::CompType& ChOutputHDF5_frames::getLinMotorType() {
    if (!m_linmotor_type) {
        struct Initializer {
            Initializer() {
                m_linmotor_type = new H5::CompType(sizeof(linmotor_info));
                m_linmotor_type->insertMember("id", HOFFSET(linmotor_info, id), H5::PredType::NATIVE_INT);
                m_linmotor_type->insertMember("x", HOFFSET(linmotor_info, x), H5::PredType::NATIVE_DOUBLE);
                m_linmotor_type->insertMember("xd", HOFFSET(linmotor_info, xd), H5::PredType::NATIVE_DOUBLE);
                m_linmotor_type->insertMember("force", HOFFSET(linmotor_info, f), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_linmotor_type;
}

const H5::CompType& ChOutputHDF5_frames::getRotMotorType() {
    if (!m_rotmotor_type) {
        struct Initializer {
            Initializer() {
                m_rotmotor_type = new H5::CompType(sizeof(rotmotor_info));
                m_rotmotor_type->insertMember("id", HOFFSET(rotmotor_info, id), H5::PredType::NATIVE_INT);
                m_rotmotor_type->insertMember("x", HOFFSET(rotmotor_info, x), H5::PredType::NATIVE_DOUBLE);
                m_rotmotor_type->insertMember("xd", HOFFSET(rotmotor_info, xd), H5::PredType::NATIVE_DOUBLE);
                m_rotmotor_type->insertMember("torque", HOFFSET(rotmotor_info, t), H5::PredType::NATIVE_DOUBLE);
            }
        };
        static Initializer ListInitializationGuard;
    }
    return *m_rotmotor_type;
}

// -----------------------------------------------------------------------------

ChOutputHDF5_frames::ChOutputHDF5_frames(H5::H5File* fileHDF5)
    : ChOutputHDF5_impl(fileHDF5), m_frame_group(nullptr), m_section_group(nullptr) {
}

ChOutputHDF5_frames::~ChOutputHDF5_frames() {
    if (m_section_group)
        m_section_group->close();
    if (m_frame_group)
        m_frame_group->close();

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
    delete m_bodyload_type;
    delete m_linmotor_type;
    delete m_rotmotor_type;
}

void ChOutputHDF5_frames::Initialize() {
    H5::Group frames_group(m_fileHDF5->createGroup("/Frames"));
}

// -----------------------------------------------------------------------------

void ChOutputHDF5_frames::WriteTime(int frame, double time) {
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

void ChOutputHDF5_frames::WriteSection(const std::string& name) {
    // Close the currently open section group
    if (m_section_group) {
        m_section_group->close();
        delete m_section_group;
        m_section_group = nullptr;
    }

    // Create the group for this section in the current frame group
    m_section_group = new H5::Group(m_frame_group->createGroup(name));
}

void ChOutputHDF5_frames::WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) {
    if (bodies.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto nbodies = bodies.size();
    hsize_t dim[] = {nbodies};
    H5::DataSpace dataspace(1, dim);
    std::vector<body_info> info(nbodies);
    for (auto i = 0; i < nbodies; i++) {
        const ChVector3d& p = bodies[i]->GetPos();
        const ChQuaternion<>& q = bodies[i]->GetRot();
        info[i] = {bodies[i]->GetIdentifier(), p.x(), p.y(), p.z(), q.e0(), q.e1(), q.e2(), q.e3()};
    }

    H5::DataSet set = crt_group->createDataSet("Bodies", getBodyType(), dataspace);
    set.write(info.data(), getBodyType());
}

void ChOutputHDF5_frames::WriteAuxRefBodies(const std::vector<std::shared_ptr<ChBodyAuxRef>>& bodies) {
    if (bodies.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto nbodies = bodies.size();
    hsize_t dim[] = {nbodies};
    H5::DataSpace dataspace(1, dim);
    std::vector<bodyaux_info> info(nbodies);
    for (auto i = 0; i < nbodies; i++) {
        const ChVector3d& p = bodies[i]->GetPos();
        const ChQuaternion<>& q = bodies[i]->GetRot();
        info[i] = {bodies[i]->GetIdentifier(), p.x(), p.y(), p.z(), q.e0(), q.e1(), q.e2(), q.e3()};
    }

    H5::DataSet set = crt_group->createDataSet("Bodies AuxRef", getBodyAuxType(), dataspace);
    set.write(info.data(), getBodyAuxType());
}

void ChOutputHDF5_frames::WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) {
    if (markers.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto nmarkers = markers.size();
    hsize_t dim[] = {nmarkers};
    H5::DataSpace dataspace(1, dim);
    std::vector<marker_info> info(nmarkers);
    for (auto i = 0; i < nmarkers; i++) {
        const ChVector3d& p = markers[i]->GetAbsCoordsys().pos;
        const ChVector3d& pd = markers[i]->GetAbsCoordsysDt().pos;
        const ChVector3d& pdd = markers[i]->GetAbsCoordsysDt2().pos;
        info[i] = {markers[i]->GetIdentifier(), p.x(), p.y(), p.z(), pd.x(), pd.y(), pd.z(), pdd.x(), pdd.y(), pdd.z()};
    }

    H5::DataSet set = crt_group->createDataSet("Markers", getMarkerType(), dataspace);
    set.write(info.data(), getMarkerType());
}

void ChOutputHDF5_frames::WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) {
    if (shafts.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto nshafts = shafts.size();
    hsize_t dim[] = {nshafts};
    H5::DataSpace dataspace(1, dim);
    std::vector<shaft_info> info(nshafts);
    for (auto i = 0; i < nshafts; i++) {
        info[i] = {shafts[i]->GetIdentifier(), shafts[i]->GetPos(), shafts[i]->GetPosDt(), shafts[i]->GetPosDt2(),
                   shafts[i]->GetAppliedLoad()};
    }

    H5::DataSet set = crt_group->createDataSet("Shafts", getShaftType(), dataspace);
    set.write(info.data(), getShaftType());
}

void ChOutputHDF5_frames::WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) {
    if (joints.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto njoints = joints.size();
    hsize_t dim[] = {njoints};
    H5::DataSpace dataspace(1, dim);
    std::vector<joint_info> info(njoints);
    for (auto i = 0; i < njoints; i++) {
        auto reaction = joints[i]->GetReaction2();
        const ChVector3d& f = reaction.force;
        const ChVector3d& t = reaction.torque;
        info[i] = {joints[i]->GetIdentifier(), f.x(), f.y(), f.z(), t.x(), t.y(), t.z()};
    }

    H5::DataSet set = crt_group->createDataSet("Joints", getJointType(), dataspace);
    set.write(info.data(), getJointType());
}

void ChOutputHDF5_frames::WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) {
    if (couples.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto ncouples = couples.size();
    hsize_t dim[] = {ncouples};
    H5::DataSpace dataspace(1, dim);
    std::vector<couple_info> info(ncouples);
    for (auto i = 0; i < ncouples; i++) {
        info[i] = {
            couples[i]->GetIdentifier(),    couples[i]->GetRelativePos(),     //
            couples[i]->GetRelativePosDt(), couples[i]->GetRelativePosDt2(),  //
            couples[i]->GetReaction1(),     couples[i]->GetReaction2()        //
        };
    }

    H5::DataSet set = crt_group->createDataSet("Couples", getCoupleType(), dataspace);
    set.write(info.data(), getCoupleType());
}

void ChOutputHDF5_frames::WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) {
    if (springs.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto nsprings = springs.size();
    hsize_t dim[] = {nsprings};
    H5::DataSpace dataspace(1, dim);
    std::vector<linspring_info> info(nsprings);
    for (auto i = 0; i < nsprings; i++) {
        info[i] = {springs[i]->GetIdentifier(), springs[i]->GetLength(), springs[i]->GetVelocity(),
                   springs[i]->GetForce()};
    }

    H5::DataSet set = crt_group->createDataSet("Lin Springs", getLinSpringType(), dataspace);
    set.write(info.data(), getLinSpringType());
}

void ChOutputHDF5_frames::WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) {
    if (springs.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto nsprings = springs.size();
    hsize_t dim[] = {nsprings};
    H5::DataSpace dataspace(1, dim);
    std::vector<rotspring_info> info(nsprings);
    for (auto i = 0; i < nsprings; i++) {
        info[i] = {springs[i]->GetIdentifier(), springs[i]->GetAngle(), springs[i]->GetVelocity(),
                   springs[i]->GetTorque()};
    }

    H5::DataSet set = crt_group->createDataSet("Rot Springs", getRotSpringType(), dataspace);
    set.write(info.data(), getRotSpringType());
}

void ChOutputHDF5_frames::WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) {
    if (loads.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto nloads = loads.size();
    hsize_t dim[] = {nloads};
    H5::DataSpace dataspace(1, dim);
    std::vector<bodyload_info> info(nloads);
    for (auto i = 0; i < nloads; i++) {
        ChVector3d f = loads[i]->GetForce();
        ChVector3d t = loads[i]->GetTorque();
        info[i] = {loads[i]->GetIdentifier(), f.x(), f.y(), f.z(), t.x(), t.y(), t.z()};
    }

    H5::DataSet set = crt_group->createDataSet("Body-body Loads", getBodyLoadType(), dataspace);
    set.write(info.data(), getBodyLoadType());
}

void ChOutputHDF5_frames::WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) {
    if (motors.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto nmotors = motors.size();
    hsize_t dim[] = {nmotors};
    H5::DataSpace dataspace(1, dim);
    std::vector<linmotor_info> info(nmotors);
    for (auto i = 0; i < nmotors; i++) {
        info[i] = {motors[i]->GetIdentifier(), motors[i]->GetMotorPos(), motors[i]->GetMotorPosDt(),
                   motors[i]->GetMotorForce()};
    }

    H5::DataSet set = crt_group->createDataSet("Lin Motors", getLinMotorType(), dataspace);
    set.write(info.data(), getLinMotorType());
}

void ChOutputHDF5_frames::WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) {
    if (motors.empty())
        return;
    auto crt_group = m_section_group ? m_section_group : m_frame_group;
    if (!crt_group)
        return;

    auto nmotors = motors.size();
    hsize_t dim[] = {nmotors};
    H5::DataSpace dataspace(1, dim);
    std::vector<linmotor_info> info(nmotors);
    for (auto i = 0; i < nmotors; i++) {
        info[i] = {motors[i]->GetIdentifier(), motors[i]->GetMotorAngle(), motors[i]->GetMotorAngleDt(),
                   motors[i]->GetMotorTorque()};
    }

    H5::DataSet set = crt_group->createDataSet("Rot Motors", getRotMotorType(), dataspace);
    set.write(info.data(), getRotMotorType());
}

}  // end namespace chrono
