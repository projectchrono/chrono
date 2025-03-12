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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHFORCE_H
#define CHFORCE_H

#include <cfloat>
#include <memory.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "chrono/core/ChFrame.h"
#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChObject.h"

namespace chrono {

// Forward reference
class ChBody;

/// Forces are objects which must be attached to rigid bodies in order
/// to apply torque or force to such body. ChForce objects are able to
/// represent either forces and torques, depending on a flag.

class ChApi ChForce : public ChObj {
  public:
    /// Types of force application
    enum ForceType { FORCE, TORQUE };

    /// Reference for position frame
    enum ReferenceFrame { BODY, WORLD };

    /// Reference for alignment
    enum AlignmentFrame { BODY_DIR, WORLD_DIR };

  public:
    ChForce();
    ChForce(const ChForce& other);
    ~ChForce() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChForce* Clone() const override { return new ChForce(*this); }

    /// Return the parent body (the force belongs to this rigid body)
    ChBody* GetBody() { return Body; }
    /// Sets the parent body (the force belongs to this rigid body)
    void SetBody(ChBody* newRB) { Body = newRB; }

    /// Sets the mode (force or torque)
    void SetMode(ForceType m_mode) { mode = m_mode; }
    ForceType GetMode() const { return mode; }

    /// Sets the alignment method.
    /// The force will rotate together with this reference.
    void SetAlign(AlignmentFrame m_align) { align = m_align; }
    AlignmentFrame GetAlign() const { return align; }

    /// Sets the alignment method.
    /// The force application point will follow this reference.
    void SetFrame(ReferenceFrame m_frame) {
        frame = m_frame;
        SetVpoint(vpoint);
    }
    ReferenceFrame GetFrame() const { return frame; }

    /// Gets the application point, in absolute coordinates.
    ChVector3d GetVpoint() const { return vpoint; }
    /// Gets the application point, in rigid body coordinates.
    ChVector3d GetVrelpoint() const { return vrelpoint; }

    /// Sets the application point, in absolute coordinates.
    void SetVpoint(ChVector3d mypoint);
    /// Sets the application point, in rigid body coordinates.
    void SetVrelpoint(ChVector3d myrelpoint);

    /// Gets the force (or torque) direction, in absolute coordinates.
    ChVector3d GetDir() const { return vdir; }
    /// Gets the force (or torque) direction, in rigid body coordinates.
    ChVector3d GetRelDir() const { return vreldir; }
    /// Sets the force (or torque) direction, in absolute coordinates.
    void SetDir(ChVector3d newf);
    /// Sets the force (or torque) direction, in rigid body coordinates.
    void SetRelDir(ChVector3d newf);

    /// Sets force (or torque) modulus.
    void SetMforce(double newf);
    /// Gets force (or torque) modulus.
    double GetMforce() const { return mforce; }

    /// Sets a function for time-modulation of the force.
    void SetModulation(std::shared_ptr<ChFunction> m_funct) { modula = m_funct; }
    std::shared_ptr<ChFunction> GetModulation() const { return modula; }

    /// Sets a function for time dependency of position (on x axis)
    void SetMove_x(std::shared_ptr<ChFunction> m_funct) { move_x = m_funct; }
    std::shared_ptr<ChFunction> GetMove_x() const { return move_x; }
    /// Sets a function for time dependency of position (on y axis)
    void SetMove_y(std::shared_ptr<ChFunction> m_funct) { move_y = m_funct; }
    std::shared_ptr<ChFunction> GetMove_y() const { return move_y; }
    /// Sets a function for time dependency of position (on z axis)
    void SetMove_z(std::shared_ptr<ChFunction> m_funct) { move_z = m_funct; }
    std::shared_ptr<ChFunction> GetMove_z() const { return move_z; }

    /// Sets a function for time dependency of force X component.
    void SetF_x(std::shared_ptr<ChFunction> m_funct) { f_x = m_funct; }
    std::shared_ptr<ChFunction> GetF_x() const { return f_x; }
    /// Sets a function for time dependency of force Y component.
    void SetF_y(std::shared_ptr<ChFunction> m_funct) { f_y = m_funct; }
    std::shared_ptr<ChFunction> GetF_y() const { return f_y; }
    /// Sets a function for time dependency of force Z component.
    void SetF_z(std::shared_ptr<ChFunction> m_funct) { f_z = m_funct; }
    std::shared_ptr<ChFunction> GetF_z() const { return f_z; }

    /// Gets the instant force vector -or torque vector- in absolute coordinates.
    ChVector3d GetForce() const { return force; }
    /// Gets the instant force vector -or torque vector- in rigid body coordinates.
    ChVector3d GetRelForce() const { return relforce; }
    /// Gets the instant force vector -or torque vector- modulus.
    double GetForceMod() const { return force.Length(); }

    /// Gets force-torque applied to rigid body, as lagrangian generalized force (7x1 matrix).
    const ChVectorN<double, 7>& GetQf() const { return Qf; }
    /// Gets force-torque applied to rigid body, as force vector (in absol.coords)
    /// and torque vector (in body coords).
    void GetBodyForceTorque(ChVector3d& body_force, ChVector3d& body_torque) const;

    // UPDATING

    void UpdateState();
    virtual void Update(double time, bool update_assets) override;

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    CH_ENUM_MAPPER_BEGIN(ForceType);
    CH_ENUM_VAL(FORCE);
    CH_ENUM_VAL(TORQUE);
    CH_ENUM_MAPPER_END(ForceType);

    CH_ENUM_MAPPER_BEGIN(ReferenceFrame);
    CH_ENUM_VAL(BODY);
    CH_ENUM_VAL(WORLD);
    CH_ENUM_MAPPER_END(ReferenceFrame);

    CH_ENUM_MAPPER_BEGIN(AlignmentFrame);
    CH_ENUM_VAL(BODY_DIR);
    CH_ENUM_VAL(WORLD_DIR);
    CH_ENUM_MAPPER_END(AlignmentFrame);

  private:
    ChBody* Body;  ///< object of application

    ForceType mode;        ///< force or torque
    ReferenceFrame frame;  ///< fix position in body csys or world csys
    AlignmentFrame align;  ///< fix direction in body csys or world csys

    ChVector3d vpoint;     ///< absolute point of application
    ChVector3d vrelpoint;  ///< relative point of application

    std::shared_ptr<ChFunction> move_x;  ///< motion x (abs or rel, depends on 'frame')
    std::shared_ptr<ChFunction> move_y;  ///< motion y  ""
    std::shared_ptr<ChFunction> move_z;  ///< motion z  ""
    ChVector3d restpos;                  ///< t=0 position (abs or rel, depends on 'frame')

    std::shared_ptr<ChFunction> f_x;  ///< fv strengh x (abs or rel, see 'align')
    std::shared_ptr<ChFunction> f_y;  ///< fv strengh y  ""
    std::shared_ptr<ChFunction> f_z;  ///< fv strengh z  ""

    double mforce;                       ///< fm scalar force strenght
    std::shared_ptr<ChFunction> modula;  ///< scalar force fm modulation

    ChVector3d vdir;     ///< force/torque abs.direction
    ChVector3d vreldir;  ///< force/torque rel direction

    ChVector3d force;     ///< TOTAL force vect (abs.coord) = fm*vdir +fx+fy+fz
    ChVector3d relforce;  ///< TOTAL force vect (rel.coord) = fm*vdir +fx+fy+fz

    ChVectorN<double, 7> Qf;  ///< Lagrangian force

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CH_CLASS_VERSION(ChForce, 0)

}  // end namespace chrono

#endif
