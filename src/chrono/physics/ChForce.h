// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "chrono/core/ChLog.h"
#include "chrono/core/ChMath.h"
#include "chrono/motion_functions/ChFunction.h"
#include "chrono/physics/ChObject.h"

namespace chrono {

// Forward reference
class ChBody;

/// Forces are objects which must be attached to rigid bodies in order
/// to apply torque or force to such body. ChForce objects are able to
/// represent either forces and torques, depending on a flag.

class ChApi ChForce : public ChObj {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChForce)

  public:
    // Types of force application
    enum ForceType { FORCE, TORQUE };

    // Reference for position frame
    enum ReferenceFrame { BODY, WORLD };

    // Reference for alignment
    enum AlignmentFrame { BODY_DIR, WORLD_DIR };

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

    ChVector<> vpoint;     ///< absolute point of application
    ChVector<> vrelpoint;  ///< relative point of application

    std::shared_ptr<ChFunction> move_x;  ///< motion x (abs or rel, depends on 'frame')
    std::shared_ptr<ChFunction> move_y;  ///< motion y  ""
    std::shared_ptr<ChFunction> move_z;  ///< motion z  ""
    ChVector<> restpos;                  ///< t=0 position (abs or rel, depends on 'frame')

    std::shared_ptr<ChFunction> f_x;  ///< fv strengh x (abs or rel, see 'align')
    std::shared_ptr<ChFunction> f_y;  ///< fv strengh y  ""
    std::shared_ptr<ChFunction> f_z;  ///< fv strengh z  ""

    double mforce;                       ///< fm scalar force strenght
    std::shared_ptr<ChFunction> modula;  ///< scalar force fm modulation

    ChVector<> vdir;     ///< force/torque abs.direction
    ChVector<> vreldir;  ///< force/torque rel direction

    ChVector<> force;     ///< TOTAL force vect (abs.coord) = fm*vdir +fx+fy+fz
    ChVector<> relforce;  ///< TOTAL force vect (rel.coord) = fm*vdir +fx+fy+fz

    ChMatrixDynamic<>* Qf;  ///< Lagrangian force

  public:
    ChForce();
    ChForce(const ChForce& other);
    ~ChForce();

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
    ChVector<> GetVpoint() const { return vpoint; }
    /// Gets the application point, in rigid body coordinates.
    ChVector<> GetVrelpoint() const { return vrelpoint; }

    /// Gets the application point, in absolute coordinates.
    void SetVpoint(ChVector<> mypoint);
    /// Gets the application point, in rigid body coordinates.
    void SetVrelpoint(ChVector<> myrelpoint);

    /// Gets the force (or torque) direction, in absolute coordinates.
    ChVector<> GetDir() const { return vdir; }
    /// Gets the force (or torque) direction, in rigid body coordinates.
    ChVector<> GetRelDir() const { return vreldir; }
    /// Sets the force (or torque) direction, in absolute coordinates.
    void SetDir(ChVector<> newf);
    /// Sets the force (or torque) direction, in rigid body coordinates.
    void SetRelDir(ChVector<> newf);

    /// Sets force (or torque) modulus.
    void SetMforce(double newf);
    /// Gets force (or torque) modulus.
    double GetMforce() const { return mforce; }

    /// Sets a f(t) function for time-modulation of the force.
    void SetModulation(std::shared_ptr<ChFunction> m_funct) { modula = m_funct; }
    std::shared_ptr<ChFunction> GetModulation() const { return modula; }

    /// Sets a f(t) function for time dependency of position (on x axis)
    void SetMove_x(std::shared_ptr<ChFunction> m_funct) { move_x = m_funct; }
    std::shared_ptr<ChFunction> GetMove_x() const { return move_x; }
    /// Sets a f(t) function for time dependency of position (on y axis)
    void SetMove_y(std::shared_ptr<ChFunction> m_funct) { move_y = m_funct; }
    std::shared_ptr<ChFunction> GetMove_y() const { return move_y; }
    /// Sets a f(t) function for time dependency of position (on z axis)
    void SetMove_z(std::shared_ptr<ChFunction> m_funct) { move_z = m_funct; }
    std::shared_ptr<ChFunction> GetMove_z() const { return move_z; }

    /// Sets a f(t) function for time dependency of force X component.
    void SetF_x(std::shared_ptr<ChFunction> m_funct) { f_x = m_funct; }
    std::shared_ptr<ChFunction> GetF_x() const { return f_x; }
    /// Sets a f(t) function for time dependency of force Y component.
    void SetF_y(std::shared_ptr<ChFunction> m_funct) { f_y = m_funct; }
    std::shared_ptr<ChFunction> GetF_y() const { return f_y; }
    /// Sets a f(t) function for time dependency of force Z component.
    void SetF_z(std::shared_ptr<ChFunction> m_funct) { f_z = m_funct; }
    std::shared_ptr<ChFunction> GetF_z() const { return f_z; }

    /// Gets the instant force vector -or torque vector- in absolute coordinates.
    ChVector<> GetForce() const { return force; }
    /// Gets the instant force vector -or torque vector- in rigid body coordinates.
    ChVector<> GetRelForce() const { return relforce; }
    /// Gets the instant force vector -or torque vector- modulus.
    double GetForceMod() const { return force.Length(); }

    /// Gets force-torque applied to rigid body, as lagrangian generalized force (7x1 matrix).
    ChMatrix<>* GetQf() { return Qf; }
    /// Gets force-torque applied to rigid body, as force vector (in absol.coords)
    /// and torque vector (in body coords).
    void GetBodyForceTorque(ChVector<>& body_force, ChVector<>& body_torque) const;

    //
    // UPDATING
    //

    void UpdateTime(double mytime);
    void UpdateState();
    void Update(double mytime);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChForce,0)


}  // end namespace chrono

#endif
