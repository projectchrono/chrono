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

/// Forces are objects which must be attached to rigid bodies in order to apply force or torque to such body.
/// ChForce objects are able to represent either forces and torques, depending on a flag.
class ChApi ChForce : public ChObj {
  public:
    /// Types of force application
    enum ForceType { FORCE, TORQUE };

    /// Reference for position frame
    enum ReferenceFrame { BODY, WORLD };

    /// Reference for alignment
    enum AlignmentFrame { BODY_DIR, WORLD_DIR };

  public:
    /// Construct a ChForce object.
    /// By default, it is initialized as `FORCE` mode, `BODY` frame, and `BODY_DIR` alignment.
    ChForce();

    ChForce(const ChForce& other);

    virtual ~ChForce() {}

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
    void SetAlignment(AlignmentFrame m_align) { align = m_align; }
    AlignmentFrame GetAlignment() const { return align; }

    /// Sets the alignment method.
    /// The force application point will follow this reference.
    void SetFrame(ReferenceFrame ref_frame);
    ReferenceFrame GetFrame() const { return frame; }

    /// Gets the application point, in absolute coordinates.
    const ChVector3d& GetApplicationPointAbs() const { return app_point_abs; }
    /// Gets the application point, in rigid body coordinates.
    const ChVector3d& GetApplicationPointRel() const { return app_point_rel; }

    /// Sets the application point, in absolute coordinates.
    void SetApplicationPointAbs(const ChVector3d& point_abs);
    /// Sets the application point, in rigid body coordinates.
    void SetApplicationPointRel(const ChVector3d& point_rel);

    /// Gets the force (or torque) direction, in absolute coordinates.
    const ChVector3d& GetDirAbs() const { return vdir; }
    /// Gets the force (or torque) direction, in rigid body coordinates.
    const ChVector3d& GetDirRel() const { return vreldir; }

    /// Sets the force (or torque) direction, in absolute coordinates.
    void SetDirAbs(const ChVector3d& newdir_abs);
    /// Sets the force (or torque) direction, in rigid body coordinates.
    void SetDirRel(const ChVector3d& newdir_rel);

    /// Sets force (or torque) base magnitude.
    /// NB: this value gets modulated by related functions to provide total force.
    void SetForceMagnitude(double newf);
    /// Gets force (or torque) base magnitude.
    /// NB: this value gets modulated by related functions to provide total force.
    double GetForceMagnitude() const { return force_magnitude; }

    /// Sets a function for time-modulation of the force (or torque) magnitude.
    void SetMagnitudeModulation(std::shared_ptr<ChFunction> func) { modulation = func; }
    std::shared_ptr<ChFunction> GetMagnitudeModulation() const { return modulation; }

    /// Sets a function for time-modulation of force (or torque), X component.
    void SetForceModulationX(std::shared_ptr<ChFunction> func) { f_x = func; }
    std::shared_ptr<ChFunction> GetForceModulationX() const { return f_x; }
    /// Sets a function for time-modulation of force (or torque), Y component.
    void SetForceModulationY(std::shared_ptr<ChFunction> func) { f_y = func; }
    std::shared_ptr<ChFunction> GetForceModulationY() const { return f_y; }
    /// Sets a function for time-modulation of force (or torque), Z component.
    void SetForceModulationZ(std::shared_ptr<ChFunction> func) { f_z = func; }
    std::shared_ptr<ChFunction> GetForceModulationZ() const { return f_z; }

    /// Sets a function for time-modulation of application point position, X component.
    void SetApplicationPointModulationX(std::shared_ptr<ChFunction> func) { move_x = func; }
    std::shared_ptr<ChFunction> GetApplicationPointModulationX() const { return move_x; }
    /// Sets a function for time-modulation of application point position, Y component.
    void SetApplicationPointModulationY(std::shared_ptr<ChFunction> func) { move_y = func; }
    std::shared_ptr<ChFunction> GetApplicationPointModulationY() const { return move_y; }
    /// Sets a function for time-modulation of application point position, Z component.
    void SetApplicationPointModulationZ(std::shared_ptr<ChFunction> func) { move_z = func; }
    std::shared_ptr<ChFunction> GetApplicationPointModulationZ() const { return move_z; }

    /// Gets the total instant force (or torque) vector in absolute coordinates.
    /// This considers the effect of all applied modulations.
    const ChVector3d& GetTotalForceAbs() const { return force; }
    /// Gets the total instant force (or torque) vector in rigid body coordinates.
    /// This considers the effect of all applied modulations.
    const ChVector3d& GetTotalForceRel() const { return relforce; }

    /// Gets total force-torque applied to rigid body, as force vector (in absololute coords) and torque vector (in body coords).
    void GetBodyForceTorque(ChVector3d& body_force, ChVector3d& body_torque) const;

    /// Gets force-torque applied to rigid body, as lagrangian generalized force (7x1 matrix).
    const ChVectorN<double, 7>& GetQf() const { return Qf; }

    // UPDATING

    void UpdateState();
    virtual void Update(double time, UpdateFlags update_flags) override;

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

    ChVector3d app_point_abs;  ///< absolute point of application
    ChVector3d app_point_rel;  ///< relative point of application

    std::shared_ptr<ChFunction> move_x;  ///< motion x (abs or rel, depends on 'frame')
    std::shared_ptr<ChFunction> move_y;  ///< motion y  ""
    std::shared_ptr<ChFunction> move_z;  ///< motion z  ""
    ChVector3d restpos;                  ///< t=0 position (abs or rel, depends on 'frame')

    std::shared_ptr<ChFunction> f_x;  ///< fv strength x (abs or rel, see 'align')
    std::shared_ptr<ChFunction> f_y;  ///< fv strength y  ""
    std::shared_ptr<ChFunction> f_z;  ///< fv strength z  ""

    double force_magnitude;                  ///< fm scalar force strength
    std::shared_ptr<ChFunction> modulation;  ///< scalar force fm modulation

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
