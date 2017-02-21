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

#ifndef CHNODEBASE_H
#define CHNODEBASE_H

#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

namespace chrono {

/// Class for a node, that has some degrees of
/// freedom and that contain a proxy to the solver.
/// It is like a lightweight version of a ChPhysicsItem,
/// often a ChPhysicsItem is used as a container for a cluster
/// of these ChNodeBase.

class ChApi ChNodeBase {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChMaterialSurfaceBase)

  protected:
    unsigned int offset_x;  ///< offset in vector of state (position part)
    unsigned int offset_w;  ///< offset in vector of state (speed part)

  public:
    ChNodeBase();
    ChNodeBase(const ChNodeBase& other);
    virtual ~ChNodeBase() {}

    ChNodeBase& operator=(const ChNodeBase& other);

    //
    // Functions for interfacing to the state bookkeeping
    //

    /// Get the number of degrees of freedom
    virtual int Get_ndof_x() const = 0;

    /// Get the number of degrees of freedom, derivative
    /// This might be different from ndof if quaternions are used for rotations,
    /// as derivative might be angular velocity.
    virtual int Get_ndof_w() const { return Get_ndof_x(); }

    /// Get offset in the state vector (position part)
    unsigned int NodeGetOffset_x() { return offset_x; }
    /// Get offset in the state vector (speed part)
    unsigned int NodeGetOffset_w() { return offset_w; }

    /// Set offset in the state vector (position part)
    void NodeSetOffset_x(const unsigned int moff) { offset_x = moff; }
    /// Set offset in the state vector (speed part)
    void NodeSetOffset_w(const unsigned int moff) { offset_w = moff; }

    virtual void NodeIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) {}
    virtual void NodeIntStateScatter(const unsigned int off_x,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const double T) {}
    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {}
    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {}
    virtual void NodeIntStateIncrement(const unsigned int off_x,
                                       ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& Dv) {
        for (int i = 0; i < Get_ndof_x(); ++i) {
            x_new(off_x + i) = x(off_x + i) + Dv(off_v + i);
        }
    }
    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {}
    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) {}
    virtual void NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {}
    virtual void NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {}

    //
    // Functions for interfacing to the solver
    //

    /// Tell to a system descriptor that there are variables of type
    /// ChVariables in this object (for further passing it to a solver)
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) {}

    /// Sets the 'fb' part (the known term) of the encapsulated ChVariables to zero.
    virtual void VariablesFbReset() {}

    /// Adds the current forces (applied to node) into the
    /// encapsulated ChVariables, in the 'fb' part: qf+=forces*factor
    virtual void VariablesFbLoadForces(double factor = 1) {}

    /// Initialize the 'qb' part of the ChVariables with the
    /// current value of speeds.
    virtual void VariablesQbLoadSpeed() {}

    /// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
    /// with v_old using VariablesQbLoadSpeed, this method can be used in
    /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
    virtual void VariablesFbIncrementMq() {}

    /// Fetches the item speed (ex. linear velocity, in xyz nodes) from the
    /// 'qb' part of the ChVariables and sets it as the current item speed.
    /// If 'step' is not 0, also should compute the approximate acceleration of
    /// the item using backward differences, that is  accel=(new_speed-old_speed)/step.
    /// Mostly used after the solver provided the solution in ChVariables.
    virtual void VariablesQbSetSpeed(double step = 0) {}

    /// Increment node positions by the 'qb' part of the ChVariables,
    /// multiplied by a 'step' factor.
    ///     pos+=qb*step
    /// If qb is a speed, this behaves like a single step of 1-st order
    /// numerical integration (Eulero integration).
    virtual void VariablesQbIncrementPosition(double step) {}

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive);
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

CH_CLASS_VERSION(ChNodeBase,0)


}  // end namespace chrono

#endif
