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

#ifndef CHNODEBASE_H
#define CHNODEBASE_H

#include "chrono/physics/ChPhysicsItem.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"

namespace chrono {

/// Class for a node, that has some degrees of freedom.
/// It is like a lightweight version of a ChPhysicsItem; often a ChPhysicsItem is used as a
/// container for a cluster of these ChNodeBase.
class ChApi ChNodeBase {
  public:
    ChNodeBase();
    ChNodeBase(const ChNodeBase& other);
    virtual ~ChNodeBase() {}

    ChNodeBase& operator=(const ChNodeBase& other);

    // Functions for interfacing to the state bookkeeping

    /// Get the number of degrees of freedom.
    virtual unsigned int GetNumCoordsPosLevel() const = 0;

    /// Get the number of degrees of freedom, derivative.
    /// This might be different from ndof_x if quaternions are used for rotations and derivative is angular velocity.
    virtual unsigned int GetNumCoordsVelLevel() const { return GetNumCoordsPosLevel(); }

    /// Get the actual number of active degrees of freedom.
    /// The default implementation returns the full number of DOFs for this node, but derived classes may allow fixing
    /// some of the node variables.
    virtual unsigned int GetNumCoordsPosLevelActive() const { return GetNumCoordsPosLevel(); }

    /// Get the actual number of active degrees of freedom, derivative.
    /// The default implementation returns the full number of DOFs for this node, but derived classes may allow fixing
    /// some of the node variables.
    virtual unsigned int GetNumCoordsVelLevelActive() const { return GetNumCoordsVelLevel(); }

    /// Return true if all node DOFs are active (no node variable is fixed).
    virtual bool IsAllCoordsActive() const { return true; }

    /// Get offset in the state vector (position part).
    unsigned int NodeGetOffsetPosLevel() { return offset_x; }

    /// Get offset in the state vector (speed part).
    unsigned int NodeGetOffsetVelLevel() { return offset_w; }

    /// Set offset in the state vector (position part).
    void NodeSetOffsetPosLevel(const unsigned int moff) { offset_x = moff; }
    /// Set offset in the state vector (speed part).
    void NodeSetOffsetVelLevel(const unsigned int moff) { offset_w = moff; }

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
                                       const ChStateDelta& Dv);
    virtual void NodeIntStateGetIncrement(const unsigned int off_x,
                                          const ChState& x_new,
                                          const ChState& x,
                                          const unsigned int off_v,
                                          ChStateDelta& Dv);
    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {}
    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) {}
    virtual void NodeIntLoadResidual_F_domain(const unsigned int off, 
                                        ChVectorDynamic<>& R,  
                                        const double c,        
                                        const ChOverlapTest& filter 
                                    );
    virtual void NodeIntLoadResidual_Mv_domain(const unsigned int off,   
                                        ChVectorDynamic<>& R,      
                                        const ChVectorDynamic<>& w, 
                                        const double c,           
                                        const ChOverlapTest& filter
                                    );
    virtual void NodeIntLoadLumpedMass_Md(const unsigned int off,
                                          ChVectorDynamic<>& Md,
                                          double& error,
                                          const double c){};
    virtual void NodeIntLoadIndicator(const unsigned int off, 
                                      ChVectorDynamic<>& N ){}
    virtual void NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {}
    virtual void NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {}

    // Functions for interfacing to the solver

    /// Register with the given system descriptor any ChVariable objects associated with this item.
    virtual void InjectVariables(ChSystemDescriptor& descriptor) {}

    /// Set the 'fb' part (the known term) of the encapsulated ChVariables to zero.
    virtual void VariablesFbReset() {}

    /// Add the current forces (applied to node) into the encapsulated ChVariables.
    /// Include in the 'fb' part: qf+=forces*factor
    virtual void VariablesFbLoadForces(double factor = 1) {}

    /// Initialize the 'qb' part of the ChVariables with the current value of speeds.
    virtual void VariablesQbLoadSpeed() {}

    /// Add M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
    /// with v_old using VariablesQbLoadSpeed, this method can be used in
    /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
    virtual void VariablesFbIncrementMq() {}

    /// Fetch the item speed (ex. linear velocity, in xyz nodes) from the
    /// 'qb' part of the ChVariables and sets it as the current item speed.
    /// If 'step' is not 0, also should compute the approximate acceleration of
    /// the item using backward differences, that is  accel=(new_speed-old_speed)/step.
    /// Mostly used after the solver provided the solution in ChVariables.
    virtual void VariablesQbSetSpeed(double step = 0) {}

    /// Increment node positions by the 'qb' part of the ChVariables,
    /// multiplied by a 'step' factor.
    ///     pos+=qb*step
    /// If qb is a speed, this behaves like a single step of 1-st order
    /// numerical integration (Euler integration).
    virtual void VariablesQbIncrementPosition(double step) {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

    /// Set an object integer tag (default: -1).
    /// Unlike the object identifier, this tag is completely under user control and not used anywhere else in Chrono.
    /// Tags are serialized and de-serialized.
    void SetTag(int tag) { m_tag = tag; }

    /// Get the tag of this object.
    int GetTag() const { return m_tag; }

    /// Get a symbolic 'center' of the node (ex. the position). 
    /// Derived classes must override this.
    virtual ChVector3d GetCenter() const =0;

  protected:
    unsigned int offset_x;  ///< offset in vector of state (position part)
    unsigned int offset_w;  ///< offset in vector of state (speed part)
    int m_tag = -1;
};

CH_CLASS_VERSION(ChNodeBase, 0)

}  // end namespace chrono

#endif
