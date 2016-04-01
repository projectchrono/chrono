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
// Authors: Radu Serban
// =============================================================================
// Generic finite element node with 9 degrees of freedom representing curvature
// =============================================================================

#ifndef CHNODEFEACURV_H
#define CHNODEFEACURV_H

#include "chrono/core/ChFrameMoving.h"
#include "chrono/lcp/ChLcpVariablesBodyOwnMass.h"
#include "chrono/physics/ChBodyFrame.h"
#include "chrono_fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// Generic finite element node with 9 degrees of freedom representing curvature.
class ChApiFea ChNodeFEAcurv : public ChNodeFEAbase {
  public:
    ChNodeFEAcurv(ChVector<> initial_pos = VNULL) {
        pos = initial_pos;
        //// TODO
    }

    ~ChNodeFEAcurv() {}

    ChNodeFEAcurv(const ChNodeFEAcurv& other) : ChNodeFEAbase(other) {
        this->pos = other.pos;
        //// TODO
    }

    ChNodeFEAcurv& operator=(const ChNodeFEAcurv& other) {
        if (&other == this)
            return *this;

        ChNodeFEAbase::operator=(other);

        this->pos = other.pos;
        //// TODO

        return *this;
    }

    //// TODO
    ////virtual ChLcpVariables& Variables() override { return this->variables; }

    /// Set the rest position as the actual position.
    virtual void Relax() override {
        //// TODO
    }

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override {
        //// TODO 
    }

    /// Set the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual void SetFixed(bool mev) override {
        //// TODO
    }
    /// Get the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual bool GetFixed() override {
        return false;
        //// TODO
    }

    /// Get mass of the node.
    virtual double GetMass() {
        return 0;
        //// TODO
    }

    /// Set mass of the node.
    virtual void SetMass(double mm) {
        //// TODO
    }

    /// Get the number of degrees of freedom.
    virtual int Get_ndof_x() override { return 9; }

    /// Get the number of degrees of freedom, derivative.
    virtual int Get_ndof_w() override { return 9; }

    //
    // Functions for interfacing to the state bookkeeping
    //

    virtual void NodeIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) override {
        //// TODO
    }

    virtual void NodeIntStateScatter(const unsigned int off_x,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const double T) override {
        //// TODO
    }

    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override {
        //// TODO
    }

    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override {
        //// TODO
    }

    virtual void NodeIntStateIncrement(const unsigned int off_x,
                                       ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& Dv) override {
        //// TODO
        //// Default base class ChNodeBase enough?
    }

    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) override {
        //// TODO
    }

    virtual void NodeIntToLCP(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) override {
        //// TODO
    }

    virtual void NodeIntFromLCP(const unsigned int off_v, ChStateDelta& v) override {
        //// TODO
    }

    //
    // Functions for interfacing to the LCP solver
    //

    virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor) override {
        //// TODO
    }

    virtual void VariablesFbReset() override {
        //// TODO
    }

    virtual void VariablesFbLoadForces(double factor = 1) override {
        //// TODO
    }

    virtual void VariablesQbLoadSpeed() override {
        //// TODO
    }

    virtual void VariablesQbSetSpeed(double step = 0) override {
        //// TODO
    }

    virtual void VariablesFbIncrementMq() override {
        //// TODO
    }

    virtual void VariablesQbIncrementPosition(double step) override {
        //// TODO
    }

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChNodeFEAbase::ArchiveOUT(marchive);

        // serialize all member data:
        marchive << CHNVP(pos);
        //// TODO
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChNodeFEAbase::ArchiveIN(marchive);

        // stream in all member data:
        marchive >> CHNVP(pos);
        //// TODO
    }

  private:
    ////ChLcpVariables variables;

    ChVector<> pos;
};

}  // end namespace fea
}  // end namespace chrono

#endif
