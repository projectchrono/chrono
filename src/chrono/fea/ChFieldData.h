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
// Authors: Alessandro Tasora 
// =============================================================================

#ifndef CHFIELDDATA_H
#define CHFIELDDATA_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/solver/ChVariablesGeneric.h"
#include "chrono/solver/ChVariablesGenericDiagonalMass.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/solver/ChVariablesGeneric.h"
#include "chrono/solver/ChVariablesNode.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Base interface for the per-node and per-materialpoint properties of some material,
/// for example it can contain some type of ChVariable for integrable state. 
/// Used, among others, in ChField and ChDomain.
/// Suggestion: if you want to inherit from this, rather consider to inherit from 
/// these ready-to-use helping subclasses: ChFieldDataNONE (if you just want to attach some 
/// generic data structures) or ChFieldDataGeneric<int n> (if you want a n-dimensional 
/// state, with its ChVariable, with automatic bookkeeping etc.)

class ChFieldData {
public:
    // Access state at node
    virtual ChVectorRef State() = 0;

    // Access state time derivative at node
    virtual ChVectorRef StateDt() = 0;

    /// Access the applied load term (ex. the atomic load/source in a Poisson equation).
    virtual ChVectorRef Load() = 0;

    /// Access the contained ChVariable. 
    virtual ChVariables& GetVariable() = 0;

    /// Fix/release this state.
    /// If fixed, its state variables are not changed by the solver.
    virtual void SetFixed(bool fixed) = 0;
    virtual bool IsFixed() const = 0;

    /// Get the number of degrees of freedom of this state.
    virtual unsigned int GetNumCoordsPosLevel() { return 0; }
    virtual unsigned int GetNumCoordsVelLevel() { return 0; }
    static unsigned int StaticGetNumCoordsPosLevel() { return 0; }
    static unsigned int StaticGetNumCoordsVelLevel() { return 0; }

    unsigned int DataGetOffsetPosLevel() { return offset_x; }
    unsigned int DataGetOffsetVelLevel() { return offset_w; }
    void DataSetOffsetPosLevel(const unsigned int moff) { offset_x = moff; }
    void DataSetOffsetVelLevel(const unsigned int moff) { offset_w = moff; }

    virtual void DataIntStateGather(const unsigned int off_x,
        ChState& x,
        const unsigned int off_v,
        ChStateDelta& v,
        double& T) {}
    virtual void DataIntStateScatter(const unsigned int off_x,
        const ChState& x,
        const unsigned int off_v,
        const ChStateDelta& v,
        const double T) {}
    virtual void DataIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {}
    virtual void DataIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {}
    virtual void DataIntStateIncrement(const unsigned int off_x,
        ChState& x_new,
        const ChState& x,
        const unsigned int off_v,
        const ChStateDelta& Dv) {
        for (unsigned int i = 0; i < GetNumCoordsPosLevel(); ++i) {
            x_new(off_x + i) = x(off_x + i) + Dv(off_v + i);
        }
    }
    virtual void DataIntStateGetIncrement(const unsigned int off_x,
        const ChState& x_new,
        const ChState& x,
        const unsigned int off_v,
        ChStateDelta& Dv) {
        for (unsigned int i = 0; i < GetNumCoordsPosLevel(); ++i) {
            Dv(off_v + i) = x_new(off_x + i) - x(off_x + i);
        }
    }
    virtual void DataIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {}
    virtual void DataIntLoadResidual_Mv(const unsigned int off,
        ChVectorDynamic<>& R,
        const ChVectorDynamic<>& w,
        const double c) {}
    virtual void DataIntLoadLumpedMass_Md(const unsigned int off,
        ChVectorDynamic<>& Md,
        double& error,
        const double c) {};
    virtual void DataIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {}
    virtual void DataIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {}
    virtual void InjectVariables(ChSystemDescriptor& descriptor) {}

private:
    unsigned int offset_x;
    unsigned int offset_w;
};


//------------------------------------------------------------------------------------------


/// Class for data structures for the per-node and per-materialpoint properties
/// that do NOT have an integrable state. Inherit from this if you need
/// to attach some data like "price", "wage", etc. 
/// It has minimal memory cost because it does not contain State, StateDt etc.
/// Used, among others, in ChField and ChDomain.

class ChApi ChFieldDataNONE : public ChFieldData {
private: 
    static ChVariablesGeneric  dumb_variables;
    static ChVectorN<double, 0> dumb_state;
public:
    virtual ChVectorRef State() { assert(false); return dumb_state; }
    virtual ChVectorRef StateDt() { assert(false); return dumb_state; }
    virtual ChVectorRef Load() { assert(false); return dumb_state; }
    virtual ChVariables& GetVariable() { assert(false); return dumb_variables; };
    virtual void SetFixed(bool fixed) { assert(false); }
    virtual bool IsFixed() const { return true; }
};


//------------------------------------------------------------------------------------------


/// Class for data structures for per-node and per-materialpoint properties
/// that that contain an integrable state vector, of size T_nstates. 
/// It contains a ChVariableGeneric of size size_T.
/// Used, among others, in ChField and ChDomain.

template <int T_nstates>
class ChFieldDataGeneric : public ChFieldData {
public:
    ChFieldDataGeneric() :
        mvariables(T_nstates) {
        state.setZero();
        state_dt.setZero();
        F.setZero();
    }

    virtual ChVectorRef State() override { return state; }

    virtual ChVectorRef StateDt() override { return state_dt; }

    virtual ChVectorRef Load() override { return F; }
 
    virtual ChVariables& GetVariable() override { return mvariables; }

    virtual void SetFixed(bool fixed) override {
        mvariables.SetDisabled(fixed);
    }
    virtual bool IsFixed() const override {
        return mvariables.IsDisabled();
    }

    /// Get the number of degrees of freedom of the field.
    virtual unsigned int GetNumCoordsPosLevel() override { return T_nstates; }
    virtual unsigned int GetNumCoordsVelLevel() override { return T_nstates; }
    static unsigned int StaticGetNumCoordsPosLevel() { return T_nstates; }
    static unsigned int StaticGetNumCoordsVelLevel() { return T_nstates; }

    virtual void DataIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) override {
        x.segment(off_x, T_nstates) = state;
        v.segment(off_v, T_nstates) = state_dt;
    }

    virtual void DataIntStateScatter(const unsigned int off_x,
                                    const ChState& x,
                                    const unsigned int off_v,
                                    const ChStateDelta& v,
                                    const double T) override {
        state = x.segment(off_x, T_nstates);
        state_dt = v.segment(off_v, T_nstates);
    }

    virtual void DataIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override {}
    virtual void DataIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override {}

    virtual void DataIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override {
        R.segment(off, T_nstates) += c * F;
    }

    virtual void DataIntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override {
        //R(off) += c * mvariables.GetMass() * w(off);
    }

    virtual void DataIntLoadLumpedMass_Md(const unsigned int off,
                                    ChVectorDynamic<>& Md,
                                    double& error,
                                    const double c) override {
        //for (int i = 0; i < T_nstates; ++i) {
        //    Md(off + i) += c * mvariables.GetMass().(i, i);
        //}
        //error += std::abs(mvariables.GetMass().sum() - mvariables.GetMass().diagonal().sum());
    }

    virtual void DataIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) override {
        mvariables.State() = v.segment(off_v, T_nstates);
        mvariables.Force() = R.segment(off_v, T_nstates);
    }

    virtual void DataIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) override {
        v.segment(off_v, T_nstates) = mvariables.State();
    }

    virtual void InjectVariables(ChSystemDescriptor& descriptor) override {
        descriptor.InsertVariables(&mvariables);
    }

    
private:
    ChVariablesGeneric mvariables;
    ChVectorN<double, T_nstates> state;
    ChVectorN<double, T_nstates> state_dt;
    ChVectorN<double, T_nstates> F;
};


//------------------------------------------------------------------------------------------


/// Class for data structure for per-node and per-materialpoint properties
/// that contain an integrable vector state, namely position in 3D space. 
/// It contains a ChVariableGeneric of size size_T.
/// Used, among others, in ChField and ChDomain.

class ChFieldDataPos3D : public ChFieldData {
public:
    ChFieldDataPos3D()  {
        pos.setZero();
        pos_dt.setZero();
        pos_dtdt.setZero();
        F.setZero();
    }
    
    // Custom properties, helpers etc.

    virtual ChVector3d GetPos() const { return ChVector3d(this->pos); }
    virtual void SetPos(const ChVector3d mpos) { this->pos = mpos.eigen(); }

    virtual ChVector3d GetPosDt() const { return ChVector3d(this->pos_dt); }
    virtual void SetPosDt(const ChVector3d mposdt) { this->pos_dt = mposdt.eigen(); }

    virtual ChVector3d GetPosDt2() const { return ChVector3d(this->pos_dtdt); }
    virtual void SetPosDt2(const ChVector3d mposdtdt) { this->pos_dtdt = mposdtdt.eigen(); }

    virtual ChVector3d GetLoad() const { return ChVector3d(this->F); }
    virtual void SetLoad(const ChVector3d mF) { this->F = mF.eigen(); }

    // interface

    virtual ChVectorRef State() override { return pos; }

    virtual ChVectorRef StateDt() override { return pos_dt; }

    virtual ChVectorRef Load() override { return F; }

    virtual ChVariables& GetVariable() override { return mvariables; }

    virtual void SetFixed(bool fixed) override {
        mvariables.SetDisabled(fixed);
    }
    virtual bool IsFixed() const override {
        return mvariables.IsDisabled();
    }

    /// Get the number of degrees of freedom of the field.
    virtual unsigned int GetNumCoordsPosLevel() override { return 3; }
    virtual unsigned int GetNumCoordsVelLevel() override { return 3; }
    static unsigned int StaticGetNumCoordsPosLevel() { return 3; }
    static unsigned int StaticGetNumCoordsVelLevel() { return 3; }

    virtual void DataIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) {
        x.segment(off_x, 3) = pos;
        v.segment(off_v, 3) = pos_dt;
    }

    virtual void DataIntStateScatter(const unsigned int off_x,
                                    const ChState& x,
                                    const unsigned int off_v,
                                    const ChStateDelta& v,
                                    const double T) {
        pos = x.segment(off_x, 3);
        pos_dt = v.segment(off_v, 3);
    }

    virtual void DataIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
        a.segment(off_a, 3) = pos_dtdt;
    }
    virtual void DataIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
        pos_dtdt = a.segment(off_a, 3);
    }

    virtual void DataIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
        R.segment(off, 3) += c * F;
    }

    virtual void DataIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) {
        R.segment(off, 3) += c * mvariables.GetNodeMass() * w.segment(off, 3);
    }

    virtual void DataIntLoadLumpedMass_Md(const unsigned int off,
                                        ChVectorDynamic<>& Md,
                                        double& error,
                                        const double c) {
        for (int i = 0; i < 3; ++i) {
            Md(off + i) += c * mvariables.GetNodeMass();
        }
    }

    virtual void DataIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
        mvariables.State() = v.segment(off_v, 3);
        mvariables.Force() = R.segment(off_v, 3);
    }

    virtual void DataIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
        v.segment(off_v, 3) = mvariables.State();
    }

    virtual void InjectVariables(ChSystemDescriptor& descriptor) {
        descriptor.InsertVariables(&mvariables);
    }

    
private:
    ChVariablesNode mvariables;
    ChVectorN<double, 3> pos;
    ChVectorN<double, 3> pos_dt;
    ChVectorN<double, 3> pos_dtdt;
    ChVectorN<double, 3> F;
};

//--------------------------------------------------------------------------------

// Some ready-to-use data fields


class ChFieldDataScalar : public ChFieldDataGeneric<1> {};

class ChFieldDataVector : public ChFieldDataGeneric<3> {};

class ChFieldDataTemperature : public ChFieldDataScalar {
public:
    double& T() { return State()[0]; }
    double& T_dt() { return StateDt()[0]; }
};

class ChFieldDataElectricPotential : public ChFieldDataScalar {
public:
    double& V() { return State()[0]; }
    double& V_dt() { return StateDt()[0]; }
};



/// @} chrono_fea

}  // end namespace fea


}  // end namespace chrono

#endif
