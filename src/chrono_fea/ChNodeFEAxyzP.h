//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File authors: Alessandro Tasora

#ifndef CHNODEFEAXYZP_H
#define CHNODEFEAXYZP_H

#include "chrono/lcp/ChLcpVariablesGeneric.h"
#include "chrono_fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// Class for a generic finite element node
/// in 3D space, with scalar field P. This can be used for
/// typical Poisson-type problems (ex. thermal, if the scalar field is temperature T,
/// or electrostatics if the scalar field is electric potential V)
class ChApiFea ChNodeFEAxyzP : public ChNodeFEAbase {
  public:
    ChNodeFEAxyzP(ChVector<> initial_pos = VNULL) {
        pos = initial_pos;
        P = 0;
        P_dt = 0;
        F = 0;
        variables.GetMass()(0) = 0.0;
    }

    ~ChNodeFEAxyzP() {}

    ChNodeFEAxyzP(const ChNodeFEAxyzP& other) : ChNodeFEAbase(other) {
        this->pos = other.pos;
        this->P = other.P;
        this->P_dt = other.P_dt;
        this->F = other.F;
        this->variables = other.variables;
    }

    ChNodeFEAxyzP& operator=(const ChNodeFEAxyzP& other) {
        if (&other == this)
            return *this;

        ChNodeFEAbase::operator=(other);

        this->pos = other.pos;
        this->P = other.P;
        this->P_dt = other.P_dt;
        this->F = other.F;
        this->variables = other.variables;
        return *this;
    }

    virtual ChLcpVariables& Variables() { return this->variables; }

    virtual void Relax() override {
        // no special effect here, just resets scalar field.
        this->P = 0;
        this->P_dt = 0;
    }

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override { this->P_dt = 0; }

    /// Set the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual void SetFixed(bool mev) override { variables.SetDisabled(mev); }
    /// Get the 'fixed' state of the node.
    /// If true, its current field value is not changed by solver.
    virtual bool GetFixed() override { return variables.IsDisabled(); }

    /// Position of the node - in absolute csys.
    ChVector<> GetPos() { return pos; }
    /// Position of the node - in absolute csys.
    void SetPos(const ChVector<>& mpos) { pos = mpos; }

    /// Set the scalar field at node.
    virtual void SetP(double mp) { P = mp; }
    /// Get the scalar field at node.
    virtual double GetP() { return P; }

    /// Set the scalar field time derivative at node.
    virtual void SetP_dt(double mp) { P_dt = mp; }
    /// Get the scalar field time derivative at node.
    virtual double GetP_dt() { return P_dt; }

    /// Set the applied term (right hand term in Poisson formulations).
    virtual void SetF(double mf) { F = mf; }
    /// Get the applied term (right hand term in Poisson formulations).
    virtual double GetF() { return F; }

    /// Get mass of the node. Not meaningful except for transients.
    /// Meaning of 'mass' changes depending on the problem type.
    virtual double GetMass() { return this->variables.GetMass()(0); }
    /// Set mass of the node.  Not meaningful except for transients.
    /// Meaning of 'mass' changes depending on the problem type.
    virtual void SetMass(double mm) { this->variables.GetMass()(0) = mm; }

    /// Get the number of degrees of freedom
    virtual int Get_ndof_x() override { return 1; }

    //
    // Functions for interfacing to the state bookkeeping
    //
    virtual void NodeIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) override {
        x(off_x) = this->P;
        v(off_v) = this->P_dt;
    }

    virtual void NodeIntStateScatter(const unsigned int off_x,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const double T) override {
        this->P = x(off_x);
        this->P_dt = v(off_v);
    }

    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override {
        // a(off_a) = this->P_dtdt; // NOT NEEDED?
    }

    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override {
        // this->P_dtdt = (a(off_a)); // NOT NEEDED?
    }

    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override {
        R(off) += this->F * c;
    }

    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) override {
        R(off) += c * this->GetMass() * w(off);
    }

    virtual void NodeIntToLCP(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) override {
        this->variables.Get_qb().PasteClippedMatrix(&v, off_v, 0, 1, 1, 0, 0);
        this->variables.Get_fb().PasteClippedMatrix(&R, off_v, 0, 1, 1, 0, 0);
    }

    virtual void NodeIntFromLCP(const unsigned int off_v, ChStateDelta& v) override {
        v.PasteMatrix(&this->variables.Get_qb(), off_v, 0);
    }

    //
    // Functions for interfacing to the LCP solver
    //

    virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor) override {
        mdescriptor.InsertVariables(&this->variables);
    }

    virtual void VariablesFbReset() override { this->variables.Get_fb().FillElem(0.0); }

    virtual void VariablesFbLoadForces(double factor = 1) override {
        if (variables.IsDisabled())
            return;
        this->variables.Get_fb().ElementN(0) += this->F * factor;
    }

    virtual void VariablesQbLoadSpeed() override {
        if (variables.IsDisabled())
            return;
        // not really a 'speed', just the field derivative (may be used in incremental solver)
        this->variables.Get_qb().SetElement(0, 0, this->P_dt);
    }

    virtual void VariablesQbSetSpeed(double step = 0) override {
        if (variables.IsDisabled())
            return;
        // not really a 'speed', just the field derivative (may be used in incremental solver)
        this->P_dt = this->variables.Get_qb().GetElement(0, 0);
    }

    virtual void VariablesFbIncrementMq() override {
        if (variables.IsDisabled())
            return;
        this->variables.Compute_inc_Mb_v(this->variables.Get_fb(), this->variables.Get_qb());
    }

    virtual void VariablesQbIncrementPosition(double step) override {
        if (variables.IsDisabled())
            return;

        double pseudospeed = variables.Get_qb().GetElement(0, 0);

        // ADVANCE FIELD: pos' = pos + dt * vel
        this->P = this->P + pseudospeed * step;
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
        marchive << CHNVP(P);
        marchive << CHNVP(P_dt);
        marchive << CHNVP(F);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChNodeFEAbase::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(P);
        marchive >> CHNVP(P_dt);
        marchive >> CHNVP(F);
    }

  private:
    ChLcpVariablesGeneric variables;  /// solver proxy: variable with scalar field P

    double P;     ///< field
    double P_dt;  ///< field derivative, if needed

    double F;  ///< applied term

    ChVector<> pos;
};

}  // end namespace fea
}  // end namespace chrono

#endif
