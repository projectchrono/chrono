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
// (2nd derivatives of the poosition vector)
// =============================================================================

#ifndef CHNODEFEACURV_H
#define CHNODEFEACURV_H

#include "chrono/solver/ChVariablesGenericDiagonalMass.h"
#include "chrono_fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// Generic finite element node with 9 degrees of freedom representing curvature.
class ChApiFea ChNodeFEAcurv : public ChNodeFEAbase {
  public:
    ChNodeFEAcurv(const ChVector<>& rxx = VNULL,  ///< initial value of xx 2nd derivative of position vector
                  const ChVector<>& ryy = VNULL,  ///< initial value of yy 2nd derivative of position vector
                  const ChVector<>& rzz = VNULL   ///< initial value of zz 2nd derivative of position vector
                  )
        : m_rxx(rxx), m_ryy(ryy), m_rzz(rzz) {
        m_rxx_dt = VNULL;
        m_ryy_dt = VNULL;
        m_rzz_dt = VNULL;
        m_rxx_dtdt = VNULL;
        m_ryy_dtdt = VNULL;
        m_rzz_dtdt = VNULL;
        m_variables = new ChVariablesGenericDiagonalMass(9);
        m_variables->GetMassDiagonal().FillElem(0);
    }

    ~ChNodeFEAcurv() { delete m_variables; }

    ChNodeFEAcurv(const ChNodeFEAcurv& other) : ChNodeFEAbase(other) {
        m_variables = new ChVariablesGenericDiagonalMass(9);
        *m_variables = *other.m_variables;
        m_rxx = other.m_rxx;
        m_ryy = other.m_ryy;
        m_rzz = other.m_rzz;
        m_rxx_dt = other.m_rxx_dt;
        m_ryy_dt = other.m_ryy_dt;
        m_rzz_dt = other.m_rzz_dt;
        m_rxx_dtdt = other.m_rxx_dtdt;
        m_ryy_dtdt = other.m_ryy_dtdt;
        m_rzz_dtdt = other.m_rzz_dtdt;
    }

    ChNodeFEAcurv& operator=(const ChNodeFEAcurv& other) {
        if (&other == this)
            return *this;

        ChNodeFEAbase::operator=(other);

        *m_variables = *other.m_variables;
        m_rxx = other.m_rxx;
        m_ryy = other.m_ryy;
        m_rzz = other.m_rzz;
        m_rxx_dt = other.m_rxx_dt;
        m_ryy_dt = other.m_ryy_dt;
        m_rzz_dt = other.m_rzz_dt;
        m_rxx_dtdt = other.m_rxx_dtdt;
        m_ryy_dtdt = other.m_ryy_dtdt;
        m_rzz_dtdt = other.m_rzz_dtdt;

        return *this;
    }

    /// Set the xx 2nd derivative of position vector.
    void SetCurvatureXX(const ChVector<>& rxx) { m_rxx = rxx; }
    /// Get the xx 2nd derivative of position vector.
    const ChVector<>& GetCurvatureXX() const { return m_rxx; }

    /// Set the yy 2nd derivative of position vector.
    void SetCurvatureYY(const ChVector<>& ryy) { m_ryy = ryy; }
    /// Get the yy 2nd derivative of position vector.
    const ChVector<>& GetCurvatureYY() const { return m_ryy; }

    /// Set the zz 2nd derivative of position vector.
    void SetCurvatureZZ(const ChVector<>& rzz) { m_rzz = rzz; }
    /// Get the zz 2nd derivative of position vector.
    const ChVector<>& GetCurvatureZZ() const { return m_rzz; }

    /// Set the time derivative of the xx 2nd derivative of position vector.
    void SetCurvatureXX_dt(const ChVector<>& rxx_dt) { m_rxx_dt = rxx_dt; }
    /// Get the time derivative of the xx 2nd derivative of position vector.
    const ChVector<>& GetCurvatureXX_dt() const { return m_rxx_dt; }

    /// Set the time derivative of the yy 2nd derivative of position vector.
    void SetCurvatureYY_dt(const ChVector<>& ryy_dt) { m_ryy_dt = ryy_dt; }
    /// Get the time derivative of the yy 2nd derivative of position vector.
    const ChVector<>& GetCurvatureYY_dt() const { return m_ryy_dt; }

    /// Set the time derivative of the zz 2nd derivative of position vector.
    void SetCurvatureZZ_dt(const ChVector<>& rzz_dt) { m_rzz_dt = rzz_dt; }
    /// Get the time derivative of the zz 2nd derivative of position vector.
    const ChVector<>& GetCurvatureZZ_dt() const { return m_rzz_dt; }

    /// Get mass of the node.
    //// TODO  is this even meaningful/needed for this type of node?
    ChVectorDynamic<>& GetMassDiagonal() { return m_variables->GetMassDiagonal(); }

    /// Set mass of the node.
    //// TODO  is this even meaningful/needed for this type of node?
    void SetMass(double mass) { m_variables->GetMassDiagonal().FillElem(mass); }

    ChVariables& Variables() { return *m_variables; }

    /// Reset the 2nd derivatives of position vector and their time derivatives.
    virtual void Relax() override {
        m_rxx = VNULL;
        m_ryy = VNULL;
        m_rzz = VNULL;
        m_rxx_dt = VNULL;
        m_ryy_dt = VNULL;
        m_rzz_dt = VNULL;
        m_rxx_dtdt = VNULL;
        m_ryy_dtdt = VNULL;
        m_rzz_dtdt = VNULL;
    }

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override {
        m_rxx_dt = VNULL;
        m_ryy_dt = VNULL;
        m_rzz_dt = VNULL;
        m_rxx_dtdt = VNULL;
        m_ryy_dtdt = VNULL;
        m_rzz_dtdt = VNULL;
    }

    /// Set the 'fixed' state of the node.
    /// If true, its current curvature values are not changed by solver.
    virtual void SetFixed(bool val) override {
        m_variables->SetDisabled(val);
    }

    /// Get the 'fixed' state of the node.
    virtual bool GetFixed() override {
        return m_variables->IsDisabled();
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
        x.PasteVector(m_rxx, off_x + 0, 0);
        x.PasteVector(m_ryy, off_x + 3, 0);
        x.PasteVector(m_rzz, off_x + 6, 0);
        v.PasteVector(m_rxx_dt, off_v + 0, 0);
        v.PasteVector(m_ryy_dt, off_v + 3, 0);
        v.PasteVector(m_rzz_dt, off_v + 6, 0);
    }

    virtual void NodeIntStateScatter(const unsigned int off_x,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const double T) override {
        m_rxx = x.ClipVector(off_x + 0, 0);
        m_ryy = x.ClipVector(off_x + 3, 0);
        m_rzz = x.ClipVector(off_x + 6, 0);
        m_rxx_dt = v.ClipVector(off_v + 0, 0);
        m_ryy_dt = v.ClipVector(off_v + 3, 0);
        m_rzz_dt = v.ClipVector(off_v + 6, 0);
    }

    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override {
        a.PasteVector(m_rxx_dtdt, off_a + 0, 0);
        a.PasteVector(m_ryy_dtdt, off_a + 3, 0);
        a.PasteVector(m_rzz_dtdt, off_a + 6, 0);
    }

    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override {
        m_rxx_dtdt = a.ClipVector(off_a + 0, 0);
        m_ryy_dtdt = a.ClipVector(off_a + 3, 0);
        m_rzz_dtdt = a.ClipVector(off_a + 6, 0);
    }

    virtual void NodeIntStateIncrement(const unsigned int off_x,
                                       ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& Dv) override {
        for (int i = 0; i < 9; i++) {
            x_new(off_x + i) = x(off_x + i) + Dv(off_v + i);
        }
    }

    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override {
        //// TODO do we even need anything here? What would the forces be?
    }

    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) override {
        for (int i = 0; i < 9; i++) {
            R(off + i) += c * GetMassDiagonal()(i)* w(off + i);
        }
    }

    virtual void NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) override {
        m_variables->Get_qb().PasteClippedMatrix(&v, off_v, 0, 9, 1, 0, 0);
        m_variables->Get_fb().PasteClippedMatrix(&R, off_v, 0, 9, 1, 0, 0);
    }

    virtual void NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) override {
        v.PasteMatrix(&m_variables->Get_qb(), off_v, 0);
    }

    //
    // Functions for interfacing to the LCP solver
    //

    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override {
        mdescriptor.InsertVariables(m_variables);
    }

    virtual void VariablesFbReset() override {
        m_variables->Get_fb().FillElem(0);
    }

    virtual void VariablesFbLoadForces(double factor = 1) override {
        //// TODO do we even need anything here? What would the forces be?
    }

    virtual void VariablesQbLoadSpeed() override {
        m_variables->Get_qb().PasteVector(m_rxx_dt, 0, 0);
        m_variables->Get_qb().PasteVector(m_ryy_dt, 3, 0);
        m_variables->Get_qb().PasteVector(m_rzz_dt, 6, 0);
    }

    virtual void VariablesQbSetSpeed(double step = 0) override {
        ChVector<> old_rxx_dt = m_rxx_dt;
        ChVector<> old_ryy_dt = m_ryy_dt;
        ChVector<> old_rzz_dt = m_rzz_dt;

        m_rxx_dt = m_variables->Get_qb().ClipVector(0, 0);
        m_ryy_dt = m_variables->Get_qb().ClipVector(3, 0);
        m_rzz_dt = m_variables->Get_qb().ClipVector(6, 0);

        if (step) {
            m_rxx_dtdt = (m_rxx_dt - old_rxx_dt) / step;
            m_ryy_dtdt = (m_ryy_dt - old_ryy_dt) / step;
            m_rzz_dtdt = (m_rzz_dt - old_rzz_dt) / step;
        }
    }

    virtual void VariablesFbIncrementMq() override {
        m_variables->Compute_inc_Mb_v(m_variables->Get_fb(), m_variables->Get_qb());
    }

    virtual void VariablesQbIncrementPosition(double step) override {
        ChVector<> new_rxx_dt = m_variables->Get_qb().ClipVector(0, 0);
        ChVector<> new_ryy_dt = m_variables->Get_qb().ClipVector(3, 0);
        ChVector<> new_rzz_dt = m_variables->Get_qb().ClipVector(6, 0);
        m_rxx = m_rxx + new_rxx_dt * step;
        m_ryy = m_ryy + new_ryy_dt * step;
        m_rzz = m_rzz + new_rzz_dt * step;
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
        marchive << CHNVP(m_rxx);
        marchive << CHNVP(m_ryy);
        marchive << CHNVP(m_rzz);
        marchive << CHNVP(m_rxx_dt);
        marchive << CHNVP(m_ryy_dt);
        marchive << CHNVP(m_rzz_dt);
        marchive << CHNVP(m_rxx_dtdt);
        marchive << CHNVP(m_ryy_dtdt);
        marchive << CHNVP(m_rzz_dtdt);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChNodeFEAbase::ArchiveIN(marchive);

        // stream in all member data:
        marchive >> CHNVP(m_rxx);
        marchive >> CHNVP(m_ryy);
        marchive >> CHNVP(m_rzz);
        marchive >> CHNVP(m_rxx_dt);
        marchive >> CHNVP(m_ryy_dt);
        marchive >> CHNVP(m_rzz_dt);
        marchive >> CHNVP(m_rxx_dtdt);
        marchive >> CHNVP(m_ryy_dtdt);
        marchive >> CHNVP(m_rzz_dtdt);
    }

  private:
    ChVariablesGenericDiagonalMass* m_variables;

    ChVector<> m_rxx;
    ChVector<> m_ryy;
    ChVector<> m_rzz;

    ChVector<> m_rxx_dt;
    ChVector<> m_ryy_dt;
    ChVector<> m_rzz_dt;

    //// TODO do we really need these?
    ChVector<> m_rxx_dtdt;
    ChVector<> m_ryy_dtdt;
    ChVector<> m_rzz_dtdt;
};

}  // end namespace fea
}  // end namespace chrono

#endif
