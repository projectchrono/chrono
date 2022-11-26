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
// Authors: Radu Serban
// =============================================================================
// Generic finite element node with 9 degrees of freedom representing curvature
// (2nd derivatives of the position vector)
// =============================================================================

#ifndef CHNODEFEACURV_H
#define CHNODEFEACURV_H

#include "chrono/solver/ChVariablesGenericDiagonalMass.h"
#include "chrono/fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_nodes
/// @{

/// Generic finite element node with 9 degrees of freedom representing curvature.
class ChApi ChNodeFEAcurv : public ChNodeFEAbase {
  public:
    ChNodeFEAcurv(const ChVector<>& rxx = VNULL,  ///< initial value of xx 2nd derivative of position vector
                  const ChVector<>& ryy = VNULL,  ///< initial value of yy 2nd derivative of position vector
                  const ChVector<>& rzz = VNULL   ///< initial value of zz 2nd derivative of position vector
                  );
    ChNodeFEAcurv(const ChNodeFEAcurv& other);
    ~ChNodeFEAcurv();

    ChNodeFEAcurv& operator=(const ChNodeFEAcurv& other);

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
    void SetMass(double mass) { m_variables->GetMassDiagonal().setConstant(mass); }

    ChVariables& Variables() { return *m_variables; }

    /// Reset the 2nd derivatives of position vector and their time derivatives.
    virtual void Relax() override;

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override;

    /// Fix/release this node.
    /// If fixed, its state variables are not changed by the solver.
    virtual void SetFixed(bool fixed) override;

    /// Return true if the node is fixed (i.e., its state variables are not changed by the solver).
    virtual bool IsFixed() const override;

    /// Get the number of degrees of freedom.
    virtual int GetNdofX() const override { return 9; }

    /// Get the number of degrees of freedom, derivative.
    virtual int GetNdofW() const override { return 9; }

    // Functions for interfacing to the state bookkeeping

    virtual void NodeIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) override;
    virtual void NodeIntStateScatter(const unsigned int off_x,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const double T) override;
    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void NodeIntStateIncrement(const unsigned int off_x,
                                       ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& Dv) override;
    virtual void NodeIntStateGetIncrement(const unsigned int off_x,
                                       const ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       ChStateDelta& Dv) override;
    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) override;
    virtual void NodeIntToDescriptor(const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R) override;
    virtual void NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) override;

    // Functions for interfacing to the solver

    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbIncrementPosition(double step) override;

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

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

/// @} fea_nodes

}  // end namespace fea
}  // end namespace chrono

#endif
