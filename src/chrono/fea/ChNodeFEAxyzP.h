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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHNODEFEAXYZP_H
#define CHNODEFEAXYZP_H

#include "chrono/solver/ChVariablesGeneric.h"
#include "chrono/fea/ChNodeFEAbase.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_nodes
/// @{

/// Class for a generic finite element node in 3D space, with scalar field P.
/// This can be used for typical Poisson-type problems (ex. thermal, if the
/// scalar field is temperature T, or electrostatics if the scalar field is
/// electric potential V)
class ChApi ChNodeFEAxyzP : public ChNodeFEAbase {
  public:
    ChNodeFEAxyzP(ChVector<> initial_pos = VNULL);
    ChNodeFEAxyzP(const ChNodeFEAxyzP& other);
    ~ChNodeFEAxyzP() {}

    ChNodeFEAxyzP& operator=(const ChNodeFEAxyzP& other);

    virtual ChVariables& Variables() { return variables; }

    virtual void Relax() override;

    /// Reset to no speed and acceleration.
    virtual void SetNoSpeedNoAcceleration() override { P_dt = 0; }

    /// Fix/release this node.
    /// If fixed, its state variables are not changed by the solver.
    virtual void SetFixed(bool fixed) override;

    /// Return true if the node is fixed (i.e., its state variables are not changed by the solver).
    virtual bool IsFixed() const override;

    /// Position of the node - in absolute csys.
    const ChVector<>& GetPos() const { return pos; }
    /// Position of the node - in absolute csys.
    void SetPos(const ChVector<>& mpos) { pos = mpos; }

    /// Set the scalar field at node.
    void SetP(double mp) { P = mp; }
    /// Get the scalar field at node.
    double GetP() const { return P; }

    /// Set the scalar field time derivative at node.
    void SetP_dt(double mp) { P_dt = mp; }
    /// Get the scalar field time derivative at node.
    double GetP_dt() const { return P_dt; }

    /// Set the applied term (right hand term in Poisson formulations).
    void SetF(double mf) { F = mf; }
    /// Get the applied term (right hand term in Poisson formulations).
    double GetF() const { return F; }

    /// Get mass of the node. Not meaningful except for transients.
    /// Meaning of 'mass' changes depending on the problem type.
    double GetMass() { return variables.GetMass()(0); }
    /// Set mass of the node.  Not meaningful except for transients.
    /// Meaning of 'mass' changes depending on the problem type.
    void SetMass(double mm) { variables.GetMass()(0) = mm; }

    /// Get the number of degrees of freedom
    virtual int GetNdofX() const override { return 1; }

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

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    ChVariablesGeneric variables;  /// solver proxy: variable with scalar field P
    double P;                      ///< field
    double P_dt;                   ///< field derivative, if needed
    double F;                      ///< applied term
    ChVector<> pos;
};

/// @} fea_nodes

}  // end namespace fea
}  // end namespace chrono

#endif
