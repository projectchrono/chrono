//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPCONSTRAINTTWOTUPLESROLLINGN_H
#define CHLCPCONSTRAINTTWOTUPLESROLLINGN_H

#include "ChLcpConstraintTwoTuplesRollingT.h"
#include "ChLcpConstraintTwoTuplesContactN.h"

namespace chrono {


/// This is enough to use dynamic_casting<> to detect all template types
/// from ChLcpConstraintTwoTuplesRollingN
class ChApi ChLcpConstraintTwoTuplesRollingNall {
};

///  This class is inherited by the ChLcpConstraintTwoTuples(),
/// It is used to represent the normal reaction between two objects,
/// each represented by a tuple of ChVariables objects,
/// ONLY when also two ChLcpConstraintTwoFrictionT objects are
/// used to represent friction. (If these two tangent constraint
/// are not used, for frictionless case, please use a simple ChConstraintTwo
/// with the CONSTRAINT_UNILATERAL mode.)
/// Differently from an unilateral constraint, this does not enforce
/// projection on positive constraint, since it will be up to the 'companion'
/// ChLcpConstraintTwoTuplesFrictionT objects to call a projection on the cone, by
/// modifying all the three components (normal, u, v) at once.
/// Templates Ta and Tb are of ChLcpVariableTupleCarrier_Nvars classes

template <class Ta, class Tb>
class ChApi ChLcpConstraintTwoTuplesRollingN : 
            public ChLcpConstraintTwoTuples<Ta,Tb>, 
            public ChLcpConstraintTwoTuplesRollingNall {

    //
    // DATA
    //

  protected:
    /// the rolling friction coefficient
    float rollingfriction;
    float spinningfriction;

    /// the pointer to U tangential component
    ChLcpConstraintTwoTuplesRollingT< Ta, Tb >* constraint_U;
    /// the pointer to V tangential component
    ChLcpConstraintTwoTuplesRollingT< Ta, Tb >* constraint_V;
    /// the pointer to N  component
    ChLcpConstraintTwoTuplesContactN< Ta, Tb >* constraint_N;

  public:
    //
    // CONSTRUCTORS
    //
    /// Default constructor
    ChLcpConstraintTwoTuplesRollingN() {
        this->mode = CONSTRAINT_FRIC;
        rollingfriction = 0.0;
        spinningfriction = 0.0;
        constraint_U = constraint_V = 0;
        constraint_N = 0;
    };


    /// Copy constructor
    ChLcpConstraintTwoTuplesRollingN(const ChLcpConstraintTwoTuplesRollingN& other) : ChLcpConstraintTwoTuples< Ta, Tb >(other) {
        rollingfriction = other.rollingfriction;
        spinningfriction = other.spinningfriction;
        constraint_U = other.constraint_U;
        constraint_V = other.constraint_V;
        constraint_N = other.constraint_N;
    }

    virtual ~ChLcpConstraintTwoTuplesRollingN(){};

    virtual ChLcpConstraintTwoTuplesRollingN* new_Duplicate() { return new ChLcpConstraintTwoTuplesRollingN(*this); };

    /// Assignment operator: copy from other object
    ChLcpConstraintTwoTuplesRollingN& operator=(const ChLcpConstraintTwoTuplesRollingN& other) {
        if (&other == this)
            return *this;
        // copy parent class data
        ChLcpConstraintTwoTuples< Ta, Tb >::operator=(other);

        rollingfriction = other.rollingfriction;
        spinningfriction = other.spinningfriction;
        constraint_U = other.constraint_U;
        constraint_V = other.constraint_V;
        constraint_N = other.constraint_N;
        return *this;
    }

    //
    // FUNCTIONS
    //

    /// Get the rolling friction coefficient
    float GetRollingFrictionCoefficient() { return rollingfriction; }
    /// Set the rolling friction coefficient
    void SetRollingFrictionCoefficient(float mcoeff) { rollingfriction = mcoeff; }

    /// Get the spinning friction coefficient
    float GetSpinningFrictionCoefficient() { return spinningfriction; }
    /// Set the spinning friction coefficient
    void SetSpinningFrictionCoefficient(float mcoeff) { spinningfriction = mcoeff; }

    /// Get pointer to U tangential component
    ChLcpConstraintTwoTuplesRollingT< Ta, Tb >* GetRollingConstraintU() { return constraint_U; }
    /// Get pointer to V tangential component
    ChLcpConstraintTwoTuplesRollingT< Ta, Tb >* GetRollingConstraintV() { return constraint_V; }
    /// Get pointer to normal contact component
    ChLcpConstraintTwoTuplesContactN< Ta, Tb >* GetNormalConstraint() { return constraint_N; }

    /// Set pointer to U tangential component
    void SetRollingConstraintU(ChLcpConstraintTwoTuplesRollingT< Ta, Tb >* mconstr) { constraint_U = mconstr; }
    /// Set pointer to V tangential component
    void SetRollingConstraintV(ChLcpConstraintTwoTuplesRollingT< Ta, Tb >* mconstr) { constraint_V = mconstr; }
    /// Set pointer to normal contact component
    void SetNormalConstraint(ChLcpConstraintTwoTuplesContactN< Ta, Tb >* mconstr) { constraint_N = mconstr; }


    /// For iterative solvers: project the value of a possible
    /// 'l_i' value of constraint reaction onto admissible set.
    /// This projection will also modify the l_i values of the two
    /// tangential friction constraints (projection onto the friction cone,
    /// as by Anitescu-Tasora theory).
    virtual void Project(){

            if (!constraint_U)
                return;

            if (!constraint_V)
                return;

            if (!constraint_N)
                return;

            // METHOD
            // Anitescu-Tasora projection on rolling-friction cone generator and polar cone
            // (contractive, but performs correction on three components: normal,u,v)

            double f_n = constraint_N->Get_l_i();
            double t_n = this->Get_l_i();
            double t_u = constraint_U->Get_l_i();
            double t_v = constraint_V->Get_l_i();
            double t_tang = sqrt(t_v * t_v + t_u * t_u);
            double t_sptang = fabs(t_n);  // = sqrt(t_n*t_n);

            // A Project the spinning friction (approximate - should do cone
            //   projection stuff as in B, but spinning friction is usually very low...)

            if (spinningfriction) {
                // inside upper cone? keep untouched!
                if (t_sptang < spinningfriction * f_n) {
                } else {
                    // inside lower cone? reset  normal,u,v to zero!
                    if ((t_sptang < -(1.0 / spinningfriction) * f_n) || (fabs(f_n) < 10e-15)) {
                        constraint_N->Set_l_i(0);
                        this->Set_l_i(0);
                    } else {
                        // remaining case: project orthogonally to generator segment of upper cone (CAN BE simplified)
                        double f_n_proj = (t_sptang * spinningfriction + f_n) / (spinningfriction * spinningfriction + 1);
                        double t_tang_proj = f_n_proj * spinningfriction;
                        double tproj_div_t = t_tang_proj / t_sptang;
                        double t_n_proj = tproj_div_t * t_n;

                        constraint_N->Set_l_i(f_n_proj);
                        this->Set_l_i(t_n_proj);
                    }
                }
            }

            // B Project the rolling friction

            // shortcut
            if (!rollingfriction) {
                constraint_U->Set_l_i(0);
                constraint_V->Set_l_i(0);
                if (f_n < 0)
                    constraint_N->Set_l_i(0);
                return;
            }

            // inside upper cone? keep untouched!
            if (t_tang < rollingfriction * f_n)
                return;

            // inside lower cone? reset  normal,u,v to zero!
            if ((t_tang < -(1.0 / rollingfriction) * f_n) || (fabs(f_n) < 10e-15)) {
                double f_n_proj = 0;
                double t_u_proj = 0;
                double t_v_proj = 0;

                constraint_N->Set_l_i(f_n_proj);
                constraint_U->Set_l_i(t_u_proj);
                constraint_V->Set_l_i(t_v_proj);

                return;
            }

            // remaining case: project orthogonally to generator segment of upper cone
            double f_n_proj = (t_tang * rollingfriction + f_n) / (rollingfriction * rollingfriction + 1);
            double t_tang_proj = f_n_proj * rollingfriction;
            double tproj_div_t = t_tang_proj / t_tang;
            double t_u_proj = tproj_div_t * t_u;
            double t_v_proj = tproj_div_t * t_v;

            constraint_N->Set_l_i(f_n_proj);
            constraint_U->Set_l_i(t_u_proj);
            constraint_V->Set_l_i(t_v_proj);
        }


};

}  // END_OF_NAMESPACE____

#endif
