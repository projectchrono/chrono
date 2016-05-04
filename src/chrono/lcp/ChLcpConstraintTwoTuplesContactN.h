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

#ifndef CHLCPCONSTRAINTTWOTUPLESCONTACTN_H
#define CHLCPCONSTRAINTTWOTUPLESCONTACTN_H


#include "ChLcpConstraintTwoTuplesFrictionT.h"

namespace chrono {


/// This is enough to use dynamic_casting<> to detect all template types
/// from ChLcpConstraintTwoTuplesContactN

class ChApi ChLcpConstraintTwoTuplesContactNall {

public:

    /// Get the friction coefficient
    double GetFrictionCoefficient() { return friction; }
    /// Set the friction coefficient
    void SetFrictionCoefficient(double mcoeff) { friction = mcoeff; }

    /// Get the cohesion
    double GetCohesion() { return cohesion; }
    /// Set the cohesion
    void SetCohesion(double mcoh) { cohesion = mcoh; }

protected:
    /// the friction coefficient 'f', for  sqrt(Tx^2+Ty^2)<f*Nz
    double friction;
    /// the cohesion 'c', positive, if any, for  sqrt(Tx^2+Ty^2)<f*(Nz+c)
    double cohesion;

};

///  This class is inherited by the ChLcpConstraintTwoTuples(),
/// It is used to represent the normal reaction between two objects,
/// each represented by a tuple of ChVariables objects,
/// ONLY when also two ChLcpConstraintTwoTuplesFrictionT objects are
/// used to represent friction. (If these two tangent constraint
/// are not used, for frictionless case, please use a simple ChConstraintTwo
/// with the CONSTRAINT_UNILATERAL mode.)
/// Differently from an unilateral constraint, this does not enforce
/// projection on positive constraint, since it will be up to the 'companion'
/// ChLcpConstraintTwoTuplesFrictionT objects to call a projection on the cone, by
/// modifying all the three components (normal, u, v) at once.
/// Templates Ta and Tb are of ChLcpVariableTupleCarrier_Nvars classes

template <class Ta, class Tb>
class ChApi ChLcpConstraintTwoTuplesContactN : 
            public ChLcpConstraintTwoTuples<Ta,Tb>, 
            public ChLcpConstraintTwoTuplesContactNall {

    //
    // DATA
    //

  protected:

    /// the pointer to U tangential component
    ChLcpConstraintTwoTuplesFrictionT< Ta, Tb >* constraint_U;
    /// the pointer to V tangential component
    ChLcpConstraintTwoTuplesFrictionT< Ta, Tb >* constraint_V;

  public:
    //
    // CONSTRUCTORS
    //
    /// Default constructor
    ChLcpConstraintTwoTuplesContactN() {
        this->mode = CONSTRAINT_FRIC;
        friction = 0.0;
        cohesion = 0.0;
        constraint_U = constraint_V = 0;
    };


    /// Copy constructor
    ChLcpConstraintTwoTuplesContactN(const ChLcpConstraintTwoTuplesContactN& other) : ChLcpConstraintTwoTuples< Ta, Tb >(other) {
        friction = other.friction;
        cohesion = other.cohesion;
        constraint_U = other.constraint_U;
        constraint_V = other.constraint_V;
    }

    virtual ~ChLcpConstraintTwoTuplesContactN(){};

    virtual ChLcpConstraintTwoTuplesContactN* new_Duplicate() override { return new ChLcpConstraintTwoTuplesContactN(*this); };

    /// Assignment operator: copy from other object
    ChLcpConstraintTwoTuplesContactN& operator=(const ChLcpConstraintTwoTuplesContactN& other) {
        if (&other == this)
            return *this;
        // copy parent class data
        ChLcpConstraintTwoTuples< Ta, Tb >::operator=(other);

        friction = other.friction;
        cohesion = other.cohesion;
        constraint_U = other.constraint_U;
        constraint_V = other.constraint_V;
        return *this;
    }

    //
    // FUNCTIONS
    //


    /// Get pointer to U tangential component
    ChLcpConstraintTwoTuplesFrictionT< Ta, Tb >* GetTangentialConstraintU() { return constraint_U; }
    /// Get pointer to V tangential component
    ChLcpConstraintTwoTuplesFrictionT< Ta, Tb >* GetTangentialConstraintV() { return constraint_V; }

    /// Set pointer to U tangential component
    void SetTangentialConstraintU(ChLcpConstraintTwoTuplesFrictionT< Ta, Tb >* mconstr) { constraint_U = mconstr; }
    /// Set pointer to V tangential component
    void SetTangentialConstraintV(ChLcpConstraintTwoTuplesFrictionT< Ta, Tb >* mconstr) { constraint_V = mconstr; }

    /// For iterative solvers: project the value of a possible
    /// 'l_i' value of constraint reaction onto admissible set.
    /// This projection will also modify the l_i values of the two
    /// tangential friction constraints (projection onto the friction cone,
    /// as by Anitescu-Tasora theory).
    virtual void Project() override {
            if (!constraint_U)
                return;

            if (!constraint_V)
                return;

            // METHOD
            // Anitescu-Tasora projection on cone generator and polar cone
            // (contractive, but performs correction on three components: normal,u,v)

            double f_n = this->l_i + this->cohesion;
            double f_u = constraint_U->Get_l_i();
            double f_v = constraint_V->Get_l_i();
            ;
            double f_tang = sqrt(f_v * f_v + f_u * f_u);

            // shortcut
            if (!friction) {
                constraint_U->Set_l_i(0);
                constraint_V->Set_l_i(0);
                if (f_n < 0)
                    this->Set_l_i(0);
                return;
            }

            // inside upper cone? keep untouched!
            if (f_tang < friction * f_n)
                return;

            // inside lower cone? reset  normal,u,v to zero!
            if ((f_tang < -(1.0 / friction) * f_n) || (fabs(f_n) < 10e-15)) {
                double f_n_proj = 0;
                double f_u_proj = 0;
                double f_v_proj = 0;

                this->Set_l_i(f_n_proj);
                constraint_U->Set_l_i(f_u_proj);
                constraint_V->Set_l_i(f_v_proj);

                return;
            }

            // remaining case: project orthogonally to generator segment of upper cone
            double f_n_proj = (f_tang * friction + f_n) / (friction * friction + 1);
            double f_tang_proj = f_n_proj * friction;
            double tproj_div_t = f_tang_proj / f_tang;
            double f_u_proj = tproj_div_t * f_u;
            double f_v_proj = tproj_div_t * f_v;

            this->Set_l_i(f_n_proj - this->cohesion);
            constraint_U->Set_l_i(f_u_proj);
            constraint_V->Set_l_i(f_v_proj);
        }


};

}  // END_OF_NAMESPACE____

#endif
