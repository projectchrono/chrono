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

#ifndef CHFX_H
#define CHFX_H

#include "chrono/core/ChMath.h"

namespace chrono {

/// Abstract interface class for math functions of the type A=f(B)
/// where A and B are vectors of real values (to the limit, also
/// single scalar values).

class ChFx {
  public:
    /// INTERFACE:
    /// Evaluate A=f(B)
    /// Pure virtual member: it _must_ be implemented by inherited classes.
    /// The parameters B and the results A are column-matrices (vectors of scalars).
    virtual void Eval(ChMatrix<>& A,       ///< result output variables here
                      const ChMatrix<>& B  ///< input variables here
                      ) = 0;
};

/// Class for A=f(B) math functions, where the function
/// is defined by wrapping a C function of the 'old' type,
/// i.e. a pointer to simple C call that works over arrays of doubles and return one value:
///   double (*func)(double p[], void* my_data);
/// as often used in C/C++ math libraries.

class ChFxCfunctionS : public ChFx {
  public:
    /// Create the function wrapper.
    ChFxCfunctionS(double (*mfunc)(double p[], void* my_data),  ///< the pointer to the C function
                   int minvars,                                 ///< the number of input vars (the size of double p[])
                   void* mdata = 0  ///< generic user data, if needed as my_data for C function.
                   ) {
        func = mfunc;
        n_invars = minvars;
        user_data = mdata;
        invars = new double[n_invars];
    }
    virtual ~ChFxCfunctionS() { delete[] invars; }

    /// INTERFACE:
    /// Evaluate A=f(B)
    virtual void Eval(ChMatrix<>& A,       ///< result  here
                      const ChMatrix<>& B  ///< input here
                      ) {
        for (int i = 0; i < A.GetRows(); i++)
            invars[i] = B(i, 0);
        double out = this->func(this->invars, this->user_data);
        A(0, 0) = out;
    }

  private:
    void* user_data;
    int n_invars;
    double* invars;
    double (*func)(double p[], void* my_data);
};

/// Class for A=f(B) math functions, where the function
/// is defined by wrapping a C function of the 'old' type,
/// i.e. a pointer to simple C call that works over arrays of values and return array of values:
///   double (*func)(double p[], void* my_data);
/// as often used in C/C++ math libraries.

class ChFxCfunction : public ChFx {
  public:
    /// Create the function wrapper.
    ChFxCfunction(void (*mfunc)(double in[], double ou[], void* my_data),  ///< the pointer to the C function
                  int minvars,     ///< the number of input vars (the size of double in[])
                  int moutvars,    ///< the number of output vars (the size of double ou[])
                  void* mdata = 0  ///< generic user data, if needed as my_data for C function.
                  ) {
        func = mfunc;
        n_invars = minvars;
        n_outvars = moutvars;
        user_data = mdata;
        invars = new double[n_invars];
        outvars = new double[n_outvars];
    }
    virtual ~ChFxCfunction() {
        delete[] invars;
        delete[] outvars;
    }

    /// INTERFACE:
    /// Evaluate A=f(B)
    virtual void Eval(ChMatrix<>& A,       ///< results here
                      const ChMatrix<>& B  ///< input here
                      ) {
        for (int i = 0; i < A.GetRows(); i++)
            invars[i] = B(i, 0);
        this->func(this->invars, this->outvars, this->user_data);
        for (int j = 0; j < B.GetRows(); j++)
            A(j, 0) = outvars[j];
    }

  private:
    void* user_data;
    int n_invars;
    int n_outvars;
    double* invars;
    double* outvars;
    void (*func)(double in[], double out[], void* my_data);
};

}  // end namespace chrono

#endif
