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

//// RADU
//// Obsolete this?

#ifndef CHFX_H
#define CHFX_H

#include "chrono/core/ChMath.h"

namespace chrono {

/// Abstract interface class for math functions of the type A=f(B)
/// where A and B are vectors of real values (to the limit, also
/// single scalar values).
class ChFx {
  public:
    /// Evaluate A=f(B).
    virtual void Eval(ChVectorDynamic<>& A,       ///< result output variables here
                      const ChVectorDynamic<>& B  ///< input variables here
                      ) = 0;
};

/// Class for A=f(B) math functions, where the function
/// is defined by wrapping a C function of the 'old' type,
/// i.e. a pointer to simple C call that works over arrays of doubles and return one value:
///   double (*func)(double p[], void* my_data);
/// as often used in C/C++ math libraries.
class ChFxCfunctionS : public ChFx {
  public:
    ChFxCfunctionS(double (*mfunc)(const double* p, void* my_data),  ///< the pointer to the C function
                   void* mdata = nullptr                             ///< user data, passed to C function.
                   )
        : func(mfunc), user_data(mdata) {}

    virtual void Eval(ChVectorDynamic<>& A, const ChVectorDynamic<>& B) {
        A(0) = this->func(B.data(), this->user_data);
    }

  private:
    void* user_data;
    double (*func)(const double* p, void* my_data);
};

/// Class for A=f(B) math functions, where the function
/// is defined by wrapping a C function of the 'old' type,
/// i.e. a pointer to simple C call that works over arrays of values and return array of values:
///   double (*func)(double p[], void* my_data);
/// as often used in C/C++ math libraries.

class ChFxCfunction : public ChFx {
  public:
    /// Create the function wrapper.
    ChFxCfunction(void (*mfunc)(const double* in, double* out, void* my_data),  ///< pointer to the C function
                  int moutvars,          ///< number of output vars (size of out[])
                  void* mdata = nullptr  ///< user data, passed C function.
                  )
        : func(mfunc), n_outvars(moutvars), user_data(mdata) {
        outvars = new double[n_outvars];
    }
    virtual ~ChFxCfunction() { delete[] outvars; }

    virtual void Eval(ChVectorDynamic<>& A, const ChVectorDynamic<>& B) {
        this->func(B.data(), this->outvars, this->user_data);
        for (int j = 0; j < A.size(); j++)
            A(j) = outvars[j];
    }

  private:
    void* user_data;
    int n_outvars;
    double* outvars;
    void (*func)(const double* in, double* out, void* my_data);
};

}  // end namespace chrono

#endif
