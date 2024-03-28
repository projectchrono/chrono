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
//
// Demo for working with Eigen references.
// (Mostly for Chrono developers)
//
// =============================================================================

//#define EIGEN_NO_MALLOC

#include <iostream>
#include <vector>
#include "chrono/core/ChMatrix.h"

using namespace chrono;

// ------------------------------------------------------------------

class BaseClass {
  public:
    virtual ~BaseClass() {}
    // virtual ChMatrixConstRef GetRotMat() const = 0;
    virtual ChMatrixRef GetRotMat() = 0;
};

class DerivedClass1 : public BaseClass {
  public:
    DerivedClass1() {
        m_A.resize(2, 3);
        m_A.setRandom();
    }
    // virtual ChMatrixConstRef GetRotMat() const override { m_A; }
    virtual ChMatrixRef GetRotMat() override { return m_A; }

  private:
    ChMatrixDynamic<double> m_A;
};

class DerivedClass2 : public BaseClass {
  public:
    DerivedClass2() { m_A.setRandom(); }
    // virtual ChMatrixConstRef GetRotMat() const override { return m_A; }
    virtual ChMatrixRef GetRotMat() override { return m_A; }

  private:
    ChMatrixNM<double, 2, 3> m_A;
};

// ------------------------------------------------------------------

double inv_cond(ChMatrixConstRef a) {
    const Eigen::VectorXd sing_vals = a.jacobiSvd().singularValues();
    return sing_vals(sing_vals.size() - 1) / sing_vals(0);
}

// ------------------------------------------------------------------

int main(int argc, char* argv[]) {
#ifdef EIGEN_NO_MALLOC
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> B;

    B.setRandom();
    double c = inv_cond(B);  //// <=== triggers heap allocation
    std::cout << c << std::endl;

    DerivedClass2 c2;
    std::cout << c2.GetRotMat() << std::endl;
#else
    ChMatrixDynamic<double> A(2, 3);
    A.setRandom();
    double Acond = inv_cond(A);
    double ATcond = inv_cond(A.transpose());
    std::cout << Acond << " " << ATcond << std::endl;

    ChMatrixNM<double, 2, 3> B;
    B = A;
    double Bcond = inv_cond(B);
    double BTcond = inv_cond(B.transpose());
    std::cout << Bcond << " " << BTcond << std::endl;

    std::vector<BaseClass*> V;

    DerivedClass1 c1;
    V.push_back(&c1);
    std::cout << "\n" << c1.GetRotMat() << std::endl;
    c1.GetRotMat()(0, 0) = 1;

    DerivedClass2 c2;
    V.push_back(&c2);
    std::cout << "\n" << c2.GetRotMat() << std::endl;
    c2.GetRotMat()(0, 0) = 1;

    for (auto v : V) {
        std::cout << "\n" << v->GetRotMat() << std::endl;
    }
#endif

    return 0;
}
