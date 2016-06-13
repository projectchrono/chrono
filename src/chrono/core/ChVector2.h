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
//
// Definition of a 2-D vector.
//
// =============================================================================

#ifndef CHVECTOR2_H
#define CHVECTOR2_H

#include <cmath>
#include <iomanip>
#include <iostream>

#include "chrono/core/ChMathematics.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Definition of a general purpose 2d vector.
/// ChVector2 is templated by precision, with default 'double'.
template <class Real = double>
class ChVector2 {
  public:
    Real x;
    Real y;

    ChVector2() : x(0), y(0) {}
    ChVector2(Real nx, Real ny) : x(nx), y(ny) {}
    ChVector2(Real val) : x(val), y(val) {}

    ChVector2(const ChVector2<Real>& other) : x(other.x), y(other.y) {}

    template <class RealB>
    ChVector2(const ChVector2<RealB>& other) : x((Real)other.x), y((Real)other.y) {}

    /// Assignment operator: copy from another vector
    ChVector2<Real>& operator=(const ChVector2<Real>& other) {
        if (&other == this)
            return *this;
        x = other.x;
        y = other.y;
        return *this;
    }
    template <class RealB>
    ChVector2<Real>& operator=(const ChVector2<RealB>& other) {
        x = (Real)other.x;
        y = (Real)other.y;
        return *this;
    }

    /// Operator for sign change
    ChVector2<Real> operator-() const { return ChVector2<Real>(-x, -y); }

    /// Operator for indexed access (0=x, 1=y).
    inline Real& operator()(int el) {
        assert(el == 0 || el == 1);
        return (el == 0) ? x : y;
    }
    inline const Real& operator()(int el) const {
        assert(el == 0 || el == 1);
        return (el == 0) ? x : y;
    }

    /// Operator for vector sum
    ChVector2<Real> operator+(const ChVector2<Real>& other) const {
        return ChVector2<Real>(x + other.x, y + other.y);
    }
    ChVector2<Real>& operator+=(const ChVector2<Real>& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    /// Operator for vector difference
    ChVector2<Real> operator-(const ChVector2<Real>& other) const { return ChVector2<Real>(x - other.x, y - other.y); }
    ChVector2<Real>& operator-=(const ChVector2<Real>& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    /// Operator for element-wise multiplication.
    ChVector2<Real> operator*(const ChVector2<Real>& other) const { return ChVector2<Real>(x * other.x, y * other.y); }
    ChVector2<Real>& operator*=(const ChVector2<Real>& other) {
        x *= other.x;
        y *= other.y;
        return *this;
    }

    /// Operator for scaling the vector by a scalar value.
    ChVector2<Real> operator*(Real s) const { return ChVector2<Real>(x * s, y * s); }
    ChVector2<Real>& operator*=(Real s) {
        x *= s;
        y *= s;
        return *this;
    }

    /// Operator for element-wise division.
    ChVector2<Real> operator/(const ChVector2<Real>& other) const { return ChVector2<Real>(x / other.x, y / other.y); }
    ChVector2<Real>& operator/=(const ChVector2<Real>& other) {
        x /= other.x;
        y /= other.y;
        return *this;
    }

    /// Operator for scaling the vector by inverse of a scalar value, as v/s
    ChVector2<Real> operator/(Real v) const { return ChVector2<Real>(x / v, y / v); }
    ChVector2<Real>& operator/=(const Real v) {
        x /= v;
        y /= v;
        return *this;
    }

    /// Operator for dot product.
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    double operator^(const ChVector2<Real>& other) const { return x * other.x + y * other.y; }

    /// Comparison operators
    bool operator<=(const ChVector2<Real>& other) const { return x <= other.x && y <= other.y; }
    bool operator>=(const ChVector2<Real>& other) const { return x >= other.x && y >= other.y; }
    bool operator<(const ChVector2<Real>& other) const { return x < other.x && y < other.y; }
    bool operator>(const ChVector2<Real>& other) const { return x > other.x && y > other.y; }

    /// Equality operators
    bool operator==(const ChVector2<Real>& other) const { return other.x == x && other.y == y; }
    bool operator!=(const ChVector2<Real>& other) const { return other.x != x || other.y != y; }

    //
    // FUNCTIONS
    //

    /// Sets the three values of the vector at once
    void Set(Real nx, Real ny) {
        x = nx;
        y = ny;
    }

    /// Sets the vector as a copy of another vector
    void Set(const ChVector2<Real>& p) {
        x = p.x;
        y = p.y;
    }

    /// Sets the vector with three components as a sample scalar
    void Set(Real p) {
        x = p;
        y = p;
    }

    /// Sets the vector as a null vector
    void SetNull() { x = y = 0; }

    /// Returns true if this vector is the null vector
    bool IsNull() const { return x == 0 && y == 0; }

    /// Returns true if vector is identical to other vector
    bool Equals(const ChVector2<Real>& other) const { return (other.x == x) && (other.y == y); }

    /// Returns true if vector equals another vector, within a tolerance 'tol'
    bool Equals(const ChVector2<Real>& other, Real tol) const {
        return (std::abs(other.x - x) < tol && std::abs(other.y - y) < tol);
    }

    /// The vector becomes the sum of the two vectors A and B:
    /// this=A+B
    void Add(const ChVector2<Real>& A, const ChVector2<Real>& B) {
        x = A.x + B.x;
        y = A.y + B.y;
    }

    /// The vector becomes the difference of the two vectors A and B:
    /// this=A-B
    void Sub(const ChVector2<Real>& A, const ChVector2<Real>& B) {
        x = A.x - B.x;
        y = A.y - B.y;
    }

    /// Gives the dot product of the two vectors A and B:
    Real Dot(const ChVector2<Real>& A, const ChVector2<Real>& B) const { return (A.x * B.x) + (A.y * B.y); }

    /// Gives the dot product with another vector: result=this*B
    Real Dot(const ChVector2<Real>& B) const { return (x * B.x) + (y * B.y); }

    /// The vector becomes the product of a vectors A and a scalar v: this=A*v
    void Mul(const ChVector2<Real>& A, const Real v) {
        x = A.x * v;
        y = A.y * v;
    }

    /// The vector is multiplied by a scalar factor 's': this*=v
    void Scale(const Real v) {
        x *= v;
        y *= v;
    }

    /// Computes the Euclidean norm of the vector, that is its length or magnitude
    Real Length() const { return std::sqrt(x * x + y * y); }

    /// Computes the squared Euclidean norm of the vector.
    Real Length2() const { return (x * x + y * y); }

    /// Computes the infinity norm of the vector, that is the maximum absolute value of one of its elements
    Real LengthInf() const { return ChMax(std::abs(x), std::abs(y)); }

    /// Normalize this vector, so that its Euclidean norm is 1.
    /// Returns false if original vector had zero length (in which case it is set to [0,1]).
    bool Normalize() {
        Real mlength = Length();
        if (mlength < CH_NANOTOL) {
            x = 1;
            y = 0;
            return false;
        }
        Scale(1 / mlength);
        return true;
    }

    /// Return a normalized copy of this vector, with Euclidean length = 1.
    ChVector2<Real> GetNormalized() const {
        ChVector2<Real> mret(*this);
        mret.Normalize();
        return mret;
    }

    /// Impose a new length to the vector, keeping the direction unchanged.
    void SetLength(Real v) {
        Normalize();
        Scale(v);
    }

    /// Apply a 2D rotation of given angle (positive counterclockwise).
    void Rotate(Real angle) {
        Real ca = std::cos(angle);
        Real sa = std::sin(angle);
        Real tmp = x * ca - y * sa;
        y = x * sa + y * ca;
        x = tmp;
    }

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data in archives.
    void ArchiveOUT(ChArchiveOut& marchive) {
        // suggested: use versioning
        marchive.VersionWrite(1);
        // stream out all member data
        marchive << CHNVP(x);
        marchive << CHNVP(y);
    }

    /// Method to allow de serialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive) {
        // suggested: use versioning
        int version = marchive.VersionRead();
        // stream in all member data
        marchive >> CHNVP(x);
        marchive >> CHNVP(y);
    }
};

/// Operator for scaling the vector by a scalar value, as s*V
template <class Real>
ChVector2<Real> operator*(const Real s, const ChVector2<Real>& V) {
    return ChVector2<Real>(V.x * s, V.y * s);
}

//
// STATIC VECTOR MATH OPERATIONS
//

template <class RealA, class RealB>
RealA Vdot(const ChVector2<RealA> va, const ChVector2<RealB> vb) {
    return (RealA)(va.x * vb.x + va.y * vb.y);
}

template <class RealA>
void Vset(ChVector2<RealA>& v, RealA mx, RealA my) {
    v.x = mx;
    v.y = my;
}

template <class RealA, class RealB>
ChVector2<RealA> Vadd(const ChVector2<RealA>& va, const ChVector2<RealB>& vb) {
    return ChVector2<RealA>(va.x + vb.x, va.y + vb.y);
}

template <class RealA, class RealB>
ChVector2<RealA> Vsub(const ChVector2<RealA>& va, const ChVector2<RealB>& vb) {
    return ChVector2<RealA>(va.x - vb.x, va.y - vb.y);
}

template <class RealA, class RealB>
ChVector2<RealA> Vmul(const ChVector2<RealA>& va, RealB fact) {
    return ChVector2<RealA>(va.x * (RealA)fact, va.y * (RealA)fact);
}

template <class RealA>
RealA Vlength(const ChVector2<RealA>& va) {
    return va.Length();
}

template <class RealA>
ChVector2<RealA> Vnorm(const ChVector2<RealA>& va) {
    return va.GetNormalized();
}

template <class RealA, class RealB>
bool Vequal(const ChVector2<RealA>& va, const ChVector2<RealB>& vb) {
    return (va == vb);
}

template <class RealA>
bool Vnotnull(const ChVector2<RealA>& va) {
    return (va.x != 0 || va.y != 0);
}

template <class RealA, class RealB>
ChVector2<RealA> Vrot(const ChVector2<RealA>& v, RealB angle) {
    ChVector2<RealA> tmp(v);
    tmp.Rotate((RealA)angle);
    return tmp;
}

}  // end namespace chrono

#endif
