//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHVECTOR_H
#define CHVECTOR_H

//////////////////////////////////////////////////
//
//   ChVector.h
//
//   Math functions for:
//      - VECTORS
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <math.h>
#include <iostream>
#include <iomanip>

#include "core/ChMathematics.h"
#include "core/ChClassRegister.h"
#include "serialization/ChArchive.h"

namespace chrono {

// CONSTANTS

// Added this here as code was not compiling (under linux at least)- Hammad
#define VNULL ChVector<double>(0., 0., 0.)
#define VECT_X ChVector<double>(1., 0., 0.)
#define VECT_Y ChVector<double>(0., 1., 0.)
#define VECT_Z ChVector<double>(0., 0., 1.)

/// Definition of general purpose 3d vector variables, such as points in 3D.
/// This class implements the vectorial algebra in 3D (Gibbs products).
/// ChVector is templated by precision, with default 'double'.
///
/// Further info at the @ref mathematical_objects manual page.
template <class Real = double>
class ChVector {
  public:
    //
    // DATA
    //
    Real x;
    Real y;
    Real z;

    //
    // CONSTRUCTORS
    //

    ChVector() : x(0), y(0), z(0){};
    ChVector(const Real nx, const Real ny, const Real nz) : x(nx), y(ny), z(nz){};
    ChVector(const Real n) : x(n), y(n), z(n){};  // initialize to a constant

    /// Copy constructor
    ChVector(const ChVector<Real>& other) : x(other.x), y(other.y), z(other.z){};

    /// Copy constructor between vectors float<->double etc
    template <class RealB>
    ChVector(const ChVector<RealB>& other)
        : x((Real)other.x), y((Real)other.y), z((Real)other.z) {}

    //
    // OPERATORS OVERLOADING
    //
    // Note: c++ automatically creates temporary objects to store intermediate
    // results in long formulas, such as a= b*c*d, so the usage of operators
    // may give slower results than a wise (less readable however) usage of
    // Dot(), Cross() etc.. Also pay attention to C++ operator precedence rules!

    /// Assignment operator: copy from another vector
    ChVector<Real>& operator=(const ChVector<Real>& other) {
        if (&other == this)
            return *this;
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    template <class RealB>
    ChVector<Real>& operator=(const ChVector<RealB>& other) {
        x = (Real)other.x;
        y = (Real)other.y;
        z = (Real)other.z;
        return *this;
    }

    /// Operator for sign change
    ChVector<Real> operator-() const { return ChVector<Real>(-x, -y, -z); }

    /// Operator for indexed access (0=x, 1=y, 2=z). Not so much performance..may be useful sometimes..
    inline Real& operator()(const int el) {
        assert(el < 3 && el >= 0);
        if (el == 0)
            return x;
        else if (el == 1)
            return y;
        return z;
    };
    inline const Real& operator()(const int el) const {
        assert(el < 3 && el >= 0);
        if (el == 0)
            return x;
        else if (el == 1)
            return y;
        return z;
    };

    /// Operator for vector sum
    ChVector<Real> operator+(const ChVector<Real>& other) const {
        return ChVector<Real>(x + other.x, y + other.y, z + other.z);
    }
    ChVector<Real>& operator+=(const ChVector<Real>& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    /// Operator for vector difference
    ChVector<Real> operator-(const ChVector<Real>& other) const {
        return ChVector<Real>(x - other.x, y - other.y, z - other.z);
    }
    ChVector<Real>& operator-=(const ChVector<Real>& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    /// Operator for element-wise multiplication (note that
    /// this is neither dot product nor cross product!)
    ChVector<Real> operator*(const ChVector<Real>& other) const {
        return ChVector<Real>(x * other.x, y * other.y, z * other.z);
    }
    ChVector<Real>& operator*=(const ChVector<Real>& other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;
        return *this;
    }

    /// Operator for scaling the vector by a scalar value, as V*s
    ChVector<Real> operator*(const Real s) const { return ChVector<Real>(x * s, y * s, z * s); }
    ChVector<Real>& operator*=(const Real s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    /// Operator for element-wise division (note that
    /// 3D vector algebra is a skew field, non-divisional algebra, so this
    /// division operation is just an element-by element division)
    ChVector<Real> operator/(const ChVector<Real>& other) const {
        return ChVector<Real>(x / other.x, y / other.y, z / other.z);
    }
    ChVector<Real>& operator/=(const ChVector<Real>& other) {
        x /= other.x;
        y /= other.y;
        z /= other.z;
        return *this;
    }

    /// Operator for scaling the vector by inverse of a scalar value, as v/s
    ChVector<Real> operator/(const Real v) const {
        Real i = (Real)1.0 / v;
        return ChVector<Real>(x * i, y * i, z * i);
    }
    ChVector<Real>& operator/=(const Real v) {
        Real i = (Real)1.0 / v;
        x *= i;
        y *= i;
        z *= i;
        return *this;
    }

    /// Operator for cross product: A%B means the vector cross-product AxB
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    ChVector<Real> operator%(const ChVector<Real>& other) const {
        ChVector<Real> mr;
        mr.Cross(*this, other);
        return mr;
    }
    ChVector<Real>& operator%=(const ChVector<Real>& other) {
        this->Cross(*this, other);
        return *this;
    }

    /// Operator for dot product: A^B means the scalar dot-product A*B
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    double operator^(const ChVector<Real>& other) const { return Dot(*this, other); }

    bool operator<=(const ChVector<Real>& other) const { return x <= other.x && y <= other.y && z <= other.z; }
    bool operator>=(const ChVector<Real>& other) const { return x >= other.x && y >= other.y && z >= other.z; }
    bool operator<(const ChVector<Real>& other) const { return x < other.x && y < other.y && z < other.z; }
    bool operator>(const ChVector<Real>& other) const { return x > other.x && y > other.y && z > other.z; }

    bool operator==(const ChVector<Real>& other) const { return other.x == x && other.y == y && other.z == z; }
    bool operator!=(const ChVector<Real>& other) const { return other.x != x || other.y != y || other.z != z; }

    //
    // FUNCTIONS
    //

    /// Sets the three values of the vector at once
    void Set(const Real nx, const Real ny, const Real nz) {
        x = nx;
        y = ny;
        z = nz;
    }

    /// Sets the vector as a copy of another vector
    void Set(const ChVector<Real>& p) {
        x = p.x;
        y = p.y;
        z = p.z;
    }

    /// Sets the vector with three components as a sample scalar
    void Set(const Real p) {
        x = p;
        y = p;
        z = p;
    }

    /// Sets the vector as a null vector
    void SetNull() { x = y = z = 0; }

    /// Returns true if this vector is the null vector
    bool IsNull() const { return x == 0 && y == 0 && z == 0; }

    /// Returns true if vector is identical to other vector
    bool Equals(const ChVector<Real>& other) const { return (other.x == x) && (other.y == y) && (other.z == z); }

    /// Returns true if vector equals another vector, within a tolerance 'tol'
    bool Equals(const ChVector<Real>& other, Real tol) const {
        return (fabs(other.x - x) < tol) && (fabs(other.y - y) < tol) && (fabs(other.z - z) < tol);
    }

    /// The vector becomes the sum of the two vectors A and B:
    /// this=A+B
    void Add(const ChVector<Real>& A, const ChVector<Real>& B) {
        x = A.x + B.x;
        y = A.y + B.y;
        z = A.z + B.z;
    }

    /// The vector becomes the difference of the two vectors A and B:
    /// this=A-B
    void Sub(const ChVector<Real>& A, const ChVector<Real>& B) {
        x = A.x - B.x;
        y = A.y - B.y;
        z = A.z - B.z;
    }

    /// The vector becomes the cross product of the two vectors A and B:
    /// this=AxB
    void Cross(const ChVector<Real>& A, const ChVector<Real>& B) {
        this->x = (A.y * B.z) - (A.z * B.y);
        this->y = (A.z * B.x) - (A.x * B.z);
        this->z = (A.x * B.y) - (A.y * B.x);
    }

    /// Gives the dot product of the two vectors A and B:
    Real Dot(const ChVector<Real>& A, const ChVector<Real>& B) const { return (A.x * B.x) + (A.y * B.y) + (A.z * B.z); }

    /// Gives the dot product with another vector: result=this*B
    Real Dot(const ChVector<Real>& B) const { return (x * B.x) + (y * B.y) + (z * B.z); }

    /// The vector becomes the product of a vectors A and a scalar v: this=A*v
    void Mul(const ChVector<Real>& A, const Real v) {
        x = A.x * v;
        y = A.y * v;
        z = A.z * v;
    }

    /// The vector is multiplied by a scalar factor 's': this*=v
    void Scale(const Real v) {
        x *= v;
        y *= v;
        z *= v;
    }

    /// Computes the euclidean norm of the vector,
    /// that is its length or magnitude
    Real Length() const { return sqrt(x * x + y * y + z * z); }

    /// Computes the euclidean norm of the vector, squared
    /// (i.e. as Length(), but skipping the square root)
    Real Length2() const { return (x * x + y * y + z * z); }

    /// Computes the infinite norm of the vector, that
    /// is the maximum absolute value of one of its elements
    Real LengthInf() const { return ChMax(ChMax(fabs(x), fabs(y)), fabs(z)); }

    /// Normalize this vector, so that its euclidean length is 1.
    /// Returns false if original vector had zero length (in such a case
    /// it will be defaulted as VECT_X) otherwise returns true for success.
    bool Normalize() {
        Real mlength = this->Length();
        if (mlength < CH_NANOTOL) {
            x = 1;
            y = 0;
            z = 0;
            return false;
        }
        this->Scale(1 / mlength);
        return true;
    }

    /// Return a normalized copy of this vector, with euclidean length =1.
    /// Not to be confused with Normalize(), that normalizes in place.
    ChVector<Real> GetNormalized() const {
        ChVector<Real> mret(*this);
        mret.Normalize();
        return mret;
    }

    /// Impose a new length to the vector, keeping the direction unchanged.
    void SetLength(Real v) {
        Normalize();
        Scale(v);
    }

    /// Use the Gram-Schmidt orthonormalization to find the three
    /// orthogonal vectors of a coordinate system whose X axis is this vector.
    /// Vsingular (optional) sets the normal to the plane on which Dz must lie.
    void DirToDxDyDz(ChVector<Real>& Vx,
                     ChVector<Real>& Vy,
                     ChVector<Real>& Vz,
                     const ChVector<Real>& Vsingular = ChVector<Real>(0, 1, 0)) const {
        // set Vx.
        if (this->IsNull())
            Vx = ChVector<Real>(1, 0, 0);
        else
            Vx = this->GetNormalized();

        Vz.Cross(Vx, Vsingular);
        Real mzlen = Vz.Length();

        // if near singularity, change the singularity reference vector.
        if (mzlen < 0.0001) {
            ChVector<Real> mVsingular;

            if (fabs(Vsingular.x) < 0.9)
                mVsingular = ChVector<Real>(1, 0, 0);
            else if (fabs(Vsingular.y) < 0.9)
                mVsingular = ChVector<Real>(0, 1, 0);
            else if (fabs(Vsingular.z) < 0.9)
                mVsingular = ChVector<Real>(0, 0, 1);

            Vz.Cross(Vx, mVsingular);
            mzlen = Vz.Length();  // now should be nonzero length.
        }

        // normalize Vz.
        Vz.Scale(1.0 / mzlen);

        // compute Vy.
        Vy.Cross(Vz, Vx);
    }

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data in archives.
    void ArchiveOUT(ChArchiveOut& marchive)
    {
        // suggested: use versioning
        marchive.VersionWrite(1);
        // stream out all member data
        marchive << CHNVP(x);
        marchive << CHNVP(y);
        marchive << CHNVP(z);
    }

    /// Method to allow de serialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive) 
    {
        // suggested: use versioning
        int version = marchive.VersionRead();
        // stream in all member data
        marchive >> CHNVP(x);
        marchive >> CHNVP(y);
        marchive >> CHNVP(z);
    }
};

/// Shortcut for faster use of typical double-precision vectors.
/// Instead of writing
///    ChVector<double> foo;
/// or
///    ChVector<> foo;
/// you can use the shorter version
///    Vector foo;
typedef ChVector<double> Vector;

/// Shortcut for faster use of typical single-precision vectors.
/// Instead of writing
///    ChVector<float> foo;
/// you can use the shorter version
///    Vector foo;
typedef ChVector<float> VectorF;

// Reversed operators

/// Operator for scaling the vector by a scalar value, as s*V
template <class Real>
ChVector<Real> operator*(const Real s, const ChVector<Real>& V) {
    return ChVector<Real>(V.x * s, V.y * s, V.z * s);
}

//
// CONSTANTS
//
// Commented this and added #defines above as code was not compiling (under linux at least)- Hammad
// static const ChVector<double> VNULL(0.,0.,0.);
// static const ChVector<double> VECT_X(1.,0.,0.);
// static const ChVector<double> VECT_Y(0.,1.,0.);
// static const ChVector<double> VECT_Z(0.,0.,1.);

//
// STATIC VECTOR MATH OPERATIONS
//
//  These functions are here for people which prefer to use static
// functions instead of ChVector class' member functions.
// NOTE: sometimes a wise adoption of the following functions may
// give faster results rather than using overloaded operators +/-/* in
// the vector class.
//  Also, these functions are here for backward compatibility with
// old parts of Chrono code.
//  For best readability of our code, it is suggested not to use
// these functions - use the member functions or operators of
// the ChVector class instead-.

template <class RealA, class RealB>
RealA Vdot(const ChVector<RealA> va, const ChVector<RealB> vb) {
    return (RealA)((va.x * vb.x) + (va.y * vb.y) + (va.z * vb.z));
}

template <class RealA>
void Vset(ChVector<RealA>* v, RealA mx, RealA my, RealA mz) {
    v->x = mx;
    v->y = my;
    v->z = mz;
}

template <class RealA, class RealB>
ChVector<RealA> Vadd(const ChVector<RealA>& va, const ChVector<RealB>& vb) {
    ChVector<RealA> result;
    result.x = va.x + vb.x;
    result.y = va.y + vb.y;
    result.z = va.z + vb.z;
    return result;
}

template <class RealA, class RealB>
ChVector<RealA> Vsub(const ChVector<RealA>& va, const ChVector<RealB>& vb) {
    ChVector<RealA> result;
    result.x = va.x - vb.x;
    result.y = va.y - vb.y;
    result.z = va.z - vb.z;
    return result;
}

template <class RealA, class RealB>
ChVector<RealA> Vcross(const ChVector<RealA>& va, const ChVector<RealB>& vb) {
    ChVector<RealA> result;
    result.x = (va.y * vb.z) - (va.z * vb.y);
    result.y = (va.z * vb.x) - (va.x * vb.z);
    result.z = (va.x * vb.y) - (va.y * vb.x);
    return result;
}

template <class RealA, class RealB>
ChVector<RealA> Vmul(const ChVector<RealA>& va, RealB fact) {
    ChVector<RealA> result;
    result.x = va.x * (RealA)fact;
    result.y = va.y * (RealA)fact;
    result.z = va.z * (RealA)fact;
    return result;
}

template <class RealA>
RealA Vlength(const ChVector<RealA>& va) {
    return (RealA)va.Length();
}

template <class RealA>
ChVector<RealA> Vnorm(const ChVector<RealA>& va) {
    ChVector<RealA> result(va);
    result.Normalize();
    return result;
}

template <class RealA, class RealB>
bool Vequal(const ChVector<RealA>& va, const ChVector<RealB>& vb) {
    return (va == vb);
}

template <class RealA>
bool Vnotnull(ChVector<RealA>* va) {
    if (va->x)
        return true;
    if (va->y)
        return true;
    if (va->z)
        return true;
    return false;
}

// Gets the zenith angle of a unit vector respect to YZ plane  ***OBSOLETE
template <class RealA>
double VangleYZplane(ChVector<RealA>* va) {
    return asin(Vdot(*va, VECT_X));
}

// Gets the zenith angle of a unit vector respect to YZ plane  ***OBSOLETE
template <class RealA>
double VangleYZplaneNorm(ChVector<RealA>* va) {
    return acos(Vdot(*va, VECT_X));
}

// Gets the angle of the projection on the YZ plane respect to
// the Y vector, as spinning about X.
template <class RealA>
double VangleRX(ChVector<RealA>* va) {
    Vector vproj;
    vproj.x = 0;
    vproj.y = va->y;
    vproj.z = va->z;
    vproj = Vnorm(vproj);
    if (vproj.x == 1)
        return 0;
    return acos(vproj.y);
}

// The reverse of the two previous functions, gets the vector
// given the angle above the normal to YZ plane and the angle
// of rotation on X
template <class RealA>
ChVector<RealA> VfromPolar(double norm_angle, double pol_angle) {
    ChVector<> res;
    double projlen;
    res.x = cos(norm_angle);  // 1) rot 'norm.angle'about z
    res.y = sin(norm_angle);
    res.z = 0;
    projlen = res.y;
    res.y = projlen * cos(pol_angle);
    res.z = projlen * sin(pol_angle);
    return res;
}

// From non normalized x direction, to versors DxDyDz.
// mVsingular (optional) sets normal to plane on which Dz must lie.
template <class RealA>
void XdirToDxDyDz(ChVector<RealA>* mVxdir,
                  ChVector<RealA>* mVsingular,
                  ChVector<RealA>* Vx,
                  ChVector<RealA>* Vy,
                  ChVector<RealA>* Vz) {
    ChVector<RealA> mdefVsingular;
    ChVector<RealA> mVnull = VNULL;
    double mzlen;

    mdefVsingular = VECT_Y;

    if (Vequal(*mVxdir, mVnull))
        *Vx = VECT_X;
    else
        *Vx = Vnorm(*mVxdir);

    if (!mVsingular)
        mVsingular = &mdefVsingular;

    *Vz = Vcross(*Vx, *mVsingular);
    mzlen = Vlength(*Vz);

    if (mzlen < 0.0001)  // was near singularity? change singularity reference vector!
    {
        if (fabs(mVsingular->z) < 0.9)
            *mVsingular = VECT_Z;
        if (fabs(mVsingular->y) < 0.9)
            *mVsingular = VECT_Y;
        if (fabs(mVsingular->x) < 0.9)
            *mVsingular = VECT_X;
        *Vz = Vcross(*Vx, *mVsingular);
        mzlen = Vlength(*Vz);  // now should be nonzero length.
    }
    // normalize Vz
    *Vz = Vmul(*Vz, 1.0 / mzlen);
    // compute Vy
    *Vy = Vcross(*Vz, *Vx);
}

}  // END_OF_NAMESPACE____

#endif  // END of ChVector.h
