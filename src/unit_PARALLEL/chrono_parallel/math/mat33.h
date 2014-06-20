#ifndef MAT33_H
#define MAT33_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real2.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/real4.h"

struct M33 {
   real3 U, V, W;
   inline M33()
         :
           U(0),
           V(0),
           W(0) {
   }
   inline M33(
         real3 u,
         real3 v,
         real3 w)
         :
           U(u),
           V(v),
           W(w) {
   }

   inline M33 operator*(
         const M33& B) const {
      M33 result;
      result.U.x = U.x * B.U.x + V.x * B.U.y + W.x * B.U.z;     //row1 * col1
      result.V.x = U.x * B.V.x + V.x * B.V.y + W.x * B.V.z;     //row1 * col2
      result.W.x = U.x * B.W.x + V.x * B.W.y + W.x * B.W.z;     //row1 * col3

      result.U.y = U.y * B.U.x + V.y * B.U.y + W.y * B.U.z;     //row2 * col1
      result.V.y = U.y * B.V.x + V.y * B.V.y + W.y * B.V.z;     //row2 * col2
      result.W.y = U.y * B.W.x + V.y * B.W.y + W.y * B.W.z;     //row2 * col3

      result.U.z = U.z * B.U.x + V.z * B.U.y + W.z * B.U.z;     //row3 * col1
      result.V.z = U.z * B.V.x + V.z * B.V.y + W.z * B.V.z;     //row3 * col2
      result.W.z = U.z * B.W.x + V.z * B.W.y + W.z * B.W.z;     //row3 * col3

      return result;
   }

   inline real3 operator*(
         const real3& B) const {
      real3 result;

      result.x = U.x * B.x + V.x * B.y + W.x * B.z;     //row1 * col1
      result.y = U.y * B.x + V.y * B.y + W.y * B.z;     //row2 * col2
      result.z = U.z * B.x + V.z * B.y + W.z * B.z;     //row3 * col3

      return result;
   }

};

static inline M33 XMatrix(
      const real3 &vect) {
   M33 Xmat;
   Xmat.U.x = 0;
   Xmat.V.x = -vect.z;
   Xmat.W.x = vect.y;

   Xmat.U.y = vect.z;
   Xmat.V.y = 0;
   Xmat.W.y = -vect.x;

   Xmat.U.z = -vect.y;
   Xmat.V.z = vect.x;
   Xmat.W.z = 0;
   return Xmat;
}
static inline M33 MatMult(
      const M33 & A,
      const M33 &B) {

   return A * B;

}
static inline real3 MatMult(
      const M33 &A,
      const real3 &B) {
   return A * B;

}

//A is transposed
static inline M33 MatTMult(
      const M33 &A,
      const M33 &B) {
   M33 result;
   result.U.x = A.U.x * B.U.x + A.U.y * B.U.y + A.U.z * B.U.z;     //row1 * col1
   result.V.x = A.U.x * B.V.x + A.U.y * B.V.y + A.U.z * B.V.z;     //row1 * col2
   result.W.x = A.U.x * B.W.x + A.U.y * B.W.y + A.U.z * B.W.z;     //row1 * col3

   result.U.y = A.V.x * B.U.x + A.V.y * B.U.y + A.V.z * B.U.z;     //row2 * col1
   result.V.y = A.V.x * B.V.x + A.V.y * B.V.y + A.V.z * B.V.z;     //row2 * col2
   result.W.y = A.V.x * B.W.x + A.V.y * B.W.y + A.V.z * B.W.z;     //row2 * col3

   result.U.z = A.W.x * B.U.x + A.W.y * B.U.y + A.W.z * B.U.z;     //row3 * col1
   result.V.z = A.W.x * B.V.x + A.W.y * B.V.y + A.W.z * B.V.z;     //row3 * col2
   result.W.z = A.W.x * B.W.x + A.W.y * B.W.y + A.W.z * B.W.z;     //row3 * col3

   return result;
}

//B is transposed
static inline M33 MatMultT(
      const M33 &A,
      const M33 &B) {
   M33 result;
   result.U.x = A.U.x * B.U.x + A.V.x * B.V.x + A.W.x * B.W.x;     //row1 * col1
   result.V.x = A.U.x * B.U.y + A.V.x * B.V.y + A.W.x * B.W.y;     //row1 * col2
   result.W.x = A.U.x * B.U.z + A.V.x * B.V.z + A.W.x * B.W.z;     //row1 * col3

   result.U.y = A.U.y * B.U.x + A.V.y * B.V.x + A.W.y * B.W.x;     //row2 * col1
   result.V.y = A.U.y * B.U.y + A.V.y * B.V.y + A.W.y * B.W.y;     //row2 * col2
   result.W.y = A.U.y * B.U.z + A.V.y * B.V.z + A.W.y * B.W.z;     //row2 * col3

   result.U.z = A.U.z * B.U.x + A.V.z * B.V.x + A.W.z * B.W.x;     //row3 * col1
   result.V.z = A.U.z * B.U.y + A.V.z * B.V.y + A.W.z * B.W.y;     //row3 * col2
   result.W.z = A.U.z * B.U.z + A.V.z * B.V.z + A.W.z * B.W.z;     //row3 * col3

   return result;
}

static inline real3 MatTMult(
      const M33 &A,
      const real3 &B) {
   real3 result;

   result.x = A.U.x * B.x + A.U.y * B.y + A.U.z * B.z;     //row1 * col1
   result.y = A.V.x * B.x + A.V.y * B.y + A.V.z * B.z;     //row2 * col2
   result.z = A.W.x * B.x + A.W.y * B.y + A.W.z * B.z;     //row3 * col3

   return result;
}

static inline M33 AMat(
      const real4 &q) {
   M33 result;

   real e0e0 = q.w * q.w;
   real e1e1 = q.x * q.x;
   real e2e2 = q.y * q.y;
   real e3e3 = q.z * q.z;
   real e0e1 = q.w * q.x;
   real e0e2 = q.w * q.y;
   real e0e3 = q.w * q.z;
   real e1e2 = q.x * q.y;
   real e1e3 = q.x * q.z;
   real e2e3 = q.y * q.z;
   result.U.x = (e0e0 + e1e1) * 2 - 1;
   result.V.x = (e1e2 - e0e3) * 2;
   result.W.x = (e1e3 + e0e2) * 2;

   result.U.y = (e1e2 + e0e3) * 2;
   result.V.y = (e0e0 + e2e2) * 2 - 1;
   result.W.y = (e2e3 - e0e1) * 2;

   result.U.z = (e1e3 - e0e2) * 2;
   result.V.z = (e2e3 + e0e1) * 2;
   result.W.z = (e0e0 + e3e3) * 2 - 1;

//	result.U.x = (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
//	result.V.x = (2 * q.x * q.y - 2 * q.w * q.z);
//	result.W.x = (2 * q.x * q.z + 2 * q.w * q.y);
//
//	result.U.y = (2 * q.x * q.y + 2 * q.w * q.z);
//	result.V.y = (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z);
//	result.W.y = (2 * q.y * q.z - 2 * q.w * q.x);
//
//	result.U.z = (2 * q.x * q.z - 2 * q.w * q.y);
//	result.V.z = (2 * q.y * q.z + 2 * q.w * q.x);
//	result.W.z = (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
   return result;

}
//[U.x,U.y,U.z]
//[V.x,V.y,V.z]
//[W.x,W.y,W.z]
static inline M33 AMatT(
      const real4 &q) {
   M33 result;

   real e0e0 = q.w * q.w;
   real e1e1 = q.x * q.x;
   real e2e2 = q.y * q.y;
   real e3e3 = q.z * q.z;
   real e0e1 = q.w * q.x;
   real e0e2 = q.w * q.y;
   real e0e3 = q.w * q.z;
   real e1e2 = q.x * q.y;
   real e1e3 = q.x * q.z;
   real e2e3 = q.y * q.z;
   result.U.x = (e0e0 + e1e1) * 2 - 1;
   result.U.y = (e1e2 - e0e3) * 2;
   result.U.z = (e1e3 + e0e2) * 2;

   result.V.x = (e1e2 + e0e3) * 2;
   result.V.y = (e0e0 + e2e2) * 2 - 1;
   result.V.z = (e2e3 - e0e1) * 2;

   result.W.x = (e1e3 - e0e2) * 2;
   result.W.y = (e2e3 + e0e1) * 2;
   result.W.z = (e0e0 + e3e3) * 2 - 1;

   return result;

}
static inline M33 AbsMat(
      const M33 &A) {
   M33 result;
   result.U.x = fabs(A.U.x);
   result.U.y = fabs(A.U.y);
   result.U.z = fabs(A.U.z);
   result.V.x = fabs(A.V.x);
   result.V.y = fabs(A.V.y);
   result.V.z = fabs(A.V.z);
   result.W.x = fabs(A.W.x);
   result.W.y = fabs(A.W.y);
   result.W.z = fabs(A.W.z);

   return result;

}

static inline M33 Transpose(
      const M33 &A) {

   M33 result;

   result.U = real3(A.U.x, A.V.x, A.W.x);
   result.V = real3(A.U.y, A.V.y, A.W.y);
   result.W = real3(A.U.z, A.V.z, A.W.z);

   return result;
}

static inline ostream &operator<<(
      ostream &out,
      const M33 &a) {
   out << a.U << a.V << a.W << endl;
   return out;
}

//[U.x,V.x,W.x]
//[U.y,V.y,W.y]
//[U.z,V.z,W.z]
//transposed:
//[U.x,U.y,U.z]
//[V.x,V.y,V.z]
//[W.x,W.y,W.z]

//[U.x,V.x,W.x][x]
//[U.y,V.y,W.y][y]
//[U.z,V.z,W.z][z]

#endif
