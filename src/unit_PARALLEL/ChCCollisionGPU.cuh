#ifndef CHC_COLLISIONGPU_CUH
#define CHC_COLLISIONGPU_CUH
using namespace chrono::collision;


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static __device__ __host__ void ComputeAABBSphere(const real &radius, const real3 &position, real3 &minp, real3 &maxp) {
    minp = position - R3(radius);
    maxp = position + R3(radius);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static __device__ __host__ void ComputeAABBTriangle(const real3 &A, const real3 &B, const real3 &C, real3 &minp, real3 &maxp) {
    minp.x = min(A.x, min(B.x, C.x));
    minp.y = min(A.y, min(B.y, C.y));
    minp.z = min(A.z, min(B.z, C.z));
    maxp.x = max(A.x, max(B.x, C.x));
    maxp.y = max(A.y, max(B.y, C.y));
    maxp.z = max(A.z, max(B.z, C.z));
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static __device__ __host__ void ComputeAABBBox(const real3 &dim, const real3 &lpositon, const real3 &positon, const real4 &lrotation, const real4 &rotation, real3 &minp, real3 &maxp) {
    real3 pos = quatRotate(lpositon, rotation) + positon; //new position
    real4 q1 = mult(rotation, lrotation); //full rotation
    real4 q = R4(q1.y, q1.z, q1.w, q1.x);
    real t[3] = { pos.x, pos.y, pos.z };
    real mina[3] = { -dim.x, -dim.y, -dim.z };
    real maxa[3] = { dim.x, dim.y, dim.z };
    real minb[3] = { 0, 0, 0 };
    real maxb[3] = { 0, 0, 0 };
    real m[3][3];
    real qx2 = q.x * q.x;
    real qy2 = q.y * q.y;
    real qz2 = q.z * q.z;
    m[0][0] = 1 - 2 * qy2 - 2 * qz2;
    m[1][0] = 2 * q.x * q.y + 2 * q.z * q.w;
    m[2][0] = 2 * q.x * q.z - 2 * q.y * q.w;
    m[0][1] = 2 * q.x * q.y - 2 * q.z * q.w;
    m[1][1] = 1 - 2 * qx2 - 2 * qz2;
    m[2][1] = 2 * q.y * q.z + 2 * q.x * q.w   ;
    m[0][2] = 2 * q.x * q.z + 2 * q.y * q.w;
    m[1][2] = 2 * q.y * q.z - 2 * q.x * q.w;
    m[2][2] = 1 - 2 * qx2 - 2 * qy2;

    // For all three axes
    for (int i = 0; i < 3; i++) {
        // Start by adding in translation
        minb[i] = maxb[i] = t[i];

        // Form extent by summing smaller and larger terms respectively
        for (int j = 0; j < 3; j++) {
            real e = m[i][j] * mina[j];
            real f = m[i][j] * maxa[j];

            if (e < f) {
                minb[i] += e;
                maxb[i] += f;
            } else {
                minb[i] += f;
                maxb[i] += e;
            }
        }
    }

    minp = R3(minb[0], minb[1], minb[2]);
    maxp = R3(maxb[0], maxb[1], maxb[2]);
}

#endif


