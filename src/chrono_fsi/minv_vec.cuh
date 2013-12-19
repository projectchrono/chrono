// This function calculates edd = Mhat^(-1)*v for a "noodle"
// made up of 3 ANCF identical beam elements.
//
// As such, the input vector 'v' and the output vector 'edd'
// are assumed to be of length (3+1)*6 = 24
//
// Note that the matrix Mhat used for this calculation is just
//    Mhat = \int_0^1 { S^T(\xi) S(\xi) d \xi } 
// As such, the result must be muliplied by 1/(rho * A * le)
//

#ifndef MINV_VEC_CUH
#define MINV_VEC_CUH

void min_vec(real_* edd, real_* v, real_ lE, int nE, bool isCantilever);

#endif
