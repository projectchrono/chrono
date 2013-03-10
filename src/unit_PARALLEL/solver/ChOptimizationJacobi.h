#ifndef CHOPTIMIZATIONJACOBI_H
#define CHOPTIMIZATIONJACOBI_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"

namespace chrono {
    class ChApiGPU ChOptimizationJacobi {
        public:
            ChOptimizationJacobi(custom_vector<real>& Jx_,       custom_vector<real>&  Jy_,        custom_vector<real>& Jz_,
                                 custom_vector<real>& Ju_,       custom_vector<real>&  Jv_,        custom_vector<real>& Jw_,
                                 custom_vector<uint>& B1_,       custom_vector<uint>&  B2_,
                                 custom_vector<real>& inv_mass_, custom_vector<real3>& inv_inertia_, custom_vector<real>& mu_, custom_vector<real>& correction_,
                                 uint& num_bodies_,  uint& num_constraints_, custom_vector<int3>& contact_triplets_, custom_vector<real3>& bilateral_ids_) {
                Jx = &Jx_;
                Jy = &Jy_;
                Jz = &Jz_;
                Ju = &Ju_;
                Jv = &Jv_;
                Jw = &Jw_;
                B1 = &B1_;
                B2 = &B2_;
                inv_mass = &inv_mass_;
                inv_inertia = &inv_inertia_;
                mu = &mu_;
                correction = &correction_;
                num_bodies = num_bodies_;
                num_constraints = num_constraints_;
            }
            virtual custom_vector<real> LinearOperator() {
                q_x.resize(num_bodies, 0), q_y.resize(num_bodies, 0), q_z.resize(num_bodies, 0), q_u.resize(num_bodies, 0), q_v.resize(num_bodies, 0), q_w.resize(num_bodies, 0);
                Ax.resize(num_constraints, 0);

                for (uint i = 0; i < num_constraints; i++) {
                    real temp = 0, m;
                    uint b1 = (*B1)[i];
                    uint b2 = (*B2)[i];

                    if (i != b1) {
                        q_x[b1] += (*Jx)[i] * (x)[i];
                        q_y[b1] += (*Jy)[i] * (x)[i];
                        q_z[b1] += (*Jz)[i] * (x)[i];
                        q_u[b1] += (*Ju)[i] * (x)[i];
                        q_v[b1] += (*Jv)[i] * (x)[i];
                        q_w[b1] += (*Jw)[i] * (x)[i];
                    }

                    if (i != b2) {
                        q_x[b2] += (*Jx)[i+num_constraints] * (x)[i];
                        q_y[b2] += (*Jy)[i+num_constraints] * (x)[i];
                        q_z[b2] += (*Jz)[i+num_constraints] * (x)[i];
                        q_u[b2] += (*Ju)[i+num_constraints] * (x)[i];
                        q_v[b2] += (*Jv)[i+num_constraints] * (x)[i];
                        q_w[b2] += (*Jw)[i+num_constraints] * (x)[i];
                    }
                }

                for (uint i = 0; i < num_constraints; i++) {
                    real temp = 0;
                    uint b1 = (*B1)[i];
                    uint b2 = (*B2)[i];
                    real m1 = (*inv_mass)[b1];
                    real m2 = (*inv_mass)[b2];

                    if (i != b1) {
                        temp += q_x[b1] * (*Jx)[i] * m1;
                        temp += q_y[b1] * (*Jy)[i] * m1;
                        temp += q_z[b1] * (*Jz)[i] * m1;
                        temp += q_u[b1] * (*Ju)[i] * m1;
                        temp += q_v[b1] * (*Jv)[i] * m1;
                        temp += q_w[b1] * (*Jw)[i] * m1;
                    }

                    if (i != b2) {
                        temp += q_x[b2] * (*Jx)[i+num_constraints] * m2;
                        temp += q_y[b2] * (*Jy)[i+num_constraints] * m2;
                        temp += q_z[b2] * (*Jz)[i+num_constraints] * m2;
                        temp += q_u[b2] * (*Ju)[i+num_constraints] * m2;
                        temp += q_v[b2] * (*Jv)[i+num_constraints] * m2;
                        temp += q_w[b2] * (*Jw)[i+num_constraints] * m2;
                    }

                    Ax[i] += temp;
                }

                return Ax;
            }

            void InvNDiag() {
                inv_n_diag.resize(num_constraints);

                for (int i = 0; i < num_constraints; i++) {
                    uint b1 = (*B1)[i];
                    uint b2 = (*B2)[i];
                    //
                    real Jx1 = (*Jx)[i];
                    real Jy1 = (*Jy)[i];
                    real Jz1 = (*Jz)[i];
                    real Ju1 = (*Ju)[i];
                    real Jv1 = (*Jv)[i];
                    real Jw1 = (*Jw)[i];
                    //
                    real Jx2 = (*Jx)[i+num_constraints];
                    real Jy2 = (*Jy)[i+num_constraints];
                    real Jz2 = (*Jz)[i+num_constraints];
                    real Ju2 = (*Ju)[i+num_constraints];
                    real Jv2 = (*Jv)[i+num_constraints];
                    real Jw2 = (*Jw)[i+num_constraints];
                    //
                    real m1 = (*inv_mass)[b1];
                    real m2 = (*inv_mass)[b2];
                    //
                    real3 i1 = (*inv_inertia)[b1];
                    real3 i2 = (*inv_inertia)[b2];
                    //
                    real part1 = m1 * (Jx1 * Jx1 + Jy1 * Jy1 + Jz1 * Jz1);
                    real part2 = i1.x * (Ju1 * Ju1) + i1.y * (Jv1 * Jv1) + i1.z * (Jw1 * Jw1);
                    real part3 = m2 * (Jx2 * Jx2 + Jy2 * Jy2 + Jz2 * Jz2);
                    real part4 = i2.x * (Ju2 * Ju2) + i2.y * (Jv2 * Jv2) + i2.z * (Jw2 * Jw2);
                    inv_n_diag[i] = 1 / (part1 + part2 + part3 + part4);
                }
            }


            virtual void Project() {}
            virtual void Rhs() {
               // real inv_hpa = 1.0 / (h + alpha); // 1/(h+a)
                #pragma omp parallel for

                for (int i = 0; i < num_constraints; i++) {
                    uint b1 = (*B1)[i];
                    uint b2 = (*B2)[i];
                    real m1 = (*inv_mass)[b1];
                    real m2 = (*inv_mass)[b2];
                    real temp = 0;

                    //if (body_data.active[body_id]) {
                    temp -= (*Jx)[i] * (*vel_x)[b1];
                    temp -= (*Jy)[i] * (*vel_y)[b1];
                    temp -= (*Jz)[i] * (*vel_z)[b1];
                    temp -= (*Ju)[i] * (*omg_x)[b1];
                    temp -= (*Jv)[i] * (*omg_y)[b1];
                    temp -= (*Jw)[i] * (*omg_z)[b1];
                    //}

                    //if (body_data.active[body_id]) {

                    temp -= (*Jx)[i+num_constraints] * (*vel_x)[b2];
                    temp -= (*Jy)[i+num_constraints] * (*vel_y)[b2];
                    temp -= (*Jz)[i+num_constraints] * (*vel_z)[b2];
                    temp -= (*Ju)[i+num_constraints] * (*omg_x)[b2];
                    temp -= (*Jv)[i+num_constraints] * (*omg_y)[b2];
                    temp -= (*Jw)[i+num_constraints] * (*omg_z)[b2];
                    //}
                    //b[i] = (temp - inv_hpa * (fmin(0, correction[i]))) * (fmin(0, correction[i]) < 0);
                    //b[i]=temp-fmax(1/h * correction[i],0 );
                }
            }
            custom_vector<real> Ax, b, x, inv_n_diag;
            custom_vector<real> q_x, q_y, q_z, q_u, q_v, q_w;
            custom_vector<real>* Jx, *Jy, *Jz, *Ju, *Jv, *Jw, *inv_mass, *mu, *correction;
            custom_vector<real>* vel_x, *vel_y, *vel_z, *omg_x, *omg_y, *omg_z;
            custom_vector<real3>* inv_inertia;
            custom_vector<uint>* B1, * B2;
            custom_vector<int3>* contact_triplets, *bilateral_ids;
            uint num_bodies;
            uint num_constraints;
    };
}

#endif
