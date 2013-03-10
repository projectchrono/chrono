#ifndef CHBODYGPU_H
#define CHBODYGPU_H

//////////////////////////////////////////////////
//
//   ChBodyGPU.h
//   Derived Class for GPU rigid bodies,
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChBody.h"
#include "ChApiGPU.h"
#include "collision/ChCCollisionModelGPU.h"
namespace chrono {
    using namespace collision;

    //////////////////////////////////////
    /// CLASS FOR SOLID GPU BODIES
    //////////////////////////////////////
    class ChApiGPU ChBodyGPU: public ChBody {
            CH_RTTI(ChBodyGPU, ChPhysicsItem);

        public:

            // CONSTRUCTORS

            ChBodyGPU();
            ~ChBodyGPU();

            virtual ChCollisionModel* InstanceCollisionModel();

            ChVector<> GetXForce() {
                return Xforce;
            }
            ChVector<> GetXTorque() {
                return Xtorque;
            }
            ChVector<> GetGyro() {
                return gyro;
            }
            ChVector<> SetGyro(ChVector<> g) {
                gyro = g;
            }
            void SetAppliedForce(ChVector<> mForce) {
                mAppliedForce = mForce;
            }
            ChVector<> GetAppliedForce() {
                return mAppliedForce;
            }
            void SetId(int identifier) {
                id = identifier;
            }
            int GetId() {
                return id;
            }

            /*virtual void SetPos(const ChVector<>& mpos) {
                coord.pos = mpos;
                if (gpu_data_manager) {
                    gpu_data_manager->host_pos_data[id] = F3(mpos.x, mpos.y, mpos.z);
                }
            }
            virtual void SetRot(const ChQuaternion<>& mrot) {
                coord.rot = mrot;
                Amatrix.Set_A_quaternion(mrot);
                if (gpu_data_manager) {
                    gpu_data_manager->host_rot_data[id] = F4(mrot.e0, mrot.e1, mrot.e2, mrot.e3);
                }
            }
            virtual void SetPos_dt(const ChVector<>& mvel) {
                coord_dt.pos = mvel;
                if (gpu_data_manager) {
                    gpu_data_manager->host_vel_data[id] = F3(mvel.x, mvel.y, mvel.z);
                }
            }

            virtual void SetPos_dtdt(const ChVector<>& macc) {
                coord_dtdt.pos = macc;
                if (gpu_data_manager) {
                    gpu_data_manager->host_acc_data[id] = F3(macc.x, macc.y, macc.z);
                }
            }

            virtual void SetWvel_loc(const ChVector<>& wl) {
                coord_dt.rot.Cross(this->coord.rot, ChQuaternion<>(0, wl));
                coord_dt.rot *= 0.5; // q_dt = 1/2 * q * (0,wl)
                if (gpu_data_manager) {
                    gpu_data_manager->host_omg_data[id] = F3(wl.x, wl.y, wl.z);
                }
            }

            ChVector<> GetPos() {

                //
                if (gpu_data_manager) {
                    float3 t = gpu_data_manager->host_pos_data[id];
                    return ChVector<>(t.x, t.y, t.z);
                } else {
                    return coord.pos;
                }
            }
            ChQuaternion<> GetRot() {
                //
                if (gpu_data_manager) {
                    float4 t = gpu_data_manager->host_rot_data[id];
                    return ChQuaternion<>(t.x, t.y, t.z, t.w);
                } else {
                    return coord.rot;
                }
            }

            ChVector<> GetPos_dt() {

                if (gpu_data_manager) {
                    float3 t = gpu_data_manager->host_vel_data[id];
                    return ChVector<>(t.x, t.y, t.z);
                } else {
                    return coord_dt.pos;
                }
            }
            ChVector<> GetPos_dtdt() {

                if (gpu_data_manager) {
                    float3 t = gpu_data_manager->host_acc_data[id];
                    return ChVector<>(t.x, t.y, t.z);
                } else {
                    return coord_dtdt.pos;
                }
            }

            bool IsActive() {
                if (gpu_data_manager) {
                    return gpu_data_manager->host_aux_data[id].x;
                } else {
                    return !BFlagGet(BF_SLEEPING | BF_FIXED);
                }

            }

            void SetBodyFixed(bool mev) {
                if (gpu_data_manager) {
                    gpu_data_manager->host_aux_data[id].x = !mev;
                } //else {
                variables.SetDisabled(mev);
                if (mev == BFlagGet(BF_FIXED))
                    return;
                BFlagSet(BF_FIXED, mev);
                //}

                //RecomputeCollisionModel(); // because one may use different model types for static or dynamic coll.shapes
            }

            void VariablesFbLoadForces(double factor) {
                // add applied forces and torques (and also the gyroscopic torque!) to 'fb' vector
                if (gpu_data_manager) {
                    ChVector<> t1 = Xforce * factor;
                    ChVector<> t2 = (Xtorque - gyro) * factor;
                    gpu_data_manager->host_frc_data[id] = (F3(t1.x, t1.y, t1.z)); //forces
                    gpu_data_manager->host_trq_data[id] = (F3(t2.x, t2.y, t2.z)); //torques
                } else {

                    this->variables.Get_fb().PasteSumVector(Xforce * factor, 0, 0);
                    this->variables.Get_fb().PasteSumVector((Xtorque - gyro) * factor, 3, 0);
                }
            }*/

            ChVector<> mAppliedForce;
            //ChGPUDataManager *gpu_data_manager;
            int id;

        private:

    };

    typedef ChSharedPtr<ChBodyGPU> ChSharedBodyGPUPtr;
} // END_OF_NAMESPACE____

#endif


