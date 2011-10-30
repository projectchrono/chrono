#ifndef CHBODYGPU_H
#define CHBODYGPU_H

//////////////////////////////////////////////////
//
//   ChBodyGPU.h
//
//   Derived Class for GPU rigid bodies, 
//   
//   
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "physics/ChBody.h"
#include "ChApiGPU.h"
#include "ChCCollisionModelGPU.h"
#include "ChGPUDataManager.h"
namespace chrono {

	using namespace collision;

	class ChSystem;
	class ChApiGPU ChBodyGPU: public ChBody {

		CH_RTTI(ChBodyGPU,ChPhysicsItem)
			;

		public:

			//
			// CONSTRUCTORS
			//
			/// Build a rigid body.
			ChBodyGPU();
			/// Destructor
			~ChBodyGPU();
			virtual ChCollisionModel* InstanceCollisionModel();
			void UpdateForces(double mytime);
			void SetBodyFixed(bool mev);

			ChVector<> GetPos() {
				if (data_manager != 0) {
					float3 pos = data_manager->host_pos_data[id];
					return ChVector<> (pos.x, pos.y, pos.z);
				}

				return coord.pos;
			}
			ChQuaternion<> GetRot() {
				if (data_manager != 0) {
					float4 rot = data_manager->host_rot_data[id];
					return ChQuaternion<> (rot.x, rot.y, rot.z, rot.w);
				}
				return coord.rot;

			}
			ChVector<> GetPos_dt() {
				if (data_manager != 0) {

					float3 vel = data_manager->host_vel_data[id];
					return ChVector<> (vel.x, vel.y, vel.z);
				}
				return coord_dt.pos;

			}
			ChVector<> GetPos_dtdt() {
				if (data_manager != 0) {
					float3 acc = data_manager->host_acc_data[id];
					return ChVector<> (acc.x, acc.y, acc.z);
				}

				return coord_dtdt.pos;
			}

			virtual void SetPos_dt(const ChVector<>& mvel) {
				coord_dt.pos = mvel;
				if (data_manager != 0) {
					data_manager->host_vel_data[id].x = mvel.x;
					data_manager->host_vel_data[id].y = mvel.y;
					data_manager->host_vel_data[id].z = mvel.z;
				}
			}

			ChVector<> GetXForce() {
				return Xforce;
			}
			ChVector<> GetXTorque() {
				return Xtorque;
			}
			ChVector<> GetGyro() {
				return gyro;
			}

			int id;

			vector<float2> interDist;

			ChGPUDataManager *data_manager;

	};

	typedef ChSharedPtr<ChBodyGPU> ChSharedBodyGPUPtr;

} // END_OF_NAMESPACE____


#endif
