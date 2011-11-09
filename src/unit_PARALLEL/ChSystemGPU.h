#ifndef CH_SYSTEMGPU_H
#define CH_SYSTEMGPU_H

#include "ChCuda.h"
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>
#include <algorithm>

#include "physics/ChSystem.h"
#include "physics/ChGlobal.h"
#include "physics/ChCollide.h"
#include "physics/ChContactContainer.h"
#include "physics/ChProximityContainerBase.h"

#include "ChLcpIterativeSolverGPUsimple.h"
#include "ChLcpSystemDescriptorGPU.h"

#include "core/ChTimer.h"
#include "ChForceSystemGPU.h"
#include "ChGPUDataManager.h"
#include "ChCCollisionSystemGPU.h"

namespace chrono {
	using namespace chrono;

	class ChApiGPU ChSystemGPU: public ChSystem {

		CH_RTTI(ChSystemGPU,ChObj)
			;

		public:
			ChSystemGPU(unsigned int max_objects = 16000, double scene_size = 500);
			virtual int Integrate_Y_impulse_Anitescu();
			double ComputeCollisions();
			void AddBody(ChSharedPtr<ChBodyGPU> newbody) {

				newbody->AddRef();
				newbody->SetSystem(this);
				bodylist.push_back((newbody).get_ptr());

				ChBodyGPU* gpubody = ((ChBodyGPU*) newbody.get_ptr());
				gpubody->id = counter;
				gpubody->data_manager = gpu_data_manager;
				if (newbody->GetCollide()) newbody->AddCollisionModelsToSystem();

				ChLcpVariablesBodyOwnMass* mbodyvar = &(newbody->Variables());

				float inv_mass = (1.0) / (mbodyvar->GetBodyMass());
				newbody->GetRot().Normalize();
				gpu_data_manager->host_vel_data.push_back(F3(mbodyvar->Get_qb().GetElementN(0), mbodyvar->Get_qb().GetElementN(1), mbodyvar->Get_qb().GetElementN(2)));
				gpu_data_manager->host_omg_data.push_back(F3(mbodyvar->Get_qb().GetElementN(3), mbodyvar->Get_qb().GetElementN(4), mbodyvar->Get_qb().GetElementN(5)));
				gpu_data_manager->host_pos_data.push_back(F3(newbody->GetPos().x, newbody->GetPos().y, newbody->GetPos().z));
				gpu_data_manager->host_rot_data.push_back(F4(newbody->GetRot().e0, newbody->GetRot().e1, newbody->GetRot().e2, newbody->GetRot().e3));
				gpu_data_manager->host_inr_data.push_back(F3(mbodyvar->GetBodyInvInertia().GetElement(0, 0), mbodyvar->GetBodyInvInertia().GetElement(1, 1), mbodyvar->GetBodyInvInertia().GetElement(
						2, 2)));
				gpu_data_manager->host_frc_data.push_back(F3(mbodyvar->Get_fb().ElementN(0), mbodyvar->Get_fb().ElementN(1), mbodyvar->Get_fb().ElementN(2))); //forces
				gpu_data_manager->host_trq_data.push_back(F3(mbodyvar->Get_fb().ElementN(3), mbodyvar->Get_fb().ElementN(4), mbodyvar->Get_fb().ElementN(5))); //torques
				gpu_data_manager->host_aux_data.push_back(F3(newbody->IsActive(), newbody->GetKfriction(), inv_mass));
				gpu_data_manager->host_lim_data.push_back(F3(newbody->GetLimitSpeed(), newbody->GetMaxSpeed(), newbody->GetMaxWvel()));

				counter++;
				gpu_data_manager->number_of_objects = counter;
				if (counter % 1000 == 0) {
					cout << "Added: " << counter << " objects" << endl;
				}
			}
			void RemoveBody(ChSharedPtr<ChBodyGPU> mbody);
			//void Update();
			//virtual void LCPprepare_load(      bool load_jacobians, ///< load jacobians into ChConstraints
			//							   bool load_v,			///< load v_old (current speeds) in q (to use LCP solver with option 'add_Mq_to_f')
			//							   double F_factor, 	///< load F (forces) in fb: fb+=F*F_factor
			///							   double Ct_factor,	///< load Ct into bi:  bi+= Ct*Ct_factor
			//							   double C_factor,		///< load C  into bi:  bi+= C*C_factor, otherwise..
			//							   double recovery_clamp,///< if do_clamp=true,  bi+= min(C*C_factor, recovery_clamp);
			//							   bool do_clamp		///< if true, limit the recovery of constraint drifting
			//						);
			//virtual void LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor);
			//virtual void LCPprepare_reset();
			void ChangeCollisionSystem(ChCollisionSystem* newcollsystem);
			void ChangeLcpSolverSpeed(ChLcpSolver* newsolver);
			float GetKineticEnergy() {
				((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->Total_KineticEnergy();
			}
			ChGPUDataManager *gpu_data_manager;
		private:
			unsigned int counter;
			unsigned int max_obj;
			bool copydata;
	};

}

#endif
