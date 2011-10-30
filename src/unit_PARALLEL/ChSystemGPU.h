#ifndef CH_SYSTEMGPU_H
#define CH_SYSTEMGPU_H

#include "../physics/ChSystem.h"
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
using namespace chrono;

namespace chrono {

#define Bpointer		    (*ibody)
#define HIER_BODY_INIT      std::vector<ChBody*>::iterator ibody = bodylist.begin();
#define HIER_BODY_NOSTOP    (ibody != bodylist.end())
#define HIER_BODY_NEXT	    ibody++;

#define Lpointer		    (*iterlink)
#define HIER_LINK_INIT      std::list<ChLink*>::iterator iterlink = linklist.begin();
#define HIER_LINK_NOSTOP    (iterlink != linklist.end())
#define HIER_LINK_NEXT	    iterlink++;

#define PHpointer		    (*iterotherphysics)
#define HIER_OTHERPHYSICS_INIT    std::list<ChPhysicsItem*>::iterator iterotherphysics = otherphysicslist.begin();
#define HIER_OTHERPHYSICS_NOSTOP  (iterotherphysics != otherphysicslist.end())
#define HIER_OTHERPHYSICS_NEXT	  iterotherphysics++;

#define Ppointer		    (*iterprobe)
#define HIER_PROBE_INIT      std::vector<ChProbe*>::iterator iterprobe = probelist.begin();
#define HIER_PROBE_NOSTOP    (iterprobe != probelist.end())
#define HIER_PROBE_NEXT	    iterprobe++;

#define Cpointer		    (*itercontrol)
#define HIER_CONTROLS_INIT      std::vector<ChControls*>::iterator itercontrol = controlslist.begin();
#define HIER_CONTROLS_NOSTOP    (itercontrol != controlslist.end())
#define HIER_CONTROLS_NEXT	    itercontrol++;

	class ChApiGPU ChSystemGPU: public ChSystem {
		public:
			ChSystemGPU(unsigned int max_objects = 16000, double scene_size = 500);
			virtual int Integrate_Y_impulse_Anitescu();
			double ComputeCollisions();
			void AddBody(ChSharedPtr<ChBody> newbody);
			void RemoveBody(ChSharedPtr<ChBody> mbody);
			void Update();
			virtual void LCPprepare_load(      bool load_jacobians, ///< load jacobians into ChConstraints
										   bool load_v,			///< load v_old (current speeds) in q (to use LCP solver with option 'add_Mq_to_f')
										   double F_factor, 	///< load F (forces) in fb: fb+=F*F_factor
										   double Ct_factor,	///< load Ct into bi:  bi+= Ct*Ct_factor
										   double C_factor,		///< load C  into bi:  bi+= C*C_factor, otherwise..
										   double recovery_clamp,///< if do_clamp=true,  bi+= min(C*C_factor, recovery_clamp);
										   bool do_clamp		///< if true, limit the recovery of constraint drifting
									);
			virtual void LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor);
			virtual void LCPprepare_reset();
			void ChangeCollisionSystem(ChCollisionSystem* newcollsystem);
			void ChangeLcpSolverSpeed(ChLcpSolver* newsolver);
			float ComputeKineticEnergy();
			ChGPUDataManager *gpu_data_manager;
		private:
			unsigned int counter;
			unsigned int max_obj;
			bool copydata;
	};

}

#endif
