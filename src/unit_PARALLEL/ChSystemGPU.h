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

#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeSymmSOR.h"
#include "lcp/ChLcpIterativeSORmultithread.h"
#include "lcp/ChLcpIterativeJacobi.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "lcp/ChLcpSolverDEM.h"

#include "core/ChTimer.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "collision/ChCModelBulletBody.h"
using namespace chrono;

namespace chrono
{

class ChApiGPU ChSystemGPU : public ChSystem
{
public:
	ChSystemGPU (unsigned int max_objects = 16000, double scene_size = 500): ChSystem(max_objects,scene_size){}
	virtual int Integrate_Y_impulse_Anitescu();
};

}

#endif
