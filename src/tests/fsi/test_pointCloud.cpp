// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// ChronoParallel test program using DEM method for frictional contact.
//
//
// The global reference frame has Z up.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>
#include <fstream>

#include "chrono_parallel/physics/ChSystemParallel.h"

#ifdef CHRONO_OPENGL
#undef CHRONO_OPENGL
#endif
#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

int objId = 0;

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddObj(ChSystemParallel* sys, ChVector<> pos = ChVector<>(0,0,0), ChQuaternion<> rot = QUNIT);
void AddObj(ChSystemParallel* sys, ChVector<> pos, ChQuaternion<> rot) {

    ChSharedPtr<ChBody> objBody(new ChBody(new ChCollisionModelParallel, ChMaterialSurfaceBase::DEM));

    objBody->SetIdentifier(1);
    objBody->SetMass(1);
    objBody->SetPos(pos);
    objBody->SetRot(rot);
    objBody->SetBodyFixed(true);
    objBody->SetCollide(true);
    objBody->SetId(objId);

	chrono::geometry::ChTriangleMeshConnected obj_mesh;
    chrono::collision::ChConvexDecompositionHACDv2 obj_convex;
	std::string obj_file("hmmwv_chassis_simple.obj");

    objBody->GetCollisionModel()->ClearModel();

    utils::LoadConvexMesh(obj_file, obj_mesh, obj_convex);
//    utils::AddConvexCollisionModel(objBody, obj_mesh,obj_convex, ChVector<>(0,0,1), QUNIT, false);
    utils::AddConvexCollisionModel(objBody, obj_mesh,obj_convex);
//    utils::AddConvexCollisionModel(objBody, obj_mesh,obj_convex, pos,rot, true);
    objBody->GetCollisionModel()->BuildModel();
    sys->AddBody(objBody);
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddBalls(ChSystemParallel* sys, ChVector<> cMin, ChVector<> cMax, real h) {
	real r = 0.1 * h;
	int nX = (cMax.x - cMin.x) / h;
	int nY = (cMax.y - cMin.y) / h;
	int nZ = (cMax.z - cMin.z) / h;

	int numPoints = sys->Get_bodylist()->size() - 1;
	for (int i = 0; i < 1; i++) {
		for (int j = 0; j < 1; j++) {
			for (int k = 0; k < 1; k++) {
//	for (int i = 0; i < nX+1; i++) {
//		for (int j = 0; j < nY+1; j++) {
//			for (int k = 0; k < nZ+1; k++) {
				numPoints ++;
				ChVector<> ballPos = cMin + h * ChVector<>(i, j, k);
				ChSharedPtr<ChBody> pointObj(new ChBody(new ChCollisionModelParallel, ChMaterialSurfaceBase::DEM));
				pointObj->SetPos(ballPos);
	            pointObj->SetBodyFixed(false);
	            pointObj->SetCollide(true);
	            pointObj->SetId(numPoints);
	            pointObj->GetCollisionModel()->ClearModel();
	            utils::AddSphereGeometry(pointObj.get_ptr(), r);
	            pointObj->GetCollisionModel()->SetFamily(10);
	            pointObj->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(10);
	            pointObj->GetCollisionModel()->BuildModel();
	            sys->AddBody(pointObj);
			}
		}
	}

	std::cout << "numPoints: " << numPoints << std::endl;
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void GetMargin(ChSystemParallel* sys, ChVector<> & cMin, ChVector<> & cMax) {
	host_vector<real3> & mData = sys->data_manager->host_data.convex_data;
	cMin = ChVector<>(1e10, 1e10, 1e10);
	cMax = -1.0 * cMin;
	for (int i = 0; i < mData.size(); i ++) {
		if (cMin.x > mData[i].x) cMin.x = mData[i].x;
		if (cMin.y > mData[i].y) cMin.y = mData[i].y;
		if (cMin.z > mData[i].z) cMin.z = mData[i].z;

		if (cMax.x < mData[i].x) cMax.x = mData[i].x;
		if (cMax.y < mData[i].y) cMax.y = mData[i].y;
		if (cMax.z < mData[i].z) cMax.z = mData[i].z;
	}
}
// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void PrintPointCloud(ChSystemParallel* sys) {
	int numContacts =
			sys->data_manager->host_data.dpth_rigid_rigid.size();
	printf("nuim contacts %d \n", numContacts);
	std::ofstream fileOut("PC.csv");
	for (int i = 0; i < numContacts; i++) {
		real depth = -1
				* sys->data_manager->host_data.dpth_rigid_rigid[i]; // depth is a negative number when the two objects are in contact.
		if (depth < 0) {
			continue;
		}
		chrono::int2 ids =
				sys->data_manager->host_data.bids_rigid_rigid[i];
		ChSharedPtr<ChBody> body1 = sys->Get_bodylist()->at(ids.x);
		ChSharedPtr<ChBody> body2 = sys->Get_bodylist()->at(ids.y);
		ChSharedPtr<ChBody> otherBody;
		if (body1->GetId() == objId) {
			otherBody = body2;
		} else if (body2->GetId() == objId) {
			otherBody = body1;
		} else {
			continue;
		}

//		(ids.x < ids.y) ? printf("ids %d %d \n", ids.x, ids.y) : printf("ids %d %d \n", ids.y, ids.x);
		ChVector<> p3 = otherBody->GetPos();
		fileOut << p3.x << ", " << p3.y << ", " << p3.z << std::endl;
	}
}
// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    int threads = 1;

    // Simulation parameters
    // ---------------------

    double gravity = 0;//9.81;
    double time_step = 1e-3;
    double time_end = 2;

    double out_fps = 50;

    uint max_iteration = 100;
    real tolerance = 1e-3;

    // Create system
    // -------------

    ChSystemParallelDEM msystem;

    // Set number of threads.
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    msystem.SetParallelThreadNumber(threads);
    CHOMPfunctions::SetNumThreads(threads);

    // Set gravitational acceleration
    msystem.Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    msystem.GetSettings()->solver.tolerance = tolerance;

    msystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
    msystem.GetSettings()->collision.bins_per_axis = I3(10, 10, 10);

    // The following two lines are optional, since they are the default options. They are added for future reference,
    // i.e. when needed to change those models.
    msystem.GetSettings()->solver.contact_force_model = ChSystemDEM::ContactForceModel::Hertz;
    msystem.GetSettings()->solver.adhesion_force_model = ChSystemDEM::AdhesionForceModel::Constant;

    // Create the fixed and moving bodies
    // ----------------------------------
    AddObj(&msystem);

    // Perform the simulation
// ----------------------

//    msystem.ComputeCollisions();
    ChVector<> cMin(-2.459156, -1.281675, -0.081883);
    ChVector<> cMax(2.211580, 1.281759, 1.569679);

//    ChVector<> cMin, cMax;
//    GetMargin(&msystem, cMin, cMax);
//    printf("cMin %f %f %f cMax %f %f %f size %d\n", cMin.x, cMin.y, cMin.z, cMax.x, cMax.y, cMax.z, msystem.Get_bodylist()->size());
//
//    printf("bbBox size %d %d convex %d\n", msystem.data_manager->host_data.aabb_min_rigid.size(), msystem.data_manager->host_data.aabb_max_rigid.size(), msystem.data_manager->host_data.convex_data.size());
    AddBalls(&msystem, cMin, cMax, 0.6);

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "ballsDEM", &msystem);
    gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Uncomment the following two lines for the OpenGL manager to automatically
    // run the simulation in an infinite loop.
    // gl_window.StartDrawLoop(time_step);
    // return 0;

    while (true) {
        if (gl_window.Active()) {
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
        } else {
            break;
        }
    }
#else
    // Run simulation for specified time
    msystem.ComputeCollisions();
    PrintPointCloud(&msystem);
#endif

    return 0;
}
