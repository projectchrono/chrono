// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Hammad Mazhar
// =============================================================================
//
// Unit test for calculation of cumulative contact forces on a body.
// The test checks that the cumulative contact force on a container body (fixed
// to ground) is equal to the sum of the weights of several bodies dropped in
// the container.
//
// =============================================================================

#include <iostream>
#include <cstdlib>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::utils;
// ====================================================================================

// Forward declaration
bool test_collisionsystem();

// ====================================================================================

int main(int argc, char* argv[]) {

	bool passed = test_collisionsystem();

    // Return 0 if all tests passed.
    return !passed;
}

// ====================================================================================
bool check_collisionsystemcount(ChSystemParallelNSC* system, const int count) {
	ChParallelDataManager* data_manager = system->data_manager;

	if (count != data_manager->shape_data.ObA_rigid.size()) { std::cout << "ObA_rigid " << data_manager->shape_data.ObA_rigid.size() << std::endl; return false; }
	if (count != data_manager->shape_data.ObR_rigid.size()) { std::cout << "ObR_rigid " << data_manager->shape_data.ObR_rigid.size() << std::endl; return false; }
	if (count != data_manager->shape_data.start_rigid.size()) { std::cout << "start_rigid" << data_manager->shape_data.start_rigid.size() << std::endl; return false; }
	if (count != data_manager->shape_data.length_rigid.size()) { std::cout << "length_rigid" << data_manager->shape_data.length_rigid.size() << std::endl; return false; }

	if (count != data_manager->shape_data.fam_rigid.size()) { std::cout << "fam_rigid" << data_manager->shape_data.fam_rigid.size() << std::endl; return false; }
	if (count != data_manager->shape_data.typ_rigid.size()) { std::cout << "typ_rigid" << data_manager->shape_data.typ_rigid.size() << std::endl; return false; }
	if (count != data_manager->shape_data.id_rigid.size()) { std::cout << "id_rigid" << data_manager->shape_data.id_rigid.size() << std::endl; return false; }

	if (count != data_manager->shape_data.sphere_rigid.size()) { std::cout << "sphere_rigid" << data_manager->shape_data.sphere_rigid.size() << std::endl; return false; }
	if (count != data_manager->shape_data.box_like_rigid.size()) { std::cout << "box_like_rigid" << data_manager->shape_data.box_like_rigid.size() << std::endl; return false; }
	if (count != data_manager->shape_data.capsule_rigid.size()) { std::cout << "capsule_rigid" << data_manager->shape_data.capsule_rigid.size() << std::endl; return false; }
	if (count != data_manager->shape_data.rbox_like_rigid.size()) { std::cout << "rbox_like_rigid" << data_manager->shape_data.rbox_like_rigid.size() << std::endl; return false; }
	if (count != data_manager->shape_data.convex_rigid.size()) { std::cout << "convex_rigid" << data_manager->shape_data.convex_rigid.size() << std::endl; return false; }
	return true;

}
bool test_collisionsystem() {
	//create parallel system
	ChSystemParallelNSC* system = new ChSystemParallelNSC();
	bool passed = true;
	auto material = std::make_shared<ChMaterialSurfaceNSC>();

	//add a chbody with a sphere model
	std::shared_ptr<ChBody> sphere_model = std::make_shared<ChBody>(new ChCollisionModelParallel);
	InitializeObject(sphere_model, 1, material, Vector(0, 0, 0), Quaternion(1, 0, 0, 0), true, true, 0, 0);
	AddSphereGeometry(sphere_model.get(), 1, Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
	FinalizeObject(sphere_model, (ChSystemParallel*)system);
	//remove sphere collision model
	sphere_model->RemoveCollisionModelsFromSystem();
	sphere_model->GetCollisionModel()->ClearModel();
	if (check_collisionsystemcount(system, 0) == false) { 
		std::cout << "Failed, removing single sphere." << std::endl;
		return false; }


	//add chbody with two collision models
	std::shared_ptr<ChBody> double_model = std::make_shared<ChBody>(new ChCollisionModelParallel);
	InitializeObject(double_model, 1, material, Vector(0, 0, 0), Quaternion(1, 0, 0, 0), true, true, 0, 0);
	AddSphereGeometry(double_model.get(), 1, Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
	AddSphereGeometry(double_model.get(), 1, Vector(0, 0, 0), Quaternion(1, 0, 0, 0));
	FinalizeObject(double_model, (ChSystemParallel*)system);
	//remove 
	double_model->RemoveCollisionModelsFromSystem();
	double_model->GetCollisionModel()->ClearModel();
	if (check_collisionsystemcount(system, 0) == false) {
		std::cout << "Failed, removing double sphere." << std::endl;
		return false; }

    delete system;
    return passed;
}
