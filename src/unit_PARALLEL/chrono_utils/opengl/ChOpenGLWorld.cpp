// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Implementation of OpenGL class 
// Authors: Hammad Mazhar
// =============================================================================

#include "ChOpenGLWorld.h"
using namespace chrono;
using namespace chrono::utils;

ChOpenGLWorld::ChOpenGLWorld() {

	render_camera.SetMode(FREE);
	render_camera.SetPosition(glm::vec3(0, 0, -10));
	render_camera.SetLookAt(glm::vec3(0, 0, 0));
	render_camera.SetClipping(.1, 1000);
	render_camera.SetFOV(45);

	simulation_frame = 0;
	simulation_time = 0;

}

ChOpenGLWorld::~ChOpenGLWorld() {

}

bool ChOpenGLWorld::Initialize(){
	main_shader.Initialize("phong.vert", "phong.frag");
	ChOpenGLMaterial white_material(glm::vec3(0, 0, 0), glm::vec3(1.0, 1.0, 1.0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0));
	sphere.Initialize(5,5, white_material);
}


void ChOpenGLWorld::RunStep(){
	if (simulation_state == RUNNING) {
		simulation_h = physics_system->GetStep();
		physics_system->DoStepDynamics(simulation_h);
		simulation_frame++;
		simulation_time += simulation_h;
	}

	render_camera.Update();

	render_camera.GetMatricies(projection, view, model);
	glm::mat4 mvp = projection * view * model;     //Compute the mvp matrix

	




	

}

void ChOpenGLWorld::DrawSphere(ChVector<> pos, float rad, float angle, ChVector<> axis, ChVector<> scale){}
void ChOpenGLWorld::DrawBox(ChVector<> pos, ChVector<> radius, float angle, ChVector<> axis, ChVector<> scale){}
void ChOpenGLWorld::DrawCyl(ChVector<> pos, double rad, float angle, ChVector<> axis, ChVector<> scale){}
void ChOpenGLWorld::DrawCone(ChVector<> pos, ChVector<> radius, float angle, ChVector<> axis){}
