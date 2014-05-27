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

#include "ChOpenGLViewer.h"

using namespace glm;
using namespace chrono;
using namespace chrono::utils;

ChOpenGLViewer::ChOpenGLViewer(ChOpenGLManager * window_manager, ChSystem * system, ivec2 size, ivec2 position, char * title) {
	window_size = size;
	window_position = position;

	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(window_size.x, window_size.y);
	glutInitWindowPosition(window_position.x, window_position.y);
	window_manager->CallGlutCreateWindow(title, this);
	glViewport(0, 0, window_size.x, window_size.y);     // This may have to be moved to after the next line on some platforms
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glEnable( GL_POINT_SMOOTH );
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (glewInit() != GLEW_OK) {
        cerr << "GLEW failed to initialize." << endl;
        exit(0);
    }

    main_world.Initialize();
}

ChOpenGLViewer::~ChOpenGLViewer() {

}

void ChOpenGLViewer::CallBackDisplayFunc(void) {
	glEnable(GL_CULL_FACE);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, window_size.x, window_size.y);
	//main_world.RunStep();
	glutSwapBuffers();
}

void ChOpenGLViewer::CallBackReshapeFunc(int w, int h) {

	if (h > 0) {
		window_size = glm::ivec2(w, h);
		window_aspect = float(w) / float(h);
	}
	main_world.render_camera.SetViewport(0, 0, window_size.x, window_size.y);
	CallBackDisplayFunc();
}

void ChOpenGLViewer::CallBackIdleFunc(void) {

	CallBackDisplayFunc();
}
void ChOpenGLViewer::CallBackKeyboardFunc(unsigned char key, int x, int y) {
	//printf("%f,%f,%f\n", render_camera->camera_pos.x, render_camera->camera_pos.y, render_camera->camera_pos.z);
	switch (key) {
		case 'w':
		main_world.render_camera.Move(FORWARD);
		break;
		case 's':
		main_world.render_camera.Move(BACK);
		break;
		case 'd':
		main_world.render_camera.Move(RIGHT);
		break;
		case 'a':
		main_world.render_camera.Move(LEFT);
		break;
		case 'q':
		main_world.render_camera.Move(DOWN);
		break;
		case 'e':
		main_world.render_camera.Move(UP);
		break;
		case 'x':
		case 27:
		exit(0);
		return;
		default:
		break;
	}

}
void ChOpenGLViewer::CallBackMouseFunc(int button, int state, int x, int y) {
	main_world.render_camera.SetPos(button, state, x, y);
}
void ChOpenGLViewer::CallBackMotionFunc(int x, int y) {
	main_world.render_camera.Move2D(x, y);
}

void ChOpenGLViewer::StartSpinning(ChOpenGLManager * window_manager) {
	
	window_manager->SetIdleToCurrentWindow();
	window_manager->EnableIdleFunction();
}



