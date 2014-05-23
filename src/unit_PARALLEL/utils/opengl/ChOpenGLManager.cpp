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
// Implementation of GL manager class based on GlutMaster code by George Stetten 
// and Korin Crawford.
// Authors: Hammad Mazhar
// =============================================================================

#include "ChOpenGLManager.h"
#include "ChOpenGLWindow.h"

using namespace chrono;
using namespace chrono::utils;

ChOpenGLWindow * viewPorts[MAX_NUMBER_OF_WINDOWS];

int ChOpenGLManager::currentIdleWindow   = 0;
int ChOpenGLManager::idleFunctionEnabled = 0;


ChOpenGLManager::ChOpenGLManager(){

   // Create dummy variables
	char * dummy_argv[1];
	dummy_argv[0] = "run";
	int dummy_argc = 1;

   // Initialize GLUT
	glutInit(&dummy_argc, dummy_argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
}

ChOpenGLManager::~ChOpenGLManager(){
}

void ChOpenGLManager::CallBackDisplayFunc(void) {
	int windowID = glutGetWindow();
	viewPorts[windowID]->CallBackDisplayFunc();
}

void ChOpenGLManager::CallBackIdleFunc(void) {
	if (idleFunctionEnabled && currentIdleWindow) {
		glutSetWindow(currentIdleWindow);
		viewPorts[currentIdleWindow]->CallBackIdleFunc();
	}
}

void ChOpenGLManager::CallBackKeyboardFunc(unsigned char key, int x, int y) {
	int windowID = glutGetWindow();
	viewPorts[windowID]->CallBackKeyboardFunc(key, x, y);
}

void ChOpenGLManager::CallBackMotionFunc(int x, int y) {
	int windowID = glutGetWindow();
	viewPorts[windowID]->CallBackMotionFunc(x, y);
}

void ChOpenGLManager::CallBackMouseFunc(int button, int state, int x, int y) {
	int windowID = glutGetWindow();
	viewPorts[windowID]->CallBackMouseFunc(button, state, x, y);
}

void ChOpenGLManager::CallBackPassiveMotionFunc(int x, int y) {
	int windowID = glutGetWindow();
	viewPorts[windowID]->CallBackPassiveMotionFunc(x, y);
}

void ChOpenGLManager::CallBackReshapeFunc(int w, int h) {
	int windowID = glutGetWindow();
	viewPorts[windowID]->CallBackReshapeFunc(w, h);
}

void ChOpenGLManager::CallBackSpecialFunc(int key, int x, int y) {
	int windowID = glutGetWindow();
	viewPorts[windowID]->CallBackSpecialFunc(key, x, y);
}

void ChOpenGLManager::CallBackVisibilityFunc(int visible) {
	int windowID = glutGetWindow();
	viewPorts[windowID]->CallBackVisibilityFunc(visible);
}

void ChOpenGLManager::CallGlutCreateWindow(char * setTitle, ChOpenGLWindow * glutWindow) {
	
	// Open new window, record its windowID ,
	int windowID = glutCreateWindow(setTitle);

	glutWindow->SetWindowID(windowID);

	// Store the address of new window in global array
	// so GlutMaster can send events to propoer callback functions.
	viewPorts[windowID] = glutWindow;

	// Hand address of universal static callback functions to Glut.
	// This must be for each new window, even though the address are constant.

	glutDisplayFunc(CallBackDisplayFunc);
	glutIdleFunc(CallBackIdleFunc);
	glutKeyboardFunc(CallBackKeyboardFunc);
	glutSpecialFunc(CallBackSpecialFunc);
	glutMouseFunc(CallBackMouseFunc);
	glutMotionFunc(CallBackMotionFunc);
	glutPassiveMotionFunc(CallBackPassiveMotionFunc);
	glutReshapeFunc(CallBackReshapeFunc);
	glutVisibilityFunc(CallBackVisibilityFunc);
}

void ChOpenGLManager::CallGlutMainLoop(void) {
	glutMainLoop();
}

void ChOpenGLManager::DisableIdleFunction(void) {
	idleFunctionEnabled = 0;
}

void ChOpenGLManager::EnableIdleFunction(void) {
	idleFunctionEnabled = 1;
}

int ChOpenGLManager::IdleFunctionEnabled(void) {
	// Is idle function enabled?
	return (idleFunctionEnabled);
}

int ChOpenGLManager::IdleSetToCurrentWindow(void) {
	// Is current idle window same as current window?
	return (currentIdleWindow == glutGetWindow());
}

void ChOpenGLManager::SetIdleToCurrentWindow(void) {
	currentIdleWindow = glutGetWindow();
}
