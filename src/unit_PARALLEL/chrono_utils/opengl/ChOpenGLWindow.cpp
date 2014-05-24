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
// Implementation of GL window class based on GlutMaster code by George Stetten 
// and Korin Crawford.
// Authors: Hammad Mazhar
// =============================================================================
#include "ChOpenGLWindow.h"
using namespace chrono;
using namespace chrono::utils;

ChOpenGLWindow::ChOpenGLWindow(void){}
ChOpenGLWindow::~ChOpenGLWindow(){}

void ChOpenGLWindow::CallBackDisplayFunc(void){}														//dummy function
void ChOpenGLWindow::CallBackIdleFunc(void){}															//dummy function
void ChOpenGLWindow::CallBackKeyboardFunc(unsigned char key, int x, int y){key; x; y;}					//dummy function
void ChOpenGLWindow::CallBackMotionFunc(int x, int y){x; y;}											//dummy function
void ChOpenGLWindow::CallBackMouseFunc(int button, int state, int x, int y){button; state; x; y;      }	//dummy function
void ChOpenGLWindow::CallBackPassiveMotionFunc(int x, int y){x; y;}										//dummy function
void ChOpenGLWindow::CallBackReshapeFunc(int w, int h){w; h;}											//dummy function
void ChOpenGLWindow::CallBackSpecialFunc(int key, int x, int y){key; x; y;}								//dummy function
void ChOpenGLWindow::CallBackVisibilityFunc(int visible){visible;}										//dummy function
void ChOpenGLWindow::SetWindowID(int newWindowID){windowID = newWindowID;}
int  ChOpenGLWindow::GetWindowID(void){return( windowID );}
