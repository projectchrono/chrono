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

#ifndef CHOPENGLWORLD_H
#define CHOPENGLWORLD_H
#include "core/ChVector.h"
#include "physics/ChSystem.h"
#include "utils/opengl/ChOpenGLCamera.h"
#include "utils/opengl/core/ChOpenGLShader.h"
#include "utils/opengl/shapes/ChOpenGLSphere.h"
#include "utils/opengl/shapes/ChOpenGLCloud.h"
namespace chrono {
	namespace utils{

		enum SimState{PAUSED, RUNNING};

		class CH_UTILS_OPENGL_API ChOpenGLWorld: public ChOpenGLBase {
		public:
			ChOpenGLWorld();
			~ChOpenGLWorld();
			void TakeDown(){}
			bool Initialize();
			void RunStep();
			void DrawSphere(ChVector<> pos, float rad, float angle, ChVector<> axis, ChVector<> scale);
			void DrawBox(ChVector<> pos, ChVector<> radius, float angle, ChVector<> axis, ChVector<> scale);
			void DrawCyl(ChVector<> pos, double rad, float angle, ChVector<> axis, ChVector<> scale);
			void DrawCone(ChVector<> pos, ChVector<> radius, float angle, ChVector<> axis);
			ChOpenGLCamera render_camera;
			ChSystem * physics_system;
			SimState simulation_state;

			ChOpenGLShader main_shader;

			ChOpenGLSphere sphere;
			ChOpenGLCloud cloud;

			int simulation_frame; // The current frame number
			float simulation_h; // The simulation step size
			float simulation_time; // The current simulation time

			glm::mat4 model, view, projection,modelview;
		};
	}
}


#endif  // END of CHOPENGLWORLD_H
