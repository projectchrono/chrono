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

#include "ChOpenGL.h"
using namespace chrono;
using namespace chrono::utils;

GLfloat whiteSpecularLight[] = { 1.0, 1.0, 1.0 };     //set the light specular to white
GLfloat blackAmbientLight[] = { 0.0, 0.0, 0.0 };     //set the light ambient to black
GLfloat whiteDiffuseLight[] = { 1.0, 1.0, 1.0 };     //set the diffuse light to white

ChOpenGL::ChOpenGL(ChOpenGLManager * window_manager, ChSystem * system, int setWidth, int setHeight, int setInitPositionX, int setInitPositionY, char * title) {
	physics_system.push_back(system);
	render_camera = new ChOpenGLCamera();

	render_camera->SetMode(FREE);
	render_camera->SetPosition(glm::vec3(0, 0, -10));
	render_camera->SetLookAt(glm::vec3(0, 0, 0));
	render_camera->SetClipping(.1, 1000);
	render_camera->SetFOV(45);

	display_mode = POINT_CLOUD;
	text_mode = NONE;
	sim_mode = RUNNING;
	render_mode = RUNNING;
	contact_mode = NOCONTACT;

	sim_frame = 0;
	sim_time = 0;

	width = setWidth;
	height = setHeight;

	initPositionX = setInitPositionX;
	initPositionY = setInitPositionY;

	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(width, height);
	glutInitWindowPosition(initPositionX, initPositionY);

	window_manager->CallGlutCreateWindow(title, this);
	glViewport(0, 0, width, height);     // This may have to be moved to after the next line on some platforms

	glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel (GL_SMOOTH);
	glEnable (GL_COLOR_MATERIAL);

	//glEnable (GL_POINT_SMOOTH);
	//glHint(GL_POINT_SMOOTH_HINT, GL_DONT_CARE);
	glPointSize(2);
	glEnable (GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable (GL_DEPTH_TEST);
	//glDepthFunc(GL_LESS);
	//glFrontFace(GL_CCW);
	glCullFace (GL_BACK);
	glEnable (GL_CULL_FACE);
	//glDepthFunc( GL_LEQUAL);
	//glClearDepth(1.0);

	glEnable (GL_LIGHT0);

	set_custom_callback = false;

}

ChOpenGL::~ChOpenGL() {

	glutDestroyWindow(windowID);
}

void ChOpenGL::CallBackDisplayFunc(void) {

	if (sim_mode != PAUSED_SIM) {
		if (set_custom_callback) {
			for (int s = 0; s < physics_system.size(); s++) {
				custom_callback(physics_system[s], sim_frame);
			}
		}
		RunStep();
	}
	if (render_mode != PAUSED_RENDER) {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearDepth(1);
		glEnable (GL_DEPTH_TEST);
		glEnable (GL_POINT_SMOOTH);
		glDepthFunc (GL_LEQUAL);
		glLoadIdentity();

		if (display_mode == WIREFRAME) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glDisable (GL_LIGHTING);
		} else if (display_mode == SOLID) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glEnable (GL_LIGHTING);
		} else if (display_mode == POINT_CLOUD) {
			glDisable (GL_LIGHTING);
		}
		GLfloat light_position[] = { 100, 100.0, 100.0, 0.0 };
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);
		glLightfv(GL_LIGHT0, GL_SPECULAR, whiteSpecularLight);
		glLightfv(GL_LIGHT0, GL_AMBIENT, blackAmbientLight);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, whiteDiffuseLight);

		glm::mat4 model, view, projection;
		render_camera->Update();
		render_camera->GetMatricies(projection, view, model);
		glm::mat4 mvp = projection * view * model;     //Compute the mvp matrix
		glLoadMatrixf(glm::value_ptr(mvp));

		glColor3f(0.0, 0.0, 0.0);
		points.clear();
		for (int i = 0; i < physics_system[0]->Get_bodylist()->size(); i++) {
			ChBody* abody = (ChBody*) physics_system[0]->Get_bodylist()->at(i);
			glPushMatrix();
			DrawObject(abody);
			glPopMatrix();
		}
		if (display_mode == POINT_CLOUD) {
			//glPushMatrix();
			glColor3f(0, 0, 0);
			glEnableClientState (GL_VERTEX_ARRAY);
			glVertexPointer(3, GL_FLOAT, sizeof(real3), &points[0].x);
			glPointSize(2.0);
			glDrawArrays(GL_POINTS, 0, points.size());
			glDisableClientState(GL_VERTEX_ARRAY);
			//glPopMatrix();
		}
		DrawContacts();
		DrawGrids();
		DrawText();
		DrawHUD();
		glutSwapBuffers();
	}
}

void ChOpenGL::CallBackReshapeFunc(int w, int h) {

	width = w;
	height = h;

	render_camera->SetViewport(0, 0, width, height);
	CallBackDisplayFunc();
}

void ChOpenGL::CallBackIdleFunc(void) {

	CallBackDisplayFunc();
}
void ChOpenGL::CallBackKeyboardFunc(unsigned char key, int x, int y) {
	//printf("%f,%f,%f\n", render_camera->camera_pos.x, render_camera->camera_pos.y, render_camera->camera_pos.z);
	switch (key) {
		case 'w':
		render_camera->Move(FORWARD);
		break;
		case 's':
		render_camera->Move(BACK);
		break;
		case 'd':
		render_camera->Move(RIGHT);
		break;
		case 'a':
		render_camera->Move(LEFT);
		break;
		case 'q':
		render_camera->Move(DOWN);
		break;
		case 'e':
		render_camera->Move(UP);
		break;
		case ' ':
		if (sim_mode == RUNNING) {
			sim_mode = PAUSED_SIM;
		} else if (sim_mode == PAUSED_SIM) {
			sim_mode = RUNNING;
		}
		break;
		case 'p':
		if (render_mode == RUNNING) {
			render_mode = PAUSED_RENDER;
		} else if (render_mode == PAUSED_RENDER) {
			render_mode = RUNNING;
		}
		break;
		case 'h':
		if (sim_mode == RUNNING) {
			sim_mode = STOPPED;
		} else if (sim_mode == STOPPED) {
			sim_mode = RUNNING;
		}
		break;
		case '1':
		display_mode = POINT_CLOUD;
		break;
		case '2':
		display_mode = BOUNDING_BOX;
		break;
		case '3':
		display_mode = WIREFRAME;
		break;
		case '4':
		display_mode = SOLID;
		break;
		case 'c':
		if (contact_mode == NOCONTACT) {
			contact_mode = POINTS;
		} else if (contact_mode == POINTS) {
			contact_mode = NORMAL;
		} else if (contact_mode == NORMAL) {
			contact_mode = LAGRANGE;
		} else if (contact_mode == LAGRANGE) {
			contact_mode = NOCONTACT;
		}
		break;
		case ',':
		if (text_mode == NONE) {
			text_mode = ACC_T;
		} else if (text_mode == POS_T) {
			text_mode = NONE;
		} else if (text_mode == VEL_T) {
			text_mode = POS_T;
		} else if (text_mode == ACC_T) {
			text_mode = VEL_T;
		}
		break;
		case '.':
		if (text_mode == NONE) {
			text_mode = POS_T;
		} else if (text_mode == POS_T) {
			text_mode = VEL_T;
		} else if (text_mode == VEL_T) {
			text_mode = ACC_T;
		} else if (text_mode == ACC_T) {
			text_mode = NONE;
		}
		break;
		case GLUT_KEY_F1:

		break;
	}

}
void ChOpenGL::CallBackMouseFunc(int button, int state, int x, int y) {
	render_camera->SetPos(button, state, x, y);
}
void ChOpenGL::CallBackMotionFunc(int x, int y) {
	render_camera->Move2D(x, y);
}

void ChOpenGL::StartSpinning(ChOpenGLManager * window_manager) {
	sim_dt = physics_system[0]->GetStep();
	window_manager->SetIdleToCurrentWindow();
	window_manager->EnableIdleFunction();
}
void ChOpenGL::SetDisplayMode(displayMode mode) {
	display_mode = mode;

}

void ChOpenGL::DrawObject(ChBody * abody) {
	if (abody->GetAssets().size() == 0) {
		return;
	}

	const Vector pos = abody->GetPos();
	const Vector vel = abody->GetPos_dt();
	const Vector acc = abody->GetPos_dtdt();

	Quaternion rot = abody->GetRot();
	double angle;
	Vector axis;
	rot.Q_to_AngAxis(angle, axis);

	if (text_mode == POS_T) {
		textInfo3D temp;
		temp.pos = pos;
		temp.text = VectorToString(pos);
		text_3d.push_back(temp);
	} else if (text_mode == VEL_T) {
		textInfo3D temp;
		temp.pos = pos;
		temp.text = VectorToString(vel);
		text_3d.push_back(temp);
	} else if (text_mode == ACC_T) {
		textInfo3D temp;
		temp.pos = pos;
		temp.text = VectorToString(acc);
		text_3d.push_back(temp);
	}
	for (int i = 0; i < abody->GetAssets().size(); i++) {

		if (display_mode == POINT_CLOUD) {
			points.push_back(real3(pos.x,pos.y,pos.z));

		} else if (display_mode == BOUNDING_BOX) {
			ChSharedPtr<ChAsset> asset = abody->GetAssets().at(i);
			Vector minp, maxp;
			ChMatrix33<> Rot;
			if (asset.IsType<ChSphereShape>()) {
				((ChSphereShape *) (asset.get_ptr()))->GetSphereGeometry().GetBoundingBox(minp.x, maxp.x, minp.y, maxp.y, minp.z, maxp.z, &Rot);
			} else if (asset.IsType<ChBoxShape>()) {
				((ChBoxShape *) (asset.get_ptr()))->GetBoxGeometry().GetBoundingBox(minp.x, maxp.x, minp.y, maxp.y, minp.z, maxp.z, &Rot);
			} else if (asset.IsType<ChCylinderShape>()) {
				((ChCylinderShape *) (asset.get_ptr()))->GetCylinderGeometry().GetBoundingBox(minp.x, maxp.x, minp.y, maxp.y, minp.z, maxp.z, &Rot);
			}
			Vector c = (minp + maxp) * .5;
			Vector size = Vector(fabs(c.x - minp.x), fabs(c.y - minp.y), fabs(c.z - minp.z));

			MakeBox(pos, size, angle, axis, Vector(1, 1, 1));
		} else {
			ChSharedPtr<ChAsset> asset = abody->GetAssets().at(i);

			ChVisualization* visual_asset = ((ChVisualization *) (asset.get_ptr()));
			Vector center = visual_asset->Pos;
			center = rot.Rotate(center);
			Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
			lrot = rot % lrot;
			lrot.Normalize();
			lrot.Q_to_AngAxis(angle, axis);

			if (asset.IsType<ChSphereShape>()) {
				ChSphereShape * sphere_shape = ((ChSphereShape *) (asset.get_ptr()));
				float radius = sphere_shape->GetSphereGeometry().rad;
				glPushMatrix();
				MakeSphere(pos + center, radius, angle, axis, Vector(1, 1, 1));
				glPopMatrix();
			} else if (asset.IsType<ChEllipsoidShape>()) {

				ChEllipsoidShape * ellipsoid_shape = ((ChEllipsoidShape *) (asset.get_ptr()));
				Vector radius = ellipsoid_shape->GetEllipsoidGeometry().rad;
				glPushMatrix();
				MakeSphere(pos + center, 1, angle, axis, Vector(radius.x, radius.y, radius.z));
				glPopMatrix();
			} else if (asset.IsType<ChBoxShape>()) {
				ChBoxShape * box_shape = ((ChBoxShape *) (asset.get_ptr()));
//					Quaternion rott = chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X);
//					Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
//					lrot = lrot % rott;
//					lrot = lrot % rot;
//					lrot.Q_to_AngAxis(angle, axis);
				glPushMatrix();
				MakeBox(pos + center, box_shape->GetBoxGeometry().Size, angle, axis, Vector(1, 1, 1));
				glPopMatrix();
			} else if (asset.IsType<ChCylinderShape>()) {
				ChCylinderShape * cylinder_shape = ((ChCylinderShape *) (asset.get_ptr()));
				double rad = cylinder_shape->GetCylinderGeometry().rad;
				double height = cylinder_shape->GetCylinderGeometry().p1.y - cylinder_shape->GetCylinderGeometry().p2.y;
				Quaternion rott = chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X);
				Quaternion lrot = visual_asset->Rot.Get_A_quaternion();
				lrot = lrot % rott;
				lrot = rot % lrot;

				lrot.Q_to_AngAxis(angle, axis);
				glPushMatrix();
				MakeCyl(pos + center, rad, angle, axis, Vector(1, height, 1));
				//MakeBox(pos + center, ChVector<>(rad,height,rad), angle, axis, Vector(1, 1, 1));
				glPopMatrix();
			} else if (asset.IsType<ChConeShape>()) {
				ChConeShape * cone_shape = ((ChConeShape *) (asset.get_ptr()));
				Vector rad = cone_shape->GetConeGeometry().rad;
				glPushMatrix();
				MakeCone(pos + center, rad, angle, axis);
				glPopMatrix();
			} else if (asset.IsType<ChTriangleMeshShape>()) {
				ChTriangleMeshShape * trimesh_shape = ((ChTriangleMeshShape *) (asset.get_ptr()));
				ChTriangleMeshConnected trimesh = trimesh_shape->GetMesh();
				glPushMatrix();
				glBegin (GL_TRIANGLES);
				for (int i = 0; i < trimesh.getNumTriangles(); i++) {
					ChTriangle temptri = trimesh.getTriangle(i);
					real3 A, B, C;
					A = R3(temptri.p1.x, temptri.p1.y, temptri.p1.z);
					B = R3(temptri.p2.x, temptri.p2.y, temptri.p2.z);
					C = R3(temptri.p3.x, temptri.p3.y, temptri.p3.z);
					real4 Q = R4(rot.e0, rot.e1, rot.e2, rot.e3);

					A = quatRotate(A, Q) + R3(pos.x, pos.y, pos.z);
					B = quatRotate(B, Q) + R3(pos.x, pos.y, pos.z);
					C = quatRotate(C, Q) + R3(pos.x, pos.y, pos.z);

					glVertex3f(A.x, A.y, A.z);
					glVertex3f(B.x, B.y, B.z);
					glVertex3f(C.x, C.y, C.z);
				}
				glEnd();
				glPopMatrix();
			}
		}
	}
}
void ChOpenGL::DrawContactsGPU(ChSystemParallel* system_gpu) {
	ChCollisionSystemParallel* coll_sys = (ChCollisionSystemParallel*) system_gpu->GetCollisionSystem();

	//coll_sys->data_container->CopyContactData(DEVICE_TO_HOST);
	if (contact_mode == POINTS) {
		glBegin (GL_POINTS);

		for (int i = 0; i < coll_sys->data_container->num_contacts; i++) {
			int2 id = coll_sys->data_container->host_data.bids_rigid_rigid[i];

			real3 vN = coll_sys->data_container->host_data.norm_rigid_rigid[i];
			real3 vpA = coll_sys->data_container->host_data.cpta_rigid_rigid[i] + coll_sys->data_container->host_data.pos_data[id.x];
			real3 vpB = coll_sys->data_container->host_data.cptb_rigid_rigid[i] + coll_sys->data_container->host_data.pos_data[id.y];
			real distance = coll_sys->data_container->host_data.dpth_rigid_rigid[i];

			glVertex3f(vpA.x, vpA.y, vpA.z);
			glVertex3f(vpB.x, vpB.y, vpB.z);

		}
		glEnd();
	}
	if (contact_mode == LAGRANGE) {
		glBegin (GL_LINES);

		for (int i = 0; i < coll_sys->data_container->num_contacts; i++) {
			int2 cid = coll_sys->data_container->host_data.bids_rigid_rigid[i];
			//real3 vN = coll_sys->data_container->host_norm_data[i];
			//real3 vpA = coll_sys->data_container->host_cpta_data[i];
			//real3 vpB = coll_sys->data_container->host_cptb_data[i];

			real3 vpA = system_gpu->gpu_data_manager->host_data.pos_data[cid.x];
			real3 vpB = system_gpu->gpu_data_manager->host_data.pos_data[cid.y];

			//real distance = coll_sys->data_container->host_dpth_data[i];
			real lagrange = coll_sys->data_container->host_data.gamma_data[i];
			real3 col = GetColour(lagrange, 0, .1);
			glColor3f(col.x, col.y, col.z);
			glVertex3f(vpA.x, vpA.y, vpA.z);
			glVertex3f(vpB.x, vpB.y, vpB.z);

		}
		glEnd();
	}

}
void ChOpenGL::DrawContactsBulletGPU(ChCollisionSystemBulletParallel* coll_sys) {
	btCollisionWorld* bt_collision_world = coll_sys->GetBulletCollisionWorld();

	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	uint contacts = 0;
	ChCollisionInfo icontact;
	glBegin (GL_POINTS);
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

		icontact.modelA = (ChCollisionModel*) obA->getUserPointer();
		icontact.modelB = (ChCollisionModel*) obB->getUserPointer();

		if (((ChBody*) (icontact.modelA->GetPhysicsItem()))->IsActive() == false && ((ChBody*) (icontact.modelB->GetPhysicsItem()))->IsActive() == false) {
			continue;
		}

		double envelopeA = icontact.modelA->GetEnvelope();
		double envelopeB = icontact.modelB->GetEnvelope();

		double marginA = icontact.modelA->GetSafeMargin();
		double marginB = icontact.modelB->GetSafeMargin();

		// Execute custom broadphase callback, if any
		bool do_narrow_contactgeneration = true;

		if (do_narrow_contactgeneration) {
			int numContacts = contactManifold->getNumContacts();

			for (int j = 0; j < numContacts; j++) {
				btManifoldPoint& pt = contactManifold->getContactPoint(j);

				if (pt.getDistance() < marginA + marginB)     // to discard "too far" constraints (the Bullet engine also has its threshold)
				{

					btVector3 ptA = pt.getPositionWorldOnA();
					btVector3 ptB = pt.getPositionWorldOnB();

					icontact.vpA.Set(ptA.getX(), ptA.getY(), ptA.getZ());
					icontact.vpB.Set(ptB.getX(), ptB.getY(), ptB.getZ());

					icontact.vN.Set(-pt.m_normalWorldOnB.getX(), -pt.m_normalWorldOnB.getY(), -pt.m_normalWorldOnB.getZ());
					icontact.vN.Normalize();

					double ptdist = pt.getDistance();

					icontact.vpA = icontact.vpA - icontact.vN * envelopeA;
					icontact.vpB = icontact.vpB + icontact.vN * envelopeB;
					icontact.distance = ptdist + envelopeA + envelopeB;

					icontact.reaction_cache = pt.reactions_cache;

					contacts++;
					// Execute some user custom callback, if any
					glVertex3f(icontact.vpA.x, icontact.vpA.y, icontact.vpA.z);
					glVertex3f(icontact.vpB.x, icontact.vpB.y, icontact.vpB.z);

				}

			}
		}
	}
	glEnd();

}
void ChOpenGL::DrawContactsBullet(ChCollisionSystemBullet* coll_sys) {
	btCollisionWorld* bt_collision_world = coll_sys->GetBulletCollisionWorld();

	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	uint contacts = 0;
	ChCollisionInfo icontact;
	glBegin (GL_POINTS);
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

		icontact.modelA = (ChCollisionModel*) obA->getUserPointer();
		icontact.modelB = (ChCollisionModel*) obB->getUserPointer();

		if (((ChBody*) (icontact.modelA->GetPhysicsItem()))->IsActive() == false && ((ChBody*) (icontact.modelB->GetPhysicsItem()))->IsActive() == false) {
			continue;
		}

		double envelopeA = icontact.modelA->GetEnvelope();
		double envelopeB = icontact.modelB->GetEnvelope();

		double marginA = icontact.modelA->GetSafeMargin();
		double marginB = icontact.modelB->GetSafeMargin();

		// Execute custom broadphase callback, if any
		bool do_narrow_contactgeneration = true;

		if (do_narrow_contactgeneration) {
			int numContacts = contactManifold->getNumContacts();

			for (int j = 0; j < numContacts; j++) {
				btManifoldPoint& pt = contactManifold->getContactPoint(j);

				if (pt.getDistance() < marginA + marginB)     // to discard "too far" constraints (the Bullet engine also has its threshold)
				{

					btVector3 ptA = pt.getPositionWorldOnA();
					btVector3 ptB = pt.getPositionWorldOnB();

					icontact.vpA.Set(ptA.getX(), ptA.getY(), ptA.getZ());
					icontact.vpB.Set(ptB.getX(), ptB.getY(), ptB.getZ());

					icontact.vN.Set(-pt.m_normalWorldOnB.getX(), -pt.m_normalWorldOnB.getY(), -pt.m_normalWorldOnB.getZ());
					icontact.vN.Normalize();

					double ptdist = pt.getDistance();

					icontact.vpA = icontact.vpA - icontact.vN * envelopeA;
					icontact.vpB = icontact.vpB + icontact.vN * envelopeB;
					icontact.distance = ptdist + envelopeA + envelopeB;

					icontact.reaction_cache = pt.reactions_cache;

					contacts++;
					// Execute some user custom callback, if any
					glVertex3f(icontact.vpA.x, icontact.vpA.y, icontact.vpA.z);
					glVertex3f(icontact.vpB.x, icontact.vpB.y, icontact.vpB.z);

				}

			}
		}
	}
	glEnd();
}
void ChOpenGL::DrawGrids() {
	real3 aabb_min, aabb_max;
	bool use_aabb = ((ChSystemParallel*) (physics_system[0]))->GetAABB(aabb_min, aabb_max);

	if (use_aabb) {

		real3 c = (aabb_min + aabb_max) * .5;
		real3 size = R3(fabs(c.x - aabb_min.x), fabs(c.y - aabb_min.y), fabs(c.z - aabb_min.z));

		MakeBox(Vector(c.x, c.y, c.z), Vector(size.x, size.y, size.z), 0, Vector(1, 0, 0), Vector(1, 1, 1));

	}

	if (((ChLcpSolverParallel*) (physics_system[0]->GetLcpSolverSpeed()))->GetWarmStart() == false) {
		return;
	}

	glDisable (GL_LIGHTING);
	glPointSize(10);
	int3 num_bins_per_axis = ((ChLcpSolverParallel*) (physics_system[0]->GetLcpSolverSpeed()))->num_bins_per_axis;
	real3 bin_size_vec = ((ChLcpSolverParallel*) (physics_system[0]->GetLcpSolverSpeed()))->bin_size_vec;
	real3 origin = ((ChLcpSolverParallel*) (physics_system[0]->GetLcpSolverSpeed()))->origin;
	glBegin (GL_POINTS);

	int l = num_bins_per_axis.x;
	int h = num_bins_per_axis.y;
	int w = num_bins_per_axis.z;

	for (int i = 0; i < num_bins_per_axis.x; i++) {
		for (int j = 0; j < num_bins_per_axis.y; j++) {
			for (int k = 0; k < num_bins_per_axis.z; k++) {
				real x = i * bin_size_vec.x + origin.x + bin_size_vec.x / 2.0;
				real y = j * bin_size_vec.y + origin.y + bin_size_vec.y / 2.0;
				real z = k * bin_size_vec.z + origin.z + bin_size_vec.z / 2.0;
				real gamma = ((ChSystemParallel*) (physics_system[0]))->gpu_data_manager->host_data.bin_number[i + j * w + k * w * h];
				real resid = ((ChLcpSolverParallel *) (physics_system[0]->GetLcpSolverSpeed()))->GetResidual();

				real3 color = GetColour(gamma, 0, resid);
				glColor3f(color.x, color.y, color.z);
				glVertex3f(x, y, z);

			}
		}
	}
	glEnd();
	glPointSize(2);
}
void ChOpenGL::DrawContacts() {

	// if (contact_mode != NOCONTACT) {
	// 	for (int i = 0; i < physics_system.size(); i++) {

	// 		ChCollisionSystem * newcollsystem = physics_system[i]->GetCollisionSystem();
	// 		if (ChCollisionSystemParallel* coll_sys = dynamic_cast<ChCollisionSystemParallel*>(newcollsystem)) {
	// 			DrawContactsGPU((ChSystemParallel*) physics_system[i]);
	// 		} else if (ChCollisionSystemBulletParallel* coll_sys = dynamic_cast<ChCollisionSystemBulletParallel*>(newcollsystem)) {
	// 			DrawContactsBulletGPU(((ChCollisionSystemBulletParallel*) newcollsystem));
	// 		} else if (ChCollisionSystemBullet* coll_sys = dynamic_cast<ChCollisionSystemBullet*>(newcollsystem)) {
	// 			DrawContactsBullet(((ChCollisionSystemBullet*) newcollsystem));
	// 		}
	// 	}
	// }

}

void ChOpenGL::DrawText() {
	glEnable (GL_DEPTH_TEST);
	for (int i = 0; i < text_3d.size(); i++) {
		RenderBitmapString3D(text_3d[i].pos, GLUT_BITMAP_HELVETICA_10, text_3d[i].text);
	}
	text_3d.clear();
	glDisable(GL_DEPTH_TEST);
}
void ChOpenGL::DrawHUD() {
	SetOrthographicProjection();
	glDisable (GL_DEPTH_TEST);
	glDisable (GL_LIGHTING);
	glColor3f(0.0, 0.0, 0.0);
	stringstream bottom;
	bottom << "Frame:" << sim_frame << " Time:" << sim_time;
	glPushMatrix();
	glLoadIdentity();
	RenderBitmapString2D(5, height - 15, GLUT_BITMAP_9_BY_15, bottom.str());
	RenderBitmapString2D(5, 15, GLUT_BITMAP_9_BY_15, ToString("Step:       ", physics_system[0]->GetTimerStep()));
	RenderBitmapString2D(5, 30, GLUT_BITMAP_9_BY_15, ToString("Update:     ", physics_system[0]->GetTimerUpdate()));
	RenderBitmapString2D(5, 45, GLUT_BITMAP_9_BY_15, ToString("Broadphase: ", physics_system[0]->GetTimerCollisionBroad()));
	RenderBitmapString2D(5, 60, GLUT_BITMAP_9_BY_15, ToString("Narrowphase:", physics_system[0]->GetTimerCollisionNarrow()));
	RenderBitmapString2D(5, 75, GLUT_BITMAP_9_BY_15, ToString("Solver:     ", physics_system[0]->GetTimerLcp()));

	RenderBitmapString2D(
		width / 2 + width / 4,
		15,
		GLUT_BITMAP_9_BY_15,
		ToString("Iterations:", ((ChLcpIterativeSolver*) (physics_system[0]->GetLcpSolverSpeed()))->GetTotalIterations()));
	RenderBitmapString2D(width / 2 + width / 4, 30, GLUT_BITMAP_9_BY_15, ToString("Bodies:", physics_system[0]->GetNbodiesTotal()));
	RenderBitmapString2D(width / 2 + width / 4, 45, GLUT_BITMAP_9_BY_15, ToString("Contacts:", physics_system[0]->GetNcontacts()));

	vector<double> history, dlambda;

	history = ((ChLcpIterativeSolver*) (physics_system[0]->GetLcpSolverSpeed()))->GetViolationHistory();
	dlambda = ((ChLcpIterativeSolver*) (physics_system[0]->GetLcpSolverSpeed()))->GetDeltalambdaHistory();
	if (history.size() > 0) {
		RenderBitmapString2D(width / 2 + width / 4, 60, GLUT_BITMAP_9_BY_15, ToString("Residual:", history[history.size() - 1]));
		RenderBitmapString2D(width / 2 + width / 4, 75, GLUT_BITMAP_9_BY_15, ToString("Correction:", dlambda[dlambda.size() - 1]));

		real max_hist = *max_element(history.begin(), history.end());

		glBegin (GL_LINES);
		for (int i = 0; i < history.size() - 1; i++) {
			real2 p1 = R2(i / real(history.size()) * 100 + width - 100, history[i] / max_hist * 100 + height - 100);
			real2 p2 = R2((i + 1) / real(history.size()) * 100 + width - 100, history[i + 1] / max_hist * 100 + height - 100);

			glVertex3f(p1.x, p1.y, 0);
			glVertex3f(p2.x, p2.y, 0);
		}
		glEnd();
	} else {
		RenderBitmapString2D(width / 2 + width / 4, 60, GLUT_BITMAP_9_BY_15, ToString("Residual:", -1));

	}
	glPopMatrix();
	glEnable(GL_DEPTH_TEST);
	RestorePerspectiveProjection();

}
void ChOpenGL::RenderBitmapString2D(float x, float y, void *font, string str) {
	glRasterPos2f(x, y);
	for (int i = 0; i < str.size(); i++) {
		glutBitmapCharacter(font, str[i]);
	}
}
void ChOpenGL::RenderBitmapString3D(Vector p, void *font, string str) {
	glRasterPos3f(p.x, p.y, p.z);
	for (int i = 0; i < str.size(); i++) {
		glutBitmapCharacter(font, str[i]);
	}
}
void ChOpenGL::MakeSphere(Vector pos, float rad, float angle, Vector axis, Vector scale) {
	glTranslatef(pos.x, pos.y, pos.z);
	glRotatef(angle * 180.0 / CH_C_PI, axis.x, axis.y, axis.z);
	glScalef(scale.x, scale.y, scale.z);
	if (display_mode == WIREFRAME) {
		glutWireSphere(rad, 10, 10);
	} else if (display_mode == SOLID) {
		glutSolidSphere(rad, 20, 20);
	}
}
void ChOpenGL::MakeBox(Vector pos, Vector radius, float angle, Vector axis, Vector scale) {
	glTranslatef(pos.x, pos.y, pos.z);
	glRotatef(angle * 180.0 / CH_C_PI, axis.x, axis.y, axis.z);
	glScalef(radius.x * 2, radius.y * 2, radius.z * 2);
	if (display_mode == SOLID) {
		glutSolidCube(1);

	} else {
		glutWireCube(1);
	}
}
void ChOpenGL::MakeCyl(Vector pos, double radius, float angle, Vector axis, Vector scale) {

	glTranslatef(pos.x, pos.y, pos.z);
	GLUquadric *quad = gluNewQuadric();

	if (display_mode == WIREFRAME) {
		gluQuadricDrawStyle(quad, GLU_LINE);
	} else {
		gluQuadricDrawStyle(quad, GLU_FILL);
	}

	glRotatef(angle * 180.0 / CH_C_PI, axis.x, axis.y, axis.z);
	//glRotatef(-90, 1, 0, 0);
	//glScalef(scale.x * 2, scale.x * 2, scale.y * 2);
	gluCylinder(quad, radius, radius, scale.y, 50, 1);
	glRotatef(-180, 1, 0, 0);
	gluCylinder(quad, radius, radius, scale.y, 50, 1);
}

void ChOpenGL::MakeCone(Vector pos, Vector radius, float angle, Vector axis) {
	glTranslatef(pos.x, pos.y, pos.z);
	glRotatef(angle * 180.0 / CH_C_PI, axis.x, axis.y, axis.z);

	glRotatef(-90, 1, 0, 0);

//glScalef(radius.x, radius.y, radius.z);

	glTranslatef(0, 0, -radius.y / 2.0);
	if (display_mode == WIREFRAME) {
		glutWireCone(radius.x, radius.y, 10, 10);
	} else if (display_mode == SOLID) {
		glutSolidCone(radius.x, radius.y, 10, 10);
	}

}
void ChOpenGL::SetOrthographicProjection() {
	glMatrixMode (GL_PROJECTION);
	glPushMatrix();
	glEnable (GL_DEPTH_TEST);
	glDepthFunc (GL_LEQUAL);
	glLoadIdentity();
	gluOrtho2D(0, width, height, 0);
	glMatrixMode (GL_MODELVIEW);
}

void ChOpenGL::RestorePerspectiveProjection() {
	glDisable (GL_DEPTH_TEST);
	glMatrixMode (GL_PROJECTION);
	glPopMatrix();
	glMatrixMode (GL_MODELVIEW);
}
void ChOpenGL::RunStep() {
	if (sim_mode == RUNNING) {
		for (int s = 0; s < physics_system.size(); s++) {
			physics_system[s]->DoStepDynamics(physics_system[s]->GetStep());
		}
		sim_frame++;
		sim_time += sim_dt;
	}

}



