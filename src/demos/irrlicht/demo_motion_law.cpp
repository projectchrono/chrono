//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// Author: Justin Madsen, 2014
///////////////////////////////////////////////////
//
//   Demo code about 
//
//     - create a custom ChLinkLock class to constrain a body, and apply a
//      translational motion to a pendulum hanging from it.
//
//	 CHRONO    
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
#include "physics/ChSystem.h"
#include "physics/ChBodyEasy.h"
#include "unit_IRRLICHT/ChBodySceneNodeTools.h" 
#include "unit_IRRLICHT/ChIrrApp.h"
#include "core/ChRealtimeStep.h"

#include <irrlicht.h>

// Use the namespace of Chrono, Irrlicht
using namespace chrono;
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;	// control the motion with a GUI slider, at some point


// ************************************************
// Simulation settings, for the user to select
// ************************************************
ChVector<> motion_dir(1,0,0); // applied motion direction
double pend_height = 1.0; // free-hanging pednulum length
// motion is a sine wave
double phase = 0;
double freq = 0.5;
double amp = 1.5; // distance traveled by free body

// Time interval between two render frames
double step_size = 0.001;
int FPS = 50;
double render_step_size = 1.0 / FPS;   // FPS = 50
int output_console_incr = 10; // print info to console every Nth rendered frame
int output_console_num = 0;
// ************************************************

/// class to apply this motion law as a ChFunction between two bodies,
///  one fixed, one free, in a given direction.
class ForcedMotion : public ChLinkLock
{
private:
  double m_offset;
  ChSharedPtr<ChFunction> m_motion_func;
  ChVector<> m_motion_dir;	// direction of applied translational motion
	bool m_user_override;  // TODO: allow user slider to interactively set motion law

public:
  CH_RTTI(ForcedMotion,ChLinkLock);

  ForcedMotion()
        : m_user_override(false)
  {
    // set motion in the z direction
    ((ChLinkMaskLF*)mask)->SetLockMask(false, false, true,
                                       false, false, false, false,
                                       false, false);
    ChangedLinkMask();
    m_offset = 0.1;
  };

  ~ForcedMotion(void) {}

  /// set or access the motion function
  void Set_motion(ChSharedPtr<ChFunction> func) { m_motion_func = func; }
  ChSharedPtr<ChFunction> Get_motion() { return m_motion_func; }

  // return the inertial displacement for the current time, for a
  // specific axis 0 = X, 1 = Y, 2 = Z.
  double Get_y(double time){ return m_motion_func->Get_y(time); }

  double Get_dy(double time)
  {
    return m_motion_func->Get_y_dx(time);
  }

  double Get_ddy(double time)
  {
    return m_motion_func->Get_y_dxdx(time);
  }

  // imposes motion law on rhs of constraint
  virtual void UpdateTime(double mytime)
  {
    // Impose relative positions/speeds
  
    // from ChLinkLinActuator
    // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);
    ChMatrix33<> ma;
    ma.Set_A_quaternion(marker2->GetAbsCoord().rot);

    Vector absdist = Vsub(marker1->GetAbsCoord().pos,  marker2->GetAbsCoord().pos);

    // specified the motion direction, will be along the m2 z-axis
    Vector mz = Vnorm(m_motion_dir);

    Vector my = ma.Get_A_Yaxis();
    if (Vequal(mz, my))
    {
      if (  std::abs(mz.z - 1.0) < 0.01 ) 
        my = VECT_Y;
      else
        my = VECT_Z;
    }
    Vector mx = Vnorm(my % mz );
    my = Vnorm(mz % mx);

    ma.Set_A_axis(mx, my, mz);

    Coordsys newmarkpos;
    ChVector<> oldpos = marker2->GetPos(); // backup to avoid numerical err.accumulation
    newmarkpos.pos = marker2->GetAbsCoord().pos;
    newmarkpos.rot = ma.Get_A_quaternion();
    marker2->Impose_Abs_Coord(newmarkpos);        //rotate "main" marker2 into tangent position (may add err.accumulation)
    marker2->SetPos(oldpos); // backup to avoid numerical err.accumulation

    // imposed relative positions/speeds
    deltaC.pos = VNULL;
    deltaC.pos.z = m_motion_func->Get_y(mytime) + m_offset;  // distance is always on M2 'Z' axis

    deltaC_dt.pos = VNULL;
    deltaC_dt.pos.z = m_motion_func->Get_y_dx(mytime); // distance speed

    deltaC_dtdt.pos = VNULL;
    deltaC_dtdt.pos.z = m_motion_func->Get_y_dxdx(mytime); // distance acceleration
    // add also the centripetal acceleration if distance vector's rotating,
    // as centripetal acc. of point sliding on a sphere surface.
    Vector tang_speed = GetRelM_dt().pos;
    tang_speed.z = 0; // only x-y coords in relative tang speed vector
    double len_absdist = Vlength(absdist);  // if this is zero, don't divide by it!
    if( len_absdist > 1E-6)
      deltaC_dtdt.pos.z -= pow(Vlength(tang_speed), 2) / len_absdist;  // An = Adelta -(Vt^2 / r)
    // if( Vlength(absdist) < 1E-6)
    //  double asdf = Vlength(absdist); // try not to divide by zero
    deltaC.rot = QUNIT;             // no relative rotations
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
  }


  virtual void UpdateForces (double mytime)
  {
    // some of this is from ChLinkEngine.cpp
    // inherit to parent class
    ChLinkLock::UpdateForces(mytime);

    // zero Z pos. violation
    // if (C->GetRows())
      // C->SetElement(C->GetRows() - 2, 0, 0.0);
  }


  void print_stepInfo(double time)
  {
  GetLog() << " time = " << time << "\n y,dy,ddy = f(t) = " <<
            Get_y(time) <<", "<< Get_dy(time) <<", "<< Get_ddy(time) 
            <<"\n F = " << Get_react_force().z <<"\n\n";
  }

  void Copy(ForcedMotion* source)
  {
    // copy parent
    ChLinkLock::Copy(source);
    // copy private stuff
    m_offset = source->m_offset;
    m_motion_func = ChSharedPtr<ChFunction>(source->m_motion_func);
    m_motion_dir = source->m_motion_dir;
    m_user_override = source->m_user_override;
  }

  ChLink* new_Duplicate()
  {
    ForcedMotion* m_l;
    m_l = new ForcedMotion;
    m_l->Copy(this);
    return(m_l);
  }

};


/// test the motion law
int main(int argc, char* argv[])
{
	// Create a ChronoENGINE physical system
	ChSystem mSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mSystem, L"motion with constraint test",core::dimension2d<u32>(800,600),false);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(-1,1,-1), core::vector3df(1,0,1));

	// fixed ground object, with concrete texture asset. No collision shape
	ChSharedPtr<ChBodyEasyBox> fixedBody(new ChBodyEasyBox( 0.4,0.2,0.4, 1000,	false, true));
	fixedBody->SetPos( ChVector<>(0,0,0) );
	fixedBody->SetBodyFixed(true);
	ChSharedPtr<ChTexture> concrete_tex(new ChTexture());
	concrete_tex->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
	fixedBody->AddAsset(concrete_tex);
	// add fixed body to the system
	mSystem.Add(fixedBody);
	
	// free body, a cylinder aligned in the direction of motion. No collision shape
	ChSharedPtr<ChBodyEasyCylinder> free(new ChBodyEasyCylinder( 0.2,0.5, 1000,	false, true));
	free->SetPos( ChVector<>(0,0,1) );
  // let's orient the y-axis of the cylinder (height) along the direction of the applied motion.
  // y_hat dot dir_hat = cos(theta)
  // axis_hat = y_hat cross dir_hat
  ChVector<> y_hat(0,1,0);
  ChVector<> dir_hat = motion_dir.GetNormalized(); // motion direction, normalized
  ChQuaternion<> cyl_rot = Q_from_AngAxis( std::acos(y_hat.Dot( dir_hat ) ), y_hat % dir_hat );
  free->SetRot(cyl_rot);
	ChSharedPtr<ChTexture> tire_tex(new ChTexture()); // see how a tire texture looks
	tire_tex->SetTextureFilename(GetChronoDataFile("tire.png"));
	free->AddAsset(tire_tex);
	// add fixed body to the system
	mSystem.Add(free);

  // constrained body, a hanging pendulum
  ChSharedPtr<ChBodyEasyBox> constrained(new ChBodyEasyBox(0.05,pend_height/2.0,0.05,1000.0, true, true));
  ChVector<> constrained_pos = free->GetPos();
  constrained_pos.y -= pend_height/2.0;
  constrained_pos.z += 0.1;  // slight offset
  constrained->SetPos( constrained_pos );

  ChSharedPtr<ChTexture> rock_tex(new ChTexture());
  rock_tex->SetTextureFilename(GetChronoDataFile("rock.jpg"));
  constrained->AddAsset(rock_tex);
  mSystem.Add(constrained);

  // use a spherical joint to constrain the pendulum at the COM of the motion body
  ChSharedPtr<ChLinkLockSpherical> joint(new ChLinkLockSpherical());
  joint->Initialize(free, constrained, ChCoordsys<>(free->GetPos(), QUNIT) );
  mSystem.Add(joint);

  // ***************************************************
  // Motion law acts like a constraint, w/ a z-axis DOF
  ChSharedPtr<ForcedMotion> motionLaw(new ForcedMotion());
  // x-axis applied motion: rotate markers about y-axis

  motionLaw->Initialize(free, fixedBody, ChCoordsys<>(free->GetPos(), QUNIT)); 
  // Apply the motion law (sine wave) to the constraint just created
  ChSharedPtr<ChFunction_Sine> motion_func(new ChFunction_Sine(phase, freq, amp));
  motionLaw->Set_motion(motion_func);

  mSystem.Add(motionLaw);
  // ***************************************************

	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();
	// Use this function for 'converting' assets into Irrlicht meshes 
	application.AssetUpdateAll();

  // Modify some setting of the physical system for the simulation, if you want
	mSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
  mSystem.SetIterLCPmaxItersStab(40);
  
  // integrator suggested step size
  application.SetTimestep(step_size);
  int step_number = 0;
  double time = 0;
  // Number of simulation steps between two 3D view render frames
  int render_steps = (int)std::ceil(render_step_size / step_size);

  while(application.GetDevice()->run()) 
	{
    time = mSystem.GetChTime();
    // Render scene
    if (step_number % render_steps == 0) {
		  application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
      application.DrawAll();
      application.GetVideoDriver()->endScene();  

      // everyone once in a while, output some pertinent info
      if(output_console_num % output_console_incr)
      {
        output_console_num = 0;
        motionLaw->print_stepInfo(time);
      } else  {
        output_console_num++;
      }

    }
    
    // advance the dynamics
		application.DoStep();	

    // on to the next step
    step_number++;
	}
	
	return 0;

}








/*

/// slider for the user to apply the translational motion law
class MyEventReceiver : public IEventReceiver
{
public:
// @param vel_max	range of slider motion, [-vel_max, vel_max]
MyEventReceiver(ChIrrAppInterface* app, double vel_max, ForcedMotion* motion_law)
: m_app(app), m_vel_max(vel_max), m_motion(motion_law)
{
	// motion scrollbar GUI ID: 101
	scrollbar_motion = m_app->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 85, 650, 100), 0, 101);
	scrollbar_motion->setMax(100); 
	scrollbar_motion->setPos(50);
	text_friction = m_app->GetIGUIEnvironment()->addStaticText(L"Friction coefficient:", rect<s32>(650,85,750,100), false);
}

bool OnEvent(const SEvent& event)
{
	// check if user moved the sliders with mouse..
	if (event.EventType == EET_GUI_EVENT)
	{
		s32 id = event.GUIEvent.Caller->getID();
		IGUIEnvironment* env = m_app->GetIGUIEnvironment();

		switch(event.GUIEvent.EventType)
		{
		case EGET_SCROLL_BAR_CHANGED:
				if (id == 101) 
				{
					s32 sliderpos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos(); // (0-100)
					double vel = vel_max * (sliderpos - 50.0)/50.0
					m_motion->Set;	// (-vel_max, vel_max)
				}
				
		break;
		}
		
	} 

	return false;
}

private: 
	ChIrrAppInterface* m_app;

	IGUIScrollBar*  scrollbar_motion;
	IGUIStaticText* text_motion;
	double 			m_vel_max;	// maximum/minimum motion velocity
	ForcedMotion*	m_motion;	// motion law
};


*/