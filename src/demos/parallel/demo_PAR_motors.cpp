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
// Authors: Radu Serban
// =============================================================================
//
// Test for motors with Chrono::Parallel.
//
// NOTE: ChLinkMotorRotationDriveline and ChLinkMotorLinearDriveline
//       are currently *not* supported in Chrono::Parallel.
//
// =============================================================================

#include "chrono/assets/ChColorAsset.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
//
#include "chrono_parallel/physics/ChSystemParallel.h"
//
#include "chrono_opengl/ChOpenGLWindow.h"

using namespace chrono;

// -----------------------------------------------------------------------------

void CreateSliderGuide(std::shared_ptr<ChBody>& mguide,
                       std::shared_ptr<ChBody>& mslider,
                       ChSystem& msystem,
                       const ChVector<> mpos) {
    mguide = std::make_shared<ChBodyEasyBox>(4, 0.3, 0.6, 1000, true, true, ChMaterialSurface::NSC,
                                             std::make_shared<collision::ChCollisionModelParallel>());
    mguide->SetPos(mpos);
    mguide->SetBodyFixed(true);
    msystem.Add(mguide);

    mslider = std::make_shared<ChBodyEasyBox>(0.4, 0.2, 0.5, 1000, true, true, ChMaterialSurface::NSC,
                                              std::make_shared<collision::ChCollisionModelParallel>());
    mslider->SetPos(mpos + ChVector<>(0, 0.3, 0));
    msystem.Add(mslider);

    auto mcolor = std::make_shared<ChColorAsset>(0.6f, 0.6f, 0.0f);
    mslider->AddAsset(mcolor);

    auto obstacle = std::make_shared<ChBodyEasyBox>(0.4, 0.4, 0.4, 8000, true, true, ChMaterialSurface::NSC,
                                                    std::make_shared<collision::ChCollisionModelParallel>());
    obstacle->SetPos(mpos + ChVector<>(1.5, 0.4, 0));
    msystem.Add(obstacle);
    auto mcolorobstacle = std::make_shared<ChColorAsset>(0.2f, 0.2f, 0.2f);
    mslider->AddAsset(mcolorobstacle);
}

void CreateStatorRotor(std::shared_ptr<ChBody>& mstator,
                       std::shared_ptr<ChBody>& mrotor,
                       ChSystem& msystem,
                       const ChVector<> mpos) {
    mstator = std::make_shared<ChBodyEasyCylinder>(0.5, 0.1, 1000, true, true, ChMaterialSurface::NSC,
                                                   std::make_shared<collision::ChCollisionModelParallel>());
    mstator->SetPos(mpos);
    mstator->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    mstator->SetBodyFixed(true);
    msystem.Add(mstator);

    mrotor = std::make_shared<ChBodyEasyBox>(1, 0.1, 0.1, 1000, true, true, ChMaterialSurface::NSC,
                                             std::make_shared<collision::ChCollisionModelParallel>());
    mrotor->SetPos(mpos + ChVector<>(0.5, 0, -0.15));
    msystem.Add(mrotor);

    auto mcolor = std::make_shared<ChColorAsset>(0.6f, 0.6f, 0.0f);
    mrotor->AddAsset(mcolor);
}

// -----------------------------------------------------------------------------

void ExampleA1(ChSystem& mphysicalSystem) {
    // EXAMPLE A.1
    //
    // - class:   ChLinkMotorRotationSpeed
    // - type:    rotational motor
    // - control: impose a time-dependent speed=v(t)
    //
    // This is a simple type of rotational actuator. It assumes that
    // you know the exact angular speed of the rotor respect to the stator,
    // as a function of time:   angular speed = w(t).
    // Use this to simulate fans, rotating cranks, etc.
    // Note: this is a rheonomic motor that enforces the motion
    // geometrically; no compliance is allowed, this means that if the
    // rotating body hits some hard contact, the solver might give unpredictable
    // oscillatory or diverging results because of the contradiction.

    ChVector<> positionA1(-3, 2, -3);
    std::shared_ptr<ChBody> stator1;
    std::shared_ptr<ChBody> rotor1;
    CreateStatorRotor(stator1, rotor1, mphysicalSystem, positionA1);

    // Create the motor
    auto rotmotor1 = std::make_shared<ChLinkMotorRotationSpeed>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor1->Initialize(rotor1,                // body A (slave)
                          stator1,               // body B (master)
                          ChFrame<>(positionA1)  // motor frame, in abs. coords
    );
    mphysicalSystem.Add(rotmotor1);

    // Create a ChFunction to be used for the ChLinkMotorRotationSpeed
    auto mwspeed = std::make_shared<ChFunction_Const>(CH_C_PI_2);  // constant angular speed, in [rad/s], 1PI/s =180°/s
    // Let the motor use this motion function:
    rotmotor1->SetSpeedFunction(mwspeed);

    // The ChLinkMotorRotationSpeed contains a hidden state that performs the time integration
    // of the angular speed setpoint: such angle is then imposed to the
    // constraint at the positional level too, thus avoiding angle error
    // accumulation (angle drift). Optionally, such positional constraint
    // level can be disabled as follows:
    //
    // rotmotor1->SetAvoidAngleDrift(false);
}

void ExampleA2(ChSystem& mphysicalSystem) {
    // EXAMPLE A.2
    //
    // - class:   ChLinkMotorRotationAngle
    // - type:    rotational motor
    // - control: impose a time-dependent angle=a(t)
    //
    // This is a simple type of rotational actuator. It assumes that
    // you know the exact angular angle of the rotor respect to the stator,
    // as a function of time:   angle = a(t).
    // Use this to simulate servo drives in robotic systems and automation,
    // where you can assume that the motor rotates with an infinitely stiff
    // and reactive control, that exactly follows your prescribed motion profiles.
    // Note: this is a rheonomic motor that enforces the motion
    // geometrically; no compliance is allowed, this means that if the
    // rotating body hits some hard contact, the solver might give unpredictable
    // oscillatory or diverging results because of the contradiction.

    ChVector<> positionA2(-3, 2, -2);
    std::shared_ptr<ChBody> stator2;
    std::shared_ptr<ChBody> rotor2;
    CreateStatorRotor(stator2, rotor2, mphysicalSystem, positionA2);

    // Create the motor
    auto rotmotor2 = std::make_shared<ChLinkMotorRotationAngle>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor2->Initialize(rotor2,                // body A (slave)
                          stator2,               // body B (master)
                          ChFrame<>(positionA2)  // motor frame, in abs. coords
    );
    mphysicalSystem.Add(rotmotor2);

    // Create a ChFunction to be used for the ChLinkMotorRotationAngle
    auto msineangle = std::make_shared<ChFunction_Sine>(0,       // phase [rad]
                                                        0.05,    // frequency [Hz]
                                                        CH_C_PI  // amplitude [rad]
    );
    // Let the motor use this motion function as a motion profile:
    rotmotor2->SetAngleFunction(msineangle);
}

void ExampleA3(ChSystem& mphysicalSystem) {
    // EXAMPLE A.3
    //
    // - class:   ChLinkMotorRotationTorque
    // - type:    rotational motor
    // - control: impose a (time-dependent) torque=T(t)
    //
    // For this motor, you must specify a time-dependent torque as torque = T(t).
    // (If you want to use this motor to follow some desired motion profiles, you
    // must implement a PID controller that continuously adjusts the value of the
    // torque during the simulation).

    ChVector<> positionA3(-3, 2, -1);
    std::shared_ptr<ChBody> stator3;
    std::shared_ptr<ChBody> rotor3;
    CreateStatorRotor(stator3, rotor3, mphysicalSystem, positionA3);

    // Create the motor
    auto rotmotor3 = std::make_shared<ChLinkMotorRotationTorque>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor3->Initialize(rotor3,                // body A (slave)
                          stator3,               // body B (master)
                          ChFrame<>(positionA3)  // motor frame, in abs. coords
    );
    mphysicalSystem.Add(rotmotor3);

    // The torque(time) function:
    auto mtorquetime = std::make_shared<ChFunction_Sine>(0,   // phase [rad]
                                                         2,   // frequency [Hz]
                                                         160  // amplitude [Nm]
    );

    // Let the motor use this motion function as a motion profile:
    rotmotor3->SetTorqueFunction(mtorquetime);
}

void ExampleA4(ChSystem& mphysicalSystem) {
    // EXAMPLE A.4
    //
    // As before, use a ChLinkMotorRotationTorque, but this time compute
    // torque by a custom function. In this example we implement a
    // basic torque(speed) model of a three-phase induction electric motor..

    ChVector<> positionA4(-3, 2, 0);
    std::shared_ptr<ChBody> stator4;
    std::shared_ptr<ChBody> rotor4;
    CreateStatorRotor(stator4, rotor4, mphysicalSystem, positionA4);

    // Create the motor
    auto rotmotor4 = std::make_shared<ChLinkMotorRotationTorque>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor4->Initialize(rotor4,                // body A (slave)
                          stator4,               // body B (master)
                          ChFrame<>(positionA4)  // motor frame, in abs. coords
    );
    mphysicalSystem.Add(rotmotor4);

    // Implement our custom  torque function.
    // We could use pre-defined ChFunction classes like sine, constant, ramp, etc.,
    // but in this example we show how to implement a custom function: a
    // torque(speed) function that represents a three-phase electric induction motor.
    // Just inherit from ChFunction and implement Get_y() so that it returns different
    // values (regrdless of time x) depending only on the slip speed of the motor:
    class MyTorqueCurve : public ChFunction {
      public:
        // put here some data that you need when evaluating y(x):
        double E2;  // voltage on coil, etc.
        double R2;
        double X2;
        double ns;
        std::shared_ptr<ChLinkMotorRotationTorque> mymotor;

        virtual MyTorqueCurve* Clone() const override { return new MyTorqueCurve(*this); }

        virtual double Get_y(double x) const override {
            // The three-phase torque(speed) model
            double w = mymotor->GetMotorRot_dt();
            double s = (ns - w) / ns;  // slip
            double T =
                (3.0 / 2 * CH_C_PI * ns) * (s * E2 * E2 * R2) / (R2 * R2 + pow(s * X2, 2));  // electric torque curve
            T -= w * 5;  // simulate also a viscous brake
            return T;
        }
    };

    // Create the function object from our custom class, and initialize its data:
    auto mtorquespeed = std::make_shared<MyTorqueCurve>();
    mtorquespeed->E2 = 120;
    mtorquespeed->R2 = 80;
    mtorquespeed->X2 = 1;
    mtorquespeed->ns = 6;
    mtorquespeed->mymotor = rotmotor4;

    // Let the motor use this motion function as a motion profile:
    rotmotor4->SetTorqueFunction(mtorquespeed);
}

// -----------------------------------------------------------------------------

void ExampleB1(ChSystem& mphysicalSystem) {
    // EXAMPLE B.1
    //
    // - class:   ChLinkMotorLinearPosition
    // - type:    linear motor
    // - control: impose a time-dependent position=f(t)
    //
    // This is the simpliest type of linear actuator. It assumes that
    // you know the exact position of the slider respect to the guide,
    // as a function of time:   position = f(t)
    // Therefore, this is a rheonomic motor that enforces the motion
    // geometrically; no compliance is allowed, this means that if the
    // sliding body hits some hard contact, the solver might give unpredictable
    // oscillatory or diverging results because of the contradiction.

    ChVector<> positionB1(0, 0, -3);
    std::shared_ptr<ChBody> guide1;
    std::shared_ptr<ChBody> slider1;
    CreateSliderGuide(guide1, slider1, mphysicalSystem, positionB1);

    // Create the linear motor
    auto motor1 = std::make_shared<ChLinkMotorLinearPosition>();

    // Connect the guide and the slider and add the motor to the system:
    motor1->Initialize(slider1,               // body A (slave)
                       guide1,                // body B (master)
                       ChFrame<>(positionB1)  // motor frame, in abs. coords
    );
    mphysicalSystem.Add(motor1);

    // Create a ChFunction to be used for the ChLinkMotorLinearPosition
    auto msine = std::make_shared<ChFunction_Sine>(0,    // phase
                                                   0.5,  // frequency
                                                   1.6   // amplitude
    );
    // Let the motor use this motion function:
    motor1->SetMotionFunction(msine);
}

void ExampleB2(ChSystem& mphysicalSystem) {
    // EXAMPLE B.2
    //
    // - class:   ChLinkMotorLinearSpeed
    // - type:    linear motor
    // - control: impose a time-dependent speed=v(t)
    //
    // This is a simple type of linear actuator. It assumes that
    // you know the exact speed of the slider respect to the guide,
    // as a function of time:   speed = v(t)
    // Therefore, this is a rheonomic motor that enforces the motion
    // geometrically; no compliance is allowed, this means that if the
    // sliding body hits some hard contact, the solver might give unpredictable
    // oscillatory or diverging results because of the contradiction.
    // It contains a hidden state that performs the time integration
    // of the required speed, such position is then imposed too to the
    // constraint at the positional level, thus avoiding position error
    // accumulation (position drift). Optionally, such constraint on
    // position level can be disabled if you are not interested in pos.drift.

    ChVector<> positionB2(0, 0, -2);
    std::shared_ptr<ChBody> guide2;
    std::shared_ptr<ChBody> slider2;
    CreateSliderGuide(guide2, slider2, mphysicalSystem, positionB2);

    // Create the linear motor
    auto motor2 = std::make_shared<ChLinkMotorLinearSpeed>();

    // Connect the guide and the slider and add the motor to the system:
    motor2->Initialize(slider2,               // body A (slave)
                       guide2,                // body B (master)
                       ChFrame<>(positionB2)  // motor frame, in abs. coords
    );
    mphysicalSystem.Add(motor2);

    // Create a ChFunction to be used for the ChLinkMotorLinearSpeed
    auto msp = std::make_shared<ChFunction_Sine>(CH_C_PI_2,            // phase
                                                 0.5,                  // frequency
                                                 1.6 * 0.5 * CH_C_2PI  // amplitude
    );
    // Let the motor use this motion function:
    motor2->SetSpeedFunction(msp);

    // The ChLinkMotorLinearSpeed contains a hidden state that performs the time integration
    // of the speed setpoint: such position is then imposed to the
    // constraint at the positional level too, thus avoiding position error
    // accumulation (position drift). Optionally, such position constraint
    // level can be disabled as follows:
    //
    // motor2->SetAvoidPositionDrift(false);
}

void ExampleB3(ChSystem& mphysicalSystem) {
    // EXAMPLE B.3
    //
    // - class:   ChLinkMotorLinearForce
    // - type:    linear motor
    // - control: impose a time-dependent force=F(t)
    //
    // This actuator is moved via force as a function of time, F=F(t).
    // The basic "open loop" option is to provide a F(t) at the beginning (ex using
    // a feedforward model), but then there is no guarantee about the
    // precise position of the slider, when the simulation runs.
    // This means that, unless you update the force F at each time
    // step using some type of feedback controller, this actuator
    // cannot be used to follow some position setpoint. Implementing
    // your controller might complicate things, but it could be closer to
    // the behavior of a real actuator, that have some delay, bandwidth
    // latency and compliance - for example, differently from
    // other types such as ChLinkMotorLinearPosition  and
    // ChLinkMotorLinearSpeed, this force motor does not enforce any
    // constraint on the direction of motion, so if it the slider hits
    // some hard contact, it just stops and keeps pushing, and no troubles
    // with the solver happen.

    ChVector<> positionB3(0, 0, -1);
    std::shared_ptr<ChBody> guide3;
    std::shared_ptr<ChBody> slider3;
    CreateSliderGuide(guide3, slider3, mphysicalSystem, positionB3);

    // just for fun: modify the initial speed of slider to match other examples
    slider3->SetPos_dt(ChVector<>(1.6 * 0.5 * CH_C_2PI));

    // Create the linear motor
    auto motor3 = std::make_shared<ChLinkMotorLinearForce>();

    // Connect the guide and the slider and add the motor to the system:
    motor3->Initialize(slider3,               // body A (slave)
                       guide3,                // body B (master)
                       ChFrame<>(positionB3)  // motor frame, in abs. coords
    );
    mphysicalSystem.Add(motor3);

    // Create a ChFunction to be used for F(t) in ChLinkMotorLinearForce.
    auto mF = std::make_shared<ChFunction_Const>(200);
    // Let the motor use this motion function:
    motor3->SetForceFunction(mF);

    // Alternative: just for fun, use a sine harmonic whose max force is F=M*A, where
    // M is the mass of the slider, A is the max acceleration of the previous examples,
    // so finally the motion should be quite the same - but without feedback, if hits a disturb, it goes crazy:
    auto mF2 = std::make_shared<ChFunction_Sine>(0,                                                 // phase
                                                 0.5,                                               // frequency
                                                 slider3->GetMass() * 1.6 * pow(0.5 * CH_C_2PI, 2)  // amplitude
    );
    // motor3->SetForceFunction(mF2); // uncomment to test this
}

void ExampleB4(ChSystem& mphysicalSystem) {
    // EXAMPLE B.4
    //
    // As before, use a ChLinkMotorLinearForce, but this time compute
    // F by a user-defined procedure (as a callback). For example, here we write a very
    // basic PID control algorithm that adjusts F trying to chase a sinusoidal position.

    ChVector<> positionB4(0, 0, 0);
    std::shared_ptr<ChBody> guide4;
    std::shared_ptr<ChBody> slider4;
    CreateSliderGuide(guide4, slider4, mphysicalSystem, positionB4);

    // Create the linear motor
    auto motor4 = std::make_shared<ChLinkMotorLinearForce>();

    // Connect the guide and the slider and add the motor to the system:
    motor4->Initialize(slider4,               // body A (slave)
                       guide4,                // body B (master)
                       ChFrame<>(positionB4)  // motor frame, in abs. coords
    );
    mphysicalSystem.Add(motor4);

    // Create a ChFunction that computes F by a user-defined algorithm, as a callback.
    // One quick option would be to inherit from the ChFunction base class, and implement the Get_y()
    // function by putting the code you wish, as explained in demo_CH_functions.cpp. However this has some
    // limitations. A more powerful approach is to inherit from ChFunction_SetpointCallback, that automatically
    // computes the derivatives, if needed, by BDF etc. Therefore:
    // 1. You must inherit from the ChFunction_SetpointCallback base class, and implement the SetpointCallback()
    //    function by putting the code you wish. For example something like the follow:

    class MyForceClass : public ChFunction_SetpointCallback {
      public:
        // Here some specific data to be used in Get_y(),
        // add whatever you need, ex:
        double setpoint_position_sine_amplitude;
        double setpoint_position_sine_freq;
        double controller_P;  // for our basic PID
        double controller_D;  // for our basic PID
        double last_time;
        double last_error;
        double F;
        std::shared_ptr<ChLinkMotorLinearForce> linearmotor;  // may be useful later

        // Here we will compute F(t) by emulating a very basic PID scheme.
        // In practice, it works like a callback that is executed at each time step.
        // Implementation of this function is mandatory!!!
        virtual double SetpointCallback(const double x) override {
            // Trick: in this PID example, we need the following if(..)  to update PID
            // only when time changes (as the callback could be invoked more than once per timestep):
            double time = x;
            if (time > last_time) {
                double dt = time - last_time;
                // for example, the position to chase is this sine formula:
                double setpoint = setpoint_position_sine_amplitude * sin(setpoint_position_sine_freq * CH_C_2PI * x);
                double error = setpoint - linearmotor->GetMotorPos();
                double error_dt = (error - last_error) / dt;
                // for example, finally compute the force using the PID idea:
                F = controller_P * error + controller_D * error_dt;
                last_time = time;
                last_error = error;
            }
            return F;
        }
    };

    // 2. Create the function from the custom class...
    auto mFcallback = std::make_shared<MyForceClass>();
    //    ...and initialize its custom data
    mFcallback->setpoint_position_sine_amplitude = 1.6;
    mFcallback->setpoint_position_sine_freq = 0.5;
    mFcallback->controller_P = 42000;  // proportional P term in PID, the higher, the "stiffer" the control
    mFcallback->controller_D = 1000;   // derivative D term in PID, the higher, the more damped
    mFcallback->last_time = 0;
    mFcallback->last_error = 0;
    mFcallback->F = 0;
    mFcallback->linearmotor = motor4;

    // 3. Let the motor use our custom force:
    motor4->SetForceFunction(mFcallback);
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double step_size = 1e-3;

    ChSystemParallelNSC mphysicalSystem;
    mphysicalSystem.Set_G_acc(ChVector<double>(0, -9.8, 0));
    mphysicalSystem.GetSettings()->solver.tolerance = 1e-5;
    mphysicalSystem.ChangeSolverType(SolverType::BB);

    // Create ground body
    auto floorBody = std::make_shared<ChBodyEasyBox>(20, 2, 20, 3000, true, true, ChMaterialSurface::NSC,
                                                     std::make_shared<collision::ChCollisionModelParallel>());
    floorBody->SetPos(ChVector<>(0, -2, 0));
    floorBody->SetBodyFixed(true);
    mphysicalSystem.Add(floorBody);

    // Add examples of rotational motors
    ExampleA1(mphysicalSystem);
    ExampleA2(mphysicalSystem);
    ExampleA3(mphysicalSystem);
    ExampleA4(mphysicalSystem);

    // Add examples of linear motors
    ExampleB1(mphysicalSystem);
    ExampleB2(mphysicalSystem);
    ExampleB3(mphysicalSystem);
    ExampleB4(mphysicalSystem);

    // Create OpenGL visualization
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Rotational motor", &mphysicalSystem);
    gl_window.SetCamera(ChVector<>(1, 3, -7), ChVector<>(0, 0, 0), ChVector<>(0, 1, 0), 0.5f);
    gl_window.SetRenderMode(opengl::WIREFRAME);

    //
    // Simulate system
    //
    while (gl_window.Active()) {
        mphysicalSystem.DoStepDynamics(step_size);
        gl_window.Render();
    }

    return 0;
}
