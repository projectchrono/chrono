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
// Authors: Alessandro Tasora
// =============================================================================
//
//  Demo code about using motors to impose rotation or translation between parts
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorRotationDriveline.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearDriveline.h"
#include "chrono/physics/ChShaftsMotorSpeed.h"
#include "chrono/physics/ChShaftsMotorAngle.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsGear.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/motion_functions/ChFunction_Sine.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

ChCollisionSystem::Type collision_type = ChCollisionSystem::Type::BULLET;

// Shortcut function that creates two bodies (a slider and a guide) in a given position,
// just to simplify the creation of multiple linear motors in this demo.
void CreateSliderGuide(std::shared_ptr<ChBody>& guide,
                       std::shared_ptr<ChBody>& slider,
                       std::shared_ptr<ChMaterialSurface> material,
                       ChSystem& sys,
                       const ChVector<> mpos) {
    guide = chrono_types::make_shared<ChBodyEasyBox>(4, 0.3, 0.6, 1000, material);
    guide->SetPos(mpos);
    guide->SetBodyFixed(true);
    sys.Add(guide);

    slider = chrono_types::make_shared<ChBodyEasyBox>(0.4, 0.2, 0.5, 1000, material);
    slider->SetPos(mpos + ChVector<>(0, 0.3, 0));
    slider->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.6f, 0.0f));
    sys.Add(slider);

    auto obstacle = chrono_types::make_shared<ChBodyEasyBox>(0.4, 0.4, 0.4, 8000, material);
    obstacle->SetPos(mpos + ChVector<>(1.5, 0.4, 0));
    obstacle->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.2f, 0.2f));
    sys.Add(obstacle);
}

// Shortcut function that creates two bodies (a stator and a rotor) in a given position,
// just to simplify the creation of multiple linear motors in this demo
// (skip this and go to main() for the tutorial)

void CreateStatorRotor(std::shared_ptr<ChBody>& stator,
                       std::shared_ptr<ChBody>& rotor,
                       std::shared_ptr<ChMaterialSurface> material,
                       ChSystem& sys,
                       const ChVector<>& mpos) {
    stator =
        chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, 0.5, 0.1, 1000, material);
    stator->SetPos(mpos);
    stator->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    stator->SetBodyFixed(true);
    sys.Add(stator);

    rotor = chrono_types::make_shared<ChBodyEasyBox>(1, 0.1, 0.1, 1000, material);
    rotor->SetPos(mpos + ChVector<>(0.5, 0, -0.15));
    rotor->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.6f, 0.0f));
    sys.Add(rotor);
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(collision_type);

    // Contact material shared among all objects
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Create a floor that is fixed (that is used also to represent the absolute reference)
    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 2, 20, 3000, material);
    floorBody->SetPos(ChVector<>(0, -2, 0));
    floorBody->SetBodyFixed(true);
    floorBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(floorBody);

    // In the following we will create different types of motors
    // - rotational motors: examples A.1, A.2, etc.
    // - linear motors, examples B.1, B.2 etc.

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
    CreateStatorRotor(stator1, rotor1, material, sys, positionA1);

    // Create the motor
    auto rotmotor1 = chrono_types::make_shared<ChLinkMotorRotationSpeed>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor1->Initialize(rotor1,                // body A (slave)
                          stator1,               // body B (master)
                          ChFrame<>(positionA1)  // motor frame, in abs. coords
    );
    sys.Add(rotmotor1);

    // Create a ChFunction to be used for the ChLinkMotorRotationSpeed
    auto mwspeed =
        chrono_types::make_shared<ChFunction_Const>(CH_C_PI_2);  // constant angular speed, in [rad/s], 1PI/s =180°/s
    // Let the motor use this motion function:
    rotmotor1->SetSpeedFunction(mwspeed);

    // The ChLinkMotorRotationSpeed contains a hidden state that performs the time integration
    // of the angular speed setpoint: such angle is then imposed to the
    // constraint at the positional level too, thus avoiding angle error
    // accumulation (angle drift). Optionally, such positional constraint
    // level can be disabled as follows:
    //
    // rotmotor1->SetAvoidAngleDrift(false);

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
    CreateStatorRotor(stator2, rotor2, material, sys, positionA2);

    // Create the motor
    auto rotmotor2 = chrono_types::make_shared<ChLinkMotorRotationAngle>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor2->Initialize(rotor2,                // body A (slave)
                          stator2,               // body B (master)
                          ChFrame<>(positionA2)  // motor frame, in abs. coords
    );
    sys.Add(rotmotor2);

    // Create a ChFunction to be used for the ChLinkMotorRotationAngle
    auto msineangle = chrono_types::make_shared<ChFunction_Sine>(0,       // phase [rad]
                                                                 0.05,    // frequency [Hz]
                                                                 CH_C_PI  // amplitude [rad]
    );
    // Let the motor use this motion function as a motion profile:
    rotmotor2->SetAngleFunction(msineangle);

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
    CreateStatorRotor(stator3, rotor3, material, sys, positionA3);

    // Create the motor
    auto rotmotor3 = chrono_types::make_shared<ChLinkMotorRotationTorque>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor3->Initialize(rotor3,                // body A (slave)
                          stator3,               // body B (master)
                          ChFrame<>(positionA3)  // motor frame, in abs. coords
    );
    sys.Add(rotmotor3);

    // The torque(time) function:
    auto mtorquetime = chrono_types::make_shared<ChFunction_Sine>(0,   // phase [rad]
                                                                  2,   // frequency [Hz]
                                                                  160  // amplitude [Nm]
    );

    // Let the motor use this motion function as a motion profile:
    rotmotor3->SetTorqueFunction(mtorquetime);

    // EXAMPLE A.4
    //
    // As before, use a ChLinkMotorRotationTorque, but this time compute
    // torque by a custom function. In this example we implement a
    // basic torque(speed) model of a three-phase induction electric motor..

    ChVector<> positionA4(-3, 2, 0);
    std::shared_ptr<ChBody> stator4;
    std::shared_ptr<ChBody> rotor4;
    CreateStatorRotor(stator4, rotor4, material, sys, positionA4);

    // Create the motor
    auto rotmotor4 = chrono_types::make_shared<ChLinkMotorRotationTorque>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor4->Initialize(rotor4,                // body A (slave)
                          stator4,               // body B (master)
                          ChFrame<>(positionA4)  // motor frame, in abs. coords
    );
    sys.Add(rotmotor4);

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
    auto mtorquespeed = chrono_types::make_shared<MyTorqueCurve>();
    mtorquespeed->E2 = 120;
    mtorquespeed->R2 = 80;
    mtorquespeed->X2 = 1;
    mtorquespeed->ns = 6;
    mtorquespeed->mymotor = rotmotor4;

    // Let the motor use this motion function as a motion profile:
    rotmotor4->SetTorqueFunction(mtorquespeed);

    // EXAMPLE A.5
    //
    //
    // - class:   ChLinkMotorRotationDriveline
    // - type:    rotational motor
    // - control: delegated to an embedded user-defined driveline/powertrain
    //
    // This is the most powerful motor type. It allows the creation of
    // generic 1D powertrain inside this 3D motor.
    // Powertrains/drivelines are defined by connecting a variable number of
    // 1D objects such as ChShaft, ChClutch, ChShaftsMotor, etc. In this way, for
    // example, you can represent a drive+flywheel+reducer, hence taking into account
    // of the inertia of the flywheel without the complication of adding a full 3D shape that
    // represents the flywheel, and withoput needing 3D constraint for gears, bearings, etc.
    // The 1D driveline is "interfaced" to the two connected threedimensional
    // parts using two "inner" 1D shafts, each rotating as the connected 3D part;
    // it is up to the user to build the driveline that connects those two shafts.
    // Most often the driveline is like a graph starting at inner shaft 2 (consider
    // it to be the truss for holding the motor drive, also the support for reducers
    // if any) and ending at inner shaft 1 (consider it to be the output, i.e. the
    // slow-rotation spindle).

    ChVector<> positionA5(-3, 2, 1);
    std::shared_ptr<ChBody> stator5;
    std::shared_ptr<ChBody> rotor5;
    CreateStatorRotor(stator5, rotor5, material, sys, positionA5);

    // Create the motor
    auto rotmotor5 = chrono_types::make_shared<ChLinkMotorRotationDriveline>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor5->Initialize(rotor5,                // body A (slave)
                          stator5,               // body B (master)
                          ChFrame<>(positionA5)  // motor frame, in abs. coords
    );
    sys.Add(rotmotor5);

    // You may want to change the inertia of 'inner' 1d shafts, (each has default 1kg/m^2)
    // Note: they adds up to 3D inertia when 3D parts rotate about the link shaft.
    // Note: do not use too small values compared to 3D inertias: it might negatively affect
    // the precision of some solvers; if so, rather diminish the 3D inertia of stator/rotor parts and add to these.
    rotmotor5->GetInnerShaft1()->SetInertia(0.2);  // [kg/m^2]
    rotmotor5->GetInnerShaft2()->SetInertia(0.2);  // [kg/m^2]

    // Now create the driveline. We want to model a drive+reducer sytem.
    // This driveline must connect "inner shafts" of s1 and s2, where:
    //  s1, is the 3D "rotor5"  part A (ex. a robot arm) and
    //  s2, is the 3D "stator5" part B (ex. a robot base).
    // In the following scheme, the motor is [ DRIVE ], the reducer is [ REDUCER ],
    // the shafts ( shown with symbol || to mean inertia) are:
    //  S1: the 1D inner shaft for s1 robot arm (already present in ChLinkMotorRotationDriveline)
    //  S2: the 1D inner shaft for s2 robot base (already present in ChLinkMotorRotationDriveline)
    //  A : the shaft of the electric drive
    //
    //      S2                A                    S1
    // 3d<--||---[ DRIVE ]---||-----[ REDUCER ]----||-->3d
    // s2   ||                      [         ]         s1
    //      ||----------------------[         ]
    //

    // Create 'A', a 1D shaft. This is the shaft of the electric drive, representing its inertia.

    auto my_shaftA = chrono_types::make_shared<ChShaft>();
    my_shaftA->SetInertia(0.03);
    sys.AddShaft(my_shaftA);

    // Create 'DRIVE', the hi-speed motor model - as a simple example use a 'imposed speed' motor: this
    // is the equivalent of the ChLinkMotorRotationSpeed, but for 1D elements:

    auto my_drive = chrono_types::make_shared<ChShaftsMotorSpeed>();
    my_drive->Initialize(my_shaftA,                   // A , the rotor of the drive
                         rotmotor5->GetInnerShaft2()  // S2, the stator of the drive
    );
    sys.Add(my_drive);
    // Create a speed(time) function, and use it in my_drive:
    auto my_driveangle = chrono_types::make_shared<ChFunction_Const>(25 * CH_C_2PI);  // 25 [rps] = 1500 [rpm]
    my_drive->SetSpeedFunction(my_driveangle);

    // Create the REDUCER. We should not use the simple ChShaftsGear because
    // it does not transmit torque to the support. So use ChShaftsPlanetary
    // and use it in ordinary mode, keeping the carrier as truss: so it
    // will connect three parts: the carrier(here the truss), the in shaft, the out shaft.

    auto my_reducer = chrono_types::make_shared<ChShaftsPlanetary>();
    my_reducer->Initialize(rotmotor5->GetInnerShaft2(),  // S2, the carrier (truss)
                           my_shaftA,                    // A , the input shaft
                           rotmotor5->GetInnerShaft1()   // S1, the output shaft
    );
    my_reducer->SetTransmissionRatioOrdinary(1.0 / 100.0);  // ratio between wR/wA
    sys.Add(my_reducer);

    // Btw:  later, if you want, you can access / plot speeds and
    // torques for whatever part of the driveline by putting lines like the following
    // in the  while() {...} simulation loop:
    //
    // GetLog() << " 1D shaft 'A' angular speed: "      << my_shaftA->GetPos_dt() << " [rad/s] \n";
    // GetLog() << " 1D Drive angular speed: rot-stat " << my_drive->GetMotorRot_dt() << " [rad/s] \n";
    // GetLog() << " 1D Drive torque: "                 << my_drive->GetMotorTorque() << " [Ns] \n";
    // GetLog() << " 3D motor angular speed: rot-stat " << rotmotor5->GetMotorRot_dt() << " [rad/s] \n";
    // GetLog() << " 3D motor torque: "                 << rotmotor5->GetMotorTorque() << " [Ns] \n";
    // etc.

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
    CreateSliderGuide(guide1, slider1, material, sys, positionB1);

    // Create the linear motor
    auto motor1 = chrono_types::make_shared<ChLinkMotorLinearPosition>();

    // Connect the guide and the slider and add the motor to the system:
    motor1->Initialize(slider1,               // body A (slave)
                       guide1,                // body B (master)
                       ChFrame<>(positionB1)  // motor frame, in abs. coords
    );
    sys.Add(motor1);

    // Create a ChFunction to be used for the ChLinkMotorLinearPosition
    auto msine = chrono_types::make_shared<ChFunction_Sine>(0,    // phase
                                                            0.5,  // frequency
                                                            1.6   // amplitude
    );
    // Let the motor use this motion function:
    motor1->SetMotionFunction(msine);

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
    CreateSliderGuide(guide2, slider2, material, sys, positionB2);

    // Create the linear motor
    auto motor2 = chrono_types::make_shared<ChLinkMotorLinearSpeed>();

    // Connect the guide and the slider and add the motor to the system:
    motor2->Initialize(slider2,               // body A (slave)
                       guide2,                // body B (master)
                       ChFrame<>(positionB2)  // motor frame, in abs. coords
    );
    sys.Add(motor2);

    // Create a ChFunction to be used for the ChLinkMotorLinearSpeed
    auto msp = chrono_types::make_shared<ChFunction_Sine>(CH_C_PI_2,            // phase
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
    CreateSliderGuide(guide3, slider3, material, sys, positionB3);

    // just for fun: modify the initial speed of slider to match other examples
    slider3->SetPos_dt(ChVector<>(1.6 * 0.5 * CH_C_2PI));

    // Create the linear motor
    auto motor3 = chrono_types::make_shared<ChLinkMotorLinearForce>();

    // Connect the guide and the slider and add the motor to the system:
    motor3->Initialize(slider3,               // body A (slave)
                       guide3,                // body B (master)
                       ChFrame<>(positionB3)  // motor frame, in abs. coords
    );
    sys.Add(motor3);

    // Create a ChFunction to be used for F(t) in ChLinkMotorLinearForce.
    auto mF = chrono_types::make_shared<ChFunction_Const>(200);
    // Let the motor use this motion function:
    motor3->SetForceFunction(mF);

    // Alternative: just for fun, use a sine harmonic whose max force is F=M*A, where
    // M is the mass of the slider, A is the max acceleration of the previous examples,
    // so finally the motion should be quite the same - but without feedback, if hits a disturb, it goes crazy:
    auto mF2 =
        chrono_types::make_shared<ChFunction_Sine>(0,                                                 // phase
                                                   0.5,                                               // frequency
                                                   slider3->GetMass() * 1.6 * pow(0.5 * CH_C_2PI, 2)  // amplitude
        );
    // motor3->SetForceFunction(mF2); // uncomment to test this

    // EXAMPLE B.4
    //
    // As before, use a ChLinkMotorLinearForce, but this time compute
    // F by a user-defined procedure (as a callback). For example, here we write a very
    // basic PID control algorithm that adjusts F trying to chase a sinusoidal position.

    ChVector<> positionB4(0, 0, 0);
    std::shared_ptr<ChBody> guide4;
    std::shared_ptr<ChBody> slider4;
    CreateSliderGuide(guide4, slider4, material, sys, positionB4);

    // Create the linear motor
    auto motor4 = chrono_types::make_shared<ChLinkMotorLinearForce>();

    // Connect the guide and the slider and add the motor to the system:
    motor4->Initialize(slider4,               // body A (slave)
                       guide4,                // body B (master)
                       ChFrame<>(positionB4)  // motor frame, in abs. coords
    );
    sys.Add(motor4);

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
        virtual double SetpointCallback(double x) override {
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
    auto mFcallback = chrono_types::make_shared<MyForceClass>();
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

    // EXAMPLE B.5
    //
    //
    // - class:   ChLinkMotorLinearDriveline
    // - type:    linear motor
    // - control: delegated to an embedded user-defined driveline/powertrain
    //
    // This is the most powerful linear actuator type. It allows the creation of
    // generic 1D powertrain inside this 3D motor.
    // Powertrains/drivelines are defined by connecting a variable number of
    // 1D objects such as ChShaft, ChClutch, ChShaftsMotor, etc. In this way, for
    // example, you can represent a drive+flywheel+reducer+pulley system, hence taking into account
    // of the inertia of the flywheel and the elasticity of the synchro belt without
    // the complication of adding full 3D shapes that represents all parts.
    // The 1D driveline is "interfaced" to the two connected threedimensional
    // parts using two "inner" 1D shafts, each translating as the connected 3D part;
    // it is up to the user to build the driveline that connects those two shafts.
    // Most often the driveline is like a graph starting at inner shaft 2 and ending
    // at inner shaft 1.
    // Note: in the part 2 there is an additional inner shaft that operates on rotation;
    // this is needed because, for example, maybe you want to model a driveline like a
    // drive+screw; you will anchor the drive to part 2 using this rotational shaft; so
    // reaction torques arising because of inner flywheel accelerations can be transmitted to this shaft.

    ChVector<> positionB5(0, 0, 1);
    std::shared_ptr<ChBody> guide5;
    std::shared_ptr<ChBody> slider5;
    CreateSliderGuide(guide5, slider5, material, sys, positionB5);

    // Create the motor
    auto motor5 = chrono_types::make_shared<ChLinkMotorLinearDriveline>();

    // Connect the rotor and the stator and add the motor to the system:
    motor5->Initialize(slider5,               // body A (slave)
                       guide5,                // body B (master)
                       ChFrame<>(positionB5)  // motor frame, in abs. coords
    );
    sys.Add(motor5);

    // You may want to change the inertia of 'inner' 1D shafts, ("translating" shafts: each has default 1kg)
    // Note: they adds up to 3D inertia when 3D parts translate about the link shaft.
    // Note: do not use too small values compared to 3D inertias: it might negatively affect
    // the precision of some solvers; if so, rather diminish the 3D inertia of guide/slider parts and add to these.
    motor5->GetInnerShaft1lin()->SetInertia(3.0);  // [kg]
    motor5->GetInnerShaft2lin()->SetInertia(3.0);  // [kg]
    motor5->GetInnerShaft2rot()->SetInertia(0.8);  // [kg/m^2]

    // Now create the driveline. We want to model a drive+reducer sytem.
    // This driveline must connect "inner shafts" of s1 and s2, where:
    //  s1, is the 3D "slider5" part B (ex. a gantry robot gripper) and
    //  s2, is the 3D "guide5"  part A (ex. a gantry robot base).
    // In the following scheme, the motor is [ DRIVE ], the reducer is [ RACKPINION ],
    // the shafts ( shown with symbol || to mean inertia) are:
    //  S1: the 1D inner shaft for s1 robot arm (already present in ChLinkMotorLinearDriveline)
    //  S2: the 1D inner shaft for s2 robot base (already present in ChLinkMotorLinearDriveline)
    //  B : the shaft of the electric drive
    // Note that s1 and s2 are translational degrees of freedom, differently
    // from ChLinkMotorLinearDriveline.
    //
    //     S2rot              B                      S1lin
    //    <--||---[ DRIVE ]---||-----[ RACKPINION ]----||--> 3d
    // 3d <                          [            ]          s1
    // s2 < S2lin                    [            ]
    //    <--||----------------------[            ]
    //

    // Tell the motor that the inner shaft  'S2rot' is Z, orthogonal to
    // the X direction of the guide (default was X, as for screw actuators)

    motor5->SetInnerShaft2RotDirection(VECT_Z);  // in link coordinates

    // Create 'B', a 1D shaft. This is the shaft of the electric drive, representing its inertia.

    auto my_shaftB = chrono_types::make_shared<ChShaft>();
    my_shaftB->SetInertia(0.33);  // [kg/m^2]
    sys.AddShaft(my_shaftB);

    // Create 'DRIVE', the hispeed motor - as a simple example use a 'imposed speed' motor: this
    // is the equivalent of the ChLinkMotorRotationAngle, but for 1D elements:

    auto my_driveli = chrono_types::make_shared<ChShaftsMotorAngle>();
    my_driveli->Initialize(my_shaftB,                   // B    , the rotor of the drive
                           motor5->GetInnerShaft2rot()  // S2rot, the stator of the drive
    );
    sys.Add(my_driveli);

    // Create a angle(time) function. It could be something as simple as
    //   auto my_functangle = chrono_types::make_shared<ChFunction_Ramp>(0,  180);
    // but here we'll rather do a back-forth motion, made with a repetition of a sequence of 4 basic functions:

    auto my_functsequence = chrono_types::make_shared<ChFunction_Sequence>();
    auto my_funcsigma1 = chrono_types::make_shared<ChFunction_Sigma>(180, 0, 0.5);  // diplacement, t_start, t_end
    auto my_funcpause1 = chrono_types::make_shared<ChFunction_Const>(0);
    auto my_funcsigma2 = chrono_types::make_shared<ChFunction_Sigma>(-180, 0, 0.3);  // diplacement, t_start, t_end
    auto my_funcpause2 = chrono_types::make_shared<ChFunction_Const>(0);
    my_functsequence->InsertFunct(my_funcsigma1, 0.5, 1.0, true);  // fx, duration, weight, enforce C0 continuity
    my_functsequence->InsertFunct(my_funcpause1, 0.2, 1.0, true);  // fx, duration, weight, enforce C0 continuity
    my_functsequence->InsertFunct(my_funcsigma2, 0.3, 1.0, true);  // fx, duration, weight, enforce C0 continuity
    my_functsequence->InsertFunct(my_funcpause2, 0.2, 1.0, true);  // fx, duration, weight, enforce C0 continuity
    auto my_functangle = chrono_types::make_shared<ChFunction_Repeat>(my_functsequence);
    my_functangle->Set_window_length(0.5 + 0.2 + 0.3 + 0.2);
    my_driveli->SetAngleFunction(my_functangle);

    // Create the RACKPINION.
    // It will connect three parts:
    // - S2lin, the carrier (here the truss) to transmit reaction force,
    // - B,     the in (rotational) shaft,
    // - S1lin, the out (translational) shaft.

    auto my_rackpinion = chrono_types::make_shared<ChShaftsPlanetary>();
    my_rackpinion->Initialize(motor5->GetInnerShaft2lin(),  // S2lin, the carrier (truss)
                              my_shaftB,                    // B,     the input shaft
                              motor5->GetInnerShaft1lin()   // S1lin, the output shaft
    );
    my_rackpinion->SetTransmissionRatios(-1, -1.0 / 100.0, 1);
    sys.Add(my_rackpinion);

    // Btw:  later, if you want, you can access / plot speeds and
    // torques for whatever part of the driveline by putting lines like the   following
    // in the  while() {...} simulation loop:
    //
    // GetLog() << " 1D shaft 'B' angular speed: "      << my_shaftB->GetPos_dt() << " [rad/s] \n";
    // GetLog() << " 1D Drive angular speed: rot-stat " << my_driveli->GetMotorRot_dt() << " [rad/s] \n";
    // GetLog() << " 1D Drive torque: "                 << my_driveli->GetMotorTorque() << " [Ns] \n";
    // GetLog() << " 3D actuator speed: rot-stat " << motor5->GetMotorPos() << " [rad/s] \n";
    // GetLog() << " 3D actuator force: "                 << motor5->GetMotorForce() << " [Ns] \n";
    // etc.

    // EXAMPLE B.6
    //
    // - class:   ChLinkMotorLinearPosition
    // - type:    linear motor
    // - control: impose a position by continuously changing it during the while{} simulation
    //
    // We use again the  ChLinkMotorLinearPosition as in EXAMPLE B.1, but
    // this time we change its position using a "brute force" approach, that is:
    // we put a line in the while{...} simulation loop that continuously changes the
    // position by setting a value from some computation.
    //  Well: here one might be tempted to do  motor6->SetMotionFunction(myconst); where
    // myconst is a ChFunction_Const object where you would continuously change the value
    // of the constant by doing  myconst->Set_yconst() in the while{...} loop; this would
    // work somehow but it would miss the derivative of the function, something that is used
    // in the guts of ChLinkMotorLinearPosition. To overcome this, we'll use a  ChFunction_Setpoint
    // function, that is able to guess the derivative of the changing setpoint by doing numerical
    // differentiation each time you call  myfunction->SetSetpoint().
    //  Note: A more elegant solution would be to inherit our custom motion function
    // from  ChFunction_SetpointCallback  as explained in EXAMPLE B.4, and then setting
    // motor6->SetMotionFunction(mycallback); this would avoid polluting the while{...} loop;
    // but sometimes is faster to do the quick & dirty approach of this example.

    ChVector<> positionB6(0, 0, 2);
    std::shared_ptr<ChBody> guide6;
    std::shared_ptr<ChBody> slider6;
    CreateSliderGuide(guide6, slider6, material, sys, positionB6);

    // Create the linear motor
    auto motor6 = chrono_types::make_shared<ChLinkMotorLinearPosition>();

    // Connect the guide and the slider and add the motor to the system:
    motor6->Initialize(slider6,               // body A (slave)
                       guide6,                // body B (master)
                       ChFrame<>(positionB6)  // motor frame, in abs. coords
    );
    sys.Add(motor6);

    // Create a ChFunction to be used for the ChLinkMotorLinearPosition;
    // Note! look later in the while{...} simulation loop, we'll continuously
    // update its value using  motor6setpoint->SetSetpoint();
    auto motor6setpoint = chrono_types::make_shared<ChFunction_Setpoint>();
    // Let the motor use this motion function:
    motor6->SetMotionFunction(motor6setpoint);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Motors");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(1, 3, -7));
    vis->AddTypicalLights();
    vis->AddLightWithShadow(ChVector<>(20.0, 35.0, -25.0), ChVector<>(0, 0, 0), 55, 20, 55, 35, 512,
                            ChColor(0.6f, 0.8f, 1.0f));
    vis->EnableShadows();

    // Modify some setting of the physical system for the simulation, if you want
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(50);

    double timestep = 0.005;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Example B.6 requires the setpoint to be changed in the simulation loop:
        // for example use a clamped sinusoid, just for fun:
        double t = sys.GetChTime();
        double Sp = ChMin(ChMax(2.6 * sin(t * 1.8), -1.4), 1.4);
        motor6setpoint->SetSetpoint(Sp, t);

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
