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

#include "gtest/gtest.h"

#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include <iostream>

using namespace chrono;

class ChTimestepperHHTvelocity : public ChTimestepperIIorder, public ChTimestepperImplicit {
  public:
    ChTimestepperHHTvelocity(ChIntegrableIIorder* intgr = nullptr)
           : ChTimestepperIIorder(intgr), ChTimestepperImplicit() {
        // Initialize method parameters with some numerical dissipation
        SetAlpha(-0.2);
    }

    //virtual Type GetType() const override { return Type::EULER_IMPLICIT_LINEARIZED; }

    /// Return the associated integrable object.
    virtual ChIntegrable* GetIntegrable() const override { return integrable; }

     /// Set the numerical damping parameter (in the [-1/3, 0] range).
    /// The closer to -1/3, the more damping.
    /// The closer to 0, the less damping (for 0, it is the trapezoidal method).
    /// The method coefficients gamma and beta are set automatically, based on alpha.
    /// Default: -0.2.
    void SetAlpha(double val) {
        alpha = val;
        if (alpha < -CH_1_3)
            alpha = -CH_1_3;
        if (alpha > 0)
            alpha = 0;
        gamma = (1.0 - 2.0 * alpha) / 2.0;
        beta = std::pow((1.0 - alpha), 2) / 4.0;
    }

    /// Return the current value of the method parameter alpha.
    double GetAlpha() { return alpha; }

    /// Perform an integration step.
    virtual void OnAdvance(double dt) override {
        // setup main vectors
        integrable->StateSetup(X, V, A);

        // setup auxiliary vectors
        Ds.setZero(integrable->GetNumCoordsVelLevel(), integrable);
        Dl.setZero(integrable->GetNumConstraints());
        Xnew.setZero(integrable->GetNumCoordsPosLevel(), integrable);
        Vnew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
        Anew.setZero(integrable->GetNumCoordsVelLevel(), integrable);
        Rnew.setZero(integrable->GetNumCoordsVelLevel());
        R.setZero(integrable->GetNumCoordsVelLevel());
        Qc.setZero(integrable->GetNumConstraints());
        L.setZero(integrable->GetNumConstraints());
        Lnew.setZero(integrable->GetNumConstraints());

        // State at current time T
        integrable->StateGather(X, V, T);        // state <- system
        integrable->StateGatherAcceleration(A);  // <- system
        integrable->StateGatherReactions(L);     // <- system

       
        // Extrapolate a prediction as warm start
        Xnew = X + V * dt;  // + dt*dt*0.5*(1-2*beta) * A;
        Vnew = V;  //+ A*dt;
        Lnew = L;

        R.setZero();
        integrable->LoadResidual_F(R, -alpha / (1.0 + alpha));       // R  = -alpha/(1.0+alpha) * f_old
        integrable->LoadResidual_CqL(R, L, -alpha / (1.0 + alpha));  // R += -alpha/(1.0+alpha) * Cq'*l_old

        // Use Newton iteration to solve for v_new
        //
        // [ 1/((1+alpha)*gamma*h)M  -dF/dv  -(h*beta/gamma)*dF/dx   Cq' ] [ Ds ] =
        // [ Cq                                                      0   ] [-Dl ]
        //                [ -1/(1+alpha)*M*(a_new) + (f_new + Cq'*l_new) - (alpha/(1+alpha))(f_old + Cq'*l_old)]
        //                [  gamma/(beta*h)*C                                                                  ]
        //
        unsigned int iteration;
        for (iteration = 0; iteration < max_iters; iteration++) {
            integrable->StateScatter(Xnew, Vnew, T + dt, UpdateFlag::UPDATE_ALL_NO_VISUAL);  // state -> system
            //R.setZero();
            Qc.setZero();
            Rnew = R;                                                         // Rnew  = - (alpha/(1+alpha))(f_old + Cq'*l_old)
            integrable->LoadResidual_F(Rnew, 1.0);                            // Rnew += f_new
            integrable->LoadResidual_Mv(Rnew, Anew, -alpha / (1.0 + alpha));  // Rnew += -1/(1+alpha)*M*(a_new)
            integrable->LoadResidual_CqL(Rnew, Lnew, 1.0);                    // Rnew += Cq'*l_new
            integrable->LoadConstraint_C(Qc, gamma / (beta*dt), Qc_do_clamp,  // Qc= (gamma/(beta*h))*C  (sign flipped later in StateSolveCorrection) 
                                         Qc_clamping);  

            if (verbose)
                std::cout << " HHTvel iteration=" << iteration << "  |R|=" << Rnew.lpNorm<Eigen::Infinity>()
                     << "  |Qc|=" << Qc.lpNorm<Eigen::Infinity>() << std::endl;

            if ((Rnew.lpNorm<Eigen::Infinity>() < abstolS) && (Qc.lpNorm<Eigen::Infinity>() < abstolL))
                break;

            integrable->StateSolveCorrection(  //
                Ds, Dl, Rnew, Qc,              //
                1 / ((1 + alpha)*gamma*dt),    // factor for M
                -1,                            // factor for dF/dv
                -(beta * dt)/gamma,            // factor for dF/dx
                Xnew, Vnew, T + dt,            // not used here (scatter = false)
                false,                         // do not scatter update to Xnew Vnew T+dt before computing correction
                UpdateFlag::UPDATE_ALL_NO_VISUAL,  // no need for full update, since no scatter
                true,                          // always call the solver's Setup
                true                           // always call the solver's Setup analyze phase
            );

            num_step_iters++;
            num_step_setups++;
            num_step_solves++;

            Vnew += Ds;
            Lnew += Dl;  // not -= Dl because we assume StateSolveCorrection flips sign of Dl
            Anew = (1/(gamma*dt))* (Vnew - V - dt*(1-gamma)*A);
            Xnew = X + V * h + A * (h * h * (0.5 - beta)) + Anew * (h * h * beta);
        }

        integrable->StateScatterAcceleration(
            (Vnew - V) * (1 / dt));  // -> system acceleration 

        X = Xnew;
        V = Vnew;
        L = Lnew;
        //R = Rnew; 
        T += dt;

        integrable->StateScatter(X, V, T, UpdateFlag::UPDATE_ALL);  // state -> system
        integrable->StateScatterReactions(L);     // react -> system  
    
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) {};

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) {};

  protected:
    double alpha;  ///< HHT method parameter:  -1/3 <= alpha <= 0
    double gamma;  ///< HHT method parameter:   gamma = 1/2 - alpha
    double beta;   ///< HHT method parameter:   beta = (1 - alpha)^2 / 4

    ChState Xnew;
    ChStateDelta Vnew;
    ChStateDelta Anew;
    ChVectorDynamic<> Lnew;  
    ChVectorDynamic<> Rnew;
};

TEST(ChLinkLockGear, cylindrical) {
    
    // Create a Chrono physical system
    ChSystemNSC sys;

    // Contact material shared among all bodies
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // Create all the rigid bodies.
    double radA = 4;
    double radB = 2;
    double torque = 1000;
    double alpha =  20 * CH_DEG_TO_RAD;  // pressure angle, normal 
    double beta = 10  * CH_DEG_TO_RAD;  // helix angle

    // ...the truss
    auto mbody_truss = chrono_types::make_shared<ChBodyEasyBox>(20, 10, 2, 1000, true, false, mat);
    sys.Add(mbody_truss);
    mbody_truss->SetFixed(true);
    mbody_truss->SetPos(ChVector3d(0, 0, 3));

    // ...the first gear
    auto mbody_gearA = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, radA, 0.2, 1000, true, false, mat);
    sys.Add(mbody_gearA);
    mbody_gearA->SetPos(ChVector3d(0, 0, -1));

    // ...the second gear bearing: motor that imposes (zero) rotation speed 
    auto link_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_motor->Initialize(mbody_gearA, mbody_truss, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    link_motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
    sys.AddLink(link_motor);

    // ...the second gear
    double interaxis12 = radA + radB;
    auto mbody_gearB = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, radB, 0.2, 1000, true, false, mat);
    sys.Add(mbody_gearB);
    mbody_gearB->SetPos(ChVector3d(interaxis12, 0, -1));
    
    // ... the second gear bearing
    auto link_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revolute->Initialize(mbody_gearB, mbody_truss, ChFrame<>(ChVector3d(interaxis12, 0, 0), QUNIT));
    sys.AddLink(link_revolute);

    
    // ...the gear constraint between the two wheels A and B.
    auto link_gearAB = chrono_types::make_shared<ChLinkLockGear>();
    link_gearAB->Initialize(mbody_gearA, mbody_gearB, ChFrame<>());
    link_gearAB->SetFrameShaft1(ChFrame<>(VNULL));
    link_gearAB->SetFrameShaft2(ChFrame<>(VNULL));
    link_gearAB->SetTransmissionRatio(radA / radB);
    link_gearAB->SetEnforcePhase(true); // needed if compliant gear is used
    link_gearAB->SetPressureAngle(alpha);  
    link_gearAB->SetPitchAngle(beta);  
    sys.AddLink(link_gearAB);

    sys.SetGravitationalAcceleration(VNULL);


    //
    // TEST 1 : apply torque to wheel 2 and see if reactions on bearings are correct
    //

    // ...apply torque to second gear
    auto mtorque = chrono_types::make_shared<ChForce>();
    mtorque->SetMode(ChForce::TORQUE);
    mtorque->SetBody(mbody_gearB.get());
    mtorque->SetDir(ChVector3d(0, 0, 1));
    mtorque->SetMforce(torque);
    mbody_gearB->AddForce(mtorque);


    // Simulate few steps
    double timestep = 0.001;
    while (sys.GetChTime()<0.02) {

        // Advance simulation by one step
        sys.DoStepDynamics(timestep);

        std::cout << "test1 time: " << sys.GetChTime() << "  angle=" << mbody_gearB->GetRotAngle() * CH_RAD_TO_DEG << "\n";
    }
    

    // check if corresponds to P contact force, from analytical solutions for helical gears 
    double P_t = torque / radB;  // tangent component 
    double P_r = P_t * (tan(alpha)/cos(beta)); // radial component
    double P_a = P_t * tan(beta);              // axial component

    /*
    std::cout << "Theoretical contact force:   P_t=" << P_t << "  P_r=" << P_r << "  P_a=" << P_a
              << "     P=" << sqrt(P_t * P_t + P_r * P_r + P_a * P_a) << std::endl;
    std::cout << "Reaction on bearing 1: " << link_motor->GetReaction2().force << std::endl;
    std::cout << "Reaction on bearing 2: " << link_revolute->GetReaction2().force << std::endl;
    std::cout << "Contact force: " << link_gearAB->GetReaction2().force.x() << std::endl;
    */

    // check equilibrium in radial dir:
    ASSERT_TRUE(std::fabs(link_revolute->GetReaction2().force.x() - P_r) < 1e-4);
    // check equilibrium in axial dir:
    ASSERT_TRUE(std::fabs(link_revolute->GetReaction2().force.z() - P_a) < 1e-4);
    // check equilibrium in tangent dir:
    ASSERT_TRUE(std::fabs(link_revolute->GetReaction2().force.y() - P_t) < 1e-4);


    //
    // TEST 2 : make the gear a bit compliant and see some small relative rotation
    //

    // Same setup as before, same applied torque, but now make the gear
    // teeth a bit compliant. Do this by providing a teeth equivalent stiffness (i.e. 
    // a stiffness in [N/m] for the tangential direction, that already consider the 
    // fact that multiple teeth are engaged, on average. Also introduce a damping coefficient
    // otherwise it will cause too many vibrations and won't settle to the steady state that we want
    // to measure.

    double teeth_K = 1e6;      // [N/m] teeth equivalent stiffness
    double teeth_D_coeff = 0.01;  // stiffness-proportional damping, as beta damping in Rayleigh damping

    link_gearAB->SetTeethStiffness(teeth_K);  
    link_gearAB->SetTeethDamping(teeth_D_coeff * teeth_K);

    
    // TEST EULER IMPLICIT 
    auto mystepper = chrono_types::make_shared<ChTimestepperEulerImplicit>(&sys);
    mystepper->SetStepControl(false);
    mystepper->SetJacobianUpdateMethod(ChTimestepperHHT::JacobianUpdate::EVERY_ITERATION);
    mystepper->SetMaxIters(1);
    sys.SetTimestepper(mystepper);
    
    /*
    // TEST HHT
    auto mystepper = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    mystepper->SetStepControl(false);
    mystepper->SetJacobianUpdateMethod(ChTimestepperHHT::JacobianUpdate::EVERY_ITERATION);
    mystepper->SetMaxIters(1);
    sys.SetTimestepper(mystepper);
    */

    sys.SetSolverType(ChSolver::Type::MINRES);
    sys.GetSolver()->AsIterative()->SetMaxIterations(250);


    timestep = 0.005;
    while (sys.GetChTime() < 4) {
        // Advance simulation by one step
        sys.DoStepDynamics(timestep);

        std::cout << "test 2 time: " << sys.GetChTime() << "  angle_B=" << mbody_gearB->GetRotAngle() * CH_RAD_TO_DEG
                  << "\n";
    }

    //double alpha_tangent = atan(tan(alpha) / cos(beta));  // effective pressure angle in tangent plane
    double teeth_K_tangent = teeth_K * (cos(alpha) * cos(alpha)) * cos(beta)*cos(beta);

    double expected_deform_B = (torque / radB) / (teeth_K_tangent);  
    double expected_angle_B = (expected_deform_B / radB) * CH_RAD_TO_DEG;  // expected angular 'slip' due to teeth compliance

    std::cout << "test 2  expected angular slip because of compliance: " << expected_angle_B << " [deg] \n";
    std::cout << "test 2  expected deformation of teeth: " << expected_deform_B << " [m] \n";
    std::cout << "Theoretical contact force:   P_t=" << P_t << "  P_r=" << P_r << "  P_a=" << P_a
              << "     P=" << sqrt(P_t * P_t + P_r * P_r + P_a * P_a) << std::endl;

    // check angular slip is what expected because of teeth compliance:
    ASSERT_TRUE(std::fabs(mbody_gearB->GetRotAngle() * CH_RAD_TO_DEG - expected_angle_B) < 1e-4);
}
