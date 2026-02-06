import pytest
import pychrono as chrono

@pytest.mark.parametrize("method", [chrono.ChContactMethod_NSC, chrono.ChContactMethod_SMC])
def test_simulate(method):
    # =============================================================================
    # SETUP
    # =============================================================================
    
    system = None
    material = None
    
    if method == chrono.ChContactMethod_SMC:
        print("Using PENALTY method.")

        use_mat_properties = False
        stiff_contact = True
        
        force_model = chrono.ChSystemSMC.Hooke
        tdispl_model = chrono.ChSystemSMC.OneStep

        sys = chrono.ChSystemSMC()
        sys.UseMaterialProperties(use_mat_properties)
        sys.SetContactForceModel(force_model)
        sys.SetTangentialDisplacementModel(tdispl_model)
        sys.SetContactStiff(stiff_contact)
        system = sys

        young_modulus = 2e4
        friction = 0.4
        restitution = 0
        adhesion = 0

        kn = 2e4
        gn = 5e2
        kt = 0
        gt = 0

        mat = chrono.ChContactMaterialSMC()
        mat.SetYoungModulus(young_modulus)
        mat.SetRestitution(restitution)
        mat.SetFriction(friction)
        mat.SetAdhesion(adhesion)
        mat.SetKn(kn)
        mat.SetGn(gn)
        mat.SetKt(kt)
        mat.SetGt(gt)
        material = mat    
    else:
        print("Using COMPLEMENTARITY method.")

        system = chrono.ChSystemNSC()

        friction = 0.4
        restitution = 0
        
        mat = chrono.ChContactMaterialNSC()
        mat.SetRestitution(restitution)
        mat.SetFriction(friction)
        material = mat
        

    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    gravity = -9.81
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, gravity))
    
    # Create the falling balls
    num_balls = 8
    balls = []
    total_weight = 0.0
    
    radius = 0.05
    mass = 5.0
    
    pos = chrono.ChVector3d(0, 0, 0.06)
    rot = chrono.ChQuaterniond(1, 0, 0, 0)
    init_vel = chrono.ChVector3d(0, 0, 0)
    init_omg = chrono.ChVector3d(0, 0, 0)
    
    for i in range(num_balls):
        ball = chrono.ChBody()

        ball.SetMass(mass)
        # ball.SetInertiaXX (0.4 * mass * radius * radius * chrono.ChVector3d(1, 1, 1))
        ball.SetInertiaXX (chrono.ChVector3d(1, 1, 1) * 0.4 * mass * radius * radius)
        ball.SetPos(pos + chrono.ChVector3d(i * 2 * radius, i * 2 * radius, 0))
        ball.SetRot(rot)
        ball.SetPosDt(init_vel)
        ball.SetAngVelParent(init_omg)
        ball.EnableCollision(True)
        ball.SetFixed(False)
        
        ct_shape = chrono.ChCollisionShapeSphere(material, radius)
        ball.AddCollisionShape(ct_shape)
        
        system.AddBody(ball)
        balls.append(ball)
        
        total_weight += ball.GetMass()
        
    total_weight *= gravity
    print(f"Total weight = {total_weight}")
    
    # Create container box
    bin_width = 20.0
    bin_length = 20.0
    bin_thickness = 0.1
    ground = chrono.CreateBoxContainer(system, material, chrono.ChVector3d(bin_width, bin_length, 2 * radius), bin_thickness)
    
    # -------------------
    # Setup linear solver
    # -------------------

    print("Using default solver.")
    if system.GetSolver().IsIterative():
        system.GetSolver().AsIterative().SetMaxIterations(100)
        system.GetSolver().AsIterative().SetTolerance(5e-9)
    
    # -------------------
    # Setup integrator
    # -------------------

    if method == chrono.ChContactMethod_SMC:
        print("Using HHT integrator.")
        system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
        integrator = chrono.CastToChTimestepperHHT(system.GetTimestepper())
        integrator.SetAlpha(0.0)
        integrator.SetMaxIters(100)
        integrator.SetAbsTolerances(1e-08)
    else:
        print("Using default integrator.")

    # =============================================================================
    
    end_time = 3.0 # total simulation time
    start_time = 0.25 # start check after this period
    time_step = 5e-3

    rtol = 1e-3 # validation relative error
    
    while system.GetChTime() < end_time:
        system.DoStepDynamics(time_step)
        
        contact_force = ground.GetContactForce()
        
        if system.GetChTime() > start_time:
            ratio = contact_force.z / total_weight
            deviation = abs(1 - ratio)
            
            assert deviation < rtol