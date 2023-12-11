using System;
using static ChronoGlobals;

namespace ChronoDemo
{
    internal class Program
    {
        static void Main(string[] args)
        {

            ChVisualSystem.Type vis_type = ChVisualSystem.Type.VSG;
            ChCollisionSystem.Type coll_type = ChCollisionSystem.Type.BULLET;

            // Set the path to the Chrono data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);

            // Simulation parameters
            double gravity = -9.81;
            double time_step = 0.00001;
            double out_step = 2000 * time_step;

            // Parameters for the falling ball
            int ballId = 100;
            double radius = 1;
            double mass = 1000;
            ChVectorD pos = new ChVectorD(0, 2, 0);
            ChQuaternionD rot = new ChQuaternionD(1, 0, 0, 0);
            ChVectorD init_vel = new ChVectorD(0, 0, 0);

            // Parameters for the containing bin
            int binId = 200;
            double width = 2;
            double length = 2;
            ////double height = 1;
            double thickness = 0.1;

            // Create the system
            ChSystemSMC sys = new ChSystemSMC();

            sys.Set_G_acc(new ChVectorD(0, gravity, 0));
            sys.SetCollisionSystemType(coll_type);

            // The following two lines are optional, since they are the default options. They are added for future reference,
            // i.e. when needed to change those models.
            sys.SetContactForceModel(ChSystemSMC.ContactForceModel.Hertz);
            sys.SetAdhesionForceModel(ChSystemSMC.AdhesionForceModel.Constant);

            // Change the default collision effective radius of curvature
            ChCollisionInfo.SetDefaultEffectiveCurvatureRadius(1);

            // Create a material (will be used by both objects)
            ChMaterialSurfaceSMC material = new ChMaterialSurfaceSMC();
            material.SetRestitution(0.1f);
            material.SetFriction(0.4f);
            material.SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

            // Create the falling ball
            ChBody ball = new ChBody();

            ball.SetIdentifier(ballId);
            ball.SetMass(mass);
            // TODO: cannot use operator between double and ChVectorD
            //ChVectorD onev = new ChVectorD(1, 1, 1);
            //ball.SetInertiaXX(0.4 * mass * radius * radius * onev);
            ball.SetPos(pos);
            ball.SetRot(rot);
            ball.SetPos_dt(init_vel);
            // ball.SetWvel_par(new ChVectorD(0,0,3));
            ball.SetBodyFixed(false);

            ChCollisionShapeSphere sphere_coll = new ChCollisionShapeSphere(material, radius);
            ball.AddCollisionShape(sphere_coll, new ChFrameD());
            ball.SetCollide(true);

            ChVisualShapeSphere sphere_vis = new ChVisualShapeSphere(radius);
            sphere_vis.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"));
            sphere_vis.SetOpacity(1.0f);
            ball.AddVisualShape(sphere_vis);

            sys.AddBody(ball);

            // Create container
            ChBody bin = new ChBody();

            bin.SetIdentifier(binId);
            bin.SetMass(1);
            bin.SetPos(new ChVectorD(0, 0, 0));
            bin.SetRot(new ChQuaternionD(1, 0, 0, 0));
            bin.SetBodyFixed(true);

            ChCollisionShapeBox box_coll = new ChCollisionShapeBox(material, width * 2, thickness * 2, length * 2);
            bin.AddCollisionShape(box_coll, new ChFrameD());
            bin.SetCollide(true);

            ChVisualShapeBox box_vis = new ChVisualShapeBox(width * 2, thickness * 2, length * 2);
            box_vis.SetColor(new ChColor(0.8f, 0.2f, 0.2f));
            box_vis.SetOpacity(0.8f);
            bin.AddVisualShape(box_vis);

            sys.AddBody(bin);

            // Create the run-time visualization system

            ChVisualSystemIrrlicht vis = new ChVisualSystemIrrlicht();
            vis.SetWindowSize(800, 600);
            vis.SetWindowTitle("[C#] SMC demonstration");
            vis.Initialize();
            vis.AddLogo();
            vis.AddSkyBox();
            vis.AddTypicalLights();
            vis.AddCamera(new ChVectorD(0, 3, -6));
            vis.AttachSystem(sys);
            vis.AddGrid(0.2, 0.2, 20, 20, new ChCoordsysD(new ChVectorD(0, 0.11, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)),
                                new ChColor(0.1f, 0.1f, 0.1f));

            // The soft-real-time cycle
            double time = 0.0;
            double out_time = 0.0;

            while (vis.Run())
            {
                vis.BeginScene();
                vis.Render();
                vis.RenderFrame(new ChFrameD(chrono.CastToChBodyFrame(ball).GetCoord()), 1.2 * radius);
                vis.EndScene();

                while (time < out_time)
                {
                    sys.DoStepDynamics(time_step);
                    time += time_step;
                }
                out_time += out_step;
            }

        }

    }
}
