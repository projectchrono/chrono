#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vsg/ChVSGApp.h"

using namespace chrono;
using namespace geometry;
using namespace chrono::vsg3d;

// ================================================================================ //
//                                  Helper Functions
// ================================================================================ //

Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
    const auto& x = v(0);
    const auto& y = v(1);
    const auto& z = v(2);

    Eigen::Matrix3d mat;
    mat <<
         0, -z,  y,
         z,  0, -x,
        -y,  x,  0;
    
    return mat;
};

// -----------------------------------------------------------------------------------

Eigen::Vector3d orthogonal_vector(Eigen::Vector3d &v1)
{
    auto abs = v1.cwiseAbs();
    Eigen::VectorXd dummy = Eigen::VectorXd::Ones(3,1);
    Eigen::VectorXd::Index max_index;
    Eigen::VectorXd::Index i = abs.maxCoeff(&max_index);
    dummy(max_index) = 0;

    Eigen::Vector3d v = (skew(v1) * dummy).normalized();
    return v;

};

// -----------------------------------------------------------------------------------

Eigen::Matrix3d triad(Eigen::Vector3d &v1)
{
    Eigen::Vector3d k = v1.normalized();
    Eigen::Vector3d i = orthogonal_vector(k).normalized();
    Eigen::Vector3d j = skew(k) * i;

    Eigen::Matrix3d mat;
    mat.col(0) = i;
    mat.col(1) = j;
    mat.col(2) = k;

    return mat;

};

// -----------------------------------------------------------------------------------

ChMatrix33<double> GetCylinderOrientation(const ChVector<> &p1, const ChVector<> &p2)
{
    auto v = Eigen::Vector3d{(p2 - p1).x(), 
                             (p2 - p1).y(), 
                             (p2 - p1).z()};
    auto frame = triad(v);
    auto x_vector = ChVector<>(frame.col(0)(0), frame.col(0)(1), frame.col(0)(2));
    auto y_vector = ChVector<>(frame.col(1)(0), frame.col(1)(1), frame.col(1)(2));
    auto z_vector = ChVector<>(frame.col(2)(0), frame.col(2)(1), frame.col(2)(2));
    
    auto matrix = ChMatrix33<double>(x_vector, y_vector, z_vector);

    return matrix;
};

// ================================================================================ //



// ================================================================================ //
//                                  Main Function
// ================================================================================ //

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Here we creat the four points representing our fourbar mechanism as ChVector
    // objects, so we can use them later to specify the location and orientation of 
    // our bodies, geometries and joints.
    auto point_A = ChVector<>(0, 0, 0);
    auto point_B = ChVector<>(0, 0, 2);
    auto point_C = ChVector<>(-7.5, -8.5, 6.5);
    auto point_D = ChVector<>(-4.0, -8.5,   0);

    // ===============================================================================

    // HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
    // --------------------------------------------------

    // 1- Create a Chrono physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.
    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, -9.81));


    // ===============================================================================


    // 2- Create the rigid bodies of the fourbar mechanical system
    //    (a crank, a rod, a rocker and the ground), maybe setting position/mass/inertias of
    //    their center of mass (COG) etc.

    // Creating the ground part
    // --------------------------------
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetBodyFixed(true);  // Fixing the ground.
    ground->SetName("ground");
    system.AddBody(ground);


    // Creating the crank part and its geometry shape asset.
    // -----------------------------------------------------
    auto crank = chrono_types::make_shared<ChBody>();
    crank->SetPos(ChVector<>(0, 0, 1));  // position of COG of crank
    
    // Creating the crank shape as a cylinder connecting points A and B.
    auto crank_shape = chrono_types::make_shared<ChCylinderShape>();
    crank_shape->GetCylinderGeometry().p1 = point_A;
    crank_shape->GetCylinderGeometry().p2 = point_B;
    crank_shape->GetCylinderGeometry().rad = 0.3;

    // Specifying the orientation of the cylinder correctly
    crank_shape->Rot = GetCylinderOrientation(point_A, point_B);

    // Assigning the shape to the body
    crank->AddAsset(crank_shape);
    
    // Adding the body to our system
    system.AddBody(crank);


    // Creating the rod part and its geometry shape asset.
    // ---------------------------------------------------
    auto rod = chrono_types::make_shared<ChBody>();
    rod->SetPos(ChVector<>(-3.75, -4.25, 4.25));  // position of COG of rod

    // Creating the crank shape as a cylinder connecting points B and C.
    auto rod_shape = chrono_types::make_shared<ChCylinderShape>();
    rod_shape->GetCylinderGeometry().p1 = point_B;
    rod_shape->GetCylinderGeometry().p2 = point_C;
    rod_shape->GetCylinderGeometry().rad = 0.3;

    // Specifying the orientation of the cylinder correctly
    rod_shape->Rot = GetCylinderOrientation(point_B, point_C);

    // Assigning the shape to the body
    rod->AddAsset(rod_shape);

    // Adding the body to our system
    system.AddBody(rod);


    // Creating the rocker part and its geometry shape asset.
    // ------------------------------------------------------
    auto rocker = chrono_types::make_shared<ChBody>();
    rocker->SetPos(ChVector<>(-5.75, -8.50, 3.25));  // position of COG of rocker

    // Creating the rocker shape as a cylinder connecting points C and D.
    auto rocker_shape = chrono_types::make_shared<ChCylinderShape>();
    rocker_shape->GetCylinderGeometry().p1 = point_C;
    rocker_shape->GetCylinderGeometry().p2 = point_D;
    rocker_shape->GetCylinderGeometry().rad = 0.3;

    // Specifying the orientation of the cylinder correctly
    rocker_shape->Rot = GetCylinderOrientation(point_C, point_D);

    // Assigning the shape to the body
    rocker->AddAsset(rocker_shape);

    // Adding the body to our system
    system.AddBody(rocker);

    
    // ===============================================================================


    // 3- Create constraints: the mechanical joints between the rigid bodies.

    // Actuation Motor
    // ---------------
    // Creating a motor between the Crank and the ground. For our configuration, the
    // motor rotation is about the x-axis of our coordinate system. Therefore, we have
    // to orient the default local z-axis of the motor along the gobal x-axis.
    auto motor_quat = ChQuaternion<>(1, 0, 0, 0);
    motor_quat.Q_from_AngAxis(CH_C_PI/2, ChVector<>(0, 1, 0));
    std::cout << motor_quat.GetZaxis() << "\n";

    auto my_link_AB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_link_AB->Initialize(ground, crank, ChFrame<>(point_A, motor_quat));
    system.AddLink(my_link_AB);
    auto my_speed_function = chrono_types::make_shared<ChFunction_Const>(2*CH_C_PI);  // speed rad/sec
    my_link_AB->SetSpeedFunction(my_speed_function);

    // Spherical Joint connecting Crank to the Rod at point B
    // ------------------------------------------------------
    auto my_link_BC = chrono_types::make_shared<ChLinkLockSpherical>();
    my_link_BC->Initialize(crank, rod, ChCoordsys<>(point_B));
    system.AddLink(my_link_BC);

    // Spherical Joint connecting Rod to the Rocker at point C
    // -------------------------------------------------------
    auto my_link_CD = chrono_types::make_shared<ChLinkLockSpherical>();
    my_link_CD->Initialize(rod, rocker, ChCoordsys<>(point_C));
    system.AddLink(my_link_CD);

    // Revolute Joint connecting Rocker to the Ground at point D
    // ---------------------------------------------------------
    // For our configuration, this revolute joint is oriented along the y-axis of our 
    // coordinate system. Therefore, we have to orient the default local z-axis of 
    // the joint along the gobal y-axis.
    auto joint_D_orientation = ChQuaternion<>(1, 0, 0, 0);
    joint_D_orientation.Q_from_AngAxis(CH_C_PI/2, ChVector<>(1, 0, 0));
    std::cout << joint_D_orientation.GetZaxis() << "\n";

    auto my_link_DA = chrono_types::make_shared<ChLinkLockRevolute>();
    my_link_DA->Initialize(rocker, ground, ChCoordsys<>(point_D, joint_D_orientation));
    system.AddLink(my_link_DA);


    // ===============================================================================


    // VISUALIZATION APP
    // =================
    ChVSGApp app;

    app.setTimeStep(0.005);
    app.setOutputStep(0.015);
    app.setUpVector(ChVector<>(0.0, 0.0, 1.0));

    bool ok = app.Initialize(800, 600, "VSG Viewer", &system);

    double modelTime = 0.0;
    double maxModelTime = 10.0;
    ChTimer timer;
    timer.reset();  // mandatory on macos, doesn't do no harm on windows and linux
    timer.start();

    while (app.GetViewer()->advanceToNextFrame()) {
        modelTime = system.GetChTime();
        if (false) { //(modelTime >= maxModelTime) {
            GetLog() << "Max. model time of " << maxModelTime << " s reached. Terminate.\n";
            timer.stop();
            GetLog() << "Max. wallclock time is " << timer.GetTimeSeconds() << " s.\n";
            app.GetViewer()->close();
        }
        app.doTimeStep();
        app.Render();
    }
    return 0;
}
