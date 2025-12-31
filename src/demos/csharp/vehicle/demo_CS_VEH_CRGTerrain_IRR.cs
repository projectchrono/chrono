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
// Authors: Radu Serban, Rainer Gericke, Josh Diyn
// =============================================================================
//
// Demonstration of an OpenCRG terrain and different driver controllers.
// This demo also enables the use of a dynamic vehicle type through input prompt
//
// The default world frame is ISO (Z up, X forward, Y to the left).
// Not currently working is that this this demo can be set up to work with
// a non-ISO frame by un-commenting the input request to set Y-Up. This will
// use a world frame with Y up, X forward, and Z to the right.
// 
//
// NOTES:
// (1) changing the world frame from the ISO default must be done *before* any
//     other Chrono::Vehicle library calls.
// (2) modifications to user code to use a different world frame are minimal:
//     - set the desired world frame
//     - properly set vehicle initial position (e.g. initial height above terrain)
//     - adjust light locations in the run-time visualization system
// (3) This program REQUIRES the Microsoft.CSharp library due to the use of
//      dynamic types (i.e. private dynamic m_driver and private
//      dynamic m_steeringController). Since these types are not known at
//      run-time, but are determined using the MyDriver method.
//      The Microsoft.CSharp library can be added through NuGet or Add
//      References in VS. On Unix systems with .NET use the following bash:
//      "dotnet add package Microsoft.CSharp" otherwise if using Mono ensure
//      you reference required libraries. This code has only been tested on
//      Windows x64 with Visual Studio.
//
// =============================================================================

using System;
using static chrono;
using static ChronoGlobals;
using static chrono_vehicle;
using System.IO;

namespace ChronoDemo
{
    internal class CRGTerrainDemo
    {
        // ---------------
        // Setup - Globals
        // ---------------
        // Enums for driver and tire models
        public enum DriverModelType
        {
            PID,      // pure PID lateral controller with constant speed controller
            STANLEY,  // geometrical P heading and PID lateral controller with constant speed controller
            XT,       // alternative PID lateral controller with constant speed controller
            SR,       // alternative PID lateral controller with constant speed controller
            HUMAN     // simple realistic human driver
        };

        // Road visualization (mesh or boundary lines)
        public static bool useMesh = false; // N.B. runs slow with mesh in IRR (both c++ and c#) vs the VSG version.
        // Desired vehicle speed (m/s)
        public static double targetSpeed = 12;
        // Minimum / maximum speed (m/s) for Human driver type
        public static double minimumSpeed = 12;
        public static double maximumSpeed = 30;

        // =============================================================================
        //
        // ------------------------------------------------  
        // Wrapper around a driver system of specified type
        // ------------------------------------------------
        // Return chosen driver type
        static DriverModelType DriverModelFromString(string str)
        {
            switch (str.ToUpper()) // parse input as case-insensitive
            {
                case "HUMAN": return DriverModelType.HUMAN;
                case "PID": return DriverModelType.PID;
                case "STANLEY": return DriverModelType.STANLEY;
                case "SR": return DriverModelType.SR;
                case "XT": return DriverModelType.XT;
                default:
                    Console.Error.WriteLine("Invalid driver model type." + str + " does not represent a valid DriverModelType (HUMAN/PID/SR/XT) - returned DriverModelType.HUMAN");
                    return DriverModelType.HUMAN; // Default or handle error
            }
        }

        class MyDriver
        {
            // Declarations
            private DriverModelType m_type;
            private string m_driverType;
            private dynamic m_driver; // set root class that's common
            private dynamic m_steeringController;

            public MyDriver(DriverModelType type, ChWheeledVehicle vehicle, ChBezierCurve path, double roadWidth)
            {
                // set types
                m_type = type;
                m_steeringController = null;
                // Initialize based on the type
                switch (type)
                {
                    case DriverModelType.PID:
                        m_driverType = "PID";
                        var driverPID = new ChPathFollowerDriver(vehicle, path, "my_path", targetSpeed);
                        driverPID.GetSteeringController().SetLookAheadDistance(5);
                        driverPID.GetSteeringController().SetGains(0.5, 0, 0);
                        driverPID.GetSpeedController().SetGains(0.4, 0, 0);
                        m_driver = driverPID;
                        m_steeringController = driverPID.GetSteeringController();
                        break;
                    case DriverModelType.STANLEY:
                        m_driverType = "STANLEY";
                        var driverStanley = new ChPathFollowerDriver(vehicle, path, "my_path", targetSpeed);
                        driverStanley.GetSteeringController().SetLookAheadDistance(5.0);
                        driverStanley.GetSteeringController().SetGains(0.5, 0.0, 0.0);
                        driverStanley.GetSpeedController().SetGains(0.4, 0, 0);
                        m_driver = driverStanley;
                        m_steeringController = driverStanley.GetSteeringController();
                        break;
                    case DriverModelType.XT:
                        m_driverType = "XT";
                        var driverXT = new ChPathFollowerDriverXT(vehicle, path, "my_path", targetSpeed, vehicle.GetMaxSteeringAngle());
                        driverXT.GetSteeringController().SetLookAheadDistance(5);
                        driverXT.GetSteeringController().SetGains(0.4, 1, 1, 1);
                        driverXT.GetSpeedController().SetGains(0.4, 0, 0);
                        m_driver = driverXT;
                        m_steeringController = driverXT.GetSteeringController();
                        break;
                    case DriverModelType.SR:
                        m_driverType = "SR";
                        var driverSR = new ChPathFollowerDriverSR(vehicle, path, "my_path", targetSpeed, vehicle.GetMaxSteeringAngle(), 3.2);
                        driverSR.GetSteeringController().SetGains(0.1, 5);
                        driverSR.GetSteeringController().SetPreviewTime(0.5);
                        driverSR.GetSpeedController().SetGains(0.4, 0, 0);
                        m_driver = driverSR;
                        m_steeringController = driverSR.GetSteeringController();
                        break;
                    case DriverModelType.HUMAN:
                        m_driverType = "HUMAN";

                        // Driver model read from JSON file
                        //// var driverHUMAN = new ChHumanDriver(
                        ////    GetVehicleDataFile("hmmwv/driver/HumanController.json"), vehicle, path, "my_path",
                        ////    road_width, vehicle.GetMaxSteeringAngle(), 3.2);

                        var driverHUMAN = new ChHumanDriver(vehicle, path, "my_path", roadWidth, vehicle.GetMaxSteeringAngle(), 3.2);
                        driverHUMAN.SetPreviewTime(0.5);
                        driverHUMAN.SetLateralGains(0.1, 2);
                        driverHUMAN.SetLongitudinalGains(0.1, 0.1, 0.2);
                        driverHUMAN.SetSpeedRange(minimumSpeed, maximumSpeed);
                        m_driver = driverHUMAN;
                        break;
                }
            }

            public DriverInputs GetInputs() { return m_driver.GetInputs(); }
            public void Initialize() { m_driver.Initialize(); }
            public void Synchronize(double time) { m_driver.Synchronize(time); }
            public void Advance(double step) { m_driver.Advance(step); }
            public string GetDriverType() { return m_driverType; }

            public ChVector3d GetTargetLocation()
            {
                if (m_type == DriverModelType.HUMAN)
                    return m_driver.GetTargetLocation();
                else
                    return m_steeringController.GetTargetLocation();
            }

            public ChVector3d GetSentinelLocation()
            {
                if (m_type == DriverModelType.HUMAN)
                    return m_driver.GetSentinelLocation();
                else
                    return m_steeringController.GetSentinelLocation();
            }

            public void PrintStats()
            {
                if (m_type != DriverModelType.HUMAN)
                    return;

                var driverHUMAN = m_driver;
                Console.WriteLine();
                Console.WriteLine($"Traveled Distance    = {driverHUMAN.GetTraveledDistance()} m");
                Console.WriteLine($"Average Speed        = {driverHUMAN.GetAverageSpeed()} m/s");
                Console.WriteLine($"Maximum Speed        = {driverHUMAN.GetMaxSpeed()} m/s");
                Console.WriteLine($"Minimum Speed        = {driverHUMAN.GetMinSpeed()} m/s");
                Console.WriteLine($"Maximum Lateral Acc. = {driverHUMAN.GetMaxLatAcc()} m^2/s");
                Console.WriteLine($"Minimum Lateral Acc. = {driverHUMAN.GetMinLatAcc()} m^2/s");
            }

        }

        // ----------------
        // Main Function
        // ----------------
        static void Main(string[] args)
        {
            Console.WriteLine("Copyright (c) 2017 projectchrono.org");
            Console.WriteLine("Chrono version: " + CHRONO_VERSION);

            // TODO: correct CHRONO_VERSION call
            //Console.WriteLine(chrono.GetLog() + "Copyright (c) 2017 projectchrono.org\nChrono version: " + CHRONO_VERSION + "\n\n");

            // Set the path to the Chrono data files and Chrono::Vehicle data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);
            chrono_vehicle.SetVehicleDataPath(CHRONO_VEHICLE_DATA_DIR);

            // Simulation specs
            double stepSize = 3e-3;
            double tireStepSize = 1e-3;

            // Tyre models - needs work (local/global c# inheritance issues)
            // public enum TireModelType { LUGRE, FIALA, PACEJKA, TMSIMPLE, TMEASY };
            // TireModelType tireModel = new TireModelType();

            // Output frame images
            bool outputImages = false;
            double fps = 60;
            string outDir = chrono.GetChronoOutputPath() + "OPENCRG_DEMO";

            // Set up parameter defaults and command-line arguments
            DriverModelType driverType = DriverModelType.HUMAN;
            string crgRoadFile = GetChronoDataFile("vehicle/terrain/crg_roads/RoadCourse.crg");
            bool yUp = false;


            // ---------------
            // Set World Frame
            // ---------------
            
            // TODO: Y-Up functionality not currently functioning correctly.
            /*
            Console.WriteLine("Use Y-UP world frame? (default false):");
            string yupInput = Console.ReadLine();
            if (!string.IsNullOrWhiteSpace(yupInput) && bool.TryParse(yupInput, out bool yupResult))
            {
                yUp = yupResult;
            }
            else
            {
                // If input is null, empty, or only whitespace, or if parsing fails, do nothing but give feedback
                // (default is set above in declarations)
                Console.WriteLine("Could not determine input value. Y-Up will be set to false.");
            }

            */

            if (yUp)
                ChWorldFrame.SetYUP();

            // Todo: Fix this. Not the most graceful print out but using this:
            // Console.WriteLine("World Frame\n" + ChWorldFrame.Rotation().ToString());
            // only prints the type. Not the values.
            var rot = ChWorldFrame.Rotation();
            Console.WriteLine("World Frame");
            Console.WriteLine($"[{rot.getitem(0, 0)}, {rot.getitem(0, 1)}, {rot.getitem(0, 2)}]");
            Console.WriteLine($"[{rot.getitem(1, 0)}, {rot.getitem(1, 1)}, {rot.getitem(1, 2)}]");
            Console.WriteLine($"[{rot.getitem(2, 0)}, {rot.getitem(2, 1)}, {rot.getitem(2, 2)}]");

            Console.WriteLine($"Vertical direction: {ChWorldFrame.Vertical().x}, {ChWorldFrame.Vertical().y}, {ChWorldFrame.Vertical().z}");
            Console.WriteLine($"Forward direction: {ChWorldFrame.Forward().x}, {ChWorldFrame.Forward().y}, {ChWorldFrame.Forward().z}");


            // ----------------
            // Process Inputs
            // ----------------

            Console.WriteLine("Enter vehicle type (HMMWV, UAZBUS, KRAZ, SEDAN, MAN, CITYBUS, GATOR):");
            string vehicleInput = Console.ReadLine();

            Console.WriteLine("Controller model type - PID, STANLEY, XT, SR, HUMAN (default HUMAN):");
            string modelInput = Console.ReadLine();
            if (!string.IsNullOrWhiteSpace(modelInput))
            {
                driverType = DriverModelFromString(modelInput);
            }

            Console.WriteLine($"Enter CRG road filename (default {crgRoadFile}):");
            string fileInput = Console.ReadLine();
            if (!string.IsNullOrWhiteSpace(fileInput))
            {
                crgRoadFile = fileInput;
            }
            else
            {
                // If input is null, empty, or only whitespace, do nothing. This isn't entirely necessary
                // (error should print to console regarding CRG failure to read/load at intialise)
            }


            // ----------------
            // Output directory
            // ----------------
            try
            {
                // Create the output directory. If it already exists, no exception is thrown.
                Directory.CreateDirectory(outDir);

            }
            catch (IOException ioEx)
            {
                // Handle or log the IO Exception specifically
                Console.WriteLine("IO Exception occurred: " + ioEx.Message);
            }
            catch (UnauthorizedAccessException unAuthEx)
            {
                // Handle lack of permission, etc.
                Console.WriteLine("UnauthorizedAccessException: " + unAuthEx.Message);
            }
            catch (Exception ex) // Catch-all for any other exceptions
            {
                Console.WriteLine("An error occurred while creating directories: " + ex.Message);
                Environment.Exit(1);
            }

            // ----------------------------
            // Create the containing system
            // ----------------------------

            ChSystemSMC sys = new ChSystemSMC();
            sys.SetCollisionSystemType(ChCollisionSystem.Type.BULLET);
            // Set the gravity. Must be set after Y-UP
            ChVector3d gravity = new ChVector3d(ChWorldFrame.Vertical());
            gravity.Scale(-9.81);
            sys.SetGravitationalAcceleration(gravity);
            Console.WriteLine($"Gravity direction: {gravity.x}, {gravity.y}, {gravity.z}");

            // Solver settings
            sys.GetSolver().AsIterative().SetMaxIterations(150);
            sys.SetMaxPenetrationRecoverySpeed(4.0);

            // ------------------
            // Create the terrain
            // ------------------
            // For a crg terrain with arbitrary start heading the terrain class must be initialized before the vehicle class

            Console.WriteLine("CRG road file: " + crgRoadFile);

            CRGTerrain terrain = new CRGTerrain(sys);
            terrain.UseMeshVisualization(useMesh); // likely needs spacing apart of mesh/filter?
            terrain.SetContactFrictionCoefficient(0.8f);
            terrain.SetRoadsidePostDistance(25.0);
            // bright concrete
            terrain.SetRoadDiffuseTextureFile("vehicle/terrain/textures/Concrete002_2K-JPG/Concrete002_2K_Color.jpg");
            //terrain.SetRoadNormalTextureFile("vehicle/terrain/textures/Concrete002_2K-JPG/Concrete002_2K_NormalGL.jpg");
            //terrain.SetRoadRoughnessTextureFile("vehicle/terrain/textures/Concrete002_2K-JPG/Concrete002_2K_Roughness.jpg");
            terrain.Initialize(crgRoadFile);

            // Get the vehicle path (middle of the road)
            var path = terrain.GetRoadCenterLine();
            bool pathIsClosed = terrain.IsPathClosed();
            double roadLength = terrain.GetLength();
            double roadWidth = terrain.GetWidth();
            var initCsys = terrain.GetStartPosition();

            // Print values
            Console.WriteLine("Road length = " + roadLength);
            Console.WriteLine("Road width  = " + roadWidth);
            Console.WriteLine("Closed loop?  " + pathIsClosed);

            // Workaround for direct expression operators not correctly wrapped (i.e. * and -);
            ChVector3d halfUpVector = new ChVector3d(ChWorldFrame.Vertical());
            halfUpVector.Scale(0.05); // Scale the vector by half rather than multiplication operator *0.5;
            ChVector3d shapeLocation = new ChVector3d();
            shapeLocation.Sub(initCsys.pos, halfUpVector);
            terrain.GetGround().AddVisualShape(new ChVisualShapeBox(new ChBox(1, roadWidth, 0.1)),
                                    new ChFramed(shapeLocation, initCsys.rot));
            path.Write(outDir + "/path.txt");


            // ----------------------------
            // Get dynamic vehicle type
            // ----------------------------


            // Implement Vehicle choice - do not intialise until after terrain initialisation. CRG requirement.
            // TODO: move away from dynamic usage and specify top level statement instead
            dynamic CreateVehicleInstance(string vehicleType)
            {
                switch (vehicleType.ToUpper()) // ensure case sensitivity
                {
                    case "HMMWV":
                        return new HMMWV_Full(sys);
                    case "UAZBUS":
                        return new UAZBUS(sys);
                    case "KRAZ":
                        return new Kraz(sys);
                    case "SEDAN":
                        return new Sedan(sys);
                    case "MAN":
                        return new MAN_10t(sys);
                    case "GATOR":
                        return new Gator(sys);
                    case "CITYBUS":
                        return new CityBus(sys);
                    // ... ARTcar crashes and will need special treatment
                    default:
                        Console.Error.WriteLine("Unknown vehicle type. Returning HMMWV");
                        return new HMMWV_Full(sys);
                }
            }

            dynamic vehicle = CreateVehicleInstance(vehicleInput);
            // Error catch
            if (vehicle == null)
            {
                // Handle unknown or invalid input
                Console.WriteLine("Unknown vehicle type entered. Please restart and enter a valid type.");
            }

            // --------------------------
            // Create the chosen vehicle
            // --------------------------
            // Initial location and orientation from CRG terrain (create vehicle 0.5 m above road)
            // workaround to get initial location while direct operators not wrapped
            ChVector3d initLoc = new ChVector3d(ChWorldFrame.Vertical());
            initLoc.Scale(0.5); // raise vehicle 0.5m above the vertical.
            initCsys.pos.Add(initLoc, initCsys.pos);

            // determine which type of vehicle this is for selective vehicle parameters below
            string vehicleName = vehicle.GetType().Name;
            // common vehicle para's
            vehicle.SetContactMethod(ChContactMethod.SMC);
            vehicle.SetChassisFixed(false);
            vehicle.SetInitPosition(initCsys);
            if (vehicleName == "Kraz")
            {
                // Kraz is a special case, with visualisation overloads for the truck and trailer
                vehicle.SetTireStepSize(tireStepSize);
                vehicle.Initialize();
                vehicle.SetChassisVisualizationType(VisualizationType.MESH, VisualizationType.PRIMITIVES);
                vehicle.SetSteeringVisualizationType(VisualizationType.PRIMITIVES);
                vehicle.SetSuspensionVisualizationType(VisualizationType.PRIMITIVES, VisualizationType.PRIMITIVES);
                vehicle.SetWheelVisualizationType(VisualizationType.MESH, VisualizationType.MESH);
                vehicle.SetTireVisualizationType(VisualizationType.MESH, VisualizationType.MESH);
            }
            else
            {
                if (vehicleName == "HMMWV")
                { // special initialisation for the full hmmwv
                    vehicle.SetEngineType(EngineModelType.SHAFTS);
                    vehicle.SetTransmissionType(TransmissionModelType.AUTOMATIC_SHAFTS);
                    vehicle.SetDriveType(DrivelineTypeWV.RWD);
                }
                // Kraz doesn't have a call to Set tire type. Enable it for the others.
                vehicle.SetTireType(TireModelType.TMEASY); // TODO: fix enumerator to enable tire selection type in CLI prompt
                
                vehicle.SetTireStepSize(tireStepSize);
                vehicle.Initialize();

                vehicle.SetChassisVisualizationType(VisualizationType.MESH);
                vehicle.SetSuspensionVisualizationType(VisualizationType.PRIMITIVES);
                vehicle.SetSteeringVisualizationType(VisualizationType.PRIMITIVES);
                vehicle.SetWheelVisualizationType(VisualizationType.MESH);
                vehicle.SetTireVisualizationType(VisualizationType.MESH);
            }


            // --------------------
            // Create driver system
            // --------------------
            // Assuming all vehicle types inherit from a common base class or interface IMyVehicle
            ChWheeledVehicle getTheVehicle;

            // Determine and call the appropriate method based on vehicle type
            if (vehicleName == "Kraz")
            {
                getTheVehicle = vehicle.GetTractor();  // Specific to Kraz tractor/trailer setup
            }
            else
            {
                getTheVehicle = vehicle.GetVehicle();  // Common to other vehicle types
            }

            // Create the MyDriver instance
            MyDriver driver = new MyDriver(driverType, getTheVehicle, path, roadWidth);
            driver.Initialize();
            Console.WriteLine("Driver model: " + driver.GetDriverType());

            // -------------------------------
            // Create the visualization system
            // -------------------------------

            ChWheeledVehicleVisualSystemIrrlicht vis = new ChWheeledVehicleVisualSystemIrrlicht();
            vis.SetHUDLocation(500, 20);
            vis.SetWindowTitle("OpenCRG Steering");
            vis.SetChaseCamera(new ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5);
            vis.Initialize();
            vis.AddSkyBox();
            vis.AddLogo();
            vis.AddLightDirectional();
            vis.AttachVehicle(getTheVehicle);

            // Set up sentinal
            var sentinel = new ChVisualShapeSphere(0.1);
            var target = new ChVisualShapeSphere(0.1);
            sentinel.SetColor(new ChColor(1, 0, 0));
            target.SetColor(new ChColor(0, 1, 0));
            int sentinelID = vis.AddVisualModel(sentinel, new ChFramed());
            int targetID = vis.AddVisualModel(target, new ChFramed());

            // ---------------
            // Simulation loop
            // ---------------

            // Number of simulation steps between image outputs
            double renderStepSize = 1 / fps;
            // Number of simulation steps between two 3D view render frames
            int renderSteps = (int)Math.Ceiling(renderStepSize / stepSize);
            int renderFrame = 0;
            // Initialize simulation frame counter
            int stepNumber = 0;
            // Initialize frame counters
            int simFrame = 0;

            while (vis.Run())
            {
                double time = vehicle.GetSystem().GetChTime();

                // Driver inputs
                DriverInputs driverInputs = driver.GetInputs();

                // Update sentinel and target location markers for the path-follower controller.
                vis.UpdateVisualModel(sentinelID, new ChFramed(driver.GetSentinelLocation()));
                vis.UpdateVisualModel(targetID, new ChFramed(driver.GetTargetLocation()));

                // Render scene and output images
                vis.BeginScene();
                vis.Render();

                // Draw the world reference frame at the sentinel location
                vis.RenderFrame(new ChFramed(driver.GetSentinelLocation()));

                // Render scene and output images
                if (outputImages && stepNumber % renderSteps == 0)
                {
                    string fileName = $"{outDir}/Image_{renderFrame + 1:D3}.bmp";
                    vis.WriteImageToFile(fileName);
                    renderFrame++;
                }
                
                // Update modules (process inputs from other modules)
                driver.Synchronize(time);
                terrain.Synchronize(time);
                vehicle.Synchronize(time, driverInputs, terrain);
                vis.Synchronize(time, driverInputs);

                // Advance simulation for one timestep for all modules
                driver.Advance(stepSize);
                terrain.Advance(stepSize);
                vehicle.Advance(stepSize);
                vis.Advance(stepSize);
                sys.DoStepDynamics(stepSize);

                // Increment simulation frame number
                simFrame++;

                vis.EndScene();
            }

            driver.PrintStats();

            // END OF PROGRAM
        }
    }
}