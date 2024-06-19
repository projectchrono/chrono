// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Test to check that when loading a URDF file, the relative filenames are
// resolved correctly when ROS is available.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono_parsers/ChParserURDF.h"
#include "chrono_thirdparty/filesystem/path.h"

#ifdef HAVE_ROS
    #include "ament_index_cpp/get_package_prefix.hpp"
    #include "ament_index_cpp/get_package_share_directory.hpp"
#endif

using namespace chrono;
using namespace chrono::parsers;

TEST(ChParserURDF, URDF_package) {
    ChSystemNSC system;

    // Create the URDF parser with a file that contains a filename with package://
    const std::string filename = "robot/r2d2/r2d2-package.urdf";
    ChParserURDF parser(GetChronoDataFile(filename));
    parser.PopulateSystem(system);

    // Check the right_tip body. It should have 1 visual model and we'll verify that the filename is resolved correctly
    auto right_tip = parser.GetChBody("right_tip");
    EXPECT_TRUE(right_tip != nullptr);
    EXPECT_TRUE(right_tip->GetVisualModel()->GetNumShapes() == 1);

    // Grab the visual shape, which should be a ChVisualShapeModelFile, and it's filename
    auto right_tip_vis_shape = right_tip->GetVisualShape(0);
    EXPECT_TRUE(std::dynamic_pointer_cast<ChVisualShapeModelFile>(right_tip_vis_shape) != nullptr);
    auto right_tip_vis_filename = std::dynamic_pointer_cast<ChVisualShapeModelFile>(right_tip_vis_shape)->GetFilename();

#ifdef HAVE_ROS
    try {
        // Checks if the urdf_tutorial package exists. Will throw an exception if it does not.
        ament_index_cpp::get_package_share_directory("urdf_tutorial");

        // If ROS is available, and the urdf_tutorial package exists, the filename should be resolved. It's going to be
        // absolute, so check that it ends with the expected filename and exists
        EXPECT_TRUE(right_tip_vis_filename.find("meshes/l_finger_tip.dae") != std::string::npos);
        EXPECT_TRUE(filesystem::path(right_tip_vis_filename).exists());
    } catch (const ament_index_cpp::PackageNotFoundError& e) {
        // If the urdf_tutorial package does not exist, the filename should not be resolved
        EXPECT_TRUE(right_tip_vis_filename == "package://urdf_tutorial/meshes/l_finger_tip.dae");
    }
#else
    // If ROS is not available, the filename should not be resolved
    EXPECT_TRUE(right_tip_vis_filename == "package://urdf_tutorial/meshes/l_finger_tip.dae");
#endif
}
