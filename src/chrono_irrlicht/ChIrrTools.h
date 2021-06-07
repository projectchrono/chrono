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

#ifndef CHIRRTOOLS_H
#define CHIRRTOOLS_H

#include <vector>
#include <irrlicht.h>

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChVector.h"
#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_irrlicht/ChApiIrr.h"

namespace irr {
namespace core {

/// Utility class to easily convert a Chrono vector into an Irrlicht vector3df.
class ChApiIrr vector3dfCH : public vector3df {
  public:
    vector3dfCH(const chrono::ChVector<>& mch) {
        X = ((f32)mch.x());
        Y = ((f32)mch.y());
        Z = ((f32)mch.z());
    }
};

}  // end namespace core
}  // end namespace irr

// -----------------------------------------------------------------------------

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

enum class IrrContactsDrawMode {
    CONTACT_NORMALS = 0,  // draw normals
    CONTACT_DISTANCES,    // draw lines connecting two contact points
    CONTACT_FORCES_N,     // draw contact forces (only normal component) applied to 2nd body
    CONTACT_FORCES,       // draw contact forces applied to 2nd body
    CONTACT_NONE          // draw nothing
};

enum class IrrContactsLabelMode {
    CONTACT_DISTANCES_VAL,  // print distance
    CONTACT_FORCES_VAL,     // print contact forces modulus
    CONTACT_FORCES_N_VAL,   // print contact forces (only normal component)
    CONTACT_FORCES_T_VAL,   // print contact forces (only tangential component)
    CONTACT_TORQUES_VAL,    // print contact torques modulus
    CONTACT_TORQUES_S_VAL,  // print contact torques modulus (only spinning component)
    CONTACT_TORQUES_R_VAL,  // print contact torques modulus (only rolling component)
    CONTACT_NONE_VAL        // print nothing
};

enum class IrrLinkDrawMode {
    LINK_REACT_FORCE = 0,  // draw reaction force
    LINK_REACT_TORQUE,     // draw reaction torque
    LINK_NONE              // draw nothing
};

enum class IrrLinkLabelMode {
    LINK_REACT_FORCE_VAL = 0,  // print reaction force modulus
    LINK_REACT_FORCE_X,        // print reaction force x
    LINK_REACT_FORCE_Y,        // print reaction force y
    LINK_REACT_FORCE_Z,        // print reaction force z
    LINK_REACT_TORQUE_VAL,     // print reaction torque modulus
    LINK_REACT_TORQUE_X,       // print reaction torque x
    LINK_REACT_TORQUE_Y,       // print reaction torque y
    LINK_REACT_TORQUE_Z,       // print reaction torque z
    LINK_NONE_VAL              // draw nothing
};

namespace tools {

/// Function to align an Irrlicht object to a Chrono coordsys.
ChApiIrr void alignIrrlichtNodeToChronoCsys(irr::scene::ISceneNode* mnode, const ChCoordsys<>& mcoords);

/// Easy-to-use function which draws contact points used by a ChSystem in the
/// current Irrlicht viewer (the IVideoDriver). The contact points are
/// visually represented with short lines, of length mlen, aligned to contact
/// normals.
ChApiIrr int drawAllContactPoints(std::shared_ptr<ChContactContainer> mcontactcontainer,
                                  irr::video::IVideoDriver* driver,
                                  double mlen = 1.0,
                                  IrrContactsDrawMode drawtype = IrrContactsDrawMode::CONTACT_NORMALS);

/// Easy-to-use function which draws contact informations as labels at the
/// contact point
ChApiIrr int drawAllContactLabels(std::shared_ptr<ChContactContainer> mcontactcontainer,
                                  irr::IrrlichtDevice* device,
                                  IrrContactsLabelMode labeltype = IrrContactsLabelMode::CONTACT_FORCES_N_VAL,
                                  irr::video::SColor mcol = irr::video::SColor(255, 255, 255, 255));

/// Easy-to-use function which draws reaction forces in all contacts in
/// current Irrlicht viewer (the IVideoDriver).
ChApiIrr int drawAllLinks(ChSystem& mphysicalSystem,
                          irr::video::IVideoDriver* driver,
                          double mlen = 1.0,
                          IrrLinkDrawMode drawtype = IrrLinkDrawMode::LINK_REACT_FORCE);

/// Easy-to-use function which draws contact informations as labels at the
/// contact point
ChApiIrr int drawAllLinkLabels(ChSystem& mphysicalSystem,
                               irr::IrrlichtDevice* device,
                               IrrLinkLabelMode labeltype = IrrLinkLabelMode::LINK_REACT_FORCE_X,
                               irr::video::SColor mcol = irr::video::SColor(255, 255, 255, 255));

/// Easy-to-use function which draws collision objects bounding boxes for
/// rigid bodies - if they have a collision shape.
ChApiIrr int drawAllBoundingBoxes(ChSystem& mphysicalSystem, irr::video::IVideoDriver* driver);

/// Easy-to-use function which draws coordinate systems of ChBody objects.
ChApiIrr int drawAllCOGs(ChSystem& mphysicalSystem, irr::video::IVideoDriver* driver, double scale = 0.01);

/// Easy-to-use function which draws coordinate systems of link frames.
ChApiIrr int drawAllLinkframes(ChSystem& mphysicalSystem, irr::video::IVideoDriver* driver, double scale = 0.01);

/// --
ChApiIrr void drawHUDviolation(irr::video::IVideoDriver* driver,
                               irr::IrrlichtDevice* mdevice,
                               ChSystem& asystem,
                               int mx = 10,
                               int my = 290,
                               int sx = 300,
                               int sy = 100,
                               double spfact = 100.0);

/// --
ChApiIrr void drawChFunction(irr::IrrlichtDevice* mdevice,
                             ChFunction* fx,
                             double xmin = 0,
                             double xmax = 1,
                             double ymin = -1,
                             double ymax = 1,
                             int mx = 10,
                             int my = 290,
                             int sx = 300,
                             int sy = 100);

/// Easy-to-use function to draw segment lines in 3D space, with given color.
ChApiIrr void drawSegment(irr::video::IVideoDriver* driver,
                          ChVector<> mstart,
                          ChVector<> mend,
                          irr::video::SColor mcol = irr::video::SColor(255, 0, 0, 0),
                          bool use_Zbuffer = false);

/// Easy-to-use function to draw a polyline in 3D space, given the array of
/// points as a std::vector.
ChApiIrr void drawPolyline(irr::video::IVideoDriver* driver,
                           std::vector<ChVector<> >& mpoints,
                           irr::video::SColor mcol = irr::video::SColor(255, 0, 0, 0),
                           bool use_Zbuffer = false);

/// Easy-to-use function to draw a circle line in 3D space, with given color.
/// Specify the center as coordsys position. Orientation as coordsys
/// quaternion (default in xy plane)
ChApiIrr void drawCircle(irr::video::IVideoDriver* driver,
                         double radius,
                         ChCoordsys<> mpos = CSYSNORM,
                         irr::video::SColor mcol = irr::video::SColor(255, 0, 0, 0),
                         int mresolution = 36,
                         bool use_Zbuffer = false);

/// Easy-to-use function to draw a spring in 3D space, with given color.
/// Specify the radius, the end points in absolute space, the resolution (i.e.
/// the number of segments approximating the helix) and the number of turns.
ChApiIrr void drawSpring(irr::video::IVideoDriver* driver,
                         double radius,
                         ChVector<> start,
                         ChVector<> end,
                         irr::video::SColor mcol = irr::video::SColor(255, 0, 0, 0),
                         int mresolution = 65,
                         double turns = 5,
                         bool use_Zbuffer = false);

/// Easy-to-use function to draw grids in 3D space, with given orientation,
/// color and spacing.
ChApiIrr void drawGrid(irr::video::IVideoDriver* driver,
                       double ustep = 0.1,
                       double vstep = 0.1,
                       int nu = 20,
                       int nv = 20,
                       ChCoordsys<> mpos = CSYSNORM,
                       irr::video::SColor mcol = irr::video::SColor(50, 80, 110, 110),
                       bool use_Zbuffer = false);

/// Easy-to-use function to draw color bar with a color map and 2D legend
ChApiIrr void drawColorbar(double vmin,
                           double vmax,
                           const std::string& label,
                           irr::IrrlichtDevice* mdevice,
                           int mx = 740,
                           int my = 20,
                           int sx = 30,
                           int sy = 300);

/// Draw the collision shapes as wireframe, overlayed to shapes.
/// Note: this works only for the Bullet collision system (i.e. not working for Chrono::Multicore)
ChApiIrr void drawCollisionShapes(ChSystem& asystem,
                                  irr::IrrlichtDevice* mdevice,
                                  irr::video::SColor mcol = irr::video::SColor(50, 0, 0, 110));

/// --
ChApiIrr void drawPlot3D(irr::video::IVideoDriver* driver,
                         ChMatrixConstRef X,  // x of points, in local csys x
                         ChMatrixConstRef Y,  // y of points, in local csys y
                         ChMatrixConstRef Z,  // z height map of points, in local csys z
                         ChCoordsys<> mpos = CSYSNORM,
                         irr::video::SColor mcol = irr::video::SColor(50, 80, 110, 110),
                         bool use_Zbuffer = false);

/// Draw run-time profiler infos
ChApiIrr void drawProfiler(irr::IrrlichtDevice* device);

}  // end namespace tools

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
