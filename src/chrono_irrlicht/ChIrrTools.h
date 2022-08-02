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
#include "chrono/core/ChFrame.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_irrlicht/ChApiIrr.h"

namespace irr {
namespace core {

/// Utility class to convert a Chrono vector into an Irrlicht vector3df.
class ChApiIrr vector3dfCH : public vector3df {
  public:
    vector3dfCH(const chrono::ChVector<>& mch);
};

/// Utility class to convert a Chrono frame into an Irrlicht transform.
class ChApiIrr matrix4CH : public matrix4 {
  public:
    matrix4CH(const chrono::ChCoordsys<>& csys);
    matrix4CH(const chrono::ChFrame<>& frame);
};

}  // end namespace core
}  // end namespace irr

// -----------------------------------------------------------------------------

namespace chrono {
namespace irrlicht {

// Forward declaration
class ChVisualSystemIrrlicht;

/// @addtogroup irrlicht_module
/// @{

enum class ContactsDrawMode {
    CONTACT_NORMALS = 0,  // draw normals
    CONTACT_DISTANCES,    // draw lines connecting two contact points
    CONTACT_FORCES_N,     // draw contact forces (only normal component) applied to 2nd body
    CONTACT_FORCES,       // draw contact forces applied to 2nd body
    CONTACT_NONE          // draw nothing
};

enum class ContactsLabelMode {
    CONTACT_DISTANCES_VAL,  // print distance
    CONTACT_FORCES_VAL,     // print contact forces modulus
    CONTACT_FORCES_N_VAL,   // print contact forces (only normal component)
    CONTACT_FORCES_T_VAL,   // print contact forces (only tangential component)
    CONTACT_TORQUES_VAL,    // print contact torques modulus
    CONTACT_TORQUES_S_VAL,  // print contact torques modulus (only spinning component)
    CONTACT_TORQUES_R_VAL,  // print contact torques modulus (only rolling component)
    CONTACT_NONE_VAL        // print nothing
};

enum class LinkDrawMode {
    LINK_REACT_FORCE = 0,  // draw reaction force
    LINK_REACT_TORQUE,     // draw reaction torque
    LINK_NONE              // draw nothing
};

enum class LinkLabelMode {
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

/// Convert a ChColor to an Irrlicht SColorf.
ChApiIrr irr::video::SColorf ToIrrlichtSColorf(const ChColor& col);

/// Convert an RGB set and an opacity value to an Irrlicht SColor.
ChApiIrr irr::video::SColor ToIrrlichtSColor(const ChColor& col, float alpha = 1.0);

/// Convert a ChVisualMaterial to an Irrlicht material.
ChApiIrr irr::video::SMaterial ToIrrlichtMaterial(std::shared_ptr<ChVisualMaterial> mat,
                                                  irr::video::IVideoDriver* driver);

/// Align an Irrlicht object to a the specified coordinate system.
ChApiIrr void alignIrrlichtNode(irr::scene::ISceneNode* mnode, const ChCoordsys<>& mcoords);

/// Draw contact points used by a ChSystem in the current Irrlicht viewer.
/// The contact points are visually represented with short lines, of length mlen, aligned to contact normals.
ChApiIrr int drawAllContactPoints(ChVisualSystemIrrlicht* vis,
                                  double mlen = 1.0,
                                  ContactsDrawMode drawtype = ContactsDrawMode::CONTACT_NORMALS);

/// Draw contact informations as labels at the contact point.
ChApiIrr int drawAllContactLabels(ChVisualSystemIrrlicht* vis,
                                  ContactsLabelMode labeltype = ContactsLabelMode::CONTACT_FORCES_N_VAL,
                                  ChColor col = ChColor(1, 1, 1));

/// Draw reaction forces in all contacts in current Irrlicht viewer.
ChApiIrr int drawAllLinks(ChVisualSystemIrrlicht* vis,
                          double mlen = 1.0,
                          LinkDrawMode drawtype = LinkDrawMode::LINK_REACT_FORCE);

/// Draw contact informations as labels at the contact point.
ChApiIrr int drawAllLinkLabels(ChVisualSystemIrrlicht* vis,
                               LinkLabelMode labeltype = LinkLabelMode::LINK_REACT_FORCE_X,
                               ChColor col = ChColor(1, 1, 1));

/// Draw collision objects bounding boxes for rigid bodies (if they have a collision shape).
ChApiIrr int drawAllBoundingBoxes(ChVisualSystemIrrlicht* vis);

/// Draw coordinate systems of ChBody objects.
ChApiIrr int drawAllCOGs(ChVisualSystemIrrlicht* vis, double scale = 0.01);

/// Draw coordinate systems of link frames.
ChApiIrr int drawAllLinkframes(ChVisualSystemIrrlicht* vis, double scale = 0.01);

ChApiIrr void drawHUDviolation(ChVisualSystemIrrlicht* vis,
                               int mx = 10,
                               int my = 290,
                               int sx = 300,
                               int sy = 100,
                               double spfact = 100.0);

ChApiIrr void drawChFunction(ChVisualSystemIrrlicht* vis,
                             std::shared_ptr<ChFunction> fx,
                             double xmin = 0,
                             double xmax = 1,
                             double ymin = -1,
                             double ymax = 1,
                             int mx = 10,
                             int my = 290,
                             int sx = 300,
                             int sy = 100,
                             ChColor col = ChColor(1, 0, 0),
                             const char* title = 0);

/// Draw line segments in 3D space with given color.
ChApiIrr void drawSegment(ChVisualSystemIrrlicht* vis,
                          ChVector<> start,
                          ChVector<> end,
                          ChColor col = ChColor(1, 1, 1),
                          bool use_Zbuffer = false);

/// Draw a polyline in 3D space, given the array of points.
ChApiIrr void drawPolyline(ChVisualSystemIrrlicht* vis,
                           std::vector<ChVector<> >& points,
                           ChColor col = ChColor(1, 1, 1),
                           bool use_Zbuffer = false);

/// Draw a circle line in 3D space with given color.
/// The circle is centered in the X-Y plane of the provided coordinate system.
ChApiIrr void drawCircle(ChVisualSystemIrrlicht* vis,
                         double radius,
                         ChCoordsys<> pos = CSYSNORM,
                         ChColor col = ChColor(1, 1, 1),
                         int resolution = 36,
                         bool use_Zbuffer = false);

/// Draw a spring in 3D space with given color.
/// Specify the radius, the end points in absolute space, the resolution (i.e. the number of segments approximating the
/// helix) and the number of turns.
ChApiIrr void drawSpring(ChVisualSystemIrrlicht* vis,
                         double radius,
                         ChVector<> start,
                         ChVector<> end,
                         ChColor col = ChColor(1, 1, 1),
                         int resolution = 65,
                         double turns = 5,
                         bool use_Zbuffer = false);

ChApiIrr void drawRotSpring(ChVisualSystemIrrlicht* vis,
                            ChCoordsys<> pos,
                            double radius,
                            double start_angle,
                            double end_angle,
                            ChColor col = ChColor(1, 1, 1),
                            int resolution = 65,
                            bool use_Zbuffer = false);

/// Draw grids in 3D space with given orientation, color, and spacing.
ChApiIrr void drawGrid(ChVisualSystemIrrlicht* vis,
                       double ustep = 0.1,
                       double vstep = 0.1,
                       int nu = 20,
                       int nv = 20,
                       ChCoordsys<> pos = CSYSNORM,
                       ChColor col = ChColor(0.7f, 0.7f, 0.7f),
                       bool use_Zbuffer = false);

/// Draw color bar with a color map and 2D legend.
ChApiIrr void drawColorbar(ChVisualSystemIrrlicht* vis,
                           double vmin,
                           double vmax,
                           const std::string& label,
                           int mx = 740,
                           int my = 20,
                           int sx = 30,
                           int sy = 300);

ChApiIrr void drawPlot3D(ChVisualSystemIrrlicht* vis,
                         ChMatrixConstRef X,  // x of points, in local csys x
                         ChMatrixConstRef Y,  // y of points, in local csys y
                         ChMatrixConstRef Z,  // z height map of points, in local csys z
                         ChCoordsys<> pos = CSYSNORM,
                         ChColor col = ChColor(1, 1, 1),
                         bool use_Zbuffer = false);

/// Render run-time profiler info.
ChApiIrr void drawProfiler(ChVisualSystemIrrlicht* vis);

/// Draw RGB coordinate system.
ChApiIrr void drawCoordsys(ChVisualSystemIrrlicht* vis, const ChCoordsys<>& coord = CSYSNORM, double scale = 1);

}  // end namespace tools

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
