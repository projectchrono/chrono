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

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChColormap.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChFrame.h"
#include "chrono/functions/ChFunctionBase.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_irrlicht/ChApiIrr.h"

/// Main Irrlicht namespace  (Chrono extensions).
namespace irr {

/// Irrlicht base classes (Chrono extensions).
namespace core {

/// Utility class to convert a Chrono vector into an Irrlicht vector3df.
class ChApiIrr vector3dfCH : public vector3df {
  public:
    vector3dfCH(const chrono::ChVector3d& mch);
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
                                  ChColor col = ChColor(1.f, 1.f, 1.f));

/// Draw reaction forces in all contacts in current Irrlicht viewer.
ChApiIrr int drawAllLinks(ChVisualSystemIrrlicht* vis,
                          double mlen = 1.0,
                          LinkDrawMode drawtype = LinkDrawMode::LINK_REACT_FORCE);

/// Draw contact informations as labels at the contact point.
ChApiIrr int drawAllLinkLabels(ChVisualSystemIrrlicht* vis,
                               LinkLabelMode labeltype = LinkLabelMode::LINK_REACT_FORCE_X,
                               ChColor col = ChColor(1.f, 1.f, 1.f));

/// Draw collision objects bounding boxes for rigid bodies (if they have a collision shape).
ChApiIrr int drawAllBoundingBoxes(ChVisualSystemIrrlicht* vis);

/// Draw coordinate systems of ChBody objects.
ChApiIrr int drawAllCOGs(ChVisualSystemIrrlicht* vis, double scale = 0.01);

/// Draw coordinate systems of link frames.
ChApiIrr int drawAllLinkframes(ChVisualSystemIrrlicht* vis, double scale = 0.01);

/// Draw the plot of solver violations.
/// Each vertical red bar of the plot represents the residual during the solver iterations.
/// The rightmost red bar represents the residual after the latest iteration.
/// The red horizontal line respresents the tolerance requested to the solver.
/// If the last red bar does not fall below the red line, the solver did not converge.
/// The Y axis is logarithmic for the error (residual) and ranges from log10(tol)-1 to log10(tol)+2.
/// If the tolerance is set to 0 the graphics will consider a default tolerance of 1e-6.
/// It is then recommended to increase the number of iterations, reduce the timestep or consider relaxing the tolerance.
/// Yellow bars refer to the variation of Lagrange multipliers during the iterations (VI solvers only).
/// For them, the Y axis ranges from zero to the maximum variation observed among all the iterations of the last step
/// (linear scale).
ChApiIrr void drawHUDviolation(ChVisualSystemIrrlicht* vis,
                               int pos_x = 10,
                               int pos_y = 290,
                               int width = 300,
                               int height = 100);

/// Draw function value.
/// Plot widget coordinates are considered from top-left corner of the Irrlicht window.
ChApiIrr void drawChFunction(ChVisualSystemIrrlicht* vis,           ///< visual system
                             std::shared_ptr<ChFunction> fx,        ///< function to draw
                             double xmin = 0,                       ///< minimum X of the plot
                             double xmax = 1,                       ///< maximum X of the plot
                             double ymin = -1,                      ///< minimum Y of the plot
                             double ymax = 1,                       ///< maximum Y of the plot
                             int pos_x = 10,                        ///< top-left corner X position of the plot window
                             int pos_y = 290,                       ///< top-left corner Y position of the plot window
                             int width = 300,                       ///< width of the plot window
                             int height = 100,                      ///< height of the plot window
                             ChColor col = ChColor(1.f, 0.f, 0.f),  ///< color of the plot line
                             std::string title = ""                 ///< title of the plot
);

/// Draw line segments in 3D space with given color.
ChApiIrr void drawSegment(ChVisualSystemIrrlicht* vis,
                          ChVector3d start,
                          ChVector3d end,
                          ChColor col = ChColor(1.f, 1.f, 1.f),
                          bool use_Zbuffer = false);

/// Draw a polyline in 3D space, given the array of points.
ChApiIrr void drawPolyline(ChVisualSystemIrrlicht* vis,
                           std::vector<ChVector3d>& points,
                           ChColor col = ChColor(1.f, 1.f, 1.f),
                           bool use_Zbuffer = false);

/// Draw a circle line in 3D space with given color.
/// The circle is centered in the X-Y plane of the provided coordinate system.
ChApiIrr void drawCircle(ChVisualSystemIrrlicht* vis,
                         double radius,
                         ChCoordsys<> pos = CSYSNORM,
                         ChColor col = ChColor(1.f, 1.f, 1.f),
                         int resolution = 36,
                         bool use_Zbuffer = false);

/// Draw a spring in 3D space with given color.
/// Specify the radius, the end points in absolute space, the resolution (i.e. the number of segments approximating the
/// helix) and the number of turns.
ChApiIrr void drawSpring(ChVisualSystemIrrlicht* vis,
                         double radius,
                         ChVector3d start,
                         ChVector3d end,
                         ChColor col = ChColor(1.f, 1.f, 1.f),
                         int resolution = 65,
                         double turns = 5,
                         bool use_Zbuffer = false);

ChApiIrr void drawRotSpring(ChVisualSystemIrrlicht* vis,
                            ChCoordsys<> pos,
                            double radius,
                            double start_angle,
                            double end_angle,
                            ChColor col = ChColor(1.f, 1.f, 1.f),
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
/// Plot widget coordinates are considered from top-left corner of the Irrlicht window.
ChApiIrr void drawColorbar(ChVisualSystemIrrlicht* vis,  ///< visual system
                           const ChColormap& colormap,   ///< current colormap
                           double value_min,             ///< minimum value of the color map
                           double value_max,             ///< maximum value of the color map
                           const std::string& label,     ///< label of the color bar
                           int pos_x = 740,              ///< top-left corner X position of the color bar
                           int pos_y = 20,               ///< top-left corner Y position of the color bar
                           int width = 30,               ///< width of the color bar
                           int height = 300              ///< height of the color bar
);

ChApiIrr void drawPlot3D(ChVisualSystemIrrlicht* vis,
                         ChMatrixConstRef X,  // x of points, in local csys x
                         ChMatrixConstRef Y,  // y of points, in local csys y
                         ChMatrixConstRef Z,  // z height map of points, in local csys z
                         ChCoordsys<> pos = CSYSNORM,
                         ChColor col = ChColor(1.f, 1.f, 1.f),
                         bool use_Zbuffer = false);

/// Render run-time profiler info.
ChApiIrr void drawProfiler(ChVisualSystemIrrlicht* vis);

/// Draw RGB coordinate system.
ChApiIrr void drawCoordsys(ChVisualSystemIrrlicht* vis,
                           const ChCoordsys<>& coord = CSYSNORM,
                           double scale = 1,
                           bool use_Zbuffer = false);

/// Draw a line arrow in 3D space with given color.
ChApiIrr void drawArrow(ChVisualSystemIrrlicht* vis,                  ///< visual system
                        const ChVector3d& start,                      ///< arrow start point
                        const ChVector3d& end,                        ///< arrow end point
                        const ChVector3d& plane_normal = VECT_Y,      ///< normal to plane containing arrow segments
                        bool sharp = false,                           ///< set arrow shape as 'sharp' or 'wide'
                        const ChColor& col = ChColor(1.f, 1.f, 1.f),  ///< color
                        bool use_Zbuffer = false                      ///< use Z buffer
);

/// Draw a label in 3D scene at given position.
ChApiIrr void drawLabel3D(ChVisualSystemIrrlicht* vis,
                        const std::string& text,
                        const ChVector3d& position,
                        const ChColor& color = ChColor(0.f, 0.f, 0.f),
                        bool use_Zbuffer = false);

}  // end namespace tools

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
