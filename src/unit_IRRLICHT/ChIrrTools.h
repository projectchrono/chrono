//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHIRRTOOLS_H
#define CHIRRTOOLS_H


#include <vector>
#include <irrlicht.h>

#include "core/ChVector.h"
#include "core/ChMatrix.h"
#include "core/ChCoordsys.h"
#include "motion_functions/ChFunction_Base.h"
#include "physics/ChSystem.h"

#include "unit_IRRLICHT/ChApiIrr.h"


namespace irr {
namespace core {

// -----------------------------------------------------------------------------
/// Utility class to easily convert a Chrono::Engine vector into an Irrlicht
/// vector3df. Simply create an Irrlicht compatible vector as:
///          myvector=vector3dfCH(mychronovector);
// -----------------------------------------------------------------------------

class ChApiIrr vector3dfCH : public vector3df
{
public:
  vector3dfCH(const chrono::ChVector<>& mch) { X=((f32)mch.x);  Y=((f32)mch.y);  Z=((f32)mch.z);}
  vector3dfCH(chrono::ChVector<>* mch)       { X=((f32)mch->x); Y=((f32)mch->y); Z=((f32)mch->z);}
};

} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


namespace irr
{

// -----------------------------------------------------------------------------
/// Class with static functions which help with the integration of
/// Chrono::Engine and Irrlicht 3D rendering library.
// -----------------------------------------------------------------------------

class ChApiIrr ChIrrTools
{
public:

  enum eCh_ContactsDrawMode {
    CONTACT_NORMALS = 0,    // draw normals
    CONTACT_DISTANCES,      // draw lines connecting two contact points
    CONTACT_FORCES_N,       // draw contact forces (only normal component) applied to 2nd body
    CONTACT_FORCES,         // draw contact forces applied to 2nd body
    CONTACT_NONE            // draw nothing
  };

  enum eCh_ContactsLabelMode{
    CONTACT_DISTANCES_VAL,  // print distance
    CONTACT_FORCES_VAL,     // print contact forces modulus
    CONTACT_FORCES_N_VAL,   // print contact forces (only normal component)
    CONTACT_FORCES_T_VAL,   // print contact forces (only tangential component)
    CONTACT_TORQUES_VAL,    // print contact torques modulus
    CONTACT_TORQUES_S_VAL,  // print contact torques modulus (only spinning component)
    CONTACT_TORQUES_R_VAL,  // print contact torques modulus (only rolling component)
    CONTACT_NONE_VAL        // print nothing
  };

  /// Function to align an Irrlicht object to a Chrono::Engine coordsys.
  static void alignIrrlichtNodeToChronoCsys(scene::ISceneNode*          mnode,
                                            const chrono::ChCoordsys<>& mcoords);

  /// Easy-to-use function which draws contact points used by a ChSystem in the
  /// current Irrlicht viewer (the IVideoDriver). The contact points are
  /// visually represented with short lines, of length mlen, aligned to contact
  /// normals.
  static int drawAllContactPoints(chrono::ChSystem&     mphysicalSystem,
                                  video::IVideoDriver*  driver,
                                  double                mlen = 1.0,
                                  eCh_ContactsDrawMode  drawtype = CONTACT_NORMALS);

  /// Easy-to-use function which draws contact informations as labels at the
  /// contact point
  static int drawAllContactLabels(chrono::ChSystem&     mphysicalSystem,
                                  irr::IrrlichtDevice*  device,
                                  eCh_ContactsLabelMode labeltype = CONTACT_FORCES_N_VAL,
                                  video::SColor         mcol = video::SColor(255,255,255,255) );

  /// Easy-to-use function which draws collision objects bounding boxes for
  /// rigid bodies - if they have a collision shape.
  static int drawAllBoundingBoxes(chrono::ChSystem&     mphysicalSystem,
                                  video::IVideoDriver*  driver);

  /// Easy-to-use function which draws coordinate systems of ChBody objects.
  static int drawAllCOGs(chrono::ChSystem&    mphysicalSystem,
                         video::IVideoDriver* driver,
                         double               scale = 0.01);

  /// Easy-to-use function which draws coordinate systems of link frames.
  static int drawAllLinkframes(chrono::ChSystem&    mphysicalSystem,
                               video::IVideoDriver* driver,
                               double               scale = 0.01);

  /// --
  static void drawHUDviolation(video::IVideoDriver* driver,
                               IrrlichtDevice*      mdevice,
                               chrono::ChSystem&    asystem,
                               int                  mx = 10,
                               int                  my = 290,
                               int                  sx = 300,
                               int                  sy = 100,
                               double               spfact = 100.0,
                               double               posfact = 500.0);

  /// --
  static void drawChFunction(IrrlichtDevice*      mdevice,
                             chrono::ChFunction*  fx,
                             double               xmin = 0,
                             double               xmax = 1,
                             double               ymin = -1,
                             double               ymax = 1,
                             int                  mx = 10,
                             int                  my = 290,
                             int                  sx = 300,
                             int                  sy = 100);

  /// Easy-to-use function to draw segment lines in 3D space, with given color.
  static void drawSegment(video::IVideoDriver*  driver,
                          chrono::ChVector<>    mstart,
                          chrono::ChVector<>    mend,
                          video::SColor         mcol = video::SColor(255,0,0,0),
                          bool                  use_Zbuffer = false);

  /// Easy-to-use function to draw a polyline in 3D space, given the array of
  /// points as a std::vector.
  static void drawPolyline(video::IVideoDriver*               driver,
                           std::vector< chrono::ChVector<> >  mpoints,
                           video::SColor                      mcol = video::SColor(255,0,0,0),
                           bool                               use_Zbuffer = false);

  /// Easy-to-use function to draw a circle line in 3D space, with given color.
  /// Specify the center as coordsys position. Orientation as coordsys
  /// quaternion (default in xy plane)
  static void drawCircle(video::IVideoDriver* driver,
                         double               radius,
                         chrono::ChCoordsys<> mpos = chrono::CSYSNORM,
                         video::SColor        mcol = video::SColor(255,0,0,0),
                         int                  mresolution = 36,
                         bool                 use_Zbuffer = false);

  /// Easy-to-use function to draw a spring in 3D space, with given color.
  /// Specify the radius, the end points in absolute space, the resolution (i.e.
  /// the number of segments approximating the helix) and the number of turns. 
  static void drawSpring(video::IVideoDriver* driver,
                         double               radius,
                         chrono::ChVector<>   start,
                         chrono::ChVector<>   end,
                         video::SColor        mcol = video::SColor(255,0,0,0),
                         int                  mresolution = 65,
                         double               turns = 5,
                         bool                 use_Zbuffer = false);

  /// Easy-to-use function to draw grids in 3D space, with given orientation,
  /// color and spacing.
  static void drawGrid(video::IVideoDriver* driver,
                       double               ustep = 0.1,
                       double               vstep = 0.1,
                       int                  nu = 20,
                       int                  nv = 20,
                       chrono::ChCoordsys<> mpos = chrono::CSYSNORM,
                       video::SColor        mcol = video::SColor(50,80,110,110),
                       bool                 use_Zbuffer = false);

  /// --
  static void drawPlot3D(video::IVideoDriver* driver,
                         chrono::ChMatrix<>   X,        // x of points, in local csys x
                         chrono::ChMatrix<>   Y,        // y of points, in local csys y
                         chrono::ChMatrix<>   Z,        // z height map of points, in local csys z
                         chrono::ChCoordsys<> mpos = chrono::CSYSNORM,
                         video::SColor        mcol = video::SColor(50,80,110,110),
                         bool                 use_Zbuffer = false);

};


} // END_OF_NAMESPACE____


#endif
