# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
#
#  Models using the ANCF gradient-deficient cable element
#
# =============================================================================

import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A beam composed of a single ANCF cable element, with one end fixed
#         and a constant force applied at the other node. A rigid body is
#         connected to node2.
# ----------------------------------------------------------------------------
class Model1:
    def __init__(self, system, mesh):
        beam_L = 0.1
        beam_diameter = 0.015

        # Create a section, i.e. thickness and material properties
        # for beams. This will be shared among some beams.
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(beam_diameter)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.000)

        # Create the nodes
        hnodeancf1 = fea.ChNodeFEAxyzD(chrono.ChVector3d(0, 0, -0.2), chrono.ChVector3d(1, 0, 0))
        hnodeancf2 = fea.ChNodeFEAxyzD(chrono.ChVector3d(beam_L, 0, -0.2), chrono.ChVector3d(1, 0, 0))

        mesh.AddNode(hnodeancf1)
        mesh.AddNode(hnodeancf2)

        # Create the element

        belementancf1 = fea.ChElementCableANCF()

        belementancf1.SetNodes(hnodeancf1, hnodeancf2)
        belementancf1.SetSection(msection_cable)

        mesh.AddElement(belementancf1)

        # Apply a force or a torque to a node:
        hnodeancf2.SetForce(chrono.ChVector3d(0, 3, 0))

        hnodeancf1.SetFixed(True)

        # Add a rigid body connected to the end of the beam:

        self.body = chrono.ChBodyEasyBox(0.1, 0.02, 0.02, 1000)
        self.body.SetPos(hnodeancf2.GetPos() + chrono.ChVector3d(0.05, 0, 0))
        system.Add(self.body)

        constraint_pos = fea.ChLinkNodeFrame()
        constraint_pos.Initialize(hnodeancf2, self.body)
        system.Add(constraint_pos)

        constraint_dir = fea.ChLinkNodeSlopeFrame()
        constraint_dir.Initialize(hnodeancf2, self.body)
        constraint_dir.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
        system.Add(constraint_dir)
        
    # Prposition of end body
    def PrintBodyPosition(self) :
        print( "Time: " + str(self.body.GetChTime()))
        print("  " + self.body.GetPos() )

# ----------------------------------------------------------------------------
# Model2: A beam composed of 10 ANCF cable element, with one end hinged to
#         ground, moving under gravity alone.
# This model demonstrates the use of the utility class ChBuilderCableANCF.
# ----------------------------------------------------------------------------
class Model2 :
    def __init__(self, system, mesh):
        # Create a section, i.e. thickness and material properties
        # for beams. This will be shared among some beams.

        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.000)

        # This ChBuilderCableANCF helper object is very useful because it will
        # subdivide 'beams' into sequences of finite elements of beam type, ex.
        # one 'beam' could be made of 5 FEM elements of ChElementBeamANCF_3333 class.
        # If new nodes are needed, it will create them for you.
        builder = fea.ChBuilderCableANCF()

        # Now, simply use BuildBeam to create a beam from a poto another:
        builder.BuildBeam(mesh,                    # the mesh where to put the created nodes and elements
                          msection_cable2,         # the ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
                          10,                      # the number of ChElementBeamANCF_3333 to create
                          chrono.ChVector3d(0, 0, -0.1),  # the 'A' poin space (beginning of beam)
                          chrono.ChVector3d(0.5, 0, -0.1))  # the 'B' poin space (end of beam)

        # After having used BuildBeam(), you can retrieve the nodes used for the beam,
        # For example say you want to fix both pos and dir of A end and apply a force to the B end:
        # builder.GetLastBeamNodes().back().SetFixed(True)
        builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.2, 0))

        # For instance, now retrieve the A end and add a constrato
        # block the position only of that node:
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)

        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
        system.Add(constraint_hinge)

# ----------------------------------------------------------------------------
# Model3: A set of beam elements with connected bodies, each with different
#         number of ANCF cable elements.
# ----------------------------------------------------------------------------

class Model3 :
    def __init__(self, system, mesh, n_chains = 6):
        self.bodies = []#[chrono.ChBodyEasyBox for i in range(n_chains)]
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.000)

        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)

        for j in range(n_chains):
            builder = fea.ChBuilderCableANCF()

            # Now, simply use BuildBeam to create a beam from a poto another:
            builder.BuildBeam(mesh,             # the mesh where to put the created nodes and elements
                              msection_cable2,  # ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
                              1 + j,            # number of ChElementBeamANCF_3333 to create
                              chrono.ChVector3d(0, 0, -0.1 * j),             # poA (beginning of beam)
                              chrono.ChVector3d(0.1 + 0.1 * j, 0, -0.1 * j)  # poB (end of beam)
            )

            builder.GetLastBeamNodes().back().SetForce(chrono.ChVector3d(0, -0.2, 0))

            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().front(), mtruss)
            system.Add(constraint_hinge)

            msphere = chrono.ChVisualShapeSphere(0.02)
            constraint_hinge.AddVisualShape(msphere)

            # make a box and connect it
            mbox = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000)
            mbox.SetPos(builder.GetLastBeamNodes().back().GetPos() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(mbox)

            constraint_pos = fea.ChLinkNodeFrame()
            constraint_pos.Initialize(builder.GetLastBeamNodes().back(), mbox)
            system.Add(constraint_pos)

            constraint_dir = fea.ChLinkNodeSlopeFrame()
            constraint_dir.Initialize(builder.GetLastBeamNodes().back(), mbox)
            constraint_dir.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir)

            # make another beam
            builder.BuildBeam(
                mesh,                # mesh where to put the created nodes and elements
                msection_cable2,     # ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
                1 + (n_chains - j),  # number of ChElementBeamANCF_3333 to create
                chrono.ChVector3d(mbox.GetPos().x + 0.1, 0, -0.1 * j),                        # poA (beginning of beam)
                chrono.ChVector3d(mbox.GetPos().x + 0.1 + 0.1 * (n_chains - j), 0, -0.1 * j)  # poB (end of beam)
            )

            constraint_pos2 = fea.ChLinkNodeFrame()
            constraint_pos2.Initialize(builder.GetLastBeamNodes().front(), mbox)
            system.Add(constraint_pos2)

            constraint_dir2 = fea.ChLinkNodeSlopeFrame()
            constraint_dir2.Initialize(builder.GetLastBeamNodes().front(), mbox)
            constraint_dir2.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir2)

            # make a box and connect it
            self.bodies.append( chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000) )
            self.bodies[j].SetPos(builder.GetLastBeamNodes().back().GetPos() + chrono.ChVector3d(0.1, 0, 0))
            system.Add(self.bodies[j])

            constraint_pos3 = fea.ChLinkNodeFrame()
            constraint_pos3.Initialize(builder.GetLastBeamNodes().back(), self.bodies[j])
            system.Add(constraint_pos3)

            constraint_dir3 = fea.ChLinkNodeSlopeFrame()
            constraint_dir3.Initialize(builder.GetLastBeamNodes().back(), self.bodies[j])
            constraint_dir3.SetDirectionInAbsoluteCoords(chrono.ChVector3d(1, 0, 0))
            system.Add(constraint_dir3)
    

    # Prpositions of end bodies in each chain
    def PrintBodyPositions(self) :
        print( "Time: " + str(self.bodies[0].GetChTime()) ) 
        for body in self.bodies :
            print( "  " << body.GetPos() )

