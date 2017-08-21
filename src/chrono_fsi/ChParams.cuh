/*
 * MyStruncts.cuh
 *
 *  Created on: Mar 4, 2015
 *      Author: Arman Pazouki
 */

#ifndef CHPARAMS_CUH_
#define CHPARAMS_CUH_
#include "chrono_fsi/custom_math.h"

namespace chrono {
namespace fsi {

enum BceVersion { ADAMI = 0, mORIGINAL = 1 };

/**
 * @brief Simulation Parameters
 * @details
 * 		The description of each variable is in front of it
 */
struct SimParams {
    int3 gridSize;       /* dx, dy, dz distances between particle centers. */
    Real3 worldOrigin;   /* Origin of the system. */
    Real3 cellSize;      /* Size of cells for collision detection*/
    uint numBodies;      /* */
    Real3 boxDims;       /* Dimensions of the domain. How big is the box that the domain is in. */
    Real sizeScale;      /* Useless (Don't change it !) */
    Real HSML;           /* Interaction Radius. (or h) */
    Real MULT_INITSPACE; /* Multiplier to hsml to determine the initial separation of the fluid particles and the fixed
     separation for the boundary particles. This means that the separation will always be a
     multiple of hsml. Default value = 1.0. */
    Real epsMinMarkersDis;   // epsilon mult for minimum distance between markers (d_min = eps * HSML)
    int NUM_BOUNDARY_LAYERS; /*  Number of particles layers that will be used in the boundary. Default value = 3. */
    Real toleranceZone; /* Helps determine the particles that are in the domain but are outside the boundaries, so they
     are not considered fluid particles and are dropped at the beginning of the simulation. */
    int NUM_BCE_LAYERS; /* Number of fixed particle layers to rigid/flexible bodies which act as the boundaries. Default
     value = 2. */
    Real solidSurfaceAdjust; /* */
    Real BASEPRES;           /* Relative value of pressure applied to the whole domain. */
    Real LARGE_PRES; /* Artificial pressure for boundary particles. Make sure fluid particles do not go through the
     boundaries. Note that if time step is not small enough particles near the boundaries might build
     up huge pressures and will make the simulation unstable. */
    Real3
        deltaPress; /* Change in Pressure. This is needed for periodic BC. The change in pressure of a particle when it
   moves from end boundary to beginning.  */
    int nPeriod;    /* Only used in snake channel simulation. Tells you how long the channel will be. */
    Real3 gravity;  /* Gravity. Applied to fluid, rigid and flexible. */
    Real3 bodyForce3; /* Constant force applied to the fluid. Flexible and rigid bodies are not affected by this force
     directly, but instead they are affected indirectly through the fluid. */
    Real rho0;        /* Density */
    Real markerMass;  /* marker mass */
    Real mu0;         /* Viscosity */
    Real v_Max; /* Max velocity of fluid used in equation of state. Run simulation once to be able to determine it. */
    Real EPS_XSPH;          /* Method to modify particle velocity. */
    Real multViscosity_FSI; /* Multiplier that helps determine the viscosity for boundary particles. For example, if the
     value is 5 then the boundary particles will be 5 times more viscous than the fluid
     particles. Boundary particles should be more viscuous becayse they are supposed to slow
     down the fluid particles near the boundary. */
    Real dT; /*  Time step. Depending on the model this will vary and the only way to determine what time step to use is
     to run simulations multiple time and find which one is the largest dT that produces a stable simulation.
     */
    Real tFinal;    /* Total simulation time. */
    Real timePause; /* Time that we let pass before applying body forces. This is done to allow the particles to
     stabilize
     first. Run the fluid only during this time, with dTm = 0.1 * dT */
    Real timePauseRigidFlex; /* Time before letting rigid/flex move. Keep the rigid and flex stationary during this time
     (timePause + timePauseRigidFlex) until the fluid is fully developed */
    Real kdT;                /* Implicit integration parameter. Not very important */
    Real gammaBB;            /* Equation of state parameter. */
    Real3 cMin; /* Lower leftmost part of the space shown in a simulation frame. This point is usually outside the
     tolerance zone. */
    Real3 cMax; /* Upper right most part of the space shown in a simulation frame. This point is usually outside the
     tolerance zone.*/
    Real3 cMinInit; /* */             // Arman : note, this need to be added to check point
    Real3 cMaxInit; /* */             // Arman : note, this need to be added to check point
    Real3 straightChannelBoundaryMin; /* Origin of the coordinate system (Point (0,0,0)). In this case (straigh channel)
     this point is where the leftmost particle of the 3rd layer of boundary particles
     is located. Everything below this point is outside the tolerance zone, this means
     that everything below is not considered part of the system. */
    Real3 straightChannelBoundaryMax; /* Upper right most part of the system, this is Point(2 * mm, 1 * mm, 3 * mm)
     where mm
     is a constant defined at the beginning of main.cpp. This is also the rightmost
     particle of the 3rd layer of the top boundary particles. Everything above this
     point is not considered part of the system. */
    Real binSize0; /* Suggests the length of the bin each particle occupies. Normally this would be 2*hsml since hsml is
     the radius of the particle, but when we have periodic boundary condition varies a little from
     2*hsml. This may change slightly due to the location of the periodic BC. */
    Real3 rigidRadius;        /* Radius of rigid bodies. */
    int densityReinit; /* */  // 0: no; 1: yes
    int contactBoundary;      /* 0: straight channel, 1: serpentine */
    int enableTweak; /* */    // 0: no tweak, 1: have tweak
    int enableAggressiveTweak;
    /* */               // 0: no aggressive tweak; 1: with aggressive tweak (if 1, enableTweak should be 1 too)
    Real tweakMultV;    /* maximum allowed velocity: tweakMultV * HSML / dT;  NOTE: HSML and dT must be defined. So this
                           line
                           comes after them */
    Real tweakMultRho;  /* maximum allowed density change in one time step: tweakMultRho * rho0 */
    BceVersion bceType; /* maximum allowed density change in one time step: tweakMultRho * rho0 */
};

}  // end namespace fsi
}  // end namespace chrono

#endif /* MYSTRUCTS_CUH_ */
