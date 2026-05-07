import pytest
import numpy as np
import pychrono as chrono
import pychrono.fsi as fsi
import time

render = True
verbose = True

# Test tolerances
v_tolerance = 5e-3
d_tolerance = 1e-4

# Dimensions of the computational domain
bxDim = 0.2
byDim = 0.1
bzDim = 0.2

# Start time, step size, number of steps, forcing term, initial spacing
t_start = 1.0
dt = 2e-3
# num_steps = 500
num_steps = 500
force = 0.05
initial_spacing = 0.01

# Analytical solution for the unsteady plane Poiseuille flow (flow between two parallel plates).
def PoiseuilleAnalytical(Z, H, ctime, sysSPH):
    nu = sysSPH.GetViscosity() / sysSPH.GetDensity()
    F = sysSPH.GetBodyForce().x
    
    # Adjust plate separation and boundary locations for analytical formula. This accounts for the fact that
    # Chrono::FSI enforces the wall no-slip condition at the mid-point between the last BCE layer and SPH particles
    # closest to the wall.
    H = H - initial_spacing
    Z = Z - 0.5 * initial_spacing

    # Truncate infinite series to 50 terms
    v = 1.0 / (2.0 * nu) * F * Z * (H - Z)
    for n in range(50):
        term_val = (2 * n + 1)
        v -= (4.0 * F * (H**2) / (nu * (np.pi**3) * (term_val**3)) *
              np.sin(np.pi * Z * term_val / H) *
              np.exp(-(term_val**2) * (np.pi**2) * nu * ctime / (H**2)))
    return v

# Callback for setting initial SPH particle velocity
class InitialVelocityCallback(fsi.ParticlePropertiesCallback):
    def __init__(self, fluid_height, time):
        super().__init__()
        self.height = fluid_height
        self.time = time

    def set(self, sysSPH, pos):
        v_x = PoiseuilleAnalytical(pos.z, self.height, self.time, sysSPH)
        self.p0 = 0.0
        self.rho0 = sysSPH.GetDensity()
        self.mu0 = sysSPH.GetViscosity()
        self.v0 = chrono.ChVector3d(v_x, 0, 0)

# ------------------------------------------------------------------
def test_fsi_poiseuille_flow():
    # Create a Chrono system and the FSI problem
    sysMBS = chrono.ChSystemSMC()
    fsiP = fsi.ChFsiProblemCartesian(initial_spacing, sysMBS) # the fsi in C++
    fsiP.SetVerbose(verbose)
    
    sysSPH = fsiP.GetFluidSystemSPH()

    # Set gravitational acceleration
    fsiP.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    
    # Set CFD fluid properties
    fluid_props = fsi.FluidProperties()
    fluid_props.density = 1000.0
    fluid_props.viscosity = 1.0
    fsiP.SetCfdSPH(fluid_props)

    # Set forcing term
    sysSPH.SetBodyForce(chrono.ChVector3d(force, 0, 0))

    # Set SPH solution parameters
    sph_params = fsi.SPHParameters()
    sph_params.integration_scheme = fsi.IntegrationScheme_RK2
    sph_params.num_bce_layers = 3
    sph_params.initial_spacing = initial_spacing
    sph_params.d0_multiplier = 1.0
    sph_params.max_velocity = 0.1
    sph_params.shifting_method = fsi.ShiftingMethod_NONE
    sph_params.density_reinit_steps = 10000
    sph_params.viscosity_method = fsi.ViscosityMethod_LAMINAR
    sph_params.use_delta_sph = False
    sph_params.eos_type = fsi.EosType_ISOTHERMAL
    sph_params.use_consistent_gradient_discretization = True
    sph_params.use_consistent_laplacian_discretization = True
    fsiP.SetSPHParameters(sph_params)

    fsiP.SetStepSizeCFD(dt)
    fsiP.SetStepsizeMBD(dt)

    # Create SPH material (do not create boundary BCEs)
    # Add box container (only bottom and top walls)
    fsize = chrono.ChVector3d(bxDim, byDim, bzDim - 2 * initial_spacing)
    fsiP.Construct(
        fsize,
        chrono.ChVector3d(0, 0, initial_spacing),
        fsi.BoxSide_Z_NEG | fsi.BoxSide_Z_POS
    )

    # Explicitly set computational domain
    c_min = chrono.ChVector3d(-bxDim/2 - initial_spacing/2, -byDim/2 - initial_spacing/2, -10 * initial_spacing)
    c_max = chrono.ChVector3d(bxDim/2 + initial_spacing/2, byDim/2 + initial_spacing/2, bzDim + 10 * initial_spacing)
    fsiP.SetComputationalDomain(chrono.ChAABB(c_min, c_max), fsi.BC_ALL_PERIODIC)

    # Set particle initial velocity
    cb = InitialVelocityCallback(bzDim, t_start)
    fsiP.RegisterParticlePropertiesCallback(cb)

    # Initialize FSI problem
    fsiP.Initialize()

    # 7. Simulation loop
    num_particles = fsiP.GetNumSPHParticles()
    v = np.zeros(num_particles)
    va = np.zeros(num_particles)
    d = np.zeros(num_particles)
    p = np.zeros(num_particles)

    passed = True
    current_time = t_start

    timer = chrono.ChTimer()
    timer.start()
    for step in range(num_steps):
        fsiP.DoStepDynamics(dt)
        current_time += dt

        pos = sysSPH.GetParticlePositionsNumpy()
        vel = sysSPH.GetParticleVelocitiesNumpy()
        dpv = sysSPH.GetParticleFluidPropertiesNumpy()

        va[:num_particles] = PoiseuilleAnalytical(pos[:num_particles, 2], bzDim, current_time, sysSPH)
        
        v = vel[:num_particles, 0]
        d = dpv[:num_particles, 0]
        p = dpv[:num_particles, 1]
        
        v_max = np.max(v)
        v_min = np.min(v)
        va_max = np.max(va)
        va_min = np.min(va)

        d_max = np.max(d)
        d_min = np.min(d)

        v_err = v - va
        v_err_RMS = np.sqrt(np.mean(v_err**2))

        v_rel_err = v_err_RMS / va_max
        d_rel_err = (d_max - d_min) / sysSPH.GetDensity()
        
        if (v_rel_err > v_tolerance or d_rel_err > d_tolerance):
            passed = False
            break
    assert passed
