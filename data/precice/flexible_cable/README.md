Coupling of a Chrono FEA cable with Chrono::FSI-SPH
===================================================

A flexible cable, modeled with 8 ANCF cable elements and anchored at the bottom of a container, is impacted by
a collapsing granular (CRM) or water (CFD) column. This is the preCICE counterpart of
`demo_FSI-SPH_Flexible_Cable` (single cable, no rigid bodies) and of the flexible cable models of
`demo_YAML_fsi` (see `data/yaml/fsi/flexible_cable`).

- The solid participant (`solid_chrono`) is a Chrono MBS model containing the FEA mesh. The cable nodes are
  exchanged through a `FEA_MESH_NODES` coupling mesh: the solid participant sends node positions, velocities,
  and accelerations, and receives nodal forces.
- The fluid participant (`fluid_sph`) is a Chrono::FSI-SPH solver. It represents the cable with a proxy mesh,
  built from the node positions and segments given in its own specification file, which is used to generate the
  BCE markers of the cable.

The preCICE configurations (`precice_config_crm.xml` and `precice_config_cfd.xml`) use a serial explicit
coupling scheme with the fluid participant first and a time window equal to the integration step size, which
matches the (default) sequential coupling scheme of a non-preCICE Chrono::FSI simulation.

Running the coupled simulation
------------------------------

The two participants must be run in two different processes, started from the same directory (the preCICE
configuration uses the current directory as exchange directory), with the same fluid problem (`-f CRM` or
`-f CFD`):

```
demo_PRECICE_flexible_cable -p Solid -f CRM
demo_PRECICE_flexible_cable -p Fluid_SPH -f CRM
```

With output enabled (the default), the solid participant writes the trajectory of the top cable node to
`DEMO_OUTPUT/PRECICE_Flexible_Cable/<CRM|CFD>_SOFT_CABLE/results.txt`, in the same format as
`demo_FSI-SPH_Flexible_Cable`.

Stiff cable
-----------

The solid participant can also use a stiff cable (`solid_chrono/mbs_model_stiff.yaml`), identical to the soft cable
(`solid_chrono/mbs_model.yaml`) except for its Young's modulus (1.2e10 instead of 6e8). Under the collapsing water
column, the soft cable is bent flat onto the floor of the container, while the stiff cable deflects moderately (tip
rotation below 20 degrees). Select it with `-m stiff_cable` (the default is `-m soft_cable`), passed to both
participants so that they share the output directory `DEMO_OUTPUT/PRECICE_Flexible_Cable/<CRM|CFD>_STIFF_CABLE`:

```
demo_PRECICE_flexible_cable -p Solid -f CFD -m stiff_cable
demo_PRECICE_flexible_cable -p Fluid_SPH -f CFD -m stiff_cable
```

The fluid participant is the same for both cable models: it only uses the positions, velocities, and accelerations
of the cable nodes.
