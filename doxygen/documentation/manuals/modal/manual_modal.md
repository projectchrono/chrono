Chrono::Modal Reference Manual {#manual_modal}
=================================

In order to enable this module the user should go through the installation process: [Install and build Chrono::Modal](@ref module_modal_installation)

The **modal module** provides two main functionalities: the *modal analysis* and the *modal reduction*.


## Modal Analysis


Through the modal *analysis* the user is capable of extracting the modal shapes of structures and mechanisms together with their frequencies (and damping ratios). The analysis may include not only finite elements, but also rigid body and constraints. Also rigid-body modes are supported.

Two main sets of classes are available to solve the eigenvalue problems:
+ *eigenvalue solvers*, meant to deal directly with _pure matrices_, they have no knowledge of Chrono @ref chrono::ChSystem "ChSystem"s nor @ref chrono::ChAssembly "ChAssembly"s thus they *are not meant to be directly used by the average user*. 
  All eigenvalue solvers:
  - accept only real matrices
  - implement shift-and-invert iterative methods
  - can handle sparse matrices
  - get the lowest eigenvalues (according to the shift)
  Two main kind of solvers are available depending on the matrix type:
  - @ref chrono::modal::ChSymGenEigenvalueSolver "ChSymGenEigenvalueSolver" for symmetric matrices
  - @ref chrono::modal::ChUnsymGenEigenvalueSolver "ChUnsymGenEigenvalueSolver" for general matrices
+ *modal solvers*, meant to deal with Chrono @ref chrono::ChSystem "ChSystem"s or @ref chrono::ChAssembly "ChAssembly"s; they act as interface classes by running an *eigen solver* on a given @ref chrono::ChAssembly "ChAssembly". They can include the damping. *They are the preferred choice for the average user*.
  Modal solvers are split between:
  - *undamped*: if the Chrono system generates symmetric matrices then the undamped problem might be symmetric; depending on the case either a symmetric or an unsymmetric eigenvalue solver is required
  - *damped*: the damping matrix is included; the eigenvalue problem is always unsymmetric, thus requiring an appropriate unsymmetric eigenvalue solver

As a general knowledge, the Chrono @ref chrono::ChSystem "ChSystem" holds one top-level @ref chrono::ChAssembly "ChAssembly". This assembly may contain other assemblies, thus allowing to analyze (and reduce) only a subset of all the objects in the system: in this case the objects need to be added to the assembly and then the assembly to the system.

In order to visualize mode shapes the [Irrlicht module](@ref irrlicht_visualization) must be enabled. In this case the CMake structure will automatically enable the modal-specific visualization class @ref chrono::modal::ChModalVisualSystemIrrlicht "ChModalVisualSystemIrrlicht". This class is not included in a shared library since it is header-only.

To run a modal analysis few steps are required:
- create an eigenvalue solver through shared pointer e.g.
  ~~~{.cpp}
  auto eig_solver = chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>();
  ~~~
  It must be compatible with the coming modal solver
- create a compatible modal solver
  ~~~{.cpp}
  ChModalSolverDamped modal_solver(num_modes, tolerance, true, false, eig_solver);
  ~~~
- instantiate matrix and vectors required to store eigenvalues and eigenvectors
- call the solve for the modal solver
  ~~~{.cpp}
  modal_solver.Solve(sys.GetAssembly(), eigvects, eigvals, freq, damping_ratios);
  ~~~
- [for visualization] create a visual interface to visualize the modal shapes. The template option should be either `double` or `std::complex<double>` depending on the eigenvalue solver
  ~~~{.cpp}
  ChModalVisualSystemIrrlicht<std::complex<double>> vis;
  ~~~
- [for visualization] **only after** the call to Solve, fetch the results from the visual interface
  ~~~{.cpp}
  vis.AttachAssembly(sys.GetAssembly(), eigvects, freq);
  ~~~
- inside a loop call the usual rendering commands:
  ~~~{.cpp}
	vis.BeginScene();
	vis.Render();
	vis.EndScene();
  ~~~
  there is no need to call any other function.


[demo_MOD_analysis](https://github.com/projectchrono/chrono/blob/main/src/demos/modal/demo_MOD_analysis.cpp) shows a slightly more complex usage where a Chrono system is changed on-the-fly.

## Modal Reduction

Modal reduction methods allow to reduce the computational effort of a given Chrono model by replacing it with a simplified version by preserving the behaviour of the original system over a given set of frequencies. To leverage this functionality the user must add all the bodies, FE nodes and constraints to be reduced into a specific @ref chrono::modal::ChModalAssembly "ChModalAssembly" instead of adding them to the system directly, by calling:
- @ref chrono::modal::ChModalAssembly::Add() "ChModalAssembly::Add()" on those objects that has be considered as "boundary" i.e. that stay on the border between the internal (reduced) nodes and the external world
- @ref chrono::modal::ChModalAssembly::AddInternal() "ChModalAssembly::AddInternal()" on those objects that has be considered as "internal" i.e. that will be later reduced
The @ref chrono::modal::ChModalAssembly "ChModalAssembly" should then be added to the usual @ref chrono::ChSystem "ChSystem".

By calling @ref chrono::modal::ChModalAssembly::DoModalReduction() "ChModalAssembly::DoModalReduction()" Chrono will apply the conversion. After the call, it can be seen that the ChModalAssembly has a lower number of states thus allowing for a significant performance improvement during the simulation. At the same time, the ChModalAssembly graphical aspect will still look the same as the full model: the full internal state is indeed retrieved from the reduced one in order to allow the visualization.

*NOTE: at the moment only linear dynamics is supported for the subassembly, in the sense that the subassembly cannot withstand large rotations, ex. in a helicopter blade. Future developments will address this*

In order to call the modal reduction on an assembly the user should build a modal solver first (as discussed in the previous section) and then pass it as an argument to the @ref chrono::modal::ChModalAssembly::DoModalReduction() "ChModalAssembly::DoModalReduction()" method as suggested in [demo_MOD_reduction](https://github.com/projectchrono/chrono/blob/main/src/demos/modal/demo_MOD_reduction.cpp).
