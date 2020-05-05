Change Log
==========

- [Unreleased (development version)](#unreleased-development-branch)
    - [Chrono::Vehicle simulation world frame](#added-chronovehicle-simulation-world-frame)
    - [CASCADE module](#changed-cascade-module)
	- [Collision shapes and contact materials](#changed-collision-shapes-and-contact-materials)
- [Release 5.0.1](#release-501---2020-02-29)
- [Release 5.0.0](#release-500---2020-02-24)
	- [Eigen dense linear algebra](#changed-refactoring-of-dense-linear-algebra)
	- [Eigen sparse matrices](#changed-eigen-sparse-matrices-and-updates-to-direct-sparse-linear-solvers)
- [Release 4.0.0](#release-400---2019-02-22)

## Unreleased (development branch)

### [Added] Chrono::Vehicle simulation world frame

While the default world frame for Chrono::Vehicle simulations is an ISO (Z up) frame, we now provide support to simulate vehicles in a scene specified in a different reference frame (for example, an Y up frame).
The world frame is uniquely defined through a rotation matrix (the rotation required to align the ISO frame with the desired world frame). To change the world frame definition from the default ISO convention, the desired world frame must be set **before** any Chrono::Vehicle library call:
```cpp
ChMatrix33<> world_rotation = ...
ChWorldFrame::Set(worl_rotation);
```
A shortcut is provided to specify a world frame with Y up (and X forward, Z to the right):
```cpp
ChWorldFrame::SetYUP();
```


### [Changed] CASCADE module

1.	Support for OpenCASCADE 7.4.0. The API of OpenCASCADE introduced some changes in the 7.4.0 version so we also updated the CASCADE module of Chrono. Please download and upgrade your OpenCASCADE version as it is not backward compatible. (The module is optionally built via CMake configuration flag ENABLE_MODULE_CASCADE, also remember to update the CASCADE_INCLUDE_DIR and CASCADE_LIBDIR paths and to update your PATH if you added the path to Cascade dlls)

2.	The method `ChCascadeDoc::CreateBodyFromShape()` is obsolete. Just use the `ChBodyEasyCascade` class to obtain the same result, for example:
    ```cpp
    auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(myshape, ...);
    ```

3.	The mesh tesselation algorithm could give coarser or finer meshes with respect to the previous release.


### [Changed] Collision shapes and contact materials

The main change is that now contact materials are associated with collision shapes, as opposed to bodies.  We've always had the underlying concept of a collision shape, with a body's collision model potentially including multiple shapes, but these were always sharing the exact same contact material.   With the new code, each collision shape in a collision model can have its own contact material properties.  Of course, through shared pointers, collision shapes in the same model or even in different models can still point to the same (shared) material.   Also, a nice consequence of this change is that now a ChBody is agnostic of contact method or contact materials (as it should be).

The new API requires you to always pass a material (as a shared pointer) whenever you create a collision shape or other contactable "primitive" (such as a contact surface or node cloud for FEA collision).   The material you pass must still be consistent with the *contact method* (NSC or SMC) of the containing system.

Here's a summary of the main API changes (there were many other changes under the hood, but here we only highlight changes to the public API).  Moreover, we discuss only the C++ API, but the equivalent changes also apply to the Python API.

1.	Renamed headers.  For consistency, we renamed various files under src/chrono/collision which had a prefix `ChC` to simply have the prefix `Ch`.  For example, `ChCCollisionModel.h`  -->  `ChCollisionModel.h`

2.	The contact method, NSC or SMC, is now a top-level enum class named `ChContactMethod` (previously it was nested under ChMaterialSurface).  So use things like:
	```cpp
  	if (system->GetContactMethod() == ChContactMethod::NSC) {
  	    ...
  	}
	```

3.  Contact materials. The functionality of the base class `ChMaterialSurface` and derived classes `ChMaterialSurfaceNSC` and `ChMaterialSurfaceSMC` is unchanged.  However, for convenience, material properties common between NSC and SMC were moved to the base class (these include coefficients of friction for sliding, rolling, spinning and the coefficient of restitution). Furthermore, we provide a utility method to create a contact material of the specified type with corresponding default parameters.  In a program that can switch between SMC and NSC formulations (say, based on a flag `ChContactMethod contact_method`), you can then write
	```cpp
	ChContactMethod contact_method = ChContactMethod::SMC;  // or ChContactMethod::NSC
	...
	auto mat = ChMaterialSurface::DefaultMaterial(contact_method);
	mat->SetFriction(0.7f);
	```
	If you also want to change a contact method-specific property, you must then use something like:
	```cpp
	if (contact_method == ChContactMethod::SMC) {
	    std::static_pointer_cast<ChMaterialSurfaceSMC>(mat)->SetYoungModulus(1e7f);
	}
	```

4.	The `ChBody` constructor does not take the contact method as an argument anymore (which previously defaulted to NSC)

5.	`ChBodyEasy***` classes. The list of constructor arguments here has changed, for a more natural order.  All collision-related arguments have been moved to the end of the list. In particular, the flag to indicate whether or not to create a visual asset (default `true`) comes before the flag to indicate whether or not to create a collision shape (default `false`).  If the latter is `true`, the next argument must be a contact material (default `nullptr`).

	Be careful here, as it's easy to overlook the correct changes (because of arguments with default values).  For example, the old:
	```cpp
  	auto body = chrono_types::make_shared<ChBodyEasySphere>(radius, density, true);  // old
	```
	would create a sphere with visualization and collision enabled (with material properties from the body itself).  This is also valid with the new code, but this will now create a body with a sphere visualization asset but **no** collision shape.  To get the expected result, you need to use:
	```cpp
  	auto body = chrono_types::make_shared<ChBodyEasySphere>(radius, density, true, true, material);  // new
	```
	and pass a valid material (consistent with the system to which you will add the body).

6.	There is now a proper `ChCollisionShape` class which is very light weight (it carries only the type of shape, from an enum ChCollisionShape::Type, and a (shared) pointer to a contact material).  There are derived classes for the various collision systems (namely the one based on Bullet and the one used in Chrono::Parallel), but few users would have to worry about or work with those.

7.	A collision model maintains a vector of collision shapes (in the order they were added to the model by the user).  There are public accessor methods on ChCollisionModel to get the list of shapes or individual shapes (by its index in the list).

8.	Adding collision shapes.  All `ChCollisionModel::Add***` methods now take as their first argument a (shared) pointer to a contact material.  There is no "default" material automatically constructed under the hood for you anymore.   However, the NSC and SMC contact materials still have constructors that set their respective properties to some default values.  So you can write something like:
	```cpp
  	// construct an SMC material with default properties
  	auto my_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>(); 
  	auto my_body = chrono_types::make_shared<ChBody>();  // note: no need to specify SMC contact here!
  	...
  	my_body->GetCollisionModel()->AddSphere(my_mat, radius);
  	...
  	my_system.AddBody(my_body);  // it is assumed that my_system is a ChSystemSMC
  	```

9.	Utility functions in ChUtilsCreators.  Similar to `ChBodyEasy***`, the various `utils::Add***Geometry` functions now also require a contact material;  this is always their 2nd argument.

10.	FEA contact surfaces.  The two options, `ChContactSurfaceMesh` and `ChContactSurfaceNodeCloud` now require a contact material at construction (there is no SetSurfaceMaterial() method anymore).  As you already know, the only acceptable type of material in this case is ChMaterialSurfaceSMC.  Note that the contact material passed at construction is shared by all components of the FEA contact surface (triangles or nodes, respectively).  Sample code:
	```cpp
  	auto my_surf = chrono_types::make_shared<ChContactSurfaceMesh>(my_materialSMC);
  	my_mesh->AddContactSurface(my_surf);
  	my_surf->AddFacesFromBoundary(0.05);
  	```

11.	`ChCollisionInfo` and user-provided contact callbacks.  A ChCollisionInfo object (which encapsulates information about a collision pair and is used internally to create contacts) now also include pointers to the colliding shapes in the two interacting collision models.   When a contact must be created internally, a composite contact material is created on the fly from the contact materials of the two colliding shapes.

	This has a direct impact on how a user can add custom contacts through a `ChSystem::CustomCollisionCallback` object.   Indeed, in its override of OnCustomCollision, a derived callback class is expected to create a ChCollisionInfo object and then pass it to the AddContact method of the underlying contact container.  However, such a custom collision has no collision shapes to work with!   For this purpose, we added a new form of `AddContact` that takes as additional argument to ChMaterialSurface objects.   In this case, the ChCollisionInfo object can set its collision shape members to `nullptr`.

	For examples, look at two new demos: `demo_IRR_custom_contact` and `demo_PAR_custom_contact`.

12.	`Chrono::Vehicle` changes.  Most of the changes here were low-level (and as such transparent to the user).  The notable difference has to do with terrain specification.  The RigidTerrain::AddPatch function now expect as their first argument a contact material (consistent with the containing system) that will be used for that one patch (box patch, trimesh patch , or height-field patch).  

	If you are using vehicles specified through JSON files, beware that for some subsystems the schema has changed for entries related to collision shapes and contact materials.   See the examples provided in the various sub-directories of `data/vehicle/`.

13.	`Chrono::Parallel` and `Chrono::Distributed`.  Here too, the vast majority of the changes happened under the hood.  Besides the changes described above (related to how collision shapes are defined), there are only a couple of things to mention:
	-	In Chrono::Parallel, the custom specification of collision shapes was merged into a new class ChCollisionShapeParallel.  If, for any reason, you need to traverse the list of collision shapes in a collision model, simply loop through the vector of shapes in ChCollisionModel (see #7 above).
	-	In Chrono::Distributed, the analytical plane boundaries now require a contact material at construction (internally, this is used in a custom collision callback, as described in #11 above).


## Release 5.0.1 - 2020-02-29

### [Fixed]

- Correct the ChElementBeamANCF applied moment calculation so that it uses normalized shape function derivatives
- Comment out code related to applying moments on ANCF elements (requires further testing))

## Release 5.0.0 - 2020-02-24

### [Changed] Refactoring of dense linear algebra

Starting with this release, Chrono relies on Eigen3 for all dense linear algebra.

1. With this change, Chrono requires `Eigen3` version 3.3.0 or newer.  Unless not possible for other reasons, we suggest you use their latest version, 3.3.7.
	- Eigen is available for download at http://eigen.tuxfamily.org/
	- Eigen is a headers-only library, so no install is required in order to use it in Chrono.
	- If the location of the Eigen headers is not automatically detected by CMake, manually specify it by setting the CMake variable `EIGEN3_INCLUDE_DIR`.
	- Note: CUDA 9.1 has removed a file (`math_functions.hpp`) which is referenced by Eigen prior to version 3.3.6.  As such, if you build any of the GPU-based Chrono modules (FSI or Granular) and use CUDA 9.1, make sure to use Eigen 3.3.7.

2. The switch to Eigen does come with a few strings attached
	- On the flip side, this makes the code a lot cleaner, easier to understand and maintain, and facilitates several further developments on our road-map.
	- Furthermore, performance did not degrade and in fact improved meaningfully on practically all our benchmark tests (measured with GCC and clang on Linux and MacOS).
	- On the other hand, libraries that use expression templates metaprogramming techniques (as Eigen does) lead to longer compile times. Moreover, while release (optimized) code is faster, code built in debug mode will likely be (significantly) slower.
	- Finally, because several Chrono classes now need to use an (Eigen-provided) overloaded operator new to ensure memory alignment and because of some limitations of the C++ language, this change has an important consequence on user code: std::make_shared **cannot always be used safely**. The solution we adopted is to provide `chrono_types::make_shared` replacement functions which should be used throughout (see below).

3. The new matrix and vector classes
	- The "old" Chrono matrix types (`ChMatrixNM`, `ChMatrixDynamic`, `ChVectorDynamic`) are now nothing but *type aliases* of appropriate Eigen types (`see ChMatrix.h`). 
	In other words, use them as you would Eigen types (see https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html).
	- For completeness, we introduced additional types (such as `ChVectorN`,  `ChRowVectorN`, `ChRowVectorDynamic`), all defined in ChMatrix.h.
	- We only implemented a handful of extensions needed for Chrono (such as a method for computing the WRMS vector norm) using the "back-door" extension mechanism provided by Eigen (see `ChMatrixEigenExtensions.h`)
	- The "old" base class ChMatrix was **eliminated**.  For the instances where a Chrono function needs to accept either a dynamic (ChMatrixDynamic) or a static (ChMatrixNM) argument, we rely on the `Eigen::Ref` mechanism. For this, we defined various type aliases, such as `ChMatrixRef` and `ChMatrixConstRef` (see ChMatrix.h).  Currently, these assume a 'double' scalar type as this is all that's needed in Chrono (they could be templated by the scalar type, but this will be done only if absolutely needed).
	- For 3x3 matrices, the `ChMatrix33` class (as before, templated by the scalar type) is derived from a 3x3 fixed-size vectorizable Eigen matrix.  Using inheritance here was needed in order to implement various custom constructors and methods.  
	- For clarity and to properly separate responsibilities, we added a small set of 3x4 and 4x4 matrices specific to multibody dynamics.  These derived classes have strict and limited functionality and are unlikely to show up in user code (see `ChMatrixMBD.h`).
	- Note: for now, the ChVector and ChQuaternion classes were left unchanged.
	- See `demo_CH_linalg.cpp` for simple examples of matrix operations with the new Chrono classes and types.

4. Obsolete/eliminated Chrono classes
	- With the switch to Eigen, we **removed** `ChLinearAlgebra`.  More robust and varied matrix factorization and linear system solvers can be used directly from Eigen (see http://eigen.tuxfamily.org/dox/group__DenseLinearSolvers__chapter.html).
	- We also **removed** the custom sparse matrix classes `ChLinkedListMatrix` and `ChMapMatrix`.  

5. Some considerations for developers
	- Consult the Eigen documentation (http://eigen.tuxfamily.org/dox/) and FAQs to understand what Eigen provides and how it should be used (e.g., when it's meaningful/appropriate to use fixed-size Eigen objects)
	- Look at block operations with Eigen matrices and vectors (https://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html)These make code much more clear (contrast this with the use of the old "paste" functions which was obfuscated and prone to mistakes).   However, always measure performance impacts;  in some cases (especially with smaller matrices), explicit loops may be more efficient than Eigen block operations. 
	- Beware of aliasing issues (https://eigen.tuxfamily.org/dox/group__TopicAliasing.html).  
	- Beware of memory alignment issues (https://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html)
	In particular, if you create a class that has a fixed-size vectorizable Eigen type (a ChMatrixNM, a ChVectorN, or most common a ChMatrix33) make sure to overload its operator new.  For that, use the Eigen-provided macro `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` (see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html)
	- When creating shared pointers, make sure to use **`chrono_types::make_shared`** (see below)

6. API and user code
	- Except for the limited situations where user code explicitly used the old-style matrix operations (which should now use the cleaner Eigen API; see e.g. https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html), the main impact on users has to do with the shared pointers in the Chrono public API.
	- The issue has to do with the fact that std::make_shared uses 'placement new' to create a shared pointer with a single memory allocation.  This means that std::make_shared will not use an overloaded 'operator new' for classes that provide it (for memory alignment considerations).
	- To maintain encapsulation (as much as possible) and not require users to know/care about the guts of Chrono (namely know which classes overload, or have a parent that overloads, their operator new), the solution we adopted is to provide **custom** make_shared functions (in the namespace chrono_types) and a mechanism to pick at compile-time which one should be used (conditional on the object's class having or not an overloaded operator new).   For classes that require aligned memory (i.e. classes that have an overloaded operator new), **`chrono_types::make_shared`** explicitly creates the shared_ptr.   Otherwise, it falls back on using std::make_shared (see `ChTypes.h`).
	- As such, user code should always create a shared_ptr using something like:
	~~~.{.cpp}
	auto foo = chrono_types::make_shared<T>(bar);
	~~~
	- See the various Chrono demos and tests for examples.

### [Changed] Eigen sparse matrices and updates to direct sparse linear solvers

Starting with this release, Chrono also uses Eigen for all sparse matrix needs (which are relatively limited and have little, if any, direct consequence on the public API).
A `ChSparseMatrix` is just a **type alias** to an Eigen SparseMatrix with double scalar type, row-major storage order, and int storage index type.  

1. The main effects on public API relate to concomitant updates we made to the sparse direct linear solvers (the Chrono::MKL interface to the Intel MKL Pardiso solver and the Chrono::Mumps interface to the MUMPS solver).  While we now rely on Eigen’s own interface to Pardiso, Chrono::Mumps still implements a custom interface to MUMPS (ChMumpsEngine).  

	For examples of usage, see `demo_FEA_cablesMKL` and `demo_FEA_cablesMUMPS`.

2. Both sparse direct solvers (and any others that may be provided in the future) share the same functionality in terms of controlling the identification and update of the matrix sparsity pattern.  The main features implemented in the base class `ChSolverDirect` are:
	- *sparsity pattern lock*
	The sparsity pattern lock skips sparsity identification or reserving memory for non-zeros on all but the first call to the solver setup. This feature is intended for problems where the system matrix sparsity pattern does not change (or changes very little) from call to call.  See ChSolverDirect::LockSparsityPattern.
	- *sparsity pattern learning*
	The sparsity pattern learning feature acquires the sparsity pattern in advance, in order to speed up matrix assembly.  Enabled by default, the sparsity matrix learner identifies the exact matrix sparsity pattern (without actually setting any non-zeros). See ChSolverDirect::UseSparsityPatternLearner.
	- In situations where the problem structure changes but relatively infrequently, it is still desirable to lock the sparsity pattern.  However, if using the sparsity pattern learner, an update must be explicitly triggered by the user after a problem modification (by calling `ChSparseDirect::ForceSparsityPatternUpdate`).  For an example, `see demo_FEA_beams_extrude`.
	- Finally, there is an option to provide an estimate for the matrix sparsity (a value in [0,1], with 0 corresponding to a fully dense matrix). When the sparsity pattern learner is disabled, this value is used if/when required to reserve space for matrix indices and non-zeros. See `ChSolverDirect::SetSparsityEstimate`.

3. If appropriate and warranted by the problem setup, it is *highly recommended* to enable the sparsity pattern lock. This can significantly improve performance for more complex problems (larger size and/or problems which include constraints).

## Release 4.0.0 - 2019-02-22

In addition to various bug fixes and enhancements, Chrono v4.0.0 includes the following main updates:
- Adopted Google test and benchmark libraries for unit testing and performance benchmarking.
	- these are set up as Git submodules
	- unit tests relocated under src/tests/unit_tests; several unit tests were migrated to use the Google test framework
  	- examples in src/tests/benchmark_tests illustrate use of the Google benchmark framework
- Core Chrono module
	- new LinkMotor elements, for modeling linear and rotational motors
	- new parsers for generating Chrono models from ADAMS adm files and OpenSim osim files (these parsers provide partial support)
- Chrono::FEA
	- FEA support is now included in the core Chrono module and not as an optional module anymore
	- new IGA (isogeometric) beam element
	- new 8-node, high-order ANCF shell element
- Chrono::MUMPS
	- new optional module providing an interface to the MUMPS sparse direct linear solver
- Chrono::Distributed
	- new optional module for MPI_based distributed parallel simulation of granular dynamics problems
	- currently only supports SMC contact formulation (penalty approach) and a reduced number of collision shapes (spheres and triangular meshes)
- Chrono::FSI
	- support for Implicit Incompressible Smoothed Particle Hydrodynamics
	- support for fluid-solid interaction with ANCF cable/shell elements
	- new GPU-based sparse linear solvers via CUBLAS and CUSPARSE libraries
- Chrono::Python
	- Python interface to Chrono now distributed as an Anaconda package
	- new interface to the Chrono::MKL and Chrono::Cascade modules
	- several new demos, including demo for use with TensorFlow
- Chrono::Vehicle
	- new tire models (TMeasy, Pacejka89)
	- new templates for continuous-band tracks
	- new suspension templates
	- new vehicle models (generic sedan, UAZ van)
