Tutorials {#tutorial_root}
==========================

<div class="ce-info">
**Dynamic objects of classes with fixed-size vectorizable Eigen object members**<br>
<ul>
<li>Many of the Chrono classes now have members that are fixed-size vectorizable Eigen types. These classes overload their `operator new` to generate 16-byte-aligned pointers (using an Eigen-provided macro).</li>
<li>This takes care of situations where one must dynamically create objects of such classes; for more details, see the [Eigen documentation](https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html).</li>
<li>If you need to create STL containers of such classes, you should use a custom allocator that always allocates aligned memory (such as the Eigen-provided `Eigen:aligned_allocator`); for more details, see the [Eigen documentation](https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html).</li>
<li>Finally, this requirement for aligned memory allocation has implications on creation of shared pointers.  Indeed, `std::make_shared` uses `placement new` instead of `operator new`.  To address this issue and preserve encapsulation (as much as possible), Chrono provides custom replacement functions for `make_shared`, available in the `chrono_types` namespace. These functions will automatically infer if they can safely fallback on `std::make_shared` or else create a shared pointer with an alternative mechanism that ensures use of aligned memory. <br>
As such, user code should **always** use `chrono_types::make_shared` as in
~~~{.cpp}
auto my_body = chrono_types::make_shared<ChBody>();
~~~
</li>
</ul>
</div>

## Chrono modules

-   @subpage tutorial_table_of_content_chrono

    Examples of the core features and capabilities of the Chrono library.

-   @subpage tutorial_table_of_content_chrono_irrlicht

    Examples using run-time visualization with the Irrlicht module.

-   @subpage tutorial_table_of_content_chrono_mbs

    Examples of MBS dynamics in Chrono.

-   @subpage tutorial_table_of_content_chrono_fea

    Examples of FEA in Chrono.

-   @subpage tutorial_table_of_content_chrono_vehicle

    Examples of modeling and simulating ground vehicles in Chrono.

-   @subpage tutorial_table_of_content_chrono_fsi

    Examples of fluid-solid interaction problems using the FSI module.

-   @subpage tutorial_table_of_content_chrono_postprocess

    Examples of producing postprocessing data (e.g. POVray or GNUplot output).

-   @subpage tutorial_table_of_content_chrono_multicore

    Examples of using the Multicore module.

-   @subpage tutorial_table_of_content_chrono_gpu

    Examples of granular dynamics problems using the GPU module.

-   @subpage tutorial_table_of_content_chrono_python

    Examples of parsing Python programs.

-   @subpage tutorial_table_of_content_chrono_matlab

    Examples of Matlab inter-operation.

-   @subpage tutorial_table_of_content_chrono_cosimulation

    Examples of cosimulation with Simulink.

-   @subpage tutorial_table_of_content_chrono_cascade

    Examples of loading CAD models.

-   @subpage tutorial_table_of_content_chrono_opengl

    Examples of run-time visualization with the OpenGL module.

-   @subpage tutorial_table_of_content_chrono_sensor

    Examples of modeling and simulating sensor for robots and autonomous vehicles in Chrono.

-   @subpage tutorial_table_of_content_chrono_synchrono

    Examples of distributed simulation of autonomous vehicles and robots.

-   @subpage tutorial_table_of_content_chrono_fmi

    Examples of generating (exporting) and using (importing) Chrono FMUs.


## Other tools


-   @subpage tutorial_table_of_content_pychrono

    Learn how to use [PyChrono](@ref pychrono_introduction)

-   @subpage tutorial_table_of_content_chrono_solidworks

    Learn how to use [Chrono::SolidWorks](@ref manual_chrono_solidworks)


## Chrono training materials

-   [Tutorial slides](@ref tutorial_slides_300)

    Set of tutorial slides for Chrono release 3.0.0


## Documentation guides

-    @subpage tutorial_table_of_content_documentation

     Guidelines on writing technical documentation for Chrono (for developers of new modules/features).
