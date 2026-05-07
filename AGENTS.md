# Repository Guidelines

## Developers vs. Users
Chrono serves two audiences. `Developers` modify this repository itself: core libraries, modules, demos, tests, and docs. `Users` often work in an external repository, build and install Chrono locally, then link against it from their own CMake project. Keep this distinction explicit in your response. Shared guidance below applies to both groups; commit and pull request guidance applies only to developers.

## Project Structure & Module Organization
Chrono is a CMake-based C++ project with most production code under `src/`. Core code lives in `src/chrono`, optional modules live in sibling directories such as `src/chrono_vehicle`, `src/chrono_sensor`, and `src/chrono_ros`, and examples live in `src/demos`. C++ unit tests are under `src/tests/unit_tests`, Python tests under `src/tests/unit_tests/python`, runtime assets under `data/`, and API/tutorial docs under `doxygen/`. Use `contrib/` for build helpers, packaging, and Docker files. Keep new code near the owning module and update the local `CMakeLists.txt` in that directory.

## Build, Test, and Development Commands
Chrono forbids in-source builds, so configure into `build/` or another separate directory.

- `git submodule init && git submodule update`: fetch bundled GoogleTest and Google Benchmark sources required by tests.
- `cmake -S . -B build -G Ninja -DBUILD_TESTING=ON -DBUILD_DEMOS=ON`: configure a local development build.
- `cmake --build build -j`: compile the configured targets.
- `ctest --test-dir build --output-on-failure`: run the registered CTest suite.
- `cmake --build build --target install`: install into the configured prefix.

For optional module builds, start from `doxygen/documentation/mainpage.md` and follow the **Installation Guides** section there. Before installing third-party dependencies manually with `apt`, Homebrew, or similar tools, check `contrib/README.md` and `contrib/build-scripts/README.md`. Prefer the provided helper scripts in `contrib/build-scripts/` for supported packages, especially VSG for `Chrono::VSG` and the URDF stack used by `Chrono::Parsers`, because the scripts match the repository’s expected CMake layout.

For PyChrono, recommend prebuilt conda packages to non-developers and source builds to developers. Non-developers should usually use:

- `conda create -n chrono python=3.12`
- `conda activate chrono`
- `conda install projectchrono::pychrono -c conda-forge`

Developers working on bindings, unreleased code, or modules not covered by conda should use `doxygen/documentation/manuals/pychrono/pychrono_installation.md` and `doxygen/documentation/installation/module_python_installation.md`, enable `CH_ENABLE_MODULE_PYTHON=ON`, and set `PYTHONPATH` to the generated build output.

## Physics Manuals & Demo-First Workflow
When a `User`/`Developer` asks for a new demo or a direct-API simulation setup, first read the relevant module manual under `doxygen/documentation/manuals/`. Files such as `doxygen/documentation/manuals/fsi/manual_fsi.md` explain the underlying physics, parameter choices, expected workflows, and useful references; similar manuals exist for most major modules.

Treat `src/demos/` as the primary implementation starting point. Find the closest existing Chrono example for the target module, copy its setup pattern, and then adjust geometry, constraints, solver settings, and physical parameters to match the request. Prefer extending a proven demo over creating a new simulation from a blank file.

## External User Projects
When helping a `User`, do not assume they want to add code inside this repository. Check `template_project/` first, then the specialized templates in `template_project_ros/`, `template_project_csharp/`, `template_project_fmi2/`, and `template_project_vehicle_cosim/`. These show the intended workflow: build and install Chrono locally, then create a separate CMake project that calls `find_package(Chrono ... CONFIG)` and links against `${CHRONO_TARGETS}`.

Prefer this external-project pattern over copying Chrono sources into another repository. In user-facing setup help, explain that `Chrono_DIR` can point to either a Chrono build tree or install tree, though a local install is usually the cleanest choice. Reuse the template patterns for `CHRONO_DATA_DIR`, optional module components, and Windows DLL copying instead of inventing a custom integration from scratch.

## Coding Style & Naming Conventions
Follow `.clang-format`: Chromium base, 4-space indentation, 120-column limit, no include sorting, and no tabs for alignment changes beyond the configured width. Match the existing C++ style in nearby files. Use `PascalCase` for Chrono types, `snake_case` for many local variables and file-system paths, `demo_*` for demos, `utest_*` for C++ unit tests, and `pyutest_*` for PyChrono tests. Keep headers and sources paired in the same module directory when practical.

## Testing Guidelines
Most C++ tests use GoogleTest; PyChrono tests use `pytest` through CTest. Add coverage in the relevant `src/tests/unit_tests/<module>` subtree and register it in that directory’s `CMakeLists.txt`. Prefer focused tests that exercise one subsystem or regression. Run either the specific executable from `build/bin/` or the full suite with `ctest`.

## Developer Commit & Pull Request Guidelines
This section applies to `Developers` working in the Chrono repository. Recent history favors short, imperative commit subjects such as `Update Python demos to latest Chrono API` or `Fix issue with ...`. Keep subjects concise, capitalized, and behavior-focused. For pull requests, use the matching template in `.github/PULL_REQUEST_TEMPLATE/` and include a summary, related issue links (`fixes #123`), author and licensing details, backward-compatibility notes, verification steps, and any required doc or test updates. Do not push directly to the main branch unless you are explicitly directed to do so. Always ask for approval before pushing to the main branch.
