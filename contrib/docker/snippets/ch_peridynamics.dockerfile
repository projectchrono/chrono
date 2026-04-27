# SPDX-License-Identifier: MIT
# This snippet enables Chrono::Peridynamics (fracture / damage mechanics).
#
# Note on Peridynamics:
# The generated API documentation for the PERIDYNAMICS module mentions an
# installation guide, but the corresponding Doxygen reference
# module_peridynamics_installation does not correspond to a separate page in
# this source tree. The public API page therefore documents the module itself,
# not a separate dependency/install procedure.
#
# Public API page:
#   https://api.projectchrono.org/group__chrono__peridynamics.html
#
# This snippet follows the current Chrono CMake implementation:
#
#   - src/chrono_peridynamics/ChApiPeridynamics.h:
#       * defines the PERIDYNAMICS module API group
#       * references module_peridynamics_installation from the generated docs
#   - src/chrono_peridynamics/CMakeLists.txt:
#       * enables the module with CH_ENABLE_MODULE_PERIDYNAMICS
#       * builds Chrono_peridynamics from in-tree sources
#       * links only against Chrono_core
#   - src/demos/peridynamics/CMakeLists.txt:
#       * links the demos against Chrono_peridynamics
#
# No external build-time dependencies are required beyond Chrono core.

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_PERIDYNAMICS=ON"
