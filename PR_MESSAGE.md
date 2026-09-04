# [UPDATE] Improve SCM memory use and long-run performance

**Summary**

This PR improves SCM performance and memory use, especially for large terrain patches and long simulations.

- Store SCM height and node data in single precision. This cuts the memory used by these fields roughly in half and allows larger or finer terrain grids.
- Use wider counters when creating the SCM visualization mesh. This prevents integer overflow on very large grids and reports a clear error when the mesh index limit is exceeded.
- Store deformed nodes in 16 x 16 tiles. This keeps terrain lookups fast as the vehicle covers more ground and prevents long runs from gradually slowing down.
- Add active-domain and deformed-node counters, along with a way to remove active domains. This makes it easier to monitor SCM work and stop processing bodies that no longer need terrain interaction.
- Limit rendering in affected demos to 60 frames per simulated second. This avoids unnecessary rendering work and makes reported simulation performance more meaningful.

**Related Issue(s)**

None.

**Author(s)**

Keshav Sharan

**Licensing**

By submitting this pull request, I agree that my contribution will be included in Chrono and redistributed under the BSD-3-Clause License.

**Backward Compatibility**

There are no input or public API breaks. Public SCM values remain double precision. Small numerical differences are possible because internal terrain storage now uses single precision.

**Implementation Notes**

The changes were tested with the Curiosity SCM demo in the `chrono-orb` container.

**Post Submission Checklist**

- [x] The changes are complete
- [x] The changes build with CMake
- [x] The SCM demo was tested
