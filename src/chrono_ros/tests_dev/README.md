# chrono_ros dev-time tests

Fast inner-loop tests for the dependency-free parts of Chrono::ROS (the
`core/` static library) plus a stub-header syntax check of the simulation-side
API sources. **No ROS and no Chrono installation is required** — that is the
point: the protocol, CDR codec, schema machinery, and shm transport are fully
verifiable in isolation.

```bash
./run_dev_tests.sh          # strict warnings + ASan/UBSan, builds & runs all suites
```

Suites:

| file | covers |
|---|---|
| `test_cdr.cpp` | wire-format golden bytes (alignment, encapsulation, strings), scalar round trips, malformed-input rejection |
| `test_schema.cpp` | schema model invariants, blob round trip, version checks |
| `test_message.cpp` | schema-driven serialization: golden bytes, Image/PointCloud2/JointState-shaped round trips, defaults, error messages |
| `test_transport.cpp` | SPSC ring (wraparound, atomic frames, threaded stress), shm channel round trips, capacity diagnostics, control payload codecs |

`.stubs/` contains three tiny header stubs (`ChApiEXPORT` etc.) used only by
the syntax-check stage; they are never installed or used by any real build.

What this does NOT cover (needs ROS, runs in CI/Docker): the `node/` sources
(introspection walker, generic pub/sub) and end-to-end bridge behavior — the
VALIDATE_TYPE handshake is the authoritative cross-check of the CDR codec
against the real rmw, and runs automatically at every `Initialize()`.

These tests are intended to also be registered with Chrono's CTest harness
once the module CMake stabilizes (CLAUDE.md, Phase 6).
