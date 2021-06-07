Unit test and benchmark tests
==============

Most Chrono unit tests rely on the [googletest](https://github.com/google/googletest) library. 
The Chrono benchmark tests use the [googlebenchmark](https://github.com/google/benchmark) library (except the Chrono::Sensor benchmark tests).
These two supporting libraries are included in the Chrono repository as submodules.  Upon cloning the Chrono repository, make sure to run the following two commands (in the Chrono *source tree*):
```
> git submodule init
> git submodule update
```

You can run individual unit or benchmark tests from the `bin/` directory.  For example (Linux):
```
> cd <build_dir>
> cd bin
> ./utest_CH_ChVector
> ./btest_CH_pendulums
```
In Windows, executables are located in a configuration-specific subfolder of `bin/` (e.g. `Release`).

You can automate batch-running all unit tests using `ctest`.  This command must be executed from the top-level directory of the *build tree*:
```
> cd <build_dir>
> ctest
```
On Windows, you may need to specify the build configuration. For example:
```
> cd <build_dir>
> & 'C:\Program Files\CMake\bin\ctest.exe' -C Release
```

During installation, all existing unit and benchmark tests are copied to the *install tree*:
- `<install_dir>/share/chrono/bin` on Linux
- `<install_dir>/bin` on Windows

These installed tests can be executed individually from the above locations, but no mechanism for batch-running all unit tests is provided.
