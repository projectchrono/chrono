### Project CHRONO _feature/multiarch_

This branch introduces changes which aim to make Chrono behave more predictably when building for
a greater variety of platforms and architectures. 

Chrono with _feature/multiarch_ includes existing support for 64-bit x86 CPUs and has also been known to
build on AArch64 and POWER8/9 under Linux.

##### Additional Features
- Some x86 compilers which are not able to build Chrono releases or the develop branch may work with _feature/multiarch_
- AArch64 experimental support includes some integration of NEON vectorization instructions
- POWER 8/9 experimental support has been tested for GCC 7 or newer and XLC 16.1 and newer

##### Previously Tested Architectures and Compilers
- x86_64 / GCC 6 and newer
- x86_64 / Clang 6.0.0 and newer
- x86_64 / MSVC 14 and newer
- AArch64 / GCC 7 and newer
- POWER / GCC 7
- POWER / XLC 16.1._x_



##### **DISCLAIMER**

Chrono is primarily an x86-based software. While _feature/multiarch_ may enable the use of Chrono
on a wider variety of platforms, these additional platforms are not officially supported by the developers. 
To be absolutely clear; the developers will not provide online support for these platforms.
Any additional platforms or architectures mentioned by the developers in any documentation have only been through a
minimum of testing and come with absolutely no guarantees as to their performance or validity. 
_Caveat emptor_, your mileage may vary, etc.
