The `vsgMrMap` directory contains a build script for a tool called vsgMrMap.

Why:

Vulkan Scene (VSG) Graph reads mapped roughness and metalness information in form of a single rgb file.
There are a lot of freely available texture sets for PBR graphics. They normally contain separate
roughness and metalness textures, these files are incompatible with VSG. `vsgMrMap` can generate
a compatible MrMap from separate texture files. Sometimes the metalness texture is missing, when
the metalness is zero (e. g. for concrete surfaces). `vsgMrMap` can also work with a roughness
texture only and converts it into the correct format.

How to:

Just call `vsgMrMap` or `vsgMrMap -h` to see the options.

Nore:

You need a c++20 compatible c++ compiler to build the tool.
