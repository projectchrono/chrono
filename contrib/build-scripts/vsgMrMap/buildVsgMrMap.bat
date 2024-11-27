rmdir /S/Q build
cmake -B build -S .
cmake --build build --config Release
echo "Executable build\Release\vsgMrMap has been built."
