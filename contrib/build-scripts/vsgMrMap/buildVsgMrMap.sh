rm -rf build
cmake -B build -S .
cmake --build build
if [ -x build/vsgMrMap ] ; then
	echo "Executable: build/vsgMrMap has been built." 
fi