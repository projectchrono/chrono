# PyChrono Sensor Module

## Extra/special dependencies
 - numpy

## Reasons why the sensor+python interface was setup like it was

#### Numpy
here is wanted to use numpy and what that gives us performance wise
 - cmake point to numpy include directory
    - an additiona cmake flag was added when sensor and python are both enabled. The cmake flag is called "NUMPY_INCLUDE_DIR" and should be pointed to the include directory of numpy. The user will have to do this manually, as there is no clear way to find the directly. (TODO: maybe change with "because findPYTHON::Numpy would require CMake >= 3.14"?)
 - data wrapping (typemap, array shape, Argout View Arrays)
    - typemap: Numpy uses SWIG typemaps to wrap arrays. SWIG typemaps look for functions with the same name and type the typemap was declared for and operates the argin and argout operations described in the typemap (in this case the typemap operations are inside numpy.i)
    - array shape
        - Cameras and Lidar: since python packages (such as opencv) image shapes are typically BxWx3 we adopted the same rule
        - GPS and IMU: using the same rule GPD and IMU data wuould have had 1x1xN shape. For this reason they are mapped to 1D array
    -  Argout View Arrays
        - numpy.i offers different ways of wrapping arrays. This in particular does not instanciate memory, but uses instead the memory allocated by the C++ code. This approach is the most efficient, and can be used since:
          - the memory is moved to the user, then only the usen detains the ownership over the array.
          - the memory does not get deleted since it does not go out of scope, due to the fact that the array is returned to the python process.

#### UserBufferHostPtr
Because swig does not support std::unique_ptr, an alternative approach was required transferring data from the render threads to the master thread on which the user requests sensor data. The final solution was to switch from unique to shared pointer for all the sensor buffers. Because these would give non-threadsafe access to underlying memory to the user, the following setup is used. The underlying filter-graph buffers are not accessible to the user, and thus can be shared_ptrs. The previously named "LockedBufferXXPtr", which is now named "UserBufferXXPtr", is a pointer to moved memory. That is the underlying buffers owndership has been moved from the filter graph's point to the UserBufferXXPtr. This operation is performed with std::move() when the user calls GetMostRecentBuffer(). This means that if the user is calling this every simulation step, they should check if the data is null before trying to use the values. In the python and C++ demos, this operation is shown.

#### Function Specialization/Extension
 - HasData()
    - To check if the data is null as mentioned in the previous section we extended we extended each UserBufferXXPtr, adding a method called "HasData". It returns True if the pointer to the data is not NULL.
 - GetMostRecentXXBuffer()
    - Since GetMostRecentBuffer is templatic, we instanced it for each type of Buffer, i. e. to get a buffer of PixelDI the user will call GetMostRecentDIBuffer.
 - GetXXData()
    - We extended each Buffer type, providing them with a GetXXData (i. e. GetDIData for Pixel DI) to wrap their data raw arrays as Numpy arrays.

#### Miscellaneous
 - user accessing device pointer (nightmare)
    - user can only access memory on the CPU. Getting a point to memory on the device was no seriously considered when setting up the interface for 2 reasons. 1) It was deemed an unlikely use case for a user to want a device pointer to sensor data when using pythong (or C++). 2) The would complicate the swig wrapping for no clear reason.
 - cudamemcpy of data directly for UserBufferHostPtr vs optix::map and memcpy
    - A couple places in the sensor module, we apply a filter that copies memory from the filter graph into host memory that will be moved into a UserBufferPtr. The can be done via mapping the optix::buffer, then calling memcpy, or can be done via cudaMemcpy from device to host directly into space that will be moved to UserBufferPtr. The cudaMemcpy was deemed faster, but this should be further analyzed, and a consistent solution should be implemented.
