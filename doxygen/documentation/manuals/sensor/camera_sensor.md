Camera Sensor Model {#camera_sensor}
=================================

\tableofcontents

In Chrono:Sensor:ChCameraSensor, the synthetic data is generated via GPU-based ray-tracing. By leveraging hardware accelerated support and the headless rendering capabilities provided by NVIDIA Optix Library.

## Camera sensor Setup

~~~{.cpp}
chrono::ChFrame<double> offset_pose({10, 2, .5},                           // Position
                                     Q_from_AngAxis(CH_C_PI, {0, 0, 1}));  // Rotation

auto Camera = chrono_types::make_shared<ChCameraSensor>(
                    parent_body,                // body camera is attached to
                    update_rate,                // update rate in Hz
                    offset_pose,                // offset pose
                    image_width,                // image width
                    image_height,               // image height
                    fov,                        // camera's horizontal field of view
                    alias_factor,               // supersample factor for antialiasing
                    lens_model,                 // lens model for optional distortion
                    use_global_illumination,    // optional for enabling global illumination on camera
                    gamma,                      // optionally set the gamma correction exponent (defaults to 2.2)
                    use_fog                     // optionally enable fog for this camera
                    );

Camera->SetName("Camera Sensor");
Camera->SetLag(lag);
Camera->SetCollectionWindow(exposure_time);

// Sensor data access filter
Camera->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

// Add sensor to manager
manager->AddSensor(Camera);
~~~

See [ChCameraSensor](@ref chrono::sensor::ChCameraSensor) for more details.

The camera setup process will automatically add the [Optix Render Filter](@ref chrono::sensor::ChFilterOptixRender)
to the filter list

If the camera supersample_factor is greater than 1, the setup process will adjust the resolution and append the
[Image Alias Filter](@ref chrono::sensor::ChFilterImgAlias) to the filter list.

## Rendering steps

The Camera sensor in Chrono::sensor uses Optix as the render engine. For each pixel, the engine will shoot out a ray at that direction and find the first object intersects with the ray. By default, the engine uses the physically based BRDF shader for rendering objects. It will spawn additional rays for shadows, reflection, and refraction in a recursive fashion.

The camera update frequency is much slower than the physics. Therefore the [ChOptixEngine](@ref chrono::sensor::ChOptixEngine) spawns a thread to perform the
rendering, and it will not block the main thread

### Each Update (main thread)
1. Check if there is any camera need to be updated. If there is such camera
    - Update the scene information
    - Push it into render queue
2. Check if any camera should have the data ready to ship, or wait until they finish.
3. Continue to the next time step

### Rendering thread
1. Wait until there is a camera in the render queue
2. Update those camera, clear the render queue, go back to step 1

## Filter Graphs

Any number of filters can be append to the list and modify the final result. The filters are executed as the order in filter list. Here are some examples.

* [Camera Noise](@ref chrono::sensor::ChFilterCameraNoiseConstNormal)
* [Image Alias Filter](@ref chrono::sensor::ChFilterImgAlias)
* [Transfer to Gray scale](@ref chrono::sensor::ChFilterGrayscale)
* [Save Result](@ref chrono::sensor::ChFilterSave)
* [Visualize Result](@ref chrono::sensor::ChFilterVisualize)

## Camera animation
The position and rotation of the camera can be easily changed using `SetOffsetPose` during simulation
~~~{.cpp}
Camera->SetOffsetPose(chrono::ChFrame<double>({8, 2, .5},    // Position
                      Q_from_AngAxis(CH_C_PI, {0, 0, 1})));  // Rotation
~~~

## Data access

Data will be ready after the lag time. To access
~~~{.cpp}
RGBA_ptr = Camera->GetMostRecentBuffer<UserRGBA8BufferPtr>();
if (RGBA_ptr->Buffer) {
    unsigned int height = RGBA_ptr->Height;
    unsigned int width = RGBA_ptr->Width;
    PixelRGBA8 pixel_at_100_100 = RGBA_ptr->Buffer[100 * width + 100];
    uint8_t red_channel_at_100_100 = unsigned(pixel_at_100_100.R);
}
~~~
