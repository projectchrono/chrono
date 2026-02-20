import pytest
import numpy as np
import pychrono as chrono
import pychrono.sensor as sens

def test_data_access():
    sys = chrono.ChSystemNSC()

    anchor = chrono.ChBody()
    anchor.SetFixed(True)
    sys.Add(anchor)

    manager = sens.ChSensorManager(sys)

    bg = sens.Background()
    bg.mode = sens.BackgroundMode_SOLID_COLOR
    bg.color_zenith  = chrono.ChVector3f(1.0, 0.0, 1.0)
    bg.color_horizon = chrono.ChVector3f(1.0, 0.0, 1.0)
    manager.scene.SetBackground(bg)

    W = 64
    H = 48

    cam = sens.ChCameraSensor(anchor,
                              30.0,
                              chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                              W, H,
                              chrono.CH_PI / 3,
                              1, 
                              sens.PINHOLE, # CameraLensModelType::PINHOLE
                              False, # use_diffuse_reflect
                              False, # use_denoiser
                              sens.Integrator_LEGACY, # integrator
                              2.2, # gamma
                              False # use_fog
                              )

    cam.SetLag(0)
    cam.SetCollectionWindow(0)

    cam.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(cam);

    rgba = None
    step = 0.01

    for i in range(0, 200):
        manager.Update()
        sys.DoStepDynamics(step)

        rgba = cam.GetMostRecentRGBA8Buffer();
        if rgba and rgba.Buffer:
            break;

    assert rgba is not None
    assert rgba.Buffer is not None
    assert rgba.Width == W
    assert rgba.Height == H

    expR = 255;
    expG = 0;
    expB = 255;

    mismatch = 0;
    data = rgba.GetRGBA8Data() # It should be a numpy array

    assert isinstance(data, np.ndarray) # This is specific to Python binding
    assert data.shape == (H, W, 4)  # This is specific to Python binding

    for y in range(H):
        for x in range(W):
            p = data[y, x]
            if not (p[0] == expR and p[1] == expG and p[2] == expB):
                mismatch += 1

    assert mismatch == 0 
