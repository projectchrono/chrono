import pytest
import pychrono as chrono
import pychrono.sensor as sens

GPS_TEST_EPSILLON = 1e-9

def test_gps_conversion():
    print("Start GPS unit test")

    # create some test coordinates are reference locations for representative coverage of the globe
    test_coords = [
        chrono.ChVector3d(0, 0, 0), 
        chrono.ChVector3d(-100, -100, -100), 
        chrono.ChVector3d(100, 100, 100)
    ]
    
    test_refs = [
        chrono.ChVector3d(0, 0, 0), 
        chrono.ChVector3d(-75, 70, 500), 
        chrono.ChVector3d(20, -60, -200)
    ]

    print(f"Testing {len(test_coords)} coord sets")

    # make sure every pair of coordinate-reference locations is handled by the conversion in an invertible way
    for coord in test_coords:
        coord_copy = chrono.ChVector3d(coord)
        coord_const = chrono.ChVector3d(coord)
        
        for ref in test_refs:
            sens.Cartesian2GPS(coord_copy, ref)
            sens.GPS2Cartesian(coord_copy, ref)

            # Assert that the transformations are inverses within the specified epsilon
            assert coord_copy.x == pytest.approx(coord_const.x, abs=GPS_TEST_EPSILLON)
            assert coord_copy.y == pytest.approx(coord_const.y, abs=GPS_TEST_EPSILLON)
            assert coord_copy.z == pytest.approx(coord_const.z, abs=GPS_TEST_EPSILLON)
