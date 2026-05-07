# This is a PyChrono only test to test numpy integration

import pytest
import pychrono as chrono
import numpy as np

def test_ChVector3():
    v1 = chrono.ChVector3d(1.0, 2.0, 3.0)
    v1_np = v1.to_numpy()
    assert np.allclose(v1_np, [1.0, 2.0, 3.0])
    assert v1_np.dtype == np.float64
    assert v1_np.shape == (3,)

def test_ChMatrix33():
    m1_np = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
    m1 = chrono.ChMatrix33d(m1_np)
    m1_np_n = m1.to_numpy()
    assert np.allclose(m1_np_n, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
    assert m1_np_n.dtype == np.float64
    assert m1_np_n.shape == (3, 3)
    assert m1[2, 1] == pytest.approx(8.0, abs = 1e-10)

def test_ChMatrixDynamic():
    m1_np = np.random.rand(4, 5)
    m1 = chrono.ChMatrixDynamicd(m1_np)
    m1_np_n = m1.to_numpy()
    assert np.allclose(m1_np_n, m1_np)
    assert m1_np_n.dtype == np.float64
    assert m1_np_n.shape == (4, 5)
    assert m1[3, 4] == pytest.approx(m1_np[3, 4], abs = 1e-10)

def test_ChQuaternion():
    q1 = chrono.ChQuaterniond(1.0, 2.0, 3.0, 4.0)
    q1_np = q1.to_numpy()
    assert np.allclose(q1_np, [1.0, 2.0, 3.0, 4.0])
    assert q1_np.dtype == np.float64
    assert q1_np.shape == (4,)