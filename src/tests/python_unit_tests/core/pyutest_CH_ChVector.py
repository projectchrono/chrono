import pytest
import pychrono as chrono

ABS_ERR_D = 1e-15
ABS_ERR_F = 1e-6

def test_normalize():
    ad = chrono.ChVector3d(1.1, -2.2, 3.3)
    assert ad.GetNormalized().Length() == pytest.approx(1.0, abs=ABS_ERR_D)
    assert ad.Normalize() is True
    assert ad.Length() == pytest.approx(1.0, abs=ABS_ERR_D)

    bd = chrono.ChVector3d(0.0)
    assert bd.Normalize() is False

    af = chrono.ChVector3f(1.1, -2.2, 3.3)
    assert af.GetNormalized().Length() == pytest.approx(1.0, abs=ABS_ERR_F)
    assert af.Normalize() is True
    assert af.Length() == pytest.approx(1.0, abs=ABS_ERR_F)

    bf = chrono.ChVector3f(0.0)
    assert bf.Normalize() is False


def test_dot():
    ad = chrono.ChVector3d(1.1, -2.2, 3.3)
    assert ad.Dot(ad) == pytest.approx(ad.Length2(), abs=ABS_ERR_D)
    assert ad.Dot(-ad) == pytest.approx(-ad.Length2(), abs=ABS_ERR_D)
    assert ad.Dot(ad.GetOrthogonalVector()) == pytest.approx(0.0, abs=ABS_ERR_D)

    af = chrono.ChVector3f(1.1, -2.2, 3.3)
    assert af.Dot(af) == pytest.approx(af.Length2(), abs=ABS_ERR_F)
    assert af.Dot(-af) == pytest.approx(-af.Length2(), abs=ABS_ERR_F)
    assert af.Dot(af.GetOrthogonalVector()) == pytest.approx(0.0, abs=ABS_ERR_F)


def test_cross():
    ad = chrono.ChVector3d(1.1, -2.2, 3.3)
    bd = chrono.ChVector3d(-0.5, 0.6, 0.7)
    cd = ad.Cross(bd)
    assert cd.Dot(ad) == pytest.approx(0.0, abs=ABS_ERR_D)
    assert cd.Dot(bd) == pytest.approx(0.0, abs=ABS_ERR_D)

    zd1 = ad.Cross(ad)
    assert zd1.x == pytest.approx(0.0, abs=ABS_ERR_D)
    assert zd1.y == pytest.approx(0.0, abs=ABS_ERR_D)
    assert zd1.z == pytest.approx(0.0, abs=ABS_ERR_D)

    z2 = ad.Cross(-ad)
    assert z2.x == pytest.approx(0.0, abs=ABS_ERR_D)
    assert z2.y == pytest.approx(0.0, abs=ABS_ERR_D)
    assert z2.z == pytest.approx(0.0, abs=ABS_ERR_D)

    pd = ad.GetOrthogonalVector()
    assert ad.Cross(pd).Length() == pytest.approx(ad.Length() * pd.Length(), abs=ABS_ERR_D)

    af = chrono.ChVector3f(1.1, -2.2, 3.3)
    bf = chrono.ChVector3f(-0.5, 0.6, 0.7)
    cf = af.Cross(bf)
    assert cf.Dot(af) == pytest.approx(0.0, abs=ABS_ERR_F)
    assert cf.Dot(bf) == pytest.approx(0.0, abs=ABS_ERR_F)

    zd1 = af.Cross(af)
    assert zd1.x == pytest.approx(0.0, abs=ABS_ERR_F)
    assert zd1.y == pytest.approx(0.0, abs=ABS_ERR_F)
    assert zd1.z == pytest.approx(0.0, abs=ABS_ERR_F)

    z2 = af.Cross(-af)
    assert z2.x == pytest.approx(0.0, abs=ABS_ERR_F)
    assert z2.y == pytest.approx(0.0, abs=ABS_ERR_F)
    assert z2.z == pytest.approx(0.0, abs=ABS_ERR_F)

    pf = af.GetOrthogonalVector()
    assert af.Cross(pf).Length() == pytest.approx(af.Length() * pf.Length(), abs=ABS_ERR_F)