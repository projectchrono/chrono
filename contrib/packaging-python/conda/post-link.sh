export PY_VER=`python -c 'import sys; version=sys.version_info[:3]; print("{0}.{1}".format(*version))'`
echo 'SetSensorShaderDir('\'$PREFIX'/lib/sensor_ptx/'\'')' >> $PREFIX/lib/python$PY_VER/site-packages/pychrono/sensor.py
