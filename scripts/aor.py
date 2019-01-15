#!/usr/bin/env python
import numpy as np

CONVERSION_FACTOR = 180 / np.pi

a = np.array([-49.67650007679626, 0, -71.92005975475116])
b = np.array([-7.5790494921838345, 0, -57.78888157778205])

delta = b - a

angle = np.arctan(delta[2] / delta[0]) * CONVERSION_FACTOR
print("for points ", a, " and ", b, " aor is ", angle)