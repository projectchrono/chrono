#!/bin/bash
id=$1
((n1=id/25))
((n2=(id-25*n1)/5))
((n3=(id-25*n1-5*n2)))

densities=(1 2 3 4 5)
gravs=(180 380 580 780 980)
diams=(2 4 6 8 10 15 20 25 30 35 40)

diam=${diams[n1]}
density=${densities[n2]}
grav=${gravs[n3]}

echo $grav, $density, $diam