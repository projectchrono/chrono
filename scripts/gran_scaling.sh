#!/bin/bash

executable=/home/raid/chrono-dev/build/bin/demo_GRAN_BoxSettlNoFric_SMC
xsize=(20 40 80 100 200 300 400 500 600 700 800 900 1000 1200 1400 1600 1800 2000)

for size in "${xsize[@]}"; do
	echo $size
	echo $executable --boxlength=$size --boxdepth=100 --boxheight=100 
	/usr/bin/time -o times.txt -a -f "%e" $executable --boxlength=$size --boxdepth=100 --boxheight=100 &> scaling_$size.txt 
done
