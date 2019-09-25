#!/bin/bash

executable=/home/raid/chrono-dev/build/bin/demo_GRAN_BoxSettlNoFric_SMC

frac=0.75
nparts=(10000 20000 40000 80000 100000 200000 400000 800000 1000000 2000000 4000000 8000000 10000000 20000000 40000000 80000000 100000000 200000000 250000000 300000000 350000000 400000000 450000000 500000000)
sz=400

mkdir bak
mv sizes.txt times.txt scaling_*.txt bak

for ((i=0;i<${#nparts[@]};++i)); do
	n=${nparts[$i]}
	ss=$( echo "sqrt($n / (.76 * $sz))" | bc )
	sx=$ss
	sy=$ss
	echo $executable --boxlength=$sx --boxdepth=$sy --boxheight=$sz --write_mode none -e .1 -br .5 --cohes_ratio 10
	echo n is $n
	/usr/bin/time -o times.txt -a -f "%e" $executable --boxlength=$sx --boxdepth=$sy --boxheight=$sz --write_mode none -e .1 -br .5 --cohes_ratio 10 &> scaling_$n.txt 
	echo $n >> sizes.txt
done
