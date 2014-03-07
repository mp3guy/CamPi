#!/bin/bash
#
# Bash version using built in tools
#
for (( ; ; ))
do
	raspiyuv -w 160 -h 120 -o img%00d.yuv -t 1000 -tl 333 -n
 	val=$(cmp img2.yuv img3.yuv -l | awk 'function abs(x){return ((x < 0) ? -x : x)} { sum += (abs($2 - $3)) } END { print sum }')
	if [ $val -gt 150000 ]
	then
		echo "Recording"
		file=$(date +"%F-%T")
		raspivid -o $file.h264 -t 5000 -n
	fi
done
