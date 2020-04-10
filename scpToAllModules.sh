#!/bin/bash

IPS=(6 8 11 27 28 34 36 37)

for i in ${IPS[@]}; do
	expect /home/jun/projects/clamour/sendTo.sh $i
done
