#!/bin/bash

IPS=(6 8 12 17 28 37)

for i in ${IPS[@]}; do
	expect /home/jun/projects/clamour/sendTo.sh $i
done
