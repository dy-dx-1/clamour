#!/bin/bash

projDir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
sourceDir=$projDir/src
destDir="/home/pi/yanjuntest/"
echo $projDir
echo $sourceDir

IPs=(6 8 12 17 28 37) # The last value of ip for the modules want to transfer files to.

for i in ${IPs[@]}; do
	expect $projDir/sendTo.sh 192.168.4.$i $sourceDir $destDir
done
