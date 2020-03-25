#!/usr/bin/expect -f

set ip [lindex $argv 0]

spawn scp -r /home/jun/projects/clamour/src pi@192.168.4.$ip:/home/pi/yanjuntest/
expect "assword"
exp_send "!clamour\r"	

interact
