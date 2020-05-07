#!/usr/bin/expect -f

set ip [lindex $argv 0]
set sourceDir [lindex $argv 1]
set destDir [lindex $argv 2]

spawn scp -r $sourceDir pi@$ip:$destDir
expect "assword"
exp_send "!clamour\r"

interact
