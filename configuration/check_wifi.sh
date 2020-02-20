#!/bin/bash
#=================================================================
# Script Variables Settings
clear
wlan='wlan0'
gateway='192.168.1.254'
#alias ifup='/sbin/ip link set wlan0 up'
#alias ifdown='/sbin/ip link set wlan0 down'
alias ifconfig='/sbin/ifconfig'
#=================================================================
date
echo " - Auto Reconnect Wi-Fi Status for $wlan Script Started ";
echo

# Only send two pings, sending output to /dev/null as we don't want to fill logs on our sd card. 
# If you want to force ping from your wlan0 you can connect next line and uncomment second line 
ping -c2 ${gateway} > /dev/null # ping to gateway from Wi-Fi or from Ethernet
# ping -I ${wlan} -c2 ${gateway} > /dev/null # only ping through Wi-Fi 

# If the return code from ping ($?) is not 0 (meaning there was an error)
if [ $? != 0 ]
then
    # Restart the wireless interface
    ifconfig wlan0 down
    ifconfig wlan0 up
	sleep 5
	ifconfig wlan0 up
fi
ping -I ${wlan} -c2 ${gateway} > /dev/null
date
echo 
echo " - Auto Reconnect Wi-Fi Status for $wlan Script Ended ";
