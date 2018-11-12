#!/bin/bash
deviceIp=$(ifconfig | grep 172 | awk '{print $2;}')
echo "GeräteIP: $deviceIp"
#Pruefen ob erstes Imputargument leer ist
if [ -z "$1" ]
then
	#Falls kein Inputargument uebergeben wurde
	echo "IP DES AUDOS NICHT ANGEGEBEN!"
else
	#Setzt die URI des Audos zusammen
	audoURI="http://$1:11311"
	echo "aduoURI:  $audoURI"
	#Fuert die Befehle aus
	export ROS_IP=$deviceIp
	echo "export ROS_IP=$deviceIp"
	export ROS_MASTER_URI=$audoURI
	echo "export ROS_MASTER_URI=$audoURI"
fi
