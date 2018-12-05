#!/bin/bash
#grep filters for strings with "172"
#akw takes the second string of the return value of grep
audoIp=$(ifconfig | grep 172 | awk '{print $2;}')
#Check if IP is not empty
if [ -z audoIp ]
then
	echo "Audo not connected to eduroam or IP changed drasticly!"
	echo "Check:"
	ifconfig
else
	#Concats the string for the URI
	audoURI="http://$audoIp:11311"
	echo "aduoURI:  $audoURI"

	#Executes the instructions
	export ROS_IP=$audoIp
	echo "export ROS_IP=$audoIp"
	export ROS_MASTER_URI=$audoURI
	echo "export ROS_MASTER_URI=$audoURI"

	echo "Launch ucbridge"
	gnome-terminal -- roslaunch pses_ucbridge ucbridge.launch
	sleep 5

	echo "Launch kinect drivers"
	gnome-terminal -- roslaunch kinect2_bridge kinect2_bridge.launch
	sleep 5

	#may be deleted later
	echo "Launch sub for keeping kinect alive"
	gnome-terminal -- rosrun listener listener_node
fi
