#!/bin/bash
RED='\033[0;31m'
YEL='\033[0;33m'
NC='\033[0m'

cp /home/georges/catkin_ws/src/pointgrey_camera_driver/pearllaunch/* /home/georges/catkin_ws/src/pointgrey_camera_driver/pointgrey_camera_driver/launch

sleep 10


echo "${RED}Starting cam 1 strobe...${NC}"
timeout 2 rostopic hz /camera1/image_raw

#sudo service dnsmasq restart

#sleep 20


sudo service robot stop
sleep 1
sudo service robot start
sleep 4

echo "${RED}RTK driver...${NC}"
roslaunch reach_ros_node reach_ros_node.launch &


echo "${RED}Checking cam 1${NC}"
timeout 2 rostopic hz /camera1/image_raw
echo "${RED}Checking cam 2${NC}"
timeout 1.5 rostopic hz /camera2/image_raw
echo "${RED}Checking cam 3${NC}"
timeout 1.5 rostopic hz /camera3/image_raw
echo "${RED}Checking 3D Lidar${NC}"
timeout 1 rostopic hz /rslidar_points
echo "${RED}Checking IMU${NC}"
timeout 1 rostopic hz /vectornav/IMU
echo "${RED}Checking RTK GPS${NC}"
timeout 1 rostopic hz /tcpfix

SAT="$(timeout 0.4 rostopic echo /vectornav/GPS/status/service)"
SAT="$(echo $SAT | cut -b 1)"
case $SAT in
''|*[!0-9]*) SAT=0 ;;
*)  ;;
esac

while [ $SAT -lt 5 ]

do

	echo "Got $SAT satellites"
	SAT="$(timeout 0.4 rostopic echo /vectornav/GPS/status/service)"
	SAT="$(echo $SAT | cut -b 1)"
case $SAT in
''|*[!0-9]*) SAT=0 ;;
*)  ;;
esac
	
	sleep 2
	
done


echo "${YEL}GPS Fixed...Got $SAT Satellites${NC}"

echo "Waiting for INS Filter"
 
INS="$(timeout 0.4 rostopic echo /vectornav/GPS_INS/status/status)"
INS="$(echo $INS | cut -b 1)"

case $INS in
''|*[!0-9]*) INS=0 ;;
*)  ;;
esac

while [ $INS -lt 1 ]
do
	sleep 2
	INS="$(timeout 0.4 rostopic echo /vectornav/GPS_INS/status/status)"
	INS="$(echo $INS | cut -b 1)"
case $INS in
''|*[!0-9]*) INS=0 ;;
*)  ;;
esac

done

echo "${YEL}INS Ready${NC}"


echo "${YEL}!!Recording rosbags in 5 Seconds!!${NC}"


sleep 5

rosbag record --split --size=2048 /camera1/camera_info /camera2/camera_info /camera3/camera_info /camera1/image_raw /camera2/image_raw /camera3/image_raw /rslidar_points /vectornav/GPS /vectornav/GPS_INS /vectornav/GPS_XYZ /vectornav/GPS_INS_XYZ /vectornav/IMU /Trimble /vectornav/Pres /vectornav/Mag /vectornav/Temp /tf /camera1/shutter_time /camera2/shutter_time /camera3/shutter_time /tcpfix /tcptime /tcpvel


