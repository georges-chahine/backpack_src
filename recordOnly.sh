#!/bin/bash


#cp /home/georges/catkin_ws/src/pointgrey_camera_driver/outdoorlaunch/* /home/georges/catkin_ws/src/pointgrey_camera_driver/pointgrey_camera_driver/launch


SAT="$(timeout 0.4 rostopic echo /vectornav/GPS/status/service)"
SAT="$(echo $SAT | cut -b 1)"
case $SAT in
''|*[!0-9]*) SAT=0 ;;
*)  ;;
esac

while [ $SAT -lt 4 ]

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

rosbag record --split --size=2048 /camera1/camera_info /camera2/camera_info /camera3/camera_info /camera1/image_raw /camera2/image_raw /camera3/image_raw /rslidar_points /vectornav/GPS /vectornav/GPS_INS /vectornav/GPS_XYZ /vectornav/GPS_INS_XYZ /vectornav/IMU /vectornav/Pres /vectornav/Mag /vectornav/Temp /tf /camera1/shutter_time /camera2/shutter_time /camera3/shutter_time
