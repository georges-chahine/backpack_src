# backpack_src

This is a backup repo for a wearable sensor suite. 

Some of the third party drivers are owned by their respective authors. I claim no ownership of the following:
The vectornav driver, the pointgrey camera driver, the RS bPearl driver, the reach RTK ROS node, the robot upstart package.

-------------------

-Make sure the path in line 6 is correct in recordPearl.sh.

To start recording, run:

./recordPearl.sh

You need at least 5 satellites to start recording. The robot upstart service should be installed. To debug the service:

sudo service robot status

To restart the drivers:

sudo service robot stop

sudo service robot start

OR

sudo service robot restart

--------------------
If the robot upstart package does not work try the following 2 tasks:

first task:

/usr/sbin/robot-start ---> edit and add export HOME=/home/georges  #replace georges with username

second task:

sudo gedit /etc/udev/rules.d/my-usbaccess.rules

Write the following into the file

KERNEL=="ttyUSB0", MODE="0666" into the file

Save the file and reboot.
