/*
 * MIT License (MIT)
 *
 * Copyright (c) 2018 Dereck Wonnacott <dereck@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <iostream>

// ROS Libraries
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"
#include "geometry_msgs/Point.h"

ros::Publisher pubIMU, pubMag, pubGPS, pubGPS_INS, pubGPS_XYZ, pubGPS_INS_XYZ, pubTemp, pubPres;


// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Method declarations for future use.
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

std::string frame_id;
bool syncOnTrigger;

int main(int argc, char *argv[])
{

    // ROS node init
    ros::init(argc, argv, "vectornav");
    ros::NodeHandle n;
    pubIMU = n.advertise<sensor_msgs::Imu>("vectornav/IMU", 1000);
    pubMag = n.advertise<sensor_msgs::MagneticField>("vectornav/Mag", 1000);
    pubGPS_INS = n.advertise<sensor_msgs::NavSatFix>("vectornav/GPS_INS", 1000);
    pubGPS_XYZ = n.advertise<geometry_msgs::Point>("vectornav/GPS_XYZ", 1000);
    pubGPS_INS_XYZ = n.advertise<geometry_msgs::Point>("vectornav/GPS_INS_XYZ", 1000);
    pubGPS = n.advertise<sensor_msgs::NavSatFix>("vectornav/GPS", 1000);
    pubTemp = n.advertise<sensor_msgs::Temperature>("vectornav/Temp", 1000);
    pubPres = n.advertise<sensor_msgs::FluidPressure>("vectornav/Pres", 1000);
    n.param<bool>("syncOnTrigger", syncOnTrigger, true);
    n.param<std::string>("frame_id", frame_id, "imu");

    // Serial Port Settings
    string SensorPort;
    int SensorBaudrate;

    n.param<std::string>("serial_port", SensorPort, "/dev/ttyUSB0");
    //n.param<std::string>("serial_port", SensorPort, "/dev/ttyS5");
    n.param<int>("serial_baud", SensorBaudrate, 115200*8);

    ROS_INFO("Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

    // Create a VnSensor object and connect to sensor
    VnSensor vs;
    vs.connect(SensorPort, SensorBaudrate);

    //vs.changeBaudRate(230400);
    //vs.disconnect();
    //vs.connect(SensorPort, SensorBaudrate);

    // Query the sensor's model number.
    string mn = vs.readModelNumber();
    ROS_INFO("Model Number: %s", mn.c_str());

     //Set Data output Freq [Hz]
    int async_output_rate;
    n.param<int>("async_output_rate", async_output_rate, 200);
    vs.writeAsyncDataOutputFrequency(async_output_rate);


    // Configure binary output message
    BinaryOutputRegister bor(
                ASYNCMODE_PORT1,
                1000 / async_output_rate,  // update rate [ms]
                COMMONGROUP_TIMESTARTUP | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES | COMMONGROUP_INSSTATUS,
                TIMEGROUP_NONE,
                IMUGROUP_NONE,
                GPSGROUP_NUMSATS | GPSGROUP_POSLLA | GPSGROUP_POSECEF,
                ATTITUDEGROUP_NONE,
                INSGROUP_POSECEF);

    vs.writeBinaryOutput1(bor);

    if (syncOnTrigger) {
        vs.writeSynchronizationControl(SYNCINMODE_ASYNC, SYNCINEDGE_RISING,0,
                                       SYNCOUTMODE_NONE, SYNCOUTPOLARITY_NEGATIVE, 0, 0, false);
    }

    vs.registerAsyncPacketReceivedHandler(NULL, BinaryAsyncMessageReceived);


    // You spin me right round, baby
    // Right round like a record, baby
    // Right round round round
    ros::spin();


    // Node has been terminated
    vs.unregisterAsyncPacketReceivedHandler();
    vs.disconnect();
    return 0;
}


//
// Callback function to process data packet from sensor
//
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{

    if (p.type() == Packet::TYPE_BINARY)
    {

        // First make sure we have a binary packet type we expect since there
        // are many types of binary output types that can be configured.
        if (!p.isCompatible(
                    COMMONGROUP_TIMESTARTUP | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES | COMMONGROUP_INSSTATUS,
                    TIMEGROUP_NONE,
                    IMUGROUP_NONE,
                    GPSGROUP_NUMSATS | GPSGROUP_POSLLA | GPSGROUP_POSECEF,
                    ATTITUDEGROUP_NONE,
                    INSGROUP_POSECEF))

            // Not the type of binary packet we are expecting.
        {
            //      std::cout <<"error"<<std::endl;
            return;
        }
        // std::cout <<"passed"<<std::endl;
        // Unpack the packet
        uint64_t timeStartup = p.extractUint64();
        vec4f q = p.extractVec4f();
        vec3f ar = p.extractVec3f();
        vec3d lla = p.extractVec3d();
        vec3f al = p.extractVec3f();
        vec3f mag = p.extractVec3f();
        float temp = p.extractFloat();
        float pres = p.extractFloat();

        uint16_t InsStatus=p.extractUint16();


        unsigned int numSat=p.extractUint8();

        vec3d lla2= p.extractVec3d();
        vec3d ecef_GPS= p.extractVec3d();
        vec3d ecef_INS= p.extractVec3d();

        InsStatus = InsStatus & (0x0003); // We really only care about the first 2 bits here (INS Filter Mode)

        // Publish ROS Message

        // IMU
        sensor_msgs::Imu msgIMU;

        msgIMU.header.stamp = ros::Time::now();
       // std::cout<<msgIMU.header.stamp<<std::endl;
        msgIMU.header.frame_id = frame_id;

        msgIMU.orientation.x = q[0];
        msgIMU.orientation.y = q[1];
        msgIMU.orientation.z = q[2];
        msgIMU.orientation.w = q[3];

        msgIMU.angular_velocity.x = ar[0];
        msgIMU.angular_velocity.y = ar[1];
        msgIMU.angular_velocity.z = ar[2];

        msgIMU.linear_acceleration.x = al[0];
        msgIMU.linear_acceleration.y = al[1];
        msgIMU.linear_acceleration.z = al[2];



        pubIMU.publish(msgIMU);


        // Magnetic Field
        sensor_msgs::MagneticField msgMag;

        msgMag.header.stamp = msgIMU.header.stamp;
        msgMag.header.frame_id = msgIMU.header.frame_id;

        msgMag.magnetic_field.x = mag[0];
        msgMag.magnetic_field.y = mag[1];
        msgMag.magnetic_field.z = mag[2];

        pubMag.publish(msgMag);

        // std::cout <<"passed 1"<<std::endl;
        // GPS from INS
        sensor_msgs::NavSatFix msgGPS_INS;

        msgGPS_INS.header.stamp = msgIMU.header.stamp;
        msgGPS_INS.header.frame_id = msgIMU.header.frame_id;

        msgGPS_INS.latitude = lla[0];
        msgGPS_INS.longitude = lla[1];
        msgGPS_INS.altitude = lla[2];

        msgGPS_INS.status.status= InsStatus;

        pubGPS_INS.publish(msgGPS_INS);
      //  std::cout << "INS Status is " <<InsStatus<<std::endl;
        // GPS
        sensor_msgs::NavSatFix msgGPS;

        msgGPS.header.stamp = msgIMU.header.stamp;
        msgGPS.header.frame_id = msgIMU.header.frame_id;

        msgGPS.status.service=numSat;
        msgGPS.latitude = lla2[0];
        msgGPS.longitude = lla2[1];
        msgGPS.altitude = lla2[2];

        pubGPS.publish(msgGPS);
        //  std::cout <<"passed 3"<<std::endl;
        // GPS in XYZ
        geometry_msgs::Point msgGPS_XYZ;

        msgGPS_XYZ.x=ecef_GPS[0];
        msgGPS_XYZ.y=ecef_GPS[1];
        msgGPS_XYZ.z=ecef_GPS[2];

        pubGPS_XYZ.publish(msgGPS_XYZ);
        // GPS

        // GPS in IMU XYZ
        geometry_msgs::Point msgGPS_IMU_XYZ;

        msgGPS_IMU_XYZ.x=ecef_INS[0];
        msgGPS_IMU_XYZ.y=ecef_INS[1];
        msgGPS_IMU_XYZ.z=ecef_INS[2];

        pubGPS_INS_XYZ.publish(msgGPS_IMU_XYZ);


        // Temperature
        sensor_msgs::Temperature msgTemp;

        msgTemp.header.stamp = msgIMU.header.stamp;
        msgTemp.header.frame_id = msgIMU.header.frame_id;

        msgTemp.temperature = temp;

        pubTemp.publish(msgTemp);


        // Barometer
        sensor_msgs::FluidPressure msgPres;

        msgPres.header.stamp = msgIMU.header.stamp;
        msgPres.header.frame_id = msgIMU.header.frame_id;

        msgPres.fluid_pressure = pres;

        pubPres.publish(msgPres);
        //  std::cout <<"passed ALL"<<std::endl;

    }
}

