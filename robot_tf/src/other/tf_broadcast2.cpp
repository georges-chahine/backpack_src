#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Vector3.h>
#include <geodesy/utm.h>

class TfTree

{
protected:

    ros::NodeHandle n;
    ros::Subscriber sub, sub2;
    tf::TransformBroadcaster broadcaster;
    geographic_msgs::GeoPoint geo_pt;
    double roll, pitch, yaw,pi, x_init, y_init, z_init;
    bool count;
    geometry_msgs::Vector3 rpy;
    geometry_msgs::Point pGPSl;
    tf::Quaternion q, q0;

    void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        pi=3.14159;
        //q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
        //tf::Quaternion q(0,0,0,1);
        q0[0]=0; q0[1]=0; q0[2]=0; q0[3]=1;
        q[0]=msg->orientation.x; q[1]=msg->orientation.y; q[2]=msg->orientation.z; q[3]=msg->orientation.w;
        // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
        // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // ROS_INFO("Imu Orientation roll: [%f], pitch: [%f], yaw: [%f]", roll, pitch, yaw);

        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(q, tf::Vector3(0,0,0.3)),
                        ros::Time::now(),"gps_current", "imu"));

        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(q0, tf::Vector3(-0.2, 0.19, 0)),
                        ros::Time::now(),"imu", "base_link"));
        //from one of the edge screws (cam1 and cam2), the sensor is at:
        //      y: 12.032 + 3  -17.99 =-2.958 mm
        //     z: 14.5 mm out of the plane
        tf::Quaternion qCam1 = tf::createQuaternionFromRPY(-pi/2, 0, 0);
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qCam1, tf::Vector3(0.03, 0.35258-2.958/1000, 14.55/1000)),
                        ros::Time::now(),"imu", "camera1"));

        tf::Quaternion qCam2 = tf::createQuaternionFromRPY(pi/2, 0, 0);
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qCam2, tf::Vector3(0.03, -0.23342+2.958/1000, 14.55/1000)),
                        ros::Time::now(),"imu", "camera2"));

        tf::Quaternion qCam3 = tf::createQuaternionFromRPY(0, 0.-72345, 0);
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qCam3, tf::Vector3(0.1175, 0.05958, 0.0558)),
                        ros::Time::now(),"imu", "camera3"));


        tf::Quaternion qLaser = tf::createQuaternionFromRPY(0, 0, pi);
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qLaser, tf::Vector3(0.0165, 0.05958, 0.055)),
                        ros::Time::now(),"imu", "laser"));

    }

    void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {

        if (msg->status.service){
    //        std::cout << "GPS SIGNAL FOUND" <<std::endl;
            geo_pt.latitude = msg->latitude;
            geo_pt.longitude = msg->longitude;
            geo_pt.altitude = msg->altitude;
            TF();
        }
        else
        {
           // std::cout << "NO GPS SIGNAL" <<std::endl;
            broadcaster.sendTransform(
                        tf::StampedTransform(
                            tf::Transform(q0, tf::Vector3(0,0, 0.3)),
                            ros::Time::now(),"gps_start", "gps_current"));
        }
    }

    void TF()
    {
        geodesy::UTMPoint utm_pt(geo_pt);

  //      std::cout <<count<<std::endl;
        if (count) {
            count=false;
       //     std::cout <<"ERRORRRRR"<<std::endl;
            x_init=utm_pt.easting;
            y_init=utm_pt.northing;
            z_init=utm_pt.altitude;

        }

        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(q0, tf::Vector3(utm_pt.easting-x_init,utm_pt.northing-y_init, utm_pt.altitude-z_init)),
                        ros::Time::now(),"gps_start", "gps_current"));
        ROS_INFO("UTM coordinates z: [%f]", utm_pt.altitude);
        ROS_INFO("GPS pose  x: [%f], y: [%f], z: [%f]", utm_pt.easting-x_init, utm_pt.northing-y_init, utm_pt.altitude-z_init);


    }

public:
    TfTree() : n("~") {
        ros::Duration(0.5).sleep();
        sub = n.subscribe("/vectornav/IMU", 1, &TfTree::IMUCallback,this);
        sub2 = n.subscribe("/vectornav/GPS", 1, &TfTree::GPSCallback,this);
    }
};

int main(int argc, char * argv[]){

    ros::init(argc, argv, "robot_tf_publisher");
    TfTree TF;
    ros::spin();
}
