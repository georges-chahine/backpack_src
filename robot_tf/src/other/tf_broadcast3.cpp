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
    ros::Subscriber sub, sub2, sub3, sub4;
    tf::TransformBroadcaster broadcaster;
    geographic_msgs::GeoPoint geo_pt;
    double roll, pitch, yaw,pi, x_init, y_init, z_init, x_init2, y_init2, z_init2, x_init3, y_init3, z_init3;
    bool count, count2, count3;
    geometry_msgs::Vector3 rpy;
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
        tf::Quaternion qCam1 = tf::createQuaternionFromRPY(-pi/2, pi/2, 0);
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qCam1, tf::Vector3(0.03, 0.35258-2.958/1000, 14.55/1000)),
                        ros::Time::now(),"imu", "camera2"));

        tf::Quaternion qCam2 = tf::createQuaternionFromRPY(pi/2, +pi/2, 0);
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qCam2, tf::Vector3(0.03, -0.23342+2.958/1000, 14.55/1000)),
                        ros::Time::now(),"imu", "camera3"));

        tf::Quaternion qCam3 = tf::createQuaternionFromRPY(0, 0.-72345, 0);
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qCam3, tf::Vector3(0.1175, 0.05958, 0.0558)),
                        ros::Time::now(),"imu", "camera1"));


        tf::Quaternion qLaser = tf::createQuaternionFromRPY(0, 0, 0);
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qLaser, tf::Vector3(0.0165, 0.05958, 0.055)),
                        ros::Time::now(),"imu", "laser"));

    }

    void GPSCallback(const geometry_msgs::Point::ConstPtr& msg)
    {


        if (count2==false){

            double x=msg->x, y=msg->y, z=msg->z;
            //        std::cout << "GPS SIGNAL FOUND" <<std::endl;
            //      geo_pt.latitude = msg->latitude;
            //      geo_pt.longitude = msg->longitude;
            //     geo_pt.altitude = msg->altitude;
            if (count) {
                count=false;
                //     std::cout <<"ERRORRRRR"<<std::endl;
                x_init=x;
                y_init=y;
                z_init=z;
            }
        }
    }

    void GPSCallback2(const geometry_msgs::Point::ConstPtr& msg)
    {

        if (msg->x>0){
            double x=msg->x, y=msg->y, z=msg->z;
            //        std::cout << "GPS SIGNAL FOUND" <<std::endl;
            //      geo_pt.latitude = msg->latitude;
            //      geo_pt.longitude = msg->longitude;
            //     geo_pt.altitude = msg->altitude;
            if (count2) {
                count2=false;
                //     std::cout <<"ERRORRRRR"<<std::endl;
                x_init2=x;
                y_init2=y;
                z_init2=z;

            }
            broadcaster.sendTransform(
                        tf::StampedTransform(
                            tf::Transform(q0, tf::Vector3(x-x_init,y-y_init, 0.3)),
                            ros::Time::now(),"gps_start", "gps_current"));
            //   ROS_INFO("UTM coordinates z: [%f]", utm_pt.altitude);
            ROS_INFO("GPS pose RAW  x: [%f], y: [%f], z: [%f]", x-x_init, y-y_init, z-z_init);

            //   ROS_INFO("UTM coordinates z: [%f]", utm_pt.altitude);
            ROS_INFO("GPS pose INS  x: [%f], y: [%f], z: [%f]", x-x_init2, y-y_init2, z-z_init2);
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

    void GPSCallback3(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {

        if (msg->status.service>4){
            //        std::cout << "GPS SIGNAL FOUND" <<std::endl;
            geo_pt.latitude = msg->latitude;
            geo_pt.longitude = msg->longitude;
            geo_pt.altitude = msg->altitude;
            TF();
        }

    }

    void TF()
    {
        geodesy::UTMPoint utm_pt(geo_pt);

        //      std::cout <<count<<std::endl;
        if (count3) {
            count3=false;
            //     std::cout <<"ERRORRRRR"<<std::endl;
            x_init3=utm_pt.easting;
            y_init3=utm_pt.northing;
            z_init3=utm_pt.altitude;
        }

        ROS_INFO("GPS pose UTM x: [%f], y: [%f], z: [%f]", utm_pt.easting-x_init3, utm_pt.northing-y_init3, utm_pt.altitude-z_init3);


    }



public:
    TfTree() : n("~") {
        ros::Duration(0.5).sleep();
        sub = n.subscribe("/vectornav/IMU", 1, &TfTree::IMUCallback,this);
        sub2 = n.subscribe("/vectornav/GPS_XYZ", 1, &TfTree::GPSCallback,this);
        sub3 = n.subscribe("/vectornav/GPS_INS_XYZ", 1, &TfTree::GPSCallback2,this);
        sub4 = n.subscribe("/vectornav/GPS_INS", 1, &TfTree::GPSCallback3,this);
    }
};

int main(int argc, char * argv[]){

    ros::init(argc, argv, "robot_tf_publisher");
    TfTree TF;
    ros::spin();
}
