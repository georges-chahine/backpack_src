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
    double roll, pitch, yaw,pi, x_init, y_init, z_init;
    bool flag;
    geometry_msgs::Vector3 rpy;
    tf::Quaternion q, q0, q1,q2, q00;

    void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        pi=3.1415926536;
        //q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
        //tf::Quaternion q(0,0,0,1);
        //q0[0]=0; q0[1]=0; q0[2]=0; q0[3]=1;
		//q00[0]=0; q00[1]=0; q00[2]=0; q00[3]=1;
        q00 = tf::createQuaternionFromRPY(pi, 0, 0);
        q1 = tf::createQuaternionFromRPY(0, 0, pi/2);
        q0 = tf::createQuaternionFromRPY(pi, 0, 0);
        double angledeg=0;
        double deg2rad=(angledeg/180)*pi;
        q2 = tf::createQuaternionFromRPY(0,0, 0);
        q[0]=msg->orientation.x; q[1]=msg->orientation.y; q[2]=msg->orientation.z; q[3]=msg->orientation.w;
        // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
        // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
       // ROS_INFO("Imu Orientation roll: [%f], pitch: [%f], yaw: [%f]", roll, pitch, yaw);



        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(q2, tf::Vector3(0,0,0.3)),
                        ros::Time::now(),"base_link", "imu"));

        tf::Quaternion qCam1 = tf::createQuaternionFromRPY(pi/2, 0, 0);  //was -pi/2  pi
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(q2*qCam1, tf::Vector3(0.03, -0.35258+2.958/1000, -14.55/1000)),
                        ros::Time::now(),"imu", "camera2"));


        tf::Quaternion qCam2 = tf::createQuaternionFromRPY(-pi/2 ,pi, 0 );
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qCam2, tf::Vector3(0.03, +0.23342-2.958/1000, -14.55/1000)),
                        ros::Time::now(),"imu", "camera3"));

        tf::Quaternion qCam3 = tf::createQuaternionFromRPY(pi-0.72345*0, 0, -pi/2);
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qCam3, tf::Vector3(0.15342, -0.0587, -0.04233)),
                        ros::Time::now(),"imu", "camera1"));


        tf::Quaternion qLaser = tf::createQuaternionFromRPY(pi, 0, 0*pi);
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(qLaser, tf::Vector3(0.01848, -0.05937, -0.113)),
                        ros::Time::now(),"imu", "laser"));


    void GPSCallback4(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {

        if ( (msg->status.status>0) && ( (msg->latitude)>0) ){

            geo_pt.latitude = msg->latitude;
            geo_pt.longitude = msg->longitude;
            geo_pt.altitude = msg->altitude;
            geodesy::UTMPoint utm_pt(geo_pt);
            double x=utm_pt.northing;    //NED
            double y=utm_pt.easting;
            double z=-utm_pt.altitude;

            if (flag) {
                flag=false;
                //     std::cout <<"ERRORRRRR"<<std::endl;
                x_init=x;
                y_init=y;
                z_init=z;
            }

            broadcaster.sendTransform(
                        tf::StampedTransform(
                            tf::Transform(q, tf::Vector3(x-x_init,y-y_init, 0)),
                            ros::Time::now(),"odom", "base_link"));
            //   ROS_INFO("UTM coordinates z: [%f]", utm_pt.altitude);
           // ROS_INFO("INS pose   x: [%f], y: [%f], z: [%f]", x-x_init, y-y_init, z-z_init);

        }
        else
        {
            // std::cout << "NO GPS SIGNAL" <<std::endl;
            broadcaster.sendTransform(
                        tf::StampedTransform(
                            tf::Transform(q, tf::Vector3(0,0,0)),
                            ros::Time::now(),"odom", "base_link"));
        }

    }

public:
    TfTree() : n("~") {
        ros::Duration(0.5).sleep();
        sub = n.subscribe("/vectornav/IMU", 1, &TfTree::IMUCallback,this);
        sub4 = n.subscribe("/vectornav/GPS_INS", 1, &TfTree::GPSCallback4,this);
        flag=true;
    }
};

int main(int argc, char * argv[]){

    ros::init(argc, argv, "robot_tf_publisher");
    TfTree TF;
    ros::spin();
}
