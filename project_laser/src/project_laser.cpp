
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/xphoto.hpp"
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <fstream>


//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
//#include <sensor_msgs/Imu.h>
//#include <tf/LinearMath/Matrix3x3.h>
//#include <opencv2/opencv.hpp>
//#include <image_transport/transport_hints.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <pcl-1.7/pcl/point_types.h>
//#include <eigen3/Eigen/Dense>


class TfTree

{
protected:

    ros::NodeHandle n;
    image_transport::ImageTransport it;
    // tf::Quaternion q;
    // Eigen::Vector3f tr;
    ros::Subscriber tfSub, scanSub;
    ros::Publisher pcPub;
    image_transport::Subscriber imageSub1, imageSub2, imageSub3;
    //tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener1, listener2,listener3, listener4, listener5;
    tf::StampedTransform transform;
    laser_geometry::LaserProjection projector1, projector2, projector3, projector;
    //double roll, pitch, yaw,pi;
    //float pi;
    //geometry_msgs::Vector3 rpy;
    std::ofstream laserStamps;
    cv::Mat intrinsics1,intrinsics2,intrinsics3;
    cv::Mat distCoeffs1,distCoeffs2,distCoeffs3;
    float fx1, fy1, cx1, cy1, k11, k21, k31, k41, k51;
    float fx2, fy2, cx2, cy2, k12, k22, k32, k42, k52;
    float fx3, fy3, cx3, cy3, k13, k23, k33, k43, k53;
    cv::Mat img1, img2, img3;
    unsigned int rows,cols;
    //  Eigen::Matrix3d intrinsics;
    //Eigen::Vector4d distCoeffs;
    bool flag;
    int counter, counter2;
    pcl::PointCloud<pcl::PointXYZRGB>  temp, pclscene;
    pcl::PCLPointCloud2::Ptr pclbuffer;
    pcl::PCLPointCloud2 vcloudcc;
    pcl::PCDWriter writer;
    pcl::VoxelGrid<pcl::PointXYZRGB> vgrid, vgrid2;
    sensor_msgs::PointCloud2 pclbuffer2;


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {


        sensor_msgs::PointCloud2 cloud1, cloud2, cloud3, cloud4, cloud5;


        pcl::PointCloud<pcl::PointXYZ> pclnewscan1,pclnewscan2,pclnewscan3;
        try
        {
            listener1.waitForTransform("/camera1",msg->header.frame_id, msg->header.stamp+ ros::Duration().fromSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0) );
            projector1.transformLaserScanToPointCloud("/camera1", *msg, cloud1, listener1);



        }

        catch (tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"laser\" to \"camera1\": %s", ex.what());
        }

        try
        {
            listener2.waitForTransform("/camera2",msg->header.frame_id, msg->header.stamp+ ros::Duration().fromSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0) );
            projector2.transformLaserScanToPointCloud("/camera2", *msg, cloud2, listener2);

        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"laser\" to \"camera2\": %s", ex.what());
        }

        try
        {
            listener3.waitForTransform("/camera3",msg->header.frame_id, msg->header.stamp+ ros::Duration().fromSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0) );
            projector3.transformLaserScanToPointCloud("/camera3", *msg, cloud3, listener3);
        }

        catch (tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"laser\" to \"camera3\": %s", ex.what());
        }



        Eigen::Matrix3f K1,K2,K3;
        K1<<fx1, 0, cx1,
                0, fy1, cy1,
                0, 0, 1;
        K2<<fx2, 0, cx2,
                0, fy2, cy2,
                0, 0, 1;
        K3<<fx3, 0, cx3,
                0, fy3, cy3,
                0, 0, 1;


        pcl::fromROSMsg(cloud1, pclnewscan1);
        pcl::fromROSMsg(cloud2, pclnewscan2);
        pcl::fromROSMsg(cloud3, pclnewscan3);

        pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud, transformed_cloudGPS;
        listener4.waitForTransform(msg->header.frame_id,msg->header.frame_id, msg->header.stamp+ ros::Duration().fromSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0) );
        projector.transformLaserScanToPointCloud("/laser", *msg, cloud4, listener4);


        try{

            listener5.waitForTransform(msg->header.frame_id,msg->header.frame_id, msg->header.stamp+ ros::Duration().fromSec(msg->ranges.size()*msg->time_increment), ros::Duration(2.0) );
            projector.transformLaserScanToPointCloud("/gps_start", *msg, cloud5, listener5);
            pcl::fromROSMsg(cloud5, transformed_cloudGPS);

        }

        catch (tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"gps_start\" to \"camera1\": %s", ex.what());
        }



        tf::StampedTransform transform;
            try{
              listener5.lookupTransform("/gps_current", "/laser",
                                       ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
            }
        std::cout<<"Quat is "<<transform.getRotation().getX()<<" "<<transform.getRotation().getY()<<" "<<transform.getRotation().getZ()<<" "<<transform.getRotation().getW()<<std::endl;





        pcl::fromROSMsg(cloud4, transformed_cloud);


        Eigen::MatrixXf uvw1(3,pclnewscan1.getMatrixXfMap(3,4,0).cols());
        Eigen::MatrixXf uvw2(3,pclnewscan2.getMatrixXfMap(3,4,0).cols());
        Eigen::MatrixXf uvw3(3,pclnewscan3.getMatrixXfMap(3,4,0).cols());
        //double pi;
        //pi=3.14159;
        //std::cout<<pclnewscan3.getMatrixXfMap(3,4,0).cols()<<std::endl;
        uvw1=K1*pclnewscan1.getMatrixXfMap(3,4,0);
        uvw2=K2*pclnewscan2.getMatrixXfMap(3,4,0);
        uvw3=K3*pclnewscan3.getMatrixXfMap(3,4,0);

        ColorFn(img1,uvw1,transformed_cloud,transformed_cloudGPS);
        ColorFn(img2,uvw2,transformed_cloud, transformed_cloudGPS);
        //std::cout <<uvw3.cols()<<std::endl;
        ColorFn(img3,uvw3,transformed_cloud, transformed_cloudGPS);

        pcPub.publish(transformed_cloud);


        //pclscene+=transformed_cloudGPS;
        temp+=transformed_cloudGPS;



       // double stamp=msg->header.stamp;


        pcl::PointCloud<pcl::PointXYZRGB> vcloud2;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud0->points.resize(transformed_cloud.size());

        for (unsigned int i=0; i<transformed_cloud.size(); i++){
            cloud0->points[i].rgb =  transformed_cloud.points[i].rgb;
            cloud0->points[i].x =  transformed_cloud.points[i].x;
            cloud0->points[i].y =  transformed_cloud.points[i].y;
            cloud0->points[i].z =  transformed_cloud.points[i].z;

        }

        vgrid2.setInputCloud (cloud0);
        vgrid2.setLeafSize (0.05f, 0.05f, 0.05f);
        vgrid2.filter (vcloud2);

        std::ostringstream filename2;
        filename2<<"stampedPC/cloud_"<<counter2<<".pcd";

        pcl::io::savePCDFileASCII (filename2.str(), vcloud2);

        counter2++;
        laserStamps << msg->header.stamp<<","<<filename2.str()<<"\n";
        //laserStamps.close();




        if (temp.size()>100000)
        {

            pcl::PointCloud<pcl::PointXYZRGB> vcloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            cloud->points.resize(temp.size());

            for (unsigned int i=0; i<temp.size(); i++){
                cloud->points[i].rgb =  temp.points[i].rgb;
                cloud->points[i].x =  temp.points[i].x;
                cloud->points[i].y =  temp.points[i].y;
                cloud->points[i].z =  temp.points[i].z;

            }

            vgrid.setInputCloud (cloud);
            vgrid.setLeafSize (0.05f, 0.05f, 0.05f);
            vgrid.filter (vcloud);

            std::ostringstream filename;
            filename<<"cloud_"<<counter<<".pcd";

            pcl::io::savePCDFileASCII (filename.str(), vcloud);

            counter++;
            temp.clear();
        }



        //}

        /*
        int buffer_size=100;
        if (temp.size()>( ( transformed_cloudGPS.size() )*buffer_size*2) ) {
            pclscene.erase(pclscene.begin(), pclscene.begin() + buffer_size );
            pclbuffer.reset (new pcl::PCLPointCloud2 ());
            pcl::toROSMsg(temp, pclbuffer2);  // copy first point to sensor::msg pc2
            pcl_conversions::toPCL(pclbuffer2, *pclbuffer); //sensor pc2 to pcl pc2


            vcloudcc.height=1; //unorganised data
            vcloudcc.width=vcloud.width+vcloudcc.width; //Set to the number of points for unorganized data.
            vcloudcc.fields.insert(vcloudcc.fields.end(), vcloud.fields.begin(), vcloud.fields.end() ) ;
            vcloudcc.point_step=vcloud.point_step; //length of a point in bytes
            vcloudcc.row_step=vcloud.row_step;  //length of a row in bytes
            vcloudcc.data.insert(vcloudcc.data.end(), vcloud.data.begin(), vcloud.data.end() ) ;
            vcloudcc.is_dense=vcloud.is_dense;     // does not affect result
            vcloudcc.is_bigendian=vcloud.is_bigendian; // does not affect result

            temp.clear();
            writer.write ("backpack_laser_map.pcd", vcloudcc, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
        }
        */

    }

    void ColorFn(cv::Mat &imgref, Eigen::MatrixXf &uvw, pcl::PointCloud<pcl::PointXYZRGB> &transformed_cloud, pcl::PointCloud<pcl::PointXYZRGB> &transformed_cloudGPS){

        //return;
        Eigen::MatrixXd uv(2,uvw.cols());
        //   std::cout <<uvw.cols()<<" "<<cols<<" "<<transformed_cloud.points.size()<<" "<<std::endl;
        int count=0;
        // cv::Point3_<Vec3b>* p;
        // std::cout <<uvw.cols()<<std::endl;
        if (!imgref.empty()){
            // cv::xphoto::balanceWhite(imgref, imgref, 'WHITE_BALANCE_SIMPLE', 0,255,0,255);
            cv::Ptr<cv::xphoto::SimpleWB> wb = cv::xphoto::createSimpleWB();
            wb->setInputMin(0);
            wb->setInputMax(255);
            wb->setOutputMin(0);
            wb->setOutputMax(255);
            // cv::imshow( "before wb", imgref);

            wb->balanceWhite(imgref, imgref);
            // cv::imshow( "after wb", imgref);
            // cv::waitKey(0);

        }
        for (unsigned int i=0; i<uvw.cols(); i++ ){
            uv(0,i)=uvw(0,i)/(uvw(2,i));
            uv(1,i)=uvw(1,i)/(uvw(2,i));

            if (uvw(2,i)<0){continue;}

            if (  (round(uv(0,i))>0) && (round(uv(0,i))<cols) && (round(uv(1,i))>0) && (round(uv(1,i))<rows) && !imgref.empty() ) {

                count ++;
                //  std::cout<<uvw.cols()<<std::endl;
                float x=round(uv(0,i));  //was rows-
                float y=round(uv(1,i));


                //std::cout <<x<<" "<<y<<std::endl;

                cv::Vec3b color1 = imgref.at<cv::Vec3b>(y,x);
                cv::Vec3b color2 = imgref.at<cv::Vec3b>(y+1,x);
                cv::Vec3b color3 = imgref.at<cv::Vec3b>(y,x+1);
                cv::Vec3b color4 = imgref.at<cv::Vec3b>(y-1,x);
                cv::Vec3b color5 = imgref.at<cv::Vec3b>(y,x-1);

                float red=((int)color1.val[2]+(int)color2.val[2]+(int)color3.val[2]+(int)color4.val[2]+(int)color5.val[2])/5;
                float green=((int)color1.val[1]+(int)color2.val[1]+(int)color3.val[1]+(int)color4.val[1]+(int)color5.val[1])/5;
                float blue=((int)color1.val[0]+(int)color2.val[0]+(int)color3.val[0]+(int)color4.val[0]+(int)color5.val[0])/5;
                //float red=255, green=0, blue=0;
                //std::cout<<round(uv(0,i))<<" "<<round(uv(1,i))<<std::endl;
                //              std::cout<<"test color " <<(int)color.val[0]<<std::endl;

                uint8_t r = round (red), g = round(green), b = round(blue);
                if (g>r && g>b){ g=139; r=34; b =34;}
                // std::cout <<red<<std::endl;
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                //std::cout << i <<" "<<transformed_cloud.points.size()<<std::endl;

                assert(i < transformed_cloud.points.size());

                transformed_cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);


                if(transformed_cloud.points.size()==transformed_cloudGPS.points.size()){
                    transformed_cloudGPS.points[i].rgb =  transformed_cloud.points[i].rgb;


                }
              //   imgref.at<cv::Vec3b>(y,x)[0]=0;
               //  imgref.at<cv::Vec3b>(y,x)[1]=0;
               //  imgref.at<cv::Vec3b>(y,x)[2]=255;
                // std::cout<<"y is "<<y<<std::endl;
                //            std::cout<<"x ix "<<x<<std::endl;
            }
        }
         //  if (!imgref.empty()){
         //    cv::imshow( "projected", imgref);
         // std::cout<<rows<< " "<<cols<<std::endl; //1200 1600
          //   cv::waitKey(0);
         // }
        imgref.release();
        uvw.resize(0,0);
        //uvw.reset();
        std::cout<<"count is " <<count<<std::endl;

        return;
    }

    void pc_callback1(const sensor_msgs::ImageConstPtr & img_msg)  //ROS callback
    {
        try{

            cv::Mat img(cv_bridge::toCvCopy(img_msg,"bgr8")->image);

            cv::Mat Undistortedimg;
            cv::undistort(img, Undistortedimg, intrinsics1, distCoeffs1);

            img1=Undistortedimg.clone();
            if (flag==false){
                cv::Size s=img.size();
                flag=true;
                // count=0;
                rows = s.height;
                cols = s.width;
            }


        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;

        }

    }

    void pc_callback2(const sensor_msgs::ImageConstPtr & img_msg)  //ROS callback
    {
        //std::cout<<"Cam2 Cb"<<std::endl;

        try{
            cv::Mat img(cv_bridge::toCvCopy(img_msg,"bgr8")->image);

            cv::Mat Undistortedimg;
            cv::undistort(img, Undistortedimg, intrinsics2, distCoeffs2);

            img2=Undistortedimg.clone();
            if (flag==false){
                cv::Size s=img.size();
                flag=true;
                // count=0;
                rows = s.height;
                cols = s.width;
            }
            // cv::imshow( "Original", img );
            // cv::imshow( "Undistorted", Undistortedimg );
            //  cv::waitKey(0);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;

        }

    }

    void pc_callback3(const sensor_msgs::ImageConstPtr & img_msg)  //ROS callback
    {
        try{
            cv::Mat img(cv_bridge::toCvCopy(img_msg,"bgr8")->image);

            cv::Mat Undistortedimg;
            cv::undistort(img, Undistortedimg, intrinsics3, distCoeffs3);

            img3=Undistortedimg.clone();
            if (flag==false){
                cv::Size s=img.size();
                flag=true;
                // count=0;
                rows = s.height;
                cols = s.width;
            }

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;

        }
    }

public:
    TfTree() : n("~"), it(n) {
        ros::Duration(0.5).sleep();
        std::string transport = "raw";
        n.param("transport",transport,transport);
        flag=false;

        //pi=3.14159;

        fx1=772.085491; fy1=773.598591; cx1=766.314131; cy1=597.031876;
        k11=-0.205104; k21= 0.084919; k31=-0.001193; k41= 0.000155; k51=-0.016763;

        fx2=775.454049; fy2=775.861759; cx2=744.33135; cy2=589.328156;
        k12=-0.202612; k22= 0.076566; k32=-0.000849; k42=0.000059; k52=-0.013072;

        fx3=774.277102; fy3=774.449911; cx3=783.93828; cy3=574.134976;
        k13=-0.200168; k23= 0.076846; k33=-0.000593; k43= -0.000582; k53=-0.01378;


        distCoeffs1 = cv::Mat::zeros(5, 1, CV_64FC1);
        intrinsics1 = cv::Mat::eye(3, 3, CV_32FC1);

        counter=0; counter2=0;
        distCoeffs1.at<double>(0,0)=k11;
        distCoeffs1.at<double>(1,0)=k21;
        distCoeffs1.at<double>(2,0)=k31;
        distCoeffs1.at<double>(3,0)=k41;
        distCoeffs1.at<double>(4,0)=k51;
        intrinsics1.at<float>(0,0)=fx1; intrinsics1.at<float>(0,1)=0; intrinsics1.at<float>(0,2)=cx1;
        intrinsics1.at<float>(1,0)=0; intrinsics1.at<float>(1,1)=fy1; intrinsics1.at<float>(1,2)=cy1;
        intrinsics1.at<float>(2,0)=0; intrinsics1.at<float>(2,1)=0; intrinsics1.at<float>(2,2)=1;

        distCoeffs2 = cv::Mat::zeros(5, 1, CV_64FC1); // was 1, 4
        intrinsics2 = cv::Mat::eye(3, 3, CV_32FC1);


        distCoeffs2.at<double>(0,0)=k12;
        distCoeffs2.at<double>(1,0)=k22;
        distCoeffs2.at<double>(2,0)=k32;
        distCoeffs2.at<double>(3,0)=k42;
        distCoeffs2.at<double>(4,0)=k52; //added
        intrinsics2.at<float>(0,0)=fx2; intrinsics2.at<float>(0,1)=0; intrinsics2.at<float>(0,2)=cx2;
        intrinsics2.at<float>(1,0)=0; intrinsics2.at<float>(1,1)=fy2; intrinsics2.at<float>(1,2)=cy2;
        intrinsics2.at<float>(2,0)=0; intrinsics2.at<float>(2,1)=0; intrinsics2.at<float>(2,2)=1;

        distCoeffs3 = cv::Mat::zeros(5, 1, CV_64FC1);
        intrinsics3 = cv::Mat::eye(3, 3, CV_32FC1);

        distCoeffs3.at<double>(0,0)=k13;
        distCoeffs3.at<double>(1,0)=k23;
        distCoeffs3.at<double>(2,0)=k33;
        distCoeffs3.at<double>(3,0)=k43;
        distCoeffs3.at<double>(4,0)=k53;
        intrinsics3.at<float>(0,0)=fx3; intrinsics3.at<float>(0,1)=0; intrinsics3.at<float>(0,2)=cx3;
        intrinsics3.at<float>(1,0)=0; intrinsics3.at<float>(1,1)=fy3; intrinsics3.at<float>(1,2)=cy3;
        intrinsics3.at<float>(2,0)=0; intrinsics3.at<float>(2,1)=0; intrinsics3.at<float>(2,2)=1;

        laserStamps.open("laserStamps.txt");

        scanSub=n.subscribe<sensor_msgs::LaserScan>("/scan",1,&TfTree::laserCallback,this);
        // imageSub1=it.subscribe("/camera1/image_raw",1,&TfTree::pc_callback1,this,transport);
        //imageSub2=it.subscribe("/camera2/image_raw",1,&TfTree::pc_callback2,this,transport);
        imageSub3=it.subscribe("/camera3/image_raw",1,&TfTree::pc_callback3,this,transport);
        pcPub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("coloredScan", 1);

    }
};

int main(int argc, char * argv[]){

    ros::init(argc, argv, "project_laser");
    TfTree TF;
    ros::spin();
}
