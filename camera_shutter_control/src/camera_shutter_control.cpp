#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Vector3.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>

using namespace std;
class TfTree

{
protected:

    ros::NodeHandle n;

    ros::Subscriber imagesub, sub2;
    ros::Publisher pub1, pub2, pub3;
    tf::TransformBroadcaster broadcaster;
    tf::Quaternion q;

    image_transport::ImageTransport it;
    image_transport::Subscriber imageSub3, imageSub2, imageSub1;

    double intensityUncertainty, pShadow, min_exposure_Time, max_exposure_Time, buffer1, buffer2, buffer3, change_limit, min_Shutter_Time, max_Shutter_Time, shutter_memory1, shutter_memory2, shutter_memory3, pSun, pNormal ,roll, rollDiff, pitch, pitchDiff, yaw, pixel;
    unsigned int  x, col, size, pixels,randNbrside, randNbrfront ,sidecam_start_to_end, frontcam_start_to_end, height, width, sidecam_offset, frontcam_offset, side_serial_idx, front_serial_idx, end_serial_idx, sidecam_right_start_idx, sidecam_left_start_idx,frontcam_start_idx, sidecam_right_end_idx,sidecam_left_end_idx, frontcam_end_idx;
    bool count, count1, invertedcam1;
    int sunIntensity, shadowIntensity, target_Intensity;
    ofstream cam3times, cam2times, cam1times;
    geometry_msgs::Vector3 rpy;

    void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
        //   std::cout<<"test 3"<<std::endl;
        q[0]=msg->orientation.x; q[1]=msg->orientation.y; q[2]=msg->orientation.z; q[3]=msg->orientation.w;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        //   std::cout <<"roll "<<roll<<" pitch "<<pitch<<" yaw "<<yaw<<std::endl;
        pitchDiff=-(pitch+3.14159/2-0.035)/2;  //0.035 IMU offset
        rollDiff=roll;
           //  std::cout <<"rolldiff "<<rollDiff<<" pitchDiff "<<pitchDiff<<std::endl;
        //   std::cout<<"test 4"<<std::endl;
        count=false;
    }
    void pc_callback3(const sensor_msgs::ImageConstPtr & img_msg){
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config conf;
        double exposure_change, p=pNormal;
        int intensity_Difference, left_Intensity;

        if (count){
            pitchDiff=0;
            rollDiff=0;
        }

        double_param.name = "shutter_speed";

        size=img_msg->data.size();
        height=img_msg->width;
        width=img_msg->height;
        //   std::cout<<"test 1"<<std::endl;

        sidecam_offset=(width/2)*tan(pitchDiff);
        sidecam_right_start_idx=(height/2)-round(sidecam_offset);
        sidecam_right_end_idx=(height/2)+round(sidecam_offset);
        //      std::cout<<"test 5"<<std::endl;
        if (sidecam_right_start_idx >round(height*0.75) || sidecam_right_end_idx >round(height*0.75)){
            sidecam_right_start_idx=round(height*0.75);
            sidecam_right_end_idx=round(height*0.75);
        }
        // sidecam_left_start_idx=sidecam_right_end_idx;
        // sidecam_left_end_idx=sidecam_right_start_idx;

        if (sidecam_right_start_idx>sidecam_right_end_idx){
            side_serial_idx=sidecam_right_start_idx*width;
        }
        else
        {
            side_serial_idx=sidecam_right_end_idx*width;
        }
        //      /  std::<<
        end_serial_idx=size;
        // srand (time(NULL));
        sidecam_start_to_end=end_serial_idx-side_serial_idx;

        left_Intensity=0;
        // std::cout<<"test 6"<<std::endl;
        pixels=0;
        x=(side_serial_idx/width)-round(height/10);;
        // std::cout<< "x is "<<x<<std::endl;

        col=height-x;

        for (unsigned int j=x; j<size; j=j+height*50){
            for (unsigned int k=0; k<col; k=k+10){
                pixels++;
                //std::cout <<"test2 "<<std::endl;
                // for (unsigned int i=0;i<pixels;i++){

                //      randNbrside= rand() % sidecam_start_to_end+side_serial_idx;
                //     randNbrfront= rand() % frontcam_start_to_end+front_serial_idx;
                left_Intensity=(img_msg->data[j+k])+left_Intensity;
                // }
            }
        }

        left_Intensity=left_Intensity/pixels;
        //  std::cout<< " cam3 intensity " <<left_Intensity <<std::endl;
        intensity_Difference=(target_Intensity-round(left_Intensity));
        if (left_Intensity>sunIntensity){
            p=pSun;
        }
        else{
            if(left_Intensity<shadowIntensity){
                p=pShadow;
            }
            else
            {
                p=pNormal;
            }

        }

        if (abs(intensity_Difference)>intensityUncertainty)
        {
            //   std::cout<< "abs diff is "<<abs(intensity_Difference)<<std::endl;
            exposure_change=(intensity_Difference*p)/(abs(target_Intensity-255));
            //   std::cout<< "exposure change 3 " << exposure_change <<std::endl;
        }
        else{
            exposure_change=0;
        }

        //  std::cout<<exposure_change<<" rT "<<abs(target_Intensity-255) <<std::endl;

        if ( (exposure_change>=0 && shutter_memory3>=max_Shutter_Time) || (exposure_change<=0 && shutter_memory3 <=min_Shutter_Time) ){
            //   std::cout <<"max reached "<<std::endl;
        }
        else{
            if (fabs(buffer3)<change_limit){
                buffer3=exposure_change+buffer3;}
            else
            {
                shutter_memory3=shutter_memory3+exposure_change+buffer3;
				if (shutter_memory3<min_Shutter_Time)
				{
					shutter_memory3=min_Shutter_Time;
				}
                buffer3=0;
                double_param.value=shutter_memory3;


                conf.doubles.push_back(double_param);
                srv_req.config = conf;
                ros::service::call("/camera3/camera_nodelet/set_parameters", srv_req, srv_resp);
                cam3times <<ros::Time::now()<<" "<<shutter_memory3<<"\n";
                //       std::cout <<"exposure cam3 is "<<double_param.value<< " coeff " << exposure_change << std::endl;
            }
            //    std::cout << "buffer 3 is "<<buffer3<< " cam3 intensity " <<left_Intensity <<std::endl;
            //   TF();
        }
        // pub1.publish(shutter_memory1);
        std_msgs::Float64 shutter3;
        shutter3.data=shutter_memory3;
        pub3.publish(shutter3);
    }

    void pc_callback2(const sensor_msgs::ImageConstPtr & img_msg){

        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config conf;

        double exposure_change, p=pNormal;
        int intensity_Difference, right_Intensity;

        if (count){
            pitchDiff=0;
            rollDiff=0;
        }

        double_param.name = "shutter_speed";
        size=img_msg->data.size();
        height=img_msg->width;
        width=img_msg->height;

        sidecam_offset=(width/2)*tan(pitchDiff);

        sidecam_right_start_idx=(height/2)-round(sidecam_offset);
        sidecam_right_end_idx=(height/2)+round(sidecam_offset);

        if (sidecam_right_start_idx >round(height*0.75) || sidecam_right_end_idx >round(height*0.75)){
            sidecam_right_start_idx=round(height*0.75);
            sidecam_right_end_idx=round(height*0.75);
        }

        sidecam_left_start_idx=sidecam_right_end_idx;
        sidecam_left_end_idx=sidecam_right_start_idx;

        if (sidecam_right_start_idx>sidecam_right_end_idx){
            side_serial_idx=sidecam_right_start_idx*width;

        }
        else
        {
            side_serial_idx=sidecam_right_end_idx*width;
        }

        end_serial_idx=size;
        // srand (time(NULL));
        sidecam_start_to_end=end_serial_idx-side_serial_idx;

        right_Intensity=0;
        pixels=0;
        x=(side_serial_idx/width);

        col=height-x+round(width/10);

        for (unsigned int j=0; j<size; j=j+height*50){
            for (unsigned int k=0; k<col; k=k+10){
                pixels++;
                //std::cout <<"test2 "<<std::endl;
                // for (unsigned int i=0;i<pixels;i++){

                //      randNbrside= rand() % sidecam_start_to_end+side_serial_idx;
                //     randNbrfront= rand() % frontcam_start_to_end+front_serial_idx;

                right_Intensity=(img_msg->data[j+k])+right_Intensity;
                // }
            }
        }
        right_Intensity=right_Intensity/pixels;
       // std::cout<< " cam2 intensity " <<right_Intensity <<std::endl;

        //  std::cout <<"right_Intensity "<<right_Intensity<<std::endl; // for side cam, positive is front
        //  std::cout <<"param value is "<<double_param.value<<std::endl;
        // double_param.value = 0.01;  //output.
        intensity_Difference=(target_Intensity-round(right_Intensity));

        if (right_Intensity>sunIntensity){
            p=pSun;
        }
        else{
            if(right_Intensity<shadowIntensity){
                p=pShadow;
            }
            else
            {
                p=pNormal;
            }
        }
        if (abs(intensity_Difference)>intensityUncertainty)
        {
            //   std::cout<< "abs diff is "<<abs(intensity_Difference)<<std::endl;
            exposure_change=(intensity_Difference*p)/(abs(target_Intensity-255));
    //        std::cout<< "P2 "<<p<<" exposure change 2 " << exposure_change<<" int_Diff " <<intensity_Difference << exposure_change <<std::endl;
        }
        else{
            exposure_change=0;
        }

        if ( (exposure_change>=0 && shutter_memory2>=max_Shutter_Time) || (exposure_change<=0 && shutter_memory2 <=min_Shutter_Time) ){
          //  std::cout <<"max reached "<<std::endl;
        }
        else{
            if (fabs(buffer2)<change_limit){
                buffer2=exposure_change+buffer2;
             //   std::cout <<"buff2 "<<buffer2<<std::endl;
            }
            else
            {
                shutter_memory2=shutter_memory2+exposure_change+buffer2;
				if (shutter_memory2<min_Shutter_Time)
				{
					shutter_memory2=min_Shutter_Time;
				}	
                buffer2=0;
                double_param.value=shutter_memory2;
                conf.doubles.push_back(double_param);
                srv_req.config = conf;
                ros::service::call("/camera2/camera_nodelet/set_parameters", srv_req, srv_resp);

                cam2times <<ros::Time::now()<<" "<<shutter_memory2<<"\n";
            //    std::cout <<"exposure cam2 is "<<double_param.value<< " coeff " << exposure_change<< std::endl;
            }

            //   TF();
        }
        std_msgs::Float64 shutter2;
        shutter2.data=shutter_memory2;
        pub2.publish(shutter2);
        // pub2.publish(shutter_memory2);
    }
    void pc_callback1(const sensor_msgs::ImageConstPtr & img_msg){
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config conf;
        double exposure_change, p=pNormal;
        int intensity_Difference, front_Intensity;

        if (count){
            pitchDiff=0;
            rollDiff=0;
        }

        double_param.name = "shutter_speed";

        size=img_msg->data.size();
        width=img_msg->width;
        height=img_msg->height;

        frontcam_offset=(width/2)*tan(-rollDiff);

        frontcam_start_idx=(height/2)+round(frontcam_offset);
        frontcam_end_idx=(height/2)-round(frontcam_offset);

        if (frontcam_start_idx >round(height*0.75) || frontcam_end_idx >round(height*0.75)){
            frontcam_start_idx=round(height*0.75);
            frontcam_end_idx=round(height*0.75);
        }
        if (frontcam_start_idx>frontcam_end_idx){
            front_serial_idx=frontcam_start_idx*width;
        }
        else
        {
            front_serial_idx=frontcam_end_idx*width;
        }

        end_serial_idx=size;
        srand (time(NULL));

        frontcam_start_to_end=end_serial_idx-front_serial_idx;

        front_Intensity=0;
        pixels=600;
        x=(front_serial_idx/width);

        col=height-x;
        //std::cout <<"test1 "<<std::endl;
        //for (unsigned int j=x; j<size; j=j+height*2){
        //   for (unsigned int k=0; k<col; k=k+4){
        //      pixels++;
        //std::cout <<"test2 "<<std::endl;
        //           std::cout<<"test 10.3"<< frontcam_start_to_end << " " << front_serial_idx <<std::endl;

        for (unsigned int i=0;i<pixels;i++){

            //      randNbrside= rand() % sidecam_start_to_end+side_serial_idx;
            randNbrfront= rand() % frontcam_start_to_end+front_serial_idx;

            // std::cout<<"front_serial_idx is "<< front_serial_idx<<" frontcam_start_to_end is "<<frontcam_start_to_end<<std::endl;



            if (invertedcam1==true){


                randNbrfront=randNbrfront-(width*height)/2;


            }
            //std::cout<<"randNbrfront is "<<randNbrfront<<std::endl;
            front_Intensity=(img_msg->data[randNbrfront])+front_Intensity;
            //           std::cout<<"test 10.46"<<std::endl;
        }
        //  std::cout<<"test 10.4"<<std::endl;
        front_Intensity=front_Intensity/pixels;

        //  std::cout<< " cam1 intensity " <<front_Intensity <<std::endl;


        intensity_Difference=(target_Intensity-round(front_Intensity));
        if (front_Intensity>sunIntensity){
            p=pSun;
        }
        else{
            if(front_Intensity<shadowIntensity){
                p=pShadow;
            }
            else
            {
                p=pNormal;
            }
        }
        //   std::cout<<"test 10.5"<<std::endl;
        if (abs(intensity_Difference)>intensityUncertainty)
        {
            //      std::cout<< "abs diff is "<<abs(intensity_Difference)<<std::endl;
            exposure_change=(intensity_Difference*p)/(abs(target_Intensity-255));
            //      std::cout<< "exposure change 1 " << exposure_change <<std::endl;
        }
        else{
            exposure_change=0;
        }


        if ( (exposure_change>=0 && shutter_memory1>=max_Shutter_Time) || (exposure_change<=0 && shutter_memory1 <=min_Shutter_Time) ){
            //   std::cout <<"max reached "<<std::endl;
        }
        else{
            if (fabs(buffer1)<change_limit){
                buffer1=exposure_change+buffer1;}
            else{
                shutter_memory1=shutter_memory1+exposure_change+buffer1;
				if (shutter_memory1<min_Shutter_Time)
				{
					shutter_memory1=min_Shutter_Time;
				}	
                buffer1=0;
                double_param.value=shutter_memory1;

                conf.doubles.push_back(double_param);
                srv_req.config = conf;
                ros::service::call("/camera1/camera_nodelet/set_parameters", srv_req, srv_resp);
                cam1times <<ros::Time::now()<<" "<<shutter_memory1<<"\n";
                //std::cout <<"exposure cam1 is "<<double_param.value<< " coeff " << exposure_change << std::endl;

            }
            //    std::cout << "buffer 1 is "<<buffer1<< " cam1 intensity " <<front_Intensity <<std::endl;

        }
        std_msgs::Float64 shutter1;
        shutter1.data=shutter_memory1;
        pub1.publish(shutter1);
    }

public:
    TfTree() : n("~"), it(n) {
        ros::Duration(0.5).sleep();
        sub2 = n.subscribe("/vectornav/IMU", 1, &TfTree::ImuCallback,this);
        std::string transport = "raw";

        n.param("transport",transport,transport);
        pNormal=0.00009;
        pSun=pNormal*5;
        pShadow=pNormal*2;
        shadowIntensity=70;
        sunIntensity=130;
        shutter_memory1=0.0005;  // TODO: rosparam
        // std::cout << "TEST1324" <<std::endl;
        shutter_memory2=0.0005;
        shutter_memory3=0.0005;
        change_limit=0.00001;
        buffer1=0; buffer2=0; buffer3=0;
        target_Intensity=90;  //was 95
        //   min_exposure_Time=-2;
        //   max_exposure_Time=1;
        intensityUncertainty=13;
        invertedcam1=false;
        cam1times.open ("/home/georges/cam1times.txt");
        cam2times.open ("/home/georges/cam2times.txt");
        cam3times.open ("/home/georges/cam3times.txt");

        min_Shutter_Time=0.0001; // 0.014 ms  -> 0.028 0.002 was too big, 0,000014 too small
        max_Shutter_Time=0.025-change_limit; // limit for 20 fps was 0.049

        imageSub3 = it.subscribe("/camera3/image_raw",1,&TfTree::pc_callback3,this,transport);
        imageSub2 = it.subscribe("/camera2/image_raw",1,&TfTree::pc_callback2,this,transport);
        imageSub1 = it.subscribe("/camera1/image_raw",1,&TfTree::pc_callback1,this,transport);
        pub1=n.advertise<std_msgs::Float64>("/camera1/shutter_time",10);
        pub2=n.advertise<std_msgs::Float64>("/camera2/shutter_time",10);
        pub3=n.advertise<std_msgs::Float64>("/camera3/shutter_time",10);
    }
};

int main(int argc, char * argv[]){
    ros::init(argc, argv, "camera_shutter_control");
    TfTree TF;
    ros::spin();
}

