/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <image_transport/image_transport.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

ros::Publisher pub_odometry;
image_transport::Publisher pub_visualization;
std::string map_frame_id, base_link_frame_id;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh("~");

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 100);

    nh.param<std::string>("map_frame_id", map_frame_id,"map");
    nh.param<std::string>("base_link_frame_id", base_link_frame_id,"base_link");
    pub_odometry = nh.advertise<nav_msgs::Odometry>("/odometry", 1);
    image_transport::ImageTransport it(nh);
    pub_visualization = it.advertise("/visualization", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat current_pose, to_show;
    current_pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    to_show = mpSLAM->GetViewerImage();

    if (!current_pose.empty()) {
        cv::Mat R = current_pose(cv::Rect(0,0,3,3));
        cv::Mat t = current_pose(cv::Rect(3,0,1,3));

        cv::Mat p = -R.t() * t;
        cv::Mat O = R.t();

        tf2::Matrix3x3 Rcw(
            O.at<float>(0,0), O.at<float>(0,1),O.at<float>(0,2),
            O.at<float>(1,0), O.at<float>(1,1), O.at<float>(1,2),
            O.at<float>(2,0), O.at<float>(2,1), O.at<float>(2,2)
        );

        tf2::Quaternion Qcw;
        Rcw.getRotation(Qcw);

        nav_msgs::Odometry odometry;

        odometry.header.frame_id = map_frame_id;
        odometry.header.stamp = ros::Time::now();
        odometry.child_frame_id = base_link_frame_id;

        odometry.pose.pose.orientation.x = Qcw.x();
        odometry.pose.pose.orientation.y = Qcw.y();
        odometry.pose.pose.orientation.z = Qcw.z();
        odometry.pose.pose.orientation.w = Qcw.w();

        odometry.pose.pose.position.x = p.at<float>(0);
        odometry.pose.pose.position.y = p.at<float>(1);
        odometry.pose.pose.position.z = p.at<float>(2);

        pub_odometry.publish(odometry);

        //output.Log();
    }

    if (!to_show.empty()) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", to_show).toImageMsg();
        pub_visualization.publish(msg);
    }
}


