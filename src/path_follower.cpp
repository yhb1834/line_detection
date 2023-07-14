#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt8MultiArray.h>

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        // Here, you can process the image as you need...

        Vec3b color_1 = image.at<Vec3b>(Point(100, 100));

        // 픽셀의 색깔 데이터 가져오기 (at)
        for (int i = 0; i < 3; i++) {
            cout << "at : " << int(color_1.val[i]) << " ";
        }
        cout << endl;

        // // 픽셀의 색깔 데이터 가져오기 (ptr)
        // uchar* color_2 = image.ptr<uchar>(100);

        // for (int i = 0; i < 3; i++) {
        //     cout << int(color_2[100 * 3 + i]) << " ";
        // }
        // cout << endl;

        // // 픽셀의 색깔 데이터 가져오기 (data)
        // uchar* color_3 = image.data;

        // for (int i = 0; i < 3; i++) {
        //     cout << int(color_3[100 * 3 * image.cols + 100 * 3 + i]) << " ";
        // }
        // cout << endl;



    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/video_frames", 1, imageCallback);
    ros::spin();

	return 0;
}