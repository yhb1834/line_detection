#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt8MultiArray.h>

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

image_transport::Publisher pub;  // Declare the publisher as a global variable


cv::Mat& get_pxl_at(cv::Mat& img){
    Vec3b color_1 = img.at<Vec3b>(Point(100, 100));

    /*-----------------픽셀의 색깔 데이터 가져오기 (at)------------------ */ 
    for (int i = 0; i < 3; i++) {
        cout << int(color_1.val[i]) << " ";
    }
    //return img;
}

cv::Mat& get_pxl_ptr(cv::Mat& img){
    uchar* color_2 = img.ptr<uchar>(100);

    for (int i = 0; i < 3; i++) {
        cout << int(color_2[100 * 3 + i]) << " ";
    }
}

cv::Mat& get_pxl_data(cv::Mat& img){
    
    uchar* color_3 = img.data;

    for (int i = 0; i < 3; i++) {
        cout << int(color_3[100 * 3 * img.cols + 100 * 3 + i]) << " ";
    }
}

cv::Mat& put_pxl(cv::Mat& img){
    /*---------------------- 픽셀의 색깔 데이터 변경하기 ------------------ */ 
    for (int y = 90; y < 105; y++) {
        for (int x = 90; x < 105; x++) {
            img.at<Vec3b>(Point(y, x)) = Vec3b(0, 0, 0);
        }
    }

    return img;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        // Here, you can process the image as you need...
        cout << "| at | ";
        get_pxl_at(image);

        cout << "| ptr | ";
        get_pxl_ptr(image);

        cout << "| data | ";
        get_pxl_data(image);


        cout << endl;


        /*---------------------- 픽셀의 색깔 데이터 변경하기 ------------------ */ 
        // for (int y = 90; y < 105; y++) {
        //     for (int x = 90; x < 105; x++) {
        //         image.at<Vec3b>(Point(y, x)) = Vec3b(0, 0, 0);
        //     }
        // }

        //cv::line(image, cv::Point(0, 0), cv::Point(image.cols - 1, image.rows - 1), cv::Scalar(0, 255, 0), 2);
        //cv::line(image, cv::Point(0, image.rows - 1), cv::Point(image.cols - 1, 0), cv::Scalar(0, 255, 0), 2);

        // for (int y = 0; y < image.rows; y++) {
        //     for (int x = 0; x < image.cols; x++) {
        //         if (y==x)
        //             image.at<Vec3b>(Point(y, x)) = Vec3b(0, 0, 0);
        //     }
        // }
        for (int y = 1; y < image.rows; y++){
            int x = image.cols * (y-image.rows) / image.rows;
            image.at<Vec3b>(Point(x, y)) = Vec3b(0, 0, 255);
            image.at<Vec3b>(Point(x+1, y)) = Vec3b(0, 0, 255);
            image.at<Vec3b>(Point(x+2, y)) = Vec3b(0, 0, 255);
        }

        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(output_msg);

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

    // image_transport::Publisher pub = it.advertise("pixel_output_img", 1);
    
    pub = it.advertise("pixel_img_output", 1);

    image_transport::Subscriber sub = it.subscribe("/video_frames", 1, imageCallback);
    ros::spin();

	return 0;
}