#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>

cv::Mat Frame;

void img_callback(const sensor_msgs::Image& msg)
{
  cv::Mat src = cv::Mat(480, 640, CV_8UC3, const_cast<uchar *>(&msg.data[0]), msg.step);
  cv::cvtColor(src, Frame, cv::COLOR_RGB2BGR);
}


int main(int argc, char** argv){

	//cv::Mat Frame;
    ros::init(argc, argv, "cam_tune");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, img_callback);

    cv::Mat grayimage;
    cv::Mat blurimage;
    cv::Mat cannyimage;
    cv::Mat roiimage;

    std::vector<cv::Vec4i> linesP;

	std::vector<int> left;
	std::vector<int> right;
    
	while (ros::ok())
	{
        ros::spinOnce();

        if (Frame.empty() || Frame.size() != cv::Size(640, 480))
            continue;


		cv::cvtColor(Frame, grayimage, cv::COLOR_BGR2GRAY); 
		cv::GaussianBlur(grayimage, blurimage, cv::Size(5, 5), 0);
		cv::Canny(blurimage, cannyimage, 150, 200, 3, true);

		roiimage = cannyimage(cv::Rect(0, 220, 640, 70));  // x, y, width, height

		cv::HoughLinesP(roiimage, linesP, 1, CV_PI / 180, 40, 40, 5);

		cv::circle(Frame, cv::Point(320, 255), 5, cv::Scalar(0,0,255), 1, 8, 0);  // 220 +(70 / 2)

		for (size_t i = 0; i < linesP.size(); i++)
		{
			cv::Vec4i l = linesP[i];
			cv::line(Frame, cv::Point(l[0], l[1] + 220), cv::Point(l[2], l[3] + 220), cv::Scalar(0, 255, 0), 2, 8);

			if((l[0]+l[2]) / 2 < 320)
			{
				left.push_back(l[2]);
			}
			else
			{
				right.push_back(l[0]);
			}
		}

		int lef = 0, rig = 0;

		for(int i = 0;i<left.size();i++)
		{
			lef = lef + left[i];
		}
		for(int i = 0;i<right.size();i++)
		{
			rig = rig + right[i];
		}

		if(left.size()>0 && right.size() > 0)
		{
			cv::circle(Frame, cv::Point((lef / left.size() + rig / right.size()) / 2, 255), 8, cv::Scalar(0,255,255), 1, 8, 0);

		}

		while(!left.empty())
			left.pop_back();
		while(!right.empty())
			right.pop_back();


		//cv::imshow("gray", grayimage);
		cv::imshow("canny", cannyimage);
		//cv::imshow("blur", blurimage);
		cv::imshow("roi", roiimage);
		cv::imshow("HoughImage", Frame);

		if (cv::waitKey(25) == 27)  // esc 
			break;

	}


    return 0;
}