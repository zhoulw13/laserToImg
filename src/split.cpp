/*************************************************************************
	> File Name: split.cpp
	> Author: 
	> Mail: 
	> Created Time: 2015年04月03日 星期五 16时07分58秒
 ************************************************************************/

#include<iostream>
using namespace std;
#include"ros/ros.h"
#include"std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <sstream>
#include <fstream>
using namespace cv;

int main(int argc, char** argv){
	ros::init(argc, argv, "camera");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	int x;
	int cell_size = 64;
	ofstream outfile("./data/depth_train.txt", std::ofstream::out | std::ofstream::app);
	ifstream infile("index.txt");
	int temp, num, pre_temp, pre_num;
	infile >> num >> pre_num;
	infile >> x;
	for(int index = pre_num; index <= num; index++){
        std::stringstream sstm;

        sstm << "./data/image/" << index << ".jpg";
		Mat src = imread(sstm.str().c_str(), 0);
        //ROS_INFO("assert 1");
		if(src.rows == 0){
			//ROS_INFO("%d", i);
			continue;
		}

		Mat points = Mat::zeros(0, 0, CV_32FC1);
		std::stringstream sstm_pt;
        sstm_pt << "./data/point/" << index << ".yml";
		cv::FileStorage storage2(sstm_pt.str().c_str(), cv::FileStorage::READ);
		storage2["point"] >> points;
		storage2.release();

		int pixel_h = src.rows;
		int pixel_w = src.cols;
		int height = pixel_h / cell_size;
		int width = pixel_w / cell_size;
		int row = points.rows;
		int j_next = 0;
		int k;
		for (int i = pixel_w - 1, j = 0; i >= cell_size; i -= cell_size / 2, j = j_next){
			float y_sum = 0;
			bool next_found = false;
			k = 0;
			float depth = 0;
			while (points.at<float>(j, 0) > (i - cell_size)){
				y_sum += points.at<float>(j, 1);
				depth += points.at<float>(j, 2);
				if (!next_found && points.at<float>(j, 0) < i - (cell_size / 2))
					next_found = true; j_next = j;
				j++; k++;
			}
            //ROS_INFO("y_sum %lf", y_sum);
            if (y_sum <= 0)
                continue;
			y_sum /= k;
			depth = round(depth / k);
			/*if (y_sum + 2 * cell_size < pixel_h)
				y = y_sum + cell_size;
			else
				y = y_sum - cell_size;*/
			CvRect rect = cvRect(int(i - cell_size), int(y_sum-cell_size), cell_size, cell_size);
            //ROS_INFO("%d %d %d %d", rect.x, rect.y, int(y_sum), cell_size);
			Mat roi = src(rect);
            //ROS_INFO("assert 2 %d", x);
			std::stringstream sstm_pic;
			sstm_pic << "./data/image_split/" << x << ".jpg";
			imwrite(sstm_pic.str().c_str(), roi);
            //cvSaveImage(sstm_pt)

			outfile << x++ << ".jpg " << depth << endl;
		}

		if(!ros::ok())
			break;
	}
	ofstream outfile_1("index.txt");
	outfile_1 << num << " " << pre_num << endl;
	outfile_1 << x << endl;
}
