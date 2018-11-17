#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "GPIOlib.h"

using namespace std;
using namespace cv;
using namespace GPIO;
const string CAM_PATH = "/devideo0";
const string MAIN_WINDOW_NAME = "Processed Image";
const string CANNY_WINDOW_NAME = "Canny";

const int CANNY_LOWER_BOUND = 50;
const int CANNY_UPPER_BOUND = 250;
const int HOUGH_THRESHOLD = 80;

const int REC_WIDTH = 500;
const int REC_HEIGHT = 500;

void mask_depth(Mat &image,Mat& th,int throld=HOUGH_THRESHOLD)
{
	int nr = image.rows; // number of rows 
	int nc = image.cols; // number of columns 
	for (int i = 0; i<nr; i++)
	{

	for (int j = 0; j<nc; j++) {
	if (image.at<ushort>(i, j)>throld)
	th.at<ushort>(i, j) = 0;
	}
	}

}
//障碍检测
vector<vector<Point> > find_obstacle(Mat &depth, int thresh = 20, int max_thresh = 255, int area = 500)
{
	Mat dep;
	depth.copyTo(dep);
	mask_depth(depth, dep, 1000);
	dep.convertTo(dep, CV_8UC1, 1.0 / 16);
	//imshow("color", color);
	imshow("depth", dep);
	Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));//核的大小可适当调整
Mat out;
	//进行开操作
morphologyEx(dep, out, MORPH_OPEN, element);
	//dilate(dhc, out, element);

//显示效果图
imshow("opencv", out);
	Mat src_copy = dep.clone();
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	RNG rng(12345);
	/// 对图像进行二值化
threshold(dep, threshold_output, thresh, 255, CV_THRESH_BINARY);
	//mask_depth(src, threshold_output);
	/// 寻找轮廓
findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

/// 对每个轮廓计算其凸包
vector<vector<Point> >hull(contours.size());
	vector<vector<Point> > result;
	for (int i = 0; i < contours.size(); i++)
	{
	convexHull(Mat(contours[i]), hull[i], false);

	}

/// 绘出轮廓及其凸包
Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (int i = 0; i< contours.size(); i++)
	{
	if (contourArea(contours[i]) < area)//面积小于area的凸包，可忽略
continue;
	result.push_back(hull[i]);
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
	drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());
	}
	//展示找出障碍后的图片
	imshow("contours", drawing);
	return result;
}

//主方法
int main(int argc, char* argv[])
{
	//小车初始化
	init();
	//小车转正
	turnTo(0);
	VideoCapture capture(CAM_PATH);
	//If this fails, try to open as a video camera, through the use of an integer param
	if (!capture.isOpened())
	{
		capture.open(atoi(CAM_PATH.c_str()));
	}
	double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);			//the width of frames of the video
	double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);		//the height of frames of the video
	//采集摄像头
	cout << "摄像头 ";
	cout << dWidth + ' ' + dHeight;

	Mat image;
	while (true)
	{
		capture >> image;
		if (image.empty())
			break;
		//Set the ROI for the image
		//调整到合适的位置
		Rect roi(0, image.rows / 3, image.cols, image.rows / 3);
		Mat imgROI = image(roi);

		//Canny algorithm
		Mat contours;
		//展示初始图像
		Canny(imgROI, contours, CANNY_LOWER_BOUND, CANNY_UPPER_BOUND);
		vector<vector<Point> results;
		//进行避障处理
		results = find_obstacle(image,20,255,500);
		//根据results进行避障处理
	}
	return 0;
}

