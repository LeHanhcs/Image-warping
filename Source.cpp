#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <conio.h>

#include<string.h>
#include "Header.h"

string first_image_name;
string second_image_name;
string new_image_name;

//using namespace std;

struct Image {
	int frame_index;
	IplImage *image;
	IplImage *new_image;
	IplImage *test_image;

	Image(int i);
	CvScalar bilinear(IplImage *image, double X, double Y);
	void Warp(IplImage *left_img, IplImage *right_img, IplImage *goal_img);
};
Image::Image(int i) {
	frame_index = i;
	CvSize ImageSize = cvSize(width, height);
	new_image = cvCreateImage(ImageSize, IPL_DEPTH_8U, 3);
	test_image = cvCreateImage(ImageSize, IPL_DEPTH_8U, 3);
}
CvScalar Image::bilinear(IplImage *image, double X, double Y) {
	int x_floor = (int)X;
	int y_floor = (int)Y;
	int x_ceil = x_floor + 1;
	int y_ceil = y_floor + 1;
	double a = X - x_floor;
	double b = Y - y_floor;
	if (x_ceil >= width - 1)
		x_ceil = width - 1;
	if (y_ceil >= height - 1)
		y_ceil = height - 1;
	CvScalar output_scalar;
	CvScalar leftdown = cvGet2D(image, y_floor, x_floor);
	CvScalar lefttop = cvGet2D(image, y_ceil, x_floor);
	CvScalar rightdown = cvGet2D(image, y_floor, x_ceil);
	CvScalar righttop = cvGet2D(image, y_ceil, x_ceil);
	for (int i = 0; i < 4; i++) {
		output_scalar.val[i] = (1 - a)*(1 - b)*leftdown.val[i] + a*(1 - b)*rightdown.val[i] + a*b*righttop.val[i] + (1 - a)*b*lefttop.val[i];
	}
	return output_scalar;
}
void Image::Warp(IplImage *left_img, IplImage *right_img, IplImage *goal_img) {

	double ratio = (double)(frame_index + 1) / (frame_count + 1);
	IplImage *ori_leftImage, *ori_rightImage;
	ori_leftImage = cvLoadImage(first_image_name.c_str());
	ori_rightImage = cvLoadImage(second_image_name.c_str());

	CvScalar pixel_src;
	CvScalar pixel_dst;
	CvScalar pixel_goal;

	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			CvPoint2D32f X_dst(x,y);
			CvPoint2D32f X_src;
			CvPoint2D32f D;
			CvPoint2D32f DSUM(0, 0);
			double weightSum = 0;

			/*CvPoint2D32f dst_point;
			dst_point.x = x;
			dst_point.y = y;
			double leftXSum_x = 0.0;
			double leftXSum_y = 0.0;
			double leftWeightSum = 0.0;
			double rightXSum_x = 0.0;
			double rightXSum_y = 0.0;
			double rightWeightSum = 0.0;*/

			for (int i = 0; i < pairs.size(); i++) {
				Line PQ = pairs[i].rightLine;
				Line PQ_src = pairs[i].leftLine;
				double u_1 = PQ.Getu(X_dst);
				double v_1 = PQ.Getv(X_dst);
				X_src = PQ.Get_Point(u_1, v_1);
				D.x = X_dst.x - X_src.x;
				D.y = X_dst.y - X_src.y;
				double w = PQ.Get_Weight(X_dst);
				DSUM.x += D.x*w;
				DSUM.y += D.y*w;
				weightSum += w;
			}
			X_src.x = X_dst.x + (DSUM.x / weightSum);
			X_src.y = X_dst.y + (DSUM.y / weightSum);

			//lay pixel
			pixel_src = cvGet2D(leftImage, y, x);
			pixel_dst = cvGet2D(rightImage, y, x);
			pixel_goal = cvGet2D(goal_img, y, x);
			
			pixel_goal.val[0] = (1 - ratio)*pixel_src.val[0] + ratio*pixel_dst.val[0];
			pixel_goal.val[1] = (1 - ratio)*pixel_src.val[1] + ratio*pixel_dst.val[1];
			pixel_goal.val[2] = (1 - ratio)*pixel_src.val[2] + ratio*pixel_dst.val[2];
			cvSet2D(goal_img, y, x, pixel_goal);
			cout << "?????" << endl;
			/*
				Line src_line = pairs[i].leftLine;
				Line dst_line = pairs[i].warpLine[frame_index];				
				double new_u = dst_line.Getu(dst_point);
				double new_v = dst_line.Getv(dst_point);

				CvPoint2D32f src_point = src_line.Get_Point(new_u, new_v);
				double src_weight = dst_line.Get_Weight(dst_point);
				leftXSum_x = leftXSum_x + (double)src_point.x * src_weight;
				leftXSum_y = leftXSum_y + (double)src_point.y * src_weight;
				leftWeightSum = leftWeightSum + src_weight;

				//¥k¹Ï¬°¨Ó·½ 
				src_line = pairs[i].rightLine;

				new_u = dst_line.Getu(dst_point);
				new_v = dst_line.Getv(dst_point);

				src_point = src_line.Get_Point(new_u, new_v);
				src_weight = dst_line.Get_Weight(dst_point);
				rightXSum_x = rightXSum_x + (double)src_point.x * src_weight;
				rightXSum_y = rightXSum_y + (double)src_point.y * src_weight;
				rightWeightSum = rightWeightSum + src_weight;
			}
			double left_src_x = leftXSum_x / leftWeightSum;
			double left_src_y = leftXSum_y / leftWeightSum;
			double right_src_x = rightXSum_x / rightWeightSum;
			double right_src_y = rightXSum_y / rightWeightSum;

			
			if (left_src_x<0)
				left_src_x = 0;
			if (left_src_y<0)
				left_src_y = 0;
			if (left_src_x >= width)
				left_src_x = width - 1;
			if (left_src_y >= height)
				left_src_y = height - 1;
			if (right_src_x<0)
				right_src_x = 0;
			if (right_src_y<0)
				right_src_y = 0;
			if (right_src_x >= width)
				right_src_x = width - 1;
			if (right_src_y >= height)
				right_src_y = height - 1;
				*/			
			
			//CvScalar left_scalar = cvGet2D(leftImage, X_src.y, X_src.x);
			//CvScalar right_scalar=cvGet2D(ori_rightImage,X_dst.y, X_dst.x);
			//CvScalar left_scalar = bilinear(ori_leftImage, X_src.x, X_src.y);
			//CvScalar right_scalar = bilinear(ori_rightImage, right_src_x, right_src_y);
			/*CvScalar new_scalar;
			new_scalar.val[0] = (1 - ratio)*left_scalar.val[0] + ratio*right_scalar.val[0];
			new_scalar.val[1] = (1 - ratio)*left_scalar.val[1] + ratio*right_scalar.val[1];
			new_scalar.val[2] = (1 - ratio)*left_scalar.val[2] + ratio*right_scalar.val[2];
			new_scalar.val[3] = (1 - ratio)*left_scalar.val[3] + ratio*right_scalar.val[3];
			cvSet2D(new_image, y, x, new_scalar);*/
			//cvSet2D(test_image, y, x, left_scalar);//test
		}
	}
	//cout << "Done 2";

	char win_name[16];
	char img_name[50];
	sprintf_s(win_name, "frame[%d]", frame_index);
	sprintf_s(img_name, "%s_%d.jpg", new_image_name.c_str(), frame_index);
	cvSaveImage(img_name, goal_img);
	//cvShowImage(test_name,test_image);
	//cvSaveImage("abc", goal_img);
//	cvShowImage("re",rightImage);
	return;
}
//======================================

void Load_Frame() {
	string n = "";
	cvNamedWindow("Warping", 1);
	for (int i = 0; i < 30; i++) {
		result = cvLoadImage("A c d", 1);
		cvShowImage("Warping", result);
	}
}

void runWarp()
{
	frame_count = 20;
	for (int i = 0; i<frame_count; i++)
	{
		Image curImage = Image(i);
		curImage.Warp(leftImage, rightImage, goal);
	}
}
void main(int argc, char** argv) {

	cout << "--------------Input value for a, b, p --------------" << endl;
	cout << "Input a (greater then 0):  ";
	cin >> parameter_a;
	cout << "Input b in range [0.5; 2.0]: ";
	cin >> parameter_b;
	cout << "Input b in range [0; 1]: ";
	cin >> parameter_p;
	leftImage = cvLoadImage("img1.jpg",1);
	rightImage = cvLoadImage("img2.jpg",1);
	goal = cvLoadImage("img3.jpg", 1);
	height = leftImage->height;
	width = leftImage->width;
	leftImageTmp = cvLoadImage("img1.jpg", 1);
	rightImageTmp = cvLoadImage("img2.jpg", 1);
	cvNamedWindow("Source Image", 1);
	cvMoveWindow("Source Image", 10, 10);
	cvNamedWindow("Destination Image", 1);
	cvMoveWindow("Destination Image", 300, 10);

	cvSetMouseCallback("Source Image", on_mousel, 0);
	cvSetMouseCallback("Destination Image", on_mouser, 0);

	cvShowImage("Source Image", leftImage);
	cvShowImage("Destination Image", rightImage);

	while (true)
	{
		key = cvWaitKey(0);
		if (key == 'c')
			counter = counter + 2;
		else if (key == 'r')
			//runWarp();
			Load_Frame();
		else if (key == 'q')
			break;
	}

	cvDestroyWindow("Source Image");
	cvDestroyWindow("Destination Image");
}
