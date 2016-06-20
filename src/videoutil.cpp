/*
*  FILE videoutil.h
*  AUTHOR Sarah
*  DATE 2016/01/8 20:10
*  TODO: 使用不同的光流方法处理视频.
*/
#include "stdafx.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv.h>
#include <highgui.h>
#include "opencv2/legacy/legacy.hpp"
#include <math.h>

#include <cstdio>
#include <iostream>

#include "optutil.h"
#include "opticalflow.h"
#include "navigation.h"
#include "video.h"
#include "test.h"

using namespace cv;
using namespace std;

void blurVideo(String inputfile, int func){
	VideoCapture capture(inputfile.c_str());
	if (!capture.isOpened())
	{
		printf("Error: Can't open video.\n");
		exit(-1);
	}
	Mat frame, frame_dst;
	capture.read(frame);

	VideoWriter writer("../video/blur.avi",CV_FOURCC('M', 'J', 'P', 'G'),20,cvSize(frame.cols, frame.rows),true);

	while(true)
	{
		capture.read(frame);
		if (frame.empty())
		{
			printf("video over.\n");
			break;
		}
		switch (func){
		case 1: //均值模糊 
			blur(frame, frame_dst, Size(KERNEL_LENGTH, KERNEL_LENGTH));
			break;
		case 2: //高斯模糊 
			GaussianBlur(frame, frame_dst, Size(KERNEL_LENGTH, KERNEL_LENGTH),0);
			break;
		case 3: //中值模糊  
			medianBlur(frame, frame_dst, KERNEL_LENGTH);
			break;
		case 4: //双边滤波 
			bilateralFilter(frame, frame_dst, KERNEL_LENGTH, KERNEL_LENGTH*2, KERNEL_LENGTH/2);
			break;
		}
		imshow("origin", frame);
		imshow("blur", frame_dst);
		writer.write(frame_dst);

		int key_pressed = waitKey(1);
		if ( (char)key_pressed ==  27 )
		{
			break;
		}
	}
	frame.release();
	frame_dst.release();
	writer.release();
}

void quickVideo(String inputfile, int skip){
	CvCapture *input_video = cvCreateFileCapture(inputfile.c_str());
	if (input_video == NULL)
	{
		printf("Error: Can't open video.\n");
		exit(-1);
	}
	IplImage *img = NULL;
    img = cvQueryFrame( input_video );

	CvVideoWriter* writer = cvCreateVideoWriter("../video/quick.avi", CV_FOURCC('M', 'J', 'P', 'G'),15,cvSize(img->width, img->height),1);
	int count = 0;

	while(true)
	{
		img = cvQueryFrame( input_video );
		if (img == NULL)
		{
			printf("video over.\n");
			break;
		}

		if (count == skip)
		{
			cvNamedWindow ("quick", 1);
			cvShowImage ("quick", img);
			cvWriteFrame(writer, img);
			count = 0;
		}else{
			count ++;
		}

		int key_pressed = waitKey(1);
		if ( (char)key_pressed ==  27 )
		{
			break;
		}
	}
	cvReleaseVideoWriter(&writer);
	cvReleaseImage(&img);
}

int video2Image(String inputfile){
	CvCapture *input_video = cvCreateFileCapture(inputfile.c_str());
	if (input_video == NULL)
	{
		printf("Error: Can't open video.\n");
		exit(-1);
	}
	IplImage *img = NULL;
	img = cvQueryFrame( input_video );

	int count = 0;

	while(true)
	{
		img = cvQueryFrame( input_video );
		if (img == NULL)
		{
			printf("video over.\n");
			break;
		}

		char buffer[50];
		sprintf(buffer, "../video/image/%d.jpg", count);
		printf("%s\n", buffer);
		cvSaveImage(buffer, img);
		count++;

		cvNamedWindow ("img", 1);
		cvShowImage ("img", img);

		int key_pressed = waitKey(1);
		if ( (char)key_pressed ==  27 )
		{
			break;
		}
	}
	cvReleaseImage(&img);
	return count;
}

int video2Image(String inputfile, String outpfile){
    printf("%s\n", inputfile.c_str());
	CvCapture *input_video = cvCreateFileCapture(inputfile.c_str());
	if (input_video == NULL)
	{
		printf("Error: Can't open video.\n");
		exit(-1);
	}
	IplImage *img = NULL;
	img = cvQueryFrame( input_video );

	int count = 0;

	while(true)
	{
		img = cvQueryFrame( input_video );
		if (img == NULL)
		{
			printf("video over.\n");
			break;
		}

		char buffer[50];
        sprintf(buffer, "%s/%d.jpg", outpfile.c_str(),count);
		printf("%s\n", buffer);
		cvSaveImage(buffer, img);
		count++;

		cvNamedWindow ("img", 1);
		cvShowImage ("img", img);

		int key_pressed = waitKey(1);
		if ( (char)key_pressed ==  27 )
		{
			break;
		}
	}
	cvReleaseImage(&img);
	return count;
}

void resizeVideo(String inputfile, int width, int height){
	CvCapture *input_video = cvCreateFileCapture(inputfile.c_str());
	if (input_video == NULL)
	{
		printf("Error: Can't open video.\n");
		exit(-1);
	}
	IplImage *img = NULL;
	img = cvQueryFrame( input_video );

	CvVideoWriter* writer = cvCreateVideoWriter("../video/resize.avi", CV_FOURCC('M', 'J', 'P', 'G'),15,cvSize(width, height),1);
	int count = 0;

	while(true)
	{
		img = cvQueryFrame( input_video );
		if (img == NULL)
		{
			printf("video over.\n");
			break;
		}

		IplImage *img_dst = cvCreateImage(cvSize(WIDTH, HEIGHT), img->depth , img->nChannels);
		cvResize(img, img_dst);

		cvNamedWindow ("resize", 1);
		cvShowImage ("resize", img_dst);
		cvWriteFrame(writer, img_dst);

		int key_pressed = waitKey(1);
		if ( (char)key_pressed ==  27 )
		{
			cvReleaseImage(&img_dst);
			break;
		}
		cvReleaseImage(&img_dst);
	}
	cvReleaseVideoWriter(&writer);
	cvReleaseImage(&img);
}

int video2ResizeImage(String inputfile, int width, int height){
	resizeVideo(inputfile, width, height);
	return video2Image(inputfile);
}

//待完善
void image2Video(int start, int end, int roll, String inputDir){

    Mat img = imread("/home/sarah/catkin_ws/src/avoid_optflow/video/image/0.jpg");
    VideoWriter writercolor("/home/sarah/catkin_ws/src/avoid_optflow/video/flow.avi",CV_FOURCC('M', 'J', 'P', 'G'),20,cvSize(img.cols, img.rows),true);

	Point2f center = Point2f(img.cols/2, img.rows/2);

	if (start < end)
	{
		for (int i = start; i < end; i++)
		{
            char buffer[200];
            sprintf(buffer, "%s/%d.jpg", inputDir.c_str(),i);
			img = imread(buffer);
            printf("%s\n", buffer);

			//Mat img_dst = getRotationMatrix2D(center, roll*90, 1.0);
			//imshow("roll",img_dst);
			writercolor.write(img);

			int key_pressed = waitKey(1);
			if ( (char)key_pressed ==  27 )
			{
				break;
			}
		}
	}else{
		for (int i = start; i >= end; i--)
		{
            char buffer[200];
            sprintf(buffer, "%s/%d.jpg", inputDir.c_str(),i);
            printf("%s\n", buffer);

			//Mat img_dst = getRotationMatrix2D(center, roll*90, 1.0);
			//imshow("roll",img_dst);
			writercolor.write(img);

			int key_pressed = waitKey(1);
			if ( (char)key_pressed ==  27 )
			{
				break;
			}
		}
	}
	img.release();
	writercolor.release();
}


void video2RotateVideo(String inputfile, int roll){
	int count = video2Image(inputfile);
	image2Video(0, count, roll,"../video/image/");
}



