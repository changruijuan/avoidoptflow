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

using namespace cv;
using namespace std;

/************************************************************************/
/* 将img转化为320*280图像，且进行灰度化处理                             */
/************************************************************************/
IplImage* imgResize(IplImage* img){
	IplImage* img_1 = cvCreateImage(cvSize(WIDTH,HEIGHT), img->depth , img->nChannels);
	cvResize(img, img_1);
	IplImage* img_2 = cvCreateImage(cvGetSize(img_1), IPL_DEPTH_8U, 1);
	cvCvtColor(img_1, img_2, CV_BGR2GRAY);
	cvReleaseImage(&img_1);
	return img_2;
}

//LK算法
float Lucaskanade(IplImage* imgprev, IplImage* imgcurr, CvMat* velx, CvMat* vely){
	CvSize winSize = cvSize(WINSIZE, WINSIZE);
	float start = (float)getTickCount();
	cvCalcOpticalFlowLK (imgprev, imgcurr, winSize, velx, vely);
	float timer = (getTickCount() - start) / getTickFrequency();
    //printf("cvCalcOpticalFlowLK : %lf sec\n", timer);
	if (IS_TIMER_WRITE_FILE)
	{
		char buffer[50];
		sprintf(buffer, "%f\n", timer);
		writeFile(buffer);
	}
	return LK_K;
}
//HS算法
float HornSchunck(IplImage* imgprev, IplImage* imgcurr, CvMat* velx, CvMat* vely){
	CvTermCriteria criteria = cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.1);
	float start = (float)getTickCount();
	cvCalcOpticalFlowHS (imgprev, imgcurr, 0, velx, vely, 100.0, criteria);
	float timer = (getTickCount() - start) / getTickFrequency();
    //printf("cvCalcOpticalFlowHS : %lf sec\n", timer);
	if (IS_TIMER_WRITE_FILE)
	{
		char buffer[50];
		sprintf(buffer, "%f\n", timer);
		writeFile(buffer);
	}
	return HS_K;
}

//BM算法
float BlockMatch(IplImage* imgprev, IplImage* imgcurr, CvMat* velx, CvMat* vely){

	float start = (float)getTickCount();
	cvCalcOpticalFlowBM(imgprev, imgcurr, cvSize(1, 1), cvSize(1, 1), cvSize(12, 12), 0, velx, vely);
	//printf("cvCalcOpticalFlowBM : %lf sec\n", (getTickCount() - start) / getTickFrequency());
	float timer = (getTickCount() - start) / getTickFrequency();
    //printf("cvCalcOpticalFlowBM : %lf sec\n", timer);
	if (IS_TIMER_WRITE_FILE)
	{
		char buffer[50];
		sprintf(buffer, "%f\n", timer);
		writeFile(buffer);
	}
	return BM_K;
}

/************************************************************************/
/* 将mat转化为320*180图像，且进行灰度化处理                             */
/************************************************************************/
Mat matResize(Mat frame){
	Mat frame_1, frame_2;
	resize(frame,frame_1,Size(WIDTH,HEIGHT),0,0,1);
	cvtColor(frame_1, frame_2, CV_BGR2GRAY);
	frame_1.release();
	return frame_2;
}

/************************************************************************/
/* 将mat转化为320*180图像，不进行灰度化处理                            */
/************************************************************************/
Mat matColorResize(Mat frame){
	Mat frame_1;
	resize(frame,frame_1,Size(WIDTH,HEIGHT),0,0,1);
	return frame_1;
}

Mat SimpleFlow(Mat frameprev, Mat framecurr, Mat flow){
	float start = (float)getTickCount();
	calcOpticalFlowSF(frameprev, framecurr,
		flow,
		3, 2, 4, 4.1, 25.5, 18, 55.0, 25.5, 0.35, 18, 55.0, 25.5, 10);
	//printf("calcOpticalFlowSF : %lf sec\n", (getTickCount() - start) / getTickFrequency());
	float timer = (getTickCount() - start) / getTickFrequency();
    //printf("calcOpticalFlowSF : %lf sec\n", timer);
	if (IS_TIMER_WRITE_FILE)
	{
		char buffer[50];
		sprintf(buffer, "%f\n", timer);
		writeFile(buffer);
	}
	return flow;
}

Mat FarneBack(Mat frameprev, Mat framecurr, Mat flow){
	float start = (float)getTickCount();
	calcOpticalFlowFarneback(frameprev, framecurr, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
	//printf("calcOpticalFlowFarneback : %lf sec\n", (getTickCount() - start) / getTickFrequency());
	float timer = (getTickCount() - start) / getTickFrequency();
    //printf("calcOpticalFlowFarneback : %lf sec\n", timer);
	if (IS_TIMER_WRITE_FILE)
	{
		char buffer[50];
		sprintf(buffer, "%f\n", timer);
		writeFile(buffer);
	}
	return flow;
}

float PyrLK(IplImage* imgprev, IplImage* imgcurr,CvPoint2D32f* cornersprev,CvPoint2D32f* cornerscurr, CvRect rect, char* track_status){
	cvSetImageROI(imgprev, rect);
	IplImage* imgprev_1 = imgprev;
	cvSetImageROI(imgcurr, rect);
	IplImage* imgcurr_1 = imgcurr;
	IplImage* eig_image=cvCreateImage(cvSize(WIDTH/2,HEIGHT),IPL_DEPTH_32F,1);
	IplImage* tmp_image=cvCreateImage(cvSize(WIDTH/2,HEIGHT),IPL_DEPTH_32F,1);
	int corner_count = MAX_CORNERS;
	cvGoodFeaturesToTrack(
		imgprev_1,
		eig_image,
		tmp_image,
		cornersprev,
		&corner_count,
		0.01,
		5.0,
		0,
		3,
		0,
		0.04
		);
	cvFindCornerSubPix(
		imgprev_1,
		cornersprev,
		corner_count,
		cvSize(WINSIZE,WINSIZE),
		cvSize(-1,-1),
		cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03)
		);
	IplImage* pyrA = cvCreateImage( cvSize(WIDTH/2+8, HEIGHT/3), IPL_DEPTH_32F, 1 );
	IplImage* pyrB = cvCreateImage( cvSize(WIDTH/2+8, HEIGHT/3), IPL_DEPTH_32F, 1 );
	float start = (float)getTickCount();
	cvCalcOpticalFlowPyrLK(
		imgprev_1,
		imgcurr_1,
		pyrA,
		pyrB,
		cornersprev,
		cornerscurr,
		MAX_CORNERS,
		cvSize(WINSIZE, WINSIZE),
		5,
        track_status,
		NULL,
		cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
		0
		);
    //track_status 是一个数组，对应点在第二帧中找到，那该位置就值为1，找不到就值为0.
	float timer = (getTickCount() - start) / getTickFrequency();
    //printf("cvCalcOpticalFlowPyrLK : %lf sec\n", timer);
	if (IS_TIMER_WRITE_FILE){
		char buffer[50];
		sprintf(buffer, "%f\n", timer);
		writeFile(buffer);
	}
	//cvReleaseImage(&imgprev_1);
	//cvReleaseImage(&imgcurr_1);
	cvReleaseImage(&eig_image);
	cvReleaseImage(&tmp_image);
	cvReleaseImage(&pyrA);
	cvReleaseImage(&pyrB);
	cvResetImageROI(imgprev);
	cvResetImageROI(imgcurr);
	return PYRLK_K;
}


float PyrLK(IplImage* imgprev, IplImage* imgcurr,CvPoint2D32f* cornersprev,CvPoint2D32f* cornerscurr, char* track_status){
	IplImage* eig_image=cvCreateImage(cvSize(WIDTH,HEIGHT),IPL_DEPTH_32F,1);
	IplImage* tmp_image=cvCreateImage(cvSize(WIDTH,HEIGHT),IPL_DEPTH_32F,1);
	int corner_count = MAX_CORNERS*2;
	cvGoodFeaturesToTrack(
		imgprev,
		eig_image,
		tmp_image,
		cornersprev,
		&corner_count,
		0.01,
		5.0,
		0,
		3,
		0,
		0.04
		);
	cvFindCornerSubPix(
		imgprev,
		cornersprev,
		corner_count,
		cvSize(WINSIZE,WINSIZE),
		cvSize(-1,-1),
		cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03)
		);
	IplImage* pyrA = cvCreateImage( cvSize(WIDTH+8, HEIGHT/3), IPL_DEPTH_32F, 1 );
	IplImage* pyrB = cvCreateImage( cvSize(WIDTH+8, HEIGHT/3), IPL_DEPTH_32F, 1 );
	float start = (float)getTickCount();
	cvCalcOpticalFlowPyrLK(
		imgprev,
		imgcurr,
		pyrA,
		pyrB,
		cornersprev,
		cornerscurr,
		MAX_CORNERS*2,
		cvSize(WINSIZE, WINSIZE),
		5,
        track_status,
		NULL,
		cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),
		0
		);
    //printf("cvCalcOpticalFlowPyrLK : %lf sec\n", (getTickCount() - start) / getTickFrequency());
	cvReleaseImage(&eig_image);
	cvReleaseImage(&tmp_image);
	cvReleaseImage(&pyrA);
	cvReleaseImage(&pyrB);
	return PYRLK_K;
}
