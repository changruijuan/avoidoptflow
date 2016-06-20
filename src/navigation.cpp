
#include "stdafx.h"
#include <cv.h>
#include <highgui.h>
#include "optutil.h"
#include "opticalflow.h"
#include "navigation.h"
#include "optcvmatutil.h"
#include "optfeatureutil.h"
#include "optmatutil.h"
#include "motioncolor.h"
#include "optdrawflow.h"
#include "optlearning.h"

using namespace cv;
using namespace std;

//利用左右光流平衡返回左1右2前3停止4，为金字塔光流算法
float imgFeatureStrategic(ImgFeatureFunType funtype,IplImage* imgprev, IplImage* imgcurr, IplImage* imgdst, Mat &color, int strategic){
	//1.计算光流
	IplImage* imgprev_1 = imgResize(imgprev);
	IplImage* imgcurr_1 = imgResize(imgcurr);
	CvPoint2D32f* cornersprev_11 = new CvPoint2D32f[ MAX_CORNERS ];
	CvPoint2D32f* cornerscurr_11 = new CvPoint2D32f[ MAX_CORNERS ];
    char track_status_11[MAX_CONTOUR];
    float k = funtype(imgprev_1, imgcurr_1, cornersprev_11, cornerscurr_11,cvRect(0,0,WIDTH/2,HEIGHT), track_status_11);
	CvPoint2D32f* cornersprev_12 = new CvPoint2D32f[ MAX_CORNERS ];
	CvPoint2D32f* cornerscurr_12 = new CvPoint2D32f[ MAX_CORNERS ];
    char track_status_12[MAX_CONTOUR];
    funtype(imgprev_1, imgcurr_1, cornersprev_12, cornerscurr_12,cvRect(WIDTH/2,0,WIDTH/2,HEIGHT), track_status_12);

	float result  = 0;
	if ((strategic >> 0 & 1) == 1) //balance
	{
        result = balanceForFeatureCvPoint(cornersprev_11, cornerscurr_11, cornersprev_12, cornerscurr_12, imgdst, k, track_status_11, track_status_12);
		printf("balance result : %lf\n", result);
	}    
	if ((strategic >> 1 & 1) == 1) //draw optflow with grap
	{
        drawFlowForFeatureCvPoint(cornersprev_11,cornerscurr_11, imgdst, track_status_11);
        drawFlowForFeatureCvPoint(cornersprev_12,cornerscurr_12, imgdst, track_status_12, false);
	}
	if ((strategic >> 2 & 1) == 1) //mation to color
	{
        motionToColor(cornersprev_11, cornerscurr_11, cornersprev_12, cornerscurr_12, color, track_status_11, track_status_12);
	}
	if ((strategic >>3 & 1) == 1)//get speed
	{ 
        getSpeedFromFlow(cornersprev_11, cornerscurr_11, cornersprev_12, cornerscurr_12, imgdst);
	}
	if ((strategic >>4 & 1) ==1 ) //draw flow without zero
	{
        drawFlowWithoutZero(cornersprev_11, cornerscurr_11, imgdst, track_status_11, true);
        drawFlowWithoutZero(cornersprev_12, cornerscurr_12, imgdst, track_status_12, false);
	}
	cvReleaseImage(&imgprev_1);
	cvReleaseImage(&imgcurr_1);
	free(cornerscurr_11);
	free(cornersprev_11);
	free(cornerscurr_12);
	free(cornersprev_12);
	return result;
}

float imgFeatAllStrategic(ImgFeatAllFunType funtype,IplImage* imgprev, IplImage* imgcurr, IplImage* imgdst, int strategic){
	IplImage* imgprev_1 = imgResize(imgprev);
	IplImage* imgcurr_1 = imgResize(imgcurr);
	
	CvPoint2D32f* cornersprev = new CvPoint2D32f[ MAX_CORNERS*2 ];
	CvPoint2D32f* cornerscurr = new CvPoint2D32f[ MAX_CORNERS*2 ];
    char track_status[MAX_CONTOUR];

    float k = funtype(imgprev_1, imgcurr_1, cornersprev, cornerscurr, track_status);
	//	funtype(imgprev_1, imgcurr_1, cornersprev_12, cornerscurr_12,cvRect(WIDTH/2,0,WIDTH/2,HEIGHT));

	float result  = 0;
	if ((strategic >> 0 & 1) == 1) //balance
	{
        result  = balanceForFeatureCvPoint(cornersprev, cornerscurr, imgdst, k, track_status);
		printf("balance result : %lf\n", result);
	}    
	if ((strategic >> 1 & 1) == 1) //draw optflow with grap
	{
        drawFlowForFeatAllCvPoint(cornersprev, cornerscurr, imgdst, track_status);
	}
	if ((strategic >> 2 & 1) == 1) //mation to color
	{
		//
	}
	if ((strategic >>3 & 1) == 1)//get speed
	{ 
		//
	}
	if ((strategic >>4 & 1) ==1 ) //draw flow without zero
	{
        drawFlowWithoutZero(cornersprev, cornerscurr, imgdst, track_status);
	}
	if ((strategic>> 5 & 1) == 1) //motion to gray 32
	{
		//
	}
	if ((strategic>> 6 & 1) == 1) //get flowTag result  64
	{ 
		//
	}
	if ((strategic>> 7 & 1) == 1) //get ttcTag result 128
	{ 
		//
	}
	if ((strategic>> 8 & 1) == 1) //get feature property(number, OF distance, OF plusminus) result 256
	{ 
		learningFeature(cornersprev,cornerscurr,imgdst);
	}
//	float result  = balanceForFeatureCvPoint(cornersprev, cornerscurr, imgdst, k);
//	drawFlowForFeatureCvPoint(cornersprev, cornerscurr, imgdst);  

	cvReleaseImage(&imgprev_1);
	cvReleaseImage(&imgcurr_1);

	free(cornerscurr);
	free(cornersprev);
	return result;
}

float imgStrategic(ImgFunType funtype, IplImage* imgprev_1, IplImage* imgcurr_1, IplImage* imgprev, IplImage* imgdst, Mat &color, Mat &gray, int strategic){
	//计算光流初始化变量
	CvMat *velx, *vely;
	velx = cvCreateMat(HEIGHT, WIDTH, CV_32FC1);
	vely = cvCreateMat(HEIGHT, WIDTH, CV_32FC1);
	cvSetZero(velx);
	cvSetZero(vely);
	//计算光流
	float k = funtype(imgprev_1,imgcurr_1,velx,vely);
	float result  = 0;
    if ((strategic >> 0 & 1) == 1) //balance 1 for tun
	{
        result = balanceForDenseCvMat(velx, vely, imgprev, imgdst, k);
	}    
	if ((strategic >> 1 & 1) == 1) //draw optical flow with gap 2
	{
		drawFlowForDenseCvMat(velx, vely, imgdst);
	}
	if ((strategic >> 2 & 1) == 1) //motion to color 4
	{
        motionToColor(velx, vely, color);
	}
    if ((strategic>> 3 & 1) == 1) //ttc 8 for cross
    {
        result = balanceWithTTCDenseCvMat(velx, vely, imgprev, imgdst, k);
    }
	if ((strategic>> 4 & 1) == 1) //draw flow without zero 16
	{ 
		drawFlowWithoutZero(velx, vely, imgdst);
	}
	if ((strategic>> 5 & 1) == 1) //motion to gray 32
	{
		motionToGray(velx, vely, gray);
		if (IS_SIMILARITY)
		{
			IplImage gray_1(gray);
			IplImage* prev_1 = cvCloneImage(imgprev);
			double tmp = huSimilarity(prev_1, &gray_1);
			printf("Similarity ： %lf\n", tmp);
			char buffer[50];
			sprintf(buffer, "%lf \n", tmp);
			writeFile(buffer);
			cvReleaseImage(&prev_1);
		}
	}
	if ((strategic>> 6 & 1) == 1) //get flowTag result  64
	{ 
		float compareFlow = flowTagForDenseCvMat(imgprev, imgdst, velx, vely);
		/*char buffer[50];
		sprintf(buffer, "%lf \n", compareFlow);
		writeFile(buffer);*/
	}
	if ((strategic>> 7 & 1) == 1) //get ttcTag result 128
	{ 
		float compareTTC = ttcTagForDenseCvMat(imgprev, imgdst, velx, vely);
		/*char buffer[50];
		sprintf(buffer, "%lf \n", compareTTC);
		writeFile(buffer);*/
	}
    if ((strategic>> 8 & 1) == 1) //learning result 256
    {
        learningCvMat(imgprev, imgdst, velx, vely);
    }
    if ((strategic>> 9 & 1) == 1) //get speed 512
    {
        getSpeedFromFlow(velx, vely, imgdst);
    }


	cvReleaseMat(&velx);
	cvReleaseMat(&vely);

	return result;
}

float matStrategic(MatFunType funtype, Mat frameprev_1, Mat framecurr_1, Mat &frameprev, Mat &framedst, Mat &color, Mat &gray, int strategic, bool issf){
	Mat flow;
	flow = funtype(frameprev_1, framecurr_1, flow);
    float k = FB_K;
    if(issf){
        k = SF_K;
    }
    float result = 0;
	if ((strategic >> 0 & 1) == 1) //balance
	{
		result = balanceForDenseMat(flow, framedst, k);
		/*printf("balance result : %lf\n", result);
		char buffer[50];
		sprintf(buffer, "%lf \n", result);
		writeFile(buffer);*/
	}
	if ((strategic >> 1 & 1) == 1) //draw optical flow with gap
	{
		drawFlowForDenseMat(flow, framedst);
	}
	if ((strategic >> 2 & 1) == 1) //motion to color
	{
		motionToColor(flow, color);
	}
	if ((strategic>> 3 & 1) == 1)//get speed
	{ 
		getSpeedFromFlow(flow, framedst);
	}
	if ((strategic>> 4 & 1) == 1) //draw flow without zero
	{ 
		drawFlowWithoutZero(flow, framedst);
	}
	if ((strategic>> 5 & 1) == 1) //motion to gray
	{
		Mat intergray;
		intergray.create(HEIGHT, WIDTH, CV_8UC1);
		motionToGray(flow, gray);
		if (IS_SIMILARITY)
		{
			IplImage prev_1(frameprev);
			IplImage gray_1(gray);
			double tmp = huSimilarity(&prev_1, &gray_1);
			printf("%lf\n", tmp);
			char buffer[50];
			sprintf(buffer, "%lf \n", tmp);
			writeFile(buffer);
		}
	}
	if ((strategic>> 6 & 1) == 1) //get flowTag result
	{ 
		float compareFlow = flowTagForDenseMat(frameprev, framedst, flow);
		/*char buffer[50];
		sprintf(buffer, "%lf \n", compareFlow);
		writeFile(buffer);*/
	}
	if ((strategic>> 7 & 1) == 1) //get ttcTag result
	{ 
		float compareTTC = ttcTagForDenseMat(frameprev, framedst, flow);
		/*char buffer[50];
		sprintf(buffer, "%lf \n", compareTTC);
		writeFile(buffer);*/
	}
	flow.release();
	return result;
}

