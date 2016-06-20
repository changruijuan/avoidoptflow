#pragma once

#include "stdafx.h"

#include "optutil.h"

using namespace cv;
using namespace std;

void drawFlowWithoutZero(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, char* track_status, bool isleft);

void drawFlowWithoutZero(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, char* track_status);

void drawFlowWithoutZero(CvMat* velx, CvMat* vely, IplImage* imgdst);

void drawFlowWithoutZero(Mat flow, Mat &framedst);

void drawFlowForFeatureCvPoint(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, IplImage* imgdst, char* track_status, bool isleft = true);

void drawFlowForFeatAllCvPoint(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, char* track_status);

void drawFlowForDenseCvMat(CvMat* velx, CvMat* vely, IplImage* imgdst);

void drawFlowForDenseMat(Mat flow, Mat &framedst);

void drawArrow(CvPoint p, CvPoint q, IplImage* imgdst, CvScalar rgb = CV_RGB(0,0,255) , int thickness = 1);

void drawArrow(CvPoint p, CvPoint q, Mat &framedst, CvScalar rgb = CV_RGB(0,0,255) , int thickness = 1);

void drawTagLine(IplImage* imgdst, int *tags, int thickness = 1);

void drawTagLine(Mat &framedst, int *tags, int thickness = 1);

void drawSquare(IplImage* img, int left, int right, int up, int down, int thickness = 1);

void drawSquare(IplImage* img, int left, int right, int thickness = 1);

void drawTagFreeSpace(IplImage* img, int *tag, int thickness = 1);

void drawSquare(Mat &frame, int left, int right, int up, int down, int thickness = 1);

void drawSquare(Mat &frame, int left, int right, int thickness = 1);

void drawTagFreeSpace(Mat &frame, int *tag, int thickness = 1);

#ifndef util_DRAW_FLOW_
#define util_DRAW_FLOW_

#endif
