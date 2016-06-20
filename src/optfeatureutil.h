/*
*  FILE optfeatureutil.h
*  AUTHOR Sarah
*  DATE 2015/08/15 10:42
*  TODO: PyrLK ?????�����������������ҹ���ƽ��??������??��FOE��TTC???
*/
#pragma once

#include "stdafx.h"
#include <cv.h>

#include "optutil.h"
#include "optcvmatutil.h"

using namespace cv;
using namespace std;

#ifndef util_OPT_FEATURE_CVMAT_
#define util_OPT_FEATURE_CVMAT_

/*
*  Method:     balanceForFeatureCvPoint
*  Description: ���ҹ���ƽ���㷨.
*  Returns:    int. ����ָ��. 1 - ??? 2 - ??? 3 - ??? 4 - ֹͣ.
*  CvPoint2D32f * cornersprev: Required = true. X�������.
*  CvPoint2D32f * cornerscurr: Required = true. Y�������.
*  IplImage * imgdst: Required = true. Ŀ��ͼ��???
*  int threshold: Required = true. ��??�����������ڴ���ֵʱ�����ؿ���ָ???.
*  float k: Required = false. ���ҹ���ƽ���Ȩ??? �����Ҳ����Ȩ??��Χ��ʱ����???.
*  int px: Required = false. ���ҹ����ķֽ���.
*  int py: Required = false. ���¹����ķֽ���.
*/
float balanceForFeatureCvPoint(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, CvPoint2D32f* cornersprev_12, CvPoint2D32f* cornerscurr_12, IplImage* imgdst, float k, char* track_status_11, char* track_status_12, int px = WIDTH/2, int py = HEIGHT/2);

float balanceForFeatureCvPoint(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, IplImage* imgdst, float k, char* track_status);

Vec2i foeForFeatureCvPoint(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr);

float ttcForFeatureCvPoint(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, int foeY, float *ttc);

void getSpeedFromFlow(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, CvPoint2D32f* cornersprev_12, CvPoint2D32f* cornerscurr_12, IplImage* imgdst);

Vec2d getSpeed(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, bool isleft = true, float kangle=0);

#endif
