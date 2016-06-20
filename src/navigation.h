/*
*  FILE navigation.h
*  AUTHOR Sarah
*  DATE 2015/08/15 10:42
*  TODO: ????????????
*/
#pragma once

#include "stdafx.h"
#include <cv.h>
#include "optutil.h"
#include "opticalflow.h"

using namespace cv;
using namespace std;

#ifndef optflow_NAVIGATION_
#define  optflow_NAVIGATION_
//��װ LK/HS/BM
typedef float (*ImgFunType)(IplImage* imgprev, IplImage* imgcurr, CvMat* velx, CvMat* vely);

//��װ SF/FB
typedef Mat (*MatFunType)(Mat frameprev, Mat framecurr, Mat flow);

//��װ PyrLK
typedef float (*ImgFeatureFunType)(IplImage* imgprev, IplImage* imgcurr,CvPoint2D32f* cornersprev,CvPoint2D32f* cornerscurr,CvRect rect, char* status_check);

typedef float (*ImgFeatAllFunType)(IplImage* imgprev, IplImage* imgcurr,CvPoint2D32f* cornersprev,CvPoint2D32f* cornerscurr, char* status_check);

/*
*  Method:     imgFeatureBalance
*  Description: ������������ù������е�???
*               ��video.h����, ����common.h & opt*util.h
*  Returns:    int.
*  ImgFeatureFunType funtype: Required = true. PyrLK����.
*  IplImage * imgprev: Required = true. ��һ֡ͼ???
*  IplImage * imgcurr: Required = true. �ڶ�֡ͼ???
*  IplImage * imgdst: Required = true. Ŀ��ͼ���ڸ�ͼ���ϻ������������������???
*/
float imgFeatureStrategic(ImgFeatureFunType funtype,IplImage* imgprev, IplImage* imgcurr, IplImage* imgdst, Mat &color, int strategic = 1);

float imgFeatAllStrategic(ImgFeatAllFunType funtype,IplImage* imgprev, IplImage* imgcurr, IplImage* imgdst, int strategic = 1);

/*
*  Method:     imgStrategic
*  Description: ������������ù������е�??? ��������ΪIplImage.
*               ��video.h����, ����common.h & opt*util.h & motioncolor.h
*  Returns:    int. ?????????ֹͣ4
*  ImgFunType funtype: Required = true. LK(Lucaskanade)/HS(HornSchunck)/BM(BlockMatch) �������㷽��.
*  IplImage * imgprev: Required = true. ��һ֡ͼ��ԭʼͼ��.
*  IplImage * imgcurr: Required = true. �ڶ�֡ͼ��ԭʼͼ��.
*  IplImage * imgdst: Required = true. Ŀ��ͼ���ڸ�ͼ���ϻ������������������???
*  Mat & color: Required = true. ��Ź���ת��Ϊ��ɫ��ͼ��.
*  int strategic: Required = false. Ĭ��???�����ҹ���ƽ�⣩. ��video.h��imgVideo����������ϸ˵��.
*/
float imgStrategic(ImgFunType funtype, IplImage* imgprev_1, IplImage* imgcurr_1, IplImage* imgprev, IplImage* imgdst, Mat &color, Mat &gray, int strategic = 1);

/*
*  Method:     matStrategic
*  Description: ������������ù������е�??? ��������Ϊcv::Mat.
*               ��video.h����, ����common.h & opt*util.h & motioncolor.h
*  Returns:    int. ?????????ֹͣ4
*  MatFunType funtype: Required = SF(SimpleFlow)/FB(FarneBack) �������㷽��
*  Mat frameprev: Required = true. ��һ֡ͼ???ԭʼͼ��û����СҲû�лҶȻ�����.
*  Mat framecurr: Required = true. �ڶ�֡ͼ???
*  Mat framedst: Required = true. Ŀ��ͼ���ڸ�ͼ���ϻ������������������???
*  Mat & color: Required = true. ��Ź���ת��Ϊ��ɫ��ͼ��.
*  int strategic: Required = false. Ĭ��???�����ҹ���ƽ�⣩. ��video.h��imgVideo����������ϸ˵��.
*  bool issf: Required = false. Ĭ��false������FB�����������SF������issf���봫??true.
*/
float matStrategic(MatFunType funtype, Mat frameprev_1, Mat framecurr_1, Mat &frameprev, Mat &framedst,Mat &color, Mat &gray, int strategic = 1, bool issf = false);

#endif /* optflow_NAVIGATION_ */



