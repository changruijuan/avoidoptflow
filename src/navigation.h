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
//封装 LK/HS/BM
typedef float (*ImgFunType)(IplImage* imgprev, IplImage* imgcurr, CvMat* velx, CvMat* vely);

//封装 SF/FB
typedef Mat (*MatFunType)(Mat frameprev, Mat framecurr, Mat flow);

//封装 PyrLK
typedef float (*ImgFeatureFunType)(IplImage* imgprev, IplImage* imgcurr,CvPoint2D32f* cornersprev,CvPoint2D32f* cornerscurr,CvRect rect, char* status_check);

typedef float (*ImgFeatAllFunType)(IplImage* imgprev, IplImage* imgcurr,CvPoint2D32f* cornersprev,CvPoint2D32f* cornerscurr, char* status_check);

/*
*  Method:     imgFeatureBalance
*  Description: 计算光流，利用光流进行导???
*               被video.h调用, 调用common.h & opt*util.h
*  Returns:    int.
*  ImgFeatureFunType funtype: Required = true. PyrLK方法.
*  IplImage * imgprev: Required = true. 第一帧图???
*  IplImage * imgcurr: Required = true. 第二帧图???
*  IplImage * imgdst: Required = true. 目的图像，在该图像上画光流，输出导航数据???
*/
float imgFeatureStrategic(ImgFeatureFunType funtype,IplImage* imgprev, IplImage* imgcurr, IplImage* imgdst, Mat &color, int strategic = 1);

float imgFeatAllStrategic(ImgFeatAllFunType funtype,IplImage* imgprev, IplImage* imgcurr, IplImage* imgdst, int strategic = 1);

/*
*  Method:     imgStrategic
*  Description: 计算光流，利用光流进行导??? 参数类型为IplImage.
*               被video.h调用, 调用common.h & opt*util.h & motioncolor.h
*  Returns:    int. ?????????停止4
*  ImgFunType funtype: Required = true. LK(Lucaskanade)/HS(HornSchunck)/BM(BlockMatch) 光流计算方法.
*  IplImage * imgprev: Required = true. 第一帧图像，原始图像.
*  IplImage * imgcurr: Required = true. 第二帧图像，原始图像.
*  IplImage * imgdst: Required = true. 目的图像，在该图像上画光流，输出导航数据???
*  Mat & color: Required = true. 存放光流转化为颜色的图像.
*  int strategic: Required = false. 默认???（左右光流平衡）. 在video.h中imgVideo函数中有详细说明.
*/
float imgStrategic(ImgFunType funtype, IplImage* imgprev_1, IplImage* imgcurr_1, IplImage* imgprev, IplImage* imgdst, Mat &color, Mat &gray, int strategic = 1);

/*
*  Method:     matStrategic
*  Description: 计算光流，利用光流进行导??? 参数类型为cv::Mat.
*               被video.h调用, 调用common.h & opt*util.h & motioncolor.h
*  Returns:    int. ?????????停止4
*  MatFunType funtype: Required = SF(SimpleFlow)/FB(FarneBack) 光流计算方法
*  Mat frameprev: Required = true. 第一帧图???原始图像没有缩小也没有灰度化处理.
*  Mat framecurr: Required = true. 第二帧图???
*  Mat framedst: Required = true. 目的图像，在该图像上画光流，输出导航数据???
*  Mat & color: Required = true. 存放光流转化为颜色的图像.
*  int strategic: Required = false. 默认???（左右光流平衡）. 在video.h中imgVideo函数中有详细说明.
*  bool issf: Required = false. 默认false，调用FB。但如果调用SF方法，issf必须传??true.
*/
float matStrategic(MatFunType funtype, Mat frameprev_1, Mat framecurr_1, Mat &frameprev, Mat &framedst,Mat &color, Mat &gray, int strategic = 1, bool issf = false);

#endif /* optflow_NAVIGATION_ */



