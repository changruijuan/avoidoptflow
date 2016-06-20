/*
*  FILE video.h
*  AUTHOR Sarah
*  DATE 2015/08/14 20:10
*  TODO: 使用不同的光流方法处理视频.
*/
#pragma once

#include "stdafx.h"
#include <cv.h>
#include "navigation.h"

using namespace cv;
using namespace std;

#ifndef _VIDEO_
#define _VIDEO_

/*
*  Method:     imgVideo
*  Description: 稠密光流处理视频，参数类型为IplImage，按ESC退出.
*  Returns:    void.  flow视频输出在 ../video/flow_img.avi, color视频输出在../video/color_img.avi.
*  ImgFunType funtype:  Required = true. LK(Lucaskanade)/HS(HornSchunck)/BM(BlockMatch) 光流方法.
*  int strategic: Required = true. 默认是1. 二进制和十进制转化，当十进制第i位为1表示使用第i中策略. 
*                 1-左右平衡策略, 2-画光流, 3-光流转化为颜色. 
*                 如strategic=5, 5的二进制第1、3位是1，则利用左右平衡策略和光流转化为颜色.
*  String inputfile: Required = false. 默认视频从ARDrone获取，同样也可以传递视频的位置处理固定视频.
*/
void imgVideo(ImgFunType funtype, int strategic = 1, String inputfile = "tcp://192.168.1.1:5555");

/*
*  Method:     matVideo
*  Description: 稠密光流处理视频，参数类型为cv::Mat，按ESC退出.
*  Returns:    void. flow视频输出在 ../video/flow_img.avi, color视频输出在../video/color_img.avi.
*  MatFunType funtype: Required = true. SF(SimpleFlow)/FB(FarneBack) 光流方法.
*  bool issf: Required = true. 默认调用FB, 因为SF计算时间比较长。如果调用SF，issf必须传递true。
*  int strategic: Required = false. 和imgVideo函数参数strategic表示相同.
*  String inputfile: Required = false. 默认视频从ARDrone获取，同样也可以传递视频的位置处理固定视频.
*/
void matVideo(MatFunType funtype, bool issf =false, int strategic = 1,String inputfile = "tcp://192.168.1.1:5555");

/*
*  Method:     imgFeatureVideo 
*  Description: 稀疏光流处理视频，参数类型为IplImage，按ESC退出.
*  Returns:    void. flow视频输出在 ../video/flow_img.avi, color视频输出在../video/color_img.avi.
*  ImgFeatureFunType funtype: Required = true. PyrLK 光流方法.
*  int strategic: Required = false. 和imgVideo函数参数strategic表示相同.
*  String inputfile: Required = false. 默认视频从ARDrone获取，同样也可以传递视频的位置处理固定视频.
*/
void imgFeatureVideo(ImgFeatureFunType funtype, int strategic = 1, String inputfile = "tcp://192.168.1.1:5555");

void imgFeatAllVideo(ImgFeatAllFunType funtype, int strategic = 1, String inputfile=  "tcp://192.168.1.1:5555");

/*
*  Method:     imgFlow
*  Description: 稠密光流处理图像帧，参数类型为IplImage.
*  Returns:    void. 左右光流平衡和画光流.
*  ImgFunType funtype: Required = true. LK/HS/BM 光流方法.
*  String imgprev:  Required = true. 前一帧图像.
*  String imgcurr: Required = true. 后一帧图像.
*/
void imgFlow(ImgFunType funtype,String imgprev, String imgcurr);

/*
*  Method:     matFlow
*  Description: 稠密光流处理图像帧，参数类型为cv::Mat.
*  Returns:    void. 左右光流平衡和画光流.
*  MatFunType funtype: Required = true. SF/FB 光流方法.
*  String frameprev: Required = true. 前一帧图像.
*  String framecurr: Required = true. 后一帧图像.
*  bool issf: 默认调用FB, 因为SF计算时间比较长。如果调用SF，issf必须传递true
*/
void matFlow(MatFunType funtype, String frameprev, String framecurr, bool issf = false);

/*
*  Method:     imgFeatureFlow
*  Description: 稀疏光流处理图像帧，参数类型为IplImageIplImage.
*  Returns:    void. 左右光流平衡和画光流.
*  ImgFeatureFunType funtype: Required = true. PyrLK 光流方法.
*  String imgprev: Required = true. 前一帧图像. 
*  String imgcurr: Required = true. 后一帧图像.
*/
void imgFeatureFlow(ImgFeatureFunType funtype, String imgprev, String imgcurr);

/*
*  Method:     avoidMain
*  Description: main函数入口，参数选取不同而执行不同的光流计算方法及策略.
*  Returns:    void. 
*  int argc： Required = true. 同main函数，传递参数个数.
*  char* argv[]: Required = true. 同main函数，传递参数. 形式（光流计算方法，策略，视频路径）
*                argc = 1 ： 光流计算方法*，默认策略左右光流平衡，视频默认从Ardrone获取
*                argc = 2 ： 光流计算方法*，策略*，视频默认从Ardrone获取
*                argc = 3 ： 光流计算方法*，策略*，视频路径*
*                光流计算方法：LK、HS、BM、PyrLK、FB、SF
*/
void avoidMain(int argc, char* argv[]);

#endif /* _VIDEO_ */


