/*
*  FILE videoutil.h
*  AUTHOR Sarah
*  DATE 2016/01/8 20:10
*  TODO: 处理视频,如加快视频，模糊化视频
*/
#pragma once

#include "stdafx.h"
#include <cv.h>

using namespace cv;
using namespace std;

#ifndef util_VIDEO_
#define util_VIDEO_

/*
*  Method:     blurVideo
*  Description: 模糊化视频
*  Returns:    void. 
*  String inputfile: Required = true
*  int func: Required = true  1 - 均值模糊；2 - 高斯模糊； 3 - 中值模糊； 4 - 双边滤波  
*/
void blurVideo(String inputfile, int func);

/*
*  Method:     quickVideo
*  Description: 每隔skip帧取一帧图像，加快视频
*  Returns:    void. 
*  String inputfile: Required = true
*  int skip: Required = false，默认不加快视频帧
*/
void quickVideo(String inputfile, int skip = 0); //每隔skip帧取一帧图像


/*
*  Method:     video2Image
*  Description: 将视频转化为一帧帧图像
*  Returns:    void. 
*  String inputfile: Required = true
*/
int video2Image(String inputfile);

int video2Image(String inputfile, String outpfile);

void resizeVideo(String inputfile, int width = WIDTH, int height = HEIGHT);

int video2ResizeImage(String inputfile, int width = WIDTH, int height = HEIGHT);

//************************************
// Method:    image2Video
// FullName:  image2Video
// Access:    public 
// Returns:   void
// Qualifier:  将inputDir目录下的图片旋转roll输出视频../video/roll.avi
// Parameter: String inputDir
// Parameter: int roll, 按照x旋转多少度，1-90,2-180,3-270,0-不旋转，默认不旋转
// Parameter: String outputfile
//************************************
void image2Video(int start, int end, int roll = 0, String inputDir = "../video/image/");


//************************************
// Method:    cutAndRollVideo
// FullName:  cutAndRollVideo
// Access:    public 
// Returns:   void
// Qualifier: inputfile video 旋转roll转化为视频../video/roll.avi
// Parameter: String inputfile
// Parameter: String outfile
// Parameter: int roll
//************************************
void video2RotateVideo(String inputfile, int roll = 0);

#endif /* util_VIDEO_ */


