/*
*  FILE videoutil.h
*  AUTHOR Sarah
*  DATE 2016/01/8 20:10
*  TODO: ������Ƶ,��ӿ���Ƶ��ģ������Ƶ
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
*  Description: ģ������Ƶ
*  Returns:    void. 
*  String inputfile: Required = true
*  int func: Required = true  1 - ��ֵģ����2 - ��˹ģ���� 3 - ��ֵģ���� 4 - ˫���˲�  
*/
void blurVideo(String inputfile, int func);

/*
*  Method:     quickVideo
*  Description: ÿ��skip֡ȡһ֡ͼ�񣬼ӿ���Ƶ
*  Returns:    void. 
*  String inputfile: Required = true
*  int skip: Required = false��Ĭ�ϲ��ӿ���Ƶ֡
*/
void quickVideo(String inputfile, int skip = 0); //ÿ��skip֡ȡһ֡ͼ��


/*
*  Method:     video2Image
*  Description: ����Ƶת��Ϊһ֡֡ͼ��
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
// Qualifier:  ��inputDirĿ¼�µ�ͼƬ��תroll�����Ƶ../video/roll.avi
// Parameter: String inputDir
// Parameter: int roll, ����x��ת���ٶȣ�1-90,2-180,3-270,0-����ת��Ĭ�ϲ���ת
// Parameter: String outputfile
//************************************
void image2Video(int start, int end, int roll = 0, String inputDir = "../video/image/");


//************************************
// Method:    cutAndRollVideo
// FullName:  cutAndRollVideo
// Access:    public 
// Returns:   void
// Qualifier: inputfile video ��תrollת��Ϊ��Ƶ../video/roll.avi
// Parameter: String inputfile
// Parameter: String outfile
// Parameter: int roll
//************************************
void video2RotateVideo(String inputfile, int roll = 0);

#endif /* util_VIDEO_ */


