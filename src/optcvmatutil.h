/*
*  FILE optcvmatutil.h
*  AUTHOR Sarah
*  DATE 2015/08/15 10:42
*  TODO: LK/HS/BM 稠密光流公共方法如左右光流平衡�?画光流�?求FOE、TTC�?
*/
#pragma once

#include "stdafx.h"
#include <cv.h>

#include "optutil.h"

using namespace cv;
using namespace std;

#ifndef util_OPT_CVMAT_
#define util_OPT_CVMAT_

void getTagOriginByColor(IplImage* imgprev, int* tagOrigin);

bool isInRange(IplImage* imghsv, int row, int col);

Vec2i foeForDenseCvMat1(CvMat* velx, CvMat* vely);

Vec2i foeForDenseCvMat2(CvMat* velx, CvMat* vely);

float ttcForDenseCvMat(CvMat* vely, float *ttc, int foeY = HEIGHT/2);

float ttcTagForDenseCvMat(IplImage* imgprev, IplImage* imgdst, CvMat* velx, CvMat* vely);

float flowForDenseCvMat(CvMat* velx, CvMat* vely, float* colflow);

float flowTagForDenseCvMat(IplImage* imgprev, IplImage* imgdst, CvMat* velx, CvMat* vely);

/*
*  Method:     balanceForDenseCvMat
*  Description: 左右光流平衡算法. 
*  Returns:    int. 控制指令. 1 - �? 2 - �? 3 - �? 4 - 停止. 
*  CvMat * velx: Required = true. X方向光流. 
*  CvMat * vely: Required = true. Y方向光流. 
*  IplImage * imgdst: Required = true. 目标图像�?
*  int threshold: Required = true. 阈�?，当光流大于此阈值时，返回控制指�?.
*  float k: Required = false. 左右光流平衡的权�? 当左右差距在权�?范围内时，返�?.
*  int px: Required = false. 左右光流的分界线.
*  int py: Required = false. 上下光流的分界线.
*  float edge: Required = false. 忽略上下左右edge倍的边界. 0.1429��Ϊ1/7
*/
float balanceForDenseCvMat(CvMat* velx, CvMat* vely, IplImage* imgprev, IplImage* imgdst, float k, int px = WIDTH/2, int py = HEIGHT/2);

float balanceWithTTCDenseCvMat(CvMat* velx, CvMat* vely, IplImage* imgprev, IplImage* imgdst, float k, int px = WIDTH/2, int py = HEIGHT/2);

bool isBigObstacleColor(IplImage* imgdst, CvMat* velx);

int isBigObstacleColor(IplImage* imgdst, CvMat* velx, bool isleft = true);

bool isBigObstacleHSV(IplImage* imgdst, CvMat* velx, bool isleft = true);

void getSpeedFromFlow(CvMat* velx, CvMat* vely, IplImage* imgdst);

//filter big flow
void filterFLowCvMat(CvMat* velx, CvMat* vely);

float ttcCrossForDenseCvMat (float result, CvMat* velx, CvMat* vely, IplImage* imgdst);

#endif
