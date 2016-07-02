/*
*  FILE optutil.h
*  AUTHOR Sarah
*  DATE 2015/08/14 22:25
*  TODO: 鍏夋祦璁＄畻鏂规硶澶勭悊鍏夋祦鐨勫叕鍏辨柟娉? 
*/
#pragma once

#include "stdafx.h"
#include <cv.h>

using namespace cv;
using namespace std;

#ifndef util_FOE_TTC_
#define util_FOE_TTC_

void tagSafeAreaByTTC(float* ttc, float threshold, int *tagSafe);

void tagSafeInter(int *tagSafe);

void tagSafeConnected(int *tagSafe);

void tagConnectionInter(int *tagSafe, int left, int midleft, int midright, int right);

void tagConnectionInterPro(int *tagSafe, int left, int midleft, int midright, int right);

float compareTag(int *tagOrigin, int *tagSafe, int *tags);

float balanceControlLR(bool isBig, int leftSumFlow, int rightSumFlow, float k);

float balanceControlLR(int isBigLeft, int isBigRight, int leftSumFlow, int rightSumFlow, float k);

float turnLRScale(float leftSumFlow, float rightSumFlow, float k);

Mat calibrate(Mat img); //摄像头标定

void calibrate(IplImage* &iplimg);

void writeFile(const char* lineStr);

void drawOrientation(Vec2i leftSumFlow, Vec2i rightSumFlow, int px, int py, float result, IplImage* imgdst);

CvSeq *getImageContours(IplImage *src);

double huSimilarity(IplImage* src, IplImage* gray);

void FillInternalContours(IplImage *pBinary, double dAreaThre);

int Otsu(IplImage* src);

void printfPointColor(Mat &frame, int x, int y);

void pixel2HueLowUp(Mat &frame, int x, int y);

int splitString(char dst[50][5], char* str, const char* spl);

void readSpeacialLine(char* line, int pos);

void getTagOriginFromFile( int* tagOrigin);

float resultToAccuracy(bool isleft);

bool isRight(int result, bool isleft);

//tags标准：start，end，turn。turn=1表示向右，turn=-1表示向左，turn=0表示unkown（无法判断向左还是向右所以这段视频不作为标准进行评定）。
//length：标定的个数，每个标定长度是3，所以3*length = tags.length
//framecount 表示整个视频帧长度
//根据tags标志将0到framecount帧做标记标记写入tag.txt文件中
void setTagToTxt(int* tags, int length, int framecount);

//tags标准：start，end，turn。turn=1表示向右，turn=-1表示向左。只标记1和-1的位置。
//length：标定的个数，每个标定长度是3，所以3*length = tags.length
//标定tags与result.txt中的结果比较得到正确率
float compareTags(int* tags, int length);

float getMaxConnectedSpace(IplImage* imgdst, int *tagSafe);

//0 - stable, 1 - increase, 2 - decrease, 3 - mess
int getArrayState(int* array, int currindex);

#endif /* util_FOE_TTC_ */
