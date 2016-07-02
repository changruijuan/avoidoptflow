/*
*  FILE optutil.h
*  AUTHOR Sarah
*  DATE 2015/08/14 22:25
*  TODO: 光流计算方法处理光流的公共方�? 
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

Mat calibrate(Mat img); //����ͷ�궨

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

//tags��׼��start��end��turn��turn=1��ʾ���ң�turn=-1��ʾ����turn=0��ʾunkown���޷��ж��������������������Ƶ����Ϊ��׼������������
//length���궨�ĸ�����ÿ���궨������3������3*length = tags.length
//framecount ��ʾ������Ƶ֡����
//����tags��־��0��framecount֡����Ǳ��д��tag.txt�ļ���
void setTagToTxt(int* tags, int length, int framecount);

//tags��׼��start��end��turn��turn=1��ʾ���ң�turn=-1��ʾ����ֻ���1��-1��λ�á�
//length���궨�ĸ�����ÿ���궨������3������3*length = tags.length
//�궨tags��result.txt�еĽ���Ƚϵõ���ȷ��
float compareTags(int* tags, int length);

float getMaxConnectedSpace(IplImage* imgdst, int *tagSafe);

//0 - stable, 1 - increase, 2 - decrease, 3 - mess
int getArrayState(int* array, int currindex);

#endif /* util_FOE_TTC_ */
