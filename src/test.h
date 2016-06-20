#pragma once

#include "stdafx.h"
#include <cv.h>

using namespace cv;
using namespace std;

#ifndef test_OPT_
#define test_OPT_

void dealMatVideo(String path);

void dealImgVideo(String path);

void getHuImgVideo(String path1, String path2);

IplImage* getDstContours(IplImage* img, CvSeq *contours);

void fillHole(const Mat srcBw, Mat &dstBw);

#endif /* util_VIDEO_ */


