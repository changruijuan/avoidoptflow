#pragma once

#include "stdafx.h"
#include <cv.h>

#include "optutil.h"

using namespace cv;
using namespace std;

#ifndef util_OPT_LEARNIING_
#define util_OPT_LEARNIING_

void learningFeature(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst);

void learningCvMat(IplImage* imgprev, IplImage* imgdst, CvMat* velx, CvMat* vely);

void learningCvMat_move(IplImage* imgprev, IplImage* imgdst, CvMat* velx, CvMat* vely);

#endif
