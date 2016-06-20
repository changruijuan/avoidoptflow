#include "optlearning.h"

#include <math.h>
#include "stdafx.h"
#include <cv.h>

#include "optutil.h"
#include "optdrawflow.h"
#include "var.h"

void learningFeature(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst){
	int count = 0;
	int sumx = 0, sumy = 0, x, y;
	float sumxy = 0;
	int left = WIDTH*LEARNING_VIEW, right = WIDTH*(1-LEARNING_VIEW);
	int up = HEIGHT*LEARNING_VIEW, down = HEIGHT*(1-LEARNING_VIEW);
	//int up = 0, down = HEIGHT;
	for (int i = 0; i < MAX_CORNERS*2; i++)
	{
		if ((int)cornersprev[i].x > left && (int)cornersprev[i].x < right && 
			(int)cornersprev[i].y > up && (int)cornersprev[i].y < down)
		{
			x = ((int) cornerscurr[i].x - (int) cornersprev[i].x);
			y = ((int) cornerscurr[i].y - (int) cornersprev[i].y);
			sumx += x;
			sumy += y;
			sumxy += sqrt((float)(x*x+y*y));
			count++;
		}
	}
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1, 1, 0, 2);
	char c[50];
	sprintf(c, "%d %d %d %.2f", count, sumx, sumy, sumxy);
	cvPutText(imgdst, c, cvPoint(10,40), &font, CV_RGB(255, 0, 0));
	drawSquare(imgdst, left, right, up+5, down-5);

    if (IS_WRITE_LEANING)
	{
		char buffer[200];
		sprintf(buffer, "%d\t%d\t%d\t%.2f\t%d\t%d\t%.2f\n", count, sumx, sumy, sumxy, sumx/count, sumy/count, sumxy/count);
		writeFile(buffer);
	}
}

void learningCvMat(IplImage* imgprev, IplImage* imgdst, CvMat* velx, CvMat* vely){
    int left = EDGE_OBS*WIDTH, right = (1-EDGE_OBS)*WIDTH;
    int up = EDGE_OBS*HEIGHT, down = (1-EDGE_OBS)*HEIGHT;

    int sumFlow = 0;
    for(int i = up; i < down; i++){
        for(int j = left; j < down; j++ ){
            sumFlow += abs(((int) cvGetReal2D(velx, i, j)));
        }
    }

    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1, 1, 0, 2);
    char c[100];
    sprintf(c, "%d %d %d %d", img_num, ardrone_action, sumFlow, filtercount);
    cvPutText(imgdst, c, cvPoint(10,40), &font, CV_RGB(255, 0, 0));
    drawSquare(imgdst, left, right, up+5, down-5);

    if (IS_WRITE_LEANING)
    {
        char buffer[200];
        sprintf(buffer, "%d\t%d\t%d\t%d\n", img_num, ardrone_action, sumFlow, filtercount);
        writeFile(buffer);
    }

}
