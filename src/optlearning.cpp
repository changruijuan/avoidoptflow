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

void learningCvMat_move(IplImage* imgprev, IplImage* imgdst, CvMat* velx, CvMat* vely){
    int up = EDGE_OBS*HEIGHT, down = (1-EDGE_OBS)*HEIGHT;
    int sumFlow[LEARNINGMOVECOL];
    int split = WIDTH/LEARNINGMOVECOL;

    for(int j = 0; j < LEARNINGMOVECOL; j++ ){
        int start = j * split, end = ( j + 1 ) * split;
        sumFlow[j] = 0;
        for (int k = start; k < end; k++) {
            for(int i = up; i < down; i++){
                sumFlow[j] += abs(((int) cvGetReal2D(velx, i, k)));
            }
        }
        CvPoint upp, downp;
        upp.x = start;
        upp.y = up;
        downp.x = start;
        downp.y = down;
        cvLine(imgdst, upp, downp, CV_RGB(0,255,0));
    }

    int grap = LEARNINGMOVECOL/3;
    int move_left = 0;
    int move_right = grap;
    int move_summax = 0;
    int maxpos = 0;
    int maxsimple = sumFlow[0];

    for (int i = 0; i < grap; i++) {
        move_summax += sumFlow[i];
        if (maxsimple < sumFlow[i]) {
            maxsimple = sumFlow[i];
            maxpos = i;
        }
    }
    int lastmax = move_summax;
    for (int i = grap; i < LEARNINGMOVECOL; i++) {
        int tmpmax = lastmax - sumFlow[i - grap] + sumFlow[i];
        if (tmpmax > move_summax) {
            move_summax = tmpmax;
            move_left = i - grap + 1;
            move_right = i;
        }
        lastmax = tmpmax;
        if (maxsimple < sumFlow[i]) {
            maxsimple = sumFlow[i];
            maxpos = i;
        }
    }

    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1, 1, 0, 2);
    char c[1000];
//    sprintf(c, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", img_num, sumFlow[0], sumFlow[1], sumFlow[2], sumFlow[3], \
//            sumFlow[4], sumFlow[5], sumFlow[6], sumFlow[7], \
//            sumFlow[8], sumFlow[9], sumFlow[10], sumFlow[11], \
//            sumFlow[12], sumFlow[13], sumFlow[14]);
    sprintf(c, "%d %d %d %d %d", img_num, maxpos, maxsimple, (move_left + move_right)/2, move_summax);
    cvPutText(imgdst, c, cvPoint(10,40), &font, CV_RGB(255, 0, 0));

    if (IS_WRITE_LEANING)
    {
        char buffer[1000];
//        sprintf(buffer, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", img_num, sumFlow[0], sumFlow[1], sumFlow[2], sumFlow[3], \
//                sumFlow[4], sumFlow[5], sumFlow[6], sumFlow[7], \
//                sumFlow[8], sumFlow[9], sumFlow[10], sumFlow[11], \
//                sumFlow[12], sumFlow[13], sumFlow[14]);
        sprintf(buffer, "%d\t%d\t%d\t%d\t%d\n", img_num, maxpos, maxsimple, (move_left + move_right)/2, move_summax);
        writeFile(buffer);
    }

}
