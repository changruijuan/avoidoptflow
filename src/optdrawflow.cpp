#include "stdafx.h"
#include "optdrawflow.h"
#include "optutil.h"

void drawFlowWithoutZero(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, char* track_status, bool isleft){
	for (int i =0; i < MAX_CORNERS; i++)
	{
		if ((int)cornersprev[i].x < 0 || (int)cornersprev[i].x > WIDTH )
		{
			break;
		}
        if (track_status[i] == 0 || ((int)cornersprev[i].x - (int)cornerscurr[i].x) == 0 && ((int)cornersprev[i].y - (int)cornerscurr[i].y) == 0)
		{
			continue;
		}
		CvPoint p,q;
		p.x = isleft ? (int) cornersprev[i].x : ((int)cornersprev[i].x + WIDTH/2);
		p.y = (int) cornersprev[i].y;
		q.x = isleft ? (int) cornerscurr[i].x : ((int)cornerscurr[i].x + WIDTH/2);;
		q.y = (int) cornerscurr[i].y;
		drawArrow(p, q, imgdst);
	}
}

void drawFlowWithoutZero(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, char* track_status){
	for (int i =0; i < MAX_CORNERS; i++)
	{
		if ((int)cornersprev[i].x < 0 || (int)cornersprev[i].x > WIDTH )
		{
			break;
		}
        if (track_status[i] == 0 || ((int)cornersprev[i].x - (int)cornerscurr[i].x) == 0 && ((int)cornersprev[i].y - (int)cornerscurr[i].y) == 0)
		{
			continue;
		}
		CvPoint p,q;
		p.x = (int) cornersprev[i].x;
		p.y = (int) cornersprev[i].y;
		q.x = (int) cornerscurr[i].x;
		q.y = (int) cornerscurr[i].y;
		drawArrow(p, q, imgdst);
	}
}

void drawFlowWithoutZero(CvMat* velx, CvMat* vely, IplImage* imgdst){
	for (int i = 0; i < HEIGHT; i += 1)
	{
		for(int j = 0; j < WIDTH; j += 1){
			if ((int) cvGetReal2D(velx, i, j) == 0 && (int) cvGetReal2D(vely, i, j) == 0 )
			{
				continue;
			}
			CvPoint p, q;
			q.x = (int) cvGetReal2D(velx, i, j) + j;
			q.y = (int) cvGetReal2D(vely, i, j) + i;
			p.x = j;
			p.y = i;
			drawArrow(p, q, imgdst);
		}
	}
}

void drawFlowWithoutZero(Mat flow, Mat &framedst){
	for (int i = 0; i < HEIGHT; i += 1)
	{
		for(int j = 0; j < WIDTH; j += 1){
			Vec2f flow_at_point = flow.at<Vec2i>(i, j);
			float fx = flow_at_point[0]/10e8;
			float fy = flow_at_point[1]/10e8;
			if (fabs(fx) > UNKNOWN_FLOW_THRESH || fabs(fy) > UNKNOWN_FLOW_THRESH || (fx == 0 && fy == 0))
			{
				continue;
			}
			CvPoint p, q;
			p.x = j;
			p.y = i;
			q.x = fx + j;
			q.y = fy + i;
			//printf("p: (%d,%d). q :(%d, %d)\n", p.x, p.y, flow.at<Vec2i>(i, j)[0], flow.at<Vec2i>(i, j)[1]);
			drawArrow(p, q, framedst);
		}
	}
}

void drawFlowForFeatureCvPoint(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, char* track_status, bool isleft){
	for (int i =0; i < MAX_CORNERS; i++)
	{
        if ((int)cornersprev[i].x < 0 || (int)cornersprev[i].x > WIDTH )
		{
			break;
		}
        if(track_status[i] == 1){
            CvPoint p,q;
            p.x = isleft ? (int) cornersprev[i].x : ((int)cornersprev[i].x + WIDTH/2);
            p.y = (int) cornersprev[i].y;
            q.x = isleft ? (int) cornerscurr[i].x : ((int)cornerscurr[i].x + WIDTH/2);;
            q.y = (int) cornerscurr[i].y;
            drawArrow(p, q, imgdst);
        }
	}
}

void drawFlowForFeatAllCvPoint(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage* imgdst, char* track_status){
	for (int i =0; i < MAX_CORNERS*2; i++)
	{
        if ((int)cornersprev[i].x < 0 || (int)cornersprev[i].x > WIDTH )
		{
			break;
		}
        if(track_status[i] == 1){
            CvPoint p,q;
            p.x = (int) cornersprev[i].x;
            p.y = (int) cornersprev[i].y;
            q.x = (int) cornerscurr[i].x;;
            q.y = (int) cornerscurr[i].y;
            drawArrow(p, q, imgdst);
        }
	}
}

void drawFlowForDenseCvMat(CvMat* velx, CvMat* vely, IplImage* imgdst){
	for (int i = DRAWGAP; i < HEIGHT; i += DRAWGAP)
	{
		for(int j = DRAWGAP; j < WIDTH; j += DRAWGAP){
			CvPoint p, q;
			q.x = (int) cvGetReal2D(velx, i, j) + j;
			q.y = (int) cvGetReal2D(vely, i, j) + i;
			p.x = j;
			p.y = i;
			drawArrow(p, q, imgdst);
		}
	}
}

void drawFlowForDenseMat(Mat flow, Mat &framedst){
	for (int i = DRAWGAP; i < HEIGHT; i += DRAWGAP)
	{
		for(int j = DRAWGAP; j < WIDTH; j += DRAWGAP){
			Vec2f flow_at_point = flow.at<Vec2i>(i, j);
			float fx = flow_at_point[0]/10e8;
			float fy = flow_at_point[1]/10e8;
			if (fabs(fx) > UNKNOWN_FLOW_THRESH || fabs(fy) > UNKNOWN_FLOW_THRESH)
			{
				continue;
			}
			CvPoint p, q;
			p.x = j;
			p.y = i;
			q.x = fx + j;
			q.y = fy + i;
			//printf("p: (%d,%d). q :(%d, %d)\n", p.x, p.y, flow.at<Vec2i>(i, j)[0], flow.at<Vec2i>(i, j)[1]);
			drawArrow(p, q, framedst);
		}
	}
}

void drawArrow(CvPoint p, CvPoint q, IplImage* imgdst, CvScalar rgb , int thickness){
	double angle; 
	angle = atan2((double) p.y - q.y, (double) p.x - q.x);
	double hypotenuse; 
	hypotenuse = sqrt(((p.y - q.y)*(p.y - q.y) +(p.x - q.x)*(p.x - q.x))*1.0);

	q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
	q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
	cvLine(imgdst, p, q, rgb,thickness);

	/*p.x = (int) (q.x + 2 * hypotenuse * cos(angle + CV_PI / 4));
	p.y = (int) (q.y + 2 * hypotenuse * sin(angle + CV_PI / 4));
	cvLine(imgdst, p, q, rgb,thickness);
	p.x = (int) (q.x + 2 * hypotenuse * cos(angle - CV_PI / 4));
	p.y = (int) (q.y + 2 * hypotenuse * sin(angle - CV_PI / 4));
	cvLine(imgdst, p, q, rgb,thickness);*/

	p.x = (int) (q.x + 10 * cos(angle + CV_PI / 4));
	p.y = (int) (q.y + 10 * sin(angle + CV_PI / 4));
	cvLine(imgdst, p, q, rgb,thickness);
	p.x = (int) (q.x + 10 * cos(angle - CV_PI / 4));
	p.y = (int) (q.y + 10 * sin(angle - CV_PI / 4));
	cvLine(imgdst, p, q, rgb,thickness);
}

void drawArrow(CvPoint p, CvPoint q, Mat &framedst, CvScalar rgb , int thickness){
	double angle; 
	angle = atan2((double) p.y - q.y, (double) p.x - q.x);
	double hypotenuse; 
	hypotenuse = sqrt(((p.y - q.y)*(p.y - q.y) +(p.x - q.x)*(p.x - q.x))*1.0);

	q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
	q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
	line(framedst, p, q, rgb,thickness);

	p.x = (int) (q.x + 10 * cos(angle + CV_PI / 4));
	p.y = (int) (q.y + 10  * sin(angle + CV_PI / 4));
	line(framedst, p, q,rgb,thickness );

	p.x = (int) (q.x + 10 * cos(angle - CV_PI / 4));
	p.y = (int) (q.y + 10 * sin(angle - CV_PI / 4));
	line(framedst, p, q, rgb,thickness );
}

void drawTagLine(IplImage* imgdst, int *tags, int thickness){
	CvPoint p, q;
	for (int i = 0; i < WIDTH; i++)
	{
		p.x = 5;
		p.y = i;
		q.x = WIDTH - 5;
		q.y = i;
		switch (tags[i]){
		case 1:
			cvLine(imgdst, p, q, CV_RGB(255,0,0), thickness);
			break;
		case 2:
			cvLine(imgdst, p, q, CV_RGB(0,255,0), thickness);
			break;
		case 3:
			cvLine(imgdst, p, q, CV_RGB(0,0,255), thickness);
			break;
		case 4:
			cvLine(imgdst, p, q, CV_RGB(255,255,0), thickness);
			break;
		}
	}
}

void drawTagLine(Mat &framedst, int *tags, int thickness){
	CvPoint p, q;
	for (int i = 0; i < WIDTH; i++)
	{
		p.x = 5;
		p.y = i;
		q.x = WIDTH - 5;
		q.y = i;
		switch (tags[i]){
		case 1:
			line(framedst, p, q, CV_RGB(255,0,0), thickness);
			break;
		case 2:
			line(framedst, p, q, CV_RGB(0,255,0), thickness);
			break;
		case 3:
			line(framedst, p, q, CV_RGB(0,0,255), thickness);
			break;
		case 4:
			line(framedst, p, q, CV_RGB(255,255,0), thickness);
			break;
		}
	}
}

void drawSquare(IplImage* img, int left, int right, int up, int down, int thickness){
	CvPoint upl, upr, downl, downr;
	upl.y = up; 
	upl.x = left;
	upr.y = up; 
	upr.x = right;
	downl.y = down; 
	downl.x = left;
	downr.y = down;
	downr.x = right;
	cvLine(img, upl, upr, CV_RGB(0,255,0), thickness);
	cvLine(img, upl, downl, CV_RGB(0,255,0), thickness);
	cvLine(img, downl, downr, CV_RGB(0,255,0), thickness);
	cvLine(img, upr, downr, CV_RGB(0,255,0), thickness);
}

void drawSquare(IplImage* img, int left, int right, int thickness){
	drawSquare(img, left, right, 5, HEIGHT-5, thickness);
}

void drawTagFreeSpace(IplImage* img, int *tag, int thickness){ //在原视频帧中画free区间，即安全区间，找tag=0的连续区间,tag指tagOrigin或tagSafe等等
	int left = -1, right = -1;
	for (int i = 0; i < WIDTH-1; i++) 
	{
		if (left == -1) 
		{
			if (tag[i] == 0 && tag[i+1] == 0)
			{
				left = i;
			}
		}else{ //left已经赋值，所以给right赋值
			if (tag[i] == 0 && tag[i+1] == 1)
			{
				right = i;
				drawSquare(img, left, right, thickness);
				left = -1;
				right = -1;
			}
		}
	}
	if (left != -1)
	{
		right = WIDTH - 1;
		drawSquare(img, left, right, thickness);
	}
}

void drawSquare(Mat &frame, int left, int right, int up, int down, int thickness){
	CvPoint upl, upr, downl, downr;
	upl.y = up; 
	upl.x = left;
	upr.y = up; 
	upr.x = right;
	downl.y = down; 
	downl.x = left;
	downr.y = down;
	downr.x = right;
	line(frame, upl, upr, CV_RGB(0,255,0), thickness);
	line(frame, upl, downl, CV_RGB(0,255,0), thickness);
	line(frame, downl, downr, CV_RGB(0,255,0), thickness);
	line(frame, upr, downr, CV_RGB(0,255,0), thickness);
}

void drawSquare(Mat &frame, int left, int right, int thickness){
	drawSquare(frame, left, right, 5, HEIGHT-5, thickness);
}

void drawTagFreeSpace(Mat &frame, int *tag, int thickness){ //在原视频帧中画free区间，即安全区间，找tag=0的连续区间,tag指tagOrigin或tagSafe等等
	int left = -1, right = -1;
	for (int i = 0; i < WIDTH-1; i++) 
	{
		if (left == -1) 
		{
			if (tag[i] == 0 && tag[i+1] == 0)
			{
				left = i;
			}
		}else{ //left已经赋值，所以给right赋值
			if (tag[i] == 0 && tag[i+1] == 1)
			{
				right = i;
				drawSquare(frame, left, right, thickness);
				left = -1;
				right = -1;
			}
		}
	}
	if (left != -1)
	{
		right = WIDTH - 1;
		drawSquare(frame, left, right, thickness);
	}
}

