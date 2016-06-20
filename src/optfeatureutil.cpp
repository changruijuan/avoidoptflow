#include "stdafx.h"
#include <cv.h>

#include "optutil.h"
#include "optfeatureutil.h"
#include "opticalflow.h"
#include "optdrawflow.h"

using namespace cv;
using namespace std;

float balanceForFeatureCvPoint(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, CvPoint2D32f* cornersprev_12, CvPoint2D32f* cornerscurr_12, IplImage* imgdst, float k, char* track_status_11, char* track_status_12, int px, int py){
	Vec2i leftSumFlow = Vec2i(0, 0);
	for (int i = 0; i < MAX_CORNERS; i++)
	{
        if(track_status_11[i] == 1){
            leftSumFlow[0] += ((int) cornerscurr_11[i].x - (int) cornersprev_11[i].x);
            leftSumFlow[1] += ((int) cornerscurr_11[i].y - (int) cornersprev_11[i].y);
        }
	}
	Vec2i rightSumFlow = Vec2i(0, 0);
	for (int i = 0; i < MAX_CORNERS; i++)
	{
        if(track_status_12[i] == 1){
            rightSumFlow[0] += ((int) cornerscurr_12[i].x - (int) cornersprev_12[i].x);
            rightSumFlow[1] += ((int) cornerscurr_12[i].y - (int) cornersprev_12[i].y);
        }
	}
	//printf("pre left :%d,%d  right:%d,%d\n", leftSumFlow[0], leftSumFlow[1], rightSumFlow[0], rightSumFlow[1]);
	leftSumFlow[0] = abs(leftSumFlow[0]*15 / MAX_CORNERS);//避免画图时左右箭头太短，所以乘5
	leftSumFlow[1] = abs(leftSumFlow[1]*15 / MAX_CORNERS);
	rightSumFlow[0] = abs(rightSumFlow[0]*15 / MAX_CORNERS);
	rightSumFlow[1] = abs(rightSumFlow[1]*15 / MAX_CORNERS);

	if(IS_FLOW_WRITE_FILE){
		char buffer[50];
		sprintf(buffer, "%d	%d\n", leftSumFlow[0], rightSumFlow[0]);
		writeFile(buffer);
	}

	float result = balanceControlLR(false, leftSumFlow[0], rightSumFlow[0], k); //这里未做大型障碍物预测，所以直接传入false
	drawOrientation(leftSumFlow, rightSumFlow, px, py, result, imgdst);

	return result;
}

float balanceForFeatureCvPoint(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, IplImage* imgdst, float k, char*  track_status){
	Vec2i leftSumFlow = Vec2i(0, 0),rightSumFlow = Vec2i(0, 0);
	int left = 0;;
	int right = 0;
	float LS = 0;
	float RS = 0;
	for (int i = 0; i < MAX_CORNERS*2; i++){
        if(track_status[i] == 1){
            int dx = abs((int) cornerscurr_11[i].x - (int) cornersprev_11[i].x);
            int dy = abs((int) cornerscurr_11[i].y - (int) cornersprev_11[i].y);
            if(dx <  WIDTH/2 && dy < HEIGHT/2){
                if(cornersprev_11[i].x < WIDTH/2){
                    leftSumFlow[0] += dx;
                    leftSumFlow[1] += dy;
                    LS += sqrt((float)(dx*dx + dy*dy));
                    left++;
                }else{
                    rightSumFlow[0] += dx;
                    rightSumFlow[1] += dy;
                    RS += sqrt((float)(dx*dx + dy*dy));
                    right++;
                }
            }
        }
	}

	float result = balanceControlLR(false, abs(LS*10/left), abs(RS*10/right), k); //这里未做大型障碍物预测，所以直接传入false

	return result;
}

//未测试
Vec2i foeForFeatureCvPoint(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr){
	double h0 = 0;
	double h1 = 0;
	double h2 = 0;
	double h3 = 0;
	double h4 = 0;
	double h5 = 0;

	double H0 = 0;
	double H1 = 0;
	double H2 = 0;
	double H3 = 0;
	double H4 = 0;
	double H5 = 0;

	double vx = 0;
	double vy = 0;
	double ai0 = 0;
	double ai1 = 0;
	double bi = 0;

	for(int i = 0; i < MAX_CORNERS*2; i++){
		h0 = 0;
		h1 = 0;
		h2 = 0;
		h3 = 0;
		h4 = 0;
		h5 = 0;

		vx = cornerscurr[i].x-cornersprev[i].x;
		vy = cornerscurr[i].y-cornersprev[i].y;
		ai0 = vy;
		ai1 = -1*vx;
		bi = cornerscurr[i].x*vy - cornerscurr[i].y*vx;

		h0 = ai0 * bi;
		h1 = ai0 * ai0;
		h2 = ai1 * bi;
		h3 = ai0 * ai1;
		h4 = ai1 * ai1;
		h5 = ai0 * ai0 * ai1 * ai1;

		H0 += h0;
		H1 += h1;
		H2 += h2;
		H3 += h3;
		H4 += h4;
		H5 += h5;
	}
	double q = (H5 - H3 * H3);
	int foeX = (int) ((H0*H4 - H2*H3) / q);
	int foeY = (int) ((-H0*H3 + H2*H1) / q);
	return Vec2i(foeX, foeY);
}

float ttcForFeatureCvPoint(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr){
	return 1;
}

void getSpeedFromFlow(CvPoint2D32f* cornersprev_11, CvPoint2D32f* cornerscurr_11, CvPoint2D32f* cornersprev_12, CvPoint2D32f* cornerscurr_12, IplImage* imgdst){
	for (int k = -10; k < -9; k+=5)
	{
		Vec2d leftp = getSpeed(cornersprev_11, cornerscurr_11, imgdst);
		Vec2d rightp = getSpeed(cornersprev_12, cornerscurr_12, imgdst, false);

		double avgx = (leftp[0] + rightp[0])/2, avgy = (leftp[1] + rightp[1])/2;

		char buffer[50];
		sprintf(buffer, "%d	%lf	%lf	", k, avgx, avgy );
		writeFile(buffer);

		CvPoint a, b;
		a.y = 40, a.x = WIDTH-30;
		b.y = 40-avgx/10, b.x = WIDTH-30+avgy/10;
		drawArrow(a,b, imgdst, CV_RGB(255,0,0),2);

		/*	CvPoint p, q;
		p.y = 80, p.x = 175;
		q.y = 125, q.x = 240;
		cvLine(imgdst, p, q, CV_RGB(255,0,0),1);*/

		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1, 1, 0, 2);
		char c[50];
		//itoa(avgx, c, 10);
		sprintf(c, "%d", (int)avgx);
		cvPutText(imgdst, c, cvPoint(WIDTH-50,70), &font, CV_RGB(255, 0, 0));
	}
	char buffer[50];
	sprintf(buffer, "\n");
	writeFile(buffer);
}

Vec2d getSpeed(CvPoint2D32f* cornersprev, CvPoint2D32f* cornerscurr, IplImage *imgdst, bool isleft, float kangle){
	int midy = HEIGHT/2, midx = WIDTH/2;
	int count = 0, x, y;
	double w = 0, v = 0;
	static double angle = TACHOGRAPH_ANGLE*CV_PI/180;
	static double sina = sin(angle);
	static double cosa = cos(angle);
	for (int i = 0; i < MAX_CORNERS; i++)
	{
		if ((int)cornersprev[i].x < 0 || (int)cornersprev[i].x > WIDTH )
		{
			break;
		}
		int px = ((int) cornerscurr[i].x - (int) cornersprev[i].x);
		int py = ((int) cornerscurr[i].y - (int) cornersprev[i].y);
		if (px != 0 && py > 0)
		{
			//py = abs(py);
			x = isleft ? cornersprev[i].x - midx : cornersprev[i].x;
			y = cornersprev[i].y - midy;

			CvPoint a;
			//a.x = isleft ? cornerscurr_11[i].x : cornerscurr_11[i].x + midx;
			//a.y = cornerscurr_11[i].y + midy;
			a.x = x + midx;
			a.y = y + midy;
			
			if (abs(0.6923*(x+midx) - 41 - y - midy)<10)
			{
				cvCircle(imgdst, a, 3, CV_RGB(255,255,0));

				double tmp = TACHOGRAPH_FOCAL*sina + y*cosa;
			    tmp = tmp == 0 ? 1 : tmp;
				//if (abs(tmp) > 0.6 && abs(tmp) < 3.5)
				{
					double w1 = (TACHOGRAPH_FOCAL*TACHOGRAPH_HEIGHT*py)/(tmp*tmp*3.6);
					double v1 = (TACHOGRAPH_HEIGHT*x*py*cosa - TACHOGRAPH_HEIGHT*px*tmp)/(tmp*tmp*3.6);
					w += (TACHOGRAPH_FOCAL*TACHOGRAPH_HEIGHT*py)/(tmp*tmp*3.6);  //因为最后m/s换算成km/h要除3.6
					v += (TACHOGRAPH_HEIGHT*x*py - TACHOGRAPH_HEIGHT*px*tmp)/(tmp*tmp*3.6);
					count++;
					char buffer[50];
					sprintf(buffer, "%d	%d	%d	%d	%lf	%lf	%lf\n", x, y, px, py, w1, v1, tmp);
					writeFile(buffer);
				}
			}
	  }
   }
	count = count == 0 ? 1 : count;
	return Vec2d(w/count, v/count);
}


