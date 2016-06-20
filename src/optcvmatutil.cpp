#include "stdafx.h"
#include <cv.h>
#include <highgui.h>

#include "optdrawflow.h"
#include "optutil.h"
#include "optcvmatutil.h"
#include <math.h>
#include "var.h"

using namespace cv;
using namespace std;

void getTagOriginByColor(IplImage* imgprev, int* tagOrigin){
	if (ISFILETAGORIGIN)
	{
		getTagOriginFromFile(tagOrigin);
	}else{
		IplImage* imghsv = cvCreateImage(cvGetSize(imgprev), 8, 3);
		//IplImage* imgsrc_32 = cvCreateImage(cvGetSize(imgprev), 8, imgprev->nChannels);
		//cvConvertScale(imgprev, imgsrc_32, 1.0/255.0, 0);
		cvCvtColor(imgprev, imghsv, CV_BGR2HSV);
		for (int i = 0; i < WIDTH; i++)
		{
			if (isInRange(imghsv, HEIGHT/2 - 10, i) || isInRange(imghsv, HEIGHT/2, i) || isInRange(imghsv, HEIGHT/2 + 10, i))
			{
				tagOrigin[i] = 1;//障碍物，不安全
			}else{
				tagOrigin[i] = 0;
			}
		}
	}
}

bool isInRange(IplImage* imghsv, int row, int col){
	uchar *data = (uchar*)imghsv->imageData;
	int step = imghsv->widthStep / sizeof(uchar);
	int dataH = (int)data[row*step+col*3+0];
	if (dataH >= HUELOW && dataH <= HUEUP)
	{
		return true;
	}else{
		return false;
	}
}

/************************************************************************/
/* 稠密光流，光流参数类型为CvMat的FOE（x，y）的计算
   velx：x方向光流；vely：y方向光流                                     */
/************************************************************************/
Vec2i foeForDenseCvMat1(CvMat* velx, CvMat* vely){
	int length = velx->height * velx->width;/*x方向与y方向的图像大小是一样的*/
	//计算H0 = +(ai0*bi);
	//计算H1 = +(ai0*ai0);
	//计算H2 = +(ai1*bi);
	//计算H3 = +(ai0*ai1);
	//计算H4 = +(ai1*ai1);
	//计算H5 = +(ai0**2 * ai1**2);
	double H0 = 0;
	double H00 = 0;
	double H1 = 0;
	double H11 = 0;
	double H2 = 0;
	double H22 = 0;
	double H3 = 0;
	double H33 = 0;
	double H4 = 0;
	double H44 = 0;
	double H5 = 0;
	double H55 = 0;
	for (int col = 0; col < velx->width; col += 200){
		for (int row = 0; row < velx->height; row += 200){
			float* vy = vely->data.fl + row * vely->step/4 + col;
			float* vx = velx->data.fl + row * velx->step/4 + col;
			float ai0 = *vy;
			float ai1 = -*vx;
			double bi = col * *vy - row * *vx;
			double h0 = ai0 * bi;
			double h1 = ai0 * ai0;
			double h2 = ai1 * bi;
			double h3 = ai0 * ai1;
			double h4 = ai1 * ai1;
			double h5 = ai0 * ai0 * ai1 * ai1;
			H00 += h0;
			H11 += h1;
			H22 += h2;
			H33 += h3;
			H44 += h4;
			H55 += h5;
		}
		H0 += H00;
		H1 += H11;
		H2 += H22;
		H3 += H33;
		H4 += H44;
		H5 += H55;
	}
	/*printf("H0->%f\n", H0);
	printf("H1->%f\n", H1);
	printf("H2->%f\n", H2);
	printf("H3->%f\n", H3);
	printf("H4->%f\n", H4);
	printf("H5->%f\n", H5);*/

	//计算foeX = (H0*H4 - H2*H3) / (H5 - H3**2)
	double q = (H5 - H3 * H3);
	//printf("q->%d\n", q);
	int foeX = (int) ((H0*H4 - H2*H3) / q);
	//printf("foeX->%d\n", foeX);
	//计算foeY = (H0*H3 - H2*H1) / (H5 - H3**2)
	int foeY = (int) ((-H0*H3 + H2*H1) / q);
	//printf("foeY->%d\n", foeY);
	return Vec2i(foeX, foeY);
}

Vec2i foeForDenseCvMat2(CvMat* velx, CvMat* vely){
	//calculate x of foe
	float cols[COLS];
	for (int col = 0; col < velx->width; col++)
	{
		float tmp = 0;
		for (int row = 0; row < velx->height; row++)
		{
			float* pData = velx->data.fl + row*velx->step/4;
			tmp += *(pData + col); 
		}
		cols[col] = tmp;
	}
	float colsLeft[COLS];//以col左侧的光流之和
	for (int i = 0; i < velx->width; i++)
	{
		colsLeft[i] = cols[i];
		if (i > 0)
		{
			colsLeft[i] += colsLeft[i-1];
		}
	}
	float colsRight[COLS];//以col右侧的光流之和
	for (int j = velx->width - 1; j >= 0; j--)
	{
		colsRight[j] = cols[j];
		if (j < velx->width -1)
		{
			colsRight[j] += colsRight[j+1];
		};
	}
	//找出x使其左右两边之差最小的为foe点的x
	float colsMin = abs(abs(colsLeft[0]) - abs(colsRight[0]));
	int foeX = 0;
	for (int i = 1; i < velx->width; i++)
	{
		float tmp = abs(abs(colsLeft[i]) - abs(colsRight[i]));
		if (tmp < colsMin)
		{
			colsMin = tmp;
			foeX = i;
		}
	}

	//calculate y of foe
	float rows[ROWS];
	for (int row = 0 ; row < vely->height; row++ ){
		float tmp = 0;
		float* pData = vely->data.fl + row*vely->step/4;
		for(int col = 0 ; col < vely->width; col++){
			tmp += *(pData + col);
		}
		rows[row] = tmp;
	}
	float rowsUp[ROWS];
	for (int i = 0; i < vely->height; i++)
	{
		rowsUp[i] = rows[i];
		if (i > 0)
		{
			rowsUp[i] += rowsUp[i-1];
		}
	}
	float rowsDown[ROWS];
	for (int j = vely->height - 1; j >= 0; j--)
	{
		rowsDown[j] = rows[j];
		if (j < vely->height-1)
		{
			rowsDown[j] += rowsDown[j+1];
		}
	}
	float rowsMin = abs(abs(rowsUp[0]) - abs(rowsDown[0]));
	int foeY = 0;
	for (int i = 1; i < vely->height; i++)
	{
		float tmp = abs(abs(rowsUp[i]) - abs(rowsDown[i]));
		if (tmp < rowsMin)
		{
			rowsMin = tmp;
			foeY = i;
		}
	}
	printf("FOE : [cols(x) = %d, rows(y) = %d]\n", foeX, foeY);

	return Vec2i(foeX, foeY);
}

/************************************************************************/
/* 稠密光流且光流参数类型为CvMat的TTC计算，返回平均TTC值
   vely：y方向光流； foeY：foe点y的坐标；ttc：计算获得的每列的ttc       */
/************************************************************************/
float ttcForDenseCvMat(CvMat* vely, float *ttc, int foeY){
	float sum  = 0;
	for (int col = 0; col < vely->width; col++)
	{
		float tmp = 0;
		for (int row = 0; row < vely->height; row++)
		{
			float* pData = vely->data.fl + row*vely->step/4 + col;
			//if (*pData == 0)
			//{
			//	tmp += abs((row - foeY)*10); //(row - foeY)/0.1,如果为0，则认为速度为0.1
			//}else{
			//	tmp += abs((row - foeY)/(*pData));
			//}
			//为了使后续ttc加tag和光流加tag原理一致，所以这里求1/ttc，因为ttc大表示安全，而光流大表示障碍物正好相反
			//所以用1/ttc作为衡量标准，1/ttc大表示障碍物,这样和光流逻辑一致
			tmp += ((row-foeY) == 0) ?  0 : (*pData)/(row-foeY); 
		}
		//ttc[col] = tmp/(vely->height);
		ttc[col] = tmp;
		sum += ttc[col];
	}
	return sum/(vely->width);
}

float ttcTagForDenseCvMat(IplImage* imgprev, IplImage* imgdst, CvMat* velx, CvMat* vely){
	float ttc[COLS];
	int tagSafe[COLS];
	int tags[COLS];
	int tagOrigin[COLS];
	getTagOriginByColor(imgprev, tagOrigin);
	if (DRAWTAGORIGINSPACE)
	{
		drawTagFreeSpace(imgprev, tagOrigin);
	}
	Vec2f foe = foeForDenseCvMat2(velx, vely);
	if (IS_FOECORRECT && foe[1] < HEIGHT/2 - FOE_GAP && foe[1] > HEIGHT/2 + FOE_GAP)
	{
		foe[0] = WIDTH/2; //在计算ttc没有用，所以条件直接根据y判断，这里也跟着y变为中点的x
		foe[1] = HEIGHT/2;
	}
	float ttcAvg = ttcForDenseCvMat(vely, ttc, foe[1]);
	tagSafeAreaByTTC(ttc, ttcAvg*TAGFORSAFE_K, tagSafe);
	if (DRAWTAGSAFESPACE)
	{
		drawTagFreeSpace(imgdst, tagSafe);
	}
	float compare = compareTag(tagOrigin, tagSafe, tags);
	if (IS_DRAWTAGLINE)
	{
		drawTagLine(imgdst, tags);
	}
	return compare;
}

float flowForDenseCvMat(CvMat* velx, CvMat* vely, float* colflow){
	float sum = 0;
	for (int col = 0; col < WIDTH; col++){
		float tmp = 0;
		for (int row = 0; row < HEIGHT; row++){
			float* px = velx->data.fl + row*velx->step/4 + col;
			float* py = vely->data.fl + row*vely->step/4 + col;
			float x = *px;
			float y = *py;
			if (IS_XANDY)
			{
				tmp += sqrt(x*x + y*y);
			}else{
				tmp += abs(x);
			}
		}
		//ttc[col] = tmp/(flow.rows);
		colflow[col] = tmp;
		sum += colflow[col];
	}
	return sum/(WIDTH);
}

float flowTagForDenseCvMat(IplImage* imgprev, IplImage* imgdst, CvMat* velx, CvMat* vely){
	float colflow[COLS];
	int tagSafe[COLS];
	int tags[COLS];
	int tagOrigin[COLS];
    getTagOriginByColor(imgprev, tagOrigin); //just for labeling, for avoidance is no useless.
	if (DRAWTAGORIGINSPACE)
	{        
		drawTagFreeSpace(imgprev, tagOrigin);
	}
	float flowAvg = flowForDenseCvMat(velx, vely, colflow);
    tagSafeAreaByTTC(colflow, flowAvg*TAGFORSAFE_K, tagSafe);

    if (DRAWTAGSAFESPACE)
	{
		drawTagFreeSpace(imgdst, tagSafe);
	}
	float compare = compareTag(tagOrigin, tagSafe, tags);
	if (IS_DRAWTAGLINE)
	{
		drawTagLine(imgdst, tags);
	}
	return compare;
}

//利用左右光流平衡
float balanceForDenseCvMat(CvMat* velx, CvMat* vely, IplImage* imgprev, IplImage* imgdst, float k, int px, int py) {
    int isBigLeft = isBigObstacleColor(imgdst, velx, true);
    int isBigRight = isBigObstacleColor(imgdst, velx, false);

    filterFLowCvMat(velx, vely);

    Vec2i leftSumFlow = Vec2i(0, 0);
    float up = EDGE*HEIGHT;
    float down = (1-EDGE)*HEIGHT;
	for (int i = 0; i < px; i++)
	{
		for (int j = up; j < down; j++)
		{
			leftSumFlow[0] += (int) cvGetReal2D(velx, j, i);
			leftSumFlow[1] += (int) cvGetReal2D(vely, j, i);
		}
	}
	Vec2i rightSumFlow = Vec2i(0, 0);
	for (int i = px; i < WIDTH; i++)
	{
		for(int j = up; j < down; j++){
			rightSumFlow[0] += (int) cvGetReal2D(velx, j, i);
			rightSumFlow[1] += (int) cvGetReal2D(vely, j, i);
		}
	}
	//printf("pre left :%d,%d  right:%d,%d\n", leftSumFlow[0], leftSumFlow[1], rightSumFlow[0], rightSumFlow[1]);
	leftSumFlow[0] = abs(leftSumFlow[0] / px);
	leftSumFlow[1] = abs(leftSumFlow[1] / px);
	rightSumFlow[0] = abs(rightSumFlow[0] / (WIDTH - px));
	rightSumFlow[1] = abs(rightSumFlow[1] / (WIDTH - px));

	if(IS_FLOW_WRITE_FILE){
		char buffer[50];
		sprintf(buffer, "leftSumFlow	%d	rightSumFlow	%d\n", leftSumFlow[0], rightSumFlow[0]);
		writeFile(buffer);
	}

    float result  = balanceControlLR(isBigLeft, isBigRight, leftSumFlow[0], rightSumFlow[0], k);

    drawOrientation(leftSumFlow, rightSumFlow, px, py, result, imgdst);

    return result;
}

//利用左右光流平衡+TTC
float balanceWithTTCDenseCvMat(CvMat* velx, CvMat* vely, IplImage* imgprev, IplImage* imgdst, float k, int px, int py) {
    int isBigLeft = isBigObstacleColor(imgdst, velx, true);
    int isBigRight = isBigObstacleColor(imgdst, velx, false);

    filterFLowCvMat(velx, vely);

    Vec2i leftSumFlow = Vec2i(0, 0);
    float up = EDGE*HEIGHT;
    float down = (1-EDGE)*HEIGHT;
    for (int i = 0; i < px; i++)
    {
        for (int j = up; j < down; j++)
        {
            leftSumFlow[0] += (int) cvGetReal2D(velx, j, i);
            leftSumFlow[1] += (int) cvGetReal2D(vely, j, i);
        }
    }
    Vec2i rightSumFlow = Vec2i(0, 0);
    for (int i = px; i < WIDTH; i++)
    {
        for(int j = up; j < down; j++){
            rightSumFlow[0] += (int) cvGetReal2D(velx, j, i);
            rightSumFlow[1] += (int) cvGetReal2D(vely, j, i);
        }
    }
    //printf("pre left :%d,%d  right:%d,%d\n", leftSumFlow[0], leftSumFlow[1], rightSumFlow[0], rightSumFlow[1]);
    leftSumFlow[0] = abs(leftSumFlow[0] / px);
    leftSumFlow[1] = abs(leftSumFlow[1] / px);
    rightSumFlow[0] = abs(rightSumFlow[0] / (WIDTH - px));
    rightSumFlow[1] = abs(rightSumFlow[1] / (WIDTH - px));

    if(IS_FLOW_WRITE_FILE){
        char buffer[50];
        sprintf(buffer, "leftSumFlow	%d	rightSumFlow	%d\n", leftSumFlow[0], rightSumFlow[0]);
        writeFile(buffer);
    }

    float result  = balanceControlLR(isBigLeft, isBigRight, leftSumFlow[0], rightSumFlow[0], k);

    result = ttcCrossForDenseCvMat (result, velx, vely, imgdst);

    drawOrientation(leftSumFlow, rightSumFlow, px, py, result, imgdst);

    return result;
}


float ttcCrossForDenseCvMat (float result, CvMat* velx, CvMat* vely, IplImage* imgdst) {

    float turn = 0;
    if (result >= -1 && result <= 1) {
        float colflow[COLS];
        int tagSafe[COLS];

        float flowAvg = flowForDenseCvMat(velx, vely, colflow);
        tagSafeAreaByTTC(colflow, flowAvg*TAGFORSAFE_K, tagSafe);

        turn =  getMaxConnectedSpace(imgdst, tagSafe);
    }
    turn =  turn == 0 ? result : turn;

    return turn;
}

bool isBigObstacleColor(IplImage* imgdst, CvMat* velx){
    uchar *data = (uchar*)imgdst->imageData;
	int step = imgdst->widthStep / sizeof(uchar);
	int channels = imgdst->nChannels;
	int sumR = 0, sumG = 0, sumB = 0;
	int count = 0;
	for(int i = EDGE_OBS*HEIGHT; i < (1-EDGE_OBS)*HEIGHT; i++){
		for(int j = EDGE_OBS*WIDTH; j < (1-EDGE_OBS)*WIDTH; j++ ){
			sumB += (int)data[i*step+j*channels+0];
			sumG += (int)data[i*step+j*channels+1];
			sumR += (int)data[i*step+j*channels+2];
			count ++ ;
		}
	}
	int avgB = sumB / count;
	int avgG = sumG / count;
	int avgR = sumR / count;
	int timers;
	int timerCount = 0;
	int flowZeroCount = 0;
	for(int i = EDGE_OBS*HEIGHT; i < (1-EDGE_OBS)*HEIGHT; i++){
		for(int j = EDGE_OBS*WIDTH; j < (1-EDGE_OBS)*WIDTH; j++ ){
			timers = 0;
			if(abs((int)data[i*step+j*channels+0] - avgB) < COLOR_SCALE){
				timers += 1;
			}
			if(abs((int)data[i*step+j*channels+1] - avgG) < COLOR_SCALE){
				timers += 1;
			}
			if(abs((int)data[i*step+j*channels+2] - avgR) < COLOR_SCALE){
				timers += 1;
			}
			if (timers == 3)
			{
				timerCount ++;
				if (abs(((int) cvGetReal2D(velx, i, j))) <= FLOW_ZERO)
				{
					flowZeroCount ++;
				}
			}
		}
	}
	float timerPro = (timerCount*1.0)/count;
	float flowZeroPro = (flowZeroCount*1.0)/timerCount;
    if ((!IS_ROS && timerPro > THRESHOLD_TIMER && flowZeroPro > THRESHOLD_ZERO)||(IS_ROS && timerPro > THRESHOLD_TIMER && flowZeroPro <= THRESHOLD_ZERO))//matlab
    {
        return true;
    }else{
        return false;
    }

}

int isBigObstacleColor(IplImage* imgdst, CvMat* velx, bool isleft){
    uchar *data = (uchar*)imgdst->imageData;
    int step = imgdst->widthStep / sizeof(uchar);
    int channels = imgdst->nChannels;
    int sumR = 0, sumG = 0, sumB = 0;
    int count = 0;
    int start = isleft ? EDGE_OBS*WIDTH : WIDTH/2;
    int end = isleft ? WIDTH/2 : (1-EDGE_OBS)*WIDTH;
    for(int i = EDGE_OBS*HEIGHT; i < (1-EDGE_OBS)*HEIGHT; i++){
        for(int j = start; j < end; j++ ){
            sumB += (int)data[i*step+j*channels+0];
            sumG += (int)data[i*step+j*channels+1];
            sumR += (int)data[i*step+j*channels+2];
            count ++ ;
        }
    }
    int avgB = sumB / count;
    int avgG = sumG / count;
    int avgR = sumR / count;
    int timers;
    int timerCount = 0;
    int flowZeroCount = 0;
    for(int i = EDGE_OBS*HEIGHT; i < (1-EDGE_OBS)*HEIGHT; i++){
        for(int j = start; j < end; j++ ){
            timers = 0;
            if(abs((int)data[i*step+j*channels+0] - avgB) < COLOR_SCALE){
                timers += 1;
            }
            if(abs((int)data[i*step+j*channels+1] - avgG) < COLOR_SCALE){
                timers += 1;
            }
            if(abs((int)data[i*step+j*channels+2] - avgR) < COLOR_SCALE){
                timers += 1;
            }
            if (timers == 3)
            {
                timerCount ++;
                if (abs(((int) cvGetReal2D(velx, i, j))) <= FLOW_ZERO)
                {
                    flowZeroCount ++;
                }
            }
        }
    }
    float timerPro = (timerCount*1.0)/count;
    float flowZeroPro = (flowZeroCount*1.0)/timerCount;
    if(IS_WRITE_BIG_OBS){
        CvFont font;
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1, 1, 0, 2);
        char c[50];
        sprintf(c, "%d ", img_num);
        cvPutText(imgdst, c, cvPoint(10,40), &font, CV_RGB(255, 0, 0));

        char buffer[200];
        sprintf(buffer, "%d\t%d\t%.2f\t%.2f\t%d\n", img_num, isleft, timerPro, flowZeroPro, filtercount);
        writeFile(buffer);
    }
//    if ((!IS_ROS && timerPro > THRESHOLD_TIMER && flowZeroPro > THRESHOLD_ZERO)||(IS_ROS && timerPro > THRESHOLD_TIMER && flowZeroPro <= THRESHOLD_ZERO))//matlab
//    {
//        return true;
//    }else{
//        return false;
//    }
    if (timerPro > THRESHOLD_TIMER && flowZeroPro <= THRESHOLD_ZERO) { // bigObstacle
        return 1;
    }
    else if (timerPro > THRESHOLD_TIMER && flowZeroPro > THRESHOLD_ZERO) { // it is sky, same color and zero flow
        return 2;
    }
    else { // use balance
        return 0;
    }
}

bool isBigObstacleHSV(IplImage* imgdst, CvMat* velx, bool isleft){
    IplImage* imgsrc = cvCreateImage(cvGetSize(imgdst), IPL_DEPTH_32F, 3);
    cvConvertScale( imgdst, imgsrc, 1.0/255.0, 0 );
    IplImage* imghsv = cvCreateImage(cvGetSize(imgdst), IPL_DEPTH_32F, 3);
    cvCvtColor(imgsrc, imghsv, CV_BGR2HSV);
    int start = isleft ? EDGE_OBS*WIDTH : WIDTH/2;
    int end = isleft ? WIDTH/2 : (1-EDGE_OBS)*WIDTH;
    uchar *data = (uchar*)imghsv->imageData;
    int step = imghsv->widthStep / sizeof(uchar);
    int channels = imghsv->nChannels;
    int sumH = 0;
    int count = 1;
    for(int i = EDGE_OBS*HEIGHT; i < (1-EDGE_OBS)*HEIGHT; i++){
        for(int j = start; j < end; j++ ){
            sumH += (int)data[i*step+j*channels+0];
            count ++ ;
        }
    }
    int avgH = sumH / count;
    int timerCount = 1;
    int flowZeroCount = 1;
    for(int i = EDGE_OBS*HEIGHT; i < (1-EDGE_OBS)*HEIGHT; i++){
        for(int j = start; j < end; j++){
            if(abs((int)data[i*step+j*channels+0] - avgH) < HUESCALE){
                timerCount ++;
                int flow = abs(((int) cvGetReal2D(velx, i, j)));
                if (flow <= FLOW_ZERO)
                {
                    flowZeroCount ++;
                }
            }
        }
    }
    float timerPro = (timerCount*1.0)/count;
    float flowZeroPro = (flowZeroCount*1.0)/timerCount;
    //cvReleaseImage(&imgsrc);
    //cvReleaseImage(&imghsv);
    if(IS_WRITE_BIG_OBS){
        CvFont font;
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1, 1, 0, 2);
        char c[50];
        sprintf(c, "%d", img_num);
        cvPutText(imgdst, c, cvPoint(10,40), &font, CV_RGB(255, 0, 0));

        char buffer[200];
        sprintf(buffer, "%d\t%d\t%.2f\t%.2f\n", img_num, isleft, timerPro, flowZeroPro);
        writeFile(buffer);
    }
    if ((!IS_ROS && timerPro > THRESHOLD_TIMER && flowZeroPro > THRESHOLD_ZERO)||(IS_ROS && timerPro > THRESHOLD_TIMER && flowZeroPro <= THRESHOLD_ZERO))//matlab
    {
        return true;
    }else{
        return false;
    }
}

void getSpeedFromFlow(CvMat* velx, CvMat* vely, IplImage *imgdst){
	static int midy = HEIGHT/2, midx = WIDTH/2;
	int count = 0, x, y;
	double w = 0, v = 0;
	for (int k = -10; k < -9; k+=5)
	{
		//double angle = TACHOGRAPH_ANGLE*CV_PI/180;
		double angle = k*CV_PI/180;
		double sina = sin(angle);
		double cosa = cos(angle);
		for (int i = HEIGHT*TACHOGRAPH_UPY; i < HEIGHT*TACHOGRAPH_DOWNY; i++ )
		{
			for (int j = WIDTH*TACHOGRAPH_UPX; j < WIDTH*TACHOGRAPH_DOWNX; j++)
			{
		/*for (int i = 80 ; i < HEIGHT*TACHOGRAPH_DOWNY; i++ )
		{
			for (int j = 175; j < WIDTH*TACHOGRAPH_DOWNX; j++)
			{*/
				//if (abs(0.6923*j - 41 - i)<10)
				{
					/*CvPoint a;
					a.x = j, a.y = i;
					cvCircle(imgdst, a, 0.1, CV_RGB(0,255,0));*/
					int px = (int) cvGetReal2D(velx, i, j);
					int py = (int) cvGetReal2D(vely, i, j);
					if (px != 0 && py > 0)
					{
						//py = abs(py);
						x = j - midx, y = i - midy;
						double tmp = TACHOGRAPH_FOCAL*sina + y*cosa;
						//if (abs(tmp) > 0.6 && abs(tmp) < 3.5)
						{
							double w1 = (TACHOGRAPH_FOCAL*TACHOGRAPH_HEIGHT*py)/(tmp*tmp*3.6);
							double v1 = (TACHOGRAPH_HEIGHT*x*py*cosa - TACHOGRAPH_HEIGHT*px*tmp)/(tmp*tmp*3.6);
							w += (TACHOGRAPH_FOCAL*TACHOGRAPH_HEIGHT*py)/(tmp*tmp*3.6);  //因为最后m/s换算成km/h要除3.6
							v += (TACHOGRAPH_HEIGHT*x*py - TACHOGRAPH_HEIGHT*px*tmp)/(tmp*tmp*3.6);
							count++;
						/*	char buffer[50];
							sprintf(buffer, "%d	%d	%d	%d	%lf	%lf	%lf\n", x, y, px, py, w1, v1, tmp);
							writeFile(buffer);*/
						}
					}
				}
			}
		}
		count = count == 0 ? 1 : count;
		char buffer[50];
		sprintf(buffer, "%d	%lf	%lf	", k, w/count, v/count);
		writeFile(buffer);

		CvPoint a, b;
		a.y = 40, a.x = WIDTH-30;
		b.y = 40-(w/count)/10, b.x = WIDTH-30+(v/count)/10;
		drawArrow(a,b, imgdst, CV_RGB(255,0,0),2);

		/*	CvPoint p, q;
		p.y = 80, p.x = 175;
		q.y = 125, q.x = 240;
		cvLine(imgdst, p, q, CV_RGB(255,0,0),1);*/
		

		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1, 1, 0, 2);
		char c[50];
		//itoa(w/count, c, 10);
		sprintf(c, "%d", (int)(w/count));
		cvPutText(imgdst, c, cvPoint(WIDTH-50,70), &font, CV_RGB(255, 0, 0));
	}
	char buffer[50];
	sprintf(buffer, "\n");
	writeFile(buffer);
}

void filterFLowCvMat(CvMat* velx, CvMat* vely){
    filtercount = 0;
    int unZeroCount = 0;
    float sumFlow = 0;
    float avgFlowThreshold = 0;
    for (int col = 0; col < WIDTH; col++){
        for (int row = 0; row < HEIGHT; row++){
            float* px = velx->data.fl;
            float* py = vely->data.fl;
            float dx = (px + row * velx->step/4)[col];
            float dy = (py + row * vely->step/4)[col];
            float dis = sqrt(dx * dx + dy * dy);
            if(dis > MAX_OPTFLOW || dis == 0){
                (px + row * velx->step/4)[col] = 0;
                (py + row * vely->step/4)[col] = 0;
                filtercount++;
            }else {
                sumFlow += dis;
                unZeroCount++;
            }
        }
    }

    avgFlowThreshold = (sumFlow / unZeroCount) * MEAN_OPTFLOW_THRESHOLD;
    for (int col = 0; col < WIDTH; col++){
        for (int row = 0; row < HEIGHT; row++){
            float* px = velx->data.fl;
            float* py = vely->data.fl;
            float dx = (px + row * velx->step/4)[col];
            float dy = (py + row * vely->step/4)[col];
            float dis = sqrt(dx * dx + dy * dy);
            if(dis > avgFlowThreshold){
                (px + row * velx->step/4)[col] = 0;
                (py + row * vely->step/4)[col] = 0;
                filtercount++;
            }
        }
    }

}



