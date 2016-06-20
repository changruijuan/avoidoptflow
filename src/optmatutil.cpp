#include "stdafx.h"
#include <cv.h>

#include "optutil.h"
#include "optmatutil.h"
#include "optdrawflow.h"
#include "optcvmatutil.h"

using namespace cv;
using namespace std;

void getTagOriginByColor(Mat &frameprev, int* tagOrigin){
	IplImage img = IplImage(frameprev);
	IplImage* imgprev = (IplImage *)cvClone(&img);
	getTagOriginByColor(imgprev, tagOrigin);
	/*Mat framehsv;
	cvtColor(frameprev, framehsv, CV_BGR2HSV);
	for (int i = 0; i < WIDTH; i++)
	{
		if (isInRange(framehsv, HEIGHT/2 - 10, i) || isInRange(framehsv, HEIGHT/2, i) || isInRange(framehsv, HEIGHT/2 + 10, i))
		{
			tagOrigin[i] = 1;
		}else{
			tagOrigin[i] = 0;
		}
	}*/
}

bool isInRange(Mat &framehsv, int row, int col){
	int dataH = (int)framehsv.row(row).col(col).data[0];
	if (dataH >= HUELOW && dataH <= HUELOW)
	{
		return true;
	}else{
		return false;
	}
}

/************************************************************************/
/* 稠密光流，光流参数类型为Mat的FOE（flow）的计算
   flow : 光流；计算方法1                                   */
/************************************************************************/
Vec2i foeForDenseMat1(Mat flow){
	double H0 = 0;
	double H1 = 0;
	double H2 = 0;
	double H3 = 0;
	double H4 = 0;
	double H5 = 0;
	
	for (int col = 0; col < flow.cols; col++){
		 for (int row = 0; row < flow.rows; row++){
			 Vec2f vel = flow.at<Vec2f>(row, col);
			 float vy = vel[1];
			 float vx = vel[0];
			 double bi = col * vy - row * vx;
			 double ai0 = vy;
			 double h0 = vy * bi;
			 double h1 = ai0 * ai0;
			 double h2 = -1 * vx * bi;
			 double h3 = -1 * vy * vx;
			 double h4 = vx * vx;
			 double h5 = vy * vy * vx * vx;
			 H0 += h0;
			 H1 += h1;
			 H2 += h2;
			 H3 += h3;
			 H4 += h4;
			 H5 += h5;
		 }
	 }
	 int foeX = (int) ((H0*H4 - H2*H3) / (H5 - pow(H3,2)));
	 int foeY = (int) ((-H0*H3 + H2*H1) / (H5 - pow(H3,2)));
	 return Vec2i(foeX, foeY);
}
/************************************************************************/
/* 稠密光流，光流参数类型为Mat的FOE（flow）的计算
   flow : 光流；计算方法2                                */
/************************************************************************/
Vec2i foeForDenseMat2(Mat flow){
	//calculate x of foe
	float cols[COLS];
	for(int col = 0; col < flow.cols; col++){
		float tmpc = 0;
		for(int row = 0; row < flow.rows; row++){
			Vec2f vel = flow.at<Vec2f>(row, col);
			tmpc += vel[0];
		}
		cols[col] = tmpc;
	}
	float colsLeft[COLS];//以col左侧的光流之和
	for(int i = 0; i < flow.cols; i++){
		colsLeft[i] = cols[i];
		if(i > 0){
			colsLeft[i] += colsLeft[i-1];
		}
	}
	float colsRight[COLS];//以col右侧的光流之和
	for(int j = flow.cols-1; j >= 0; j--){
		colsRight[j] = cols[j];
		if(j < flow.cols-1){
			colsRight[j] += colsRight[j+1];
		}
	}
	//找出x使其左右两边之差最小的为foe点的x
	float colsMin = abs(colsLeft[0] - colsRight[0]);
	int foeX = 0;
	for(int i = 1; i < flow.cols; i++){
		float tmp = abs(colsLeft[i] - colsRight[i]);
		if(tmp < colsMin){
			colsMin = tmp;
			foeX = i;
		}
	}
	//calculate y of foe
	float rows[ROWS];
	for(int row = 0; row < flow.rows; row++){
		float tmpr = 0;
		for(int col = 0; col < flow.cols; col++){
			Vec2f vel = flow.at<Vec2f>(row, col);
			tmpr += vel[0];
		}
		rows[row] = tmpr;
	}
	float rowsUp[ROWS];
	for(int i = 0; i < flow.rows; i++){
		rowsUp[i] = rows[i];
		if(i > 0){
			rowsUp[i] += rowsUp[i-1];
		}
	}
	float rowsDown[ROWS];
	for(int j = flow.rows-1; j >= 0; j--){
		rowsDown[j] = rows[j];
		if(j < flow.rows-1){
			rowsDown[j] += rowsDown[j+1];
		}
	}
	float rowsMin = abs(rowsUp[0] - rowsDown[0]);
	int foeY = 0;
	for(int i = 1; i < flow.rows; i++){
		float tmp = abs(rowsUp[i] - rowsDown[i]);
		if(tmp < rowsMin){
			rowsMin = tmp;
			foeY = i;
		}
	}
	return Vec2i(foeX, foeY);
}

/************************************************************************/
/* 稠密光流且光流参数类型为Mat的TTC计算，返回平均TTC值
   flow：光流； foeY：foe点y的坐标；ttc：计算获得的每列的ttc       */
/************************************************************************/

float ttcForDenseMat(Mat flow, float *ttc, int foeY){
	float sum  = 0;
	for (int col = 0; col < flow.cols; col++){
		float tmp = 0;
		for (int row = 0; row < flow.rows; row++){
			Vec2f vel = flow.at<Vec2f>(row, col);
			float pData = vel[1];
			//if (pData == 0){
			//	tmp += abs((row - foeY)*10); //(row - foeY)/0.1,如果为0，则认为速度为0.1
			//}else{
			//	tmp += abs((row - foeY)/(pData));
			//}
			//为了使后续ttc加tag和光流加tag原理一致，所以这里求1/ttc，因为ttc大表示安全，而光流大表示障碍物正好相反
			//所以用1/ttc作为衡量标准，1/ttc大表示障碍物,这样和光流逻辑一致
			tmp += ((row-foeY) == 0) ?  0 : pData/(row-foeY); 
		}
		//ttc[col] = tmp/(flow.rows);
		ttc[col] = tmp;
		sum += ttc[col];
	}
	return sum/(flow.cols);
}

float ttcTagForDenseMat(Mat &frameprev, Mat &framedst, Mat flow){
	float ttc[COLS];
	int tagSafe[COLS];
	int tags[COLS];
	int tagOrigin[COLS];
	getTagOriginByColor(frameprev, tagOrigin);
	if (DRAWTAGORIGINSPACE)
	{
		drawTagFreeSpace(frameprev, tagOrigin);
	}

	Vec2f foe = foeForDenseMat2(flow);
	if (IS_FOECORRECT && foe[1] < HEIGHT/2 - FOE_GAP && foe[1] > HEIGHT/2 + FOE_GAP)
	{
		foe[0] = WIDTH/2; //在计算ttc没有用，所以条件直接根据y判断，这里也跟着y变为中点的x
		foe[1] = HEIGHT/2;
	}
	float ttcAvg = ttcForDenseMat(flow, ttc, foe[1]);
	tagSafeAreaByTTC(ttc, ttcAvg*TAGFORSAFE_K, tagSafe);
	if (DRAWTAGSAFESPACE)
	{
		drawTagFreeSpace(framedst, tagSafe);
	}

	float compare = compareTag(tagOrigin, tagSafe, tags);
	if (IS_DRAWTAGLINE)
	{
		drawTagLine(framedst, tags);
	}
	return compare;
}

float flowForDenseMat(Mat flow, float* colflow){
	float sum = 0;
	for (int col = 0; col < flow.cols; col++){
		float tmp = 0;
		for (int row = 0; row < flow.rows; row++){
			Vec2f vel = flow.at<Vec2f>(row, col);
			float x = vel[0];
			float y = vel[1];
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
	return sum/(flow.cols);
}

float flowTagForDenseMat(Mat &frameprev, Mat &framedst, Mat flow){
	float colflow[COLS];
	int tagSafe[COLS];
	int tags[COLS];
	int tagOrigin[COLS];
	getTagOriginByColor(frameprev, tagOrigin);
	if (DRAWTAGORIGINSPACE)
	{
		drawTagFreeSpace(frameprev, tagOrigin);
	}

	float flowAvg = flowForDenseMat(flow, colflow);
	tagSafeAreaByTTC(colflow, flowAvg*TAGFORSAFE_K, tagSafe);
	if (DRAWTAGSAFESPACE)
	{
		drawTagFreeSpace(framedst, tagSafe);
	}

	float compare = compareTag(tagOrigin, tagSafe, tags);
	if (IS_DRAWTAGLINE)
	{
		drawTagLine(framedst, tags);
	}
	return compare;
}

float balanceForDenseMat(Mat flow, Mat &framedst, float k, int px, int py){
	Vec2i leftSumFlow = Vec2i(0, 0);
	float up = EDGE*HEIGHT;
	float down = (1-EDGE)*HEIGHT;
	for (int i = 0; i < px; i++)
	{
		for (int j = up; j < down; j++)
		{
			leftSumFlow[0] += flow.at<Vec2i>(j, i)[0];
			leftSumFlow[1] += flow.at<Vec2i>(j, i)[1];
		}
	}
	Vec2i rightSumFlow = Vec2i(0, 0);
	for (int i = px; i < WIDTH; i++)
	{
		for(int j = up; j < down; j++){
			rightSumFlow[0] += flow.at<Vec2i>(j, i)[0];
			rightSumFlow[1] += flow.at<Vec2i>(j, i)[1];
		}
	}

	leftSumFlow[0] = abs(leftSumFlow[0] / px);
	leftSumFlow[1] = abs(leftSumFlow[1] / px);
	rightSumFlow[0] = abs(rightSumFlow[0] / (WIDTH - px));
	rightSumFlow[1] = abs(rightSumFlow[1] / (WIDTH - px));

	if(IS_FLOW_WRITE_FILE){
		char buffer[50];
		sprintf(buffer, "%d	%d\n", leftSumFlow[0], rightSumFlow[0]);
		writeFile(buffer);
	}

	//bool isBig = isBigObstacleMat(framedst, flow);
	float result = balanceControlLR(false, leftSumFlow[0], rightSumFlow[0], k);

	Vec2i diffFlow = Vec2i(leftSumFlow[0] - rightSumFlow[0], rightSumFlow[1] - leftSumFlow[1]);
	//line(framedst, cvPoint(px, py), cvPoint(px+diffFlow[0], py), CV_RGB(0,255,0), 1);
	drawArrow(cvPoint(px, py),cvPoint(px+diffFlow[0]/10e5, py), framedst, CV_RGB(0,255,0) , 3);

	int resulttxt = 0;
	if (result == -2*INT_FLOAT)
	{
		resulttxt = -2;
		putText(framedst, "S", cvPoint(20, 20),CV_FONT_HERSHEY_DUPLEX, 1.0f, CV_RGB(255, 0, 0));
	}else if (result  == 0)
	{
		resulttxt = 0;
		putText(framedst, "F", cvPoint(20, 20), CV_FONT_HERSHEY_DUPLEX, 1.0f, CV_RGB(255, 0, 0));
	}else if (result < 0)
	{
		resulttxt = -1;
		putText(framedst, "L", cvPoint(20, 20), CV_FONT_HERSHEY_DUPLEX, 1.0f, CV_RGB(255, 0, 0));
	}else if (result > 0)
	{
		resulttxt = 1;
		putText(framedst, "R", cvPoint(20, 20), CV_FONT_HERSHEY_DUPLEX, 1.0f, CV_RGB(255, 0, 0));
	}
	
	/*char c[50];
	sprintf(c, "%f", result/100.0);
	putText(framedst, c, cvPoint(10, 50), CV_FONT_HERSHEY_DUPLEX, 1.0f, CV_RGB(255, 0, 0));*/

	if (IS_BALANCE_RESULT_WRITE_FILE)
	{
		char buffer[50];
		sprintf(buffer, "%d\n", resulttxt);
		writeFile(buffer);
	}

	return result;
}

bool isBigObstacleMat(Mat &framedst, Mat flow){
	int sumR = 0, sumG = 0, sumB = 0;
	int count = 0;
	for(int i = EDGE_OBS*HEIGHT; i < (1-EDGE_OBS)*HEIGHT; i++){
		for(int j = EDGE_OBS*WIDTH; j < (1-EDGE_OBS)*WIDTH; j++ ){
			sumB += (int)framedst.row(i).col(j).data[0];
			sumG += (int)framedst.row(i).col(j).data[1];
			sumR += (int)framedst.row(i).col(j).data[2];
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
			if(abs((int)framedst.row(i).col(j).data[0] - avgB) < COLOR_SCALE){
				timers += 1;
			}
			if(abs((int)framedst.row(i).col(j).data[1] - avgG) < COLOR_SCALE){
				timers += 1;
			}
			if(abs((int)framedst.row(i).col(j).data[2] - avgR) < COLOR_SCALE){
				timers += 1;
			}
			if (timers == 3)
			{
				timerCount ++;
				if (abs((flow.at<Vec2i>(i, j)[0])) <= FLOW_ZERO)
				{
					flowZeroCount ++;
				}
			}
		}
	}
	float timerPro = (timerCount*1.0)/count;
	float flowZeroPro = (flowZeroCount*1.0)/timerCount;
	//if (timerPro > THRESHOLD_TIMER && flowZeroPro > THRESHOLD_ZERO) //matlab
	if ((!IS_ROS && timerPro > THRESHOLD_TIMER && flowZeroPro > THRESHOLD_ZERO)||(IS_ROS && timerPro > THRESHOLD_TIMER && flowZeroPro <= THRESHOLD_ZERO))//matlab
	{
		return true;
	}else{
		return false;
	}	
}

void getSpeedFromFlow(Mat flow, Mat &framedst){
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
					Vec2f flow_at_point = flow.at<Vec2i>(i, j);
					float px = flow_at_point[0];
					float py = flow_at_point[1];
					if (fabs(px) > UNKNOWN_FLOW_THRESH || fabs(py) > UNKNOWN_FLOW_THRESH)
					{
						continue;
					}
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
		drawArrow(a,b, framedst, CV_RGB(255,0,0),2);

		/*	CvPoint p, q;
		p.y = 80, p.x = 175;
		q.y = 125, q.x = 240;
		cvLine(imgdst, p, q, CV_RGB(255,0,0),1);*/
		
		char c[50];
		//itoa(w/count, c, 10);
		sprintf(c, "%d", (int)(w/count));
		putText(framedst, c, cvPoint(WIDTH-50,70), CV_FONT_HERSHEY_DUPLEX, 1.0f, CV_RGB(255, 0, 0));
	}
	char buffer[50];
	sprintf(buffer, "\n");
	writeFile(buffer);
}


