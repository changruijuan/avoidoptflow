#include "stdafx.h"
#include "optutil.h"
#include "optdrawflow.h"
#include "var.h"
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

using namespace std;
using namespace cv;

/***************************************************************************************/
/*    根据TTC，标记是否为安全区域，暂定大于ttcAvg为安全0，反之，不安全1，然后再调整        
      cols 列数；ttc：计算的ttc数组；ttcAvg：ttc平均值；k：比例系数
      tagSafe : 通过比较规则获得是否安全，标记之                                       */
/***************************************************************************************/
void tagSafeAreaByTTC(float* ttc, float threshold, int *tagSafe){
	for (int i = 0; i < WIDTH; i++)
	{
		if (ttc[i] < threshold)
		{
			tagSafe[i] = 0; 
		}else{
			tagSafe[i] = 1;
		}
	}
	 if (IS_TAGINTER) //为了充分插值，则插值5次
	{
		tagSafeInter(tagSafe);
		tagSafeInter(tagSafe);
		tagSafeInter(tagSafe);
		tagSafeInter(tagSafe);
		tagSafeInter(tagSafe);
		tagSafeConnected(tagSafe);
		//tagSafeConnected(tagSafe);
		//tagSafeConnected(tagSafe);
		//tagSafeConnected(tagSafe);
		//tagSafeConnected(tagSafe);
	}
}

void tagSafeInter(int *tagSafe){
	for (int i = 0; i < WIDTH; i++)
	{
		int j, tag, count  = 0, all = 0;
		for (j = i - TAGINTER_GAP; j < i + TAGINTER_GAP; j ++)
		{
			if (j!=i && j>=0 && j<WIDTH){
				if (tagSafe[j] != tagSafe[i])
				{
					count ++;
					tag = tagSafe[j];
				}
				all ++;
			}
		}
        if (count >= all*0.7)
		{
			tagSafe[i] = tag;
		}
	}
}

void tagSafeConnected(int *tagSafe){
	int left = 0, midleft = -1, midright = -1, right = -1;
	int origin = tagSafe[0];
	int i;
    //先找出the first, left、midleft、midright、right四个变量
    for (; i < WIDTH; i++)
	{
		if (tagSafe[i] != origin)
		{
			if (midleft == -1)
			{
				midleft = i;
				origin = tagSafe[i];
			}else if (midright == -1)
			{
				midright = i;
				origin = tagSafe[i];
			}else if (right == -1)
			{
				right = i;
				origin = tagSafe[i];
				break;
			}
		}
	}
	for(; i < WIDTH; i++)
	{
		if (right != -1)
		{
			if (ISTAGCONINTERPRO)
			{
				tagConnectionInterPro(tagSafe, left, midleft, midright, right);
			}else{
				tagConnectionInter(tagSafe, left, midleft, midright, right);
			}	
			left = midleft;
			midleft = midright; 
			midright = right;
			right = -1;
		}else{
			if (tagSafe[i] != origin)
			{
				right = i;
				origin = tagSafe[i];
			}
		}
	}
}

float getMaxConnectedSpace(IplImage* imgdst, int *tagSafe) {
    int maxleft = 0, maxright = 0, tmpleft = 0, tmpright = 0;
    int maxfree = 0;
    int allfree = 0;
    int count = 0;
    for (int i = 1; i < WIDTH; i++) {
        if(tagSafe[i] != tagSafe[i-1]) {
            if(tagSafe[i] == 0) {
                tmpleft = i;
            } else {
                tmpright = i - 1;
                if (maxfree < (tmpright - tmpleft)) {
                    maxleft = tmpleft;
                    maxright = tmpright;
                    maxfree = tmpright - tmpleft;
                }
                allfree += tmpright - tmpleft;
                count ++;
            }
        }
    }
    int left = WIDTH * EDGE_CROSS, right = WIDTH * (1 - EDGE_CROSS);
    float turn = 0;
    if (count > 0 && maxfree > 3*(allfree/count)) {
        drawSquare(imgdst, maxleft, maxright);
        if (maxright < left) {
            turn = -8*INT_FLOAT;
        }
        else if (maxleft > right) {
            turn = 8*INT_FLOAT;
        }
        else {
            turn = 0;
        }
    }

    return turn;
}

void tagConnectionInterPro(int *tagSafe, int left, int midleft, int midright, int right){
	int leftspace = midleft - left;
	int midspace = midright -  midleft;
	int rightspace  =  right - midright;
	if (tagSafe[left] == 0) //左右两边为free，中间为obstacle，则为obstacle插值free
	{
		if (leftspace > OBSTACLETOFREE * midspace && rightspace > OBSTACLETOFREE * midspace)
		{
			for (int i = midleft; i < midright; i++)
			{
				tagSafe[i] = tagSafe[left];
			}
		}
	}else{
		if (leftspace + rightspace > FREETOOBSTACLE * midspace)
		{
			for (int i = midleft; i < midright; i++)
			{
				tagSafe[i] = tagSafe[left];
			}
		}
	}
}

void tagConnectionInter(int *tagSafe, int left, int midleft, int midright, int right){
	if (right - left > TAGCONNECTION_K*(midright - midleft))
	{
		for (int i = midleft; i < midright; i++)
		{
			tagSafe[i] = tagSafe[left];
		}
	}
}

/***************************************************************************************/
/*  判定标记是否准确。安全判定安全1，安全非安全2，非安全判定安全3，非安全判定非安全4    
  tagOrigin: 准确的tag标记； tagSafe：根据算法估计的tag标记；tags：根据上述原则判定结果
  函数返回：根据1-4结果，返回此光流方法避撞的评价指数,值越大性能越差                   */
/***************************************************************************************/
float compareTag(int *tagOrigin, int *tagSafe, int *tags){
	int tag_1 = 0, tag_2 = 0, tag_3 = 0, tag_4 = 0;
	for (int i = 0; i < WIDTH; i++)
	{
		if (tagOrigin[i] == 1 && tagSafe[i] == 1)
		{
			tags[i] = 1;
			tag_1++;
		}else if (tagOrigin[i] == 1 && tagSafe[i] == 0)
		{
			tags[i] = 2;
			tag_2++;
		}else if (tagOrigin[i] == 0 && tagSafe[i] == 1)
		{
			tags[i] = 3;
			tag_3++;
		} 
		else if (tagOrigin[0] == 0 && tagSafe[i] == 0)
		{
			tags[i] = 4;
			tag_4++;
		}
	}
	char buffer[50];
    sprintf(buffer, "%d %d  %d  %d \n", tag_1, tag_2, tag_3, tag_4);
	writeFile(buffer);
	return tag_1*TAG_1_K + tag_2*TAG_2_K + tag_3*TAG_3_K + tag_4*TAG_4_K;//各比例项暂定1 -1.5 -2 1；
}

float balanceControlLR(bool isBig, int leftSumFlow, int rightSumFlow, float k){
    if(isBig){ //前方遇到同一色巨型障碍物，返回-2
        return -2*INT_FLOAT;
    }
    if(leftSumFlow == 0 || rightSumFlow == 0){
        return turnLRScale(leftSumFlow, rightSumFlow, k);
    }
    float gain = (rightSumFlow*INT_FLOAT)/(leftSumFlow*1.0);
    //return gain;
    if(gain < INT_FLOAT*k && gain > INT_FLOAT/k){
        return 0;
    }else{
        return turnLRScale(leftSumFlow, rightSumFlow, k);
    }
}


float balanceControlLR(int isBigLeft, int isBigRight, int leftSumFlow, int rightSumFlow, float k){

    float gain = (rightSumFlow*INT_FLOAT)/(leftSumFlow*1.0);

    if(isBigLeft == 1 && isBigRight == 1){ //left and right is all big obstacle
        if(gain < INT_FLOAT*k && gain > INT_FLOAT/k){
            return 4*INT_FLOAT;
        }else{
            if(leftSumFlow > rightSumFlow){
                return 4*INT_FLOAT;
            }else {
                return -4*INT_FLOAT;
            }
        }
    }

    if(isBigLeft == 1){
       return 2*INT_FLOAT;
    }

    if(isBigRight == 1){
        return -2*INT_FLOAT;
    }

    if (isBigLeft == 2 && isBigRight == 2) {
        return 5; // sky
    }

    if(leftSumFlow == 0 || rightSumFlow == 0){
        return turnLRScale(leftSumFlow, rightSumFlow, k);
    }

    if(gain < INT_FLOAT*k && gain > INT_FLOAT/k){
        return 0;
    }else{
        return turnLRScale(leftSumFlow, rightSumFlow, k);
    }
}

float turnLRScale(float leftSumFlow, float rightSumFlow, float k){
    if(leftSumFlow == rightSumFlow){
        return 0;
    }else{
        if(K_FLAG){
            k = 1;
        }
        if(leftSumFlow > rightSumFlow){
            return ((leftSumFlow - k*rightSumFlow)*INT_FLOAT)/(leftSumFlow + rightSumFlow);
        }else{
            return ((k*leftSumFlow - rightSumFlow)*INT_FLOAT)/(leftSumFlow + rightSumFlow);
        }
    }
}

Mat calibrate(Mat img)
{
	static double mtx[3][3]={562.89836209,0,314.41994795,0 ,552.27825968,160.37443571,0,0,1};
	static Mat matrix(Size(3,3),CV_64F,mtx);
	static Vec<float,5> dist(-5.25438307e-01,3.76324874e-01,-4.78114662e-04,3.51717002e-04,-1.37709825e-01); 
	static Mat newcameramtx = getOptimalNewCameraMatrix(matrix , dist ,Size(img.rows,img.cols), 0,Size(img.rows,img.cols));
	Mat img2;
	undistort(img,img2,matrix,dist,newcameramtx);
    //matrix.release();
    //img2.release();
    //newcameramtx.release();
	return img2;
}

void calibrate(IplImage* &iplimg)
{
	Mat img(iplimg,false);
	Mat img2 = calibrate(img);
	//iplimg = (IplImage *) cvClone(&IplImage(img2));
	IplImage img3 = IplImage(img2);
	iplimg = (IplImage *)cvClone(&img3);
    img.release();
    img2.release();
}

void writeFile(const char* lineStr){
    FILE *fp = fopen("/home/sarah/catkin_ws/src/avoid_optflow/video/result.txt", "a+");
	if (fp == 0)
	{
		printf("can't open file\n");
		return;
	}
	fseek(fp, 0, SEEK_END);
	fwrite(lineStr, strlen(lineStr), 1, fp);
	fclose(fp);
}

void drawOrientation(Vec2i leftSumFlow, Vec2i rightSumFlow, int px, int py, float result, IplImage* imgdst){
	Vec2i diffFlow = Vec2i(leftSumFlow[0] - rightSumFlow[0], leftSumFlow[1] - rightSumFlow[1]);
	//printf("diffFlow: %d , %d \n", diffFlow[0], diffFlow[1]);
	//printf("left :%d  right:%d\n", leftSumFlow[0], rightSumFlow[0]);
	drawArrow(cvPoint(px, py), cvPoint(px+diffFlow[0], py), imgdst,  CV_RGB (0, 255, 0) , 3);
	//cvLine(imgdst, cvPoint(px, py), cvPoint(px+diffFlow[0], py), CV_RGB (0, 255, 0), 3, CV_AA, 0);

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1, 1, 0, 2);

	int resulttxt = 0;
    char c[100];
    if (result == 2*INT_FLOAT)
	{
        resulttxt = 2;
        sprintf(c, "%d  RS", img_curr_num);
    }else if (result == -2*INT_FLOAT)
    {
        resulttxt = -2;
        sprintf(c, "%d  LS", img_curr_num);
        cvPutText(imgdst, "LS", cvPoint(20, 20), &font, CV_RGB(255, 0, 0));
    }else if (result == 4*INT_FLOAT)
    {
        resulttxt = 4;
        sprintf(c, "%d  RRS", img_curr_num);
    }else if (result == -4*INT_FLOAT)
    {
        resulttxt = -4;
        sprintf(c, "%d  LLS", img_curr_num);
    }else if (result == 8*INT_FLOAT)
    {
        resulttxt = 8;
        sprintf(c, "%d  R*", img_curr_num);
    }else if (result == -8*INT_FLOAT)
    {
        resulttxt = -8;
        sprintf(c, "%d  L*", img_curr_num);
    }else if (result == 16*INT_FLOAT)
    {
        resulttxt = 16;
        sprintf(c, "%d  MR", img_curr_num);
    }else if (result == -16*INT_FLOAT)
    {
        resulttxt = -16;
        sprintf(c, "%d  ML", img_curr_num);
    }else if (result == 32*INT_FLOAT)
    {
        resulttxt = 32;
        sprintf(c, "%d  MH", img_curr_num);
    }else if (result == -32*INT_FLOAT)
    {
        resulttxt = -32;
        sprintf(c, "%d  MH", img_curr_num);
    }else if (result  == 0)
	{
		resulttxt = 0;
        sprintf(c, "%d  F", img_curr_num);
	}else if (result < 0)
	{
		resulttxt = -1;
        sprintf(c, "%d  L", img_curr_num);
	}else if (result > 0)
	{
		resulttxt = 1;
        sprintf(c, "%d  R", img_curr_num);
	}

    cvPutText(imgdst, c, cvPoint(20, 20), &font, CV_RGB(255, 0, 0));

	//char c[50];
	//sprintf(c, "%f", result/100.0);
	//cvPutText(imgdst, c, cvPoint(10, 50), &font, CV_RGB(255, 0, 0));

	if (IS_BALANCE_RESULT_WRITE_FILE)
	{
		char buffer[50];
		sprintf(buffer, "%d\n", resulttxt);
		writeFile(buffer);
	}
}

CvSeq *getImageContours(IplImage *src)  
{  
	cvThreshold(src, src, 100, 255, CV_THRESH_BINARY);  
	CvMemStorage * storage = cvCreateMemStorage(0);  
	CvSeq * contours;  
	cvFindContours(src, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL); //如果有轮廓，则返回第一个轮廓指针，如果没有轮廓，则返回null 
	return contours;  
} 

double huSimilarity(IplImage* src, IplImage* gray){
	CvSeq* contourssrc = getImageContours(src);  //得到src的轮廓
	CvSeq* contoursgray = getImageContours(gray);  //得到gray的轮廓
	if (contoursgray == NULL)
	{
		return MAX_CONTOUR;
	}else{

	}
	return cvMatchShapes(contourssrc, contoursgray, 1);   // 根据输入的图像或轮廓来计算它们的hu矩的相似度 
}

// 内轮廓填充   
// 参数:   
// 1. pBinary: 输入二值图像，单通道，位深IPL_DEPTH_8U。  
// 2. dAreaThre: 面积阈值，当内轮廓面积小于等于dAreaThre时，进行填充。   
void FillInternalContours(IplImage *pBinary, double dAreaThre)   
{   
	double dConArea;   
	CvSeq *pContour = NULL;   
	CvSeq *pConInner = NULL;   
	CvMemStorage *pStorage = NULL;   
	// 执行条件   
	if (pBinary)   
	{   
		// 查找所有轮廓   
		pStorage = cvCreateMemStorage(0);   
		cvFindContours(pBinary, pStorage, &pContour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);   
		// 填充所有轮廓   
		cvDrawContours(pBinary, pContour, CV_RGB(255, 255, 255), CV_RGB(255, 255, 255), 2, CV_FILLED, 8, cvPoint(0, 0));  
		// 外轮廓循环   
		int wai = 0;  
		int nei = 0;  
		for (; pContour != NULL; pContour = pContour->h_next)   
		{   
			wai++;  
			// 内轮廓循环   
			for (pConInner = pContour->v_next; pConInner != NULL; pConInner = pConInner->h_next)   
			{   
				nei++;  
				// 内轮廓面积   
				dConArea = fabs(cvContourArea(pConInner, CV_WHOLE_SEQ));  
				//printf("%f\n", dConArea);  
				if (dConArea <= dAreaThre)   
				{   
					cvDrawContours(pBinary, pConInner, CV_RGB(255, 255, 255), CV_RGB(255, 255, 255), 0, CV_FILLED, 8, cvPoint(0, 0));  
				}   
			}   
		}   
		//printf("wai = %d, nei = %d\n", wai, nei);  
		cvReleaseMemStorage(&pStorage);   
		pStorage = NULL;   
	}   
}  

int Otsu(IplImage* src)      
{      
	int height=src->height;      
	int width=src->width;          

	//histogram      
	float histogram[256] = {0};      
	for(int i=0; i < height; i++)    
	{      
		unsigned char* p=(unsigned char*)src->imageData + src->widthStep * i;      
		for(int j = 0; j < width; j++)     
		{      
			histogram[*p++]++;      
		}      
	}      
	//normalize histogram      
	int size = height * width;      
	for(int i = 0; i < 256; i++)    
	{      
		histogram[i] = histogram[i] / size;      
	}      

	//average pixel value      
	float avgValue=0;      
	for(int i=0; i < 256; i++)    
	{      
		avgValue += i * histogram[i];  //整幅图像的平均灰度    
	}       

	int threshold;        
	float maxVariance=0;      
	float w = 0, u = 0;      
	for(int i = 0; i < 256; i++)     
	{      
		w += histogram[i];  //假设当前灰度i为阈值, 0~i 灰度的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例    
		u += i * histogram[i];  // 灰度i 之前的像素(0~i)的平均灰度值： 前景像素的平均灰度值    

		float t = avgValue * w - u;      
		float variance = t * t / (w * (1 - w) );      
		if(variance > maxVariance)     
		{      
			maxVariance = variance;      
			threshold = i;      
		}      
	}      

	return threshold;      
}   

void printfPointColor(Mat &frame, int x, int y){
	CvPoint p;
	p.x = x;
	p.y = y;
	int r = (int)frame.row(x).col(y).data[0];
	int g = (int)frame.row(x).col(y).data[1];
	int b = (int)frame.row(x).col(y).data[2];	
	circle(frame, p, 2, CV_RGB(255,255,0));
	printf(" r = %d, g = %d, b = %d\n", r, g, b);
}

//通过rgb值得到hue的上界和下界
void pixel2HueLowUp(Mat &frame, int x, int y){
	Mat frame_hsv;
	int low, up;
	cvtColor(frame, frame_hsv, CV_BGR2HSV);
    int hue = (int)frame_hsv.row(y).col(x).data[0];
    if ((hue > HUESCALE) && (hue < 255-HUESCALE))
	{
        low = hue - HUESCALE;
        up = hue + HUESCALE;
    }else if (hue <= HUESCALE)
	{
		low = 0;
        up = hue + HUESCALE;
    }else if (hue >= 255-HUESCALE)
	{
        low = hue - HUESCALE;
		up = 255;
	}
	printf("low = %d, up = %d", low, up);
}

// 将str字符以spl分割,存于dst中，并返回子字符串数量
int splitString(char dst[][5], char* str, const char* spl)
{
	int n = 0;
	char *result = NULL;
	result = strtok(str, spl);
	while( result != NULL )
	{
		strcpy(dst[n++], result);
		result = strtok(NULL, spl);
	}
	return n;
}

void readSpeacialLine(char* line, int pos){
	FILE* fp;
	int currentIndex = 0;
    if ((fp = fopen("/home/sarah/catkin_ws/src/avoid_optflow/video/tag.txt", "r")) == NULL)
	{
		printf("can't open file\n");
		return;
	}
	while (fgets(line, 1024, fp) != NULL)
	{
		int len = strlen(line);
		line[len - 1] = '\0';
		if (currentIndex == pos)
		{
			fclose(fp);
			return;
		}
		currentIndex++;
	}
	fclose(fp);
	return;
}

void getTagOriginFromFile( int* tagOrigin){ //为了方便，直接从tag.txt中读取
	static int pos = 0;
	char line[1024];
	readSpeacialLine(line, pos);
	char dst[50][5];
	int count = splitString(dst, line, " ");
	for (int i = 0; i < count; i+=3)
	{
		int tag = atoi(dst[i]);
		int left = atoi(dst[i+1]);
		int right = atoi(dst[i+2]);
		for (int i = left; i <= right; i++)
		{
			tagOrigin[i] = tag;
		}
	}
	pos++;
}

float resultToAccuracy(bool isleft){ //result大于0是向右，小于0是向左偏,如果标准给的是left，则isleft传入true
	FILE* fp;
    if ((fp = fopen("/home/sarah/catkin_ws/src/avoid_optflow/video/result.txt", "r")) == NULL)
	{
		printf("can't open file\n");
		return -1;
	}
	int count = 0;
	int right = 0;
	int continuous = 0;
	char line[1024];
	while (fgets(line, 1024, fp) != NULL)
	{
		int len = strlen(line);
		line[len - 1] = '\0';
		int result = atoi(line);
		if (continuous == ACCURACYGRAP)
		{
			if (isRight(result, isleft))
			{
				right ++;
			}
			count ++;
		}else{
			if (isRight(result, isleft))
			{
				continuous ++;
			}else{
				continuous = 0;
			}
		}
	}
	fclose(fp);
    float ac = (right*1.0)/count;
    printf("accuracy : %f\n", ac);
    return ac;
}

bool isRight(int result, bool isleft){
	if ((result > 0 && !isleft) || ( result < 0 && isleft))
	{
		return true;
	}else{
		return false;
	}
}

void setTagToTxt(int* tags, int length, int framecount){ 
    FILE *fp = fopen("/home/sarah/catkin_ws/src/avoid_optflow/video/tag.txt", "a+");
	if (fp == 0)
	{
		printf("can't open file\n");
		return;
	}
	for (int i = 0; i < length; i++)
	{
		for (int j = tags[3*i]; j < tags[3*i+1]; j++)
		{
			fprintf(fp, "%d", tags[3*i+2]);
		}
	}
	fclose(fp);
}

float compareTags(int* tags, int length){
	int tagReal[FRAMECOUNT], k = 0;
	//int tagStd[FRAMECOUNT];
	/*for (int i = 0; i < length; i++)
	{
		for (int j = tags[3*i]; j < tags[3*i+1]; j++)
		{
			tagStd[k] = tags[3*i+2];
			k++;
		}
	}
	k=0;*/
	
    FILE *fp = fopen("/home/sarah/catkin_ws/src/avoid_optflow/video/result.txt", "a+"); //实际结果文件
	if (fp == 0)
	{
		printf("can't open file\n");
		return -1;
	}
	int count = 0;
	int right = 0;
	char line[1024];
	while (fgets(line, 1024, fp) != NULL)
	{
		int len = strlen(line);
		line[len - 1] = '\0';
		int result = atoi(line);
		if (result == 0)
		{
			tagReal[k] = 0;
		}else if (result < 0)
		{
			tagReal[k] = -1;
		}else{
			tagReal[k] = 1;
		}
	}
	fclose(fp);

	for (int i = 0; i < length; i++)
	{
		int continuous = 0;
		for (int j = tags[3*i]; j < tags[3*i+1]; j++)
		{
			if (continuous == ACCURACYGRAP)
			{
				printf("pos : %d", j);
				continuous = ACCURACYGRAP+1;
				count += tags[3*i+1] - j;
			}
			if (continuous < ACCURACYGRAP)
			{
				if (tagReal[j] == tags[3*i+2])
				{
					continuous ++;
				}else{
					continuous = 0;
				}
			}else{
				if (tagReal[j] == tags[3*i+2])
				{
					right++;
				}
			}
		}
	}
	return right*1.0/count;
}

//0 - stable, 1 - increase, 2 - decrease, 3 - mess
int getArrayState (int* array, int currindex) {
    int countstable = 0;
    int countincr = 0;
    int countdecr = 0;
    for (int i = currindex % ARRAYSTATELENGTH; i < currindex - 1; i++ ) {
        if (array[i % ARRAYSTATELENGTH] == array[(i+1) % ARRAYSTATELENGTH]) {
            countstable ++;
        } else if (array[i % ARRAYSTATELENGTH] < array[(i+1) % ARRAYSTATELENGTH]) {
            countincr ++;
        } else {
            countdecr ++;
        }
    }
    if (countstable >= ARRAYSTATELENGTH - 1) {
        return 0;
    } else if (countstable + countincr >= ARRAYSTATELENGTH - 1) {
        return 1;
    } else if (countstable + countdecr >= ARRAYSTATELENGTH - 1) {
        return 2;
    } else {
        return 3;
    }
}
