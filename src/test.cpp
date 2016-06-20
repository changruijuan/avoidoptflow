#include "stdafx.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv.h>
#include <highgui.h>
#include "opencv2/legacy/legacy.hpp"
#include <math.h>

#include <cstdio>
#include <iostream>

#include "optutil.h"
#include "opticalflow.h"
#include "navigation.h"
#include "video.h"
#include "test.h"

void dealMatVideo(String path){
	CvCapture *input_video = cvCreateFileCapture(path.c_str());
	if (input_video == NULL)
	{
		printf("Error: Can't open video.\n");
		exit(-1);
	}
	IplImage* img = NULL;
	cvNamedWindow ("Image", 1);
	while (true)
	{
		img = cvQueryFrame( input_video );
		Mat mat(img, false);
		if (img == NULL)
		{
			printf("video over.\n");
			exit(-1);
		}
		/*cvLine(img, cvPoint(80, 108), cvPoint(240, 108), CV_RGB (0, 255, 0), 3, CV_AA, 0);
		cvLine(img, cvPoint(80, 108), cvPoint(80, 162), CV_RGB (0, 255, 0), 3, CV_AA, 0);
		cvLine(img, cvPoint(80, 162), cvPoint(240,162), CV_RGB (0, 255, 0), 3, CV_AA, 0);
		cvLine(img, cvPoint(240, 108), cvPoint(240, 162), CV_RGB (0, 255, 0), 3, CV_AA, 0);*/
		/*line(mat, cvPoint(80, 108), cvPoint(240, 108), CV_RGB (0, 255, 0), 3);
		line(mat, cvPoint(80, 108), cvPoint(80, 162), CV_RGB (0, 255, 0), 3);
		line(mat, cvPoint(80, 162), cvPoint(240,162), CV_RGB (0, 255, 0), 3);
		line(mat, cvPoint(240, 108), cvPoint(240, 162), CV_RGB (0, 255, 0), 3);*/



		cvShowImage ("Image", img);
		int key_pressed = waitKey(20);
		if ( (char)key_pressed ==  27 )
		{
			break;
		}
	}
	cvDestroyAllWindows();
	cvReleaseCapture(&input_video);
	cvReleaseImage(&img);
}

void dealImgVideo(String path){
	CvCapture *input_video = cvCreateFileCapture(path.c_str());
	if (input_video == NULL)
	{
		printf("Error: Can't open video.\n");
		exit(-1);
	}
	IplImage *img_1 = NULL;
	CvVideoWriter* writer = cvCreateVideoWriter("../video/flow_img.avi", CV_FOURCC('P', 'I', 'M', '1'),20,cvSize(WIDTH, HEIGHT),1);

	while(true)
	{
		img_1 = cvQueryFrame( input_video );
		if (img_1 == NULL)
		{
			printf("video over.\n");
			break;
		}
		IplImage* img = imgResize(img_1);

		CvMemStorage *storage = cvCreateMemStorage(0);    
		CvSeq *first_contour = NULL;    
		IplImage *dst = cvCreateImage(cvGetSize(img), 8, 3);    
		IplImage *dsw = cvCreateImage(cvGetSize(img), 8, 1); 

		//turn the src image to a binary image    
		//cvThreshold(src, dsw, 125, 255, CV_THRESH_BINARY_INV);    
		cvThreshold(img, dsw, 100, 255, CV_THRESH_BINARY);    

		cvFindContours(dsw, storage, &first_contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);    
		cvZero(dst);    
		int cnt = 0;    
		for(; first_contour != 0; first_contour = first_contour->h_next)    
		{    
			cnt++;    
			CvScalar color = CV_RGB(rand()&255, rand()&255, rand()&255);    
			cvDrawContours(dst, first_contour, color, color, 0, 2, CV_FILLED, cvPoint(0, 0));    
			CvRect rect = cvBoundingRect(first_contour,0);  
			cvRectangle(dst, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255, 0, 0), 1, 8, 0);  
		}    

		cvNamedWindow ("flow", 1);
		cvShowImage ("flow", img);

		cvNamedWindow("dst", 1);
		cvShowImage("dst",dst);

		cvWriteFrame(writer, dst);
		int key_pressed = waitKey(1);
		if ( (char)key_pressed ==  27 )
		{
			break;
		}
		cvReleaseMemStorage(&storage);
		cvReleaseImage(&dst);
		cvReleaseImage(&dsw);
		cvReleaseImage(&img);
	}
	cvDestroyAllWindows();
	
	cvReleaseImage(&img_1);
	cvReleaseCapture(&input_video);
	cvReleaseVideoWriter(&writer);
}

void getHuImgVideo(String path1, String path2){
	CvCapture *input_video1 = cvCreateFileCapture(path1.c_str());
	CvCapture *input_video2 = cvCreateFileCapture(path2.c_str());
	if (input_video1 == NULL)
	{
		printf("Error: Can't open video.\n");
		exit(-1);
	}
	if (input_video2 == NULL)
	{
		printf("Error: Can't open video.\n");
		exit(-1);
	}
	IplImage *img_1 = NULL,*img_2 = NULL;
	CvVideoWriter* writerA = cvCreateVideoWriter("../video/dstA.avi", CV_FOURCC('P', 'I', 'M', '1'),20,cvSize(WIDTH, HEIGHT),1);
	CvVideoWriter* writerB = cvCreateVideoWriter("../video/dstB.avi", CV_FOURCC('P', 'I', 'M', '1'),20,cvSize(WIDTH, HEIGHT),1);

	while(true)
	{
		img_1 = cvQueryFrame( input_video1 );
		if (img_1 == NULL)
		{
			printf("video over.\n");
			break;
		}
		img_2 = cvQueryFrame( input_video1 );
		if (img_2 == NULL)
		{
			printf("video over.\n");
			break;
		}
		IplImage* imgA = imgResize(img_1);
		IplImage* imgB = imgResize(img_2);

		CvSeq *contours1 = getImageContours(imgA); 
		CvSeq *contours2 = getImageContours(imgB);

		IplImage* dstA = getDstContours(imgA, contours1);
		IplImage* dstB = getDstContours(imgB, contours2);


		cvNamedWindow ("dstA", 1);
		cvShowImage ("dstA", dstA);

		cvNamedWindow("dstB", 1);
		cvShowImage("dstB",dstB);

		cvWriteFrame(writerA, dstA);
		cvWriteFrame(writerB, dstB);

		int key_pressed = waitKey(1);
		if ( (char)key_pressed ==  27 )
		{
			break;
		}
	}
	cvDestroyAllWindows();

	cvReleaseImage(&img_1);
	cvReleaseImage(&img_2);
	cvReleaseVideoWriter(&writerA);
	cvReleaseVideoWriter(&writerB);
}

IplImage* getDstContours(IplImage* img, CvSeq *first_contour){
	
	IplImage *dst = cvCreateImage(cvGetSize(img), 8, 3);    
	IplImage *dsw = cvCreateImage(cvGetSize(img), 8, 1); 
	
	CvMemStorage *storage = cvCreateMemStorage(0);

	cvThreshold(img, dsw, 100, 255, CV_THRESH_BINARY);    

	cvFindContours(dsw, storage, &first_contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);    
	cvZero(dst);    
	int cnt = 0;    
	for(; first_contour != 0; first_contour = first_contour->h_next)    
	{    
		cnt++;    
		CvScalar color = CV_RGB(rand()&255, rand()&255, rand()&255);    
		cvDrawContours(dst, first_contour, color, color, 0, 2, CV_FILLED, cvPoint(0, 0));    
		CvRect rect = cvBoundingRect(first_contour,0);  
		cvRectangle(dst, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),CV_RGB(255, 0, 0), 1, 8, 0);  
	}    
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&dsw);
	cvReleaseImage(&img);

	return dst;
}

void fillHole(const Mat srcBw, Mat &dstBw)
{
	Size m_Size = srcBw.size();
	Mat Temp=Mat::zeros(m_Size.height+2,m_Size.width+2,srcBw.type());//ÑÓÕ¹Í¼Ïñ
	srcBw.copyTo(Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)));

	cv::floodFill(Temp, Point(0, 0), Scalar(255));

	Mat cutImg;//²Ã¼ôÑÓÕ¹µÄÍ¼Ïñ
	Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)).copyTo(cutImg);

	dstBw = srcBw | (~cutImg);
}

