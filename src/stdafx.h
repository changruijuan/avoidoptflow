/*
*  FILE stdafx.h
*  AUTHOR Sarah
*  DATE 2015/08/14 22:27
*  TODO: 定义常量
*/
#pragma once

#include <stdio.h>

#define ISSF false //是否是SF方法，因为SF方法处理是彩色图，所以resize时不同
#define ROWS 350  // height, 在计算ttc过程中为了不越界，申请范围大于实际图像大小
#define COLS 350  // width, 在计算ttc过程中为了不越界，申请范围大于实际图像大小
//#define HEIGHT 190  //matlab
//#define WIDTH 288   //matlab
#define HEIGHT  180   //ARDrone 180
#define WIDTH 320   //ARDrone 320
#define MAX_CORNERS  200
#define WINSIZE 5
#define DRAWGAP 15
#define UNKNOWN_FLOW_THRESH 1e10
#define LK_K  4 //6
#define HS_K  3 //4.3
#define BM_K  3 //1.4
#define FB_K  2 //2
#define SF_K  2  //未测试
#define PYRLK_K 4
#define INT_FLOAT 100.0 //matlab中int向上float转型有问题，所以结果乘100，处理时再除100.
#define K_FLAG true  //在避撞偏移量中，false表示k取1，而非上述*_K变量
#define EDGE 0.1439 //0.1429  计算边界
#define EDGE_OBS 0.02 // 计算统一颜色巨型障碍物颜色时的边界0.33
#define EDGE_CROSS 0.35 // 当free区域在此区域外面时，直接平移
#define COLOR_SCALE 20  //认为是同一颜色的范围
#define THRESHOLD_TIMER 0.75 //同色所占比例大于整幅图像的75%，则认为是墙，停止。
#define THRESHOLD_ZERO 0.80 //同一颜色中光流为0的所占比例，此处matlab中同一颜色处光流为0，而gazebo中同一颜色处光流非0
#define FLOW_ZERO 0  //光流小于等于此值认为光流为0
#define FB_SCALE 10e8  //fb计算出的光流比较大，缩小的倍数
#define IS_FLOW_WRITE_FILE false   //是否将左右光流数据写入文件
#define IS_BALANCE_RESULT_WRITE_FILE false //balance result是否写入文件
#define IS_TIMER_WRITE_FILE false //是否将光流计算时间写入文件
#define IS_CALIBRATE false//是否进行摄像头标定
//#define TACHOGRAPH_UPX 0.25  //地面
//#define TACHOGRAPH_UPY 0.6
//#define TACHOGRAPH_DOWNX 0.75
//#define TACHOGRAPH_DOWNY 0.9
#define TACHOGRAPH_UPX 0
#define TACHOGRAPH_UPY 0
#define TACHOGRAPH_DOWNX 1
#define TACHOGRAPH_DOWNY 1
#define TACHOGRAPH_ANGLE 40
#define TACHOGRAPH_FOCAL 68 //单位190像素，注：1080p焦距为1141像素
#define TACHOGRAPH_HEIGHT 1.2 // 单位米
#define IS_ROS true //matlab和ros仿真环境中大型障碍物判断不一样，因为matlab中大型障碍物无纹理；光流为0，ROS中反之，所以不同。
#define MAX_CONTOUR 100  //huSimilarity函数hu阵相似度，如果相似则为0，越不相似越大，这里为了防止光流没有轮廓造成空指针，则hu阵直接返回100作为标记。
#define IS_SIMILARITY false  //如果画轮廓，默认求相似度且写入文件result
#define IS_INTERPOLATION true //motiontogray函数时是否插值，即其上下左右INTERPOLATION_GAP范围内，有两个白点，则其插值为白点
#define IS_FILLINTERNALCONTOURS false //是否填充内轮廓
#define INTERPOLATION_GAP 5 //按照半径为INTERPOLATION_GAP的窗口插白点
#define DRAWCONTOUR true //画轮廓
#define IS_TAGINTER true  //在tag标记中，是否进行插值
#define TAGINTER_GAP 5  //tag插值中，正负TAGINTER_GAP内tag相同，则其变为此tag
#define TAGFORSAFE_K 1.1 //标记tag为安全的阈值，即 ttcAvg*TAGFORSAFE_K 
#define ISTAGCONINTERPRO true // 插值使用简单的（false）插值方法还是使用改进的（true）方法
#define TAGCONNECTION_K 4 //如果此区域的前后两个区域相加是此区域的（TAGCONNECTION_K-1）倍，则此区域根据左右进行插值,此值必须大于3
#define FREETOOBSTACLE 4 //free空间插值为obstacle，因为宁可把free空间认为为obstacle空间，则此条件弱化，即左右相加大于此区域的FREETOOBSTACLE倍。此区域根据左右进行插值
#define OBSTACLETOFREE 2 //obstacle插值free，条件苛刻，把障碍物认为是free的会犯比较严重的错误。此为左右free空间必须为obstacle的OBSTACLETOFREE倍才可
#define TAG_1_K 1   //非安全判定非安全1
#define TAG_2_K -1.5  //非安全判定安全2
#define TAG_3_K -2  //安全非安全3
#define TAG_4_K 1  //安全判定安全4
#define IS_FOECORRECT true //是否对foe进行中点修正
#define FOE_GAP 20 //foe修正范围：foe在重点正负TTC_GAP范围内，认为得到的是合理的，反之，则直接用中点
#define IS_XANDY true // false：只求x方向的光流；true：求x和y方向光流标量
#define IS_DRAWTAGLINE false //是否在目标图图上画四种tags的分布；
#define IS_DRAWTAGSQUARE true //是否在目标图上将tag标记相同的画成一个长方形，与IS_DRAWTAGLINE不能同时为true
//#define RGBOBSTACLE CV_RGB(255, 208, 180) //障碍物的RGB值
#define HUELOW 123  //障碍物hue的low值
#define HUEUP  167//障碍物hue的up值
#define HUESCALE 20 //障碍物认为是同一色调的范围
#define HSVLOW 70
#define HSVUP 220
#define DRAWTAGORIGINSPACE false //在prev图像上画tagOrigin（原图像的）free区间
#define DRAWTAGSAFESPACE true //在dst图像上画由ttc或者flow的tagSafe的free区间
#define KERNEL_LENGTH 5  //模糊化窗口大
#define ISFILETAGORIGIN false //是否从文件中获取tagOrigin
#define ACCURACYGRAP 5 //如果连续大于5帧是正确的，则开始计数，计算正确率
#define FRAMECOUNT 100 //视频帧数目
#define LEARNING_VIEW 0.25 // uav将要经过的图像区域
#define ISALLPYRLK true //pyrLK方法，如果整幅图像求特征点，则为true，如果以图像中线分割左右分别求，则为false
#define IS_WRITE_LEANING false //是否将learning结果写入result.txt文件
#define IS_WRITE_BIG_OBS true // write big obstacle result to result.txt
#define TURNTIME 75 //turn 75 times is same to turn 90 degree
#define TIMEWINDOW 5 // time windows, every timewindow, count result
#define COUNTTIME 3
#define COUNTSTOP 2
#define COUNTLINER 1

//need to false
#define ISSKIP true

//   <<<<<<<<<<<<<<<<< heyu >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define ARDRONE_SPEED 0.5  // 0.1手动控制ARDrone的速度 **********************
#define ROTATION_SPEED 0.5  // 0.2 手动控制ARDrone的旋转速度???????????????????/  *************************
#define MAX_OPTFLOW  20 // 最大的光流值    20??????  *****************************
#define MEAN_OPTFLOW_THRESHOLD  7 //  平均光流值的阕值(shangxian)   20  ***********************
#define EDGE_CEILING 0.2 // 1/5,2/7???? ****************
#define EDGE_FLOOR 0.92 // 0.86  ??? ****************
#define EDGE_FARLEFT 0.18 // 0.18*******************
#define EDGE_LEFT  0.4 //  0.36*************
#define EDGE_RIGHT 0.6  // 0.64
#define EDGE_FARRIGHT 0.82 // 0.82
#define ADD_OPTFLOW_Y false
#define ADD_ROTATION false
#define EDGE_ROTATION_LEFT  0.45
#define EDGE_ROTATION_RIGHT 0.55
#define FEATERE_LEAST 10 // *********************
#define BALANCE_LEVEL 0.1 // 光流平衡****************
#define BUFFER 200 //
#define OPT_FRONT_FLOOR  0.5 // 正向光流和地面光流的比例*******************
#define ROTATION_LEVEL1 0.1 // 控制转向的调节因子*****************************
#define ROTATION_LEVEL2 0.5 // *******
#define ROTATION_LEVEL3 1 // ********
#define ROTATION_MAX_SPEED 1 // ************
#define TRANSLATE_MAX_SPEED 1 // ************
#define TRANSLATE_MIN_SPEED 0.01 // **********
#define TRANSLATE_LEVEL 0.3 //
#define OPT_SUDDEN 100  // *************
#define LOSTRATE 0.5 // 特征点缺失率 ******************



