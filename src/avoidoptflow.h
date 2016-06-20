#ifndef DRONETEST_H_
#define DRONETEST_H_

    #include <ros/ros.h>
    #include <image_transport/image_transport.h>
    #include <gazebo_msgs/ModelState.h>
    #include <gazebo_msgs/ModelStates.h>
    #include <sensor_msgs/image_encodings.h>
    #include <cv_bridge/cv_bridge.h>
    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv/cv.h>
    #include <opencv/highgui.h>
    #include <std_msgs/String.h>
    #include <std_msgs/Empty.h>
    #include <iostream>
    #include <stdio.h>
    #include <geometry_msgs/Twist.h>
    #include <ardrone_autonomy/Navdata.h>
    #include <nav_msgs/Odometry.h>
    #include "stdafx.h"
    #include "opticalflow.h"
    #include "navigation.h"
    #include "optdrawflow.h"





#if 0
    #define SPACE 1048608
    #define ENTER 1048586
    #define UP 1113938
    #define DOWN 1113940
    #define LEFT 1113937
    #define RIGHT 1113939
    #define P 1048688 // 截图并存储图像
    #define W 1048695
    #define S 1048691
    #define A 1048673
    #define D 1048676
    #define H 1048680
    #define C 1048675
    #define N 1048686
    #define VEDIO_START 1048637  // +
    #define VEDIO_OVER 1048621 // -
#define 1 1048687

#endif

#if 1
    #define SPACE 32
    #define ENTER 10
    #define UP 65362
    #define DOWN 65364
    #define LEFT 65361
    #define RIGHT 65363
    #define O 111
    #define P 112 // 截图并存储图像
    #define W 119
    #define S 115
    #define A 97
    #define D 100
    #define H 104
    #define C 99
    #define N 110
    #define VEDIO_START 61  // +
    #define VEDIO_OVER 45 // -
#endif

#endif

class my_ardrone_node{

    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;
    image_transport::Subscriber image_sub;
    ros::Subscriber nav_sub;
    ros::Subscriber odom_sub;
    geometry_msgs::Twist pid_cmd_vel; // 这个6维速度变量为pid使用的值，每次pid修改会对其进行微调。
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist fresh_vel;
    ros::Publisher vel_pub;
    ros::Publisher model_state_pub;
    ros::Subscriber model_state_sub;
    Mat last_image;//上一张图片
    Mat current_image;//下一张图片
    bool begin_rec; // 是否进行录像
    CvVideoWriter* writer;
    CvVideoWriter* writerprev;
    CvVideoWriter* writercolor;
    CvVideoWriter* writergray;
    int x;//ardrone x
    int y;//ardrone y
    int z;//ardrone z
    double ardronePosX;
    double ardronePosY;
    int theta;//ardrone 当前飞机方向
    int curDirection; //
    int oriDirection; //
    int rotationNum; //
    bool dirIsChange; // 方向不变则需要PID调节rotation
    bool flag;
    bool textObstable;
    bool noTextObstable;

    int scene; //1-tun, 2-cross
    int strategic;
    int function; //1-lk, 2-hs, 3-bm, 4-prylk, 5-fb, 6-sf

    void takeoff();
    void land();
    void forward(float i);
    void back(float i);
    void rotation_clock(float i);
    void rotation_inclock(float i);
    void hover();
    void left(float i);
    void right(float i);
    void up(float i);
    void down(float i);
    void fly(float x, float y, float z, float rx, float ry, float rz);

    void process(const sensor_msgs::ImageConstPtr& cam_image);

    void nav_callback(ardrone_autonomy::Navdata msg);

    void odom_callback(nav_msgs::Odometry msg);

    void modelstate_callback(const gazebo_msgs::ModelStatesConstPtr &msg);

    // 根据计算得到的result进行控制飞机（左右光流平衡）
    void balanceCtrl_heyu(int result);

    void autoTunCtrl(float result);

    void autoCrossCtrl(float result);

    void linerCtrl(bool isleft);

    void bigObsCtrl_turn90();

    void bigObsCtrl(float result);

    void turnWaitTime(bool isLeft, int waittime);

    void linerWaitTime(bool isLeft, int waittime);

    void manualCtrl();

    float getNextRotation();

    void allToZero();

    void updateLastRotation(int r );

    float getSpeed();

public:
    /*
     *init the subscribers and publishers
     */
    my_ardrone_node(int function, int scene);
    ~my_ardrone_node();

};
