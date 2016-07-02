#include "avoidoptflow.h"
#include "video.h"
#include "videoutil.h"

using namespace std;
using namespace cv;

bool optCtrl = false; // 是否光流控制，true由光流控制，false由手动控制
bool optCal = true; // 是否计算光流
static const std_msgs::Empty emptymsg;
static int tag_timer = 0;
static int sim_timer = 0;
static int tag = 1;
static const int T = 50;
static int i = 0;
int img_num = 0; //picture num
int img_curr_num = 0; // picture current number
int maxpos[ARRAYSTATELENGTH];
int maxsimple[ARRAYSTATELENGTH];
int midpos[ARRAYSTATELENGTH];
int maxsum[ARRAYSTATELENGTH];
int currindex = 0;
Vec2i leftSumFlow = Vec2i(0, 0);
Vec2i rightSumFlow = Vec2i(0, 0);

bool  video_record = false;
double result1 = 0; //
double result2 = 0; //
double result3 = 0; //
double OF_front = 0;
double OF_left_right = 0;
double OF_farLeft_right = 0;
double OF_front_last = 0;
int LastRotation = 1;

// 1 - forward; 2 - rotion_clock; 3 - rotion_inclock.
int ardrone_action = 0;

int times = 0;
float lastresult = 0;
int isleft = 1;
int isLastLeft = 1;
int isLinerLeft = 1;
int isLastLinerLeft = 1;
float timer = 0;
int binary = 1;
int bigturn = 0;

int filtercount = 0;
int avgFlow = 0;

int countleft = 0;
int countright = 0;
int countstop = 0;
int countlinerleft = 0;
int countlinerright = 0;
int counthover = 0;
int countturn = 0;
bool isBigCtrl = false;
bool isLinerCtrl = false;
bool isHoverCtrl = false;
bool isObjCtrl = false;
bool isForwardCtrl = false;

void my_ardrone_node::takeoff(){
    takeoff_pub.publish(emptymsg);
}
void my_ardrone_node::land(){
    land_pub.publish(emptymsg);
}
void my_ardrone_node::forward(float i){
    cmd_vel = fresh_vel;
    cmd_vel.linear.x = i;
    vel_pub.publish(cmd_vel);
    ardrone_action = 1;
    cout << "forward" << endl;
}
void my_ardrone_node::back(float i){
    cmd_vel = fresh_vel;
    cmd_vel.linear.x = -i;
    vel_pub.publish(cmd_vel);

}
void my_ardrone_node::rotation_clock(float i){
    cmd_vel = fresh_vel;
    cmd_vel.angular.z = i;
    vel_pub.publish(cmd_vel);
    ardrone_action = 2;
    cout << "rotion_clock" << endl;
}
void my_ardrone_node::rotation_inclock(float i){
    cmd_vel = fresh_vel;
    cmd_vel.angular.z = -i;
    vel_pub.publish(cmd_vel);
    ardrone_action = 3;
    cout << "rotion_inclock" << endl;
}
void my_ardrone_node::hover(){
    cmd_vel = fresh_vel;
    vel_pub.publish(cmd_vel);
}
void my_ardrone_node::left(float i){
    cmd_vel = fresh_vel;
    cmd_vel.linear.y = i;
    vel_pub.publish(cmd_vel);
    ardrone_action = 4;
    cout << "left" << endl;
}
void my_ardrone_node::right(float i){
    cmd_vel = fresh_vel;
    cmd_vel.linear.y = -i;
    vel_pub.publish(cmd_vel);
    ardrone_action = 5;
    cout << "right" << endl;
}
void my_ardrone_node::up(float i){
    cmd_vel = fresh_vel;
    cmd_vel.linear.z = i;
    vel_pub.publish(cmd_vel);
}
void my_ardrone_node::down(float i){
    cmd_vel = fresh_vel;
    cmd_vel.linear.z = -i;
    vel_pub.publish(cmd_vel);
}
void my_ardrone_node::fly(float x, float y, float z, float rx, float ry, float rz){
    cmd_vel = fresh_vel;
    cmd_vel.linear.x = x;
    cmd_vel.linear.y = y;
    cmd_vel.linear.z = z;
    cmd_vel.angular.x = rx;
    cmd_vel.angular.y = ry;
    cmd_vel.angular.z = rz;
    vel_pub.publish(cmd_vel);
}

void my_ardrone_node::process(const sensor_msgs::ImageConstPtr& cam_image){
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(cam_image,sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception:%s",e.what());
        return;
    }

    current_image = cv_ptr->image;
    img_num ++;
    img_curr_num ++;
    img_curr_num %= 100000;

    if(IS_FLOW_WRITE_FILE){
        char buffer[BUFFER];
        sprintf(buffer, "***********img:%d****************\n", img_num);
        writeFile(buffer);
    }

    if (IS_CALIBRATE) // 是否需要对图像矫准
    {
        current_image = calibrate(current_image);
    }

    if(!last_image.empty()){
        //cout << "get picture " << img_num << endl;

        IplImage temp_last = IplImage(last_image);
        IplImage* img_1 = &temp_last;
        IplImage temp_curr = IplImage(current_image);
        IplImage* img_2 = &temp_curr;

        Mat color, gray;
        color.create(HEIGHT, WIDTH, CV_8UC3);
        gray.create(HEIGHT, WIDTH, CV_8UC1);

        IplImage *img_prev = cvCreateImage(cvSize(WIDTH, HEIGHT), img_1->depth , img_1->nChannels);
        cvResize(img_1, img_prev);

        IplImage *img_dst = cvCreateImage(cvSize(WIDTH, HEIGHT), img_2->depth , img_2->nChannels);
        cvResize(img_2, img_dst);

        IplImage* imgprev_1 = imgResize(img_1);
        IplImage* imgcurr_1 = imgResize(img_2);

        float result = 0;
        if(optCal){
            switch (function){
            case 1:
                result = imgStrategic(Lucaskanade, imgprev_1, imgcurr_1, img_prev, img_dst, color, gray, strategic)/INT_FLOAT;
                break;
            case 2:
                result = imgStrategic(HornSchunck, imgprev_1, imgcurr_1, img_prev, img_dst, color, gray, strategic)/INT_FLOAT;
                break;
            case 3:
                result = imgStrategic(BlockMatch, imgprev_1, imgcurr_1, img_prev, img_dst, color, gray, strategic)/INT_FLOAT;
                break;
            case 4:
                if (ISALLPYRLK){
                    result = imgFeatAllStrategic(PyrLK, img_1, img_2, img_dst, strategic)/INT_FLOAT;
                }else{
                    result = imgFeatureStrategic(PyrLK, img_1, img_2, img_dst, color, strategic)/INT_FLOAT;
                }
                break;
            }

            cvNamedWindow ("flow", 1);
            cvShowImage ("flow", img_dst);
            cvNamedWindow("prev", 1);
            cvShowImage("prev", img_prev);
            //imshow("color",color);
            //imshow("gray", gray);
            if(video_record){
                cvWriteFrame(writer, img_dst);
                cvWriteFrame(writerprev, img_prev);
                //IplImage tmp_color = color;
                //IplImage* img_color = cvCloneImage(&tmp_color);
                //IplImage tmp_gray = gray;
                //IplImage* img_gray = cvCloneImage(&tmp_gray);
                //cvWriteFrame(writercolor, img_color);
                //cvWriteFrame(writergray, img_gray);
                waitKey(1);
            }
        }
//        cvNamedWindow ("flow", 1);
//        cvShowImage ("flow", img_dst);
//        cvNamedWindow("prev", 1);
//        cvShowImage("prev", img_prev);
//        cvWriteFrame(writer, img_dst);
//        cvWriteFrame(writerprev, img_prev);
//        waitKey(1);


        if(optCtrl){
            // 左右光流平衡策略避撞：
            switch (scene) {
            case 1:
                autoTunCtrl(result);
                break;
            case 2:
                autoCrossCtrl(result);
                break;
            case 3:
                autoMoveCtrl(result);
                break;
            case 4:
                autoPosCrossCtrl(result);
                break;
            default:
                manualCtrl();
                break;
            }

            int k = waitKey(1);
            if(k == N){
                optCtrl = false;
                optCal=false;
                cout << "manual control" << endl;
            }
            if(k == VEDIO_START){
                video_record = true;
                cout << "video start" << endl;
            }
            if(k == VEDIO_OVER){
                video_record = false;
                cout << "video over" << endl;
            }

        }else {
            manualCtrl();
        }
    }
    last_image = current_image;
    return;
}

void my_ardrone_node::manualCtrl(){
    int k = waitKey(1);
    if(k != -1){
        cout << "key: " << k << endl;
    }

    if(k == SPACE){//space for takeoff or hover
        curDirection = theta;
        oriDirection = theta;
        dirIsChange = false;
        takeoff();
    }
    if(k == ENTER){//enter for land
        hover();
        land();
    }
    if(k == UP){//up for forward
        forward(ardrone_speed);
    }
    if(k == DOWN){//down for back
        back(ardrone_speed);
    }
    if(k == LEFT){//left for left
        left(ardrone_speed);
    }
    if(k == RIGHT){//right for right
        right(ardrone_speed);
    }
    if(k == W){//W for lift
        up(ardrone_speed);
    }
    if(k == S){//S for down
        down(ardrone_speed);
    }
    if(k == A){//A for turn left
        dirIsChange = true;
        rotation_clock(ROTATION_SPEED);
    }
    if(k == D){//D for turn right
        dirIsChange = true;
        rotation_inclock(ROTATION_SPEED);
    }
    if(k == H){ //H for hover
        hover();
    }
    if(k == O){ //按下O键切换回光流控制
        optCtrl = true;
        dirIsChange = false;
        optCal = true;
        cout << "optflow control" << endl;
    }
    if(k == C){
        optCal = true;
        cout << "optflow caculate" << endl;
    }
    if(k == N){
        optCal=false;
        cout << "optflow not caculate" << endl;
    }
    if(k == VEDIO_START){
        video_record = true;
        cout << "video start" << endl;
    }
    if(k == VEDIO_OVER){
        video_record = false;
        cout << "video over" << endl;
    }
}

void my_ardrone_node::autoTunCtrl(float result){
    //count each condition
    cout << img_num << "result " << result << " countstop " << countstop << " islastleft " << isLastLeft << " isleft " << isleft << " times " << times << endl;
    if(result == -2 || result == 2 || result == -4 || result == 4) {
        countstop++;
    }
    else if (result > 0) {
        countright++;
    }
    else if (result < 0) {
        countleft++;
    }

    if(isBigCtrl){
        turnWaitTime(isleft, times);
    }

    if(!isBigCtrl && countstop >= COUNTSTOP){
        bigObsCtrl(result);
        isBigCtrl = true;
        return;
    }

    if(!isBigCtrl && (img_num % TIMEWINDOW == 0)){ // if  equal to timewindow, turn each
        //if not big obstacle, "isLastLeft = isleft" in order to first time to big ctrl binary = 1;
        isLastLeft = isleft;
        bigturn = 0;
        binary = 1;
        if(countright >= COUNTTIME){
            dirIsChange = true;
            rotation_inclock(ROTATION_SPEED);
            countturn -= TIMEWINDOW;

        }else if(countleft >= COUNTTIME){
            dirIsChange = true;
            rotation_clock(ROTATION_SPEED);
            countturn += TIMEWINDOW;

        }else {
            forward(ardrone_speed);
        }
        allToZero();
    }
    countturn = countturn % (TURNTIME*4);

}

void my_ardrone_node::autoCrossCtrl(float result){
    if (abs(ardronePosX - OBJECTPOSITIONX) < 1 && abs(ardronePosY - OBJECTPOSITIONY) < 1) {
        hover();
        land();
    }
    //count each condition\

    if(result == -2 || result == 2 || result == -4 || result == 4) {
        countstop++;
    }
    else if (result == -8) {
        countlinerleft++;
    }
    else if (result == 8) {
        countlinerright++;
    }
    else if (result > 0) {
        countright++;
    }
    else if (result < 0) {
        countleft++;
    }

    cout << img_num << "result " << result << " countstop " << countstop << \
            " countleft " << countleft << " countright " << countright << \
            " countlinerleft " << countlinerleft << " countlinerright " << countlinerright << \
            " islastleft " << isLastLeft << " isleft " << isleft << \
            " islastlinerleft " << isLastLinerLeft << " islinerleft " << isLinerLeft << \
            " times " << times << endl;

    if(isBigCtrl){
        turnWaitTime(isleft, times);
    }

    if (isLinerCtrl) {
        linerWaitTime(isleft, times);
    }

    if(!isBigCtrl && !isLinerCtrl && countstop >= COUNTSTOP){
        bigObsCtrl(result);
        isBigCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && countlinerleft >= COUNTLINER){
        linerCtrl(true);
        isLinerCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && countlinerright >= COUNTLINER){
        linerCtrl(false);
        isLinerCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && (img_num % TIMEWINDOW == 0)){ // if  equal to timewindow, turn each
        //if not big obstacle, "isLastLeft = isleft" in order to first time to big ctrl binary = 1;
        isLastLeft = isleft;
        isLastLinerLeft = isLinerLeft;
        bigturn = 0;
        binary = 1;
        if(countright >= COUNTTIME){
            dirIsChange = true;
            rotation_inclock(ROTATION_SPEED);
            countturn -= TIMEWINDOW;
        }
        else if(countleft >= COUNTTIME){
            dirIsChange = true;
            rotation_clock(ROTATION_SPEED);
            countturn += TIMEWINDOW;
        }
        else {
            forward(ardrone_speed);
        }
        allToZero();
    }
    countturn = countturn % (TURNTIME*4);
}

void my_ardrone_node::autoPosCrossCtrl(float result) {

    if (abs(ardronePosX - OBJECTPOSITIONX) < 1 && abs(ardronePosY - OBJECTPOSITIONY) < 1) {
        hover();
        land();
    }

    if(result == -2 || result == 2 || result == -4 || result == 4) {
        countstop++;
    }
    else if (result == -8) {
        countlinerleft++;
    }
    else if (result == 8) {
        countlinerright++;
    }
    else if (result > 0) {
        countright++;
    }
    else if (result < 0) {
        countleft++;
    }

    cout << img_num << "result " << result << " countstop " << countstop << \
            " countleft " << countleft << " countright " << countright << \
            " countlinerleft " << countlinerleft << " countlinerright " << countlinerright << \
            " islastleft " << isLastLeft << " isleft " << isleft << \
            " islastlinerleft " << isLastLinerLeft << " islinerleft " << isLinerLeft << \
            " times " << times << endl;

    if(isBigCtrl){
        turnWaitTime(isleft, times);
    }

    if (isLinerCtrl) {
        linerWaitTime(isleft, times);
    }

    if (isObjCtrl) {
        if (isForwardCtrl) {
            cout << "objctrl forwardctrl " << times << endl;
            forwardWaitTime(times);
        } else {
            cout << "objctrl objturnctrl " << times << endl;
            objTurnWaitTime(isleft, times);
        }
    }


    if(!isBigCtrl && !isLinerCtrl && !isObjCtrl && countstop >= COUNTSTOP){
        bigObsCtrl(result);
        isBigCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isObjCtrl && countlinerleft >= COUNTLINER){
        linerCtrl(true);
        isLinerCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isObjCtrl && countlinerright >= COUNTLINER){
        linerCtrl(false);
        isLinerCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isObjCtrl && (img_num % TIMEWINDOW == 0)){ // if  equal to timewindow, turn each
        //if not big obstacle, "isLastLeft = isleft" in order to first time to big ctrl binary = 1;
        isLastLeft = isleft;
        isLastLinerLeft = isLinerLeft;
        bigturn = 0;
        binary = 1;
        if(countright >= COUNTTIME){
            dirIsChange = true;
            rotation_inclock(ROTATION_SPEED);
            countturn -= TIMEWINDOW;
        }
        else if(countleft >= COUNTTIME){
            dirIsChange = true;
            rotation_clock(ROTATION_SPEED);
            countturn += TIMEWINDOW;
        }
        else {
            objectCtrl();
            isObjCtrl = true;
            return;
        }
        allToZero();
    }
    countturn = countturn % (TURNTIME*4);
}

void my_ardrone_node::autoPosCtrl(float result){

    if (abs(ardronePosX - OBJECTPOSITIONX) < 1 && abs(ardronePosY - OBJECTPOSITIONY) < 1) {
        hover();
        land();
    }

    //count each condition
    if(result == -2 || result == 2 || result == -4 || result == 4) {
        countstop++;
    }
    else if (result == -32) {
        countlinerleft++;
    }
    else if (result == 32) {
        countlinerright++;
    }
    else if (result == -16 || result == 16) {
        counthover++;
    }
    else if (result > 0) {
        countright++;
    }
    else if (result < 0) {
        countleft++;
    }

    cout << img_num << "result " << result << " countstop " << countstop << \
            " countleft " << countleft << " countright " << countright << \
            " countlinerleft " << countlinerleft << " countlinerright " << countlinerright << \
            " counthover " << counthover << " islastleft " << isLastLeft << " isleft " << isleft << \
            " islastlinerleft " << isLastLinerLeft << " islinerleft " << isLinerLeft << \
            " times " << times << endl;

    if(isBigCtrl){
        cout << "bigctrl " << times << endl;
        turnWaitTime(isleft, times);
    }

    if (isLinerCtrl) {
        cout << "linerctrl " << times << endl;
        linerWaitTime(isleft, times);
    }

    if (isHoverCtrl) {
        cout << "hoverctrl " << times << endl;
        hoverWaitTime(times);
    }

    if (isObjCtrl) {
        if (isForwardCtrl) {
            cout << "objctrl forwardctrl " << times << endl;
            forwardWaitTime(times);
        } else {
            cout << "objctrl objturnctrl " << times << endl;
            objTurnWaitTime(isleft, times);
        }
    }

    if(!isBigCtrl && !isLinerCtrl && !isHoverCtrl && !isObjCtrl && countstop >= COUNTSTOP){
        bigObsCtrl(result);
        isBigCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isHoverCtrl && !isObjCtrl && countlinerleft >= COUNTLINER){
        linerCtrl(true);
        isLinerCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isHoverCtrl && !isObjCtrl && countlinerright >= COUNTLINER){
        linerCtrl(false);
        isLinerCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isHoverCtrl && !isObjCtrl && counthover >= COUNTHOVER){
        hoverCtrl();
        isHoverCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isHoverCtrl && !isObjCtrl && (img_num % TIMEWINDOW == 0)){ // if  equal to timewindow, turn each
        //if not big obstacle, "isLastLeft = isleft" in order to first time to big ctrl binary = 1;
        isLastLeft = isleft;
        isLastLinerLeft = isLinerLeft;
        bigturn = 0;
        binary = 1;
        if(countright >= COUNTTIME){
            dirIsChange = true;
            rotation_inclock(ROTATION_SPEED);
            countturn -= TIMEWINDOW;
        }
        else if(countleft >= COUNTTIME){
            dirIsChange = true;
            rotation_clock(ROTATION_SPEED);
            countturn += TIMEWINDOW;
        }
        else {
            objectCtrl();
            isObjCtrl = true;
            return;
        }
        allToZero();
    }
    countturn = countturn % (TURNTIME*4);
}


void my_ardrone_node::autoMoveCtrl(float result){
    if (abs(ardronePosX - OBJECTPOSITIONX) < 1 && abs(ardronePosY - OBJECTPOSITIONY) < 1) {
        hover();
        land();
    }
    //count each condition

    if(result == -2 || result == 2 || result == -4 || result == 4) {
        countstop++;
    }
    else if (result == -32) {
        countlinerleft++;
    }
    else if (result == 32) {
        countlinerright++;
    }
    else if (result == -16 || result == 16) {
        counthover++;
    }
    else if (result > 0) {
        countright++;
    }
    else if (result < 0) {
        countleft++;
    }

    cout << img_num << "result " << result << " countstop " << countstop << \
            " countleft " << countleft << " countright " << countright << \
            " countlinerleft " << countlinerleft << " countlinerright " << countlinerright << \
            " counthover " << counthover << " islastleft " << isLastLeft << " isleft " << isleft << \
            " islastlinerleft " << isLastLinerLeft << " islinerleft " << isLinerLeft << \
            " times " << times << endl;

    if(isBigCtrl){
        turnWaitTime(isleft, times);
    }

    if (isLinerCtrl) {
        linerWaitTime(isleft, times);
    }

    if (isHoverCtrl) {
        hoverWaitTime(times);
    }

    if(!isBigCtrl && !isLinerCtrl && !isHoverCtrl && countstop >= COUNTSTOP){
        bigObsCtrl(result);
        isBigCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isHoverCtrl && countlinerleft >= COUNTLINER){
        linerCtrl(true);
        isLinerCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isHoverCtrl && countlinerright >= COUNTLINER){
        linerCtrl(false);
        isLinerCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isHoverCtrl && counthover >= COUNTHOVER){
        hoverCtrl();
        isHoverCtrl = true;
        return;
    }

    if(!isBigCtrl && !isLinerCtrl && !isHoverCtrl && (img_num % TIMEWINDOW == 0)){ // if  equal to timewindow, turn each
        //if not big obstacle, "isLastLeft = isleft" in order to first time to big ctrl binary = 1;
        isLastLeft = isleft;
        isLastLinerLeft = isLinerLeft;
        bigturn = 0;
        binary = 1;
        if(countright >= COUNTTIME){
            dirIsChange = true;
            rotation_inclock(ROTATION_SPEED);
            countturn -= TIMEWINDOW;
        }
        else if(countleft >= COUNTTIME){
            dirIsChange = true;
            rotation_clock(ROTATION_SPEED);
            countturn += TIMEWINDOW;
        }
        else {
            forward(ardrone_speed);
        }
        allToZero();
    }
    countturn = countturn % (TURNTIME*4);
}

void my_ardrone_node::bigObsCtrl_turn90(){

    int tmpcountstop = countstop;

    if(img_num % TIMEWINDOW == 0){ // if not equal to timewindow, count each condition
        allToZero();
    }
    countturn = countturn % (TURNTIME*4);

    cout << isleft << " times: " << times << " countstop " << tmpcountstop << " img_num " << img_num << " bigturn " << bigturn << endl;

    //if a big obstacle, not use result, turn 90 degree.
    if(isleft && times > 0 && times < TURNTIME){
        times++;
        countturn ++;
        return;
    }

    //-2表示巨型障碍物,75次Ardrone转90°
    if(isleft && times == TURNTIME){
        if(tmpcountstop >= COUNTSTOP){ //still obstacle
            isleft = 0;
            times = 1;
            img_num = 0;
            dirIsChange = true;
            rotation_inclock(ROTATION_SPEED);
            countturn--;
            return;
        }else{  // not obstacle
            times = 0;
            img_num = 0;
            forward(ardrone_speed);
            return;
        }
    }

    // before this time, has not detect big obstacle
    if(isleft && times == 0){
        if(tmpcountstop >= COUNTSTOP){ //first time to detect a big obstacle
            if(countturn > TURNTIME - 10 && img_num < 50){
                isleft = 0;
                times = 1;
                img_num = 0;
                dirIsChange = true;
                rotation_inclock(ROTATION_SPEED);
                countturn--;
                return;
            }else{
                times ++;
                img_num = 0;
                dirIsChange = true;
                rotation_clock(ROTATION_SPEED);
                countturn++;
                return;
            }
        }else{
            forward(ardrone_speed);
            return;
        }
    }

    // turn right and not turn 180 degree
    if(!isleft && times < TURNTIME*2){
        times++;
        countturn--;
        return;
    }
    // turn right is 180 degree
    if(!isleft && times == TURNTIME*2){
       if(tmpcountstop >= COUNTSTOP){ // still a big obstacle
           times ++;
           img_num = 0;
           countturn--;
           return;
       }else{ //not big obstcle
           times = 0;
           isleft = 1;
           forward(ardrone_speed);
           return;
       }
    }
    // has turn right more than 180 degree
    if(!isleft && times > TURNTIME*2 && times < TURNTIME*3){
        times++;
        countturn--;
        return;
    }
    // has turn right is 270 degree and is the orgin directioin
    if(!isleft && times == TURNTIME*3){
        times = 0;
        img_num = 0;
        isleft = 1;
        forward(ardrone_speed);
        return;
    }
}

void my_ardrone_node::allToZero(){
    countstop = 0;
    countright = 0;
    countleft = 0;
    countlinerleft = 0;
    countlinerright = 0;
    counthover = 0;
    img_num = 0;
}

void my_ardrone_node::linerCtrl (bool isleft) {
    allToZero();
    isLastLinerLeft = isLinerLeft;
    isLinerLeft = isleft ? 1 : 0;
    if (scene == 4) {
        times = 15;
    } else {
        times = 3;
    }
    if(isLastLinerLeft == isLinerLeft){
        binary = 1;
    }else{
        binary *= 2;
        if(binary > 8){
            binary = 1;
        }
    }
    times *= binary;
    linerWaitTime(isleft, times);
}

void my_ardrone_node::hoverCtrl () {
    allToZero();
    times = 5;
    hoverWaitTime(times);
}

void my_ardrone_node::objectCtrl(){
    allToZero();
    // 换算为需要旋转的次数
    double needy = OBJECTPOSITIONY - ardronePosY;
    double needx = OBJECTPOSITIONX - ardronePosX;
    double angle = atan(needy/needx) * 57.3; // (180/3.14)
    int turn = (angle - theta) * 0.83; // (TURNTIME/90)
    cout << "objectCtrl turn " << turn << " angle " << angle  << " theta " << theta << " ardroneX " << ardronePosX << " ardronePosY " << ardronePosY << endl;
    times = turn % (TURNTIME*4);
    if (abs(times) < 5) {
        isForwardCtrl = true;
        times = 5;
        forwardWaitTime(times);
    } else {
        if (times > 0) {
            times = abs(times);
            isleft = true;
            objTurnWaitTime(isleft, times);
        } else if(times < 0) {
            times = abs(times);
            isleft = false;
            objTurnWaitTime(isleft, times);
        }
    }
}

void my_ardrone_node::bigObsCtrl (float result) {
    allToZero();
    isLastLeft = isleft;
    if(result == -4) {
        times = 15;
        isleft = 1;
    }else if(result == 4) {
        times = 15;
        isleft = 0;
    }else if(result == -2) {
        times = 5;
        isleft = 1;
    }else if(result == 2) {
        times = 5;
        isleft = 0;
    }
    if(isLastLeft == isleft){
        binary = 1;
    }else{
        binary *= 2;
        if(binary > 8){
            binary = 1;
        }
    }
    times *= binary;
    turnWaitTime(isleft, times);
}

void my_ardrone_node::turnWaitTime(bool isLeft, int waittime) {
    // turn 180 degree, means that return back origin road, so should continue turn.
    if(abs(bigturn - TURNTIME*2) <= 10){
        isBigCtrl = true;
        times = 30;
        bigturn = 0;
        return;
    }
    if(waittime == 0){
        isBigCtrl = false;
        return;
    }
    if(isLeft){
         dirIsChange = true;
         rotation_clock(ROTATION_SPEED);
         countturn ++;
         bigturn ++;
     }else {
        dirIsChange = true;
        rotation_inclock(ROTATION_SPEED);
        countturn --;
        bigturn --;
    }

    if(img_num % TIMEWINDOW == 0){
        allToZero();
    }
    countturn = countturn % (TURNTIME*4);
    times--;
}

void my_ardrone_node::linerWaitTime(bool isLeft, int waittime) {
    if(waittime == 0){
        isLinerCtrl = false;
        return;
    }
    if(isLeft){
         left(ROTATION_SPEED);
     }else {
        right(ROTATION_SPEED);
    }

    if(img_num % TIMEWINDOW == 0){
        allToZero();
    }
    times--;
}

void my_ardrone_node::hoverWaitTime(int waittime) {
    if(waittime == 0){
        isHoverCtrl = false;
        return;
    }
    hover();

    if(img_num % TIMEWINDOW == 0){
        allToZero();
    }
    times--;
}

void my_ardrone_node::objTurnWaitTime(bool isLeft, int waittime) {
    if(waittime == 0){
        times = 5;
        isForwardCtrl = true;
        forwardWaitTime(times);
        return;
    }

    if(isLeft){
         dirIsChange = true;
         rotation_clock(ROTATION_SPEED);
         countturn ++;
     }else {
        dirIsChange = true;
        rotation_inclock(ROTATION_SPEED);
        countturn --;
    }

    if(img_num % TIMEWINDOW == 0){
        allToZero();
    }

    countturn = countturn % (TURNTIME*4);
    times--;
}

void my_ardrone_node::forwardWaitTime(int waittime){
    if (waittime == 0) {
        isObjCtrl = false;
        isForwardCtrl = false;
        return;
    }

    forward(ardrone_speed);

    if(img_num % TIMEWINDOW == 0){
        allToZero();
    }

    times--;
}

float my_ardrone_node::getNextRotation(){
    if(result1 <= -BALANCE_LEVEL || result1 >= BALANCE_LEVEL){
        return (result1>0)?-1:1;
    }
    if(result2 <= -BALANCE_LEVEL || result2 >= BALANCE_LEVEL){
        return  (result2>0)?-1:1;
    }
    if(result3 <= -BALANCE_LEVEL || result3 >= BALANCE_LEVEL){
        return  (result3>0)?-1:1;
    }
    // 如果三层都平衡
    return 0;   // ????????????????????????????????????
}

void my_ardrone_node::updateLastRotation(int r){
    if(r == 1){
        LastRotation = 1;
    }else if(r == -1){
        LastRotation = -1;
    }
}

float my_ardrone_node::getSpeed(){
    // 光流越大速度越小
    double optflow = OF_front;
    if(optflow == 0){  // 什么情况下光流为０非常重要
        optflow = OF_left_right;
    }
    if(optflow == 0){  // 什么情况下光流为０非常重要
        optflow = OF_farLeft_right;
    }
    if(optflow == 0){
        return TRANSLATE_MAX_SPEED;
    }else{
        return TRANSLATE_LEVEL; // ?????????????????????????????????
    }
}

void my_ardrone_node::nav_callback(ardrone_autonomy::Navdata msg){
    theta = msg.rotZ;

    int theta_tmp= (theta < 0 )? (360+theta):theta;
    int curDirection_tmp=(curDirection<0)?(360+curDirection):curDirection;
    rotationNum = curDirection_tmp - theta_tmp ; //
    if(rotationNum>180){
        rotationNum=rotationNum-360;
    }
    else if(rotationNum<-180){
        rotationNum=rotationNum+360;
    }

    rotationNum=0.1*rotationNum; // 简单的P控制，可能有点粗糙????????????????????????????
}

void my_ardrone_node::odom_callback(nav_msgs::Odometry msg) {
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    z = msg.pose.pose.position.z;

}

void my_ardrone_node::modelstate_callback (const gazebo_msgs::ModelStatesConstPtr &msg) {
    size_t modelCount = msg->name.size(); 
    double brickPosX = 0;
    double brickPosY = 0;
    for(size_t modelInd = 0; modelInd < modelCount; ++modelInd)
    {
        if(msg->name[modelInd] == "quadrotor")
        {
            ardronePosX = msg->pose[modelInd].position.x;
            ardronePosY = msg->pose[modelInd].position.y;
            //cout << "ardrone x: " << ardronePosX << "  y: " << ardronePosY << endl;
        }
        if(scene == 3 && msg->name[modelInd] == "brick_box_3x1x3") // move right
        {
            brickPosX = msg->pose[modelInd].position.x;
            brickPosY = msg->pose[modelInd].position.y;
            //cout << "brick x: " << brickPosX << "  y: " << brickPosY << endl;
            gazebo_msgs::ModelState state;
            state.model_name = "brick_box_3x1x3";
            state.pose.position.y = brickPosY + 0.1;
            state.pose.position.x = brickPosX;
            if (state.pose.position.y > -16 && state.pose.position.y < 11 && state.pose.position.x > -10.5 && state.pose.position.x < 27.5) {
                model_state_pub.publish(state);
            }
        }
        if(scene == 3 && msg->name[modelInd] == "brick_box_3x1x3_0") // move left
        {
            brickPosX = msg->pose[modelInd].position.x;
            brickPosY = msg->pose[modelInd].position.y;
            //cout << "brick x: " << brickPosX << "  y: " << brickPosY << endl;
            gazebo_msgs::ModelState state;
            state.model_name = "brick_box_3x1x3_0";
            state.pose.position.y = brickPosY - 0.1;
            state.pose.position.x = brickPosX;
            if (state.pose.position.y > -16 && state.pose.position.y < 11 && state.pose.position.x > -10.5 && state.pose.position.x < 27.5) {
                model_state_pub.publish(state);
            }
        }
        if(scene == 3 && msg->name[modelInd] == "brick_box_3x1x3_1") //move down
        {
            brickPosX = msg->pose[modelInd].position.x;
            brickPosY = msg->pose[modelInd].position.y;
            //cout << "brick x: " << brickPosX << "  y: " << brickPosY << endl;
            gazebo_msgs::ModelState state;
            state.model_name = "brick_box_3x1x3_1";
            state.pose.position.y = brickPosY;
            state.pose.position.x = brickPosX + 0.1;
            if (state.pose.position.y > -16 && state.pose.position.y < 11 && state.pose.position.x > -10.5 && state.pose.position.x < 27.5) {
                model_state_pub.publish(state);
            }
        }
        if(scene == 3 && msg->name[modelInd] == "brick_box_3x1x3_2") //move up
        {
            brickPosX = msg->pose[modelInd].position.x;
            brickPosY = msg->pose[modelInd].position.y;
            //cout << "brick x: " << brickPosX << "  y: " << brickPosY << endl;
            gazebo_msgs::ModelState state;
            state.model_name = "brick_box_3x1x3_2";
            state.pose.position.y = brickPosY;
            state.pose.position.x = brickPosX - 0.1;
            if (state.pose.position.y > -16 && state.pose.position.y < 11 && state.pose.position.x > -10.5 && state.pose.position.x < 27.5) {
                model_state_pub.publish(state);
            }
        }
    }
}

my_ardrone_node::my_ardrone_node(int function, int scene) {
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_sub = it.subscribe("/ardrone/image_raw",1,&my_ardrone_node::process,this);
    takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    land_pub = n.advertise<std_msgs::Empty>("/ardrone/land",1);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    nav_sub = n.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata",1,&my_ardrone_node::nav_callback,this);
    odom_sub = n.subscribe<nav_msgs::Odometry>("ardrone/odom",1,&my_ardrone_node::odom_callback,this);
    model_state_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);
    model_state_sub = n.subscribe("/gazebo/model_states", 1, &my_ardrone_node::modelstate_callback, this);
    begin_rec = false;
    rotationNum = 0;
    curDirection = 0;
    theta = 0;
    textObstable = false;
    noTextObstable = false;
    dirIsChange = false;
    ardronePosX = 0;
    ardronePosY = 0;
    ardrone_speed = ARDRONE_CROSS_SPEED;
    if (scene == 1) {
        strategic = 7;
        ardrone_speed = ARDRONE_SPEED;
    }
    else if (scene == 2) {
        strategic = 11;
    }
    else if (scene == 3) {
        strategic = 19;
    }
    else if (scene == 4) {
        strategic = 11;
    }
    this->scene = scene;
    this->function = function;
    writer = cvCreateVideoWriter("/home/sarah/catkin_ws/src/avoid_optflow/video/flow.avi", CV_FOURCC('M', 'J', 'P', 'G'),20,cvSize(WIDTH, HEIGHT),1);
    writerprev = cvCreateVideoWriter("/home/sarah/catkin_ws/src/avoid_optflow/video/prev_img.avi", CV_FOURCC('M', 'J', 'P', 'G'),20,cvSize(WIDTH, HEIGHT),1);
    writercolor = cvCreateVideoWriter("/home/sarah/catkin_ws/src/avoid_optflow/video/color.avi", CV_FOURCC('M', 'J', 'P', 'G'),20,cvSize(WIDTH, HEIGHT),true);
    writergray = cvCreateVideoWriter("/home/sarah/catkin_ws/src/avoid_optflow/video/gray_img.avi", CV_FOURCC('M', 'J', 'P', 'G'),20,cvSize(WIDTH, HEIGHT),false);
}

my_ardrone_node::~my_ardrone_node(){
    cvReleaseVideoWriter(&writer);
    cvReleaseVideoWriter(&writerprev);
    cvReleaseVideoWriter(&writercolor);
    cvReleaseVideoWriter(&writergray);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"avoid_optflow");
    my_ardrone_node man = my_ardrone_node(1, 2);
    cout << "Press Up Down Left Right to control the quadrotor in the direction of forward,backward,turn left and turn right.\n  \
            Press A and D to control the quadrotor flying left and right. Presss W and S to control the quadrotor lift and down.\n \
            Press P to start/stop the record. Press esc to exit."<< endl;
            ros::spin();

    //avoidMain(argc, argv);
    //video2Image("/home/sarah/catkin_ws/src/avoid_optflow/video/flow.avi", "/home/sarah/catkin_ws/src/avoid_optflow/video/image");
    //image2Video(836, 1095, 0, "/home/sarah/catkin_ws/src/avoid_optflow/video/image");

    return 0;
}
