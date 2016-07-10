#include "avoidoptflow.h"
#include "video.h"
#include "videoutil.h"

using namespace std;
using namespace cv;


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

