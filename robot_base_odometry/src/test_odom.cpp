#include <math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <N_Robot_Topic/NMotionCtrlTopic_EncoderCount_msg.h>//为嘛是红色我不知道，感觉应该是包的名字大写的原因
#include <N_Robot_Topic/NMotionCtrlTopic_ClearEncoderCount_msg.h>

using namespace std;
using namespace Eigen;

double kWheelRadius = 0.2032/2;
double kDecelerationRatio = 32;
double kEncoderLines = 2500;
Vector4d current_encoder,previous_encoder,encoder_diff,each_dist;
double coeffiecient_t = 2*M_PI*kWheelRadius/(kDecelerationRatio*kEncoderLines*4);

void encoderCallBack(const N_Robot_Topic::NMotionCtrlTopic_EncoderCount_msgConstPtr &Msg)
{
    if(Msg->isCommunicationOK)
    {
        current_encoder[0] = -Msg->encoder2Count;
        current_encoder[1] = -Msg->encoder3Count;
        current_encoder[2] = Msg->encoder4Count;
        current_encoder[3] = Msg->encoder1Count;
        each_dist = coeffiecient_t * current_encoder;
        std::cout << "左前轮【1】 ： " << each_dist[0] << "左后轮【2】 ： " << each_dist[1] << std::endl;
        std::cout << "右前轮【4】 ： " << each_dist[3] << "右后轮【3】 ： " << each_dist[2] << std::endl;
    }
}


int main(int argc,char** argv)
{
    ros::init(argc, argv, "test_odom");
    
    ros::NodeHandle node;
    ros::Publisher clear_pub = node.advertise<N_Robot_Topic::NMotionCtrlTopic_ClearEncoderCount_msg>("/NMotionCtrlTopic/ClearEncoderCount",50);

    sleep(2);

    N_Robot_Topic::NMotionCtrlTopic_ClearEncoderCount_msg clear_msg;
// 只包含一个值， bool command
    clear_msg.command = 1;
    clear_pub.publish(clear_msg);
    clear_pub.publish(clear_msg);
    clear_pub.publish(clear_msg);
// 向 ClearEncoderCount 话题上发布一条清零编码器的消息
// 然后等待一秒

    sleep(1);   

    ros::Subscriber encoder_sub = node.subscribe("/NMotionCtrlTopic/EncoderCount",1,encoderCallBack);
    ros::spin();
    return 0;
}