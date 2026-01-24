//
// Created by leo on 18-6-2.
//

#include "robot_base_odometry.h"

RobotBaseOdometry::RobotBaseOdometry(double wheel_radius, double base_width, double base_length, int deceleration_radio, int encoder_lines):
    kWheelRadius(wheel_radius),kBaseWidth(base_width),kBaseLength(base_length),kDecelerationRatio(deceleration_radio),kEncoderLines(encoder_lines)
{
    clear_pub = node.advertise<N_Robot_Topic::NMotionCtrlTopic_ClearEncoderCount_msg>("/NMotionCtrlTopic/ClearEncoderCount",50);

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

    encoder_sub = node.subscribe("/NMotionCtrlTopic/EncoderCount",1,&RobotBaseOdometry::EncoderCallBack,this);
    odometry_pub = node.advertise<nav_msgs::Odometry>("odom", 50);

    coeffiecient_t = 2*M_PI*kWheelRadius/(kDecelerationRatio*kEncoderLines*4);
    coeffiecient_k = (kBaseLength+kBaseWidth)/2;					// (2a+2b)/2 = (a+b)
    matrix_f.resize(3,4);
    matrix_f <<  0.25,     0.25,     0.25,     0.25,
                -0.25,     0.25,    -0.25,     0.25,
                -1/(4*coeffiecient_k),-1/(4*coeffiecient_k), 1/(4*coeffiecient_k), 1/(4*coeffiecient_k);
    current_time = ros::Time::now();
    previous_time = ros::Time::now();
    base_distance = Vector3d::Zero();
    base_velocity = Vector3d::Zero();
    base_distance_diff = Vector3d::Zero();
    current_encoder = Vector4d::Zero();
    previous_encoder = Vector4d::Zero();
    encoder_diff = Vector4d::Zero();
}

void RobotBaseOdometry::EncoderCallBack(const N_Robot_Topic::NMotionCtrlTopic_EncoderCount_msgConstPtr &Msg)
{
/*
bool isCommunicationOK
int32 encoder1Count
int32 encoder2Count
int32 encoder3Count
int32 encoder4Count
*/

    if(Msg->isCommunicationOK)
    {
        current_time = ros::Time::now();
        double dt = (current_time-previous_time).toSec();

        //这里要调整顺序
/*
一般的麦轮机器人		y = (w1+w2+w3+w4)/4
^ y
|				x = (-w1+w2-w3+w4)/4
| 
------> x			w = (w1-w2-w3+w4)/(2a+2b)
一般麦轮车的编码器顺序

2	1



3	4

当前车编码器顺序

2	1



3	4
现在的坐标系
	^ x			x = (w1+w2+w3+w4)/4
	|			y = (-w1+w2-w3+w4)/4
	|			w = (-w1-w2+w3+w4)/4
y<-------
现在车的编码器顺序
1	4



2	3

*/
        current_encoder[0] = -Msg->encoder2Count;
        current_encoder[1] = -Msg->encoder3Count;
        current_encoder[2] = Msg->encoder4Count;
        current_encoder[3] = Msg->encoder1Count;
        encoder_diff = current_encoder - previous_encoder;		// 编码器的差值

        //根据四个轮子绝对码盘值进行运动学正解，先求得当前机器人底盘速度，然后速度积分获得位置
        base_distance_diff = matrix_f * (encoder_diff * coeffiecient_t);			// 变换矩阵 * 四个轮子的转速组成的向量,括号里是每个车轮的速度
        base_velocity = base_distance_diff/dt;

        //由局部坐标系转化到全局坐标系下
        double delta_x = base_distance_diff[0]*cos(base_distance[2]) - base_distance_diff[1]*sin(base_distance[2]);
        double delta_y = base_distance_diff[0]*sin(base_distance[2]) + base_distance_diff[1]*cos(base_distance[2]);
        double delta_theta = base_distance_diff[2];

        base_distance[0] += delta_x;
        base_distance[1] += delta_y;
        base_distance[2] += delta_theta;

        //cout<<base_distance<<endl<<endl;

        //向前赋值
        previous_time = current_time;
        previous_encoder = current_encoder;

        //通过tf讲z轴转角转为四元数
        geometry_msgs::Quaternion odometry_quaternion = tf::createQuaternionMsgFromYaw(base_distance[2]);

        //发布odom和base_link的tf变换
        geometry_msgs::TransformStamped odometry_tf;
        odometry_tf.header.stamp = current_time;
        odometry_tf.header.frame_id = "odom";
        odometry_tf.child_frame_id = "base_link";
        odometry_tf.transform.translation.x = base_distance[0];
        odometry_tf.transform.translation.y = base_distance[1];
        odometry_tf.transform.translation.z = 0.0;
        odometry_tf.transform.rotation = odometry_quaternion;
        odometry_broadcaster.sendTransform(odometry_tf);


        //将odometry发布到odom的topic上
        nav_msgs::Odometry odometry;
        odometry.header.stamp = current_time;
        odometry.header.frame_id = "odom";
        odometry.child_frame_id = "base_link";
        odometry.pose.pose.position.x = base_distance[0];
        odometry.pose.pose.position.y = base_distance[1];
        odometry.pose.pose.position.z = 0.0;
        odometry.pose.pose.orientation = odometry_quaternion;
        odometry.twist.twist.linear.x = base_velocity[0];
        odometry.twist.twist.linear.y = base_velocity[1];
        odometry.twist.twist.angular.z = base_velocity[2];

        //cout<<odometry.pose.pose<<endl<<endl;
        cout<<odometry.pose.pose.position.x<<" "
            <<odometry.pose.pose.position.y<<" "
            <<odometry.pose.pose.position.z<<" "
            <<odometry.pose.pose.orientation.x<<" "
            <<odometry.pose.pose.orientation.y<<" "
            <<odometry.pose.pose.orientation.z<<" "
            <<odometry.pose.pose.orientation.w<<" "<<endl;
        odometry_pub.publish(odometry); 

        //cout<<base_distance<<endl<<endl;
        //cout<<current_base_velocity<<endl<<endl;

    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"robot_base_odometry");
    RobotBaseOdometry RobotBaseOdometry(0.2032/2,0.595,0.479,32,2500);//单位都是m
    ros::spin();
    return 0;
}

