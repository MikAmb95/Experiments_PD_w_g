#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <chrono>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include "my_roscpp_library/my_roscpp_library.h"
#include "my_roscpp_function/my_roscpp_function.h"

#include <iostream>

using namespace Eigen;
using namespace std;
using namespace std::chrono;



int main(int argc, char **argv) {


	ros::init(argc,argv,"ros_topic_publisher");
	ros::NodeHandle nh;
	ros::Publisher topic_pub = nh.advertise< std_msgs::Float64MultiArray > ("/iiwa_cmd",0);

        VectorXd q0 = VectorXd::Zero(7);
        VectorXd qf = VectorXd::Zero(7);
        MatrixXd qd(1001,7);
       int index = 0;
       ros::Rate rate(200);

        
        std_msgs::Float64MultiArray des_q;
        des_q.data.resize(7);

        
        //msg_r = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/iiwa_q");


        q0 << 0, 0, 0, 0, 0, 0,0; //Desired intermidiate position
        qf << -0.174, 0.174, -0.174, 0.174, -0.174, +0.174,-0.174; //Desired intermidiate position
       
        cout<<"DATA TRJ:"<<endl;
        cout<<q0.transpose()<<endl; 
        cout<<qf.transpose()<<endl; 
        qd = trj(q0,qf); //compute the trajectory from the initial position to the intermidiate one


        cout<<"New Start"<<endl;
	while (ros::ok()) {
                
                if(index == 1001) index = 1000; //when the trajectory is finished, we keep sending the last desired position
                for (int i=0; i<7; i++)  des_q.data[i] = qd(index,i); //the message that is sent over the socket is 14 dimension, the first 7 are the desired position
                index = index + 1;


		topic_pub.publish(des_q);
		rate.sleep();
	}
	return 0;
}

