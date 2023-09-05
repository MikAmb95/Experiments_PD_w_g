
#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>
#include <chrono>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>




using namespace Eigen;
using namespace std;
using namespace KDL;
using namespace std::chrono;


class KUKA_INVKIN {
	public:
		KUKA_INVKIN();
		void run();
		bool init_robot_model();
		void ctrl_loop();
                void robot_status(const std_msgs::Float64MultiArray::ConstPtr& msg);


	private:

		ros::NodeHandle _nh;

                ros::Subscriber sub_robot;

 
                _Float64 mex_robot[21];
                VectorXd v_mex_robot = VectorXd::Zero(21);

};



bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	
	return true;
}

KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
        sub_robot = _nh.subscribe("/iiwa_status", 0, &KUKA_INVKIN::robot_status,this); //subscriber to the desired cmd
}

 
void KUKA_INVKIN::robot_status(const std_msgs::Float64MultiArray::ConstPtr& msg) {


 for(int i=0; i<21; i++ ) mex_robot[i] = msg->data[i];
 for(int i=0; i<21; i++ ) v_mex_robot[i] = msg->data[i];

}





void KUKA_INVKIN::ctrl_loop() {

        ros::Rate r(200); // rate ros Node
        

        ofstream file_iiwa;
        ofstream file_time;

        file_iiwa.open("/home/utente/ros_ws/src/iiwa_kdl/src/matlab_results/iiwa.m");
        file_time.open("/home/utente/ros_ws/src/iiwa_kdl/src/matlab_results/time.m");

        file_iiwa<<"data_iiwa = [";
        file_time<<"data_time = [";

        double my_time = 0;
        auto start = high_resolution_clock::now();
        while( ros::ok() ) {
 
                        if(my_time<15.0){
                        file_iiwa<<v_mex_robot.transpose()<<" ";
                        file_iiwa<<"\n";
                        file_time<<my_time<<" ";
                        file_time<<"\n";

                        }
                        else{
                        file_iiwa<<v_mex_robot.transpose()<<"]; \n";
                        file_iiwa.close();
                        file_time<<my_time<<"]; \n";
                        file_time.close();
                        } 


                auto stop = high_resolution_clock::now();        
                auto duration = duration_cast<microseconds>(stop - start);
                my_time = duration.count()*0.000001;
                cout<<"time: "<<my_time<<endl;

		r.sleep();
	}


}


void KUKA_INVKIN::run() {
	boost::thread ctrl_loop_t ( &KUKA_INVKIN::ctrl_loop, this);
	ros::spin();	
}




int main(int argc, char** argv) {
         
        cout<<"Start Fin"<<endl;
	ros::init(argc, argv, "ros_topic_subscriber");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
