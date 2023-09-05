// This script

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
#include "my_roscpp_library/my_roscpp_library.h"
#include "my_roscpp_function/my_roscpp_function.h"



using namespace Eigen;
using namespace std;
using namespace std::chrono;


typedef struct ROBOT_STATE_STR {
  double jstate[14];
}ROBOT_STATE_STR;

typedef struct ROBOT_CMD_STR {
  double jcmd[14];
}ROBOT_CMD_STR;

//Creazione socket in LETTURA
inline bool listener_socket(int port_number, int *sock) {
      sockaddr_in si_me;

  if ( (*sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    std::cout << "Listener::Open: error during socket creation!" << std::endl;
    return false;
  }

  memset((char *) &si_me, 0, sizeof(si_me));

  /* allow connections to any address port */
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port_number);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  int bind_ok = bind(*sock, (struct sockaddr*)&si_me, sizeof(si_me));

  if ( bind_ok == -1 )
    return false;
  else
    return true;

}

//Creazione socket in SCRITTURA
inline int create_socket(char* dest, int port, int *sock) {
  struct sockaddr_in temp;
  struct hostent *h;
  int error;

  temp.sin_family=AF_INET;
  temp.sin_port=htons(port);
  h=gethostbyname(dest);

  if (h==0) {
    printf("Gethostbyname fallito\n");
    exit(1);
  }

  bcopy(h->h_addr,&temp.sin_addr,h->h_length);
  *sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  error=connect(*sock, (struct sockaddr*) &temp, sizeof(temp));
  return error;
}




class KUKA_INVKIN {
	public:
		KUKA_INVKIN();
		void run();
		bool init_robot_model();
		void ctrl_loop();

	private:
      	        int _jstate_socket;
                int _jcommand_socket;
		ros::NodeHandle _nh;
                ros::Publisher _my_pub;



};



bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	
	return true;
}

KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
	listener_socket(9030, &_jstate_socket);
        create_socket("192.170.10.146",9031,&_jcommand_socket);
        _my_pub = _nh.advertise< std_msgs::Float64MultiArray > ("/iiwa_status",0); //publish the robot status to be printed on a file

}


void KUKA_INVKIN::ctrl_loop() {


        VectorXd q_read = VectorXd::Zero(7);
        VectorXd tau_read = VectorXd::Zero(7);
        VectorXd q_des = VectorXd::Zero(7);
        VectorXd q0 = VectorXd::Zero(7);
        VectorXd qf = VectorXd::Zero(7);
        VectorXd qf2 = VectorXd::Zero(7);

        MatrixXd qd(1001,7);
	
        int slen2, rlen2;
	sockaddr_in si_other2;

        ROBOT_STATE_STR rs2;
        ROBOT_CMD_STR rc2;
	
        std_msgs::Float64MultiArray cmd_pub; //publisher
        cmd_pub.data.resize(21); //7 position 7desired position and 7 torques 

        int index = 0;
        int flg = 0;
        int flg2 = 0;
        int click = 0;

        ros::Rate r(200); // rate ros Node
        
       
        qf << 0.7854, 0, 0, 1.5708, -1.5708, -1.5708,-1.5708; //Desired intermidiate position
        
        qf2 << 0.7854-0.174, 0.174, -0.147, 1.5708+0.174, -1.5708-0.174, -1.5708+0.174,-1.5708-0.174; //Desired intermidiate position
        

        rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
        while(rlen2<0) cout<<"nothing received"<<endl; 
        
        if(rlen2>0) for(int i=0; i<7; i++ ) q0(i) = rs2.jstate[i];
         

        cout<<"DATA TRJ:"<<endl;
        cout<<q0.transpose()<<endl; 
        cout<<qf2.transpose()<<endl; 
        qd = trj(q0,qf2); //compute the trajectory from the initial position to the intermidiate one

        cout<<"START, PRESS 1 + ENTER! :";
        cin>>click;
        
        index = 0;
        while( ros::ok() ) {
 
                //START ROS NODE               
                
                
		

                rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
                if(rlen2>0) for(int i = 0;i<14;i++) cmd_pub.data[i] = rs2.jstate[i]; //copy the current q and trq
                for(int i = 14;i<21;i++) cmd_pub.data[i] = rc2.jcmd[i-14];
                _my_pub.publish(cmd_pub);


                if(index == 1001) index = 1000; //when the trajectory is finished, we keep sending the last desired position
                for (int i=0; i<7; i++)  rc2.jcmd[i] = qd(index,i); //the message that is sent over the socket is 14 dimension, the first 7 are the desired position
                for (int i=7; i<14; i++) rc2.jcmd[i] = 0; //the other 7 should be torque, this is why we wrote 0, these are not used in the control scheme robot side
                write( _jcommand_socket, &rc2, sizeof(rc2) ); //write commands over socket

              
		index = index + 1; //increase the flag to move the desired q
                
                r.sleep();
	}


}


void KUKA_INVKIN::run() {
	boost::thread ctrl_loop_t ( &KUKA_INVKIN::ctrl_loop, this);
	ros::spin();	
}




int main(int argc, char** argv) {
         
        cout<<"Start Fin"<<endl;
	ros::init(argc, argv, "iiwa_kdl");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
