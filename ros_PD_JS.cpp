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
using namespace KDL;
using namespace std::chrono;



                                                                                                             




typedef struct ROBOT_STATE {
  double jstate[7];
}ROBOT_STATE;

typedef struct ROBOT_CMD {
  double jcmd[7];
}ROBOT_CMD;

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
                void pd_g();
                void Joint_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
		
                

	private:
      	        int _jstate_socket;
                int _jcommand_socket;

	        
                ros::Subscriber _js_sub;
                VectorXd ref_q = VectorXd::Zero(7);
 
		ros::NodeHandle _nh;


                Eigen::MatrixXd Kd; 	
		Eigen::MatrixXd Kp;
                ros::Publisher _my_pub;
                	

                

	
};


bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	

        Kp = MatrixXd::Zero(7,7);
        Kp(0,0) = 100;
        Kp(1,1) = 200;
        Kp(2,2) = 100;
        Kp(3,3) = 700;
        Kp(4,4) = 100;
        Kp(5,5) = 100;
        Kp(6,6) = 100;
        


	Kd = MatrixXd::Zero(7,7);
        Kd(0,0) = 10;
        Kd(1,1) = 12;
        Kd(2,2) = 10;
        Kd(3,3) = 10;
        Kd(4,4) = 10;
        Kd(5,5) = 10;
        Kd(6,6) = 10;
	return true;
}


//SUBSCRIBER

void KUKA_INVKIN::Joint_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {

  for(int i = 0; i<7; i++) ref_q(i)= msg->data[i];
   cout<<"ref: "<<ref_q.transpose()<<endl;     
}



// CODE FROM HERE

KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
	

        _js_sub = _nh.subscribe("/iiwa_cmd", 0, &KUKA_INVKIN::Joint_Callback, this);
        _my_pub = _nh.advertise< std_msgs::Float64MultiArray > ("/iiwa_status",0);

 	listener_socket(9030, &_jstate_socket);
        create_socket("192.170.10.146",9031,&_jcommand_socket);

}





void KUKA_INVKIN::pd_g() {

int slen2, rlen2;	
ROBOT_STATE rs2;
sockaddr_in si_other2;
ROBOT_CMD rc;

std_msgs::Float64MultiArray cmd_pub;
cmd_pub.data.resize(21);

VectorXd q_in = VectorXd::Zero(7);
VectorXd q_0 = VectorXd::Zero(7);
VectorXd q0 = VectorXd::Zero(7);
VectorXd q_dot = VectorXd::Zero(7);

float Ts = 0.005;
float K = 1000;



q0(0) = 45*3.14/180;
q0(1) = 0*3.14/180;
q0(2) = 0*3.14/180;
q0(3) = 90*3.14/180;
q0(4) = -90*3.14/180;
q0(5) = -90*3.14/180;
q0(6) = -90*3.14/180;


rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
          if(rlen2>0) {
                 for(int i=0; i<7; i++) q_0(i) = rs2.jstate[i];
        }

q_in = q0;

       while(true) {
       
       //cout<<"PD_g"<<endl;
               
       rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
          if(rlen2>0) {
                 for(int i=0; i<7; i++) q_in(i) = rs2.jstate[i];
         }
            


        q_dot = (2*K*q_in - 2*K*q_0 - (Ts*K-2)*q_dot)/(Ts*K+2);
        q_0 = q_in;


        Eigen::VectorXd e = ref_q - q_in + q0; 
        //cout<<"ERR: "<<e.transpose()<<endl;
        Eigen::VectorXd tao = Kp*e - Kd*q_dot;
        //cout<<"trq: "<<tao.transpose()<<endl;

        for (int i=0; i<7; i++) rc.jcmd[i] = tao(i);
        write( _jcommand_socket, &rc, sizeof(rc) ); //write commands over socket

        for(int i=0; i<7; i++ ) cmd_pub.data[i] = rs2.jstate[i];
        for(int i=7; i<14; i++ ) cmd_pub.data[i] = tao(i-7);
        for(int i=14; i<21; i++ ) cmd_pub.data[i] = ref_q(i-14)+q0(i-14);
        _my_pub.publish(cmd_pub);
       
        
        
        }




}


void KUKA_INVKIN::run() {

	boost::thread pd_g ( &KUKA_INVKIN::pd_g, this);
	
	ros::spin();	

}




int main(int argc, char** argv) {
      

        cout<<"INIT"<<endl;

        
	ros::init(argc, argv, "pd_ral");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
