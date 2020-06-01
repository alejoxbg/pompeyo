#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv == 0)
        ROS_INFO("no_key_pressed");
    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double base_wheels_inc= 0.2; 
    double base_wheels= 0.66; 
    char c;
    double posx=0, posy=0;
    double incx= 0 ,incy=0;

    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(4);
        joint_state.position.resize(4);
	joint_state.name[0] ="base_to_unionhueca";
        joint_state.position[0] = base_link;
	joint_state.name[1] ="unionhueca_to_tubo1";
        joint_state.position[1] = unionhueca;
	joint_state.name[2] ="unionhueca_to_tubo2";
        joint_state.position[2] = unionhueca;
	joint_state.name[3] ="tubo2_to_unionsalida";
        joint_state.position[3] = tubo2;
	joint_state.name[4] ="tubo1_to_unionsalida2";
        joint_state.position[4] = tubo1;
	joint_state.name[5] ="unionsalida_to_unionhueca2";
        joint_state.position[5] = unionsalida;
	joint_state.name[6] ="unionhueca2_to_tubo2_2";
        joint_state.position[6] = unionhueca2;
	joint_state.name[7] ="unionhueca2_to_tubo2_3";
        joint_state.position[7] = unionhueca2;
	joint_state.name[8] ="tubo2_3_to_unionsalida3";
        joint_state.position[8] = tubo2_3;
	joint_state.name[9] ="tubo2_2_to_unionsalida4";
        joint_state.position[9] = tubo2_2;
	joint_state.name[10] ="unionsalida2_to_rueda1";
        joint_state.position[10] = unionsalida2;
	joint_state.name[11] ="unionsalida3_to_rueda2";
        joint_state.position[11] = unionsalida3;
	joint_state.name[12] ="unionsalida4_to_rueda3";
        joint_state.position[12] = unionsalida4;
	joint_state.name[13] ="base_to_unionhueca3";
        joint_state.position[13] = base_link;
	joint_state.name[14] ="unionhueca3_to_tubo1_2";
        joint_state.position[14] = unionhueca3;
	joint_state.name[15] ="unionhueca3_to_tubo2_4";
        joint_state.position[15] = unionhueca3;
	joint_state.name[16] ="tubo2_4_to_unionsalida5";
        joint_state.position[16] = tubo2_4;
	joint_state.name[17] ="tubo1_2_to_unionsalida6";
        joint_state.position[17] = tubo1_2;
	joint_state.name[18] ="unionsalida5_to_unionhueca4";
        joint_state.position[18] = unionsalida5;
	joint_state.name[19] ="unionhueca4_to_tubo2_5";
        joint_state.position[19] = unionhueca4;
	joint_state.name[20] ="unionhueca4_to_tubo2_6";
        joint_state.position[20] = unionhueca4;
	joint_state.name[21] ="tubo2_5_to_unionsalida7";
        joint_state.position[21] = tubo2_5;
	joint_state.name[22] ="tubo2_6_to_unionsalida8";
        joint_state.position[22] = tubo2_6;
	joint_state.name[23] ="unionsalida8_to_rueda4";
        joint_state.position[23] = unionsalida8;
	joint_state.name[24] ="unionsalida6_to_rueda5";
        joint_state.position[24] = unionsalida6;
	joint_state.name[25] ="unionsalida7_to_rueda6";
        joint_state.position[25] = unionsalida7;

                c=getch();
        switch(c)
            {
            case 'w':if(incx>0.02){}else{incx+=0.01;};
            break;
            case'd':if(incy>0.02){}else{incy+=0.01;};
            break;
            case 'a':if(incy<-0.02){}else{incy-=0.01;};
            break;
            case's': if(incx<-0.02){}else{incx-=0.01;};
            break;
            default:;
            }
        
                 if (c == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }

        // update transform
        // (moving in a circle with radius)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = posx;
        odom_trans.transform.translation.y = posy;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);


	// Create new robot state
          
          if(incx>0){
          base_wheels -= base_wheels_inc;
          if (base_wheels<0) base_wheels = 0.66;
		
		  }
          if(incx<0){
          base_wheels += base_wheels_inc;
          if (base_wheels<0) base_wheels = 0.66;

          }

            c='q';
            posx+=incx;
            posy+=incy;


        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
