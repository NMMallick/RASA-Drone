#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

/* GLOBALS */
mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose; 
mavros_msgs::AttitudeTarget att;


/* PID gain values */
struct { 
    const double KP=0.1,
                 KI=1.0, 
                 KD=20;

    const double TS = 20.0; 
} constants; 

double u=0, u_1=0, e=0, e_1=0, e_2=0; 

/* Call backs */

// PX4  STATE CALL BACK 
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{ 
    current_state = *msg; 
}

// POSITION CALL BACK 
void pose_cb( const geometry_msgs::PoseStamped::ConstPtr& msg ) 
{ 
    pose = *msg; 
}

/* PID IMPLEMENTATION  */
void pid(float sp) 
{ 
    e = sp - pose.pose.position.z; 

    u = u_1 + 
        ( constants.KP + (constants.KD / constants.TS))*e + 
        (-constants.KP - (2*constants.KD/constants.TS))*e_1 + 
        (constants.KD/constants.TS)*e_2; 
    
    if ( u > 1 ) att.thrust = 1.0; 
    else if ( u < 0 ) att.thrust = 0.0; 
    else att.thrust = u;  

    // Updating variables 
    u_1 = u; 
    e_2 = e_1; 
    e_1 = e; 
}


int main(int argc, char **argv) { 
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 20, pose_cb); 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher sp_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(constants.TS);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ROS_INFO_ONCE("Waiting for FCU connection..."); 
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connected to FCU"); 
    
    // Create message for FCU 
    geometry_msgs::Quaternion q; 
    q.w = 1;
    q.x = 0;
    q.y = 0;
    q.z = 0;

    geometry_msgs::Vector3 v; 
    v.x = 0; 
    v.y = 0; 
    v.z = 0; 

    //
    // Initial values to send to the FCU 
    //
    att.body_rate = v; 
    att.orientation = q; 
    att.thrust = 0.0; 

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        sp_attitude_pub.publish(att); 
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else 
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        
        if( current_state.mode == "OFFBOARD" && current_state.armed ) 
        {
            ROS_INFO_ONCE("Calling PID"); 
            pid(10.0);
        }

        //message
        sp_attitude_pub.publish(att);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
