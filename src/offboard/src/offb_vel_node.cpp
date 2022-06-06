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
double kp=1/40.0, 
       ki=1/40.0, 
       kd=0; 

/* ERROR */
double e, prev_e=0; 
double P, I, D; 

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
void pid(float sp) { 
    
    float u; 
    
    e = sp - pose.pose.position.z;
    prev_e = e;

    // ROS_INFO("e: %f", sp - pose.pose.position.z);  
    P = kp*e; 
    I = ki*(e+prev_e); 

    // ROS_INFO("actual e: %f", e);  
    // ROS_INFO("kp: %f", kp); 
    // ROS_INFO("p: %f", P); 
    u = P + I; 
    
    ROS_INFO("u: %f", u); 

    if (u > 1) att.thrust = 1.0; 
    else if ( u < 0 ) att.thrust = 0.0; 
    else att.thrust = u;  
}


int main(int argc, char **argv) { 
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb); 
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
    ros::Rate rate(20.0);

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
            pid(10.0);
        }
        
        //message
        sp_attitude_pub.publish(att); 

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
