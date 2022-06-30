/**
 * @file pid.hh
 * @author NMM (nmm109@pitt.edu)
 * @brief Header file for the PID class 
 * @version 0.1
 * @date 2022-06-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __PID_H
    #define __PID_H

#pragma once 

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>

/* PID gain values and sampling time*/
struct pid_opts { 
    double  KP=1.2,
            KI=1.0, //unused for now
            KD=2.5;
    double TS = 1/120.0; 
} options; 
 
/**
 * @brief Pid class implementation
 * 
 * @todo add interface documentation for functions (nmm109)
 */
class PID 
{ 
public:
    PID( const geometry_msgs::PoseStamped&,
            mavros_msgs::AttitudeTarget&,
            const pid_opts &opts = options);
    void loop(const geometry_msgs::PoseStamped&);  
    double samplingTime(); 

private: 
    // priv variable to write to ros bag
    double u, u_1;  
    double e, e_1, e_2; 
    pid_opts opts; 
    geometry_msgs::PoseStamped::ConstPtr pose; 
    mavros_msgs::AttitudeTarget::Ptr att; 
};

#endif


