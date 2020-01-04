/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Description: Core configuration file for foxbot robot.
 *  Author: Matthieu Magnon
 *
 *  Serial communication is enabled with the following command:
 *  rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
 */

#ifndef FOXBOT_CORE_CONFIG_H_
#define FOXBOT_CORE_CONFIG_H_

// enable this line for Arduino Due
#define USE_USBCON

/* Include librairies */
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <Timer.h>
#include <MedianFilter.h>

/* Global parameters */
#define FREQUENCY_RATE 						30 			// [ms] default 50ms
#define FREQUENCY_ODOMETRY 				    150 		// [ms] default 250ms
#define FREQUENCY_ROSPINONCE 				150 		// [ms]
#define FREQUENCY_CONTROLLER 				30 			// [ms] default 50ms

/* Rate computing parameters */
#define RATE_DIRECTION_MEDIAN_FILTER_SIZE 	3
#define RATE_CONV 							0.0073882 	// [inc] -> [rad]
// 853 (234) inc per wheel rotation
#define RATE_AVERAGE_FILTER_SIZE 			4

/* Rate controller parameters */
#define PWM_MAX				 				4095            // 12 bit
#define RATE_CONTROLLER_KP 					130.0 		    // default 150
#define RATE_CONTROLLER_KD 					5000000000000.0 //4500000000000
#define RATE_CONTROLLER_KI 					0.00005 	    //0.00001
#define RATE_INTEGRAL_FREEZE				250
#define RATE_CONTROLLER_MIN_PWM 			-500
#define RATE_CONTROLLER_MAX_PWM 			500

/* Mechanical parameters */
#define WHEEL_RADIUS 						0.0326 		// [m]
// distance between the two wheels
#define BASE_LENGTH 						0.272 		// [m]  0.288

/* Define frequency loops */
Timer _frequency_rate(FREQUENCY_RATE);
Timer _frequency_odometry(FREQUENCY_ODOMETRY);
Timer _frequency_rospinonce(FREQUENCY_ROSPINONCE);
Timer _frequency_controller(FREQUENCY_CONTROLLER);

/* Define median filter for direction */
MedianFilter motor_right_direction_median_filter(RATE_DIRECTION_MEDIAN_FILTER_SIZE);
MedianFilter motor_left_direction_median_filter(RATE_DIRECTION_MEDIAN_FILTER_SIZE);

/* Define pins */
// motor A (right)
const byte motorRightEncoderPinA = 38;
const byte motorRightEncoderPinB = 34;
const byte enMotorRight = 2;
const byte in1MotorRight = 4;   //26 C1 M1
const byte in2MotorRight = 3;   //28 C2 M2

// motor B (left)
const byte motorLeftEncoderPinA = 26;
const byte motorLeftEncoderPinB = 30;
const byte enMotorLeft = 7;
const byte in1MotorLeft = 6;    //30
const byte in2MotorLeft = 5;    //32

/* Define motors variables */
// right
volatile int motor_right_inc;
int motor_right_direction;
int motor_right_filtered_direction;
float motor_right_filtered_inc_per_second;
float motor_right_rate_est;
float motor_right_rate_ref;
int motor_right_check_dir;
int motor_right_pwm_rate;
unsigned long motor_right_prev_time;
int pwmMotorRight = 0;
// left
volatile int motor_left_inc;
int motor_left_direction;
int motor_left_filtered_direction;
float motor_left_filtered_inc_per_second;
float motor_left_rate_est;
float motor_left_rate_ref;
int motor_left_check_dir;
int motor_left_pwm_rate;
unsigned long motor_left_prev_time;
int pwmMotorLeft = 0;

/* Define controllers variables */
// right
unsigned long controler_motor_right_prev_time;
float controler_motor_right_prev_epsilon = 0.0;
float controler_motor_right_int = 0.0;
// left
unsigned long controler_motor_left_prev_time;
float controler_motor_left_prev_epsilon = 0.0;
float controler_motor_left_int = 0.0;

/* Mixer variable */
float linear_velocity_ref;
float angular_velocity_ref;
float linear_velocity_est;
float angular_velocity_est;

float yaw_est;
unsigned long odom_prev_time;

/* ROS Nodehanlde */
ros::NodeHandle  nh;

/* Prototype function */
void rateControler(const float rate_ref, const float rate_est, int & pwm_rate,
                   unsigned long & prev_time, float & previous_epsilon,
                   float & integral_epsilon);
float runningAverage(float prev_avg, const float val, const int n);
void setMotorRateAndDirection(int pwm_ref, const float rate_ref,
                              const byte enMotor, const byte in1Motor, const byte in2Motor);

/* Velocity command subscriber */
// callback function prototype
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
// message
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", commandVelocityCallback);

/* Odometry publisher */
nav_msgs::Odometry odom;
ros::Publisher odom_publisher("odom", &odom);

/* DEBUG */
#include <geometry_msgs/Point.h>
geometry_msgs::Point debug_left;
ros::Publisher debug_publisher1("debug_left", &debug_left);

#include <geometry_msgs/Point.h>
geometry_msgs::Point debug_right;
ros::Publisher debug_publisher2("debug_right", &debug_right);

template <typename type>
type sign(type value) {
    return type((value>0)-(value<0));
}


#endif // FOXBOT_CORE_CONFIG_H_
