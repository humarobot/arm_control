#ifndef _ISAAC_GIMBAL_H_
#define _ISAAC_GIMBAL_H_

#include "../Hardware/can.h"
#include "../Hardware/motor.h"
#include "Eigen/Core"
#include "Hardware/teleop.h"
#include "utility.h"
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kdl.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <memory.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>

//
#define ARX5_PRO_L1 0.27
#define ARX5_PRO_L2 0.2766
#define A4310_kp 160
#define A4310_kD 5
// #define spd 100
// #define cur 1000
#define YAW_WORLD 0
#define BASE 1
#define ELBOW 2
#define PITCH_WAIST 3
#define YAW_WAIST 4
#define ROLL 5
#define GRASP 6

#define X 0
#define Y 1
#define Z 2

// Number of joints
#define NJ 6

#define FORWARD 0
#define DOWNWARD 1

#define JOYSTICK_DEADZONE 0.15

#define SIM_JOINT_KP 150
#define SIM_JOINT_KD 10

#define filter_sensor 0.3f
#define filter_vel 0.3f
#define filter_cmd 0.3f
#define filter_torque 0.3f
#define encos_up 2.09f
#define encos_down -2.09f

/* currently, one mission one class, if there exists multi mission and share arm
variables, either use a more gerenal class including mission instance or use
global struct
*/

struct command {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float base_yaw = 0.0f;
  float gripper_roll = 0.0f;
  float gripper = 0.0f;
  float waist_yaw = 0.0f;
  float waist_pitch = 0.0f;
  bool reset = false;
  int mode = FORWARD;
};


enum arx5_state { NORMAL, OUT_RANGE, OUT_BOUNDARY };

class FIFO_Queue {
public:
  void write_data(geometry_msgs::PoseStamped pos_stamped);
  geometry_msgs::PoseStamped read_nearest_data(ros::Time time);
  uint count = 0;
  std::vector<geometry_msgs::PoseStamped> data;
  uint write_ptr = 0, read_ptr = 0;
};

class arx_arm {
public:
  arx_arm(){}
  ~arx_arm() = default;

  void set_zero_torque();
  void get_joints_info();
  void get_joints_info(Eigen::VectorXd&, Eigen::VectorXd& );
  void set_joints_pos(const Eigen::VectorXd& pos);
  void set_joints_torque(const Eigen::VectorXd& torque);
  can CAN_Handlej;

private:

  float current_pos[7] = {0.0f};
  float current_vel[7] = {0.0};
  float pos_filted[3], vel_filted[3];
};

#endif