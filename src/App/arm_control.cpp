#include "App/arm_control.h"
#include "Eigen/src/Core/Matrix.h"

#define postion_control_spd 300
#define postion_control_cur 1000
extern OD_Motor_Msg rv_motor_msg[8];
extern m_rmd_t rmd_01;

void arx_arm::get_joints_info(Eigen::VectorXd& p, Eigen::VectorXd& v) {
  // current mode   力位混控模式
  current_pos[0] = rv_motor_msg[0].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[0] * filter_sensor;
  current_pos[1] = rv_motor_msg[1].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[1] * filter_sensor;
  current_pos[2] = rv_motor_msg[3].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[2] * filter_sensor;
  current_pos[3] = rv_motor_msg[4].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[3] * filter_sensor;
  current_pos[4] = rv_motor_msg[5].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[4] * filter_sensor;
  current_pos[5] = rv_motor_msg[6].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[5] * filter_sensor;
  p.setZero(6);
  p << current_pos[0], current_pos[1], current_pos[2], current_pos[3],
      current_pos[4], current_pos[5];

  current_vel[0] = rv_motor_msg[0].speed_actual_rad;
  current_vel[1] = rv_motor_msg[1].speed_actual_rad;
  current_vel[2] = rv_motor_msg[3].speed_actual_rad;
  current_vel[3] = rv_motor_msg[4].speed_actual_rad;
  current_vel[4] = rv_motor_msg[5].speed_actual_rad;
  current_vel[5] = rv_motor_msg[6].speed_actual_rad;
  v.setZero(6);
  v << current_vel[0], current_vel[1], current_vel[2], current_vel[3],
      current_vel[4], current_vel[5];

  ROS_INFO("\033[34m >>>>>>>>>> pos >>>>>>>>>> \033[34m"); //
  ROS_INFO("\033[34m pos = 1>%f 2>%f 3>%f 4>%f 5>%f 6>%f 7>%f \033[34m",
           current_pos[0], current_pos[1], current_pos[2], current_pos[3],
           current_pos[4], current_pos[5], current_pos[6]);
  ROS_INFO("\033[34m vel = 1>%f 2>%f 3>%f 4>%f 5>%f 6>%f 7>%f \033[34m",
           current_vel[0], current_vel[1], current_vel[2], current_vel[3],
           current_vel[4], current_vel[5], current_vel[6]);
}
void arx_arm::get_joints_info() {
  // ！！！ ID 3 电机为另一版本型号并联的第二个关节，在此不使用
  // position mode  位置模式
  // current_pos[0] = rv_motor_msg[0].angle_actual_float/180*M_PI;
  // current_pos[1] = rv_motor_msg[1].angle_actual_float/180*M_PI;
  // current_pos[2] = rv_motor_msg[3].angle_actual_float/180*M_PI;
  // current_pos[3] = rv_motor_msg[4].angle_actual_float/180*M_PI;
  // current_pos[4] = rv_motor_msg[5].angle_actual_float/180*M_PI;
  // current_pos[5] = rv_motor_msg[6].angle_actual_float/180*M_PI;

  // current mode   力位混控模式
  current_pos[0] = rv_motor_msg[0].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[0] * filter_sensor;
  current_pos[1] = rv_motor_msg[1].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[1] * filter_sensor;
  current_pos[2] = rv_motor_msg[3].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[2] * filter_sensor;
  current_pos[3] = rv_motor_msg[4].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[3] * filter_sensor;
  current_pos[4] = rv_motor_msg[5].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[4] * filter_sensor;
  current_pos[5] = rv_motor_msg[6].angle_actual_rad * (1 - filter_sensor) +
                   current_pos[5] * filter_sensor;
  current_vel[0] = rv_motor_msg[0].speed_actual_rad;
  current_vel[1] = rv_motor_msg[1].speed_actual_rad;
  current_vel[2] = rv_motor_msg[3].speed_actual_rad;
  current_vel[3] = rv_motor_msg[4].speed_actual_rad;
  current_vel[4] = rv_motor_msg[5].speed_actual_rad;
  current_vel[5] = rv_motor_msg[6].speed_actual_rad;

  ROS_INFO("\033[34m >>>>>>>>>> pos >>>>>>>>>> \033[34m"); //
  ROS_INFO("\033[34m pos = 1>%f 2>%f 3>%f 4>%f 5>%f 6>%f 7>%f \033[34m",
           current_pos[0], current_pos[1], current_pos[2], current_pos[3],
           current_pos[4], current_pos[5], current_pos[6]);
  ROS_INFO("\033[34m vel = 1>%f 2>%f 3>%f 4>%f 5>%f 6>%f 7>%f \033[34m",
           current_vel[0], current_vel[1], current_vel[2], current_vel[3],
           current_vel[4], current_vel[5], current_vel[6]);
}

void arx_arm::set_joints_pos(const Eigen::VectorXd& pos) {
  assert(pos.size() == 6);
  CAN_Handlej.Can_cmd_all(1, 60, 1.2, pos(0), 0, 0);
  CAN_Handlej.Can_cmd_all(2, 60, 1.2, pos(1), 0, 0);
  CAN_Handlej.Can_cmd_all(4, 60, 1.2, pos(2), 0, 0);
  CAN_Handlej.Can_cmd_all(5, 25, 0.8, pos(3), 0, 0);
  CAN_Handlej.Can_cmd_all(6, 10, 0.8, pos(4), 0, 0);
  CAN_Handlej.Can_cmd_all(7, 10, 0.8, pos(5), 0, 0);
  ROS_INFO("\033[32m <<<<< Send joints position command <<<<<< \033[32m");
}

void arx_arm::set_joints_torque(const Eigen::VectorXd& torque){
  assert(torque.size() == 6);
  CAN_Handlej.Can_cmd_all(1, 0, 0, 0, 0, torque(0));
  CAN_Handlej.Can_cmd_all(2, 0, 0, 0, 0, torque(1));
  CAN_Handlej.Can_cmd_all(4, 0, 0, 0, 0, torque(2));
  CAN_Handlej.Can_cmd_all(5, 0, 0, 0, 0, torque(3));
  CAN_Handlej.Can_cmd_all(6, 0, 0, 0, 0, torque(4));
  CAN_Handlej.Can_cmd_all(7, 0, 0, 0, 0, torque(5));
  ROS_INFO("\033[32m <<<<< Send joints torque command <<<<<< \033[32m");  
}


void arx_arm::set_zero_torque() {

  CAN_Handlej.Can_cmd_all(1, 0, 10, 0, 0, 0);
  CAN_Handlej.Can_cmd_all(2, 0, 10, 0, 0, 0);
  CAN_Handlej.Can_cmd_all(4, 0, 10, 0, 0, 0);
  CAN_Handlej.Can_cmd_all(5, 0, 0.2, 0, 0, 0);
  CAN_Handlej.Can_cmd_all(6, 0, 0.2, 0, 0, 0);
  CAN_Handlej.Can_cmd_all(7, 0, 0.2, 0, 0, 0);

}
