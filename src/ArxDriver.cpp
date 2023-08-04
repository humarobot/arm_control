#include "ArxDriver.h"

void ArxDriver::Init() {
  std::cout << "ArxDriver: Can initializing ... " << std::endl;
  InitCan();
  can_size = sizeof(struct can_frame);
  last_time_ = high_resolution_clock::now();
}

void ArxDriver::Update() {
  const auto current_time = high_resolution_clock::now();
  // Compute desired duration rounded to clock decimation
  const duration<double> desired_duration(1.0 / loop_hz_);
  // Get change in time
  duration<double> time_span = duration_cast<duration<double>>(current_time - last_time_);
  last_time_ = current_time;

  // Check Data Integrity
  index_++;
  if (index_ == loop_hz_) {
    index_ = 0;
    for (int i = 0; i < 7; i++) frame_num_[i] = 0;
  }
  StatisticPrinter(index_);

  // TransReceive
  CanSendRecOnce();

  // Sleep
  const auto sleep_till = current_time + duration_cast<high_resolution_clock::duration>(desired_duration);
  std::this_thread::sleep_until(sleep_till);
}

void ArxDriver::InitCan() {
  struct ifreq ifr2;
  int setflag = 0, ret2 = 0;
  std::string n2 = "can2";
  const char* name2 = n2.c_str();
  if ((_s2 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while opening socket2");
  }
  strcpy(ifr2.ifr_name, name2);
  ioctl(_s2, SIOCGIFINDEX, &ifr2);
  _addr_2.can_family = AF_CAN;
  _addr_2.can_ifindex = ifr2.ifr_ifindex;
  if (bind(_s2, (struct sockaddr*)&_addr_2, sizeof(_addr_2)) < 0) {
    perror("Error in socket1 bind");
  }
  setflag = setflag | O_NONBLOCK;
  ret2 = fcntl(_s2, F_SETFL, setflag);
  fcntl(_s2, F_GETFL, 0);
  can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
  ret2 = setsockopt(_s2, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
  if (ret2 != 0) printf("setsockopt2 fail\n");

  std::cout << "ArxMotors: CAN init done!!!\n" << std::endl;
}

void ArxDriver::CanSendRecOnce() {
  int nbytes;
  for (u_char i = 1; i < 8; i++) {
    if (i != 3) {
      MsgToRaw(i, _frame2_s);
      nbytes = write(_s2, &_frame2_s, can_size);
      nbytes = read(_s2, &_frame_buffer, can_size);
      if (nbytes == 16) memcpy(&_frame2_r, &_frame_buffer, sizeof(_frame_buffer));

      RawToMsg(_frame2_r.can_id, _frame2_r);
    }
  }
}

void ArxDriver::StatisticPrinter(int index) {
  if (index == 499) {
    // TODO print connectivity
    float conn[7] = {0};
    for (int i = 0; i < 7; i++) {
      conn[i] = (float)frame_num_[i] / 4.99;
      std::cout << "Arx " << i + 1 << " connectivity is " << conn[i] << "%" << std::endl;
    }
    std::cout << "----------------------------------------------" << std::endl;
  }
}

void ArxDriver::MsgToRaw(int id, struct can_frame& frame) {
  frame.can_dlc = 8;
  send_motor_ctrl_cmd(id, 0, 0, 0, 0, 0, frame.data, &frame.can_id);
}

void ArxDriver::RawToMsg(int motor_id, struct can_frame& frame) {
  if (motor_id >= 0 && motor_id <= 7) {
    RV_can_data_repack(frame.can_id, frame.data, frame.can_dlc, 0);
    std::cout << "motor_id: " << rv_motor_msg[motor_id - 1].motor_id << std::endl;
    std::cout << "motor_angle: " << rv_motor_msg[motor_id - 1].angle_actual_rad << std::endl;
    std::cout << "motor_speed: " << rv_motor_msg[motor_id - 1].speed_actual_rad << std::endl;
    std::cout << "motor_torque: " << rv_motor_msg[motor_id - 1].current_actual_float << std::endl;
    frame_num_[motor_id - 1]++;
  }
}

void ArxDriver::CleanUp() {
  for (u_char i = 1; i < 8; i++) {
    if (i != 3) {
      can_frame frame;
      frame.can_dlc = 8;
      send_motor_ctrl_cmd(i, 0, 0, 0, 0, 0, frame.data, &frame.can_id);
      int nbytes = write(_s2, &frame, can_size);
    }
  }
  std::cout << "ArxDriver: Clean Done!!!!" << std::endl;
}