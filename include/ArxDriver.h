#pragma once

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <thread>
#include "Hardware/motor.h"

using namespace std::chrono;

class ArxDriver {
 public:
  ArxDriver(double loop_hz) : loop_hz_(loop_hz) {}
  ~ArxDriver() { CleanUp(); }
  void Init();
  void Update();

 private:
  void InitCan();
  void CanSendRecOnce();
  void CleanUp();
  void RawToMsg(int, struct can_frame&);
  void MsgToRaw(int, struct can_frame&);

  void StatisticPrinter(int);

  double loop_hz_;
  int index_{0};
  int frame_num_[7] = {0};
  int _s2;
  struct sockaddr_can _addr_2;
  size_t can_size;
  struct can_frame _frame2_s, _frame_buffer, _frame2_r;

  // Timing
  high_resolution_clock::time_point last_time_;
};