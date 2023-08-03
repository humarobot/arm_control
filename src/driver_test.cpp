#include "App/arm_control.h"

int main(int argc, char** argv){
  arx_arm arm_driver_;
  while(1){
    arm_driver_.CAN_Handlej.can2_adapter.socketcan_receiver_thread();
  }

  return 0;
}