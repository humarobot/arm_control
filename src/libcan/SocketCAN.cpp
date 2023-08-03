/**
 * @file
 * This file implements functions to receive
 * and transmit CAN frames via SocketCAN.
 */

#ifndef MINGW

#include "libcan/SocketCAN.h"
#include <stdio.h>
// strncpy
#include <string.h>
// close sleep
#include <unistd.h>
// socket
#include <sys/socket.h>
// SIOCGIFINDEX
#include <sys/ioctl.h>

/*
 * https://github.com/JCube001/socketcan-demo
 * http://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-c-in-linux
 * https://github.com/linux-can/can-utils/blob/master/candump.c
 */

/*
#include <endian.h>
#include <sys/socket.h>
#include <sys/types.h>
*/

SocketCAN::SocketCAN() : CANAdapter(), sockfd(-1), receiver_thread_id(0) {
  adapter_type = ADAPTER_SOCKETCAN;
  printf("SocketCAN adapter created.\n");
}

SocketCAN::~SocketCAN() {
  printf("Destroying SocketCAN adapter...\n");
  if (this->is_open()) {
    this->close();
  }
}

void SocketCAN::open(const char *interface) {
  // Request a socket
  sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sockfd == -1) {
    printf("Error: Unable to create a CAN socket\n");
    return;
  }
  printf("Created CAN socket with descriptor %d.\n", sockfd);

  // Get the index of the network interface
  strncpy(if_request.ifr_name, interface, IFNAMSIZ);
  if (ioctl(sockfd, SIOCGIFINDEX, &if_request) == -1) {
    printf("Unable to select CAN interface %s: I/O control error\n", interface);

    // Invalidate unusable socket
    close();
    return;
  }
  printf("Found: %s has interface index %d.\n", interface,
         if_request.ifr_ifindex);

  // Bind the socket to the network interface
  addr.can_family = AF_CAN;
  addr.can_ifindex = if_request.ifr_ifindex;
  int rc =
      bind(sockfd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
  if (rc == -1) {
    printf("Failed to bind socket to network interface\n");

    // Invalidate unusable socket
    close();
    return;
  }
  printf("Successfully bound socket to interface %d.\n",
         if_request.ifr_ifindex);

}

void SocketCAN::close() {
  printf("Waiting for receiver thread to terminate.\n");

  // terminate_receiver_thread = true;
  // while (receiver_thread_running) {
  //   sleep(1);
  // }

  if (!is_open())
    return;

  // Close the file descriptor for our socket
  ::close(sockfd);
  sockfd = -1;

  printf("CAN socket destroyed.\n");
}

bool SocketCAN::is_open() { return (sockfd != -1); }

void SocketCAN::transmit(can_frame_t *frame) {
  CANAdapter::transmit(frame);
  if (!is_open()) {
    printf("Unable to transmit: Socket not open\n");
    return;
  }

  write(sockfd, frame, sizeof(can_frame_t));
}

void SocketCAN::socketcan_receiver_thread() {

  // Holds the set of descriptors, that 'select' shall monitor
  fd_set descriptors;

  // Highest file descriptor in set
  int maxfd = this->sockfd;

  // How long 'select' shall wait before returning with timeout
  struct timeval timeout;

  // Buffer to store incoming frame
  can_frame_t rx_frame;

  // Set running flag
  // sock->receiver_thread_running = true;

  // Run until termination signal received
  // while (!sock->terminate_receiver_thread) {
    // Clear descriptor set
    FD_ZERO(&descriptors);
    // Add socket descriptor
    FD_SET(this->sockfd, &descriptors);
    //        printf("Added %d to monitored descriptors.\n", sock->sockfd);

    // Set timeout
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    // Wait until timeout or activity on any descriptor
    if (select(maxfd + 1, &descriptors, NULL, NULL, &timeout) == 1) {
      //            printf("Something happened.\n");
      int len = read(this->sockfd, &rx_frame, CAN_MTU);
      //            printf("Received %d bytes: Frame from 0x%0X, DLC=%d\n", len,
      //            rx_frame.can_id, rx_frame.can_dlc);

      if (len < 0)
        return;

      if (this->reception_handler != NULL) {
        this->reception_handler(&rx_frame, this->reception_handler_data);
      }

      if (this->parser != NULL) {
        //                printf("Invoking parser...\n");
        this->parser->parse_frame(&rx_frame);
      } else {
        //                printf("sock->parser is NULL.\n");
      }
    } else {
      //            printf("Received nothing.\n");
    }
  // }

  // printf("Receiver thread terminated.\n");

  // Thread terminates
  // sock->receiver_thread_running = false;

}


#endif
