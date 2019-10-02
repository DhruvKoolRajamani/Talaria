#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include "EthernetInterface.h"
#include "lwip/inet.h"
#include "mbed.h"

class CommManager
{
private:
  EthernetInterface net;
  const char static_ip_[24];
  int port_;

public:
  CommManager(const char* static_ip, int port);
  bool establishConnection();
  bool sendData(const char* data);
  const char* receiveData();
};

#endif  // COMM_MANAGER_H