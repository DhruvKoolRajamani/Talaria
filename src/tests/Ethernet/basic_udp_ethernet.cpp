#include "EthernetInterface.h"
#include "lwip/inet.h"
#include "mbed.h"

// Network interface
EthernetInterface net;

// Time protocol implementation : Address: time.nist.gov UDPPort: 37

typedef struct
{
  uint32_t secs;  // Transmit Time-stamp seconds.
} ntp_packet;

int main()
{
  // Bring up the ethernet interface
  printf("Setting a static IP\n");

  // net.set_network("192.168.1.101","255.255.255.0","192.168.1.255");

  if (0 != net.connect())
  {
    printf("Error connecting\n");
    return -1;
  }

  // Show the network address
  const char* stm_ip = net.get_ip_address();
  printf("IP address is: %s\n", stm_ip ? stm_ip : "No IP");
  // Show the mac address
  const char* mac_addr = net.get_mac_address();
  printf("MAC address is: %s\n", mac_addr ? mac_addr : "No MAC");

  UDPSocket sock(&net);
  SocketAddress sockAddr;

  char out_buffer[] = "time";
  if (0 > sock.sendto("time.nist.gov", 37, out_buffer, sizeof(out_buffer)))
  {
    printf("Error sending data\n");
    return -1;
  }

  ntp_packet in_data;
  int n = sock.recvfrom(&sockAddr, &in_data, sizeof(ntp_packet));
  in_data.secs = ntohl(in_data.secs) - 2208988800;  // 1900-1970
  printf("Time Received %lu seconds since 1/01/1900 00:00 GMT\n", (uint32_t)in_data.secs);
  printf("Time = %s", ctime((const time_t*)&in_data.secs));

  printf("Time Server Address: %s Port: %d\n\r", sockAddr.get_ip_address(), sockAddr.get_port());

  // Close the socket and bring down the network interface
  sock.close();
  net.disconnect();
  return 0;
}