
#include "mbed.h"
#include "TCPSocket.h"
#include "EthernetInterface.h"

EthernetInterface eth;
TCPSocket socket;

#define IP "192.168.1.177"
#define GATEWAY "192.168.1.1"
#define MASK "255.255.255.0"

int main()
{
  printf("Example network-socket TCP Server\r\n");
  // eth.disconnect();
  // int i=eth.set_network(IP,MASK,GATEWAY);
  // printf("set IP status: %i \r\n",i);
  int i = eth.connect();
  printf("connect status: %i \r\n", i);
  const char* ip = eth.get_ip_address();
  const char* mac = eth.get_mac_address();
  printf("IP address is: %s\n\r", ip ? ip : "No IP");
  printf("MAC address is: %s\n\r", mac ? mac : "No MAC");

  while (true)
  {
    socket.close();
    socket.open(&eth);

    socket.connect("93.175.29.245", 80);

    char sbuffer[] = "GET /checkPin.php?\r\n";
    int scount = socket.send(sbuffer, sizeof sbuffer);

    char rbuffer[64];
    int rcount = socket.recv(rbuffer, sizeof rbuffer);
    printf("recv %d [%.*s]\r\n", rcount, strstr(rbuffer, "\r\n") - rbuffer,
           rbuffer);
  }
}