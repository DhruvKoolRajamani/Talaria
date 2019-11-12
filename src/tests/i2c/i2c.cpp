#include "mbed.h"

int main()
{
  I2C i2c(p9, p10);
  printf("Initiating Blind Scan\n");
  while (true)
  {
    char id[3];
    int ack = 0;
    char reg_addr[1] = { 10 };
    for (int i = 0; i < 255; i++)
    {
      printf("\nPINGING %d\n", i);
      ack = i2c.write(i, reg_addr, 1, false);
      if (ack == 0)
        printf("Write received %s\n", (ack == 0) ? "ACK" : "NACK");

      // ack = i2c.read(i, id, 3);
      // printf("Read received %s\n", (ack == 0) ? "ACK" : "NACK");

      printf("\n");
      printf("Data in buffer is: %s\n", id);
      wait_ms(10);
    }
    wait_ms(1000);
  }

  return 0;
}