#include "mbed.h"

int main()
{
  I2C i2c(PB_9, PB_8);
  printf("Initiating Blind Scan\n");
  while (true)
  {
    char id[1];
    int ack = 0;
    for (int i = 208; i < 209; i++)
    {
      printf("\nPINGING %d\n", i);
      ack = i2c.write(i, 0, 1, true);
      printf("Write received %s\n", (ack == 0) ? "ACK" : "NACK");

      ack = i2c.read(i, id, 1);
      printf("Read received %s\n", (ack == 0) ? "ACK" : "NACK");

      printf("\n");
      printf("Data in buffer is: %s\n", id);
      wait_ms(100);
    }
    wait_ms(1000);
  }

  return 0;
}