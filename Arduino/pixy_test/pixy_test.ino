#include <SPI.h>
#include <Pixy.h>

Pixy pixy;
int data_x;
int data_y;

void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();
}

void loop() {
  static int i = 0;
  uint16_t blocks;
  char buf[32];

  blocks = pixy.getBlocks();
  
  if (blocks) {
    i++;
    if (i%50==0) {
      data_x = pixy.blocks[0].x;
      data_y = pixy.blocks[0].y;
      sprintf(buf, "x:%d ", data_x);
      Serial.print(buf);
      sprintf(buf, "y:%d \n", data_y);
      Serial.print(buf);
    }
    
  }

}
