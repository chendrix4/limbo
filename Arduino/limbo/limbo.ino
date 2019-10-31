/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#define MIN_BOUND 60
#define MAX_BOUND 120

Servo A;
Servo B;
Servo C;

int pos = 0;    // variable to store the servo position

void setup() {
  A.attach(9);  // attaches the servo on pin 9 to the servo object
  B.attach(10);
  C.attach(11);
  Serial.begin(115200);
  delay(500);
}

void loop() {
  for (int c = MIN_BOUND; c < MAX_BOUND; c++) {
    A.write(c);
    B.write(c);
    C.write(c);
    delay(20);
  }
  for (int c = MAX_BOUND; c > MIN_BOUND; c--) {
    A.write(c);
    B.write(c);
    C.write(c);
    delay(20);
  }
}

