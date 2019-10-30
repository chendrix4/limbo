/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#define A_MIN 
#define B_MIN
#define C_MIN 10

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

int c;

void loop() {
  c = analogRead(A0);
  c = map(c, 0, 1023, 0, 179);
  Serial.println(c);
  A.write(c);
  delay(15);
}

