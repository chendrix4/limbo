/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#define MIN_BOUND 65
#define MAX_BOUND 120
#define NUM_CHARS 2

Servo A;
Servo B;
Servo C;

int x;

char id;

void setup() {
  pinMode(30, OUTPUT);
  A.attach(9);  // attaches the servo on pin 9 to the servo object
  B.attach(10);
  C.attach(11);
  Serial.begin(115200);
  delay(500);

  A.write(MAX_BOUND);
  B.write(MAX_BOUND);
  C.write(MAX_BOUND);
  
}

void loop() {

  // Get Servo ID
  if (Serial.available() > 0) {
    id = Serial.read();

    // Get Commanded Angle
    if (Serial.available() > 0) {
      x = Serial.parseInt();

      // Skip x's that are zero, do some error bounding
      if (x != 0) {
        x = min(max(x, MIN_BOUND), MAX_BOUND);

        // Write to servo
        switch (id) {
          case 'A':
            A.write(x);
            break;
          case 'B':
            B.write(x);
            break;
          case 'C':
            C.write(x);
            break;
        }
      }
      
    }
  }

}

