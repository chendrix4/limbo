#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_LSM9DS1.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Constants
#define LSM9DS1_SCK 3 // SCL
#define LSM9DS1_MOSI 2 // SDA
#define R 8.5
#define L1 7
#define L2 8.5
#define O7z 13
#define P 9.3
#define MINBOUND 91
#define MAXBOUND 140

// Rotation matrix*-
#define Ux (  cos(psi_y)*cos(psi_z) )
#define Uy (  cos(psi_z)*sin(psi_x)*sin(psi_y) + cos(psi_x)*sin(psi_z) )
#define Uz (  sin(psi_x)*sin(psi_z) - cos(psi_x)*cos(psi_z)*sin(psi_y) )
#define Vx ( -cos(psi_y)*sin(psi_z) )
#define Vy (  cos(psi_x)*cos(psi_z) - sin(psi_x)*sin(psi_y)*sin(psi_z) )
#define Vz (  sin(psi_x)*cos(psi_z) + cos(psi_x)*sin(psi_y)*sin(psi_z) )

// Other niceties
#define C_A1m cos(alpha_1m)
#define S_A1m sin(alpha_1m)

int theta[4] = {0, 0, 0}; // B, A, C servo angles
float ax, ay, az;
float psi_x, psi_y, psi_z;  // roll, pitch, yaw of top plate (yaw is calculated from other two)
float O7x, O7y;
float O7j[4] = {0, 0, 0};
float Am, Bm, Cm; 
float alpha_1m;
float det;
float thN;
bool validConfig;

// servo calibration offsets
const int8_t offset[4] = {-5, 0, 0};

// smoothing
const float s = 0.25;


Servo A, B, C;

void setup() {

  // Open serial port
  Serial.begin(115200);

  // Open communications with IMU
  if (!lsm.begin()) {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }

  // Setup servo motors
  A.attach(9);
  B.attach(10);
  C.attach(11);

  // helper to just set the default scaling we want, see above!
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);

  A.write(115 + offset[1]); theta[1] = A.read();
  B.write(115 + offset[0]); theta[0] = B.read();
  C.write(115 + offset[2]); theta[2] = C.read();

  delay(1000);
  
  for (uint8_t d = MINBOUND; d < MAXBOUND; d++) {
    A.write(d + offset[1]);
    delay(20);
  }
  A.write(theta[1] + offset[1]);
  
  for (uint8_t d = MINBOUND; d < MAXBOUND; d++) {
    B.write(d + offset[0]);
    delay(20);
  }
  B.write(theta[0] + offset[0]);
  
  for (uint8_t d = MINBOUND; d < MAXBOUND; d++) {
    C.write(d + offset[2]);
    delay(20);
  }
  C.write(theta[2] + offset[2]);

}

void loop() {

  // Reset configuration error flag
  validConfig = true;

  // Read sensor
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  ax = s*a.acceleration.x + (1-s)*ax;
  ay = s*a.acceleration.y + (1-s)*ay;
  az = s*a.acceleration.z + (1-s)*az;

  // Determine the commanded roll and pitch
  psi_x = atan2(ay, az);
  psi_y = atan2(-ax, sqrt(ay*ay + az*az));

  //Serial.print(psi_x); Serial.print('\t'); Serial.println(psi_z);

  // Calculate yaw, plate (x,y) coord
  psi_z = atan2(-sin(psi_x)*sin(psi_y), cos(psi_x) + cos(psi_y));
  O7x = (P*(Ux - Vy)) / 2.0;
  O7y = -Uy*P;

  //Serial.print(O7x); Serial.print('\t'); Serial.println(O7y);

  for (uint8_t i=0 ; i < 3; i++) {

    // Motor angle on base (-120, 0, 120deg) in rads
    alpha_1m = 120*(i-1)*PI/180.0;

    // Point where this motor arm is attached to plate
    O7j[0] = O7x + P*(Ux*C_A1m + Vx*S_A1m);
    O7j[1] = O7y + P*(Uy*C_A1m + Vy*S_A1m);
    O7j[2] = O7z + P*(Uz*C_A1m + Vz*S_A1m);

    // Some nice coefficients
    Am = 2*L1*C_A1m*(R*C_A1m - O7j[0]);
    Bm = 2*L1*O7j[2]*(C_A1m*C_A1m);
    Cm = O7j[0]*O7j[0] - 2*R*O7j[0]*C_A1m + C_A1m*C_A1m*(R*R + L1*L1 - L2*L2 + O7j[2]*O7j[2]);

    // Calculate the angle
    det = Am*Am + Bm*Bm - Cm*Cm;

    // Imaginary answer -- set invalid configuration flag and read the next joint
    if (det < 0) {
      validConfig = false;
      break;

    // Possible answer -- calculate the angle in degrees (take value that yields elbows in)
    } else {
      thN = -2*atan2(-Bm - sqrt(det), Cm - Am)*180/PI + offset[i];
      //thP = -2*atan2(-Bm + sqrt(det), Cm - Am);

      // If outside bounds, it's a bad config
      if ((thN < MINBOUND) || (thN > MAXBOUND)) {
        validConfig = false;
        break;
      }
      
      theta[i] = thN;
      
    }
    
  }

  Serial.print(validConfig); Serial.print('\t');
  Serial.print(theta[1]); Serial.print('\t');
  Serial.print(theta[0]); Serial.print('\t');
  Serial.println(theta[2]);

  if (validConfig) {
    A.write(theta[1]);
    B.write(theta[0]);
    C.write(theta[2]);
  }

}
