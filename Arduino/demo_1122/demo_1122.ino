#include <Servo.h>
#include <Pixy.h>
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
#define MINTHETA 91
#define MAXTHETA 140
#define MIN_X 60
#define MIN_Y 3
#define MAX_X 296
#define MAX_Y 197
#define CENTER_X ((MIN_X + MAX_X) >> 1)
#define CENTER_Y ((MIN_X + MAX_X) >> 1)

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

// Inverse Kinematics
uint8_t theta[4] = {0, 0, 0}; // B, A, C servo angles
float ax, ay, az;
float psi_x, psi_y, psi_z;  // roll, pitch, yaw of top plate (yaw is calculated from other two)
float O7x, O7y;
float O7j[4] = {0, 0, 0};
float Am, Bm, Cm; 
float alpha_1m;
float det;
float thN;

// servo calibration offsets
const int8_t offset[4] = {-5, 0, 0};

// input smoothing
const float s = 0.25;

// PID Control
const float Kp = 8 * PI / 180.0;
const float Ki = 0;
const float Kd = 0.02; //0.02;
float cumError_x = 0;
float cumError_y = 0;
float lastError_x = 0;
float lastError_y = 0;

// Camera control
Pixy pixy;
uint16_t blocks;
Servo A, B, C;

// 
bool manual_control = false;
int8_t notFound = 0;

void setup() {

  // Open serial port
  Serial.begin(9600);

  // Open communications with IMU
  if (!lsm.begin()) {
    Serial.println("Unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }

  // Setup servo motors
  A.attach(9);
  B.attach(10);
  C.attach(11);

  // helper to just set the default scaling we want, see above!
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);

  pixy.init();

  //initRobot();
  
}

void loop() {

  float ball_x, ball_y;

  // Read sensor
  if (manual_control) {
    
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    // Apply smoothing
    ax = s*a.acceleration.x + (1-s)*ax;
    ay = s*a.acceleration.y + (1-s)*ay;
    az = s*a.acceleration.z + (1-s)*az;
  
    // Determine the commanded roll and pitch
    psi_x = atan2(ay, az);
    psi_y = atan2(-ax, sqrt(ay*ay + az*az));
  
  } else {
    blocks = pixy.getBlocks();
    if (blocks) {
      notFound = false;
      ball_x = pixy.blocks[0].x;
      ball_y = pixy.blocks[0].y;

      // Check if ball is on the plate
      if (sqrt(pow(ball_x - CENTER_X,2) + pow(ball_y-CENTER_Y,2)) < ((MAX_X - MIN_X) << 1)) {
        limboPID_Center(ball_x, ball_y, CENTER_X, CENTER_Y);
      }
      
      //psi_x = 0.000025*pow(ball_x - center_x, 3);
      //psi_y = 0.000025*pow(ball_y - center_y, 3);
    }
    else {
      notFound = true;
    }
  }

  if ((limboIK(psi_x, psi_y, theta)) && (notFound == 0)) {
    A.write(theta[1]);
    B.write(theta[0]);
    C.write(theta[2]);
  }
  
}

// initRobot
// ------------------------------------------------------------------------------
// Step through the robot's range as a quick test of mobility
// ------------------------------------------------------------------------------
void initRobot() {
  
  // Quick bounding check for startup
  int8_t start = (MAXTHETA+MINTHETA) >> 1;
  A.write(start + offset[1]); theta[1] = A.read();
  B.write(start + offset[0]); theta[0] = B.read();
  C.write(start + offset[2]); theta[2] = C.read();

  for (uint8_t d = MINTHETA; d < MAXTHETA; d++) {
    A.write(d + offset[1]);
    delay(20);
  }
  A.write(theta[1] + offset[1]);
  
  for (uint8_t d = MINTHETA; d < MAXTHETA; d++) {
    B.write(d + offset[0]);
    delay(20);
  }
  B.write(theta[0] + offset[0]);
  
  for (uint8_t d = MINTHETA; d < MAXTHETA; d++) {
    C.write(d + offset[2]);
    delay(20);
  }
  C.write(theta[2] + offset[2]);

}

// limboIK
// ------------------------------------------------------------------------------
// Calculate the inverse kinematics (servo angles given top plate configuration)
// Roll and pitch are determined from the sensor or the camera in the above block
// The height is a constant set way up at the top (O7z)
// 0 degrees is when the servo arm is pointing directly away from center
// 180 degrees is when the servo arm is pointing directly in
// ------------------------------------------------------------------------------
// RETURNS boolean indicating whether or not the commanded configuration is valid
// ------------------------------------------------------------------------------
bool limboIK(float psi_x, float psi_y, uint8_t* theta) {

  // Calculate yaw, plate (x,y) coord
  psi_z = atan2(-sin(psi_x)*sin(psi_y), cos(psi_x) + cos(psi_y));
  O7x = (P*(Ux - Vy)) / 2.0;
  O7y = -Uy*P;

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
      return false;

    // Possible answer -- calculate the angle in degrees (take value that yields elbows in)
    } else {
      thN = -2*atan2(-Bm - sqrt(det), Cm - Am)*180/PI + offset[i];
      //thP = -2*atan2(-Bm + sqrt(det), Cm - Am); // This gives elbows out

      // If outside bounds, it's a bad config
      if ((thN < MINTHETA) || (thN > MAXTHETA)) {
        return false;
      }
      
      theta[i] = thN;
      
    }
    
  }

  return true;

}

// limboPID_Center
// ------------------------------------------------------------------------------
// Calculate the configuration (roll/psi_x, pitch/psi_y) to keep ball centered
// ------------------------------------------------------------------------------
void limboPID_Center(int16_t ball_x, int16_t ball_y, int16_t target_x, int16_t target_y) {
  float error_x, error_y;

  error_x = ball_x - target_x;
  error_y = ball_y - target_y;

  float p_stuff_x = error_x / ((MAX_X - MIN_X) >> 1);
  float p_stuff_y = -error_y / ((MAX_Y - MIN_Y) >> 1);

  float d_stuff_x = (error_x - lastError_x);
  float d_stuff_y = -(error_y - lastError_y);

  cumError_x += error_x*Ki;
  cumError_y += -error_y*Ki;
  Serial.println(cumError_x);

  psi_y = Kp*p_stuff_x + cumError_x + Kd*d_stuff_x;
  psi_x = Kp*p_stuff_y + cumError_y + Kd*d_stuff_y;

  // update error
  lastError_x = error_x;
  lastError_y = error_y;
}

// limboPID_Circle
// ------------------------------------------------------------------------------
// Calculate the configuration (roll/psi_x, pitch/psi_y) to keep ball moving in a
// clockwise circle of radius r
// ------------------------------------------------------------------------------
void limboPID_Circle(int16_t ball_x, int16_t ball_y, float r) {
  
}
