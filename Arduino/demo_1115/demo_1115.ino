#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_LSM9DS1.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK 3 // SCL
#define LSM9DS1_MOSI 2 // SDA
#define L0 8.5
#define L1 7
#define L2 8.5
#define HEIGHT 11.5
#define PLATE_RADIUS 9.3
#define MINBOUND 91
#define MAXBOUND 140

float alpha[4] = {0, 0, 0};
float inVec[4] = {0, 0, 1};
float p1[4] = {0,0,0};
float p2[4] = {0,0,0};
float p3[4] = {0,0,0};
float crosskv[4] = {0,0,0};
float normSave;
float dotSave;
float radius;
float gamma;
float beta;
float q;
float th = 0;

Servo A, B, C;
float s = 1; // smoothing coefficient. Set to 1 for no smoothing.

void setup() 
{
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("LSM9DS1 data read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  
  A.attach(9);  // attaches the servo on pin 9 to the servo object
  B.attach(10);
  C.attach(11);

  alpha[0] = A.read();
  alpha[1] = B.read();
  alpha[2] = C.read();

  // helper to just set the default scaling we want, see above!
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  
}

void loop() 
{

  // Get unit vector from accelerometer
  //sensors_event_t a, m, g, temp;
  //lsm.getEvent(&a, &m, &g, &temp);
  //getUnitVec(a.acceleration.x, a.acceleration.y, a.acceleration.z, inVec);

  th += 0.05;
  if (th > 2*PI) {
    th = 0;
  }
  inVec[0] = cos(th);
  inVec[1] = sin(th);
  inVec[2] = sqrt(99.0);

  Serial.print(inVec[0]); Serial.print(',');
  Serial.print(inVec[1]); Serial.print(',');
  Serial.print(inVec[2]); Serial.print('\n');

  // Inverse kinematics
  p1[0] =    inVec[2];
  p1[2] = -1*inVec[0];
  normSave = sqrt(p1[0]*p1[0]+p1[1]*p1[1]+p1[2]*p1[2]);
  p1[0] = PLATE_RADIUS*p1[0]/normSave;
  p1[1] = PLATE_RADIUS*p1[1]/normSave;
  p1[2] = PLATE_RADIUS*p1[2]/normSave;

  crosskv[0] = inVec[1]*p1[2] - inVec[2]*p1[1];
  crosskv[1] = inVec[2]*p1[0] - inVec[0]*p1[2];
  crosskv[2] = inVec[0]*p1[1] - inVec[1]*p1[0];
  dotSave = inVec[0]*p1[0] + inVec[1]*p1[1] + inVec[2]*p1[2];
  p2[0] = -0.5*p1[0] + crosskv[0]*0.866 + inVec[0]*dotSave*1.5;
  p2[1] = -0.5*p1[1] + crosskv[1]*0.866 + inVec[1]*dotSave*1.5;
  p2[2] = -0.5*p1[2] + crosskv[2]*0.866 + inVec[2]*dotSave*1.5;
  normSave = sqrt(p2[0]*p2[0]+p2[1]*p2[1]+p2[2]*p2[2]);
  p2[0] = PLATE_RADIUS*p2[0]/normSave;
  p2[1] = PLATE_RADIUS*p2[1]/normSave;
  p2[2] = PLATE_RADIUS*p2[2]/normSave;

  p3[0] = -0.5*p1[0] + crosskv[0]*-0.866 + inVec[0]*dotSave*1.5;
  p3[1] = -0.5*p1[1] + crosskv[1]*-0.866 + inVec[1]*dotSave*1.5;
  p3[2] = -0.5*p1[2] + crosskv[2]*-0.866 + inVec[2]*dotSave*1.5;
  normSave = sqrt(p3[0]*p3[0]+p3[1]*p3[1]+p3[2]*p3[2]);
  p3[0] = PLATE_RADIUS*p3[0]/normSave;
  p3[1] = PLATE_RADIUS*p3[1]/normSave;
  p3[2] = PLATE_RADIUS*p3[2]/normSave;

  p1[2] = HEIGHT+p1[2];
  p2[2] = HEIGHT+p2[2];
  p3[2] = HEIGHT+p3[2];

//  Serial.print("p1 = "); Serial.print(p1[0]); Serial.print(','); Serial.print(p1[1]); Serial.print(','); Serial.println(p1[2]);
//  Serial.print("p2 = "); Serial.print(p2[0]); Serial.print(','); Serial.print(p2[1]); Serial.print(','); Serial.println(p2[2]);
//  Serial.print("p3 = "); Serial.print(p3[0]); Serial.print(','); Serial.print(p3[1]); Serial.print(','); Serial.println(p3[2]);

  radius = sqrt((p1[0]-L0)*(p1[0]-L0) + p1[1]*p1[1] + p1[2]*p1[2]);
  gamma  = atan2(p1[2], sqrt((p1[0]-L0)*(p1[0]-L0)+(p1[1])*(p1[1])));
  q = (L1*L1+radius*radius-L2*L2)/(2*L1*radius);
  if (q < -1) {
    beta = PI;
  } else if (q > 1) {
    beta = 0;
  } else {
    beta = acos(q);
  }
  alpha[0] = max(MINBOUND, min(MAXBOUND, s*(gamma + beta)*180/PI)); // + (1-s)*alpha[0];

  radius = sqrt((p2[0]+0.5*L0)*(p2[0]+0.5*L0)+(p2[1]-0.866*L0)*(p2[1]-0.866*L0)+(p2[2])*(p2[2]));
  gamma  = atan2(p2[2],sqrt((p2[0]+0.5*L0)*(p2[0]+0.5*L0)+(p2[1]-0.866*L0)*(p2[1]-0.866*L0)));
  q = (L1*L1+radius*radius-L2*L2)/(2*L1*radius);
  if (q < -1) {
    beta = PI;
  } else if (q > 1) {
    beta = 0;
  } else {
    beta = acos(q);
  }
  alpha[1] = max(MINBOUND, min(MAXBOUND, s*(gamma + beta)*180/PI)); // + (1-s)*alpha[1];

  radius = sqrt((p3[0]+0.5*L0)*(p3[0]+0.5*L0)+(p3[1]+0.866*L0)*(p3[1]+0.866*L0)+(p3[2])*(p3[2]));
  gamma  = atan2(p3[2],sqrt((p3[0]+0.5*L0)*(p3[0]+0.5*L0)+(p3[1]+0.866*L0)*(p3[1]+0.866*L0)));
  q = (L1*L1+radius*radius-L2*L2)/(2*L1*radius);
  if (q < -1) {
    beta = PI;
  } else if (q > 1) {
    beta = 0;
  } else {
    beta = acos(q);
  }
  alpha[2] = max(MINBOUND, min(MAXBOUND, s*(gamma + beta)*180/PI)); // + (1-s)*alpha[2];

  Serial.print(alpha[0]); Serial.print(',');
  Serial.print(alpha[1]); Serial.print(',');
  Serial.println(alpha[2]);

  A.write(alpha[0]);
  B.write(alpha[1]);
  C.write(alpha[2]);

  delay(50);

}

void getUnitVec(float ax, float ay, float az, float* inVec)
{

//  Serial.print(ax); Serial.print(',');
//  Serial.print(ay); Serial.print(',');
//  Serial.print(az); Serial.print('\t');

  float roll  = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  inVec[0] = cos(pitch)*sin(roll);
  inVec[2] = cos(pitch)*cos(roll);
  inVec[1] = sin(pitch);

  //Serial.print(inVec[0]); Serial.print(',');
  //Serial.print(inVec[1]); Serial.print(',');
  //Serial.print(inVec[2]); Serial.print('\n');
}
