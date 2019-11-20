#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_LSM9DS1.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK 3 //SCL
#define LSM9DS1_MOSI 2 //SDA

float L0 = 8.5;
float L1 = 7;
float L2 = 8.5;
float inVec[3];
float height = 11.5;
float plateRadius = 9.3;
float alpha[4] = {0, 0, 0};
float normVec[4];
float p1[4] = {0,0,0};
float p2[4] = {0,0,0};
float p3[4] = {0,0,0};
float crosskv[4] = {0,0,0};
float normSave;
float dotSave;
float radius;
float gamma;
float beta;

Servo A, B, C;
float s = 0.5; // smoothing coefficient. Set to 1 for no smoothing.

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
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  getUnitVec(a.acceleration.x, a.acceleration.y, a.acceleration.z, inVec);

  // Inverse kinematics
  normVec[0] = inVec[0]/(sqrt(inVec[0]*inVec[0]+inVec[1]*inVec[1]+inVec[2]*inVec[2]));
  normVec[1] = inVec[1]/(sqrt(inVec[0]*inVec[0]+inVec[1]*inVec[1]+inVec[2]*inVec[2]));
  normVec[2] = inVec[2]/(sqrt(inVec[0]*inVec[0]+inVec[1]*inVec[1]+inVec[2]*inVec[2]));

  p1[0] =    normVec[2];
  p1[2] = -1*normVec[0];
  normSave = sqrt(p1[0]*p1[0]+p1[1]*p1[1]+p1[2]*p1[2]);
  p1[0] = plateRadius*p1[0]/normSave;
  p1[1] = plateRadius*p1[1]/normSave;
  p1[2] = plateRadius*p1[2]/normSave;

  crosskv[0] = normVec[1]*p1[2] - normVec[2]*p1[1];
  crosskv[1] = normVec[2]*p1[0] - normVec[0]*p1[2];
  crosskv[2] = normVec[0]*p1[1] - normVec[1]*p1[0];
  dotSave = normVec[0]*p1[0] + normVec[1]*p1[1] + normVec[2]*p1[2];
  p2[0] = -0.5*p1[0] + crosskv[0]*0.866 + normVec[0]*dotSave*1.5;
  p2[1] = -0.5*p1[1] + crosskv[1]*0.866 + normVec[1]*dotSave*1.5;
  p2[2] = -0.5*p1[2] + crosskv[2]*0.866 + normVec[2]*dotSave*1.5;
  normSave = sqrt(p2[0]*p2[0]+p2[1]*p2[1]+p2[2]*p2[2]);
  p2[0] = plateRadius*p2[0]/normSave;
  p2[1] = plateRadius*p2[1]/normSave;
  p2[2] = plateRadius*p2[2]/normSave;

  p3[0] = -0.5*p1[0] + crosskv[0]*-0.866 + normVec[0]*dotSave*1.5;
  p3[1] = -0.5*p1[1] + crosskv[1]*-0.866 + normVec[1]*dotSave*1.5;
  p3[2] = -0.5*p1[2] + crosskv[2]*-0.866 + normVec[2]*dotSave*1.5;
  normSave = sqrt(p3[0]*p3[0]+p3[1]*p3[1]+p3[2]*p3[2]);
  p3[0] = plateRadius*p3[0]/normSave;
  p3[1] = plateRadius*p3[1]/normSave;
  p3[2] = plateRadius*p3[2]/normSave;

  p1[2] = height+p1[2];
  p2[2] = height+p2[2];
  p3[2] = height+p3[2];

  radius = sqrt((p1[0]-L0)*(p1[0]-L0) + p1[1]*p1[1] + p1[2]*p1[2]);
  gamma  = atan2(p1[2], sqrt((p1[0]-L0)*(p1[0]-L0)+(p1[1])*(p1[1])));
  beta   = acos((L1*L1+radius*radius-L2*L2)/(2*L1*radius));
  if ~isnan(gamma+beta) alpha[0] = s*(gamma + beta) + (1-s)*alpha[0];

  radius = sqrt((p2[0]+0.5*L0)*(p2[0]+0.5*L0)+(p2[1]-0.866*L0)*(p2[1]-0.866*L0)+(p2[2])*(p2[2]));
  gamma  = atan2(p2[2],sqrt((p2[0]+0.5*L0)*(p2[0]+0.5*L0)+(p2[1]-0.866*L0)*(p2[1]-0.866*L0)));
  beta   = acos((L1*L1+radius*radius-L2*L2)/(2*L1*radius));
  if ~isnan(gamma+beta) alpha[1] = s*(gamma + beta) + (1-s)*alpha[1];

  radius = sqrt((p3[0]+0.5*L0)*(p3[0]+0.5*L0)+(p3[1]+0.866*L0)*(p3[1]+0.866*L0)+(p3[2])*(p3[2]));
  gamma  = atan2(p3[2],sqrt((p3[0]+0.5*L0)*(p3[0]+0.5*L0)+(p3[1]+0.866*L0)*(p3[1]+0.866*L0)));
  beta   = acos((L1*L1+radius*radius-L2*L2)/(2*L1*radius));
  if ~isnan(gamma+beta) alpha[2] = s*(gamma + beta) + (1-s)*alpha[2];

  Serial.print(alpha[0]*180/3.1416); Serial.print(',');
  Serial.print(alpha[1]*180/3.1416); Serial.print(',');
  Serial.println(alpha[2]*180/3.1416);

  A.write(alpha[0]*180/3.1416);
  B.write(alpha[1]*180/3.1416);
  C.write(alpha[2]*180/3.1416);

}

void getUnitVec(float ax, float ay, float az, float* inVec)
{
  float roll  = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  inVec[0] = cos(pitch)*sin(roll);
  inVec[2] = cos(pitch)*cos(roll);
  inVec[1] = sin(pitch);

  //Serial.print(inVec[0]); Serial.print(',');
  //Serial.print(inVec[1]); Serial.print(',');
  //Serial.print(inVec[2]); Serial.print('\n');
}
