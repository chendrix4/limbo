#include <math.h>
#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

#include <Servo.h>
#define MIN_BOUND 65
#define MAX_BOUND 120

#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_LSM9DS1.h>
#include <Wire.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK 3 //SCL
#define LSM9DS1_MOSI 2 //SDA

//setting variables
//input from accel: setting 0
//input from pixy: setting 1
int setting = 0;

//pid variables
float Kp = 1;
float Ki = 0;
float Kd = 0;
float target_x;
float target_y;
float error_x;
float error_y;
float currentTime;
float elapsedTime;
float cumError_x;
float cumError_y;
float rateError_x;
float rateError_y;
float output_x;
float output_y;
float previousTime = 0;
float lastError_x = 0;
float lastError_y = 0;
  
//invkinematic variables
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

//serial variables
char buf[32];
char str_temp1[15];
char str_temp2[15];
char str_temp3[15];

//pixy variables
static int count = 0;
uint16_t blocks;
int dataX;
int dataY;
float pos_x;
float pos_y;

//Servo variables
Servo A, B, C;
float s = 0.5; // smoothing coefficient. Set to 1 for no smoothing.

void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();

    // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops doopsy ... unable to initialize the LSM9DS1. Check your wiring!");
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

void loop() {
  if (setting == 1) {
    
    //PIXY CAMERA CODE (outputs x and y)
  
    blocks = pixy.getBlocks();
    
    if (blocks)
    {
      count++;
      if (count%25==0)
      {
        dataX = pixy.blocks[0].x;
        dataY = pixy.blocks[0].y;
        sprintf(buf, "x:%d ", dataX);
        Serial.print(buf);
        sprintf(buf, "y:%d \n", dataY);
        Serial.print(buf);
      }
    }
  
    //add data to position calc
  
    pos_x = -1;
    pos_y = -1;
  
    //DETERMINE TARGET LOCATION
    //default:(0,0)
    
    target_x = 0;
    target_y = 0;
  
    //PID CODE (inputs x and y and outputs normal vector and height)
  
    error_x = target_x - pos_x;
    error_y = target_y - pos_y;
    
    currentTime = millis();
    elapsedTime = currentTime - previousTime;
    if(elapsedTime==0){
      elapsedTime = 1;
    }
   
    dtostrf(error_x, 5, 3, str_temp1);
    dtostrf(error_y, 5, 3, str_temp2);
    sprintf(buf, "errorX:%s errorY:%s\n", str_temp1, str_temp2);
    Serial.print(buf);
  
    
    cumError_x += error_x * elapsedTime / 1000;
    cumError_y += error_y * elapsedTime / 1000;
    rateError_x = (error_x - lastError_x)/(elapsedTime/1000);
    rateError_y = (error_y - lastError_y)/(elapsedTime/1000);
  
    output_x = Kp*error_x + Ki*cumError_x + Kd * rateError_x;
    output_y = Kp*error_y + Ki*cumError_y + Kd * rateError_y;
  
    inVec[0] = output_x/plateRadius;
    inVec[1] = output_y/plateRadius;
    inVec[2] = plateRadius;
  
    lastError_x = error_x;
    lastError_y = error_y;
    previousTime = currentTime;

  } else if (setting == 0) {
      // Get unit vector from accelerometer
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    getUnitVec(a.acceleration.x, a.acceleration.y, a.acceleration.z, inVec);
  }

  //INVKINEMATIC (inputs normal vector and height and outputs alpha values)

  normVec[0] = inVec[0]/(sqrt(inVec[0]*inVec[0]+inVec[1]*inVec[1]+inVec[2]*inVec[2]));
  normVec[1] = inVec[1]/(sqrt(inVec[0]*inVec[0]+inVec[1]*inVec[1]+inVec[2]*inVec[2]));
  normVec[2] = inVec[2]/(sqrt(inVec[0]*inVec[0]+inVec[1]*inVec[1]+inVec[2]*inVec[2]));

  p1[0] = normVec[2];
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
  if (~isnan(gamma+beta)) alpha[0] = s*(gamma + beta) + (1-s)*alpha[0];

  radius = sqrt((p2[0]+0.5*L0)*(p2[0]+0.5*L0)+(p2[1]-0.866*L0)*(p2[1]-0.866*L0)+(p2[2])*(p2[2]));
  gamma  = atan2(p2[2],sqrt((p2[0]+0.5*L0)*(p2[0]+0.5*L0)+(p2[1]-0.866*L0)*(p2[1]-0.866*L0)));
  beta   = acos((L1*L1+radius*radius-L2*L2)/(2*L1*radius));
  if (~isnan(gamma+beta)) alpha[1] = s*(gamma + beta) + (1-s)*alpha[1];

  radius = sqrt((p3[0]+0.5*L0)*(p3[0]+0.5*L0)+(p3[1]+0.866*L0)*(p3[1]+0.866*L0)+(p3[2])*(p3[2]));
  gamma  = atan2(p3[2],sqrt((p3[0]+0.5*L0)*(p3[0]+0.5*L0)+(p3[1]+0.866*L0)*(p3[1]+0.866*L0)));
  beta   = acos((L1*L1+radius*radius-L2*L2)/(2*L1*radius));
  if (~isnan(gamma+beta)) alpha[2] = s*(gamma + beta) + (1-s)*alpha[2];
  
  dtostrf(normVec[0], 5, 3, str_temp1);
  dtostrf(normVec[1], 5, 3, str_temp2);
  dtostrf(normVec[2], 5, 3, str_temp3);
  sprintf(buf, "nx:%s ny:%s nz:%s\n", str_temp1, str_temp2, str_temp3);
  Serial.print(buf);

  Serial.print(alpha[0]*180/3.1416); Serial.print(',');
  Serial.print(alpha[1]*180/3.1416); Serial.print(',');
  Serial.println(alpha[2]*180/3.1416);

  A.write(alpha[0]*180/3.1416);
  B.write(alpha[1]*180/3.1416);
  C.write(alpha[2]*180/3.1416);

  //OUTPUT alpha to servos
  
  delay(1000);
}

void getUnitVec(float ax, float ay, float az, float* inVec) {
  float roll  = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  inVec[0] = cos(pitch)*sin(roll);
  inVec[2] = cos(pitch)*cos(roll);
  inVec[1] = sin(pitch);

  //Serial.print(inVec[0]); Serial.print(',');
  //Serial.print(inVec[1]); Serial.print(',');
  //Serial.print(inVec[2]); Serial.print('\n');
}
