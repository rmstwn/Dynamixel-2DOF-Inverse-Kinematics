/* Dynamixel Basic Position Control Example
 
 Turns left the dynamixel , then turn right for one second,
 repeatedly.
 
                   Compatibility
 CM900                  O
 OpenCM9.04             O
 
                  Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          O      O      O        O      X
 OpenCM9.04     O      O      O        O      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */
/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP  

/* Dynamixel ID defines */
#define ID_NUM_JOINT_1 1
#define ID_NUM_JOINT_2 1

/* Minimum_Source*/
#include <math.h>

Dynamixel Dxl(DXL_BUS_SERIAL1);

//Inverse Kinematics Variable
float L1 = 165.0; //Long of link 1 (Joint 1 -> Joint 2) 
float L2 = 160.0; //Long of link 2 (Joint 2 -> toe)
float x;
float y;
double ServoAngle1;
double ServoAngle2;
float pi = M_PI;
double alpha;
double beta;
double a,b,c;
float Y,X;
float Y1,Y2,X1,X2;
float new_val1,new_val2;

void setup() {
  // put your setup code here, to run once:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  //jointMode() is to use position mode
  Dxl.jointMode(ID_NUM_JOINT_1); //Dxl Joint 1
  Dxl.jointMode(ID_NUM_JOINT_2); //Dxl Joint 2
}

void loop() {
  // put your main code here, to run repeatedly: 
  
  ServoAngle(0,325); //((x target),(y target))
  Dxl.goalPosition(ID_NUM_JOINT_1, new_val1); //ID 1 dynamixel moves to position ServoAngle1
  delay(1000);
  Dxl.goalPosition(ID_NUM_JOINT_2, new_val2);//ID 2 dynamixel moves to position ServoAngle2
  delay(1000);
}

void ServoAngle(float x,float y) {
  //inverse kinematics formula
  a = sq(x) + sq(y);
  b = sqrt( sq(x) + sq(y) );
  alpha = atan2(y,x);
  beta = acos( ( sq(b) + sq(L1) - sq(L2) ) / (2 * b * L1) );

  ServoAngle1 = ( alpha + beta ) * (180/pi); //angle for joint 1
  ServoAngle2 = (acos(((sq(x) + sq(y)) - sq(L1) - sq(L2)) / (2*L1*L2))) * (180/pi); //angle for joint 2 

  conversion1(ServoAngle1); //conversion angle to dxl value
  conversion2(ServoAngle2); //conversion angle to dxl value
}

void conversion1(float X){

  new_val1=0;

  Y1 = 3072; //dxl value when angle = 0
  Y2 = 1024; //dxl value when angle = 180
  X1 = 0; //angle value when leg is 0 degree
  X2 = 180; //angle value when leg is 180 degree
  new_val1 = ( (X - X1) / (X2 - X1) ) * (Y2 - Y1) + Y1; //conversion formula
}

void conversion2(float X){

  new_val2=0;

  Y1 = 2048; //dxl value when angle = 0
  Y2 = 1024; //dxl value when angle = 90
  X1 = 0; //angle value when leg is 0 degree
  X2 = 90; //angle value when leg is 90 degree
  new_val2 = ( (X - X1) / (X2 - X1) ) * (Y2 - Y1) + Y1; //conversion formula
}

