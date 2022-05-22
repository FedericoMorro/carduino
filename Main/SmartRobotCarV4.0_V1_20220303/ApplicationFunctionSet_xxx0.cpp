/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2021-01-05 09:30:14
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */
#include <avr/wdt.h>
//#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"
<<<<<<< HEAD
#include <IRremote.hpp>
=======
#include <IRremote.h>
>>>>>>> 6555265 ((wrong) traffic light implementation)

#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

<<<<<<< HEAD
#define IR_RECEIVE_PIN 9
#define PIN_RBGLED 4


static bool is_moving = 0;

=======

  
static IRrecv irrecv(9);
static decode_results results;
>>>>>>> 6555265 ((wrong) traffic light implementation)

ApplicationFunctionSet Application_FunctionSet;

/*Hardware device object list*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;

DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;
void delay_xxx(uint16_t _ms)
{

  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}

static void my_delay(uint16_t _ms){
  uint16_t end, start, delayed;

  start = millis();
  Application_FunctionSet.ApplicationFunctionSet_StopWhiteLine(); 
  end = millis();

  delayed = end-start;

  if(delayed > _ms){
    return;
  }
  

  wdt_reset();
  for (unsigned long i = 0; i < _ms/delayed; i++)
  {
    Application_FunctionSet.ApplicationFunctionSet_StopWhiteLine();
    delay(1);
  }
}


/*Movement Direction Control List*/
enum SmartRobotCarMotionControl
{
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it        //(9)
};               //direction方向:（1）、（2）、 （3）、（4）、（5）、（6）

/*Mode Control List*/
enum SmartRobotCarFunctionalModel
{
  Standby_mode,           /*Standby Mode*/
  TraceBased_mode,        /*Line Tracking Mode*/
  ObstacleAvoidance_mode, /*Obstacle Avoidance Mode*/
  Follow_mode,            /*Following Mode*/
  Rocker_mode,            /*Rocker Control Mode*/
  CMD_inspect,
  CMD_Programming_mode,                   /*Programming Mode*/
  CMD_ClearAllFunctions_Standby_mode,     /*Clear All Functions And Enter Standby Mode*/
  CMD_ClearAllFunctions_Programming_mode, /*Clear All Functions And Enter Programming Mode*/
  CMD_MotorControl,                       /*Motor Control Mode*/
  CMD_CarControl_TimeLimit,               /*Car Movement Direction Control With Time Limit*/
  CMD_CarControl_NoTimeLimit,             /*Car Movement Direction Control Without Time Limit*/
  CMD_MotorControl_Speed,                 /*Motor Speed Control*/
  CMD_ServoControl,                       /*Servo Motor Control*/
  CMD_LightingControl_TimeLimit,          /*RGB Lighting Control With Time Limit*/
  CMD_LightingControl_NoTimeLimit,        /*RGB Lighting Control Without Time Limit*/

};

/*Application Management list*/
struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx Application_SmartRobotCarxxx0;

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  //bool res_error = true;
  Serial.begin(9600);
  irrecv.enableIRIn();
  AppVoltage.DeviceDriverSet_Voltage_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppServo.DeviceDriverSet_Servo_Init(90);
  AppKey.DeviceDriverSet_Key_Init();
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
  AppIRrecv.DeviceDriverSet_IRrecv_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  //res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  // while (Serial.read() >= 0)
  // {
  //   /*Clear serial port buffer...*/
  // }
  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
}

/*ITR20001 Check if the car leaves the ground*/
static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void)
{
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() > Application_FunctionSet.TrackingDetection_V)
  {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  }
  else
  {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}

/*
  Straight line movement control：For dual-drive motors, due to frequent motor coefficient deviations and many external interference factors, 
  it is difficult for the car to achieve relative Straight line movement. For this reason, the feedback of the yaw control loop is added.
  direction：only forward/backward
  directionRecord：Used to update the direction and position data (Yaw value) when entering the function for the first time.
  speed：the speed range is 0~255
  Kp：Position error proportional constant（The feedback of improving location resuming status，will be modified according to different mode），improve damping control.
  UpperLimit：Maximum output upper limit control
*/
//static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
//{
//  static float Yaw; //Yaw
//  static float yaw_So = 0;
//  static uint8_t en = 110;
//  static unsigned long is_time;
//  if (en != directionRecord || millis() - is_time > 10)
//  {
//    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
//                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
//    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
//    is_time = millis();
//  }
//  //if (en != directionRecord)
//  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
//  {
//    en = directionRecord;
//    yaw_So = Yaw;
//  }
//  //Add proportional constant Kp to increase rebound effect
//  int R = (Yaw - yaw_So) * Kp + speed;
//  if (R > UpperLimit)
//  {
//    R = UpperLimit;
//  }
//  else if (R < 10)
//  {
//    R = 10;
//  }
//  int L = (yaw_So - Yaw) * Kp + speed;
//  if (L > UpperLimit)
//  {
//    L = UpperLimit;
//  }
//  else if (L < 10)
//  {
//    L = 10;
//  }
//  if (direction == Forward) //Forward
//  {
//    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
//                                           /*direction_B*/ direction_just, /*speed_B*/ L, /*controlED*/ control_enable);
//  }
//  else if (direction == Backward) //Backward
//  {
//    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
//                                           /*direction_B*/ direction_back, /*speed_B*/ R, /*controlED*/ control_enable);
//  }
//}

/*
  Movement Direction Control:
  Input parameters:     1# direction:Forward（1）、Backward（2）、 Left（3）、Right（4）、LeftForward（5）、LeftBackward（6）、RightForward（7）RightBackward（8）
                        2# speed(0--255)
*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;
<<<<<<< HEAD

  if(speed > 0){
    is_moving = true;
  }else{
    is_moving = false;
  }

  //Control mode that requires straight line movement adjustment（Car will has movement offset easily in the below mode，the movement cannot achieve the effect of a relatively straight direction
  //so it needs to add control adjustment）
  /*
  switch (Application_SmartRobotCarxxx0.Functional_Mode)
  {
  case Rocker_mode:
    Kp = 10;
    UpperLimit = 255;
    break;
  case ObstacleAvoidance_mode:
    Kp = 2;
    UpperLimit = 180;
    break;
  case Follow_mode:
    Kp = 2;
    UpperLimit = 180;
    break;
  case CMD_CarControl_TimeLimit:
    Kp = 2;
    UpperLimit = 180;
    break;
  case CMD_CarControl_NoTimeLimit:
    Kp = 2;
    UpperLimit = 180;
    break;
  default:
    Kp = 10;
    UpperLimit = 255;
    break;
  }
  */
=======
>>>>>>> 195e261 (Removed useless parts)

  Kp = 2;
  UpperLimit = 180;

  switch (direction)
  {
  //case /* constant-expression */ Forward:
  //  //When moving forward, enter the direction and position approach control loop processing
  //  ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
  //  directionRecord = 1;
  //
  //  break;
  //case /* constant-expression */ Backward:
  //  //When moving backward, enter the direction and position approach control loop processing
  //  ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
  //  directionRecord = 2;
  //
  //  break;
  case Forward:
    directionRecord = 1;
    AppMotor.DeviceDriverSet_Motor_control(direction_just, speed,
                                           direction_just, speed, control_enable);
    break;

  case Backward:
    directionRecord = 2;
    AppMotor.DeviceDriverSet_Motor_control(direction_back, speed,
                                           direction_back, speed, control_enable);
    break;
  case /* constant-expression */ Left:
    /* code */
    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Right:
    /* code */
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftForward:
    /* code */
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftBackward:
    /* code */
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightForward:
    /* code */
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightBackward:
    /* code */
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ stop_it:
    /* code */
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control

    break;
  default:
    directionRecord = 10;
    break;
  }
}

/*
  Obstacle Avoidance Mode
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void)
{
  uint16_t get_Distance;
  static uint8_t searchObstCnt = 0;
  uint8_t speed = 50;
  uint8_t dist = 25;
  uint8_t reduced_dist = 15;
  uint16_t dly = 600;
  uint8_t max_dist, max_i;
  uint8_t min_dist, min_i;
  uint8_t dist_array[] = {0,0,0,0,0};
  uint8_t cnt;
  static bool set = false;

  if (set == false) {
    AppServo.DeviceDriverSet_Servo_control(90);
    set = true;
  }

  min_i = -1;
  min_dist = 50000;
  if (searchObstCnt == 12) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    for (uint8_t i = 0; i <= 4; i++) {
      if (i == 0 || i == 2) {
        AppServo.DeviceDriverSet_Servo_control(90);
      } else if (i == 1) {
        AppServo.DeviceDriverSet_Servo_control(0);
      } else {
        AppServo.DeviceDriverSet_Servo_control(180);
      }
      AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);
      if (get_Distance < min_dist) {
        min_dist = get_Distance;
        min_i = i;
      }
    }
    searchObstCnt = 0;
    set = false;
  }
  searchObstCnt++;


  /*
  if (Car_LeaveTheGround == false)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    return;
  }
  */

  Application_FunctionSet.ApplicationFunctionSet_StopWhiteLine();
  delay_xxx(10);
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);
  delay_xxx(10);

  if(get_Distance < dist || min_dist < reduced_dist)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

    if (min_i == 1 || min_i == 3)
      dly = dly / 2;

    max_dist = 0;
    max_i = 0;
    
    for (uint8_t i = 0; i <= 4; i++) // Omnidirectional detection of obstacle avoidance status
    {
      AppServo.DeviceDriverSet_Servo_control(45 * i /*Position_angle*/);
      delay_xxx(5);
      AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);
      delay_xxx(10);
      
      dist_array[i] = get_Distance;
      if (get_Distance > max_dist) {
        max_dist = get_Distance;
        max_i = i;
      }
      set = false;
    }

    cnt = 0;
    for (uint8_t i=0; i<5; i++) {
      if (dist_array[i] < dist + 5) {
        cnt++;
      }
    }

    if (cnt < 3) {
      switch (max_i)
      {
      // in the manouvre operations we shall not check for the line
      case 0:
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 2* speed);
        delay_xxx(3 * dly / 2);
        break;
      case 1:
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 2* speed);
        delay_xxx(dly);
        break;
      case 2:
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, speed);
        delay_xxx(dly);
        break;
      case 3:
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 2* speed);
        delay_xxx(dly);
        break;
      case 4:
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 2 * speed);
        delay_xxx(3 * dly / 2);
        break;
      }
      //first_is = true;
    } else {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 2*speed);
      delay_xxx(2 * dly);
    }
    
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  }
  else //if (function_xxx(get_Distance, 20, 50))
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, speed);
  }
  Application_FunctionSet.ApplicationFunctionSet_StopWhiteLine();
}


void ApplicationFunctionSet::ApplicationFunctionSet_Servo(uint8_t Set_Servo)
{
  static int z_angle = 9;
  static int y_angle = 9;
  uint8_t temp_Set_Servo = Set_Servo; 

  switch (temp_Set_Servo)
  {
  case 1 ... 2:
  {
    if (1 == temp_Set_Servo)
    {
      y_angle -= 1;
    }
    else if (2 == temp_Set_Servo)
    {
      y_angle += 1;
    }
    if (y_angle <= 3) //minimum control
    {
      y_angle = 3;
    }
    if (y_angle >= 11) //maximum control
    {
      y_angle = 11;
    }
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--y*/ 2, /*unsigned int Position_angle*/ y_angle);
  }
  break;

  case 3 ... 4:
  {
    if (3 == temp_Set_Servo)
    {
      z_angle += 1;
    }
    else if (4 == temp_Set_Servo)
    {
      z_angle -= 1;
    }

    if (z_angle <= 1) //minimum control
    {
      z_angle = 1;
    }
    if (z_angle >= 17) //maximum control
    {
      z_angle = 17;
    }
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ z_angle);
  }
  break;
  case 5:
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--y*/ 2, /*unsigned int Position_angle*/ 9);
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 9);
    break;
  default:
    break;
  }
}



void ApplicationFunctionSet::ApplicationFunctionSet_StopWhiteLine () {
  uint8_t lvl = 60;
  int L, M, R;
<<<<<<< HEAD
<<<<<<< HEAD

  IRdata data;

  // bool is_moving = bitRead(PIN_Motor_AIN_1, 1) | bitRead(PIN_Motor_BIN_1, 1) | bitRead(PIN_Motor_PWMA, 1) | bitRead(PIN_Motor_PWMB, 1);
  // is_moving globale

=======
  bool is_moving = bitRead(PIN_Motor_AIN_1, 1) | bitRead(PIN_Motor_BIN_1, 1) | bitRead(PIN_Motor_PWMA, 1) | bitRead(PIN_Motor_PWMB, 1);
=======
>>>>>>> cb50f20 (Changes to detect white stop line)
  
>>>>>>> 195e261 (Removed useless parts)
  L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
  M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
  R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();

<<<<<<< HEAD
<<<<<<< HEAD

  Serial.println("Fotocellula:");
  Serial.println(L);
  Serial.println(M);
  Serial.println(R);

<<<<<<< HEAD
  if (L < lvl && M < lvl && R < lvl) {

    if (IrReceiver.decode()) {
        Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
        IrReceiver.printIRResultShort(&Serial); // optional use new print version
        data = IrReceiver.decodedIRData();

        while(data.address == 0xCODE && data.command == 0xDEAD){
          ApplicationFunctionSet_SmartRobotCarMotionControl (stop_it, 0);
          data = IrReceiver.decodedIRData();
        }

        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
        Serial.println("Riparto");
        delay_xxx(1000);
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 50);


        IrReceiver.resume(); // Enable receiving of the next value
    }else{
        ApplicationFunctionSet_SmartRobotCarMotionControl (stop_it, 0);
        Serial.println("Mi sono fermato");
        delay_xxx(5000);
        Serial.println("Finito il primo delay");
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
        Serial.println("Riparto");
        delay_xxx(1000);
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 50);
        Serial.println("Velocità normale");
    }
  }
}
=======
  if (is_moving && L < lvl && M < lvl && R < lvl) {
=======
  if (L < lvl || M < lvl || R < lvl) {
>>>>>>> cb50f20 (Changes to detect white stop line)
=======
  
  if (L < lvl || M < lvl || R < lvl) {
    if(irrecv.decode(&results)) {
      while(results.value == 57005) 
      {
        Serial.print("The strretlight is RED");
        Serial.println(results.value);
        ApplicationFunctionSet_SmartRobotCarMotionControl (stop_it, 0);
        irrecv.resume();
        irrecv.decode(&results);
        irrecv.resume();
      }

      Serial.println(results.value);

    } else {
>>>>>>> 6555265 ((wrong) traffic light implementation)
      ApplicationFunctionSet_SmartRobotCarMotionControl (stop_it, 0);
      delay_xxx(2000);
    }

    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 50);
      delay_xxx(1000);
      ApplicationFunctionSet_SmartRobotCarMotionControl (Forward, 50);
  }
}
<<<<<<< HEAD
>>>>>>> 195e261 (Removed useless parts)
=======
>>>>>>> 4ea5616 (v0.3 - Finally functioning maybe)
