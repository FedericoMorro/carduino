/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-18 14:14:35
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"
#include <IRremote.h>

#define IR_RECEIVE_PIN 9
#define PIN_RBGLED 4




void setup()
{
  // put your setup code here, to run once:
  Application_FunctionSet.ApplicationFunctionSet_Init();
<<<<<<< HEAD
<<<<<<< HEAD
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
<<<<<<< HEAD
  wdt_enable(WDTO_2S);
=======
=======
  Serial.begin(9600);
>>>>>>> 644d8fd (che merda)
  //wdt_enable(WDTO_2S);
>>>>>>> 4ea5616 (v0.3 - Finally functioning maybe)
=======
  //wdt_enable(WDTO_2S);
>>>>>>> 6cce433 (Tab)
}

void loop()
{
  //put your main code here, to run repeatedly :
  //wdt_reset();
  Application_FunctionSet.ApplicationFunctionSet_Obstacle();
}
