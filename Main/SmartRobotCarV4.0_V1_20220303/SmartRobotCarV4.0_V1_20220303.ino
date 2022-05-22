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


void setup()
{
  // put your setup code here, to run once:
  Application_FunctionSet.ApplicationFunctionSet_Init();
  //wdt_enable(WDTO_2S);
}

void loop()
{
  //put your main code here, to run repeatedly :
  //wdt_reset();
  Application_FunctionSet.ApplicationFunctionSet_Obstacle();
}
