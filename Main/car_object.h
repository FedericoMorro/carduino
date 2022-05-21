# ifndef CAR_OBJECT_H
# define CAR_OBJECT_H

#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"


const uint8_t DISTANCE = 20;


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
};  


class my_car{

public:
    uint8_t speed;
    uint8_t status;
    uint16_t dist_array[] = {0,0,0,0,0}; // an array keeping information on the distance between each block
                                        // the directions are:  0° 45° 90° 135° 180°
    uint16_t max_dist;
    uint8_t max_i;

public:
    my_car(){
        speed = 0;
        status = stop_it;
    }

    void fill_dist_array(){

        uint16_t dist = -1;

        max_i = 0;
        max_dist = 0;

        for(uint8_t i=0; i<5; i++){

            AppServo.DeviceDriverSet_Servo_control(45 * i /*Position_angle*/);
            
            delay_xxx(5);
            AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&dist /*out*/);
            delay_xxx(10);

            dist_array[i] = dist;

            if(dist_array[max_i] < dist){
                max_i = i;
            }

        }
    }


}






# endif