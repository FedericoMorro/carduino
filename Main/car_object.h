# ifndef CAR_OBJECT_H
# define CAR_OBJECT_H

#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"


const uint8_t DISTANCE = 15;
const uint16_t EXECUTION_TIME_INTERVAL = 600;
const uint8_t SENSIBILITY_LEVEL = 55;



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

enum status_car{
    stop,
    moving,
    controlling_obstacle,
}


class my_car{

public:
    uint8_t speed;
    uint8_t status;
    uint8_t pos_echo_radar;
    uint8_t free_spots;

    int L, M, R;


    uint16_t dist_array[5];    // an array keeping information on the distance.Between each block the directions are:  0° 45° 90° 135° 180°
    uint16_t pos_radar[4];  // an array keeping information on the distance between each block {45,90,135,90}   -- it's useless to check left and right since it is very unlikely to find anything there
    
    uint16_t space;

    uint16_t max_dist;
    uint8_t max_i;

public:
    my_car(){

        speed = 0;
        status = stop;
        pos_echo_radar = 0;
        space = 0;

        pos_array[0] = 45;
        pos_array[0] = 90;
        pos_array[0] = 135;
        pos_array[0] = 90;

    }

    void fill_dist_array(){

        max_i = 0;
        max_dist = 0;
        free_spots = 0;

        for(uint8_t i=0; i<5; i++){

            // maps all possible directions
            AppServo.DeviceDriverSet_Servo_control(45 * i /*Position_angle*/);
            
            delay_xxx(5);
            AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&space /*out*/);
            delay_xxx(10);

            dist_array[i] = dist;

            if(dist > DISTANCE){
                free_spots += 1;
            }


            if(dist_array[max_i] < dist){
                max_i = i;
            }

        }
    }

    void run(){
        status = moving;
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, speed); // first it goes back to have some space
        delay_xxx(EXECUTION_TIME_INTERVAL);
    }

    void check_line(){
  
        L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
        M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
        R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();

        if (L < SENSIBILITY_LEVEL && M < SENSIBILITY_LEVEL && R < SENSIBILITY_LEVEL) {
            status = stop; // stop at the line
            ApplicationFunctionSet_SmartRobotCarMotionControl (stop_it, 0);
            delay_xxx(5000);
        }
    }

    void check_obstacles(){
    
        pos_echo_radar += 1;
        pos_echo_radar %= 4;
        AppServo.DeviceDriverSet_Servo_control(pos_echo_radar /*Position_angle*/);
        AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&space /*out*/);
        delay_xxx(10);

        if(space < DISTANCE){
            
            pos_eco_radar = 0;
            status = controlling_obstacle; // checking an obstacle

            do{
                ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, speed); // first it goes back to have some space
                delay_xxx(EXECUTION_TIME_INTERVAL/2);     // executes the command for a specific time
                ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
                delay_xxx(5);


                fill_dist_array()   // this fills the dist array with a series of data and finds the maximum value

                if(free_spots >= 1){
                    switch (max_i){
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
                    }
                }
            }while(free_spots == 0);

        }

    }



}






# endif