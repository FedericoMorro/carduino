#include "car_object.h"


void my_car::my_car(){

    speed = 0;
    status = stop;
    pos_echo_radar = 0;
    space = 0;

    pos_array[0] = 45;
    pos_array[0] = 90;
    pos_array[0] = 135;
    pos_array[0] = 90;

}


void my_car::fill_dist_array(){
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

void my_car::run(){
    status = moving;
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, speed); // first it goes back to have some space
    delay_xxx(EXECUTION_TIME_INTERVAL);
}

void my_car::check_line(){
    
    L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();

    if (L < SENSIBILITY_LEVEL && M < SENSIBILITY_LEVEL && R < SENSIBILITY_LEVEL) {
        status = stop; // stop at the line
        ApplicationFunctionSet_SmartRobotCarMotionControl (stop_it, 0);
        delay_xxx(5000);
    }
}


void my_car::check_obstacles(){
    
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